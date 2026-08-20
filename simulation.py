"""Turn-based simulation for Fly-in.

Each drone is assigned one of the routes found by the pathfinder, then
the simulation advances turn by turn while enforcing every rule of the
subject:

* a zone holds at most `max_drones` drones (start/end are unlimited),
* a connection carries at most `max_link_capacity` drones per turn,
* entering a restricted zone takes 2 turns: the drone spends one turn
  "in flight" on the connection and MUST land the next turn, so the
  destination slot is reserved at departure,
* drones leaving a zone free its capacity for that same turn (moves are
  resolved in passes until nothing can move any more, so convoys work).

`run` returns one list of movement tokens per turn, ready to print in
the subject's output format (`D<id>-<zone>` or `D<id>-<from>-<to>`).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, FrozenSet, List, Optional, Tuple

from dijkstra import PathResult
from parser import MapData

Link = FrozenSet[str]  # frozenset({zone_a, zone_b}) identifies a connection


@dataclass
class Drone:
    ident: int              # 1-based id used in the output (D1, D2, ...)
    path: List[str]
    step: int = 0           # index of the zone the drone stands in
    flying_to: Optional[str] = None  # set while crossing to a restricted zone
    delivered: bool = False

    @property
    def zone(self) -> str:
        return self.path[self.step]

    @property
    def next_zone(self) -> str:
        return self.path[self.step + 1]


@dataclass
class TurnReport:
    """Everything that happened during one turn (for output and visuals)."""
    moves: List[str] = field(default_factory=list)
    positions: Dict[str, List[int]] = field(default_factory=dict)
    zone_load: Dict[str, int] = field(default_factory=dict)
    link_load: Dict[Tuple[str, str], int] = field(default_factory=dict)


class Simulation:
    """Runs the drones along their assigned paths, one turn at a time."""

    def __init__(self, data: MapData, paths: List[PathResult]) -> None:
        self.data = data
        self.drones = [
            Drone(ident=i + 1, path=paths[p].zones)
            for i, p in enumerate(self._assign_paths(paths))
        ]
        self.occupancy: Dict[str, int] = {name: 0 for name in data.hubs}
        self.occupancy[data.start or ""] = data.nb_drones
        self.link_caps: Dict[Link, int] = {
            frozenset((c.zone1, c.zone2)): c.max_link_capacity
            for c in data.connections
        }
        self.next_link_load: Dict[Link, int] = {}

    def _assign_paths(self, paths: List[PathResult]) -> List[int]:
        """Spread drones over paths so the last arrival is as early as
        possible: each drone picks the path with the lowest estimated
        finish time (path cost + queueing delay behind drones already
        assigned, given the path's per-turn throughput)."""
        throughput = [self._throughput(p) for p in paths]
        assigned = [0] * len(paths)
        choice: List[int] = []
        for _ in range(self.data.nb_drones):
            best = min(
                range(len(paths)),
                key=lambda p: paths[p].cost + assigned[p] // throughput[p],
            )
            choice.append(best)
            assigned[best] += 1
        return choice

    def _throughput(self, path: PathResult) -> int:
        """How many drones per turn the path can sustain (its narrowest
        link or intermediate zone; a restricted hop halves its link)."""
        caps = {
            frozenset((c.zone1, c.zone2)): c.max_link_capacity
            for c in self.data.connections
        }
        rate = self.data.nb_drones or 1
        for a, b in zip(path.zones, path.zones[1:]):
            link = caps[frozenset((a, b))]
            if self.data.hubs[b].zone_type == "restricted":
                link = max(1, link // 2)
            rate = min(rate, link)
        for zone in path.zones[1:-1]:
            rate = min(rate, self.data.hubs[zone].max_drones or 1)
        return max(1, rate)

    def run(self) -> List[TurnReport]:
        turns: List[TurnReport] = []
        while not all(d.delivered for d in self.drones):
            turns.append(self._play_turn())
            if len(turns) > 10_000:  # safety net: never loop forever
                raise RuntimeError("simulation exceeded 10000 turns")
        return turns

    def _play_turn(self) -> TurnReport:
        report = TurnReport()
        link_load = self.next_link_load  # in-flight drones still hold links
        self.next_link_load = {}

        landed = self._land_flying_drones(report)
        self._move_grounded_drones(report, link_load, landed)

        report.positions = self._positions()
        report.zone_load = dict(self.occupancy)
        report.link_load = {
            (min(k), max(k)): v for k, v in link_load.items() if v
        }
        return report

    def _land_flying_drones(self, report: TurnReport) -> set[int]:
        """In-flight drones always land: their slot was reserved.
        Returns their ids so they do not move a second time this turn."""
        landed: set[int] = set()
        for drone in self.drones:
            if drone.flying_to is None:
                continue
            drone.step += 1
            drone.flying_to = None
            landed.add(drone.ident)
            report.moves.append(f"D{drone.ident}-{drone.zone}")
            if drone.zone == self.data.end:
                drone.delivered = True
                self.occupancy[drone.zone] -= 1
        return landed

    def _move_grounded_drones(
        self, report: TurnReport, link_load: Dict[Link, int], landed: set[int]
    ) -> None:
        """Try to advance every grounded drone; repeat passes so drones
        that just vacated a zone unblock the ones behind them."""
        moved_ids: set[int] = set(landed)
        progressed = True
        while progressed:
            progressed = False
            for drone in self.drones:
                busy = drone.delivered or drone.flying_to is not None
                if busy or drone.ident in moved_ids:
                    continue
                move = self._try_step(drone, link_load)
                if move is not None:
                    report.moves.append(move)
                    moved_ids.add(drone.ident)
                    progressed = True

    def _try_step(
        self, drone: Drone, link_load: Dict[Link, int]
    ) -> Optional[str]:
        """Advance `drone` one step if every capacity rule allows it."""
        here, there = drone.zone, drone.next_zone
        link = frozenset((here, there))
        capacity = self.link_caps.get(link, 1)
        if link_load.get(link, 0) >= capacity:
            return None

        if self.data.hubs[there].zone_type == "restricted":
            # Two-turn move: also needs the link free next turn, and a
            # guaranteed slot in the destination on arrival.
            if self.next_link_load.get(link, 0) >= capacity:
                return None
            if not self._has_room(there):
                return None
            link_load[link] = link_load.get(link, 0) + 1
            self.next_link_load[link] = self.next_link_load.get(link, 0) + 1
            self.occupancy[here] -= 1
            self.occupancy[there] += 1  # reserved for next turn's landing
            drone.flying_to = there
            return f"D{drone.ident}-{here}-{there}"

        if there != self.data.end and not self._has_room(there):
            return None
        link_load[link] = link_load.get(link, 0) + 1
        self.occupancy[here] -= 1
        self.occupancy[there] += 1
        drone.step += 1
        if there == self.data.end:
            drone.delivered = True
            self.occupancy[there] -= 1
        return f"D{drone.ident}-{there}"

    def _has_room(self, zone: str) -> bool:
        hub = self.data.hubs[zone]
        if zone in (self.data.start, self.data.end):
            return True
        return self.occupancy[zone] < (hub.max_drones or 1)

    def _positions(self) -> Dict[str, List[int]]:
        """Zone name -> ids of drones standing there (for the visuals)."""
        where: Dict[str, List[int]] = {}
        for drone in self.drones:
            if drone.delivered:
                continue
            spot = drone.zone
            if drone.flying_to:
                spot = f"{drone.zone}-{drone.flying_to}"
            where.setdefault(spot, []).append(drone.ident)
        return where
