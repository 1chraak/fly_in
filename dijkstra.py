"""Pathfinding for Fly-in.

A custom Dijkstra over the zone graph. The cost of a step is the number
of turns it takes to enter the destination zone (1 for normal/priority,
2 for restricted, blocked zones cannot be entered). Ties in turn count
are broken in favour of paths that cross more priority zones, as the
subject asks.

`find_paths` extracts several routes. After each path is found, its
links get a usage mark; a link used beyond its capacity costs extra
turns in the next search, so later routes prefer fresh links but may
still share unavoidable ones (a capacity-1 link still carries one drone
per turn - the simulation enforces the real per-turn limits).
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, FrozenSet, List, Optional, Tuple

from parser import HubConfig, MapData

Edge = FrozenSet[str]  # frozenset({zone_a, zone_b}) = one connection


@dataclass
class PathResult:
    zones: List[str]
    cost: int  # turns for a single drone to walk this path alone


class Dijkstra:
    """Finds shortest routes from start to end, respecting zone types."""

    CONGESTION_PENALTY = 3  # extra search turns per over-capacity reuse

    def __init__(self, data: MapData) -> None:
        self.data = data
        self.capacity: Dict[Edge, int] = {
            frozenset((c.zone1, c.zone2)): c.max_link_capacity
            for c in data.connections
        }
        self.usage: Dict[Edge, int] = {}
        self.neighbours: Dict[str, List[str]] = {
            name: [] for name in data.hubs
        }
        for conn in data.connections:
            self.neighbours[conn.zone1].append(conn.zone2)
            self.neighbours[conn.zone2].append(conn.zone1)

    @staticmethod
    def _step_cost(hub: HubConfig) -> int:
        """Turns needed to enter this zone."""
        return 2 if hub.zone_type == "restricted" else 1

    def _shortest_path(self, start: str, end: str) -> Optional[PathResult]:
        """One Dijkstra run over links that still have capacity left.

        The heap is ordered by (search cost, non_priority_count): the
        search cost is the real turn cost plus a congestion penalty for
        links already claimed by earlier paths, and among equally cheap
        routes the one crossing more priority zones wins.
        """
        heap: List[Tuple[int, int, int, List[str]]] = [(0, 0, 0, [start])]
        best: Dict[str, Tuple[int, int]] = {start: (0, 0)}

        while heap:
            search_cost, non_prio, cost, path = heapq.heappop(heap)
            current = path[-1]
            if current == end:
                return PathResult(zones=path, cost=cost)
            if best.get(current, (search_cost, non_prio)) < (search_cost,
                                                             non_prio):
                continue

            for nxt in self.neighbours[current]:
                hub = self.data.hubs[nxt]
                if hub.zone_type == "blocked":
                    continue
                link = frozenset((current, nxt))
                overuse = max(
                    0,
                    self.usage.get(link, 0) + 1 - self.capacity.get(link, 1),
                )
                step = self._step_cost(hub)
                key = (search_cost + step + overuse * self.CONGESTION_PENALTY,
                       non_prio + (0 if hub.zone_type == "priority" else 1))
                if key < best.get(nxt, (1 << 30, 0)):
                    best[nxt] = key
                    heapq.heappush(
                        heap, (key[0], key[1], cost + step, path + [nxt])
                    )
        return None

    def find_paths(self, start: str, end: str) -> List[PathResult]:
        """Extract distinct routes (at most one per drone), stopping as
        soon as the search only rediscovers an already known route."""
        paths: List[PathResult] = []
        seen: set[Tuple[str, ...]] = set()
        while len(paths) < self.data.nb_drones:
            result = self._shortest_path(start, end)
            if result is None or tuple(result.zones) in seen:
                break
            seen.add(tuple(result.zones))
            paths.append(result)
            for a, b in zip(result.zones, result.zones[1:]):
                link = frozenset((a, b))
                self.usage[link] = self.usage.get(link, 0) + 1
        return paths
