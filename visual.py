"""Terminal visualization for Fly-in.

Two layers of output are produced:

1. The canonical, machine-readable lines the subject requires (one line
   per turn, `D<id>-<zone>` tokens) are always printed as-is.
2. When enabled, each turn is followed by a coloured "flight board" that
   paints every zone in the `color` from its metadata and lists the
   drones standing in it, so a human can watch the fleet progress.
"""

from __future__ import annotations

from typing import List

from parser import MapData
from simulation import TurnReport

ANSI_COLORS = {
    "black": 30, "red": 31, "green": 32, "yellow": 33, "blue": 34,
    "magenta": 35, "purple": 35, "cyan": 36, "white": 37, "gray": 90,
    "grey": 90, "orange": 33, "pink": 95,
}
TYPE_MARK = {"restricted": "!", "priority": "*", "blocked": "x"}


class Visualizer:
    """Prints the coloured flight board and the final summary."""

    def __init__(self, data: MapData, use_color: bool) -> None:
        self.data = data
        self.use_color = use_color

    def paint(self, zone_name: str) -> str:
        """Wrap a zone name in the ANSI color from its metadata."""
        hub = self.data.hubs.get(zone_name)
        code = ANSI_COLORS.get(hub.color or "") if hub else None
        if not self.use_color or code is None:
            return zone_name
        return f"\033[{code}m{zone_name}\033[0m"

    def print_board(self, turn: int, report: TurnReport) -> None:
        """One line per occupied zone/connection: who is where right now."""
        cells: List[str] = []
        for spot, drones in sorted(report.positions.items()):
            ids = ",".join(f"D{d}" for d in sorted(drones))
            base = spot.split("-")[0]
            hub = self.data.hubs.get(spot)
            mark = TYPE_MARK.get(hub.zone_type, "") if hub else ">"
            cells.append(f"{self.paint(spot if hub else base)}{mark}[{ids}]")
        board = "  ".join(cells) if cells else "all drones delivered"
        print(f"  turn {turn:>3} | {board}")

    def print_capacity_info(self, report: TurnReport) -> None:
        """The `--capacity-info` view: zone and link usage this turn."""
        for zone, used in sorted(report.zone_load.items()):
            hub = self.data.hubs[zone]
            if used <= 0 or zone in (self.data.start, self.data.end):
                continue
            print(f"  Zone {zone}: {used}/{hub.max_drones or 1} drones")
        for conn in self.data.connections:
            a, b = sorted((conn.zone1, conn.zone2))
            used = report.link_load.get((a, b), 0)
            if used:
                print(f"  Connection {conn.zone1}-{conn.zone2}: "
                      f"{used}/{conn.max_link_capacity} capacity used")

    def print_summary(self, turns: List[TurnReport]) -> None:
        total_moves = sum(len(t.moves) for t in turns)
        drones = self.data.nb_drones
        print(f"# total turns: {len(turns)}")
        print(f"# total moves: {total_moves}")
        print(f"# average turns per drone: {total_moves / max(1, drones):.1f}")

    def render(
        self,
        turns: List[TurnReport],
        *,
        board: bool,
        capacity_info: bool,
    ) -> None:
        """Print the whole run: canonical lines plus the optional extras."""
        for number, report in enumerate(turns, start=1):
            print(" ".join(report.moves))
            if capacity_info:
                self.print_capacity_info(report)
            if board:
                self.print_board(number, report)
        if board:
            self.print_summary(turns)
