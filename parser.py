from __future__ import annotations

import re
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

VALID_ZONE_TYPES = {"normal", "blocked", "restricted", "priority"}


class ParserError(Exception):
    """Raised when the map file does not respect the subject's grammar."""

    def __init__(self, line_number: int, message: str) -> None:
        prefix = f"line {line_number}: " if line_number > 0 else ""
        super().__init__(prefix + message)
        self.line_number = line_number


@dataclass
class HubConfig:
    kind: str
    name: str
    x: int
    y: int
    zone_type: str = "normal"
    color: Optional[str] = None
    max_drones: Optional[int] = None


@dataclass
class ConnectionConfig:
    zone1: str
    zone2: str
    max_link_capacity: int = 1


@dataclass
class MapData:
    nb_drones: int = 0
    hubs: Dict[str, HubConfig] = field(default_factory=dict)
    start: Optional[str] = None
    end: Optional[str] = None
    connections: List[ConnectionConfig] = field(default_factory=list)


class Parser:

    HUB_LINE_RE = re.compile(
        r"^(hub|start_hub|end_hub):"
        r"\s+(.+?)\s+(-?\d+)\s+(-?\d+)\s*(\[[^\[\]]*\])?\s*$"
    )
    CONNECTION_LINE_RE = re.compile(
        r"^connection:\s*(.+?)-(.+?)\s*(\[[^\[\]]*\])?\s*$"
    )
    NB_DRONES_RE = re.compile(r"^nb_drones:\s*(\S+)\s*$")

    def __init__(self, file_name: str) -> None:
        self.file_name = file_name
        self.lines: List[Tuple[int, str]] = []
        self.data = MapData()
        self._seen_connections: set[frozenset[str]] = set()

    def parse(self) -> MapData:
        self._read_file()

        nb_drones_seen = False
        for line_number, raw_line in self.lines:
            line = self._strip_comment(raw_line).strip()
            if not line:
                continue

            line_type = self._detect_line(line, line_number)

            if line_type == "nb_drones":
                self.data.nb_drones = self._parse_nb_drones(line, line_number)
                nb_drones_seen = True
            elif line_type in ("hub", "start_hub", "end_hub"):
                self._parse_hub(line, line_number, line_type)
            elif line_type == "connection":
                self._parse_connection(line, line_number)

        if not nb_drones_seen:
            raise ParserError(0, "missing 'nb_drones' line")
        if self.data.start is None:
            raise ParserError(0, "missing 'start_hub' line")
        if self.data.end is None:
            raise ParserError(0, "missing 'end_hub' line")

        if not self.has_path(self.data.start, self.data.end):
            raise ParserError(0, "no valid path between start_hub and end_hub")

        return self.data

    def has_path(self, start: str, end: str) -> bool:
        """BFS reachability check between two zones, ignoring blocked zones."""
        if start not in self.data.hubs or end not in self.data.hubs:
            return False

        graph: Dict[str, List[str]] = {name: [] for name in self.data.hubs}
        for conn in self.data.connections:
            graph[conn.zone1].append(conn.zone2)
            graph[conn.zone2].append(conn.zone1)

        visited = {start}
        queue = deque([start])
        while queue:
            current = queue.popleft()
            if current == end:
                return True
            for neighbour in graph[current]:
                if neighbour in visited:
                    continue
                if self.data.hubs[neighbour].zone_type == "blocked":
                    continue
                visited.add(neighbour)
                queue.append(neighbour)
        return end in visited

    def _read_file(self) -> None:
        """Load the non-empty, non-comment lines with their line numbers.

        ``utf-8-sig`` transparently drops a byte-order mark; a file that
        is not valid UTF-8 text is reported as a parse error rather than
        crashing the program.
        """
        try:
            with open(self.file_name, "r", encoding="utf-8-sig") as handle:
                numbered = list(enumerate(handle, start=1))
        except UnicodeDecodeError:
            raise ParserError(0, "file is not valid UTF-8 text") from None
        for index, line in numbered:
            stripped = line.strip()
            if stripped and not stripped.startswith("#"):
                self.lines.append((index, line))

    @staticmethod
    def _strip_comment(line: str) -> str:
        """Strip a trailing '#' comment ("#" inside [metadata] is kept)."""
        depth = 0
        for i, char in enumerate(line):
            if char == "[":
                depth += 1
            elif char == "]":
                depth = max(0, depth - 1)
            elif char == "#" and depth == 0:
                return line[:i]
        return line

    @staticmethod
    def _detect_line(line: str, line_number: int) -> str:
        prefixes = ("nb_drones", "start_hub", "end_hub", "hub", "connection")
        for prefix in prefixes:
            if line.startswith(f"{prefix}:"):
                return prefix
        raise ParserError(line_number, f"unrecognized line: {line!r}")

    def _parse_nb_drones(self, line: str, line_number: int) -> int:
        match = self.NB_DRONES_RE.match(line)
        if not match:
            raise ParserError(
                line_number,
                f"malformed nb_drones line: {line!r}",
            )
        raw_value = match.group(1)
        if not raw_value.isdigit() or int(raw_value) <= 0:
            raise ParserError(
                line_number,
                "nb_drones must be a positive integer",
            )
        return int(raw_value)

    def _parse_hub(self, line: str, line_number: int, kind: str) -> None:
        match = self.HUB_LINE_RE.match(line)
        if not match:
            raise ParserError(line_number, f"malformed {kind} line: {line!r}")

        _, name, x_str, y_str, metadata_block = match.groups()
        name = name.strip()

        if not name:
            raise ParserError(line_number, "hub name cannot be empty")
        if "-" in name or " " in name:
            raise ParserError(
                line_number,
                f"hub name {name!r} cannot contain dashes or spaces",
            )
        if name in self.data.hubs:
            raise ParserError(line_number, f"duplicate hub name: {name!r}")

        if kind == "start_hub" and self.data.start is not None:
            raise ParserError(line_number, "duplicate start_hub")
        if kind == "end_hub" and self.data.end is not None:
            raise ParserError(line_number, "duplicate end_hub")

        metadata = self._parse_metadata(metadata_block, line_number)

        zone_type = metadata.pop("zone", "normal")
        if zone_type not in VALID_ZONE_TYPES:
            raise ParserError(line_number, f"invalid zone type: {zone_type!r}")

        color = metadata.pop("color", None)

        max_drones: Optional[int] = None
        if "max_drones" in metadata:
            max_drones = self._parse_positive_int(
                metadata.pop("max_drones"), line_number, "max_drones"
            )
            if kind in ("start_hub", "end_hub"):
                max_drones = None

        if metadata:
            unknown = ", ".join(metadata.keys())
            raise ParserError(
                line_number,
                f"unknown metadata key(s): {unknown}",
            )

        x = self._parse_int(x_str, line_number, "x")
        y = self._parse_int(y_str, line_number, "y")

        other_name = self.data.end if kind == "start_hub" else self.data.start
        if other_name is not None:
            other = self.data.hubs[other_name]
            if other.x == x and other.y == y:
                raise ParserError(
                    line_number,
                    "start_hub and end_hub cannot share coordinates",
                )

        hub = HubConfig(kind=kind, name=name, x=x, y=y,
                        zone_type=zone_type, color=color,
                        max_drones=max_drones)
        self.data.hubs[name] = hub

        if kind == "start_hub":
            self.data.start = name
        elif kind == "end_hub":
            self.data.end = name

    def _parse_connection(self, line: str, line_number: int) -> None:
        match = self.CONNECTION_LINE_RE.match(line)
        if not match:
            raise ParserError(
                line_number,
                f"malformed connection line: {line!r}",
            )

        zone1, zone2, metadata_block = match.groups()
        zone1, zone2 = zone1.strip(), zone2.strip()

        if not zone1 or not zone2:
            raise ParserError(
                line_number,
                f"malformed connection line: {line!r}",
            )
        for name in (zone1, zone2):
            if name not in self.data.hubs:
                raise ParserError(
                    line_number,
                    f"connection references undefined zone: {name!r}",
                )
        if zone1 == zone2:
            raise ParserError(
                line_number,
                f"self-connection is forbidden: {zone1!r}",
            )

        key = frozenset((zone1, zone2))
        if key in self._seen_connections:
            raise ParserError(
                line_number,
                f"duplicate connection: {zone1}-{zone2}",
            )
        self._seen_connections.add(key)

        metadata = self._parse_metadata(metadata_block, line_number)
        capacity = 1
        if "max_link_capacity" in metadata:
            capacity = self._parse_positive_int(
                metadata.pop("max_link_capacity"),
                line_number, "max_link_capacity",
            )
        if metadata:
            unknown = ", ".join(metadata.keys())
            raise ParserError(
                line_number,
                f"unknown metadata key(s): {unknown}",
            )

        self.data.connections.append(ConnectionConfig(zone1, zone2, capacity))

    @staticmethod
    def _parse_metadata(
        block: Optional[str], line_number: int
    ) -> Dict[str, str]:
        if not block:
            return {}
        inner = block[1:-1].strip()
        if not inner:
            return {}

        metadata: Dict[str, str] = {}
        for token in inner.split():
            if "=" not in token:
                raise ParserError(
                    line_number,
                    f"malformed metadata token: {token!r}",
                )
            key, _, value = token.partition("=")
            if not key or not value:
                raise ParserError(
                    line_number,
                    f"malformed metadata token: {token!r}",
                )
            if key in metadata:
                raise ParserError(
                    line_number,
                    f"duplicate metadata key: {key!r}",
                )
            metadata[key] = value
        return metadata

    @staticmethod
    def _parse_int(value: str, line_number: int, field_name: str) -> int:
        try:
            return int(value)
        except ValueError:
            raise ParserError(
                line_number, f"{field_name} must be an integer: {value!r}"
            ) from None

    @staticmethod
    def _parse_positive_int(
        value: str, line_number: int, field_name: str
    ) -> int:
        if not value.isdigit() or int(value) <= 0:
            raise ParserError(
                line_number,
                f"{field_name} must be a positive integer: {value!r}",
            )
        return int(value)
