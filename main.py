"""Fly-in entry point.

Usage:
    python3 main.py [--quiet] [--no-color] [--capacity-info] <map_file>

Parses the map, finds routes, runs the simulation and prints one line
per turn in the subject's format. When stdout is a terminal, a coloured
flight board follows each line (disable with --quiet).
"""

from __future__ import annotations

import argparse
import sys

from dijkstra import Dijkstra
from parser import Parser, ParserError
from simulation import Simulation
from visual import render


def build_arg_parser() -> argparse.ArgumentParser:
    cli = argparse.ArgumentParser(description="Fly-in drone routing simulator")
    cli.add_argument("map_file", help="path to the map file")
    cli.add_argument("--quiet", action="store_true",
                     help="only print the canonical turn lines")
    cli.add_argument("--no-color", action="store_true",
                     help="disable ANSI colors")
    cli.add_argument("--capacity-info", action="store_true",
                     help="show zone and connection capacity usage per turn")
    return cli


def main() -> int:
    args = build_arg_parser().parse_args()
    try:
        data = Parser(args.map_file).parse()
    except OSError as exc:
        print(f"Error: cannot read {args.map_file}: {exc.strerror}",
              file=sys.stderr)
        return 1
    except ParserError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    assert data.start is not None and data.end is not None
    paths = Dijkstra(data).find_paths(data.start, data.end)
    if not paths:
        print("Error: no valid path from start_hub to end_hub",
              file=sys.stderr)
        return 1

    turns = Simulation(data, paths).run()
    show_board = sys.stdout.isatty() and not args.quiet
    render(data, turns, board=show_board,
           capacity_info=args.capacity_info,
           use_color=sys.stdout.isatty() and not args.no_color)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:  # e.g. output piped into `head`
        sys.exit(0)
