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
from visual import Visualizer


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

    if data.start is None or data.end is None:
        print("Error: map has no start_hub or no end_hub", file=sys.stderr)
        return 1

    paths = Dijkstra(data).find_paths(data.start, data.end)
    if not paths:
        print("Error: no valid path from start_hub to end_hub",
              file=sys.stderr)
        return 1

    turns = Simulation.best_schedule(data, paths)
    on_terminal = sys.stdout.isatty()
    viz = Visualizer(data, use_color=on_terminal and not args.no_color)
    viz.render(turns, board=on_terminal and not args.quiet,
               capacity_info=args.capacity_info)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except BrokenPipeError:  # output piped into e.g. `head`
        sys.exit(0)
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        sys.exit(1)
    except Exception as exc:  # never end on a traceback
        print(f"Error: unexpected failure: {exc}", file=sys.stderr)
        sys.exit(1)
