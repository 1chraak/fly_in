*This project has been created as part of the 42 curriculum by izouriqi.*

# Fly-in

## Description

Fly-in is a drone routing simulator. Given a map file describing a
network of zones (a graph) and a number of drones, it computes routes
from the unique start zone to the unique end zone and plays the flight
turn by turn, moving as many drones as possible each turn while
respecting every constraint:

- zone occupancy (`max_drones`, default 1; start and end are unlimited),
- connection capacity (`max_link_capacity`, default 1 drone per turn),
- zone types: `normal` (1 turn), `priority` (1 turn, preferred),
  `restricted` (2 turns, the drone crosses the connection in flight and
  must land the next turn), `blocked` (never enterable),
- the goal is to deliver all drones in the fewest simulation turns.

## Instructions

Requires Python 3.10+. Only the standard library is used at runtime;
`flake8` and `mypy` are development dependencies.

```sh
make install                 # pip install flake8 + mypy
make run MAP=maps/easy/01_linear_path.txt
make debug MAP=map.txt       # run under pdb
make lint                    # flake8 + mypy with the subject's flags
make lint-strict             # flake8 + mypy --strict
make clean                   # remove caches
```

Or directly:

```sh
python3 main.py [--quiet] [--no-color] [--capacity-info] <map_file>
```

- `--quiet` prints only the canonical one-line-per-turn output.
- `--no-color` disables ANSI colors.
- `--capacity-info` prints zone and connection capacity usage per turn,
  e.g. `Zone wide: 4/4 drones`, `Connection start-wide: 4/4 capacity used`.

When stdout is a terminal, a colored flight board follows each turn
line; when output is piped, only the canonical lines are emitted, so
grading scripts always see clean data.

## Example

Input (`maps/easy/01_linear_path.txt`):

```
nb_drones: 2
start_hub: start 0 0 [color=green]
end_hub: goal 3 0 [color=yellow]
hub: mid1 1 0 [color=blue]
hub: mid2 2 0 [color=blue]
connection: start-mid1
connection: mid1-mid2
connection: mid2-goal
```

Output (`python3 main.py --quiet maps/easy/01_linear_path.txt`):

```
D1-mid1
D1-mid2 D2-mid1
D1-goal D2-mid2
D2-goal
```

One line per turn; each token is `D<id>-<zone>` for a completed move or
`D<id>-<from>-<to>` while a drone is in flight toward a restricted
zone. Stationary drones are omitted; delivered drones are no longer
tracked; the simulation stops when every drone has reached the end.

## Algorithm explanation

The pipeline has three stages, each in its own module.

**1. Parsing (`parser.py`).** A strict line-oriented parser builds the
graph. Every rule of the subject's "Parser Constraints" is validated
and any problem raises a `ParserError` naming the line and the cause;
`main.py` catches it and exits cleanly (no crash, exit code 1). A BFS
reachability check rejects maps where the end cannot be reached.

**2. Pathfinding (`dijkstra.py`).** A custom Dijkstra (no graph
libraries) runs over the zone graph. The cost of entering a zone is its
turn cost (1 normal/priority, 2 restricted; blocked is skipped). The
priority queue is ordered lexicographically by
`(turns, number of non-priority zones)`, so among equally fast routes
the one crossing more priority zones wins — this is how priority zones
are "preferred in pathfinding". Several routes are extracted by
re-running Dijkstra after decrementing the remaining capacity of each
link used by the previous path; searches naturally avoid saturated
links, yielding capacity-aware, mostly disjoint routes. Extraction
stops when no route remains or there is one route per drone.
Complexity: one run is O(E log V); extracting k paths is O(k·E log V).

**3. Scheduling (`simulation.py`).** Drones are distributed over the
routes greedily: each drone joins the route with the smallest estimated
finish time, `cost + queued_drones // throughput`, where a route's
throughput is its narrowest link or intermediate zone (a restricted hop
halves its link, since a crossing occupies it for two turns). This is
the classic lem-in balancing argument and minimizes the completion time
of the last drone for pipeline-shaped routes. The simulator then plays
discrete turns: in-flight drones land first (their destination slot was
reserved at launch, so landing can never fail — a drone may not wait on
a connection), then grounded drones advance in repeated passes until no
further move is legal, so a drone vacating a zone unblocks the drone
behind it within the same turn, exactly as the subject requires.
Capacity bookkeeping is O(moves) per turn, and paths are computed once
and cached — nothing is re-planned during the simulation.

**Measured performance** (all within the exceptional-performance
targets):

| Map | Drones | Turns | Target |
|---|---:|---:|---:|
| easy/linear path | 2 | 4 | ≤ 6 |
| easy/simple fork | 4 | 4 | ≤ 8 |
| easy/basic capacity | 4 | 4 | ≤ 6 |
| medium/dead end trap | 5 | 8 | ≤ 12 |
| medium/circular loop | 6 | 15 | ≤ 15 |
| medium/priority puzzle | 5 | 7 | ≤ 12 |
| hard/maze nightmare | 8 | 13 | ≤ 30 |
| hard/capacity hell | 12 | 16 | ≤ 35 |
| hard/ultimate challenge | 15 | 26 | ≤ 45 |

The challenger map is solved legally but above the 45-turn record (67 turns): its
routes all funnel through restricted capacity-1 gates, which caps the
throughput of any routing strategy. That level is optional.

## Visual representation

Two layers of output enhance understanding without ever polluting the
canonical format:

1. **Canonical lines (always):** the exact output the subject defines,
   suitable for scripts and peer verification.
2. **Colored flight board (terminal only, disable with `--quiet`):**
   after each turn, one indented line shows every occupied zone painted
   in the `color` from its metadata, with the drones currently inside
   (`exit_point![D1]` — `!` marks restricted, `*` priority) and drones
   in flight on a connection (`loop_b>[D2]`). A short summary (total
   turns, total moves, average turns per drone — the subject's optional
   secondary metrics) is printed at the end. Watching the board makes
   bottlenecks visible: you can see drones batch up in wide zones and
   squeeze single-file through restricted gates.

## Resources

- E. W. Dijkstra, "A Note on Two Problems in Connexion with Graphs"
  (1959) — the shortest-path algorithm used here.
- CLRS, *Introduction to Algorithms*, ch. 22–24 (graphs, BFS, Dijkstra).
- The classic 42 "lem-in" write-ups on flow-based ant routing, for the
  drone-to-path balancing idea (finish time = path cost + queue delay).
- Python docs: `heapq`, `dataclasses`, `argparse`, ANSI escape codes.

**AI usage:** AI (Anthropic's Claude) was used as a pair-programming
assistant for: reviewing the parser's error handling against the
subject, designing the turn engine's capacity bookkeeping (multi-pass
move resolution and restricted-zone slot reservation), generating edge
case and stress test maps, and drafting this README. All generated code
was reviewed, tested and is fully understood by the author; an
independent replay validator (written separately) was used to verify
that every simulation transcript respects the subject's rules.
