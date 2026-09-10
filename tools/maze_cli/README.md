# maze_cli

Runs the exact maze-solving algorithm the robot runs (`../../src/maze.cpp`
— flood fill for exploration, Dijkstra for the turn-minimizing speed run —
zero Arduino dependencies) against a maze, entirely on the host. Built so
the algorithm can be validated/visualized without needing to script the
`mms` simulator's Qt GUI — see `../dashboard`, which drives this tool.

## Build

```
./build.sh
```

Produces `./maze_cli`. Requires a C++17 compiler (g++/clang++); no other
dependencies.

## Protocol

Reads whitespace-separated `KEY VALUE...` tokens from stdin (not JSON —
this file has no JSON dependency; `../dashboard/server.py` translates the
frontend's JSON request into this before piping it in), one JSON object
to stdout.

Input keys (any order, all optional except `MODE`):

| Key      | Meaning |
|----------|---------|
| `MODE`   | `search` or `speedrun` |
| `START`  | `<x> <y>` — default `0 0` |
| `GOALS`  | `<n> <x1> <y1> ... <xn> <yn>` — default: the standard 4-cell center |
| `RANDOM` | `0`/`1` — search mode: generate a random solvable maze instead of using `WALLS` |
| `SEED`   | RNG seed for `RANDOM 1` |
| `WALLS`  | `<n> <w1> ... <wn>` — n = width×height cells, row-major (`y*width+x`), each a wall bitmask (bit0=N, bit1=E, bit2=S, bit3=W). Ground truth for `search`, or the known map for `speedrun`. |

Maze size is fixed at compile time by `MAZE_WIDTH`/`MAZE_HEIGHT` in
`../../include/maze.h` (16×16 by default) — `WIDTH`/`HEIGHT` input keys
are accepted and ignored.

### Examples

```
echo "MODE search
RANDOM 1
SEED 7" | ./maze_cli
```

```
echo "MODE speedrun
START 0 0
GOALS 4 7 7 8 7 7 8 8
WALLS 256 0 0 0 ..." | ./maze_cli
```

### Output

`search` mode:
```json
{
  "ok": true, "width": 16, "height": 16,
  "start": {"x":0,"y":0}, "goals": [{"x":7,"y":7}, ...],
  "ground_truth_walls": [ /* 256 bitmasks */ ],
  "search": {
    "path": [{"x":0,"y":0,"heading":"N"}, ...],
    "actions": ["FWD","LEFT", ...],
    "discovered_walls": [ /* 256 bitmasks, as known after exploring */ ],
    "reached_center_at_step": 121,
    "total_moves": 242, "total_turns": 144, "est_time_ms": 171400
  }
}
```

`speedrun` mode (requires `WALLS` — a known map, typically a prior
search's `discovered_walls`):
```json
{
  "ok": true, "width": 16, "height": 16,
  "speedrun": {
    "path": [...], "actions": [...],
    "total_moves": 121, "total_turns": 71, "est_time_ms": 85350
  }
}
```

`est_time_ms` uses the same `SOLVER_CELL_MOVE_TIME_MS`/`SOLVER_TURN_90_TIME_MS`
constants as the firmware (`../../include/solver.h`) — it's only as
accurate as those are tuned.
