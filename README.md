# sokoban-solver-01-run-20261121

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/androyesfernandez/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/androyesfernandez/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver. Every generated level is a 1-wide hallway
plus 1–4 rooms (one door each), and a box pushed through a door can never
come back, so every box stays in its room and the level's push-optimal
count is the sum of the rooms' optima. `main.ts` decomposes each level
into its rooms, solves every room independently with a push-optimal A*
(admissible minimum-assignment heuristic over box-ignoring BFS distances,
dead-square and 2×2 frozen-block pruning, transposition table), then
stitches the rooms together with plain hallway walks. A global push-A*
with the same heuristic is the fallback for any level shape that does not
decompose. All search budgets are fixed counts of expansions, never the
clock, so two runs produce byte-identical output. Dependency-free,
Node 22, no network.

## Run it

```sh
MAKEFUN_INPUT_DIR=/input MAKEFUN_OUTPUT_DIR=/output node main.ts
```

---

<sub>Entered on [make.fun](https://make.fun/?ref=ktpzxevq&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
