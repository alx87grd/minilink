# Plans index

Three backlog homes (do not invent a fourth). Two indexes point into
them without holding work of their own: [QUESTIONS.md](../dev/QUESTIONS.md), the design
questions that wait for the maintainer (architecture, API and naming, vision, performance; a
detail waits in its TODO row instead), and [KNOWN_ISSUES.md](../dev/KNOWN_ISSUES.md), the defects a
user can hit today, each with the row that fixes it.

| Home | Use for |
| --- | --- |
| [ROADMAP.md](../dev/ROADMAP.md) | Plan of record: releases, two lanes, TRL ledger, course checklists, the path to v1.0, open decisions, out-of-scope |
| [TODO.md](TODO.md) | The workboard: every open step, by rung, with files and "done when" |
| **This folder** (other docs) | Design writeups for steps that need one — multi-step architecture / tradeoff docs |

**One fact, one home.** What to build lives in the plan doc; the order of work and each
step's "done when" in TODO; milestones, maturity and course goals in ROADMAP; *open* design questions
in QUESTIONS.md, and the details waiting for the maintainer in their TODO row; landed
contracts in DESIGN. Other docs link to the home instead of restating it. A ruling the
maintainer takes updates the doc it changes (DESIGN, ROADMAP, TODO or the plan doc it shapes)
in the same change; the commit message carries its why. It then leaves QUESTIONS.md.

Pyro parity rows stay in [pyro-port-remaining.md](pyro-port-remaining.md)
(v0.5; open rows plus the name map). Landed contracts move into
[DESIGN.md](../dev/DESIGN.md) and the plan doc is deleted; one-line ideas go
to a Later bullet in [TODO.md](TODO.md) first — open a plan doc only when the
design needs a writeup.

Step ids (`A3`, `P4`, `S60`, …) are global and assigned in [TODO.md](TODO.md) only. A plan doc numbers
its own steps locally (geometry-module.md's steps 1–4, cited as "steps 1–4 of geometry-module.md") or with a
prefix of its own (`CBF-1`, `NS-3`, `PP-7`, `MS-D1`), never with a bare workboard-style id. The prefix
also marks a plan's rulings and the ids it cites from an audit, so they never collide with a
workboard id: a plan's own steps (`NS-1`, `CBF-2`, `RS-3`). Workboard ids (`A5`,
`P4`, `G1`, …) stay bare. In prose, an id never opens a sentence: the plain title comes first.

## The plan docs

| Doc | Scope | Where it sits |
| --- | --- | --- |
| [consolidation.md](consolidation.md) | The inputs of the maintainer's pass over demos and features, in one document: the demo set as a draft (thirteen stories, the nine drafted scripts, where the 94 old scripts would go) and the tutorials beside it; what the kept surface runs of the library; one recommendation per area of the repository (keep, clean, minimize, reduce to core, move, delete), judged against the three courses and the direction of the library; behind the kept surface, the twins, the reporting extras, the large drawing code, the unused options and the leftovers, each with its size, its callers and what its removal costs; the test layers with their open options (`MS-D2a`–`e`) for the clean slate. Candidates, not decisions | v0.4, the next step: the maintainer's pass (TODO §1.12) |
| [plans-as-policies.md](plans-as-policies.md) | The compiled faces of LQR and value iteration on one contract (two axes: open loop or feedback, stationary or time-varying), the MPC and the follower reading the state; the rulings that shape them | v0.4 or after the tag (TODO §1.8) |
| [vehicle-models.md](vehicle-models.md) | The car family, what is left to build: the tire and slip switch (the brush law by default, one smooth slip, the longitudinal car on the tire module), the UdeS Racecar on the 3-D car, `Car3D` (the double track with quasi-static load transfer), the 3-D car's options (wheel hop, an open differential, the contact along the suspension axis), and the choices taken while building that wait for the maintainer's look | after the maintainer's pass over the demos; the options unscheduled |
| [native-optimizers.md](native-optimizers.md) | Native solvers as pure laws, the solve as a rollout, traced solves, implicit differentiation, least squares, constraints, convergence as stability; the stochastic program (§7); steps NS-2's rest to NS-7 | v0.5 (TODO §4) |
| [estimation-and-state-feedback.md](estimation-and-state-feedback.md) | State feedback and estimation as duals: `LQGPlanner` (§5), the later estimators and their slots (§3, §6) | v0.5; v0.9 |
| [geometry-module.md](geometry-module.md) | Workspace geometry in `core/geometry/`: the track, the scene, `mount`, the spatial fields, a catalog of courses; `planning.spatial` retires | v0.5 (A3) |
| [pyro-port-remaining.md](pyro-port-remaining.md) | Pyro parity: the open rows and the name map, the source of the migration guide | v0.5 (C1) |
| [repository-structure.md](repository-structure.md) | The repositories around minilink (the teaching one, the private ones) and what is left of the move; the layout of the examples tree, its options and the asks | v0.9 (K5, K6); the examples tree with the demo pass |
| [cbf-safety-filter.md](cbf-safety-filter.md) | Control barrier functions: SDF interpolation, DCBF with slacks, HOCBF, the safety-filter block; the barrier a `Field` | unscheduled (TODO §6) |
| [rl-reward-scaling.md](rl-reward-scaling.md) | Automatic reward scaling for the RL planner (RS-1–RS-4) | unscheduled (TODO §6) |

Landed or superseded plan docs are deleted. Their history is git.
