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
(v0.5; open rows plus the landed name map). Landed contracts move into
[DESIGN.md](../dev/DESIGN.md) and the plan doc is deleted; one-line ideas go
to a Later bullet in [TODO.md](TODO.md) first — open a plan doc only when the
design needs a writeup.

Step ids (`A3`, `P4`, `S60`, …) are global and assigned in [TODO.md](TODO.md) only. A plan doc numbers
its own steps locally (geometry-module.md's steps 1–6, cited as "steps 1–6 of geometry-module.md") or with a
prefix of its own (`CBF-1`, `OPW-1`, `AM-1`, `MS-D1`), never with a bare workboard-style id. The prefix
also marks a plan's rulings and the ids it cites from an audit, so they never collide with a
workboard id: a plan's own steps (`NS-1`, `CBF-2`, `RS-3`). Workboard ids (`A5`,
`P4`, `G1`, …) stay bare. In prose, an id never opens a sentence: the plain title comes first.

## The plan docs

| Doc | Scope | Where it sits |
| --- | --- | --- |
| [plans-as-policies.md](plans-as-policies.md) | The compiled faces of LQR and value iteration on one contract (two axes: open loop or feedback, stationary or time-varying), the MPC and the follower reading the state; the rulings that shape it (its first two changes landed in v0.4) | v0.4 or after the tag (TODO §1.8) |
| [api-stabilization.md](api-stabilization.md) | The review of the public surface before the example rebuild (2026-11-10): defects, names a rule already decides, additions, 24 decisions for the maintainer, planned rows to pull forward; its defects section landed, the rest waiting for the maintainer's go | v0.4, before the demo rebuild (TODO §1.12) |
| [minimal-surface.md](minimal-surface.md) | The inputs of the end-of-v0.4 rebuild: the test layers and their open options (`MS-D2a`–`e`), a starting demo list, the course-link constraint | v0.4, the last steps (TODO §1.12) |
| [native-optimizers.md](native-optimizers.md) | Native solvers as pure laws, the solve as a rollout, traced solves, implicit differentiation, least squares, constraints, convergence as stability; the stochastic program (§7); NS-2's rest to NS-7 | v0.5 (the proof of concept landed in v0.3) |
| [estimation-and-state-feedback.md](estimation-and-state-feedback.md) | State feedback and estimation as duals: `LQGPlanner` (§5), the estimation follow-ups and their slots (§3, §6). | v0.5; v0.9 |
| [geometry-module.md](geometry-module.md) | Workspace geometry in `core/geometry/` (shapes and paths landed; track, scene, `mount`, spatial fields, course catalog left); `planning.spatial` retires | v0.5 (A3) |
| [cbf-safety-filter.md](cbf-safety-filter.md) | Control barrier functions: SDF interpolation, DCBF with slacks, HOCBF, the safety-filter block; the barrier a `Field` | unscheduled (TODO §6) |
| [pyro-port-remaining.md](pyro-port-remaining.md) | Pyro parity: open rows and the name map; the migration guide | v0.5 (C1) |
| [repository-structure.md](repository-structure.md) | The examples tree and where the teaching material lives; options, a recommendation, the asks | v0.9 (K5, K6) |
| [rl-reward-scaling.md](rl-reward-scaling.md) | Automatic reward scaling for the RL planner (RS-1–RS-4) | unscheduled (TODO §6) |

Landed or superseded plan docs are deleted. Their history is git.
