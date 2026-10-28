# Plans index

Three backlog homes (do not invent a fourth); audits live separately. Two indexes point into
them without holding work of their own: [QUESTIONS.md](../dev/QUESTIONS.md), the design
questions that wait for the maintainer (architecture, API and naming, vision, performance; a
detail waits in its TODO row instead), and [KNOWN_ISSUES.md](../dev/KNOWN_ISSUES.md), the defects a
user can hit today, each with the row that fixes it.

| Home | Use for |
| --- | --- |
| [ROADMAP.md](../dev/ROADMAP.md) | Plan of record: releases, two lanes, TRL ledger, course checklists, the path to v1.0, open decisions, out-of-scope |
| [TODO.md](TODO.md) | The workboard: every open step, by rung, with files and "done when" |
| **This folder** (other docs) | Design writeups for steps that need one — multi-step architecture / tradeoff docs |
| [../reviews/](../reviews/) | Dated architecture audits and decision records (read-only history) |

**One fact, one home.** What to build lives in the plan doc; the order of work and each
step's "done when" in TODO; milestones, maturity and course goals in ROADMAP; *open* design questions
in QUESTIONS.md, and the details waiting for the maintainer in their TODO row; how and why a decision was taken in a dated record under `docs/reviews/`; landed
contracts in DESIGN. Other docs link to the home instead of restating it. A ruling goes to
the plan doc it shapes and to its dated record; it leaves QUESTIONS.md when it is taken.

Pyro parity rows stay in [pyro-port-remaining.md](pyro-port-remaining.md)
(v0.4; open rows plus the landed name map). Landed contracts move into
[DESIGN.md](../dev/DESIGN.md) and the plan doc is deleted; one-line ideas go
to a Later bullet in [TODO.md](TODO.md) first — open a plan doc only when the
design needs a writeup.

Step ids (`A3`, `P4`, `S60`, …) are global and assigned in [TODO.md](TODO.md) only; `R1`–`R166`
are rulings, numbered in their dated records under [../reviews/](../reviews/). A plan doc numbers
its own steps locally (fields.md's 4.1–4.8, cited as "steps 4.1–4.8 of fields.md") or with a
prefix of its own (`CBF-1`, `OPW-1`, `AM-1`), never with a bare workboard-style id. The prefix
also marks a plan's rulings and the ids it cites from an audit, so they never collide with a
workboard id: randomness.md's rules, rulings and amendments (`RD-R5`, `RD-D13`, `RD-A2`), the
2026-09-15 foundations review's findings and comments (`FD-F9`, `FD-C6`). Workboard ids (`A5`,
`P4`, `G1`, …) stay bare. The decision ids `D13`–`D30`, kept inside the plan docs that hold them,
are anchors only; TODO and QUESTIONS cite them by their heading. In prose, an id never opens a
sentence: the plain title comes first.

**The v0.3 core has one design home:**
- **The design:** [core-architecture.md](core-architecture.md), one section per workstream,
  numbered as TODO §1.N.
- **The rows:** TODO §1.N, with the same N.
- **The other plan docs:** they hold design detail only, never a rung, an order or a restated
  status; they link their TODO row instead.
- **Editing:** edit in place when a fact changes; no new consolidation pass. The last ones, and
  why: [the simplification record](../reviews/2026-10-26-plan-simplification.md) (2026-10-26)
  and the pass that brought every plan doc onto R141–R153 (2026-10-27, at the maintainer's
  request; [the decisions record](../reviews/2026-10-27-core-vision-decisions.md)).

## The plan docs

| Doc | Scope | Where it sits |
| --- | --- | --- |
| [core-architecture.md](core-architecture.md) | **Read first for the core.** The v0.3 workstreams (two invariants, seven pillars), each with its goal, status, code today, open decisions and rows; the order (three lanes); the proposed definition of done; the core against the features | v0.3 |
| [sampled-path.md](sampled-path.md) | Time in the core and the sampled loop (S31, R145–R147, R158): what `t` is for every kind and every diagram, `SampledSystem` the sampled-data kind, periods on blocks, `SampledDiagramSystem` and its plan, the two run verbs, the two runners; D13–D15; kept for S53 | core §2, §5 |
| [randomness.md](randomness.md) | One convention for randomness: a `Distribution` has no time, noise is a held law, one key per experiment; RD-D1–RD-D24; RN-2–RN-6 | core §1, §3 |
| [differentiable-problem.md](differentiable-problem.md) | What remains of the path from any parameter to any metric: the `StepRollout` fold, steps 7, 7b, 8, 9; §9's one score (decided in part, R141, R142); traceable linear analysis (S73, §4.6) | core §1, §2, §3, §5, §6; S73 v0.4 |
| [parametric-program.md](parametric-program.md) | One program family with every problem parameter bound at solve time (PP-0–PP-5, D25 in §2.1); one optimizer backend table (OPW-1–OPW-5) | core §7 |
| [native-optimizers.md](native-optimizers.md) | Native solvers as pure laws, the solve as a rollout, traced solves; the fixed-iteration MPC; the stochastic program (§7); NS-1–NS-9 | core §7, §8; the rest v0.4 |
| [planner-and-mpc.md](planner-and-mpc.md) | The planner as a description with a compiled pure face (`n_z`, `initial`, `solve`, `shift`, `policy`, `prediction`, `solution`, `pack`); `ModelPredictiveController` in two forms on it; `t_start`; inputs as bindings (R148) | core §7, §8 |
| [fields.md](fields.md) | The `Field` noun on `(x, u, t)`: `QuadraticField`, `GridField`, `CallableField`; steps 4.1–4.8 | core §9 |
| [future-policies.md](future-policies.md) | The API projected onto MPPI, behavior cloning and diffusion policy; the seeded policy (D29, §5.1); questions for discussion (§7) | core §4, §5; after v1.0 |
| [automation-by-convention.md](automation-by-convention.md) | What is left of the conventions: role-aware plots and animation (AC-5, AC-6), the cost's draws (AC-7), the sampled loop in step and mainstream (AC-8a, AC-8b). History: [the decision record](../reviews/2026-10-11-automation-by-convention.md), [the second opinion](../reviews/2026-10-13-automation-by-convention-second-opinion.md) | core §3, §5; plotting lane |
| [mppi.md](mppi.md) | Path integral control: `MPPIPlanner` beside trajopt, wrapped by the MPC block (MP-1–MP-6; D26–D28 and D30 in §10, D29 in future-policies.md §5.1) | v0.4 (S72); MP-4, MP-5 before v0.9 |
| [estimation-and-state-feedback.md](estimation-and-state-feedback.md) | State feedback and estimation as duals: `LQGPlanner` (§5), the estimation follow-ups and their slots (§3, §6). History: [the rulings](../reviews/2026-10-14-state-space-design-rulings.md) | v0.4; v0.9 |
| [geometry-module.md](geometry-module.md) | Workspace geometry in `core/geometry/` (shapes, paths, track, scene, `mount`, spatial fields, course catalog); `planning.spatial` retires | v0.4 (A3) |
| [pyro-port-remaining.md](pyro-port-remaining.md) | Pyro parity: open rows and the name map; the migration guide | v0.4 (C1) |
| [repository-structure.md](repository-structure.md) | The examples tree and where the teaching material lives; options, a recommendation, the asks | v0.9 (K5, K6) |
| [rl-reward-scaling.md](rl-reward-scaling.md) | Automatic reward scaling for the RL planner (RS-1–RS-4) | unscheduled (TODO §6) |
| [cbf-safety-filter.md](cbf-safety-filter.md) | Control barrier functions: SDF interpolation, DCBF with slacks, HOCBF, the safety-filter block; the barrier a `Field` | unscheduled |

Landed or superseded plan docs are deleted. Their history is git, and the rulings they carried
are in the dated records (for example [P11 dropped](../reviews/2026-10-15-gro501-app-notebooks-dropped.md)).
