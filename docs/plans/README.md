# Plans index

Three backlog homes (do not invent a fourth); audits live separately:

| Home | Use for |
| --- | --- |
| [ROADMAP.md](../dev/ROADMAP.md) | Plan of record: releases, two lanes, TRL ledger, course checklists, the path to v1.0, open decisions, out-of-scope |
| [TODO.md](TODO.md) | The workboard: every open step, by rung, with files and "done when" |
| **This folder** (other docs) | Design writeups for steps that need one — multi-step architecture / tradeoff docs |
| [../reviews/](../reviews/) | Dated architecture audits and decision records (read-only history) |

**One fact, one home.** What to build lives in the plan doc; the order of work and each
step's "done when" in TODO; milestones, maturity, course goals and *open* questions in
ROADMAP; how and why a decision was taken in a dated record under `docs/reviews/`; landed
contracts in DESIGN. Other docs link to the home instead of restating it. A ruling goes to
the plan doc it shapes and to its dated record; it leaves ROADMAP §6 when it is taken.

Pyro parity rows stay in [pyro-port-remaining.md](pyro-port-remaining.md)
(v0.3; open rows plus the landed name map). Landed contracts move into
[DESIGN.md](../dev/DESIGN.md) and the plan doc is deleted; one-line ideas go
to a Later bullet in [TODO.md](TODO.md) first — open a plan doc only when the
design needs a writeup.

Step ids (`R1`, `A3`, `P4`, `S60`, …) are global and assigned in [TODO.md](TODO.md) only. A
plan doc numbers its own steps locally (fields.md's 4.1–4.8, cited as "steps 4.1–4.8 of
fields.md") or with a prefix of its own (`CBF-1`, `OPW-1`, `AM-1`), never with a bare
workboard-style id. The prefix also marks a plan's rulings and the ids it cites from an
audit, so they never collide with a workboard id: randomness.md's rules, rulings and
amendments (`RD-R5`, `RD-D13`, `RD-A2`), the 2026-09-07 GRO501 audit's findings (`GRO-C1`,
`GRO-G1`, `GRO-F1`), the 2026-09-15 foundations review's findings and comments (`FD-F9`,
`FD-C6`). Workboard ids (`A5`, `P4`, `G1`, …) stay bare.

## Scheduled (a rung of ROADMAP §5 names them)

| Doc | Scope | Rung | Status |
| --- | --- | --- | --- |
| [automation-by-convention.md](automation-by-convention.md) | Port names drive the wiring; three reserved names (`plant`, `controller`, `observer`) are the roles every tool reads; the operators; the cost scored on the plant; steps AC-0–AC-8 in build order. History: [the decision record](../reviews/2026-10-11-automation-by-convention.md) and [the 2026-10-13 second opinion](../reviews/2026-10-13-automation-by-convention-second-opinion.md) | v0.2 B (P10), later steps to v0.3 and v0.9 | decided 2026-10-11, amended 2026-10-13, ready to build |
| [estimation-and-state-feedback.md](estimation-and-state-feedback.md) | State feedback and state estimation as duals: gains on arrays, blocks, designs on a model or a plant (`lqr`, `place`, `kalman`, `luenberger`, `lqg`), the estimation problem and tools to come, where the EKF fits. History: [the rulings](../reviews/2026-10-14-state-space-design-rulings.md) | v0.2 B (P4, landed 2026-10-14), later rows to v0.3 and v0.9 | decided 2026-10-13; P4 landed, the long view open |
| [randomness.md](randomness.md) | One convention for randomness: a `Distribution` has no time, a noise signal is a law held over a period, spectral density is the dt-free number; `NoiseSource` / `WhiteNoise`, counter-based draws, one key per experiment (`realize`) | v0.2 A (RN-2), v0.4 (RN-3 to RN-5, with V1's steps 7–9) | decided (RD-D1–RD-D24); RN-1 landed, RN-2 to RN-6 open |
| [fields.md](fields.md) | The `Field` object on `(x, u, t)`: `QuadraticField`, `GridField`, `CallableField`; the Lyapunov certificate, DP and the approximator speaking it | v0.4 (A1) | agreed, not started |
| [cost-params.md](cost-params.md) | Cost parameters as a dictionary like a system's; composite costs nested like diagram params | v0.3 wave V (A2, V1's step 6) | agreed, not started |
| [differentiable-problem.md](differentiable-problem.md) | One path from any parameter the problem holds to any traceable metric, through the native optimizer: `problem.compile(controller)` → an evaluator (the trajectory, the cost, the smallest margin, the mean over the draws), one scoring kernel on the native `Trajectory`, the program bridge `program(wrt=)`, the optimizer's program classes and first-order backend (S74), traceable analysis (S73); the guardrails, the bottom-up steps, the asks. History: [the core-objects review](../reviews/2026-10-18-core-objects-review.md) (the nouns are right, the verbs are duplicated) and [the consolidation record](../reviews/2026-10-18-differentiable-problem-consolidation.md) (three analyses compared, the probes) | v0.3 wave V (V1 steps 0–6, S74), v0.4 (V1 steps 7–9); S73 any time | consolidated proposal 2026-10-18, asks in its §9 |
| [geometry-module.md](geometry-module.md) | Workspace geometry home `core/geometry/` (Shape, Path, Track, Scene, bind, spatial Fields, course catalog); `planning.spatial` retires | v0.4 (A3) | agreed, not started |
| [pyro-port-remaining.md](pyro-port-remaining.md) | Pyro parity — open rows + name map | v0.3 wave G (C1) | open |
| [mppi.md](mppi.md) | Path integral control (MPPI): a planner beside trajopt (working name `PathIntegralPlanner`), wrapped by `ModelPredictiveController`; steps MP-1–MP-6 | v0.4 (S72, on the sampled kernel), v0.9 (MP-4, MP-5) | proposed; asks in its §10 |
| [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md) | One optimizer backend table for the offline and the parametric (MPC) paths | v0.3 wave V (OPW-1–OPW-5 with S74, V1's step 5; the evaluator placement a D3 row) | draft |
| [sampled-path.md](sampled-path.md) | The sampled path: one step model from a continuous plant to a step map (the eight discretizations in the code today, one contract with four knobs, `discretize` as the verb, one compiled step taking `params`, steps SM-0–SM-6) and the sampled loop as a tool on one kernel in two faces (§8) | v0.3 wave S (S75, then S31) | ideas to review 2026-10-21; open decisions in ROADMAP §6, maintainer-owned |
| [core-consolidation.md](core-consolidation.md) | The convergence pass: the vision as one spine (the differentiable path) and six pillars (classical and modern, nonlinear, trajectory and MPC, RL, identification and imitation, estimation); every module sorted as spine, pillar, content or side feature, with the usage inventory as evidence and the places the two lenses disagree; the tiers (side features out now · content between terms · pillar seeds kept and finished · the core that remains); the seeds the planned pillars grow on; the exit rule for the lane contract | v0.3 wave K (K7, with K1–K4) | analysis 2026-10-20, second pass by the vision; every item an ask, maintainer-owned |
| [repository-structure.md](repository-structure.md) | The long-term shape of the repository: the package split (keep the four bands, fix the hot spots), the examples tree (one folder per textbook chapter plus `courses/` and `research/`), where the teaching material lives (pinned Colab cells now, a course repo at v1.0); options, a recommendation, the asks | v0.9 (K5, K6), between terms | ideas to review 2026-10-20; serves K5 / K6 (ROADMAP §6), maintainer-owned |

## Research lane, unscheduled (a design exists; no rung yet)

| Doc | Scope | Status |
| --- | --- | --- |
| [rl-reward-scaling.md](rl-reward-scaling.md) | Automatic reward scaling for `ReinforcementLearningPlanner`: raw costs throttle PPO through the joint policy / critic gradient clip; `reward_scale="auto"` = `1 / (g_max dt)` from the failure-price sampling; evidence, options, steps RS-1–RS-4 | proposal 2026-09-27; rung for the maintainer (moves every RL demo's numbers once: between cohorts) |
| [cbf-safety-filter.md](cbf-safety-filter.md) | Control Barrier Functions: bicubic SDF interpolation in JAX, DCBF with slacks, HOCBF relative degree, `CBFSafetyFilter` block; the barrier becomes a `Field` once A1 lands | design draft 2026-09-11 |
| [future-policies.md](future-policies.md) | The current proposals projected onto MPPI, behavior cloning and diffusion policy: what fits, the three gaps (a `fit` verb for recorded data, memory and a random key per tick in the sampled loop, one stochastic-policy contract), six points to absorb and when | ideas to review 2026-10-21; feeds S31, V1's names, C4 and the behavior-cloning row; maintainer-owned |

Landed or superseded plan docs are deleted; their history is git, and the rulings they carried
are in the dated records (for example [P11 dropped](../reviews/2026-10-15-gro501-app-notebooks-dropped.md)).
