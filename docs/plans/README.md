# Plans index

Three backlog homes (do not invent a fourth); audits live separately. Two indexes point into
them without holding work of their own: [QUESTIONS.md](../dev/QUESTIONS.md), every question
that waits for the maintainer, and [KNOWN_ISSUES.md](../dev/KNOWN_ISSUES.md), the defects a
user can hit today, each with the row that fixes it.

| Home | Use for |
| --- | --- |
| [ROADMAP.md](../dev/ROADMAP.md) | Plan of record: releases, two lanes, TRL ledger, course checklists, the path to v1.0, open decisions, out-of-scope |
| [TODO.md](TODO.md) | The workboard: every open step, by rung, with files and "done when" |
| **This folder** (other docs) | Design writeups for steps that need one — multi-step architecture / tradeoff docs |
| [../reviews/](../reviews/) | Dated architecture audits and decision records (read-only history) |

**One fact, one home.** What to build lives in the plan doc; the order of work and each
step's "done when" in TODO; milestones, maturity and course goals in ROADMAP; *open* questions in
QUESTIONS.md; how and why a decision was taken in a dated record under `docs/reviews/`; landed
contracts in DESIGN. Other docs link to the home instead of restating it. A ruling goes to
the plan doc it shapes and to its dated record; it leaves QUESTIONS.md when it is taken.

Pyro parity rows stay in [pyro-port-remaining.md](pyro-port-remaining.md)
(v0.4; open rows plus the landed name map). Landed contracts move into
[DESIGN.md](../dev/DESIGN.md) and the plan doc is deleted; one-line ideas go
to a Later bullet in [TODO.md](TODO.md) first — open a plan doc only when the
design needs a writeup.

Step ids (`R1`, `A3`, `P4`, `S60`, …) are global and assigned in [TODO.md](TODO.md) only. A
plan doc numbers its own steps locally (fields.md's 4.1–4.8, cited as "steps 4.1–4.8 of
fields.md") or with a prefix of its own (`CBF-1`, `OPW-1`, `AM-1`), never with a bare
workboard-style id. The prefix also marks a plan's rulings and the ids it cites from an
audit, so they never collide with a workboard id: randomness.md's rules, rulings and
amendments (`RD-R5`, `RD-D13`, `RD-A2`), the 2026-09-15 foundations review's findings and comments (`FD-F9`,
`FD-C6`). Workboard ids (`A5`, `P4`, `G1`, …) stay bare.

## Scheduled (a rung of ROADMAP §5 names them)

| Doc | Scope | Rung | Status |
| --- | --- | --- | --- |
| [core-architecture.md](core-architecture.md) | **Read first for the core.** The pillars of v0.3, the complete core (the maintainer's direction of 2026-10-26): the two foundations (every object pure, every number a parameter; the step algebra first-class), one discrete judge, the draws and the seeded stochastic policy, the hybrid loop (one model, two runners), one generic parametric program with the stochastic program its case, the solve as a rollout (a proof of concept), Fields; for each, the vision with its status, the code today, the open points and the steps; the proposed build order; the core against the features; where `end-state-api.md`'s entries went | v0.3 | a consolidation 2026-10-26, no ruling; replaces `end-state-api.md` (deleted, its entries moved to the plan docs they shape) |
| [automation-by-convention.md](automation-by-convention.md) | What is left of the conventions: role-aware plots and the camera chosen when animating, the cost's draws, the sampled loop kept in step with the core rules and then made mainstream (AC-5–AC-8b). The conventions, the resolver and the operators landed (DESIGN §4). History: [the decision record](../reviews/2026-10-11-automation-by-convention.md) and [the 2026-10-13 second opinion](../reviews/2026-10-13-automation-by-convention-second-opinion.md) | plotting lane (AC-5, AC-6); v0.3 core (AC-7's draws with V1's step 7 and RN-4/RN-5; AC-8a and AC-8b with S31; direction of 2026-10-26) | decided 2026-10-11, amended 2026-10-13; AC-0–AC-4 landed 2026-10-10/14, AC-7's deterministic half in v0.3; AC-5–AC-8b open |
| [estimation-and-state-feedback.md](estimation-and-state-feedback.md) | State feedback and state estimation as duals: the layers side by side, the slots the later estimators take (time-varying and trajectory filters, the EKF, the sampled filters, parameter estimators), the estimation problem and its tools (Layer 3). History: [the rulings](../reviews/2026-10-14-state-space-design-rulings.md) | v0.4 (`LQGPlanner`, §5); v0.9 (the estimation follow-ups, §3, §5, §6) | decided 2026-10-13; P4 landed 2026-10-14 (DESIGN §3–§4); the long view open |
| [randomness.md](randomness.md) | One convention for randomness: a `Distribution` has no time, a noise signal is a law held over a period, spectral density is the dt-free number; `NoiseSource` / `WhiteNoise`, counter-based draws, one key per experiment (`realize`); its rulings and the 2026-09-30 review | v0.3 core (RN-2 in the first foundation; RN-3, RN-4, the rest of RN-5 and RN-6 the draws pillar; direction of 2026-10-26) | decided (RD-D1–RD-D24, its evaluator rulings superseded in part by R10, R15, R19, R27, R44); RN-1 landed 2026-09-30, RN-5's v0.3 part 2026-10-24; RN-2 to RN-6 open |
| [fields.md](fields.md) | The `Field` object on `(x, u, t)`: `QuadraticField`, `GridField`, `CallableField`; the Lyapunov certificate, DP and the approximator speaking it | v0.3 core (A1, the Fields pillar; direction of 2026-10-26) | agreed 2026-09-15, not started; step 4.7 done 2026-10-12 |
| [differentiable-problem.md](differentiable-problem.md) | What remains of the path from any parameter to any traceable metric: the target (the parameters × metrics table, landed and open cells), the three objects still to change (`StepRollout`, `Set`, `TrajectorySource`), traceable linear analysis (S73, §4.6), the guardrails for the open steps, the open steps (the `StepRollout` fold, 7, 7b, 8, 9). The landed path (steps 0–6, 7a) is DESIGN §4–§6. History: [the rulings](../reviews/2026-10-22-rulings-session-1.md) and [the run's log](../reviews/2026-10-24-v03-run-decisions.md) | v0.3 core (direction of 2026-10-26): the `StepRollout` fold, steps 7, 7b, 8, 9 and §9's discrete judge (a pillar); S73 any time, a v0.4 row (R76) | steps 0–6 and 7a landed 2026-10-24 (DESIGN §4–§6; the run log §2); the rest open, rows in TODO §1 |
| [geometry-module.md](geometry-module.md) | Workspace geometry home `core/geometry/` (Shape, Path, Track, Scene, bind, spatial Fields, course catalog); `planning.spatial` retires | v0.4 (A3) | agreed, not started |
| [pyro-port-remaining.md](pyro-port-remaining.md) | Pyro parity — open rows + name map | v0.4 (C1; moved from v0.3 by R48) | open: the parity rows and the migration guide; re-audit against the code before the guide |
| [mppi.md](mppi.md) | Path integral control (MPPI): a planner beside trajopt (`MPPIPlanner`, R106), wrapped by `ModelPredictiveController`; steps MP-1–MP-6 | v0.4 (S72, a feature on the core's sampled kernel and draws), v0.9 (MP-4, MP-5) | proposed 2026-10-02, not started; D26, D28 and D30 ruled 2026-10-25 (R106–R108), D27 open for v0.4 (D26–D30 now in its §10); waits on S31, RN-4 and the rest of RN-5 |
| [parametric-program.md](parametric-program.md) | The compiled MPC program binds every problem parameter at solve time (the start, the plant, the cost, the sets; the scene with A3): one program family whose frozen form is the plain `MathematicalProgram` (recommended), one evaluator, direct collocation and the MPC block first, steps PP-0–PP-5; and one optimizer backend table for the offline and the compiled paths (OPW-1–OPW-5) | v0.3 core (S76 and OPW-1–OPW-5, the generic parametric program finished all at once, the stochastic program its case; direction of 2026-10-26) | draft; its names ruled (`ParametricProgram`, the key `plant`, D19); the binding and D25 (now §2.1) open |
| [sampled-path.md](sampled-path.md) | The sampled loop (S31): a tool, not a `System`, on integer ticks; one kernel over clock ticks in a NumPy and a JAX face, the controller held over its period and the plant stepped by the step model; the details parked to the hybrid phase (D13–D15); the tentative draft vision of 2026-10-26 (§8.1–§8.5: `computer @ plant` a step diagram on a base clock that knows it is hybrid, pure blocks, held signals as state, the orchestrator kept as the clock runner, the simulation default open); kept for S53, the price of a parameter family. The step model it runs on is DESIGN §5 | v0.3 core: S31, the two-runner row, AC-8a / AC-8b and V1's step 7b (direction of 2026-10-26, which reassigns R86's v0.4) | ruled a tool with ticks (R13, R14), D14 and D15 ruled 2026-10-25 (R104, R105), D13 open (D13–D15 now its §8.6); the MPC's computational delay ruled (R123); §8.1–§8.5 a tentative draft vision, not decided; the step-model half landed 2026-10-24 (SM-0 to SM-8) and was cut |
| [native-optimizers.md](native-optimizers.md) | The upgraded program and optimizer: program categories (least squares, stochastic, parametric beside today's three), wrapped backends beside native algorithms written as pure array laws, the solve as a rollout (the update law in feedback with a gradient oracle, a `StepDiagramSystem`), and fully traced JAX solves (jit, vmap, unrolled and implicit gradients); first target a fixed-iteration MPC compiled and differentiable with its closed loop, long term convergence analysed with control tools; the state of the code, steps NS-1–NS-9 | v0.3 core (the proof of concept: NS-1, NS-2's fixed-`K` form, NS-8, NS-9; TODO §1.7); the rest v0.4 (TODO §4) | ideas to review 2026-10-25, from the maintainer's vision and two goals; three asks in QUESTIONS.md; the discrete step analysis goal 2 needs ruled in (R122); its proof of concept (NS-1, NS-2's fixed-`K` form, NS-8, NS-9) is v0.3 core (TODO §1.7, direction of 2026-10-26), the rest a v0.4 row (TODO §4); maintainer-owned |
| [repository-structure.md](repository-structure.md) | The examples tree (one folder per textbook chapter plus `courses/` and `research/`) and where the teaching material lives (pinned Colab cells now, a course repo at v1.0); options, a recommendation, the asks. The package question is settled (R66, R67) | v0.9 (K5, K6; the pin with V6), between terms | ideas to review 2026-10-20; the package question settled 2026-10-23/24; B and C open, maintainer-owned |

## Research lane, unscheduled (a design exists; no rung yet)

| Doc | Scope | Status |
| --- | --- | --- |
| [rl-reward-scaling.md](rl-reward-scaling.md) | Automatic reward scaling for `ReinforcementLearningPlanner`: raw costs throttle PPO through the joint policy / critic gradient clip; `reward_scale="auto"` = `1 / (g_max dt)` from the failure price's bound (`problem.cost_bound()`); evidence, options, steps RS-1–RS-4 | proposal 2026-09-27, not started (a Later row of TODO §6); rung for the maintainer (moves every RL demo's numbers once: between cohorts) |
| [cbf-safety-filter.md](cbf-safety-filter.md) | Control Barrier Functions: bicubic SDF interpolation in JAX, DCBF with slacks, HOCBF relative degree, `CBFSafetyFilter` block; the barrier becomes a `Field` once A1 lands | design draft 2026-09-11, amended 2026-10-12 (the barrier a `Field`); not started |
| [future-policies.md](future-policies.md) | The current proposals projected onto MPPI, behavior cloning and diffusion policy: what fits, the gaps (controller memory per tick in the sampled loop, one standard stochastic policy built as a seeded block like the noise block; a `fit` verb for recorded data noted, not critical), six points to absorb and when | ideas to review 2026-10-21; feeds S31, V1's names, C4 and the behavior-cloning row; D29 now its §5.1; points 2–4 of §5 are v0.3 core pillars (direction of 2026-10-26); maintainer-owned |

Landed or superseded plan docs are deleted; their history is git, and the rulings they carried
are in the dated records (for example [P11 dropped](../reviews/2026-10-15-gro501-app-notebooks-dropped.md)).
