# Plans index

Three backlog homes (do not invent a fourth); audits live separately:

| Home | Use for |
| --- | --- |
| [ROADMAP.md](../../ROADMAP.md) | Plan of record: releases, two lanes, TRL ledger, course checklists, the path to v1.0, open decisions, out-of-scope |
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
[DESIGN.md](../../DESIGN.md) and the plan doc is deleted; one-line ideas go
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
| [automation-by-convention.md](automation-by-convention.md) | Port names drive the wiring; three reserved names (`plant`, `controller`, `estimator`) are the roles every tool reads; the operators; the cost scored on the plant; steps AC-0–AC-8 in build order. History: [the decision record](../reviews/2026-10-11-automation-by-convention.md) | v0.2 B (P10), later steps to v0.3 and v0.9 | decided 2026-10-11, ready to build |
| [gro501-classical-control.md](gro501-classical-control.md) | The GRO501 stack: `minreal`, `place`, sensitivity verbs, observers, facades, the z question, the two notebooks | v0.2 B (P2–P11) | mostly landed; open: P4, P6 (held), TB-b, S61, P11 |
| [randomness.md](randomness.md) | One convention for randomness: a `Distribution` has no time, a noise signal is a law held over a period, spectral density is the dt-free number; `NoiseSource` / `WhiteNoise`, counter-based draws, one key per experiment (`realize`) | v0.2 A (RN-2), v0.3 (RN-3 to RN-5) | decided (RD-D1–RD-D24); RN-1 landed, RN-2 to RN-6 open |
| [naming.md](naming.md) | One naming rule for blocks and signals: the audit, the quick wins, the larger alignments | v0.2 A (A4) | quick wins 1, 3, 6 open |
| [fields.md](fields.md) | The `Field` object on `(x, u, t)`: `QuadraticField`, `GridField`, `CallableField`; the Lyapunov certificate, DP and the approximator speaking it | v0.3 (A1) | agreed, not started |
| [cost-params.md](cost-params.md) | Cost parameters as a dictionary like a system's; composite costs nested like diagram params | v0.3 (A2) | agreed, not started |
| [geometry-module.md](geometry-module.md) | Workspace geometry home `core/geometry/` (Shape, Path, Track, Scene, bind, spatial Fields, course catalog); `planning.spatial` retires | v0.3 (A3) | agreed, not started |
| [pyro-port-remaining.md](pyro-port-remaining.md) | Pyro parity — open rows + name map | v0.3 C (C1) | open |
| [mppi.md](mppi.md) | Path integral control (MPPI): a planner beside trajopt (working name `PathIntegralPlanner`), wrapped by `ModelPredictiveController`; steps MP-1–MP-6 | v0.3 C (S72), v0.9 (MP-4, MP-5) | proposed; asks in its §10 |
| [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md) | One optimizer backend table for the offline and the parametric (MPC) paths | v0.2 D (D3: OPW-1–OPW-5 a hardening row of their own, out of T5's byte-identical pass; the evaluator placement another D3 row) | draft |

## Research lane, unscheduled (a design exists; no rung yet)

| Doc | Scope | Status |
| --- | --- | --- |
| [rl-reward-scaling.md](rl-reward-scaling.md) | Automatic reward scaling for `ReinforcementLearningPlanner`: raw costs throttle PPO through the joint policy / critic gradient clip; `reward_scale="auto"` = `1 / (g_max dt)` from the failure-price sampling; evidence, options, steps RS-1–RS-4 | proposal 2026-09-27; rung for the maintainer (moves every RL demo's numbers once: between cohorts) |
| [cbf-safety-filter.md](cbf-safety-filter.md) | Control Barrier Functions: bicubic SDF interpolation in JAX, DCBF with slacks, HOCBF relative degree, `CBFSafetyFilter` block; the barrier becomes a `Field` once A1 lands | design draft 2026-09-11 |
| [fidelity-maps.md](fidelity-maps.md) | `lift` / `project` maps across the car fidelity ladder, MPC broadcast at high fidelity | draft, July 2026 |
| [articulated-mechanism.md](articulated-mechanism.md) | One mechanism description (geometry, inertia, topology) feeding spatial RNEA/ABA and the symbolic path | draft, July 2026; the general serial chain of [TODO.md](TODO.md) §5 (after v1.0) |

Deleted (landed; decision records folded into DESIGN / ROADMAP / tests README / reviews):
- 2026-09-05: `control-block-contract.md`, `test-benchmark-consolidation.md`.
- 2026-09-11: `control-plots.md`, `derivatives-facade.md`, `planning-pipeline-architecture.md`, `neural-blocks-collection.md`, `mpc-tuning.md`.
- 2026-09-12: `constitution-integration.md` (rulings in [2026-09-12-governance-stack-audit.md](../reviews/2026-09-12-governance-stack-audit.md)), `v01-scope-alignment.md` (rulings in ROADMAP §6), `vehicle-abstraction.md` (teaching ladder S25; view ports → TODO Later).
- 2026-09-15: `core-objects-1-governance.md`, `core-objects-2-sets-distributions.md`, `core-objects-3-exit-rule.md` (contracts in CONSTITUTION / RULES / DESIGN), `rl-planner-vision.md` (R1–R7 landed; planner is DESIGN / ROADMAP TRL).
- 2026-09-22: `lyapunov-certificates.md` (implemented 2026-09-11; contract in DESIGN §3, open rulings in ROADMAP §6), `planning-solution-comparison.md` (landed 2026-09-17; contract in DESIGN §6, the demo rewrites are TODO D1.4), `standard-planning-problems.md` (the deterministic / stochastic pair shipped; the robust class is a Later idea), `core-objects-5-later-nouns.md` and `core-objects-6-demos.md` (folded into TODO A5 and D1); `core-objects-4-fields.md` renamed `fields.md`, `phase4-fidelity-maps.md` renamed `fidelity-maps.md`.
