# Plans index

Three backlog homes (do not invent a fourth); audits live separately:

| Home | Use for |
| --- | --- |
| [ROADMAP.md](../../ROADMAP.md) | Plan of record: releases, two lanes, TRL ledger, course checklists, the path to v1.0, open decisions, out-of-scope |
| [TODO.md](TODO.md) | The workboard: every open step, by rung, with files and "done when" |
| **This folder** (other docs) | Design writeups for steps that need one — multi-step architecture / tradeoff docs |
| [../reviews/](../reviews/) | Dated architecture audits and decision records (read-only history) |

Pyro parity rows stay in [pyro-port-remaining.md](pyro-port-remaining.md)
(v0.2; open rows plus the landed name map). Landed contracts move into
[DESIGN.md](../../DESIGN.md) and the plan doc is deleted; one-line ideas go
to a Later bullet in [TODO.md](TODO.md) first — open a plan doc only when the
design needs a writeup.

Step ids (`R1`, `A3`, `P4`, `S60`, …) are global and assigned in [TODO.md](TODO.md) only. A
plan doc numbers its own steps locally (fields.md's 4.1–4.8, cited as "steps 4.1–4.8 of
fields.md") or with a prefix of its own (`CBF-1`, `OPW-1`, `AM-1`), never with a bare
workboard-style id.

## Scheduled (a rung of ROADMAP §5 names them)

| Doc | Scope | Rung |
| --- | --- | --- |
| [fields.md](fields.md) | The `Field` object on `(x, u, t)`: `QuadraticField`, `GridField`, `CallableField`; the Lyapunov certificate, DP and the approximator speaking it | v0.2 wave A (A1) — agreed 2026-09-15, not started |
| [cost-params.md](cost-params.md) | Cost parameters as a dictionary like a system's; composite costs nested like diagram params | v0.2 wave A (A2) — agreed 2026-09-13, not started |
| [geometry-module.md](geometry-module.md) | Workspace geometry home `core/geometry/` (Shape, Path, Track, Scene, bind, spatial Fields, course catalog); `planning.spatial` retires | v0.2 wave A (A3) — agreed 2026-09-21, not started |
| [naming.md](naming.md) | One naming rule for blocks and signals: the audit, six quick wins (five open), the larger alignments | v0.2 wave A (A4) — audit 2026-09-13 |
| [gro501-classical-control.md](gro501-classical-control.md) | The GRO501 stack: `minreal`, `place`, sensitivity verbs, observers, facades, the z question, the two notebooks | v0.2 wave B (P2–P11) — wave 1 partly landed 2026-09-07 |
| [pyro-port-remaining.md](pyro-port-remaining.md) | Pyro parity — open rows + name map | v0.3 wave C (C1) |
| [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md) | One optimizer backend table for the offline and the parametric (MPC) paths | v0.2 wave D (T5; the evaluator placement is a D3 row) — draft, July 2026 |

## Research lane, unscheduled (a design exists; no rung yet)

| Doc | Scope | Status |
| --- | --- | --- |
| [cbf-safety-filter.md](cbf-safety-filter.md) | Control Barrier Functions: bicubic SDF interpolation in JAX, DCBF with slacks, HOCBF relative degree, `CBFSafetyFilter` block; the barrier becomes a `Field` once A1 lands | design draft 2026-09-11 |
| [fidelity-maps.md](fidelity-maps.md) | `lift` / `project` maps across the car fidelity ladder, MPC broadcast at high fidelity | draft, July 2026 |
| [articulated-mechanism.md](articulated-mechanism.md) | One mechanism description (geometry, inertia, topology) feeding spatial RNEA/ABA and the symbolic path | draft, July 2026 |

Deleted (landed; decision records folded into DESIGN / ROADMAP / tests README / reviews):
- 2026-09-05: `control-block-contract.md`, `test-benchmark-consolidation.md`.
- 2026-09-11: `control-plots.md`, `derivatives-facade.md`, `planning-pipeline-architecture.md`, `neural-blocks-collection.md`, `mpc-tuning.md`.
- 2026-09-12: `constitution-integration.md` (rulings in [2026-09-12-governance-stack-audit.md](../reviews/2026-09-12-governance-stack-audit.md)), `v01-scope-alignment.md` (rulings in ROADMAP §6), `vehicle-abstraction.md` (teaching ladder S25; view ports → TODO Later).
- 2026-09-15: `core-objects-1-governance.md`, `core-objects-2-sets-distributions.md`, `core-objects-3-exit-rule.md` (contracts in CONSTITUTION / RULES / DESIGN), `rl-planner-vision.md` (R1–R7 landed; planner is DESIGN / ROADMAP TRL).
- 2026-09-22: `lyapunov-certificates.md` (implemented 2026-09-11; contract in DESIGN §3, open rulings in ROADMAP §6), `planning-solution-comparison.md` (landed 2026-09-17; contract in DESIGN §6, the demo rewrites are TODO D1.4), `standard-planning-problems.md` (the deterministic / stochastic pair shipped; the robust class is a Later idea), `core-objects-5-later-nouns.md` and `core-objects-6-demos.md` (folded into TODO A5 and D1); `core-objects-4-fields.md` renamed `fields.md`, `phase4-fidelity-maps.md` renamed `fidelity-maps.md`.
