# Plans index

Three backlog homes (do not invent a fourth); audits live separately:

| Home | Use for |
| --- | --- |
| [ROADMAP.md](../../ROADMAP.md) | Plan of record: releases, two lanes, TRL ledger, GRO860 checklist, phases, review queue, out-of-scope |
| [TODO.md](TODO.md) | Step-level workboard for the phases, carried-over hardening rows, Later ideas |
| **This folder** (other docs) | Design writeups for the **research lane** — multi-step architecture / tradeoff docs |
| [../reviews/](../reviews/) | Dated architecture audits and interview decision records (read-only history) |

Pyro parity rows stay in [pyro-port-remaining.md](pyro-port-remaining.md)
(v0.2 milestone; open rows plus the landed name map). Landed contracts move
into [DESIGN.md](../../DESIGN.md) and the plan doc is deleted; one-line ideas
go to a Later bullet in [TODO.md](TODO.md) first — open a plan doc only when
the design needs a writeup.

| Doc | Scope | Status |
| --- | --- | --- |
| [TODO.md](TODO.md) | Operational workboard (phases D–3, v0.2 pulls) | Ongoing |
| [pyro-port-remaining.md](pyro-port-remaining.md) | Pyro parity — open rows + name map | v0.2 |
| [gro501-classical-control.md](gro501-classical-control.md) | Fix plan for the GRO501 objective — phantom `PID` modes, `minreal`, `place`, observers, sensitivity verbs, the z-domain question | Teaching surface — draft, awaiting rulings (v0.2) |
| [cbf-safety-filter.md](cbf-safety-filter.md) | Control Barrier Functions (CBF): bicubic SDF interpolation in JAX, DCBF with slacks, HOCBF relative degree, and `CBFSafetyFilter` block | Research lane — design draft (2026-09-11) |
| [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md) | Solver backend factory for offline + parametric MPC | Research lane — draft, Later |
| [standard-planning-problems.md](standard-planning-problems.md) | Deterministic / stochastic / robust problem taxonomy | Research lane — draft, Later |
| [cost-params.md](cost-params.md) | Cost parameters as a dictionary like a system's, composite costs nested like diagram params; the cost-integrator block toward a differentiable closed-loop cost | Core contract — design agreed 2026-09-13, not started |
| [naming.md](naming.md) | One naming rule for blocks and signals: audit of class names, display names, ids, role keys, wires, params paths and labels; six quick wins and the larger alignments | Core contract — audit done 2026-09-13, nothing applied |
| [core-objects-4-fields.md](core-objects-4-fields.md) | The `Field` object on `(x, u, t)`: `QuadraticField`, `GridField`, `CallableField`, the Lyapunov certificate, DP and the approximator speaking it; CBF plan amended | Core contract — agreed 2026-09-15, not started |
| [geometry-module.md](geometry-module.md) | Workspace geometry home: `core/geometry/` (Shape, Path, Track, Scene, bind, spatial Fields, course catalog); `planning.spatial` retires; PurePursuit and CBF read it with no planner | Core contract — agreed 2026-09-21, not started |
| [core-objects-5-later-nouns.md](core-objects-5-later-nouns.md) | `Gaussian(cov=)` with the disturbance convention, parameter-dictionary sets and distributions, the Hamiltonian, `NoiseSource`, `UnionSet` | Later / v0.2 |
| [core-objects-6-demos.md](core-objects-6-demos.md) | Demos and notebooks to the minimal rule: native plots and prints, the flat-demo ratchet, the sweep file by file | Student-facing — rule agreed 2026-09-15, sweep not started |
| [lyapunov-certificates.md](lyapunov-certificates.md) | Region of attraction as an analysis verb: quadratic Lyapunov certificate, sampled sublevel search, Monte Carlo verification, phase-plane plot | Teaching surface — implemented 2026-09-11, awaiting rulings (D1–D5) |
| [phase4-fidelity-maps.md](phase4-fidelity-maps.md) | Jax bicycle ladder lift/project maps + MPC broadcast | Research lane — draft, Later |
| [articulated-mechanism.md](articulated-mechanism.md) | Mechanism IR, sym/num dual, spatial RNEA/ABA | Research lane — draft, Later |

Deleted (landed; decision records folded into DESIGN / ROADMAP / tests README / reviews):
- 2026-09-05: `control-block-contract.md`, `test-benchmark-consolidation.md`.
- 2026-09-11: `control-plots.md`, `derivatives-facade.md`, `planning-pipeline-architecture.md`, `neural-blocks-collection.md`, `mpc-tuning.md`.
- 2026-09-12: `constitution-integration.md` (rulings in [2026-09-12-governance-stack-audit.md](../reviews/2026-09-12-governance-stack-audit.md)), `v01-scope-alignment.md` (rulings in ROADMAP §6), `vehicle-abstraction.md` (teaching ladder S25; view ports → TODO Later).
- 2026-09-15: `core-objects-1-governance.md`, `core-objects-2-sets-distributions.md`, `core-objects-3-exit-rule.md` (contracts in CONSTITUTION / RULES / DESIGN), `rl-planner-vision.md` (R1–R7 landed; planner is DESIGN / ROADMAP TRL).
