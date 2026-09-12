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
| [rl-planner-vision.md](rl-planner-vision.md) | RL as a policy-family planner on the stochastic problem: exit rule and horizon semantics, feature/policy block, bare JAX vs Flax, Monte Carlo evaluation | Research lane — in progress: R1–R6 landed, R7 open |
| [planning-solution.md](planning-solution.md) | One result for every planner: `PlanningSolution` with policy, trajectory, evaluation, optional cost-to-go and a typed solver record; open-loop plans return a time-based policy block | Teaching surface — design converged 2026-09-12, implementation postponed |
| [lyapunov-certificates.md](lyapunov-certificates.md) | Region of attraction as an analysis verb: quadratic Lyapunov certificate, sampled sublevel search, Monte Carlo verification, phase-plane plot | Teaching surface — implemented 2026-09-11, awaiting rulings (D1–D5) |
| [phase4-fidelity-maps.md](phase4-fidelity-maps.md) | Jax bicycle ladder lift/project maps + MPC broadcast | Research lane — draft, Later |
| [articulated-mechanism.md](articulated-mechanism.md) | Mechanism IR, sym/num dual, spatial RNEA/ABA | Research lane — draft, Later |

Deleted (landed; decision records folded into DESIGN / ROADMAP / tests README / reviews):
- 2026-09-05: `control-block-contract.md`, `test-benchmark-consolidation.md`.
- 2026-09-11: `control-plots.md`, `derivatives-facade.md`, `planning-pipeline-architecture.md`, `neural-blocks-collection.md`, `mpc-tuning.md`.
- 2026-09-12: `constitution-integration.md` (record: [constitution second opinion](../reviews/2026-09-12-constitution-second-opinion.md)), `v01-scope-alignment.md` (rulings in ROADMAP §6), `vehicle-abstraction.md` (teaching ladder S25; view ports → TODO Later).
