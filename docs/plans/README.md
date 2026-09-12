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
| [planning-pipeline-architecture.md](planning-pipeline-architecture.md) | Result families + parametric scene bind (pipeline B) | Research lane — partial, Later |
| [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md) | Solver backend factory for offline + parametric MPC | Research lane — draft, Later |
| [standard-planning-problems.md](standard-planning-problems.md) | Deterministic / stochastic / robust problem taxonomy | Research lane — draft, Later |
| [neural-blocks-collection.md](neural-blocks-collection.md) | MLP / neural blocks | Research lane — proposal, Later |
| [rl-planner-vision.md](rl-planner-vision.md) | RL as a policy-family planner on the stochastic problem: exit rule and horizon semantics, feature/policy block, bare JAX vs Flax, Monte Carlo evaluation | Research lane — in progress: R1–R5b landed 2026-09-10, R6–R7 open |
| [lyapunov-certificates.md](lyapunov-certificates.md) | Region of attraction as an analysis verb: quadratic Lyapunov certificate, sampled sublevel search, Monte Carlo verification, phase-plane plot | Teaching surface — design draft 2026-09-11, awaiting rulings (D1–D5) |
| [vehicle-abstraction.md](vehicle-abstraction.md) | Vehicle model view ports | Research lane — draft; the teaching ladder itself is settled (TODO S25) |
| [phase4-fidelity-maps.md](phase4-fidelity-maps.md) | Jax bicycle ladder lift/project maps + MPC broadcast | Research lane — draft, Later |
| [mpc-tuning.md](mpc-tuning.md) | MPC tuning project harness | Research lane — project notes |
| [articulated-mechanism.md](articulated-mechanism.md) | Mechanism IR, sym/num dual, spatial RNEA/ABA | Research lane — draft, Later |
| [v01-scope-alignment.md](v01-scope-alignment.md) | v0.1 doc/lane alignment: continuous core, RL vs SB3, hybrid long-term, module demote/promote checklist, examples `tutorial/`/`teaching/`, pip timing | Maintainer draft 2026-09-11 — apply to ROADMAP when approved |
| [constitution-integration.md](constitution-integration.md) | Doc consolidation behind [CONSTITUTION.md](../../CONSTITUTION.md): separation of concerns across DESIGN/AGENTS/ROADMAP, the fate of DESIGN, the seven-layer review ladder | Main slices landed 2026-09-12 ([second opinion](../reviews/2026-09-12-constitution-second-opinion.md)) |

Deleted (landed; decision records folded into DESIGN / tests README):
- 2026-09-05: `control-block-contract.md`, `test-benchmark-consolidation.md`.
- 2026-09-11: `control-plots.md`, `derivatives-facade.md`.
