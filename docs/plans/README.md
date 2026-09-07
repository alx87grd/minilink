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
| [control-plots.md](control-plots.md) | Root locus, Nyquist, margins, step response; MATLAB look; one figure spec rendered on matplotlib and plotly | Teaching surface — implemented 2026-09-07 |
| [derivatives-facade.md](derivatives-facade.md) | `sys.jacobian("f", "x")`, `sys.linearize(...)` and one calling pattern for the analysis family (`bode`, `pzmap`, `transfer_function`, …) | Teaching surface — implemented 2026-09-06 |
| [planning-pipeline-architecture.md](planning-pipeline-architecture.md) | Result families + parametric scene bind (pipeline B) | Research lane — partial, Later |
| [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md) | Solver backend factory for offline + parametric MPC | Research lane — draft, Later |
| [standard-planning-problems.md](standard-planning-problems.md) | Deterministic / stochastic / robust problem taxonomy | Research lane — draft, Later |
| [neural-blocks-collection.md](neural-blocks-collection.md) | MLP / neural blocks | Research lane — proposal, Later |
| [vehicle-abstraction.md](vehicle-abstraction.md) | Vehicle model view ports | Research lane — draft; the teaching ladder itself is settled (TODO S25) |
| [phase4-fidelity-maps.md](phase4-fidelity-maps.md) | Jax bicycle ladder lift/project maps + MPC broadcast | Research lane — draft, Later |
| [mpc-tuning.md](mpc-tuning.md) | MPC tuning project harness | Research lane — project notes |
| [articulated-mechanism.md](articulated-mechanism.md) | Mechanism IR, sym/num dual, spatial RNEA/ABA | Research lane — draft, Later |

Deleted 2026-09-05 (landed; decision records folded into DESIGN / tests README):
`control-block-contract.md`, `test-benchmark-consolidation.md`.
