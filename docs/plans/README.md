# Plans index

Three backlog homes (do not invent a fourth). Pyro parity is a dedicated audit,
not a separate “backlog kind”:

| Home | Use for |
| --- | --- |
| [ROADMAP.md](../../ROADMAP.md) | Maturity (TRL), teaching-release criteria & priorities, review queue, out-of-scope |
| [TODO.md](TODO.md) | Operational backlog — small fixes, pre-v0.2 hardening, demo pulls, new modules, Later ideas |
| **This folder** (other docs) | Active **design** writeups — multi-step architecture / tradeoff docs |

Pyro library/example rows stay in [pyro-port-remaining.md](pyro-port-remaining.md)
(source of truth; link from TODO, do not copy).

Landed contracts move into [DESIGN.md](../../DESIGN.md) and [ROADMAP.md](../../ROADMAP.md);
delete finished plan docs here. Do not park one-line ideas only as a new plan —
add a Later bullet in [TODO.md](TODO.md) first; open a plan doc only when the
design needs a writeup.

| Doc | Scope | Status |
| --- | --- | --- |
| [TODO.md](TODO.md) | Operational workboard (all kinds) | Ongoing |
| [pyro-port-remaining.md](pyro-port-remaining.md) | Pyro 2.0 demo/tool backlog | Ongoing |
| [planning-pipeline-architecture.md](planning-pipeline-architecture.md) | Result families + parametric scene bind (pipeline B) | Partial — A largely landed; B open |
| [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md) | Unify solver backend factory for offline + parametric MPC (IPOPT, etc.) | Draft — P1–P5 |
| [standard-planning-problems.md](standard-planning-problems.md) | Deterministic / stochastic / robust problem taxonomy | Draft |
| [neural-blocks-collection.md](neural-blocks-collection.md) | MLP / neural blocks roadmap | Future |
| [vehicle-abstraction.md](vehicle-abstraction.md) | Vehicle model view ports | Draft |
| [phase4-fidelity-maps.md](phase4-fidelity-maps.md) | Jax bicycle ladder lift/project maps + MPC Engine broadcast | Draft |
| [mpc-tuning.md](mpc-tuning.md) | MPC tuning project harness | Draft |
| [test-benchmark-consolidation.md](test-benchmark-consolidation.md) | Demo/catalog check layout | Partial |
