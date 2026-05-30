# Minilink Roadmap

Maturity and priorities. Contracts: [DESIGN.md](DESIGN.md). Agent rules:
[agent.md](agent.md).

## 1. Maturity

| Area | TRL | Rationale | Next |
| --- | --- | --- | --- |
| Core + diagrams | 7 | Public API and diagram API are probably stable after the final port-declaration update. | Keep stable; finish export policy and remaining edge cases. |
| Compile | 4 | Integrated, but dynamic evaluator methods and exposed surface still need user review. | Review evaluator API, diagram parametric tier, and backend parity. |
| Simulation | 7 | Mature workflow with stable API and solver/forcing coverage. | Keep behavior stable; treat `SimulationOptions` as ergonomic cleanup, not a redesign. |
| Optimization | 5 | `MathematicalProgram` and `Optimizer` are integrated and useful, but backend details still need hardening. | Harden SciPy/Ipopt behavior and evaluator details before test-gated promotion. |
| Planning/trajopt | 2 | Some user review happened, but much of the module remains AI one-shot prototype work. | Re-evaluate architecture/API before deeper integration. |
| Graphical | 3 | Useful, but plotting/diagram APIs are still evolving. | Re-evaluate graphical API before freezing. |
| Animation | 3 | Substantial work exists, but renderer, camera, and live-loop contracts may still change. | Re-evaluate renderer/camera/live-loop API before freezing. |
| Dynamics catalog | 0 | Mostly empty relative to planned Pyro port. | Port and review interesting Pyro models by domain. |
| Symbolic/JAX physics engine | 1 | One-shot AI-generated demos, not a validated subsystem. | Keep isolated until clear use cases justify review. |
| Control | 0 | Mostly empty; planned Pyro port has not really happened yet. | Port and review Pyro control blocks when needed. |

TRL definitions: [agent.md §8](agent.md#8-trl-lifecycle).

## 2. Done (architecture)

- Separated model / compile / simulate / optimize / plan / graphics.
- `ExecutionPlan` + NumPy/JAX evaluators; shared `Trajectory`.
- Composition shortcuts (`+`, `>>`, `@`, `autowire`) → ordinary `DiagramSystem`.
- Explicit ports; `DynamicSystem` textbook ports via constructor options.
- Pure `MathematicalProgram` + `Optimizer`; backend-native trajopt transcriptions.
- Phase-plane plotting (`plot_phase_plane`, matplotlib).
- User docs: [README.md](README.md), [flows.md](flows.md) (minimal chains).

## 3. Priorities

**P0** — Docs/contracts aligned with code; compiled vs reference path parity.

**P1** — Dynamic evaluator API review; diagram parametric evaluators (`f_p`,
`h_p`); diagram validation; top-level `minilink` exports; NLP hardening.

**P2** — PID/transfer-function blocks; linearization; nested-diagram ergonomics;
forced-input helpers; swappable live graphics backends.

## 4. Review queue (needs maintainer sign-off)

- Optional `System` facade split from math contract (keep API equally readable).
- Public export policy for `minilink/__init__.py`.
- Diagram validation as separate `validate()` vs inline wiring.
- Trajopt transcription internal consolidation.
- Dynamic bicycle module split; placeholder planning modules (RRT, DP).
- Graphics/camera contract consolidation.

## 5. Future

Differentiable rollouts; hybrid/events; LQR; Gymnasium; cosimulation; multibody
import. Phase-plane follow-ups: Plotly, 3D, compiled grid eval.
