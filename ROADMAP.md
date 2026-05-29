# Minilink Roadmap

Maturity and priorities. Contracts: [DESIGN.md](DESIGN.md). Agent rules:
[agent.md](agent.md).

## 1. Maturity

| Area | TRL | Next |
| --- | --- | --- |
| Core | 7 | Lock exports, finish contract edge cases |
| Compile | 4 | Diagram parametric tier, backend parity |
| Simulation | 4 | `SimulationOptions`, forcing clarity |
| Optimization | 1 | Harden SciPy/Ipopt + evaluators |
| Planning/trajopt | 1 | Validate API, grow methods carefully |
| Graphical | 2 | Stabilize render/interactive loops |
| Dynamics catalog | 0–1 | Reviewed plants by domain |
| Symbolic/physics/control | 0–1 | MVPs only until clear use cases |

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

**P1** — Diagram parametric evaluators (`f_p`, `h_p`); diagram validation;
top-level `minilink` exports; `SimulationOptions`; NLP hardening.

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
