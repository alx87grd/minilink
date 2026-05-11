# Minilink Roadmap

This document tracks active priorities and maturity. Design contracts live in
[DESIGN.md](DESIGN.md); maintainer/agent contribution rules live in
[agent.md](agent.md).

## 1. Maturity Matrix

| Area | Status | Next milestone |
| --- | --- | --- |
| Core objects | TRL 7 | Lock public exports and finish remaining contract details. |
| Compile/evaluators | TRL 4 | Add diagram parametric tier and strengthen backend parity. |
| Simulation | TRL 4 | Clarify solver options and forcing behavior. |
| Optimization | TRL 1 | Harden generic NLP pipeline with SciPy/Ipopt and NumPy/JAX evaluators. |
| Planning/trajopt | TRL 1 | Validate deterministic planning API and grow method coverage carefully. |
| Graphical | TRL 2 | Signal/topology backend seams added; stabilize render/interactive loops after core contracts settle. |
| Dynamics catalog | TRL 0-1 | Grow reviewed plants by domain. |
| Symbolic/physics/control | TRL 0-1 | Keep MVP examples working; expand only behind clear use cases. |

TRL definitions are in [agent.md (section 8)](agent.md#8-trl-lifecycle).

## 2. Completed Architecture Work

Modeling is separated from compile, simulation, optimization, planning, and
graphics. Diagrams compile through a shared `ExecutionPlan`; NumPy and JAX
dynamics evaluators exist for leaves and diagrams. `Trajectory` is the shared
sampled state-input object; `core.blocks` holds lightweight diagram blocks.

Optimization uses pure `MathematicalProgram` plus external evaluators and
`Optimizer` method presets (`scipy_slsqp`, `scipy_trust_constr`, `ipopt`).
Trajectory-optimization transcriptions emit `MathematicalProgram`; JAX comes from
traceable equations and program evaluators, not parallel JAX transcription types.
The native-array equation rule applies across those paths (see DESIGN §3).

## 3. Active Priorities

### P0

- Keep docs and public contracts aligned with code.
- Fix correctness issues that make reference paths disagree with compiled paths.

### P1

- Add diagram parametric evaluator tier for `f_p`, `h_p`, and `outputs_p`.
- Make diagram params semantics explicit and tested across NumPy and JAX.
- Add diagram validation for subsystem ids, port ids, wiring shape, and common
  connection mistakes.
- Clarify top-level public exports in `minilink/__init__.py`.
- Introduce a small `SimulationOptions`-style solver configuration surface.
- Continue hardening the generic NLP pipeline with SciPy and Ipopt backends.

### P2

- Add reusable core blocks such as state-space, transfer-function, and PID
  blocks once the API shape is stable.
- Add linearization and differentiation helpers on compiled evaluators.
- Improve diagram port exporting and nested-diagram ergonomics.
- Extend simulator-level forced-input helpers only after current workflows are
  stable.
- Factor interactive graphics into swappable real-time integrator and live-input
  backends.

## 4. Technical Direction

**Optimization stack** — Keep `MathematicalProgram` minimal (`J`, aggregate `h`
and `g`, bounds, optional derivatives, metadata). Keep `Optimizer` as the
method-preset surface; trajectory optimization stays as single transcription
classes per method where equations remain backend-native. Use JAX program
evaluators for `jit`, gradients, and Jacobians when traceable. Long term: richer
program classes (QP/LP) when justified; optimizers may inspect structure for solver
choice; generic NLP remains the fallback.

**JAX** — Prefer explicit `compile_backend` / evaluator choices; backend-native
algebra for simple sets and costs; `Jax<Plant>` twins only when one readable
implementation cannot serve both NumPy and JAX. Frame limits around traceability
and diagram params, not legacy parallel JAX layers.

## 5. Future Directions

- differentiable simulation rollouts;
- hybrid/event systems;
- LQR and control synthesis helpers;
- Gymnasium/RL bridges;
- richer interactive/cosimulation loops;
- multibody plant workflows and model import.
