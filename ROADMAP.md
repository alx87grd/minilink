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
| Graphical | TRL 2 | Stabilize render/interactive loops after core contracts settle. |
| Dynamics catalog | TRL 0-1 | Grow reviewed plants by domain. |
| Symbolic/physics/control | TRL 0-1 | Keep MVP examples working; expand only behind clear use cases. |

TRL definitions are in [agent.md (section 8)](agent.md#8-trl-lifecycle).

## 2. Completed Architecture Work

- Core modeling is separated from compile, simulation, optimization, planning,
  and graphics.
- Diagram compilation uses a shared `ExecutionPlan`.
- NumPy and JAX dynamics evaluators exist for leaves and diagrams.
- `Trajectory` is the official sampled state-input object.
- `core.blocks` is the home for sources, integrators, and lightweight signal
  blocks.
- `MathematicalProgram` is now a pure NLP description with external
  `program_evaluator` objects.
- `Optimizer` uses method presets such as `scipy_slsqp`, `scipy_trust_constr`,
  and `ipopt`; it compiles the program at initialization and solves from `z0`.
- Trajectory optimization transcriptions emit generic `MathematicalProgram`
  objects; JAX comes from traceable equations plus program evaluators, not from
  separate JAX transcription classes.
- The native-array equation rule is now explicit for `System.f/h`, output-port
  compute functions, sets, costs, transcriptions, and mathematical programs.

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

## 4. Planning And Optimization Direction

Near term:

- keep `MathematicalProgram` pure: `J`, aggregate `h`, aggregate `g`, bounds,
  optional derivatives, metadata, and `problem_class`;
- keep `Optimizer` as the bound method-preset interface;
- keep trajectory-optimization methods as single public transcription classes
  with backend-native equations where possible;
- use JAX program evaluators for `jit`, gradients, Jacobians, and optional
  dense Hessians when the program is traceable.

Long term:

- add more precise program abstractions such as QP or LP when there is a real
  solver-selection use case;
- let optimizers inspect program class and structure to choose better solvers;
- keep the generic NLP path as the stable fallback.

## 5. JAX Direction

JAX support should grow through explicit backend choices:

- `compile_backend="jax"` for compatible dynamics/simulation/trajopt paths;
- backend-native algebraic helpers for simple sets and costs;
- explicit `Jax<Plant>` twins only for complex plants or physics where one
  implementation cannot stay readable for both NumPy and JAX.

Remaining limitations should be framed around traceability and diagram params,
not old parallel JAX transcription/cost classes.

## 6. Future Directions

- differentiable simulation rollouts;
- hybrid/event systems;
- LQR and control synthesis helpers;
- Gymnasium/RL bridges;
- richer interactive/cosimulation loops;
- multibody plant workflows and model import.
