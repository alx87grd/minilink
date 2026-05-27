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
Optional diagram composition shortcuts (`+`, `>>`, `@`, and conservative
`autowire`) now build ordinary `DiagramSystem` objects while the explicit
named-port API remains the canonical general interface.

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

- Add phase-plane vector-field plotting with trajectory overlays as a small
  Pyro-parity graphics feature.
- Add remaining reusable control/core blocks such as transfer-function and PID
  once the API shape is stable; exact state-space lives under dynamics
  abstractions.
- Add linearization and differentiation helpers after the first-pass dynamics
  abstraction tree is stable.
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

**Dynamics abstraction tree** — Keep the executable root at `DynamicSystem`.
Reusable dynamics bases live in `dynamics/abstraction`; catalog plants keep short
domain names such as `Pendulum`, `CartPole`, and `DynamicBicycle`.

First-pass bases:

- `StateSpaceSystem`: exact LTI `dx = A @ x + B @ u`, `y = C @ x + D @ u`.
- `MechanicalSystem`: generalized-coordinate mechanics with `x = [q, dq]`;
  default hooks are native-array, but concrete subclass traceability depends on
  the overridden equations.
- `GeneralizedMechanicalSystem`: generalized/body-velocity mechanics with
  `x = [q, v]`, `qdot = N(q) @ v`, and native-array default hooks.

Use named input ports plus explicit force/allocation hooks for mixed input
semantics such as steering, elevator angles, tire forces, aerodynamic surfaces,
and propulsors. Do not add `WithPositionInputs` inheritance branches.

Backburner items: `KinematicSystem`, `ManipulatorSystem`, linearization, and
analysis helpers such as task-space dynamics and manipulability tools.

## 5. Phase B Review Queue

These are larger simplification or contract moves identified during the Phase A
cleanup pass. They need maintainer review before implementation.

- Split the `System` facade from the math contract: keep `f`, `h`, ports,
  params, and state metadata central; move plotting, graph, animation, and game
  shortcuts behind a small facade/mixin layer only if the public API stays
  equally readable.
- Freeze top-level public exports in `minilink/__init__.py`; decide whether the
  package should stay namespace-only or expose a tiny textbook API such as
  `System`, `DiagramSystem`, `Simulator`, `Trajectory`, and common blocks.
- Decide whether diagram validation belongs in a separate debug/check function
  rather than in the plain wiring methods. The default source path should stay
  readable and direct.
- Design the diagram parametric evaluator tier (`f_p`, `h_p`, `outputs_p`) so
  NumPy and JAX diagram params have one explicit contract instead of today’s
  bind-or-recompile limitation.
- Consolidate trajectory-optimization transcription internals: direct
  collocation, shooting, and multiple shooting repeat objective integration,
  path constraints, boundary constraints, and result reconstruction.
- Split the large dynamic bicycle module into reviewed math, tire, graphics, and
  JAX-specific sections or modules without changing the user-facing plant names.
- Decide what to do with placeholder planning modules (`search/rrt.py` and
  `policy_synthesis/dynamic_programming.py`): implement behind minimal contracts
  or mark them clearly as non-public prototypes.
- Introduce the planned solver-options object for simulation so `Simulator`,
  `compute_trajectory`, and forced-input paths do not grow more keyword
  ceremony.
- Review the graphics contract after core stabilizes: camera/framing is now
  useful but still provisional, and renderers share enough behavior to justify a
  focused consolidation pass.

## 6. Future Directions

- differentiable simulation rollouts;
- hybrid/event systems;
- LQR and control synthesis helpers;
- Gymnasium/RL bridges;
- richer interactive/cosimulation loops;
- multibody plant workflows and model import.

## 7. Phase-Plane Plot Plan

Phase-plane plotting is the next small Pyro-parity graphics feature. Pyro's
`PhasePlot` builds a 2D grid over two selected state coordinates, evaluates
`f(x, u, t)` at each grid point with all other state components held at a
nominal value, and renders the selected derivative components as a quiver or
stream plot. Minilink should keep the same user value while fitting the current
graphics contracts.

### Scope

- Implement 2D phase-plane vector fields for any `System` with at least one
  state.
- Support selecting `x_axis` and `y_axis`; allow the same state index on both
  axes for one-state systems, matching Pyro's simple-integrator demos.
- Support optional trajectory overlays with start/end markers using Minilink's
  canonical `Trajectory` shape `(n, N)`.
- Support `show=False` for tests and headless notebooks.
- Use matplotlib first. Plotly and 3D phase plots are follow-up features.

### Public API

- Add `minilink.graphical.phase_plane` with:
  - `PhasePlaneSpec`: backend-neutral grid, vector field, labels, units,
    bounds, title, and optional trajectory-overlay data.
  - `build_phase_plane_spec(sys, traj=None, *, x_axis=0, y_axis=None, u=None,
    x_ref=None, t=0.0, bounds=None, grid_shape=(21, 21), params=None)`.
  - `plot_phase_plane(sys, traj=None, *, x_axis=0, y_axis=None, u=None,
    x_ref=None, t=0.0, bounds=None, grid_shape=(21, 21), streamplot=False,
    show=True, **kwargs)`.
- Export the new helpers through `minilink.graphical.plotting`.
- Add `System.plot_phase_plane(...)` as the object-level facade. If `traj` is
  omitted and `self.traj` exists, overlay it; otherwise plot only the vector
  field.
- Add `System.plot_phase_plane_trajectory(...)` only as a Pyro-friendly alias if
  the facade remains simple; internally it should call `plot_phase_plane`.

### Data And Defaults

- Default `x_ref` to `sys.state.nominal_value` when available, otherwise zeros.
- Default `u` to `sys.get_u_from_input_ports()` so named-port systems respect
  input-port nominal values.
- Default `bounds` to finite `sys.state.lower_bound` / `upper_bound` for the two
  selected axes.
- If selected bounds are infinite, derive bounds from the overlay trajectory
  with padding; if no trajectory is available, use a conservative centered
  fallback such as `[-10, 10]`.
- Validate axis indices, grid dimensions, vector dimensions, and finite plotting
  bounds with clear `ValueError` messages.

### Rendering

- Keep vector-field construction NumPy boundary code. It can call `sys.f` in a
  simple nested loop; no compiled/vectorized path is needed for the first
  implementation.
- Render with `Axes.quiver` by default and `Axes.streamplot` when requested.
- Label axes from `sys.state.labels` and `sys.state.units`.
- Return the existing `PlotResult` shape (`backend`, `payload`, `figure`,
  `axes`) for consistency with time-signal plots.
- Follow existing matplotlib environment behavior: set PDF/PS font embedding,
  avoid blocking when `show=False`, and rely on style helpers where they fit.

### Tests And Demo

- Add unit tests covering:
  - vector-field values for a simple two-state test system;
  - one-state same-axis behavior;
  - finite state-bound defaults and infinite-bound fallback;
  - trajectory overlay shape/orientation;
  - `show=False` matplotlib rendering under Agg.
- Add `examples/scripts/plots/demo_phase_plane.py` with `Pendulum` and
  `FloatingMass1D` examples.
- Update README/DESIGN graphics text after the API lands.

### Follow-Ups

- 3D phase plots.
- Open-loop versus closed-loop dual vector-field overlays.
- Plotly rendering.
- Compiled/vectorized evaluator acceleration for dense grids.
- Cost-to-go/policy overlays once dynamic programming is implemented.
