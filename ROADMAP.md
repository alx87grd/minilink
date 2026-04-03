# Minilink Roadmap & Pyro 2.0 Migration

This document tracks the evolution of `minilink` towards full **Pyro 2.0** feature parity.

## 1. Project Maturity Matrix

| Component | Status | Next Milestone |
| :--- | :--- | :--- |
| **Core Abstractions** | **TRL 9** | Optimization of `global_signals` |
| **Simulation (Analysis)** | **TRL 2** | Refactor analysis interface and solve_ivp input fix (P0) |
| **Visualization** | **TRL 2** | Refactor system-to-rendering interface |
| **JAX Integration** | **TRL 2** | Experimental re-entrancy and tracing support |
| **Controllers** | **Planned** | Base classes and linear library |
| **Planning** | **Planned** | RRT and Direct Collocation ports |
| **Mechanical Templates** | **Planned** | `MechanicalSystem` base class |

> [!NOTE]
> Progress is tracked via **Task Readiness Levels (TRL 1-9)**. See [agent.md](agent.md) for detailed definitions of each level and the **3-Level Testing Strategy** (Automated, Manual, Demo).

---

## 2. Development Phases

### Phase 1: Modern Infrastructure (Done)
- [x] `pyproject.toml` (Hatch), CI (GitHub Actions).
- [x] `ruff` linting/formatting, `mypy` type hinting.

### Phase 2: Critical Bug Fixes (P0)
- [ ] **Fix `solve_ivp` External Inputs**: Ensure external inputs are polled during SciPy integration.
- [ ] **Trajectory Shape Alignment**: Fix docstring vs implementation mismatch `(n, time-steps)`.

### Phase 3: Core Architecture (P1)
- [x] Decouple Modeling from Graphics/Simulation (Lazy imports).
- [ ] **Consolidate Compilation Backends**: Refactor into a unified IR with NumPy/JAX classes. See [COMPILATION_REFACTOR.md](COMPILATION_REFACTOR.md) for the detailed plan.
- [ ] **Eliminate Shared Mutable Buffers**: Move to per-call `global_signals` allocation.
- [ ] **Standardize Signal Gathering**: Consolidate `src_type` loops into a shared helper.
- [ ] **Public API Surface**: Populate `minilink/__init__.py`.
- [ ] **Diagram Validation**: Add wiring guards in `add_subsystem()` and `connect()` (unique names, orphan ports, double-connection detection).
- [ ] **Simulation Config**: Create a `SimulationOptions` dataclass to replace ad-hoc `solve_ivp` kwargs.
- [ ] **Simulation Config**: Create a `SimulationOptions` dataclass to replace ad-hoc `solve_ivp` kwargs.
- [ ] **Context-Based Evaluation**: (Planned Re-evaluation) Transition from `(x, u, t, p)` arguments to a structured `Context` object *only if* hybrid systems (discrete state, modes) are introduced; keep flat textbook-style args for now.
- [ ] **Parameter Registry & Mapping**: Implement a compile-time map that translates user-friendly parameter names (dict/namespace) into high-performance flat arrays for the engine.

### Phase 4-5: API UX & Refactoring
- [ ] Operator overloading for composition (`>>`, `+`, `|`).
- [ ] Reference-based connections (`ctl.inputs.ref`).
- [ ] Automated output port dependency inference (`auto` dependencies).
- [ ] **Library Core Blocks**: Implement `StateSpaceSystem` (with explicit feedthrough via D matrix), `TransferFunction` and `PID`.
- [ ] **Linearization Tool**: Add a `linearize(diagram, x0, u0)` utility using JAX `jvp`.
- [ ] **Port Exporting**: Implement `export_input` and `export_output` in `DiagramSystem` to cleanly nest diagrams.
- [ ] **Port Caching**: Add optional results caching to `OutputPort` to avoid expensive re-computations in a single time step.
- [ ] **Unconnected Port Defaults**: Enforce or allow default values for unconnected input ports to ensure robust evaluation.

---

## 3. Pyro 2.0 Migration Strategy

`minilink` aims to replace the `pyro` robotics toolbox with a more flexible port-based architecture.

### Guiding Principles

-   **Mathematical Readability**: All code must read as close as possible to textbook equations (e.g., `dx = A@x + B@u`).
-   **MIMO Wiring**: Named ports replace flat vectors.
-   **True Composition**: `DiagramSystem` handles complex topologies.
-   **Compiled Execution**: `f_fast` (NumPy) and future JAX/XLA paths.
-   **Clean Decoupling**: Modeling core is headless-first.
-   **3-Level Verification**: Automated (pytest), User Manual, and Demo scripts for all features.

### Status of Core Dynamics Migration

| Pyro Feature | Minilink Equivalent | Status |
| :--- | :--- | :--- |
| `ContinuousDynamicSystem` | `DynamicSystem` | **TRL 9** |
| `StateSpaceSystem` | `StateSpaceSystem` | Planned |
| `MechanicalSystem` | `MechanicalSystem` | Planned |
| `Manipulator` | `Manipulator` | Planned |
| `StateObserver` | `DiagramSystem` composition | Planned |

### Estimated Timeline
-   **Phase 6-8 (Controllers, Cost, Mechanical)**: 1-2 weeks.
-   **Phase 9-11 (Planning, Tools, State-Space)**: 2-3 weeks.
-   **Total Estimated**: ~4-6 weeks to reach full feature parity.

---

## 4. Future Vision

-   **JAX-Based Optimization**: Leverage autodiff for trajectory optimization (Direct Collocation).
-   **Full Differentiable Simulation**: Introduce immutable `SimState` dataclass trees, `jax.custom_vjp` for the integrator, and parameter tracking (dynamic vs static) for end-to-end `jax.grad`.
-   **Hybrid System Simulation**: Support for discrete state updates and zero-crossing events for mode transitions.
-   **Differentiable LQR**: Integrate learning with control via differentiable Riccati solvers.
-   **Gymnasium Bridge**: Native `sys2gym` conversion for RL experiments.
-   **Interactive Game Loop**: Enhanced pygame-based real-time control.
-   **Multibody Plant Framework**: Introduce a "Plant" system that parses URDF/SDFormat descriptions into rigid body dynamics blocks.
-   **Scalar-Agnostic Blocks**: Design systems to be "scalar polymorphic" (NumPy, JAX, SymPy) via backend-agnostic mathematical expressions.
