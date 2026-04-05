# Minilink Roadmap & Pyro 2.0 Migration

This document tracks the evolution of `minilink` towards full **Pyro 2.0** feature parity.

## 1. Project Maturity Matrix

| Component | Status | Next Milestone |
| :--- | :--- | :--- |
| **Core Abstractions** | **TRL 9** | Populate `minilink/__init__.py` |
| **DynamicsEvaluator ABC** | **TRL 9** | Mother class for all evaluators — done |
| **Leaf system backends** | **TRL 6** | Implement integration/differentiation methods |
| **Diagram backends** | **TRL 8** | Implement parametric tier (`f_p`) for diagrams |
| **JAX Integration** | **TRL 7** | Finalize jittable ODE solvers |
| **Simulation (Analysis)** | **TRL 4** | Fix `solve_ivp` external input + refactor interface |
| **Planning** | **Planned** | RRT and Direct Collocation ports |
| **Mechanical Templates** | **Planned** | `MechanicalSystem` base class |

> [!NOTE]
> Progress is tracked via **Task Readiness Levels (TRL 1-9)**. See [agent.md](agent.md) for definitions and the **3-Level Testing Strategy** (Automated, Manual, Demo).

---

## 2. Compilation API

### What's implemented

**`DynamicsEvaluator` ABC** ([`minilink/compile/evaluator.py`](minilink/compile/evaluator.py)) — the mother class for all compiled evaluators. Three tiers with math-first naming:

| Tier | Dynamics | Output | What's fixed |
|------|----------|--------|--------------|
| Standard | `f(x, u, t)` | `h(x, u, t)` | params frozen at compile time |
| Parametric | `f_p(x, u, t, params)` | `h_p(x, u, t, params)` | nothing — caller supplies params dict (JAX pytree) |
| IVP | `f_ivp(x, t)` | `h_ivp(x, t)` | u frozen (nominal ports) + params frozen |

Additional: `outputs(x, u, t)` / `outputs_p(x, u, t, params)` → `dict` of all output ports.

**Leaf system backends** (non-diagram):
- [`NumpyLeafEvaluator`](minilink/compile/numpy_evaluator.py) — wraps `System.f`/`System.h` with frozen params, nominal u snapshot. Full 3-tier support.
- [`JaxLeafEvaluator`](minilink/compile/jax_evaluator.py) — same, with core methods **pre-JIT-compiled and warm-started** at construction.

**Diagram backends** (compiled `DiagramSystem`):
- [`NumpyDiagramEvaluator`](minilink/compile/numpy_evaluator.py) — inherits from `DynamicsEvaluator`. `f(x, u, t)` implemented; `h`/`outputs`/parametric tier → `NotImplementedError`. Diagram-specific: `compute_outputs`, `compute_internal_signals`, `compute_internal_signals_dict`.
- [`JaxDiagramEvaluator`](minilink/compile/jax_evaluator.py) — same, JAX-traceable. `get_f_jit()` convenience method.

**Entry point**: `compile(system, backend="numpy"|"jax")` in [`compiler.py`](minilink/compile/compiler.py). Dispatches to leaf or diagram evaluator.

### File organization (`compile/`)

```
compile/
  evaluator.py         — DynamicsEvaluator ABC
  numpy_evaluator.py   — NumpyLeafEvaluator + NumpyDiagramEvaluator
  jax_evaluator.py     — JaxLeafEvaluator + JaxDiagramEvaluator
  execution_plan.py    — ExecutionPlan, PortOperation, StateOperation
  compiler.py          — compile(), compile_diagram(), build_execution_plan()
```

### Key design decisions

- **Dict params as JAX pytrees** — no flat `p` vector needed. JAX handles `dict` natively for `jit`, `grad`, `vmap`. `ParameterLayout` pack/unpack deferred until needed for scipy/CasADi optimizers.
- **Separate methods per tier** (not optional args) — avoids JAX pytree-structure issues when `params` is sometimes `None` vs `dict`.
- **ABC over Protocol** — children inherit default implementations (integration, differentiation); only `f`/`h`/`outputs` and their `_p` variants are abstract.
- **IVP tier snapshots `_u_nominal` at compile time** — JIT-safe on all backends (no runtime port reads).
- **Diagram `h()` / `outputs()` = NotImplementedError** — diagrams don't have a single primary output. Use diagram-specific `compute_outputs(x, u, t, ports)` for port-level access.

### TODO — next steps

- [ ] **Implement integration methods** in ABC: `rk4_step`/`euler_step`/`rollout` × 3 tiers. NumPy: Python loops. JAX: override with `lax.scan`.
- [ ] **Implement differentiation** (JAX): `jacobian_f_x`, `jacobian_f_u`, `jacobian_h_x`, `jacobian_h_u`, `linearize`. NumPy: finite differences later.
- [ ] **Diagram parametric tier** (`f_p`): requires adding `sys_id` to `PortOperation`/`StateOperation` so caller can supply per-subsystem params dict.
- [ ] **Diagram `h()` / `outputs()`**: define what "diagram output" means (exported ports via `connect_new_output_port`?) and wire through evaluator.
- [ ] **`vmap` batch rollout** (JAX): `vmap_rollout` over `x0_batch`, `u_batch`, or `params_batch`.
- [ ] **Tests**: full test suite for leaf evaluators (numpy + jax), param freeze, IVP, scipy integration.
- [ ] **`as_scipy_rhs()`**: already implemented in ABC, needs integration test with `solve_ivp`.
- [ ] **Update `minilink/__init__.py`** public exports.

---

## 3. Development Phases

### Phase 1: Modern Infrastructure (Done)
- [x] `pyproject.toml` (Hatch), CI (GitHub Actions).
- [x] `ruff` linting/formatting, `mypy` type hinting.

### Phase 2: Critical Bug Fixes (P0)
- [ ] **Fix `solve_ivp` External Inputs**: Ensure external inputs are polled during SciPy integration.
- [ ] **Trajectory Shape Alignment**: Fix docstring vs implementation mismatch `(n, time-steps)`.

### Phase 3: Core Architecture (P1)
- [x] Decouple Modeling from Graphics/Simulation (Lazy imports).
- [x] **Consolidate Compilation Backends**: Unified IR with NumPy/JAX classes. (Validated 2026-04-03)
- [x] **Eliminate Shared Mutable Buffers**: Fully stateless per-call `global_signals` allocation.
- [x] **Standardize Signal Gathering**: Consolidated `src_type` loops into a shared helper.
- [x] **`DynamicsEvaluator` ABC**: Public interface for all evaluators (`f`/`h` 3-tier API).
- [x] **Leaf system `compile()`**: `NumpyLeafEvaluator` + `JaxLeafEvaluator` with frozen params/u.
- [x] **Port diagram evaluators to `DynamicsEvaluator`**: `NumpyDiagramEvaluator` + `JaxDiagramEvaluator` inherit ABC. `f()` implemented, parametric/output tiers deferred. (2026-04-05)
- [x] **Reorganize `compile/` folder**: Fused leaf + diagram evaluators per backend (`numpy_evaluator.py`, `jax_evaluator.py`). (2026-04-05)
- [ ] **Diagram parametric tier**: Add `sys_id` to operations, implement `f_p` for diagrams.
- [ ] **Populate `minilink/__init__.py`**: Public exports.
- [ ] **Diagram Validation**: Add wiring guards in `add_subsystem()` and `connect()`.
- [ ] **Simulation Config**: Create `SimulationOptions` dataclass.

### Phase 4-5: API UX & Refactoring
- [ ] Operator overloading for composition (`>>`, `+`, `|`).
- [ ] Reference-based connections (`ctl.inputs.ref`).
- [ ] Automated output port dependency inference (`auto` dependencies).
- [ ] **Library Core Blocks**: `StateSpaceSystem`, `TransferFunction`, `PID`.
- [ ] **Linearization Tool**: `linearize(system, x0, u0)` via JAX evaluator's `linearize()`.
- [ ] **Port Exporting**: `export_input` / `export_output` in `DiagramSystem`.

---

## 4. Pyro 2.0 Migration Strategy

`minilink` aims to replace the `pyro` robotics toolbox with a more flexible port-based architecture.

### Guiding Principles

- **Mathematical Readability**: Code reads like textbook equations (e.g., `dx = A@x + B@u`).
- **MIMO Wiring**: Named ports replace flat vectors.
- **True Composition**: `DiagramSystem` handles complex topologies.
- **Compiled Execution**: NumPy and JAX backends with frozen params.
- **Clean Decoupling**: Modeling core is headless-first.
- **3-Level Verification**: Automated (pytest), User Manual, and Demo scripts.

### Status of Core Dynamics Migration

| Pyro Feature | Minilink Equivalent | Status |
| :--- | :--- | :--- |
| `ContinuousDynamicSystem` | `DynamicSystem` | **TRL 9** |
| Compiled evaluation (leaf) | `NumpyLeafEvaluator` / `JaxLeafEvaluator` | **TRL 6** |
| Compiled evaluation (diagram) | `NumpyDiagramEvaluator` / `JaxDiagramEvaluator` | **TRL 8** |
| `StateSpaceSystem` | `StateSpaceSystem` | Planned |
| `MechanicalSystem` | `MechanicalSystem` | Planned |
| `Manipulator` | `Manipulator` | Planned |

---

## 5. Future Vision

- **JAX-Based Optimization**: Leverage autodiff for trajectory optimization (Direct Collocation). Dict params as JAX pytrees enable `jax.grad` over physical parameters directly.
- **Full Differentiable Simulation**: `jax.custom_vjp` for the integrator, `lax.scan` rollouts, `vmap` batch simulations.
- **Hybrid System Simulation**: Discrete state updates and zero-crossing events.
- **Differentiable LQR**: Differentiable Riccati solvers.
- **Gymnasium Bridge**: Native `sys2gym` conversion for RL experiments.
- **Interactive Game Loop**: Enhanced pygame-based real-time control.
- **Multibody Plant Framework**: URDF/SDFormat → rigid body dynamics blocks.
