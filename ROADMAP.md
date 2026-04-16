# Minilink Roadmap & Pyro 2.0 Migration

This document tracks the evolution of `minilink` towards full **Pyro 2.0** feature parity.

## 1. Project Maturity Matrix

| Component | Status | Next Milestone |
| :--- | :--- | :--- |
| **Core Abstractions** | **TRL 7** | Finalize public exports and validate remaining details around the stable user-facing API |
| **Compile pipeline** | **TRL 4** | Validate architecture and integrate the evaluator stack more fully across the library |
| **Leaf evaluators** | **TRL 4** | Validate architecture; add differentiation and richer rollout helpers |
| **Diagram evaluators** | **TRL 4** | Validate architecture; add parametric tier (`f_p` / `outputs_p`) and diagram `h_p` |
| **Simulation** | **TRL 4** | Complete the transition away from legacy `core.analysis` simulator code and unify nominal / forced paths |
| **Graphical** | **TRL 1** | Keep MVP renderers usable while avoiding premature API stabilization |
| **Mechanics** | **TRL 1** | Keep numeric mechanics path functional; defer maturity claims until user validation begins |
| **Symbolic mechanics** | **TRL 1** | Keep symbolic derivation/export path functional enough for review and examples |
| **Physics** | **TRL 1** | Keep JAX contact world demos working; continue sphere-plane MVP |
| **Blocks library** | **TRL 0** | Decide what belongs in the long-term reusable block library before rating maturity |
| **Planning** | **TRL 0** | Not started as a rated subsystem |
| **Control** | **TRL 0** | Not started as a rated subsystem |

## 1.1 Snapshot of the repository today

- **Most mature:** core abstractions and the basic system/diagram composition model.
- **Architecture still under validation:** compile pipeline, leaf evaluators, diagram evaluators, and simulation.
- **Very early MVP work:** graphical backends, mechanics, symbolic mechanics, and physics.
- **Not yet rated as a mature library surface:** blocks, planning, and control.
- **Still not a stabilized package surface:** top-level package exports (`minilink/__init__.py` is still empty) and several higher-level convenience APIs.

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

Additional: `outputs(x, u, t)` / `outputs_p(...)` → `dict` of **boundary** output ports (leaf: port ids; diagram: ports added via `connect_new_output_port`, often empty).

**Leaf system backends** (non-diagram):
- [`NumpyLeafEvaluator`](minilink/compile/numpy_evaluator.py) — wraps `System.f`/`System.h` with frozen params, nominal u snapshot. Full 3-tier support.
- [`JaxLeafEvaluator`](minilink/compile/jax_evaluator.py) — `f`/`h`/`outputs`/`outputs_p` (and IVP tier) **JIT-compiled** and warm-started.

**Diagram backends** (compiled `DiagramSystem`):
- [`NumpyDiagramEvaluator`](minilink/compile/numpy_evaluator.py) — `f`; boundary `outputs` / `h` (single boundary port only); `compute_internal_signals` + `compute_internal_signals_dict` for the **subsystem** buffer. Parametric tier not implemented.
- [`JaxDiagramEvaluator`](minilink/compile/jax_evaluator.py) — same, with JIT for `f`, boundary `outputs`, and `compute_internal_signals`. **`get_f_jit()`**, **`get_outputs_jit()`**, **`get_internal_signals_jit()`**.

**Removed API:** `compute_outputs(..., ports=...)` — use `compute_internal_signals_dict` and index by `"sys_id:port_id"`, or slice the flat buffer.

**Entry point**: `compile(system, backend="numpy"|"jax")` in [`compiler.py`](minilink/compile/compiler.py). Dispatches to leaf or diagram evaluator.

### File organization (`compile/`)

```
compile/
  evaluator.py         — DynamicsEvaluator ABC
  numpy_evaluator.py   — NumpyLeafEvaluator + NumpyDiagramEvaluator
  jax_evaluator.py     — JaxLeafEvaluator + JaxDiagramEvaluator
  execution_plan.py      — ExecutionPlan, PortOperation, StateOperation
  compiler.py            — compile(), compile_diagram(), build_execution_plan()
```

### Key design decisions

- **Dict params as JAX pytrees** — no flat `p` vector needed. JAX handles `dict` natively for `jit`, `grad`, `vmap`. `ParameterLayout` pack/unpack deferred until needed for scipy/CasADi optimizers.
- **Separate methods per tier** (not optional args) — avoids JAX pytree-structure issues when `params` is sometimes `None` vs `dict`.
- **ABC over Protocol** — children inherit default implementations (integration, differentiation); only `f`/`h`/`outputs` and their `_p` variants are abstract.
- **IVP tier snapshots `_u_nominal` at compile time** — JIT-safe on all backends (no runtime port reads).
- **Diagram `h()`** — defined only with exactly one boundary output; otherwise use `outputs()` or `compute_internal_signals_dict()` for subsystem ports.

### TODO — next steps

- [ ] **Implement integration methods** in ABC: `rk4_step`/`euler_step`/`rollout` × 3 tiers. NumPy: Python loops. JAX: override with `lax.scan`.
- [ ] **Implement differentiation** (JAX): `jacobian_f_x`, `jacobian_f_u`, `jacobian_h_x`, `jacobian_h_u`, `linearize`. NumPy: finite differences later.
- [ ] **Diagram parametric tier** (`f_p`): requires adding `sys_id` to `PortOperation`/`StateOperation` so caller can supply per-subsystem params dict.
- [x] **Diagram boundary `outputs()` / `h()`** — wired via `ExecutionPlan.external_output_slices` and `connect_new_output_port` (2026).
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
- [x] **Port diagram evaluators to `DynamicsEvaluator`**: `NumpyDiagramEvaluator` + `JaxDiagramEvaluator` inherit ABC. `f()` + boundary `outputs()`; internal buffer methods split from boundary outputs; JAX JIT for `outputs` and `compute_internal_signals`. (2026-04-05, refined 2026-04)
- [x] **Reorganize `compile/` folder**: Fused leaf + diagram evaluators per backend (`numpy_evaluator.py`, `jax_evaluator.py`). (2026-04-05)
- [x] **Add backend-pluggable simulation package**: `minilink/simulation/` now provides `Simulator`, solver backends, and interpolation helpers. `System.compute_trajectory(...)` delegates to this path. (2026-04)
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

## 4. Lessons from Drake & Pycollimator

Cross-read of [Drake](https://github.com/RobotLocomotion/drake) (rigorous systems + contexts) and [pycollimator](https://github.com/kuzja111/pycollimator) (Python block diagrams, JAX-first). Full notes: [drake_analysis.md](drake_analysis.md), [pycollimator_analysis.md](pycollimator_analysis.md).

### 4.1 How the frameworks relate

| Idea | Drake | Pycollimator | Minilink today |
| :--- | :--- | :--- | :--- |
| **Diagram assembly** | `DiagramBuilder` → immutable `Diagram` | Same pattern (clone of Drake) | `DiagramSystem` + string `connect()`; `connect_new_output_port` ≈ export |
| **Runtime data** | `Context` per system (state, time, params, cache) | Frozen `Context` PyTrees | Flat `(x, u, t)` + dict `params`; evaluators are stateless w.r.t. model |
| **Evaluation** | Lazy `Eval*` + ticket invalidation | Lazy ports + dependency tickets | **Compile-once** `ExecutionPlan`; eager topological steps |
| **AD / types** | `AutoDiffXd`, `Expression` | JAX + `custom_vjp` on simulator | JAX via `JaxLeafEvaluator` / `JaxDiagramEvaluator` |

**Synthesis:** Drake is “rigor first”; pycollimator is “differentiable simulation first”; minilink is “math-readable equations + explicit compilation first.” All three agree that **equations should stay separate from runtime state** and that **diagram-level I/O** should be first-class (export / hierarchy).

### 4.2 What to keep (minilink strengths)

- **Explicit evaluator classes** (NumPy vs JAX at compile time) — avoid pycollimator-style **global backend switches** (`set_backend("jax")`): clearer and thread-safe.
- **Compiled topological IR** — simpler and often faster than full lazy evaluation + cache graphs at current scale; add **tickets / invalidation** only when discrete events, sample-and-hold, or very expensive blocks demand it.
- **Flat `f(x, u, t)` as the primary API** — keep for readability; optional frozen `SimState`-style bundles later if JAX `vmap` over structured state becomes a real requirement.

### 4.3 Consolidated borrow list (prioritized)

**Near-term (overlaps Phase 2–3)**

- **Diagram validation**: unique subsystem ids, port existence, no double-connection on inputs (builder-style guards without mandating a separate `DiagramBuilder` class).
- **`SimulationOptions`**: frozen dataclass for SciPy (and future JAX) solver knobs — replaces ad-hoc `**kwargs` on `solve_ivp`.
- **Default / disconnected inputs**: clarify semantics (Drake-style defaults) where ports are unconnected.

**Mid-term (overlaps Phase 4–5)**

- **Port exporting / nesting**: ensure inner `DiagramSystem` can present diagram-level ports to a parent (Drake `ExportInput` / `ExportOutput` pattern); align with `export_input` / `export_output` roadmap items.
- **`StateSpaceSystem`, `TransferFunction`, `PID`**: follow pycollimator-style LTI layout; map feedthrough to explicit output `dependencies` (e.g. from `D` matrix).
- **`linearize(system, x0, u0)`**: JAX JVP on compiled evaluator when differentiation API lands.
- **Parameter metadata**: distinguish dynamic vs static parameters for tracing (lighter than full `Parameter` framework at first).

**Long-term / research**

- **Optional `Context` / `SimState`**: bundle `x, u, t, params` for multi-instance sims, checkpointing, and less argument plumbing — conceptual Drake alignment without C++ machinery.
- **Caching**: `last_t` or `CacheEntry` on hot outputs before a full ticket graph.
- **Hybrid systems**: periodic discrete updates, zero-crossings, modes — design toward pure `f` / evaluator calls and avoid baking “only `solve_ivp`” into the core.
- **Symbolic / analytical linearization**: only if needed beyond JAX; Drake-style `Expression` trace is the reference.
- **Multibody “plant”**: URDF/SDF → dynamics blocks (Drake `MultibodyPlant` direction).

### 4.4 Anti-patterns (explicit non-goals)

- Global mutable math backend selection.
- Heavy metaclass magic for parameter resolution.
- Mandatory PyTree-wrapped state before a concrete differentiable-sim use case.
- Replacing compile-once plans with full lazy evaluation **unless** profiling shows the IR approach is the bottleneck.

---

## 5. Pyro 2.0 Migration Strategy

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
| `ContinuousDynamicSystem` | `DynamicSystem` | **TRL 7** |
| Compiled evaluation (leaf) | `NumpyLeafEvaluator` / `JaxLeafEvaluator` | **TRL 4** |
| Compiled evaluation (diagram) | `NumpyDiagramEvaluator` / `JaxDiagramEvaluator` | **TRL 4** |
| `StateSpaceSystem` | `StateSpaceSystem` | Planned |
| `MechanicalSystem` | `mechanics.MechanicalSystem` | **TRL 1** |
| Symbolic EoM + `MechanicalModel` | `mechanics.symbolic` | **TRL 1** |
| `Manipulator` | `Manipulator` | Planned |

---

## 6. Future Vision

- **JAX-Based Optimization**: Leverage autodiff for trajectory optimization (Direct Collocation). Dict params as JAX pytrees enable `jax.grad` over physical parameters directly.
- **Full Differentiable Simulation**: `jax.custom_vjp` for the integrator, `lax.scan` rollouts, `vmap` batch simulations.
- **Hybrid System Simulation**: Discrete state updates and zero-crossing events.
- **Differentiable LQR**: Differentiable Riccati solvers.
- **Gymnasium Bridge**: Native `sys2gym` conversion for RL experiments.
- **Interactive Game Loop**: Enhanced pygame-based real-time control.
- **Multibody Plant Framework**: URDF/SDFormat → rigid body dynamics blocks.
