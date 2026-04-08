# Minilink Technical Design & Standards

`minilink` is a block-diagram simulation framework built natively in Python. It heavily leverages object-oriented design to represent dynamical systems, mathematical blocks, and their interconnected topological execution graphs.

## 1. Core Philosophy

1. **Mathematical Readability First**: The primary goal is for the source code to read as close as possible to handwritten mathematical operations (e.g., `dx = A@x + B@u`).
2. **Readability Over Performance**: Prioritize pure readability in the core library. Optimization shifts (like the `compile` package) should remain isolated so core equations stay clean.
3. **Expressiveness:** Define custom dynamic systems using standard Python.
4. **Performance:** Compile topological block execution graphs into optimized array-evaluation sequences (`f_fast`).
5. **Transparency:** Lightweight and transparent, avoiding heavy, black-box legacy dependencies.
6. **Separation of Concerns:** Modeling core (signals, ports, systems, diagrams) is independent of simulation, visualization, and analysis.
7. **Pyro Compatibility:** Full feature parity with the [pyro](https://github.com/SherbyRobotics/pyro) toolbox.

---

## 2. Directory Structure


| Module       | Status  | Description                                                                                                                                                                  |
| ------------ | ------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `core/`      | **1.0** | Pure modeling abstractions (framework, diagram, analysis).                                                                                                                   |
| `blocks/`    | **1.0** | Pre-built reusable blocks (integrators, sources, pendulums).                                                                                                                 |
| `compile/`   | **1.0** | `ExecutionPlan` IR; `DynamicsEvaluator` (leaf + diagram); NumPy/JAX; boundary vs internal outputs; JIT on JAX leaf and diagram (`f`, `outputs`, `compute_internal_signals`). |
| `graphical/` | MVP     | Animation orchestration; Matplotlib, MeshCat, Pygame backends; kinematic hooks on systems.                                                                                   |
| `physics/`   | MVP     | JAX contact world (`PhysicsWorldSystem`) + demos; optional dependency on JAX.                                                                                                |
| `mechanics/` | MVP     | Symbolic multibody + export to `MechanicalSystem` / `JaxMechanicalSystem`.                                                                                                   |
| `control/`   | Planned | Controller base classes and library.                                                                                                                                         |
| `planning/`  | Planned | Planners: RRT, direct collocation, DP.                                                                                                                                       |
| `analysis/`  | Partial | `Simulator` and trajectory utilities; extended cost/phase tooling planned.                                                                                                   |


---

## 3. Core Abstractions

### Signal and Port Model

- `**VectorSignal`**: A named vector with dimension, labels, units, bounds, and a nominal value.
- `**InputPort(VectorSignal)**`: An input channel. Defaults to returning the nominal value if not connected.
- `**OutputPort(VectorSignal)**`: An output channel. Declares `dependencies` on input ports for algebraic-loop detection.

### System Hierarchy

- `**System**`: Base class. Holds states, inputs, outputs, and solver hints. Defines `f(x,u,t)` and `h(x,u,t)`.
- **User dynamics are not purity-checked:** Python cannot enforce that `f`, `h`, or port `compute` avoid mutating `self` or reading hidden mutable state. The intended contract is documented on `System`; `bind_params=True` only deep-copies the `params` dict into the compiled plan, not arbitrary instance fields. JAX tracing can surface some impure patterns but is not a full guarantee.
- `**StaticSystem(System)**`: Pure feedthrough blocks (`n=0`).
- `**DynamicSystem(System)**`: Systems with state (`n>0`).
- `**DiagramSystem(System)**`: Composite system. Supports `compile()` for flattened execution plans.

---

## 4. Execution & Compilation Pipeline

1. `**DiagramSystem.compile()**` / `**compile_diagram()**`: Algebraic-loop detection (DFS) and construction of an `**ExecutionPlan**`.
2. **Compiled evaluators**: `evaluator.f(x,u,t)` applies the flat plan (stateless).
3. `**ExecutionPlan**`: Immutable schedule (`port_ops`, `state_ops`, `output_slices`, `**external_output_slices**`) consumed by NumPy or JAX backends.

### 4.1 Parameters vs compilation (default, bound, explicit vector)

Evaluators call `f(local_x, local_u, t, params)` and port `compute(local_x, local_u, t, params)`. The fourth argument is `op.bound_params` from the :class:`~minilink.compile.execution_plan.ExecutionPlan`: `None` means blocks use `params or self.params` (live `subsystem.params`); if you change `params` after `compile()` without `bind_params`, the next `compute_dx` reflects that without recompiling.

- `**compile_diagram(..., bind_params=True)**` (and `DiagramSystem.compile(..., bind_params=True)`): each `PortOperation` / `StateOperation` stores a **deep copy** of that subsystem's `params` in `bound_params` and passes it on every evaluation. Mutating `subsystem.params` later does **not** change behavior until you compile again.
- **Explicit parameter vector** (planned): a separate entry point (e.g. `compile_diagram_parameterized` in a future `minilink.compile.parameterized` module) would build a flat vector `p` and a `DiagramParameterLayout` (subsystem id, key, slice, shape). The compiler would produce a plan whose operations are `(x, u, t, p) → …` so **JAX can differentiate w.r.t. `p**`, reusing `build_execution_plan` for topology and gather recipes, then transforming the plan; the base `ExecutionPlan` builder would stay free of parameter-layout logic. Today, dict `params` as JAX pytrees cover most autodiff use cases via the parametric tier on leaf evaluators.

### 4.2 `DynamicsEvaluator` — Public compiled API

All compiled evaluators inherit from the `DynamicsEvaluator` ABC (`[minilink/compile/evaluator.py](minilink/compile/evaluator.py)`). It provides a **three-tier** callable API matching standard math notation:


| Tier           | Dynamics               | Output                 | What's fixed                          |
| -------------- | ---------------------- | ---------------------- | ------------------------------------- |
| **Standard**   | `f(x, u, t)`           | `h(x, u, t)`           | params frozen at compile time         |
| **Parametric** | `f_p(x, u, t, params)` | `h_p(x, u, t, params)` | nothing — caller supplies params dict |
| **IVP**        | `f_ivp(x, t)`          | `h_ivp(x, t)`          | u + params both frozen                |


Additional: `outputs(x, u, t)` / `outputs_p(...)` return a `dict` of **boundary** output ports. Keys are port ids for **leaf** systems (e.g. `"y"`). For **diagram** evaluators, keys match the diagram’s external output ports only (`DiagramSystem.outputs`); the full internal signal map is `compute_internal_signals_dict` (all subsystem ports).

**Leaf system backends** (single `System`, non-diagram):

- `NumpyLeafEvaluator` — wraps `System.f`/`System.h` with frozen params + nominal u snapshot.
- `JaxLeafEvaluator` — same, with `f`/`h`/`outputs`/`outputs_p` pre-JIT-compiled and warm-started at construction.

**Entry point**: `compile(system, backend="numpy"|"jax")` → returns a `DynamicsEvaluator`.

**Parameter handling**: params are Python `dict` at all tiers. JAX handles dicts natively as pytrees — no flat parameter vector needed. `jax.grad`, `jax.jacobian`, `vmap` all preserve dict structure.

**Scipy bridge**: `as_scipy_rhs()` → `(t, x) -> dx` using the IVP tier.

**Future**: Integration utilities (`rk4_step`, `rollout`), differentiation (`jacobian_f_x`, `linearize`), and batch simulation (`vmap_rollout`) are defined in the ABC as `NotImplementedError` stubs, to be implemented incrementally.

### 4.3 Diagram compilation pipeline

The diagram evaluators (`NumpyDiagramEvaluator`, `JaxDiagramEvaluator`) implement `f(x, u, t)` and `outputs(x, u, t)` / `outputs_p(...)` for **diagram boundary** ports only (same contract as leaf; often an empty dict when no `connect_new_output_port` was used). `h(x, u, t)` is defined only when there is **exactly one** such external output. The parametric tier is not implemented for diagrams yet (per-subsystem `params` dispatch requires `sys_id` on operations — deferred).

**Diagram-only accessors (subsystem internal buffer)**  
These are **not** the same as `outputs()`:

- `**compute_internal_signals(x, u, t)**` — flat vector of length `plan.signal_dim` (all subsystem output ports laid out in compiler order).
- `**compute_internal_signals_dict(x, u, t)**` — `dict["sys_id:port_id"] → array` for every subsystem output slice in `output_slices`.

There is **no** separate `compute_outputs(..., ports=...)` API; pick ports by key from `compute_internal_signals_dict` or slice the flat buffer using `plan.output_slices`.

**JAX diagram evaluator** (`JaxDiagramEvaluator`) JIT-compiles `**f**`, boundary `**outputs**`, and `**compute_internal_signals**` independently (each warm-started at construction). Convenience: `**get_f_jit()**`, `**get_outputs_jit()**`, `**get_internal_signals_jit()**`.

`**ExecutionPlan.external_output_slices**` maps **diagram boundary** output port ids (keys of `DiagramSystem.outputs`) to slices into the **same** internal signal buffer as the connected source port — used solely to implement `**outputs()**` on the diagram evaluator.

`build_execution_plan` stays focused on topology, slices, and gather recipes. `bind_params=True` deep-copies subsystem params into each operation; `bind_params=False` passes `None` so blocks use live `self.params`.

### JAX Compilation Vision

Future performance scales via JAX (XLA) by breaking the Python GIL:

- **Implicit Tracing**: Using PyTrees and avoiding in-place mutations.
- **Native Duck-Typing**: Standard `numpy` calls are overridden by JAX.
- **`f_jax` Fallback**: Optional handwritten functional overrides for complex logic.

---

## 5. Coding Conventions

Following these naming schemes ensures consistency and readability across the codebase.

### General Standards

- **Python Version**: **3.10+** (LTS stable). Keep **`agent.md`** §2 in sync when changing the minimum version; update **`pyproject.toml`** `requires-python` accordingly.
- **Type Hinting**: **Uniform & Mandatory**. All functions and methods must have clear type hints.
- **Docstrings**: **NumPy Style**. Required for all public classes and methods.

### The "Math Rule" (Naming Patterns)

The top priority is that the code reads like math equations (e.g., `dx = A@x + B@u`). 

- **Matrices**: Use Uppercase single letters (`A`, `B`, `H`, `M`, `K`).
- **Vectors**: Use lowercase single letters (`x`, `u`, `y`, `q`, `v`, `dq`).
- **Dimensions**: Use `n`, `m`, `p` for dimension.
  - Matrix `A` has dimension `(n, n)`.
  - Vector `x` has dimension `(n, 1)`.
  - Vector `u` has dimension `(m, 1)`.
  - Vector `y` has dimension `(p, 1)`.
  - Matrix `B` has dimension `(n, m)`.

### Programmatic Variable Naming (Non-Math Context)

- `**name**`: **Type of System.** Human-readable string for class/type.
  - *Examples*: `"Pendulum"`, `"Controller"`, `"Diagram"`.
-  `**xxx_id**`: **Programmatic Identifier.** Keys in dictionaries within a diagram. Always prefix to avoid shadowing built-in `id()`.
  - *Examples*: `sys_id="plant"`, `port_id="y"`.
- `**label` / `labels`**: **Display String.** Formatted for plotting and terminal output.
  - *Examples*: `"theta"`, `"torque"`. Always use the plural `labels` for lists.

### File & Module Standards

- All core modeling files (under `core/`) must *only* import `numpy` at the module level.
- Heavy dependencies (SciPy, Matplotlib, Graphviz) must be lazy-imported within methods.
- Use Python type hints and docstrings for every public method.

