# Minilink Technical Design & Standards

`minilink` is a block-diagram simulation framework built natively in Python. It heavily leverages object-oriented design to represent dynamical systems, mathematical blocks, and their interconnected topological execution graphs.

## 1. Core Philosophy

1.  **Mathematical Readability First**: The primary goal is for the source code to read as close as possible to handwritten mathematical operations (e.g., `dx = A@x + B@u`).
2.  **Readability Over Performance**: Prioritize pure readability in the core library. Optimization shifts (like the `compile` package) should remain isolated so core equations stay clean.
3.  **Expressiveness:** Define custom dynamic systems using standard Python.
4.  **Performance:** Compile topological block execution graphs into optimized array-evaluation sequences (`f_fast`).
5.  **Transparency:** Lightweight and transparent, avoiding heavy, black-box legacy dependencies.
6.  **Separation of Concerns:** Modeling core (signals, ports, systems, diagrams) is independent of simulation, visualization, and analysis.
7.  **Pyro Compatibility:** Full feature parity with the [pyro](https://github.com/SherbyRobotics/pyro) toolbox.

---

## 2. Directory Structure

| Module | Status | Description |
| :--- | :--- | :--- |
| `core/` | **1.0** | Pure modeling abstractions (framework, diagram, analysis). |
| `blocks/` | **1.0** | Pre-built reusable blocks (integrators, sources, pendulums). |
| `compile/` | **0.9** | Stateless IR and multi-backend (NumPy/JAX) evaluators. |
| `graphical/` | MVP | Visualization and animation renderers. |
| `control/` | Planned | Controller base classes and library. |
| `planning/` | Planned | Planners: RRT, direct collocation, DP. |
| `analysis/` | Planned | Extended analysis: cost functions, phase plots. |

---

## 3. Core Abstractions

### Signal and Port Model

-   **`VectorSignal`**: A named vector with dimension, labels, units, bounds, and a nominal value.
-   **`InputPort(VectorSignal)`**: An input channel. Defaults to returning the nominal value if not connected.
-   **`OutputPort(VectorSignal)`**: An output channel. Declares `dependencies` on input ports for algebraic-loop detection.

### System Hierarchy

-   **`System`**: Base class. Holds states, inputs, outputs, and solver hints. Defines `f(x,u,t)` and `h(x,u,t)`.
-   **User dynamics are not purity-checked:** Python cannot enforce that `f`, `h`, or port `compute` avoid mutating `self` or reading hidden mutable state. The intended contract is documented on `System`; `bind_params=True` only deep-copies the `params` dict into the compiled plan, not arbitrary instance fields. JAX tracing can surface some impure patterns but is not a full guarantee.
-   **`StaticSystem(System)`**: Pure feedthrough blocks (`n=0`).
-   **`DynamicSystem(System)`**: Systems with state (`n>0`).
-   **`DiagramSystem(System)`**: Composite system. Supports `compile()` for flattened execution plans.

---

## 4. Execution & Compilation Pipeline

1.  **`DiagramSystem.compile()`**: Runs DFS-based algebraic-loop detection and builds the execution plan.
2.  **`f_fast(x,u,t)`**: Iterates over a flat execution plan using array slices to avoid dictionary traversals.
3.  **`DiagramIR`**: (MVP) Snapshot of the execution plan for multi-backend execution (NumPy, JAX).

### 4.1 Parameters vs compilation (default, bound, explicit vector)

Evaluators call ``f(local_x, local_u, t, params)`` and port ``compute(local_x, local_u, t, params)``. The fourth argument is ``op.bound_params`` from the :class:`~minilink.compile.execution_plan.ExecutionPlan`: ``None`` means blocks use ``params or self.params`` (live ``subsystem.params``); if you change ``params`` after ``compile()`` without ``bind_params``, the next ``compute_dx`` reflects that without recompiling.

- **`compile_diagram(..., bind_params=True)`** (and ``DiagramSystem.compile(..., bind_params=True)``): each ``PortOperation`` / ``StateOperation`` stores a **deep copy** of that subsystem's ``params`` in ``bound_params`` and passes it on every evaluation. Mutating ``subsystem.params`` later does **not** change behavior until you compile again.

- **Explicit parameter vector** (separate entry point, e.g. ``compile_diagram_parameterized`` in ``minilink.compile.parameterized``): a **second pipeline** builds a flat vector ``p`` and a ``DiagramParameterLayout`` (subsystem id, key, slice, shape). The compiler produces a plan whose operations are ``(x, u, t, p) → …`` so **JAX can differentiate w.r.t. ``p``**. This path reuses ``build_execution_plan`` for topology and gather recipes, then **transforms** the plan (wrap callables); the base ``ExecutionPlan`` builder stays free of parameter-layout logic.

### 4.2 `DynamicsEvaluator` — Public compiled API

All compiled evaluators inherit from the `DynamicsEvaluator` ABC ([`minilink/compile/evaluator.py`](minilink/compile/evaluator.py)). It provides a **three-tier** callable API matching standard math notation:

| Tier | Dynamics | Output | What's fixed |
|------|----------|--------|--------------|
| **Standard** | `f(x, u, t)` | `h(x, u, t)` | params frozen at compile time |
| **Parametric** | `f_p(x, u, t, params)` | `h_p(x, u, t, params)` | nothing — caller supplies params dict |
| **IVP** | `f_ivp(x, t)` | `h_ivp(x, t)` | u + params both frozen |

Additional: `outputs(x, u, t)` / `outputs_p(...)` return a `dict` of all output ports.

**Leaf system backends** (single `System`, non-diagram):
- `NumpyLeafEvaluator` — wraps `System.f`/`System.h` with frozen params + nominal u snapshot.
- `JaxLeafEvaluator` — same, with core methods pre-JIT-compiled and warm-started at construction.

**Entry point**: `compile(system, backend="numpy"|"jax")` → returns a `DynamicsEvaluator`.

**Parameter handling**: params are Python `dict` at all tiers. JAX handles dicts natively as pytrees — no flat parameter vector needed. `jax.grad`, `jax.jacobian`, `vmap` all preserve dict structure.

**Scipy bridge**: `as_scipy_rhs()` → `(t, x) -> dx` using the IVP tier.

**Future**: Integration utilities (`rk4_step`, `rollout`), differentiation (`jacobian_f_x`, `linearize`), and batch simulation (`vmap_rollout`) are defined in the ABC as `NotImplementedError` stubs, to be implemented incrementally.

### 4.3 Diagram compilation pipeline

The diagram evaluators (`NumpyDiagramEvaluator`, `JaxDiagramEvaluator`) inherit from `DynamicsEvaluator`. Only `f(x, u, t)` maps to the ABC; `h`, `outputs`, and the parametric tier raise `NotImplementedError` (diagrams don't have a single primary output, and per-subsystem params dispatch requires `sys_id` on operations — deferred).

Diagram-specific methods are preserved: `compute_outputs(x, u, t, ports)` for port-filtered output selection, `compute_internal_signals_dict(x, u, t)` for all internal subsystem port signals as a dict, and `compute_internal_signals(x, u, t)` for the raw flat signal buffer.

`build_execution_plan` stays focused on topology, slices, and gather recipes. `bind_params=True` deep-copies subsystem params into each operation; `bind_params=False` passes `None` so blocks use live `self.params`.

### JAX Compilation Vision
Future performance scales via JAX (XLA) by breaking the Python GIL:
-   **Implicit Tracing**: Using PyTrees and avoiding in-place mutations.
-   **Native Duck-Typing**: Standard `numpy` calls are overridden by JAX.
-   **`f_jax` Fallback**: Optional handwritten functional overrides for complex logic.

---

## 5. Coding Conventions

Following these naming schemes ensures consistency and readability across the codebase.

### General Standards
- **Python Version**: **3.10+** (LTS stable). Use modern syntax like `|` for unions and structural pattern matching.
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

-   **`name`**: **Type of System.** Human-readable string for class/type.
    -   *Examples*: `"Pendulum"`, `"Controller"`, `"Diagram"`.
-   **`id` or `xxx_id`**: **Programmatic Identifier.** Keys in dictionaries within a diagram. Always prefix to avoid shadowing built-in `id()`.
    -   *Examples*: `sys_id="plant"`, `port_id="y"`.
-   **`label` / `labels`**: **Display String.** Formatted for plotting and terminal output.
    -   *Examples*: `"theta"`, `"torque"`. Always use the plural `labels` for lists.

### File & Module Standards
-   All core modeling files (under `core/`) must *only* import `numpy` at the module level.
-   Heavy dependencies (SciPy, Matplotlib, Graphviz) must be lazy-imported within methods.
-   Use Python type hints and docstrings for every public method.
