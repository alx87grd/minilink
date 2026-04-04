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

### 4.2 Thin callable API (NumPy / JAX evaluators)

For SciPy ``solve_ivp`` the signature is ``f(t, x)``, while the diagram uses ``dx(x, u, t)``. Evaluators expose:

- **`as_dx_callable()`** → ``(x, u, t) -> dx`` (thin wrapper around ``compute_dx``).
- **`as_scipy_ivp_fun(u=None)`** → ``(t, x) -> dx`` with a fixed or empty external input vector ``u``.

Parameterized evaluators add **`as_dx_callable(p)`** (fixed ``p``, returns ``(x, u, t) -> dx``) and **`get_jit_compute_dx()`** on the JAX parameterized class for ``jit`` over ``(x, u, t, p)``.

### 4.3 Detailed design: thin API, `bind_params`, and explicit-`p` pipeline

**Why a separate module for explicit `p` (recommended yes).**  
`build_execution_plan` should stay focused on topology, slices, and gather recipes. Parameter layout (ordering, flattening shapes, unpacking slices into per-subsystem dicts) is a **second concern**. A dedicated module (e.g. `minilink/compile/parameterized.py`) should:

1. Call `build_execution_plan(diagram)` and `check_algebraic_loops(diagram)` unchanged.
2. Build `DiagramParameterLayout` by scanning subsystems in **deterministic order** (e.g. sorted `sys_id`, sorted param keys); each entry records `sys_id`, `key`, `slice` into `p`, and original `shape` for `reshape` after read.
3. Run **`inject_parameter_vector_into_plan`**: clone `PortOperation` / `StateOperation` rows with wrappers `lambda x,u,t,p: base(x,u,t, unpack(p, sys_id))` so existing blocks keep their dict-based `f` / `compute` signatures while the evaluator thread passes one array `p`.

The base `NumpyEvaluator` / `JaxEvaluator` loops stay on the **3-argument** call path; **parameterized** evaluators duplicate the small interpretation loop (or share private helpers) with a **4-argument** call path. That avoids optional `p` branches on every line of the default hot path.

**Thin API (concrete behavior).**

| Method | Returns | Notes |
| :--- | :--- | :--- |
| `as_dx_callable()` | `Callable[[x,u,t], dx]` | `functools.partial`-style thin wrapper; no closure over mutable diagram state beyond what `compute_dx` already closes. |
| `as_scipy_ivp_fun(u=None)` | `Callable[[t,x], dx]` | Default `u` to `np.zeros(m)` or `np.array([])` to match current diagram external-input convention; document `m` vs empty. |

**`bind_params=True` (concrete behavior).**

`build_execution_plan(..., bind_params=True)` sets ``bound_params`` on each ``PortOperation`` / ``StateOperation`` to ``copy.deepcopy`` of that subsystem's ``params`` (one copy per operation row). Evaluators pass ``op.bound_params`` as the fourth argument to ``f`` / ``compute``; no closure wrapping.

**Explicit `p` (concrete behavior).**

- `pack_from_diagram(diagram)` fills `p` from current `params` for regression tests and initial guesses in optimization.
- `compile_diagram_parameterized(diagram, backend="numpy"|"jax")` returns a **parameterized evaluator** plus layout on the object (`evaluator.layout`) so callers can `jit(evaluator.compute_dx)` and use `jax.grad` w.r.t. `p` when dynamics are traceable.
- Non-numeric `params` entries are out of scope until a schema extension defines them.

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
