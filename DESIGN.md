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
-   **`StaticSystem(System)`**: Pure feedthrough blocks (`n=0`).
-   **`DynamicSystem(System)`**: Systems with state (`n>0`).
-   **`DiagramSystem(System)`**: Composite system. Supports `compile()` for flattened execution plans.

---

## 4. Execution & Compilation Pipeline

1.  **`DiagramSystem.compile()`**: Runs DFS-based algebraic-loop detection and builds the execution plan.
2.  **`f_fast(x,u,t)`**: Iterates over a flat execution plan using array slices to avoid dictionary traversals.
3.  **`DiagramIR`**: (MVP) Snapshot of the execution plan for multi-backend execution (NumPy, JAX).

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
