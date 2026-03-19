# Minilink Architecture and Vision

`minilink` is a block-diagram simulation framework built natively in Python. It heavily leverages object-oriented design to represent dynamical systems, mathematical blocks, and their interconnected topological execution graphs.

## 1. Core Philosophy
The primary goal of `minilink` is to provide an intuitive, flexible environment for modeling closed-loop control systems, signal processing pipelines, and physical simulations. It prioritizes:
1. **Expressiveness:** Making it easy for users to define custom dynamic systems using standard Python.
2. **Performance:** Compiling the topological block execution graph into a flattened, heavily optimized array-evaluation sequence (`f_fast`), enabling rapid ODE integration.
3. **Transparency:** Remaining lightweight and transparent, avoiding heavy, black-box legacy C++ dependencies unless explicitly requested by the user.

## 2. Directory Structure
- **`core/`**: The heart of the framework.
  - `framework.py`: Defines the base `System` and port primitives (`InputPort`, `OutputPort`, `VectorSignal`).
  - `diagram.py`: Orchestrates block connections, checks for algebraic loops, and compiles the highly optimized `f_fast` execution plans.
  - `analysis.py`: Contains the `Simulator` handling trajectory recording and wrapping `scipy.integrate.solve_ivp`.
- **`blocks/`**: Pre-built static and dynamic systems such as controllers, linear filters, and signal sources (`basic.py`, `sources.py`).
- **`graphical/`**: Utilities for plotting simulation trajectories using `matplotlib` and visualizing topological block diagrams using `graphviz`.

In practice, typical user workflows involve:

1. Defining or importing reusable blocks under `blocks/` (e.g., `Integrator`, `Pendulum`, controllers, and signal sources).
2. Assembling them into a `DiagramSystem` under `core/diagram.py` by wiring input/output ports.
3. Simulating the resulting closed-loop system via `Simulator` in `core/analysis.py`.
4. Plotting and/or animating trajectories via `graphical/` utilities.

## 3. Current Architecture & Known Flaws

### Evaluation Mechanism (`f_fast`)
To simulate a diagram, `diagram.compile()` traces all connections, verifies the absence of illegal algebraic loops, and builds a tuple-based `execution_plan`. The `f_fast(x, u, t)` method then iterates over this plan, slicing global NumPy arrays and avoiding string/dictionary lookups entirely.

### Known Design Flaws
- **Stochastic Logging:** Stochastic inputs are re-evaluated linearly post-integration rather than logging the exact values used by the ODE steps.
- **Side-Effects in Pure Functions:** Post-simulation internal-signal reconstruction mutates a `global_signals` buffer on the `DiagramSystem` itself, breaking encapsulation and limiting parallel safety (especially for multi-threaded or multi-scenario evaluation).
- **Verbose API Setup:** Diagram string-based connection calls are repetitive and lack IDE autocompletion (e.g., `diagram.connect("sys", "y", "ctrl", "ref")`).
- **Matplotlib Backend Fragility:** The plotting layer currently hard-codes a small set of preferred backends (e.g., `Qt5Agg`, `MacOSX`), which can be brittle in headless or non-standard environments and may need to be revisited for broader deployment scenarios.

### External Dependencies and Integrations

`minilink` intentionally avoids databases, web servers, or cloud integrations. All computation is in-memory and local. The main external dependencies are:

- **NumPy** for array math and vectorized operations throughout the core and blocks.
- **SciPy** (`solve_ivp`) for ODE integration in the primary continuous-time simulation path.
- **matplotlib** for plotting and animation of trajectories and system geometry.
- **Graphviz** for rendering diagrams as graphs.

This keeps the runtime surface area small and focused on scientific computing, while leaving room for users to integrate their own I/O, persistence, or orchestration layers as needed.

### Tests and Example Scenarios

The `tests/` tree contains:

- **Unit tests** that validate the core abstractions (`VectorSignal`, `System`, `StaticSystem`, `DynamicSystem`), basic blocks (`Source`, `Step`), and advanced plotting/diagram compilation flows.
- **Manual/demo tests** that assemble small closed-loop diagrams, run simulations, and exercise plotting/graphical utilities.

These tests double as executable examples of typical usage patterns (e.g., step → controller → plant diagrams) and are a good starting point for understanding how to compose systems idiomatically.

## 4. Future Vision: JAX Compilation via Native Duck-Typing

While the current `f_fast` NumPy evaluation is optimized for the Python Interpreter, future performance scale (and Autodiff for direct collocation optimization) requires breaking the Python GIL and leveraging GPU/TPU acceleration via JAX (XLA).

Instead of a heavy "Dual-Backend Injection" forcing users to import an array namespace `xp`, `minilink` embraces **Native Duck-Typing with a Graceful Fallback**:

### The Hybrid JAX Architecture
1. **Implicit Tracing via PyTrees:**
   `DiagramSystem.f_fast` will be refactored to remove all in-place mutable array assignments (`global_signals[slice] = val`). It will instead gather signals functionally (e.g., in lists) and use a final `concatenate`. This allows `jax.jit` and `jax.jacfwd` to trace the entire graphical execution automatically.
2. **Duck-Typing for Base Users:**
   Beginners and standard users continue to import pure `numpy` and write equations normally (avoiding in-place mutations like `x[0] = 5`, preferring `np.array([val1, val2])`). JAX will seamlessly override methods like `np.sin` through Python's `__array_function__` protocol without the user modifying their imports.
3. **Optional `f_jax` Overrides (The Fallback):**
   If a block's logic is fundamentally incompatible with JAX tracing (e.g., heavy `if/else` procedural control flow or non-JAX compiled dependencies), the user can supply a dedicated purely functional `f_jax(self, x, u, t)` method. The diagram's compilation step will automatically select this method if executed within a JAX optimization context.
