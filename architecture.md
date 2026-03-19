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

## 4. Future Vision: JAX Compilation & Dual-Backend

While the current `f_fast` NumPy evaluation is highly optimized for standard Python iteration, the future of computational simulation requires breaking free from the Python Interpreter lock (GIL) and taking advantage of GPU/TPU acceleration.

To achieve this, `minilink` is transitioning toward a **JAX (XLA)** compiled architecture through a **Dual Backend Injection Strategy**.

### The Dual-Backend Goal
The framework will not force JAX on beginners. By default, standard fast NumPy mutation handles simulations. However, power users can opt-in to XLA JIT compilation to gain 10-100x performance scaling.

### Implementation Requirements
1. **The Array Namespace (`xp`):**
   Core blocks and `System` primitives will stop importing `numpy` directly. Instead, they will accept an injected array namespace `xp` (defaulting to `numpy`). When simulating with JAX, `jax.numpy` is injected into the evaluation trace.
2. **Functional Execution Maps:**
   The `DiagramSystem.compile(backend="jax")` flag will unroll operations functionally. Instead of `global_signals[slice] = val`, it will use JAX-safe immutable updates like `global_signals.at[slice].set(val)`.
3. **Pure Math Blocks:**
   Blocks will be refactored to construct arrays purely functionally (e.g., `xp.array([x[1], -g*xp.sin(x)])`), dropping in-place allocations (`np.zeros()`). Control flow based on runtime signals will rely on `xp.where()` rather than standard Python `if/else` statements.
