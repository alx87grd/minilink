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

## 3. Current Architecture & Known Flaws

### Evaluation Mechanism (`f_fast`)
To simulate a diagram, `diagram.compile()` traces all connections, verifies the absence of illegal algebraic loops, and builds a tuple-based `execution_plan`. The `f_fast(x, u, t)` method then iterates over this plan, slicing global NumPy arrays and avoiding string/dictionary lookups entirely.

### Known Design Flaws
- **Stochastic Logging:** Stochastic inputs are re-evaluated linearly post-integration rather than logging the exact values used by the ODE steps.
- **Side-Effects in Pure Functions:** Post-simulation signal reconstruction mutates a `global_signals` buffer on the Diagram object itself, breaking encapsulation and parallel safety.
- **Verbose API Setup:** Diagram string-based connection calls are repetitive and lack IDE autocompletion (e.g., `diagram.connect("sys", "y", "ctrl", "ref")`).

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
