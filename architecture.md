# Minilink Architecture and Vision

`minilink` is a block-diagram simulation framework built natively in Python. It heavily leverages object-oriented design to represent dynamical systems, mathematical blocks, and their interconnected topological execution graphs.

Its long-term goal is to serve as a **Pyro 2.0**: a complete replacement for the [pyro](https://github.com/SherbyRobotics/pyro) robotics toolbox with a more flexible port-based foundation that supports arbitrary block-diagram composition, compiled execution graphs, and JAX-based autodiff/optimization — while preserving every feature Pyro users depend on (controllers, planners, cost functions, mechanical system templates, analysis tools, and interactive simulation).

## 1. Core Philosophy

1. **Expressiveness:** Making it easy for users to define custom dynamic systems using standard Python.
2. **Performance:** Compiling the topological block execution graph into a flattened, heavily optimized array-evaluation sequence (`f_fast`), enabling rapid ODE integration.
3. **Transparency:** Remaining lightweight and transparent, avoiding heavy, black-box legacy C++ dependencies unless explicitly requested by the user.
4. **Separation of concerns:** Keeping the modeling core (signals, ports, systems, diagrams) free from simulation, visualization, and analysis dependencies so that the core can be used in headless, JAX-only, or deployment contexts without pulling in matplotlib/graphviz/scipy.
5. **Pyro compatibility:** Every feature in Pyro should have a minilink equivalent that is at least as expressive and ideally more composable.

## 2. Directory Structure and Maturity

> **Maturity legend:**
> - **1.0** — Stable foundation. API is settled; build on top of it.
> - **MVP prototype** — Functional but expect API changes, missing edge cases, and incomplete test coverage.
> - **Planned** — Stub or empty; not yet implemented.

```
minilink/
├── core/                      # [1.0] Pure modeling abstractions
│   ├── framework.py           # [1.0] VectorSignal, InputPort, OutputPort, System, StaticSystem, DynamicSystem
│   ├── diagram.py             # [1.0] DiagramSystem: connections, algebraic-loop detection, compiled f_fast
│   ├── analysis.py            # [1.0] Trajectory, Simulator, compute_internal_signals
│   └── jax_utils.py           # [MVP prototype] JAX helper (get_f_jax)
├── blocks/                    # [1.0] Pre-built reusable blocks
│   ├── basic.py               # [1.0] Integrator, PropController
│   ├── sources.py             # [1.0] Source, Step, WhiteNoise
│   └── tests.py               # [1.0] Pendulum, PendulumPDController (demo blocks)
├── compile/                   # [MVP prototype] IR and backend-specific compiled artifacts
│   ├── ir.py                  # [MVP prototype] DiagramIR (frozen dataclasses: PortOp, StateOp)
│   ├── api.py                 # [MVP prototype] lower_diagram_to_ir, compile_numpy
│   └── numpy_backend.py       # [MVP prototype] CompiledNumpyDiagram
├── graphical/                 # [MVP prototype] Visualization
│   ├── primitives.py          # [MVP prototype] GraphicPrimitive, Point, Circle, Rectangle, CustomLine
│   ├── graphe.py              # [MVP prototype] Graphviz diagram rendering
│   ├── plotting.py            # [MVP prototype] Trajectory time-series plots
│   ├── animation.py           # [MVP prototype] Animator + game loop
│   └── renderers/             # [MVP prototype] AnimationRenderer backends (matplotlib, meshcat, pygame)
├── control/                   # [Planned] Controller base classes and library
├── planning/                  # [Planned] Planners: RRT, direct collocation, DP
└── analysis/                  # [Planned] Extended analysis: cost functions, phase plots
```

In practice, typical user workflows involve:

1. Defining or importing reusable blocks under `blocks/` (e.g., `Integrator`, `Pendulum`, controllers, and signal sources).
2. Assembling them into a `DiagramSystem` under `core/diagram.py` by wiring input/output ports.
3. Simulating the resulting closed-loop system via `Simulator` in `core/analysis.py`.
4. Plotting and/or animating trajectories via `graphical/` utilities.

## 3. Core Abstractions

### Signal and Port Model

- **`VectorSignal`** — A named vector with dimension, labels, units, bounds, and a nominal value. The base data descriptor for all signals flowing through the diagram.
- **`InputPort(VectorSignal)`** — An input channel on a system. Has a `get_signal(t)` callback (defaults to returning the nominal value).
- **`OutputPort(VectorSignal)`** — An output channel on a system. Has a `compute(x, u, t, param)` callable and a `dependencies` annotation declaring which input ports it feeds through from (used for algebraic-loop detection).

### System Hierarchy

- **`System`** — Base class. Holds `n` states, `m` inputs, `p` outputs, dictionaries of `InputPort`/`OutputPort`, state properties, solver hints. Defines `f(x,u,t)` (state derivative), `h(x,u,t)` (output), and `fsim(t,x)` (standalone simulation RHS). Also provides the kinematic geometry protocol (`get_kinematic_geometry`, `get_kinematic_transforms`, `get_dynamic_geometry`) for animation.
- **`StaticSystem(System)`** — `n=0`. Pure feedthrough blocks (gains, controllers, routing).
- **`DynamicSystem(System)`** — `n>0`. By default, output `"y"` has no feedthrough (`dependencies=()`), and an extra output port `"x"` is added that exposes the state vector directly.
- **`DiagramSystem(System)`** — Composite system. Holds subsystems and connections. Supports `compile()` which runs algebraic-loop detection (DFS) and builds flattened execution plans. Provides `f(x,u,t)` (recursive, slow) and `f_fast(x,u,t)` (compiled, fast). Also has MVP prototype JAX paths (`compile_jax`, `f_fast_jax`).

### Compilation Pipeline

> The in-class `compile()` / `f_fast` on `DiagramSystem` is **1.0** (stable, the primary simulation path). The external `compile/` package (`DiagramIR`, `CompiledNumpyDiagram`) and all JAX paths (`compile_jax`, `f_fast_jax`, `jax_utils`) are **MVP prototypes** — functional for benchmarking and experimentation but not yet hardened for production use.

1. **`DiagramSystem.compile()`** — Runs `check_algebraic_loops()` (DFS with feedthrough-aware cycle detection) then `build_execution_plan()` (maps every port computation and state derivative to flat array slices and source-type tuples).
2. **`DiagramIR`** (in `compile/ir.py`) — *(MVP prototype)* Frozen dataclass snapshot of the execution plan. Immutable, safe to pass to backend evaluators.
3. **`CompiledNumpyDiagram`** — *(MVP prototype)* Stateless evaluator that allocates a fresh `global_signals` buffer per call (no shared mutable state).
4. **JAX path** — *(MVP prototype)* `compile_jax` / `f_fast_jax` use `jnp.zeros(...).at[...].set(...)` for JAX-traceable functional evaluation.

### Simulation

- **`Trajectory`** — Container for `x` (n × T), `u` (m × T), `t` (T,) arrays, plus optional per-port internal signal recordings.
- **`Simulator`** — Selects solver (scipy/euler/discrete), time vector, and integrates. Wraps `scipy.integrate.solve_ivp` for the default continuous-time path.

## 4. Current Architecture Strengths

Compared to Pyro (the predecessor), minilink's **1.0 core** provides:

1. **Named multi-port MIMO connections** — Pyro has a single flat `u`/`y` per system. Minilink's `InputPort`/`OutputPort` dictionaries allow blocks to expose distinct named channels (e.g., `"ref"`, `"y"`, `"feedforward"`), enabling clean MIMO wiring.
2. **True block-diagram composition** — Pyro can only compose `controller + plant` into a `ClosedLoopSystem`. Minilink's `DiagramSystem` can wire N arbitrary systems together in arbitrary topologies.
3. **Topological compilation with algebraic-loop detection** — DFS-based feedthrough analysis with per-output `dependencies` annotation breaks false loops in MIMO systems.
4. **Compiled execution plans** — `f_fast` avoids all string lookups and dict traversal in the hot simulation loop.

Additionally, the **MVP prototypes** lay groundwork for:

5. **IR layer for multi-backend support** — *(MVP prototype)* `DiagramIR` separates "what to compute" from "how", enabling numpy, JAX, and future backends.
6. **JAX-compatible functional path** — *(MVP prototype)* `f_fast_jax` using immutable array operations for traceability.

## 5. Known Flaws and Technical Debt

### Critical Bugs

- **SciPy solver passes empty `u` to diagrams:** The `solver == "scipy"` branch in `Simulator.solve()` calls `sys.f_fast(x, np.array([]), t)`, ignoring external diagram inputs. Euler/discrete correctly use `get_u_from_input_ports(t)`. Any diagram with `m > 0` will produce wrong results under the default solver.
- **Trajectory shape documentation mismatch:** The `Trajectory` docstring says `(time-steps, n)` but actual layout is `(n, time-steps)`. Additionally, `traj.n` and `traj.m` shadow the system dimension convention (they mean state/input dimension, not time-step count, but the docstring suggests otherwise).

### Architectural Debt

- ~~**Core imports simulation + graphics:**~~ **Fixed.** `framework.py` and `diagram.py` now only import `numpy` at module level. All simulation/graphics dependencies (`Simulator`, `Animator`, `graphe`, `primitives`) are lazy-imported inside the convenience method bodies. Importing core classes no longer pulls in scipy, matplotlib, or graphviz.
- **Mutable shared `global_signals` buffer:** `DiagramSystem.f_fast` writes to `self.global_signals`, making concurrent calls unsafe (breaks reentrancy for parallel scenarios or adaptive ODE steppers). `CompiledNumpyDiagram` already fixes this with per-call allocation.
- **Gather/dispatch code duplication:** The `src_type == 0/1/2` signal-gathering loop appears 6+ times across `f_fast`, `f_fast_jax`, `compile_jax`, `CompiledNumpyDiagram.eval_dx`, `eval_outputs`, and `compute_internal_signals`. A shared helper function should consolidate this.
- **No public API surface:** All `__init__.py` files are empty. Users must know internal module paths.
- **Parameters don't flow through compiled paths:** `System.f()` accepts `params` but the compiled execution plan doesn't thread them. This will block parametric optimization.

### Minor Issues

- Typo `gloabl` in `diagram.py` ~line 176; `Comuting` in debug print.
- French error strings in `jax_utils.py` vs English elsewhere.
- Module named `graphe` (French) vs conventional English.
- `DummySystem` defined twice in `graphe.py`.
- `tempfile.mktemp` usage in `plot_graphviz` (deprecated, race-prone).
- Accidental artifact PDFs at repo root (`Diagram.gv.pdf`, `aaa.pdf`).

## 6. External Dependencies

`minilink` intentionally avoids databases, web servers, or cloud integrations. All computation is in-memory and local.

**Core (required):**
- **NumPy** for array math and vectorized operations.

**Simulation (required for `Simulator`):**
- **SciPy** (`solve_ivp`) for ODE integration.

**Visualization (optional — should be lazy-imported):**
- **matplotlib** for plotting and animation.
- **Graphviz** for block-diagram rendering.
- **meshcat** / **pygame** for alternative animation backends.

**Acceleration (optional):**
- **JAX** / **jaxlib** for JIT compilation, autodiff, and GPU execution.

## 7. Future Vision: JAX Compilation via Native Duck-Typing

> **Status: MVP prototype.** The current JAX paths (`compile_jax`, `f_fast_jax`, `jax_utils`) are functional proofs of concept. The architecture described below is the long-term target, not yet fully realized.

While the current `f_fast` NumPy evaluation is optimized for the Python Interpreter, future performance scale (and Autodiff for direct collocation optimization) requires breaking the Python GIL and leveraging GPU/TPU acceleration via JAX (XLA).

Instead of a heavy "Dual-Backend Injection" forcing users to import an array namespace `xp`, `minilink` embraces **Native Duck-Typing with a Graceful Fallback**:

### The Hybrid JAX Architecture
1. **Implicit Tracing via PyTrees:**
   `DiagramSystem.f_fast` will be refactored to remove all in-place mutable array assignments (`global_signals[slice] = val`). It will instead gather signals functionally (e.g., in lists) and use a final `concatenate`. This allows `jax.jit` and `jax.jacfwd` to trace the entire graphical execution automatically.
2. **Duck-Typing for Base Users:**
   Beginners and standard users continue to import pure `numpy` and write equations normally (avoiding in-place mutations like `x[0] = 5`, preferring `np.array([val1, val2])`). JAX will seamlessly override methods like `np.sin` through Python's `__array_function__` protocol without the user modifying their imports.
3. **Optional `f_jax` Overrides (The Fallback):**
   If a block's logic is fundamentally incompatible with JAX tracing (e.g., heavy `if/else` procedural control flow or non-JAX compiled dependencies), the user can supply a dedicated purely functional `f_jax(self, x, u, t)` method. The diagram's compilation step will automatically select this method if executed within a JAX optimization context.

## 8. Pyro 2.0 Migration Strategy

See `pyro_migration.md` for a detailed per-feature migration plan mapping every Pyro module/class to its minilink equivalent.
