# Minilink Technical Design & Standards

`minilink` is a Python-native block-diagram framework for writing dynamical systems in a math-readable way, composing them through ports, compiling them into flat evaluators, and simulating or visualizing the result.

## 1. Design Principles

1. **Math readability first**: code should look close to textbook notation such as `dx = A@x + B@u`.
2. **Readable core, optimized edges**: keep equations simple in `core/`; isolate performance work in `compile/` and `simulation/`.
3. **Transparent architecture**: prefer explicit objects and data flow over heavy hidden machinery.
4. **Separation of concerns**: modeling, compilation, simulation, and visualization are distinct layers.
5. **Pyro successor direction**: `minilink` is the long-term port-based foundation for future Pyro-style workflows.

## 2. Package Map

| Module | Status | Description |
| --- | --- | --- |
| `core/` | **TRL 7** | Main modeling abstractions plus the canonical `Trajectory` in `trajectory.py` |
| `compile/` | **TRL 4** | `ExecutionPlan`, `DynamicsEvaluator`, NumPy/JAX evaluator backends |
| `simulation/` | **TRL 4** | `Simulator`, solver backends, interpolation helpers |
| `graphical/` | **TRL 1** | Plotting, animation, renderers, and provisional visualization hooks |
| `mechanics/` | **TRL 1** | Numeric and symbolic mechanics paths |
| `physics/` | **TRL 1** | JAX contact-world MVP and demos |
| `blocks/` | **TRL 0** | Early reusable blocks, not yet a stabilized library layer |
| `benchmark/` | **TRL 1** | Optional timing helpers and no core dependency |
| `planning/` | **TRL 0** | Future planners |
| `control/` | **TRL 0** | Future controllers |

## 3. Core Contracts

### 3.1 Signals, ports, and diagrams

- `VectorSignal`: named vector metadata with dimension, labels, units, bounds, and nominal value
- `InputPort(VectorSignal)`: input channel; falls back to nominal value when unconnected
- `OutputPort(VectorSignal)`: output channel; may declare dependencies for algebraic-loop detection
- `DiagramSystem`: composite system built by wiring subsystem ports

### 3.2 `System` contract

`System` intentionally combines four concerns:

1. **Core dynamical contract**
   - `f(x, u, t, params)` and `h(x, u, t, params)`
   - dimensions `n`, `m`, `p`
   - ports, labels, bounds, and state description
2. **Model defaults and metadata**
   - `params`
   - `x0`
   - `solver_info`
   - nominal input values stored on ports
3. **Visualization / forward-kinematic contract**
   - `get_kinematic_geometry()`
   - `get_kinematic_transforms(x, u, t)`
   - `get_dynamic_geometry(x, u, t)`
   - This API is still MVP / TRL 1.
4. **User shortcut facade**
   - `compile()`
   - `compute_trajectory()`
   - `compute_forced()`
   - `render()`, `animate()`, `game()`
   - graph / HTML display helpers

`System` may also cache the last computed trajectory for convenience. That cache is not part of the mathematical model.

### 3.3 Functional intent

- `f`, `h`, and output-port `compute` callables are intended to behave as functions of `(x, u, t, params)`.
- The Python object still stores defaults and convenience state, but dynamics code should avoid depending on unrelated mutable instance fields.
- `self.params` is the default model parameter set; explicit `params=...` arguments override it.

### 3.4 Trajectories

`Trajectory` lives in `minilink.core.trajectory` and is the official sampled **state-input trajectory** object:

- core fields: `t`, `x`, `u`
- array convention: sampled signals use shape `(dim, N)`
- optional extra sampled channels live in `signals`
- helper methods such as interpolation and resampling stay generic to sampled signals

Simulation, planning, tracking control, and animation should all share this object where a state-input trajectory is the right abstraction.

## 4. Compile and Simulation Architecture

### 4.1 Main workflow layers

The intended flow is:

```text
System / DiagramSystem
    -> compile()
    -> DynamicsEvaluator
    -> Simulator
    -> Trajectory
    -> plotting / animation / internal-signal reconstruction
```

`System.compute_trajectory(...)` is the high-level convenience path and delegates to `minilink.simulation.Simulator`.

### 4.2 `ExecutionPlan` and evaluators

- `compile(system, backend="numpy"|"jax")` returns a `DynamicsEvaluator`
- `DiagramSystem.compile()` builds an `ExecutionPlan`
- the plan stores flat operations and slices for subsystem signals and external outputs
- NumPy and JAX backends consume the same plan

The `DynamicsEvaluator` public callable tiers are:

| Tier | Dynamics | Output | What is fixed |
| --- | --- | --- | --- |
| Standard | `f(x, u, t)` | `h(x, u, t)` | params frozen at compile time |
| Parametric | `f_p(x, u, t, params)` | `h_p(x, u, t, params)` | caller supplies params |
| IVP | `f_ivp(x, t)` | `h_ivp(x, t)` | `u` and params frozen |

`outputs(...)` returns **boundary** outputs only. For diagram internals, use the diagram-specific internal-signal API instead.

### 4.3 Diagram-specific signals

Diagram evaluators and diagrams distinguish:

- **boundary outputs**: the external outputs of the diagram
- **internal signals**: subsystem outputs inside the flattened signal buffer

The official trajectory-level postprocess lives on `DiagramSystem`:

- `diagram.reconstruct_internal_signals(traj)`
- `diagram.compute_internal_signals(traj)` as a compatibility alias

There is no `compute_outputs(..., ports=...)` API.

### 4.4 Parameters and compilation

- with `bind_params=False`, compiled operations read live `self.params`
- with `bind_params=True`, subsystem params are deep-copied into the plan
- dict params remain the main parameter format and work naturally with JAX pytrees
- a flat parameter-vector tier is deferred until there is a concrete optimizer-driven need

### 4.5 Simulation policy

- Unconnected input ports contribute a **constant default value** only.
- Time-varying signals do **not** live in input-port defaults.
- If a time signal is part of the reusable **model**, represent it with a source block in the diagram.
- If forcing is part of a **simulation run**, pass sampled `u(t)` data to the simulator.
- Solver backends may interpolate `u(t)` internally, but the public simulation input remains grid-based.

Planned ergonomic shortcut:

- `System.compute_forced(...)` now provides that high-level shortcut for sampled inputs or simple `u(t)` callables
- richer forcing helpers can still grow on top of the same simulator-level path later

Current solver modes:

- `solver="euler"`
- `solver="scipy"`
- `solver="scipy_stiff"`
- `solver="scipy_max"`
- `solver="scipy_ultra"`

### 4.6 Benchmark package (`minilink.benchmark`)

Optional helpers for timing and regression-style comparisons. They are **not** part of the core dynamical or simulation contract; scripts may import them for local experiments.

| Module | Purpose |
| --- | --- |
| `minilink.benchmark.f_speed` | `benchmark_f_speeds` measures native `system.f`, then `compile("numpy").f` and `compile("jax").f` over a fixed `(x, u, t)`; optional ASCII table via `print_f_speed_table`. Native timing is skipped on `RecursionError` so deep recursive diagrams can still benchmark compiled paths. |
| `minilink.benchmark.simulation_speed` | Timing helpers (`run_timed`, `summarize_durations`, `relative_l2_error`); `benchmark_sim_backend(system, …)` compares one candidate `(solver, compile_backend)` to a **truth** pair (defaults `TRUTH_SOLVER` / `TRUTH_BACKEND`, i.e. `scipy_ultra` + `numpy`) on a given **built** `system`; `benchmark_sim_speed_matrix(system, pairs=…, …)` sweeps an explicit ordered list of `(solver, backend)` pairs vs the same truth pair, colors rows by ``rel_err_l2 < accuracy_threshold_pct``, prints ``speed_vs_truth = truth_mean_time / cell_mean_time``; ``DEFAULT_SWEEP_PAIRS`` is the full product of ``DEFAULT_SOLVERS`` × ``DEFAULT_BACKENDS``. Result rows and printed tables label the JAX compile key as ``jax(cpu)`` / ``jax(gpu)`` / etc. via ``jax.default_backend()`` (`format_benchmark_backend_label`). The same module also owns the fixed benchmark scenarios (`STANDARD_SIM_CASES`) plus `run_standard_sim_suite` / `print_standard_sim_suite`. |

Programmatic imports should use `from minilink.benchmark import ...` (re-exports include `STANDARD_SIM_CASES`, `run_standard_sim_suite`, `DEFAULT_SWEEP_PAIRS`, `MatrixResult`, `MatrixRow`). Runnable examples are flat scripts under `tests/benchmark/` (see `agent.md` for manual script style). Optional helper ``tests/benchmark/tune_scipy_vs_rk4.py`` runs ten ``solve_ivp`` search rounds against an RK4+JAX wall-time bar.

### 4.7 Animation playback (`System.animate` / `Animator.animate_simulation`)

`sys.animate(...)` takes three **orthogonal** keyword arguments that are resolved
independently:

| kwarg | selects | values |
| --- | --- | --- |
| `renderer` | graphics tech | `"matplotlib"`, `"meshcat"`, `"pygame"` |
| `html` | output channel | `True` = inline notebook object, `False` = local window, `None` = auto (see env policy below) |
| `native` | playback engine | `True` = backend's own animation API (**default**), `False` = legacy per-frame Python loop |

Behavior:

- `native=True` (default) drives each backend's own animation engine (see
  below). This is the recommended path for new code: smoother playback,
  less overhead, clean JSHTML export in notebooks.
- `native=False` falls back to the legacy per-frame Python loop
  (`draw_frame` + `present`). Kept unchanged byte-for-byte for debugging
  and for the meshcat dynamic-geometry case (see limitation at the bottom
  of this section).
- `native=True` + matplotlib drives `matplotlib.animation.FuncAnimation`. On
  the window path (`is_blocking_needed()` True) it calls `plt.show(block=True)`
  and cleans up after the window closes; otherwise (IPython REPL, Jupyter with
  an interactive backend) it calls `plt.show(block=False)` and intentionally
  retains the figure + `FuncAnimation` on the renderer so the event loop drives
  playback without garbage collection killing the animation.
- `native=True` + meshcat builds a `meshcat.animation.Animation` and calls
  `Visualizer.set_animation(...)`; the browser plays keyframes with no Python
  loop.
- `html=True` returns an `IPython.display.HTML` object: matplotlib uses
  `FuncAnimation.to_jshtml()`; meshcat uses `Visualizer.render_static(...)`
  around `static_html()` (ideal for Colab cells).

#### Env policy (`html=None` resolution and blocking)

Env detection lives in a single module, `minilink/graphical/environment.py`,
and exposes `detect_env()`, `prefers_inline_animation()`, `is_inline_capable()`,
`is_blocking_needed()`, and `override_env()`. All env-sensitive sites
(`Animator.animate_simulation`, `System.animate`, `plot_trajectory`,
`plot_signals`, `MatplotlibRenderer.present` / `play_native`,
`MeshcatRenderer.present`) read from it.

The matrix below summarizes defaults. Minilink **never** mutates matplotlib
settings (no `matplotlib.use(...)`, no `%matplotlib ...` magic); it only
reads `matplotlib.get_backend()` to decide between window and inline HTML
in Jupyter.

| Env | `detect_env()` | matplotlib backend | `html=None` resolves to | `is_blocking_needed()` | Default matplotlib `native=True` playback |
| --- | --- | --- | --- | --- | --- |
| Bare Python script | `script` | any GUI (`macosx` / `qt*` / `tkagg`) | False | True | Window, `plt.show(block=True)`, cleanup |
| IPython terminal REPL | `ipython` | any GUI | False | False | Window, `plt.show(block=False)`, refs retained |
| Local Jupyter, `%matplotlib inline` (default) | `jupyter` | `inline` / `agg` | **True** (auto-fallback) | False | Inline `HTML(ani.to_jshtml())` |
| Local Jupyter, `%matplotlib widget` (`ipympl`) | `jupyter` | `module://ipympl...` | False | False | Inline widget; `plt.show(block=False)` |
| Local Jupyter, `%matplotlib qt` / `macosx` / `tk` | `jupyter` | `qt*` / `macosx` / `tkagg` | False | False | Pop-up window, `plt.show(block=False)` |
| Colab | `colab` | `inline` (forced) | True | False | Inline `HTML(ani.to_jshtml())` |

Notes:

- Explicit `html=True` / `html=False` is always honored; only `html=None`
  consults the env.
- Jupyter users who want fast native-animation playback can opt into
  `%matplotlib widget` (requires `ipympl`) or `%matplotlib qt` themselves.
  Neither is a minilink dependency; the default Jupyter `inline` configuration
  is fully supported via the HTML fallback.
- Static figures (`plot_trajectory`, `plot_signals`) use
  `plt.show(block=is_blocking_needed())`: they block only in script mode;
  Jupyter / Colab render them inline via the active backend.
- `override_env("script" | "ipython" | "jupyter" | "colab" | None)` is the
  testing / CI escape hatch; pass `None` to reset the cache.

Limitation of meshcat native playback: `meshcat.animation.Animation` only
keyframes rigid pose (position+quaternion) per path. Primitives whose geometry
is rebuilt each frame (currently only `TorqueArrow`) are frozen at `t=0` in
the native path and a one-line notice is printed when any are present. Pass
`native=False` (the legacy Python-loop path) for frame-accurate playback of
those primitives.

## 5. Coding Standards

### General rules

- **Python**: 3.10+
- **Type hints**: required on public APIs
- **Docstrings**: NumPy style on public classes and methods
- **Imports**: keep heavy dependencies lazy outside the headless core

### The math rule

The default naming style should read like equations:

- matrices: `A`, `B`, `H`, `M`, `K`
- vectors: `x`, `u`, `y`, `q`, `v`, `dq`
- dimensions: `n`, `m`, `p`

For non-math context:

- use `sys_id`, `port_id`, `block_id` for programmatic identifiers
- use `label` / `labels` for display strings
- keep the code explicit and boring when that improves readability
