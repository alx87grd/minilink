# Minilink Technical Design & Standards

`minilink` is a Python-native block-diagram framework for writing dynamical systems in a math-readable way, composing them through ports, compiling them into flat evaluators, and simulating or visualizing the result.

## 1. Design Principles

1. **Math readability first**: code should look close to textbook notation such as `dx = A@x + B@u`.
2. **Readable core, optimized edges**: keep equations simple in `core/`; isolate performance work in `compile/` and `simulation/`.
3. **Transparent architecture**: prefer explicit objects and data flow over heavy hidden machinery.
4. **Separation of concerns**: modeling, compilation, simulation, and visualization are distinct layers.
5. **Pyro successor direction**: `minilink` is the long-term port-based foundation for future Pyro-style workflows.

## 2. Package Map

The table below is the on-disk layout and TRL. **Structural rules**, **pluggable file naming**, and the **reference tree** for new modules are in §2.1–2.6.

| Module | Status | Description |
| --- | --- | --- |
| `core/` | **TRL 7** | Main modeling abstractions; `core/blocks/` diagram primitives (sources, integrators, lightweight signal blocks—not full plants); canonical `Trajectory` in `trajectory.py` |
| `compile/` | **TRL 4** | `ExecutionPlan`, `DynamicsEvaluator`, NumPy/JAX evaluator backends |
| `simulation/` | **TRL 4** | `Simulator`, solver backends, compile-backend selection (`"auto"` → try JAX, fall back to NumPy), optional auto fixed-step RK4 for long uniform grids with JAX, interpolation helpers |
| `graphical/` | **TRL 2** | Matplotlib theming (`matplotlib_style`), env-aware layout, animation renderers; kinematic hooks still provisional (see §3.2, §4.7–4.8) |
| `symbolic/` | **TRL 1** | Optional SymPy stack under `symbolic/mechanics/`; must not be required to import `minilink.dynamics` |
| `dynamics/abstraction/` | **TRL 1** | Inheritance abstractions (e.g. `MechanicalSystem` in `mechanical.py`) |
| `physics/` | **TRL 1** | JAX contact-world MVP (`engine_jax`, `PhysicsWorldSystem`); engine-backed dynamics, not folded into analytic `dynamics/` |
| `dynamics/catalog/` | **TRL 0** | Reusable **plant** models as `DynamicSystem` leaves (`pendulum/`, `vehicles/`, `msd/`, …) |
| `compile/evaluator_timing` | **TRL 1** | Optional compiled-`f` timing (`benchmark_f_speeds`, `print_f_speed_table`); not part of the core contract |
| `simulation/integration_timing` | **TRL 1** | Optional simulator timing sweeps, `STANDARD_SIM_CASES`, and helpers (`run_timed`, …) |
| `simulation/scenarios/` | **TRL 1** | Shared **stress** scenarios for timing matrices, not user plants |
| `planning/` | **TRL 1** | Deterministic planning architecture MVP: pure `PlanningProblem`; costs and sets live in `core/`; solver-family packages for search, trajectory optimization, and policy synthesis |
| `optimization/` | **TRL 1** | Thin finite-dimensional mathematical-program contracts shared by planning and future control workflows |
| `control/` | **TRL 0** | Controller and static law blocks (e.g. PD), separate from `dynamics/` plants |

### 2.1 Structural principles (on-disk)

1. **Dependency direction** — Generic types (`Trajectory`, costs, sets, ports) live in `core/` so other packages do not import `planning/` just to score or constrain trajectories.
2. **Language vs solvers** — *What* the system is → `core/`, `dynamics/`, `physics/`. *How* you integrate, compile, or optimize → `simulation/`, `optimization/`, `compile/`, `planning/`.
3. **Pluggable backends** — Use role-named subpackages and contract modules; naming details in §2.2.
4. **Inheritance ladders** (`MechanicalSystem`, …) — `dynamics/abstraction/` only; this is a different pattern from swappable solvers or renderers.
5. **Optional heavy dependencies** — SymPy (`symbolic/`), renderers (`graphical/`), future engines—prefer `pyproject` extras where practical.

### 2.2 Pluggable roles and file naming

When a package supports **multiple swappable mechanisms** (optimizers, solvers, evaluators, renderers):

1. **Role-specific folders** — Prefer `optimizers/`, `solvers/`, `renderers/`, `evaluators/`, and planning family packages (`search/`, …) over a generic `backends/` name. A type that orchestrates all families in a package may sit at the package root (e.g. `planning/planner.py`).
2. **Singular contract module** — The abstract API lives in a module named for the role, not `base.py`—e.g. `optimizers/optimizer.py`, `planning/planner.py`, `graphical/renderers/renderer.py`, `compile/evaluators/evaluator.py`.
3. **One file per concrete implementation** — e.g. `optimizers/scipy_minimize.py`; `renderers/matplotlib_renderer.py` (use `*_renderer.py` for third-party library clarity, not a bare `matplotlib.py`).
4. **Long, explicit names under `compile/evaluators/`** — `numpy_evaluator.py`, `jax_evaluator.py` (not `numpy.py` / `jax.py`) so imports do not clash with libraries.
5. **No barrel re-exports** — Import from the module that defines each symbol; package `__init__.py` files are namespace markers (docstring-only) unless a future API freeze says otherwise. See `agent.md`.
6. **Domain types at package root** — e.g. `mathematical_program.py` for `MathematicalProgram`; it is not an “optimizer” module.
7. **Compile orchestration** — `compiler.py` is the only orchestrator (plan, dispatch, return evaluator). Do **not** add `compile/compilers/`; the name collides. Per-backend construction lives in `compile/evaluators/*_evaluator.py` and small helpers next to `compiler.py`.

**Layout template:**

```text
minilink/<package>/
  <core_types>.py
  <role>/
    __init__.py              # docstring only
    <role_singular>.py       # ABC / protocol
    <impl>.py                # one primary mechanism per file
```

**Pluggable area map**

| Area | On-disk |
| --- | --- |
| `minilink/compile` | `compiler.py`, `execution_plan.py`, `jax_utils.py` (JAX + `format_benchmark_backend_label`), `evaluators/*`, `evaluator_timing.py` |
| `minilink/simulation` | `simulator.py`, `solvers/`, `integration_timing.py`, `scenarios/` |
| `minilink/graphical` | `renderers/renderer.py` + `*_renderer.py` |
| Timing and stress | No top-level `benchmark/` package; use `compile/evaluator_timing`, `simulation/integration_timing`, `simulation/scenarios` |

`CostFunction` and set types live in `minilink.core` (`core/costs.py`, `core/sets.py`).

**Reference snippets** (implemented layout):

```text
minilink/optimization/
  mathematical_program.py
  optimizers/
    __init__.py
    optimizer.py
    scipy_minimize.py

minilink/planning/
  planner.py
  problems.py
  search/
  trajectory_optimization/
  policy_synthesis/
```

```text
minilink/compile/
  compiler.py
  execution_plan.py
  jax_utils.py
  evaluator_timing.py
  evaluators/
    evaluator.py
    numpy_evaluator.py
    jax_evaluator.py

minilink/simulation/
  solvers/
    solver.py
    scipy_ivp.py
    euler.py
    rk4_fixed.py
  simulator.py
  input_interpolation.py
  integration_timing.py
  scenarios/

minilink/graphical/renderers/
  __init__.py
  renderer.py
  timing.py
  matplotlib_renderer.py
  meshcat_renderer.py
  pygame_renderer.py
```

| Concern | Subpackage | Contract module |
| --- | --- | --- |
| Finite-dimensional programs | `optimizers/` | `optimizer.py` |
| Planning across families | `planning/` (root) | `planner.py` |
| ODE / IVP / fixed step | `solvers/` | `solver.py` |
| Compiled dynamics | `evaluators/` | `evaluator.py` |
| Draw / animate | `renderers/` | `renderer.py` |
| Compile orchestration | `compile/` (root) | `compiler.py` (not `compilers/`) |

### 2.3 Canonical `minilink/` tree (reference)

`compiler.py` is the **orchestrator** (build or accept an `ExecutionPlan`, dispatch, return an evaluator). There is **no** `compile/compilers/`. `jax_utils.py` sits under `compile/` (JAX and evaluator-adjacent helpers); `tools/` is for **external** bridges, not that sugar.

**Naming:** Long, explicit evaluator modules under `evaluators/`, e.g. `numpy_evaluator.py`, `jax_evaluator.py` (not bare `numpy.py` / `jax.py`).

```text
minilink/
├── __init__.py
│
├── core/                           # modeling — no ODE/IVP or compile evaluators here
│   ├── __init__.py
│   ├── system.py
│   ├── diagram.py
│   ├── trajectory.py
│   ├── costs.py
│   ├── sets.py
│   └── blocks/
│       ├── __init__.py
│       ├── basic.py
│       └── sources.py
│
├── symbolic/
│   ├── __init__.py
│   └── mechanics/
│       └── ...
│
├── dynamics/
│   ├── __init__.py
│   ├── abstraction/
│   │   ├── __init__.py
│   │   ├── mechanical.py
│   │   └── ...
│   └── catalog/
│       ├── __init__.py
│       ├── pendulum/
│       ├── vehicles/
│       ├── msd/
│       └── equations/
│
├── physics/
│   ├── __init__.py
│   ├── engine_jax.py
│   └── system.py
│
├── compile/
│   ├── __init__.py
│   ├── compiler.py
│   ├── execution_plan.py
│   ├── jax_utils.py
│   ├── evaluator_timing.py
│   └── evaluators/
│       ├── __init__.py
│       ├── evaluator.py
│       ├── numpy_evaluator.py
│       └── jax_evaluator.py
│
├── simulation/
│   ├── __init__.py
│   ├── simulator.py
│   ├── input_interpolation.py
│   ├── integration_timing.py
│   ├── scenarios/
│   │   ├── __init__.py
│   │   └── ...
│   └── solvers/
│       ├── __init__.py
│       ├── solver.py
│       ├── scipy_ivp.py
│       ├── euler.py
│       └── rk4_fixed.py
│
├── estimation/
│   └── __init__.py
│
├── control/
│   ├── __init__.py
│   └── ...
│
├── planning/
│   ├── __init__.py
│   ├── planner.py
│   ├── problems.py
│   ├── search/
│   ├── trajectory_optimization/
│   └── policy_synthesis/
│
├── optimization/
│   ├── __init__.py
│   ├── mathematical_program.py
│   └── optimizers/
│       ├── __init__.py
│       ├── optimizer.py
│       └── scipy_minimize.py
│
├── analysis/
│   └── __init__.py
│
├── graphical/
│   ├── __init__.py
│   ├── plotting.py
│   ├── animation.py
│   ├── environment.py
│   ├── primitives.py
│   ├── matplotlib_style.py
│   ├── graphe.py
│   └── renderers/
│       ├── __init__.py
│       ├── renderer.py
│       ├── timing.py
│       ├── matplotlib_renderer.py
│       ├── meshcat_renderer.py
│       └── pygame_renderer.py
│
└── tools/                          # external only: Gym, URDF, MuJoCo, …
    └── __init__.py
```

### 2.4 Package roles (quick reference)

| Package | Role |
| --- | --- |
| `core/` | Diagram algebra, ports, `System` hierarchy, `Trajectory`, `costs.py`, `sets.py`, `blocks/`. |
| `symbolic/` | CAS; must not be required to `import minilink.dynamics`. |
| `dynamics/` | `abstraction/` + `catalog/`. |
| `physics/` | Engine-backed worlds; peer of `dynamics/`. |
| `compile/` | `compiler.py`, `execution_plan.py`, `evaluators/`, `jax_utils.py`, `evaluator_timing.py`. |
| `simulation/` | `simulator.py`, `solvers/`, `input_interpolation.py`, `integration_timing.py`, `scenarios/`. |
| `estimation/` | Observers, filters, sensor models. |
| `control/` | Feedback laws, RL policy wrappers as `StaticSystem`. |
| `planning/` | `PlanningProblem`, `planner.py`, families under `search/`, `trajectory_optimization/`, `policy_synthesis/`, … |
| `optimization/` | `MathematicalProgram`, `optimizers/`. |
| `analysis/` | Diagnostics and numerics (arrays, structured results), not “how to plot.” |
| `graphical/` | Plotting, animation, `renderers/renderer.py` and backends. |
| `tools/` | External ecosystem bridges; not internal JAX compile helpers. |

### 2.5 Scope, vocabulary, and boundaries

- **Typical dependency flow** — `core` ← (`dynamics`, `physics`, `estimation`, `control`, `planning`, …); `symbolic` → `dynamics`; `compile` consumes `core`; `simulation` consumes `compile` + `core`; `analysis` consumes trajectories or linearizations; `graphical` consumes results from anywhere.
- **`compile/` vs `simulation/`** — *Compile* answers: given a model and a diagram schedule, what **closed-form callable** is `f` / boundary outputs? *Simulation* answers: given an evaluator, how do we **advance time** to a `Trajectory`? `compiler.py` dispatches to `evaluators/*_evaluator.py`. `simulator.py` uses `solvers/`.
- **Benchmarks** — Optional timing lives in `compile/evaluator_timing` and `simulation/integration_timing`; stress graphs in `simulation/scenarios/`. Table labels for JAX use `format_benchmark_backend_label` in `compile/jax_utils.py`.
- **`analysis/` vs `graphical/`** — Reusable numerics and diagnostics → `analysis/`. Producing figures and animations → `graphical/`. Routines that are *only* plotting belong in `graphical/`. Multi-plot and diagram-export backends are deferred until a second implementation is real; keep `plotting.py` and `graphe.py` as the single stack for now.
- **`estimation/` vs `planning/`** — State reconstruction and filters live in `estimation/`; avoid duplicating that under `planning/`. Planning **uses** `core` costs and sets.
- **`control/` vs `planning/` (RL angle)** — Training or synthesis in `planning/` (or a dedicated family later); a **deployed** policy as a block → `control/`.
- **Core and “solvers”** — “No solvers in `core`” means no IVP stack and no compile *evaluators*—not “no NumPy.” `costs` and `sets` may use NumPy.
- **Placeholders** — `estimation/`, `analysis/`, and `tools/` grow as first modules land. Repository-level `examples/` and `tests/` sit outside `minilink/` and are not shown in the tree.

### 2.6 Pyro alignment (conceptual)

- Pyro `dynamic/*` → `core/blocks` + `dynamics/abstraction` + `dynamics/catalog`
- Pyro `analysis/*` → `analysis/` + `graphical/` + `simulation/` + `core/costs` / `core/sets`
- Pyro `planning/*`, `control/*`, `tools/*` → same-named packages here

### 2.7 Notes (dynamics + mechanics)

- **`dynamics/` Pyro ports**: `minilink.dynamics.catalog.pendulum.cartpole.CartPole` and
  `minilink.dynamics.catalog.pendulum.double_pendulum.DoublePendulum` are numeric ports of
  SherbyRobotics/pyro `CartPole` / `DoublePendulum` manipulator equations.
- **Kinematics vs dynamics**: animation is allowed to add display-only parameters
  (for example cart footprint sizes) and small out-of-plane offsets for volumetric
  renderers, without changing the ODE.
- **`MechanicalSystem` matrix hooks**: `H`, `C`, `B`, `g`, and `d` accept an
  optional trailing `params` argument, consistent with `f(x, u, t, params)`.

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

`MechanicalSystem` follows the same `params` story for its split dynamics
implementation: subclasses implement `H(q, params=None)`, `C(q, dq, params=None)`,
`B(q, params=None)`, `g(q, params=None)`, and `d(q, dq, params=None)`.

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

### 3.5 Deterministic planning

`minilink.planning` is the architecture MVP for deterministic planning problems.
It separates the continuous mathematical problem from numerical solver choices:

- `PlanningProblem` owns the system, boundary sets, allowable sets, optional
  cost, and optional parameter bundle.
- `CostFunction` uses the textbook notation `g(x, u, t)` for running cost and
  `h(x, t)` for terminal cost.
- Set objects model `x(t) in X(t)`, `u(t) in U(x, t)`, `x(0) in X0`, and
  `x(tf) in Xf` through `contains(...)` and nonnegative `margin(...)`.
- Planner implementations are grouped by family under subpackages: `search/`
  for feasibility/path search, `trajectory_optimization/` for optimal trajectory
  generation, and `policy_synthesis/` for feedback-policy computation—parallel
  in spirit to ``optimizers/`` under :mod:`minilink.optimization`, but with
  family-specific concrete planners rather than a single swapped interface.
- All families share one non-generic :class:`~minilink.planning.planner.Planner`
  orchestration base. The object returned by :meth:`~minilink.planning.planner.Planner.compute_solution`
  is defined by each concrete planner; there is no shared planning-result
  envelope type yet. Helpers such as :meth:`~minilink.planning.planner.Planner.plot_solution`
  assume the stored result is a :class:`~minilink.core.trajectory.Trajectory`.
- Trajectory-optimization transcriptions may emit generic
  `minilink.optimization` mathematical programs of the form
  `minimize J(z)` subject to `h(z) = 0`, `g(z) >= 0`, and bounds on `z`.

This first pass is intentionally deterministic and high-level. Stochastic
planning, chance constraints, belief states, and full direct-collocation/RRT/DP
internals are deferred until the architecture is reviewed.

### 3.6 Pluggable implementations layout

How `minilink` arranges abstract contracts and concrete implementations
(``optimization/optimizers``, ``planning/planner.py`` and family packages,
simulation solvers, renderers, and compile evaluators) follows **§2.1–2.2** (structural
principles and pluggable file naming) and the **reference tree in §2.3**. New
work in `planning/` and `optimization/` should match those rules; other packages
should converge when touched (see [ROADMAP.md](ROADMAP.md)).

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

Current solver modes (see `minilink.simulation.simulator` and tests):

- `solver="euler"`, `solver="rk4_fixedsteps"` (fixed step; not available for `solve_forced` today)
- `solver="scipy"`, `solver="scipy_stiff"`, `solver="scipy_max"`, `solver="scipy_ultra"`
- `solver="scipy_lsoda"` (alias that selects LSODA; useful for stiff/variable-tolerance cases)
- **Auto RK4 (heuristic)**: for long, **uniform** time grids, JAX compile backend, and a non-stiff profile, the simulator may select `rk4_fixedsteps` automatically for wall-clock reasons. This is best-effort; defaults remain explicit where conservatism matters.

`Simulator(..., compile_backend=...)` accepts `"numpy"`, `"jax"`, or **`"auto"`** (`minilink.simulation.COMPILE_BACKEND_AUTO`): try JAX, fall back to NumPy on failure. User-facing `System.compute_trajectory` / `compute_forced` default to **`"numpy"`** (predictable, no extra JIT) unless overridden.

**SciPy preset tolerances** (`scipy`, `scipy_stiff`, …) pass explicit `rtol` / `atol` into `solve_ivp` so behavior is not left to global SciPy defaults.

### 4.6 Benchmark package (`minilink.compile.evaluator_timing` / `minilink.simulation.integration_timing`)

Optional helpers for timing and regression-style comparisons. They are **not** part of the core dynamical or simulation contract; scripts may import them for local experiments. Stress builders live under `minilink.simulation.scenarios` (imported from timing scripts when needed).

| Module | Purpose |
| --- | --- |
| `minilink.compile.evaluator_timing` | `benchmark_f_speeds` measures native `system.f`, then `compile("numpy").f` and `compile("jax").f` over a fixed `(x, u, t)`; optional ASCII table via `print_f_speed_table`. Native timing is skipped on `RecursionError` so deep recursive diagrams can still benchmark compiled paths. |
| `minilink.simulation.integration_timing` | Timing helpers (`run_timed`, `summarize_durations`, `relative_l2_error`); `benchmark_sim_backend(system, …)` compares one candidate `(solver, compile_backend)` to a **truth** pair (defaults `TRUTH_SOLVER` / `TRUTH_BACKEND`, i.e. `scipy_ultra` + `numpy`) on a given **built** `system`; `benchmark_sim_speed_matrix(system, pairs=…, …)` sweeps an explicit ordered list of `(solver, backend)` pairs vs the same truth pair, colors rows by ``rel_err_l2 < accuracy_threshold_pct``, prints ``speed_vs_truth = truth_mean_time / cell_mean_time``; ``DEFAULT_SWEEP_PAIRS`` is the full product of ``DEFAULT_SOLVERS`` × ``DEFAULT_BACKENDS``. Result rows and printed tables label the JAX compile key as ``jax(cpu)`` / ``jax(gpu)`` / etc. via ``jax.default_backend()`` (`format_benchmark_backend_label`). The same module also owns the fixed benchmark scenarios (`STANDARD_SIM_CASES`) plus `run_standard_sim_suite` / `print_standard_sim_suite`. |

Programmatic imports should use the defining modules, e.g. `from minilink.simulation.integration_timing import run_standard_sim_suite, DEFAULT_SWEEP_PAIRS, ...` and `from minilink.compile.evaluator_timing import ...` as needed. Runnable examples are flat scripts under `tests/benchmark/` (see `agent.md` for manual script style). Optional helper ``tests/benchmark/tune_scipy_vs_rk4.py`` runs ten ``solve_ivp`` search rounds against an RK4+JAX wall-time bar.

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
`is_blocking_needed()`, `allow_tall_stacked_figures()`, and `override_env()`. All env-sensitive sites
(`Animator.animate_simulation`, `System.animate`, `plot_trajectory`,
`plot_signals`, `MatplotlibRenderer.present` / `play_native`,
`MeshcatRenderer.present`) read from it where applicable.

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

#### Roadmap: interactive integration and live I/O

Real-time paths (`Animator.game`, `Animator.run_interactive`, `System.game`) today combine
a **fixed Euler** step (with substeps in `game`), **pygame** for live `u`, and the chosen
**renderer** for geometry. Planned evolution (see `ROADMAP.md` §7, P2, Phase 4):

- **Integrator backends** — base class + multiple schemes (same *idea* as multiple solvers
  on `Simulator`), so the animator does not hard-code one stepping rule.
- **Live input backends** — base class + options (keyboard vs **TCP/UDP** or similar for
  cosimulation, etc.); keep demos thin and hide protocols in `graphical/` or a small helper
  module.
- **Live output push** — optional later (stream state/outputs to a peer); no concrete API yet.

### 4.8 Matplotlib look, figure sizing, and `plot_trajectory` modes

Central policy lives in `minilink/graphical/matplotlib_style.py`:

- **Figure / DPI**: shared `FIGSIZE_ANIMATION`, `DPI_FIGURE`, `DPI_EXPORT` (export uses a higher DPI than on-screen); font size and axis styling helpers `style_animation_axes`, `style_trajectory_subplot`, and related caps.
- **Stacked x/u plots**: `trajectory_stack_figsize` / `signal_stack_figsize` apply row caps; **tall** stacked layouts in **Jupyter/Colab** are gated by `minilink.graphical.environment.allow_tall_stacked_figures()` (console scripts keep a shorter default so windows stay usable). `plot_trajectory` and `plot_signals` share this policy.
- **`System.plot_trajectory` / `plot_trajectory`**: `plot=...` selects **state** (`"x"`), **input** (`"u"`), or **both** (`"xu"`) in stacked subplots. `System.compute_trajectory(..., plot=True, plot="xu")` can return a pre-styled `Trajectory` with plotting side effects as documented on `System`.
- **Animator** does not take per-call `figsize`/`dpi`; the matplotlib renderer and `matplotlib_style` own defaults (Pyro-style consistency).

`MatplotlibRenderer` and trajectory plotting are covered by unit tests; treat them as **more stable than the raw kinematic `get_kinematic_*` API**, but still not frozen at the same level as `core/`.

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
