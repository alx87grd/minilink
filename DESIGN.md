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
| `minilink.benchmark.simulation_speed` | Shared timing helpers (`run_timed`, `summarize_durations`, `relative_l2_error`); `benchmark_sim_backend` compares one `(solver, compile_backend)` `Simulator` run to a baseline; `benchmark_sim_speed_matrix` sweeps explicit or default solver and backend lists and prints a fixed-layout matrix. |

Programmatic imports should use `from minilink.benchmark import ...`. Runnable examples are flat scripts under `tests/benchmark/` (see `agent.md` for manual script style).

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
