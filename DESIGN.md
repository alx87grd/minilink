# Minilink Technical Design

`minilink` is a Python-native block-diagram framework for writing readable
dynamical systems, composing them through named ports, compiling them into flat
evaluators, and using those evaluators for simulation, optimization, and
visualization.

## 1. Design Principles

1. **Math readability first**: equation code should look close to textbook
   notation, for example `dx = A @ x + B @ u`.
2. **Pure contracts, convenient boundaries**: core equations are functional in
   intent; high-level objects may expose convenience methods at API boundaries.
3. **Explicit data flow**: prefer visible objects and direct calls over hidden
   registries, global switches, or broad metaprogramming.
4. **Backend-native math where simple**: small algebraic helpers should work
   with NumPy or JAX inputs without parallel class hierarchies.
5. **Specialize only when it clarifies**: complex plants may use explicit
   `Jax<X>` twins when a separate implementation keeps the equations readable.

## 2. Package Map

| Package | Status | Role |
| --- | --- | --- |
| `core/` | TRL 7 | `System`, `DiagramSystem`, ports, `Trajectory`, sets, costs, and lightweight blocks. |
| `compile/` | TRL 4 | Backend policy, `ExecutionPlan`, and NumPy/JAX `DynamicsEvaluator` implementations. |
| `simulation/` | TRL 4 | `Simulator`, solver backends, forced-input handling, simulation benchmarks. |
| `optimization/` | TRL 1 | Pure `MathematicalProgram`, program evaluators, optimizer method presets, and NLP benchmarks. |
| `planning/` | TRL 1 | Deterministic planning problems, initial guesses, search, policy synthesis, and trajectory optimization. |
| `dynamics/` | TRL 1 | Plant abstractions and catalog models. |
| `symbolic/` | TRL 1 | Optional SymPy mechanics derivation/export helpers. |
| `physics/` | TRL 1 | Engine-backed physics MVPs, currently JAX-focused. |
| `graphical/` | TRL 2 | Plotting, animation, graph display, renderer backends, and matplotlib style policy. |
| `control/` | TRL 0 | Controller/static-law blocks. |

Current on-disk shape:

```text
minilink/
  core/
    system.py
    diagram.py
    trajectory.py
    costs.py
    sets.py
    blocks/
      basic.py
      sources.py
  compile/
    backend_policy.py
    compiler.py
    execution_plan.py
    jax_utils.py
    benchmark.py
    evaluators/
      evaluator.py
      numpy_evaluator.py
      jax_evaluator.py
  simulation/
    simulator.py
    input_interpolation.py
    benchmark.py
    scenarios/
    solvers/
      solver.py
      scipy_ivp.py
      euler.py
      rk4_fixed.py
  optimization/
    mathematical_program.py
    optimizer.py
    benchmark.py
    evaluators/
      compiler.py
      program_evaluator.py
      numpy_evaluator.py
      jax_evaluator.py
    optimizers/
      optimizer_backend.py
      scipy_minimize.py
      ipopt.py
  planning/
    problems.py
    planner.py
    initial_guess.py
    search/
    trajectory_optimization/
      benchmark.py
      direct_collocation.py
      live_plot.py
      multiple_shooting.py
      planner.py
      shooting.py
      transcription.py
    policy_synthesis/
  dynamics/
    abstraction/
    catalog/
  graphical/
    plotting.py
    animation.py
    environment.py
    primitives.py
    graphe.py
    matplotlib_style.py
    renderers/
  symbolic/
    mechanics/
      derivation.py
      export.py
      model.py
      symbolic_system.py
      utils.py
  physics/
    engine_jax.py
    system.py
  control/
    pendulum_pd.py
```

## 3. Core Object Contracts

### `System`

`System` is the main dynamical object. It intentionally owns both the pure math
contract and a small user-facing facade.

Pure contract:

- dimensions: `n` state dimensions, `m` input dimensions, `p` output dimensions;
- dynamics: `f(x, u, t=0, params=None) -> dx`;
- output: `h(x, u, t=0, params=None) -> y`;
- ports: `InputPort` and `OutputPort`, with output-port compute callables using
  the same `(x, u, t, params)` argument shape.

`f`, `h`, and output-port compute functions are equation paths. They should
preserve the active array backend: NumPy-like inputs produce NumPy-compatible
expressions, and JAX inputs produce JAX-compatible expressions when the formula
is intended to be traceable.

Boundary conveniences:

- model defaults: `params`, `x0`, port nominal values, and `solver_info`;
- visualization hooks: `get_kinematic_geometry`,
  `get_kinematic_transforms(x, u, t)`, and `get_dynamic_geometry(x, u, t)`;
- facade methods: `compile`, `compute_trajectory`, `compute_forced`, `render`,
  `animate`, `game`, plotting, and graph helpers.

The facade methods may import simulation or graphics lazily. They do not change
the math contract of `f`, `h`, or output-port compute functions.

### Native-Array Equation Rule

The default math contract is native-array in, native-array out. This applies to:

- `System.f`, `System.h`, and output-port compute functions;
- `Set.margin`, `InputSet.margin`, and `SingletonSet.residual`;
- `CostFunction.g` and `CostFunction.h`;
- trajectory-optimization transcription equations;
- `MathematicalProgram.J`, `MathematicalProgram.h`, and
  `MathematicalProgram.g`.

Inside those equation paths, avoid forced boundary conversion such as
`np.asarray(...)`, `np.array(...)`, or `float(...)` unless the object is
explicitly NumPy-only. Use direct array math, `array_module(...)` for small
hybrid helpers, or a dedicated `Jax<Plant>` twin for complex dynamics.

Conversion belongs at boundaries: constructors and shape checks, `Trajectory`,
set `contains` / `sample`, cost reporting helpers, program evaluators, solver
adapters, plotting, animation, benchmarks, and file I/O. For example,
`MathematicalProgramEvaluator.objective(...)` may return a Python `float`, but
the underlying `program.J(z)` should remain a native scalar expression.

### Parameters

The parameter rule is global:

- `params is None` means use the object's default `self.params`;
- any non-`None` `params` value is explicit and must override `self.params`;
- equation methods should not use `params or self.params`, because an empty dict
  or empty pytree can be a valid explicit parameter set.

Compilation has two parameter modes:

- `bind_params=True`: copy params into the compiled plan/evaluator;
- `bind_params=False`: use object params where the backend can support that
  behavior.

Diagram-level parametric evaluator support is still active work. Until that tier
is implemented and tested, use `bind_params=True` or recompile when JAX diagram
parameters change.

### `DiagramSystem`

`DiagramSystem` composes subsystems through named ports. Its core duties are:

- maintain subsystem ids, connections, and flattened state/input slices;
- evaluate local subsystem inputs from external inputs, defaults, or internal
  subsystem outputs;
- expose boundary outputs separately from internal signals;
- compile to an `ExecutionPlan` for fast NumPy/JAX evaluation.

The compiled path is the main execution path. The recursive reference path is a
debug/reference implementation and must stay contract-equivalent to compiled
evaluation for `f`, boundary outputs, and params semantics.

### `Trajectory`

`Trajectory` is the canonical sampled state-input object:

- `t` has shape `(N,)`;
- `x` has shape `(n, N)`;
- `u` has shape `(m, N)`;
- optional sampled channels live in `signals` with shape `(dim, N)`.

It is a NumPy/reporting object. Simulation, planning, tracking, plotting, and
animation should share it when representing sampled state-input data.

### Sets And Costs

Sets and costs live in `core` because planning, optimization, control, and
analysis can all need them.

Set contract:

- `margin(z, t=0, params=None) -> native array`;
- feasible means all margins are nonnegative;
- `contains` and `sample` are boundary utilities and may convert to NumPy.

Input-set contract:

- `margin(u, x=None, t=0, params=None) -> native array`.

Cost contract:

- running cost: `g(x, u, t=0, params=None) -> native scalar expression`;
- terminal cost: `h(x, t=0, params=None) -> native scalar expression`;
- reporting helpers such as `evaluate_trajectory`, `terminal_cost`, and
  `total_cost` convert to NumPy/Python at the boundary.

Simple algebraic sets and costs should be single backend-native classes. Do not
add `Jax<Cost>` or `Jax<Set>` twins when ordinary array math is traceable.

## 4. Compilation And Simulation

### Dynamics Compilation

`compile(system, backend="numpy"|"jax")` returns a `DynamicsEvaluator`.

Evaluator tiers:

| Tier | Dynamics | Output | Fixed values |
| --- | --- | --- | --- |
| Standard | `f(x, u, t)` | `h(x, u, t)` | compiled/default params |
| Parametric | `f_p(x, u, t, params)` | `h_p(x, u, t, params)` | caller supplies params |
| IVP | `f_ivp(x, t)` | `h_ivp(x, t)` | nominal input and params |

For diagrams, `ExecutionPlan` is the canonical flattened representation. It
stores ordered port/state operations, slices, and boundary-output metadata.
`outputs(...)` means boundary outputs only; internal signals are reconstructed
through diagram-specific APIs such as `DiagramSystem.reconstruct_internal_signals`.

Backend strings are centralized in `minilink.compile.backend_policy`:
`"numpy"`, `"jax"`, `"auto"`, and `"direct"`.

### Simulation

The public simulation path is:

```text
System / DiagramSystem -> compile -> DynamicsEvaluator -> Simulator -> Trajectory
```

`System.compute_trajectory(...)` and `System.compute_forced(...)` are thin
convenience wrappers around `Simulator`.

Simulation policy:

- unconnected input ports contribute constant nominal values;
- reusable time-varying model signals should be source blocks in a diagram;
- run-specific forcing should be passed to the simulator as sampled data or a
  callable through `compute_forced`;
- `Simulator(..., compile_backend="auto")` may try JAX and fall back to NumPy;
- high-level `System` shortcuts default to NumPy for predictability.

Solver presets currently include fixed-step Euler/RK4 and SciPy IVP variants.
A clearer solver-options object is planned.

## 5. Optimization And Planning

### Mathematical Programs

`MathematicalProgram` is a pure finite-dimensional NLP description:

```text
minimize J(z)
subject to h(z) = 0
           g(z) >= 0
           lower <= z <= upper
```

Object contract:

- `J : (n_z,) -> native scalar expression`;
- `h : (n_z,) -> (n_h,)` or `None`;
- `g : (n_z,) -> (n_g,)` or `None`;
- optional derivative callables: `grad_J`, `hess_J`, `jac_h`, `jac_g`;
- optional box bounds and metadata;
- no initial guess and no solver state.

`compile_program_evaluator(program, backend=...)` creates the solver-facing
`MathematicalProgramEvaluator`. Evaluators own backend-specific validation,
JAX `jit`/autodiff, and SciPy/Ipopt-friendly wrappers such as `objective`,
`equality_residual`, and `inequality_margin`.

`Optimizer` is a bound solver object. It compiles the program at initialization,
stores `z0`, and selects a method preset such as:

- `scipy_slsqp`;
- `scipy_trust_constr`;
- `ipopt`.

A method preset is the user-facing bundle of optimizer backend, backend method,
and default options. User options override preset defaults.

### Planning And Trajectory Optimization

`PlanningProblem` owns the system, boundary sets, allowable sets, optional cost,
and optional params.

Boundary-set contract:

- `X0` and `Xf` are the authoritative initial and terminal feasibility sets.
- `x_start` and `x_goal` are representative points and ergonomic shortcuts.
- If `X0` is omitted, `x_start` creates a singleton `X0`; if `x_start` is
  omitted and `X0` is a singleton, it is derived from that set; otherwise it
  defaults to `sys.x0`.
- If `Xf` is omitted, `x_goal` creates a singleton `Xf`; if `x_goal` is omitted
  and `Xf` is a singleton, it is derived from that set.
- When a representative point and a boundary set are both supplied, the point
  must belong to the set.

Trajectory optimization is a deterministic planning family:

- planners own workflow, guesses, warm starts, callbacks, and result storage;
- transcriptions own decision-vector layout and emit `MathematicalProgram`;
- `compile_backend` selects NumPy, direct, or JAX-compatible equation paths;
- JAX gradients come from the program evaluator when the system, cost, and
  constraints are traceable.

There are no parallel JAX transcription classes. Direct collocation, shooting,
and multiple shooting should stay single public transcription classes whenever
their equations can be written backend-natively.

## 6. JAX And NumPy Policy

NumPy is the baseline. A NumPy-only install must import the library and run
non-JAX tests. JAX is optional and must be imported lazily.

Use one of two patterns:

- `array_module(x)` for small hybrid algebraic math;
- `require_jax_numpy()` / backend-policy helpers inside JAX-only methods.

Complex plant twins:

- new plants start NumPy-only;
- add `Jax<Plant>` only when JAX trajectory optimization, JIT rollouts, or
  differentiable physics need traceable dynamics;
- place the twin in the same module, subclass the NumPy class, and override only
  equation methods.

Do not add a global NumPy/JAX mode, a top-level `minilink.jax` package, or twin
classes for simple sets/costs/transcriptions.

## 7. Graphics, Benchmarks, And Style

Graphics:

- plotting and animation live in `graphical`;
- `System.render`, `System.animate`, and `System.game` are convenience facade
  methods;
- kinematic geometry hooks are useful but still provisional;
- matplotlib style and environment policy are centralized in `graphical`.

Benchmarks:

- benchmark helpers live beside the subsystem they measure, for example
  `compile/benchmark.py`, `simulation/benchmark.py`,
  `optimization/benchmark.py`, and `planning/trajectory_optimization/benchmark.py`;
- runnable benchmark scripts live under `tests/benchmark/`;
- benchmark helpers are not core contracts.

Coding standards:

- Python 3.10+;
- public APIs use type hints and NumPy-style docstrings;
- keep heavy optional imports lazy;
- keep equation code explicit, readable, and close to the math;
- use package `__init__.py` files as namespace markers, not broad barrel
  re-export layers, unless a future API freeze deliberately changes that.
