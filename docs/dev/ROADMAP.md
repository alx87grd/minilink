# Minilink Roadmap

Maturity and priorities, the **plan of record**; the steps themselves are the workboard,
[docs/plans/TODO.md](../plans/TODO.md), and the doc map is [AGENTS.md](../../AGENTS.md).

## 1. Releases

Identity: [CONSTITUTION.md](CONSTITUTION.md).

| Release | Milestone | When |
| --- | --- | --- |
| **v0.1** | **GRO860 end to end**, plus a working **`pip install minilink` on PyPI**. Every topic of the running optimal-control & RL course runs on the teaching surface, in Colab (git-clone cell) and in the conda env: value iteration / DP on a grid · LQR + linearization · trajectory optimization · RL via native `ReinforcementLearningPlanner` (with `Sys2Gym` + SB3 as an optional bridge). See §4.1. | Fall 2026 — `0.1.0` on PyPI (2026-09-16); `0.1.1` (2026-09-26) is the first release a GitHub tag published. Conda from `environment.yml` is the recommended install. The course notebooks' names are frozen until v1.0, with the freeze paused between terms (§4.1 gate 7). |
| **v0.2** | **GRO501 end to end** (the classical-control course: multi-physics modelling · root locus / Bode / margins · PID to spec · digital implementation · state feedback, pole placement, LQR, observers — see §4.2). Conda stays the recommended install. | `0.2.0` on PyPI (2026-10-17), from the GitHub tag; every §4.2 row green |
| **v0.3** | **The complete core** (§5.3): one architecture from the plant to the evaluation and the optimizer, supporting stochastic problems and the sampled loop, built in the natural order of the code. First (DESIGN §4–§6): **the convergence pass** (the core kept, side features to the experimental tier) and **the continuous core** (one step model under every sampled tool, one evaluation of any run, and the deterministic differentiable path to the cost behind a clean API: `evaluate`, `compare`, `problem.compile(loop)`, the program export, cost parameters; a closing PID-tuning example by hand and by the pipeline, S77). Then two invariants (every object pure, every number a parameter; the step algebra first-class) and seven pillars (the eight items the maintainer named, the parametric and the stochastic programs grouped as one): one discrete judge, the draws over starts, parameters, disturbances and block seeds, the seeded stochastic policy, the sampled loop as a step diagram with two run verbs, one generic parametric program with the stochastic program its case, a native solver and an MPC on the planner's pure face as a proof of concept (MPPI on the same face), Fields. The teaching surface covers every GMC714 lab (§4.3); the course material stays on `main` until v0.3 merges. | `0.3.0` on PyPI (2026-10-30), from the GitHub tag; the core's small leftovers carried into v0.4 |
| **v0.4** | **The clean surface** (§5.4, [TODO §1](../plans/TODO.md#1-v04--the-clean-surface)): the API one vocabulary and one shape across the tools (RULES 2.4 and 2.7, the planner's four kinds of argument; landed on `dev` after the tag), the tutorials in three layers (landed), the one parameters model and no cache (DESIGN §4; landed 2026-11-05), the nine API surface rulings (landed 2026-11-05), plans as policies (one face for every planner, value iteration's finite horizon, the cost-to-go as fields; [plans-as-policies.md](../plans/plans-as-policies.md); its first two changes landed 2026-11-08), the core API proposals CA-1, CA-2 and CA-5 and the root exports (landed 2026-11-08), the v0.3 core's small leftovers (mostly landed 2026-11-08), the API naming sweep (landed 2026-11-08, PR #138), the API review's defects (landed 2026-11-10) and the rest of the API settled before the demo rebuild ([api-stabilization.md](../plans/api-stabilization.md); left), and the demos, notebooks, tests and CI redrafted on the core (left). Breaking names land here, once, with every notebook in the same change. | After v0.3 (no date) |
| **v0.5** | **Features on the core** (§5.5), not intertwined with it: identification as the program export's first data consumer; `LQGPlanner`; workspace geometry; the RL follow-ups; native solvers beyond the proof of concept; the general serial chain (C6); pyro parity and the former wave G rows; traceable linear analysis. | After v0.3 (no date) |
| **v0.9** | **Freeze candidate** (§5.6): the foundation questions that could still break a public name are decided (Diffrax as an optional solve, the glyph/solid rename, the frozen surface); the trust contract is in place (deprecation policy, pinned export list, CHANGELOG, pinned installs); the standard-syllabus holes close (estimation; the discrete-time one closes with S75 in v0.3); the docs site renders the tutorials. | Spring 2027 (indicative, set before 2026-10-26; the releases carry no dates now) |
| **v1.0** | **A colleague can build a course on it and trust it for three years** (§5.7): the API frozen under the deprecation policy, the standard control syllabus covered and checked against textbook worked examples, course-neutral labs an instructor adopts outside UdeS, green on Windows, macOS and Linux; after two cohorts. Two Stable-Baselines3 teaching notebooks retire (S49) and the GRO860 name freeze becomes the general one. | 2027 (indicative, set before 2026-10-26; v1.0 comes when the API feels mature) |

## 2. Two lanes

Philosophy: [CONSTITUTION.md](CONSTITUTION.md) §3 (the student syntax never
breaks). Placement shorthand: [RULES.md](RULES.md) §3.3. The operating
contract is the table below — not a pointer.

Minilink serves two audiences with one codebase. The boundary between them is
a **contract**, not a documentation convention.

| Lane | What it is | Rules |
| --- | --- | --- |
| **Teaching surface** | The names students meet: root prelude (`from minilink import …`) and the band facades (`minilink.catalog`, `.blocks`, `.control`, `.estimation`, `.analysis`, `.simulation`, `.planning`, `.learning`, `.optimization`, `.dynamics`, `.core`). Registered in one place, tested as a set. | *Soft rule:* nothing enters without a demo or notebook, a both-backends test where it defines dynamics, and a docstring. Names and semantics change only with a deprecation note. Student-facing examples and notebooks import **only** through it (CI-checked). |
| **Research lane** | Everything else: the provisional bands (hybrid, MPC, realtime), the experimental tier (`minilink/experimental/`: symbolic mechanics, the contact-engine leaf, C export, the unsupported plants once the catalog clean-up moves them; the top-level `experimental/` folder of prototype scripts), `examples/projects/`. | No stability promise, no entry requirements, out of the release contract. The provisional bands ship in the wheel. The experimental tier is **out of the wheel**, **untested by default** (nightly smoke at most; `c_export`'s flagship smoke and the quick C-export demo of `showcase_jax.ipynb` the two gate exceptions), off the docs site, never cited by a course, and the staging area for what may leave into its own project at v1.0. Deep imports always remain valid. |

**Provisional teaching names:** the spatial names the planning facade exports (`Scene`,
`mount`, `ReferenceTrack`, `car_outline`, `plot_track`, …) sit in the teaching lane under
`planning.spatial`, checked like every band-facade name (a docstring, a teaching-lane home)
but outside the registry, and stay provisional until A3 moves them to `core/geometry/`.

**Wheel scope:** the published wheel is the `minilink` package without
`minilink/experimental/`: the teaching surface, the provisional bands with no stability
promise, and the provisional spatial names. `minilink/experimental/` and the research
examples (`examples/projects/`, the top-level `experimental/` folder) are repo-only.

**Breaking changes so far** (each migrated every course notebook in the same commit; from
v0.9 they go to `CHANGELOG.md`, step V5):
- 2026-09-15: every planner returns a `PlanningSolution` (`policy`, `solver`, `trajectory`,
  `evaluation`, `cost_to_go`) and the evaluator an `Evaluation`; `TrajectoryPlan`,
  `PolicyPlan`, `SolveMetadata` and `MonteCarloReport` are gone. `PlanningProblem.on_exit`
  is gone, `exit_cost` is `failure_cost`, and `X` is unconstrained unless declared.
- 2026-09-30: `WhiteNoise(p, *, psd, sample_period, seed, hold)` is a pure block; its
  `var`, `mean`, `t0`, `tf` and `refresh()` are gone (an old variance reads as
  `psd = var × sample_period`), and seeded realizations changed once.
- 2026-10-14: the state-feedback designs live in `control/design.py` (`lqr`, `place`,
  `lqr_finite_horizon`, `lqg`, `trajectory_lqr`); the modules `control/lqr.py`, `place.py`
  and `output.py` are gone, their names unchanged on the teaching surface.
- 2026-10-17 (`0.2.0`): a block's default `name` is its class name and a shortcut is named
  as the expression it builds, so diagram labels and plot titles changed; the pip extras are
  `full` and `dev` only (`[jax]`, `[plotting]`, `[visualization]`, `[diagrams]`,
  `[symbolic]`, `[rl]`, `[ipopt]` are gone); `install.md` and `environment-basic.yml` are
  gone, the install story is the README's.
- 2026-10-24 (`0.3.0`; every lesson migrated in the same commits): the removals (`ProblemEnv` off the facade, `to_gymnasium`, the camera
  helpers, `Replay`, `Sampler`, `RateLimiter`, `Lag`, `DeadZone`, `Relay`, `NotchFilter`,
  `Washout`, `SimpleIntegrator`; `ZOHHold` is `UnitDelay`, `NeuralNetwork` folded into `MLP`);
  the package moves (`planning/dynamic_programming/`, `minilink.learning`, MPC under
  trajectory optimization); A6's short names and every block's default id its class name; the step model: the evaluator's integrator grid is `ev.step` / `ev.rollout`
  (`solver=`, `substeps=`, `hold=`), `integrator=` is `solver=` everywhere; every evaluator
  method one body with `params=None`, compiled once on JAX (`f_p`, `*_trace`, `*_jit`, the
  `_ivp` and SciPy adapters and `ev.jit` gone),
  `discretize(sys, dt, *, solver, substeps)` with `dt` required, and `compute_forced` on
  `rk4_fixedsteps` holds a sampled input (those forced numbers changed once); the problem: a
  plain class whose `params` is a live view of its parts (`ProblemParameters` and `params=`
  gone), `U` unconstrained unless declared (`U=plant.input_box`), `x_goal` never creates `Xf`,
  `discount_rate` on the problem (gone from the cost), `failure_cost` `None` (the
  cost bound, `cost_bound()`) or a number, value iteration's default price the cost
  bound instead of `1e6` (its numbers changed once); the costs: parameters on the System
  pattern, a sum's terms keyed by id, `evaluate_trajectory` / `terminal_cost` / `total_cost` off
  the cost (`traj.with_cost`, `evaluate_trajectory(problem, traj)`); the evaluation:
  `MonteCarloEvaluator`, `score_trajectory` and `Comparison.evaluate` gone for
  `problem.compile(controller, dt)`, `evaluate` and `compare(problem, ...)` (the continuous loop
  integrated to `tf`, so the scores changed once); the optimizer: presets `gd`, `adam`,
  `l-bfgs`, `method="auto"`, `z0` optional; `MechanicalSystem` and `GeneralizedMechanicalSystem`
  invent no bounds (each catalog plant states its own, the same numbers), so a user's own
  mechanical subclass starts unbounded.
- 2026-11-04 (on `dev`, for `0.4.0`; every notebook migrated in the same commits, no alias):
  the vocabulary of RULES 2.7: `x_start` → `x0`, `t_start` → `t0`, a problem's `tf` →
  `horizon` (`tf` is an absolute end time), `N` → `n_steps`, MPPI's `n_samples` →
  `n_rollouts`, `iterations` → `n_iterations`, the discount `alpha` → `gamma` (RRT*'s
  `gamma` → `radius_constant`), `algorithm=` → `method=`, `solution.solver` →
  `solution.record`, `xbar` / `ubar` / `ybar` → `x_bar` / `u_bar` / `y_bar`, `TimeCost(eps=)`
  → `radius=`, `plot_cost2go` / `plot_policy` → `plot_cost_to_go` / `plot_control_law`, the
  `solve_*` aliases gone, `verbose=False` by default, the `Optimizer`'s default method
  `"auto"`; the option records (`DynamicProgrammingOptions`, `RRTOptions`, `RRTStarOptions`,
  `TrajectoryOptimizationOptions`) gone for flat keywords, `options=` only the external
  solver's dict; every planner's `params` tree and `solve(params=)` (the neural RL planner's tree a report); trajectory
  optimization's `last_optimization_result`, `iteration_history`, `last_solve_time_s` and
  `last_step_time_s` gone for `solution.record` (RULES 2.4).
- 2026-11-05 (on `dev`, for `0.4.0`; every notebook migrated in the same commits, no alias):
  `mean_at`, `mean_draw` and `Gaussian.full_cov` → `sample(None, params=)` and `cov`; `BoxSet.from_system_state`, `BoxInputSet.from_system_inputs` and
  `from_bounds` → `sys.state.box`, `sys.input_box`, `BoxSet(lower, upper)` (`U=` takes a box);
  `traj.sample` / `t2x` / `t2u` / `with_signal` / `time_duration` / `copy` → `traj.at(t, name)`,
  `with_signals`, `tf - t0`; `ev.jacobian(of, wrt)` returns the array at a point (no callable);
  the program evaluator's `gradient` / `hessian` / `jacobian_h` / `jacobian_g` → `grad_J` /
  `hess_J` / `jac_h` / `jac_g`; `record_solve_time` gone, the solve time always recorded at a
  user's verb; `solve_trajectory_from`, `step`, `compile_parametric_program` and the parametric
  transcription gone for `planner.solve(params=)` and `planner.compile()`, the RL planner's
  rollout verb gone for `evaluate(problem, solution.controller, record=True)`,
  `PlanningProblem.start_point` gone for `x0`; `diagram.trajectory_of` → `traj.of("plant")` on
  every diagram run (`record=("plant:u",)` for the input), `diagram.of("plant")`;
  `compute_forced` / `solve_forced` → `compute_trajectory(u=)` / `solve(u=)`, one input rule on
  every solver (a sampled input held, a function read where the solver evaluates `f`),
  `compute_rollout`'s input sequence `(m, n_steps)`; an intersection's, a union's and a scene's
  members keyed by class name; the `input_interp` solver option gone.
- 2026-11-06 (on `dev`, for `0.4.0`): a diagram run records the plant's whole input by default, `record="plant"`, on every solver and on the sampled loop
  (`record=None` records nothing, a tuple names signals, nested paths included, `"all"` every
  output port), so `traj.of("plant")` is complete on every default run; `record=None` turns off
  the sampled loop's forced record of `"plant:u"`; a default adaptive run of a diagram takes one
  signal pass per saved sample after the run (about 0.13 s on 10 001 samples); a fixed-step run
  records from its own first stage at no measurable cost.
- 2026-11-07 and 2026-11-08 (on `dev`, merged by PRs #135 and #136, for `0.4.0`; every
  notebook migrated in the same commits, no alias):
  - *The controller is the block, the policy the function.* `PlanningSolution.policy` →
    `solution.controller`; on every planner `get_controller()` → the property `controller`
    (its `interpolation=` variant built directly, `LookupPolicy(grid, pi, interpolation=)`),
    `value_at(x)` → `cost_to_go(x, t=None)`, `plot_solution` / `animate_solution` →
    `plot_trajectory` / `animate`; value iteration's `animate_policy()` →
    `animate_control_law()`; the deep RL planner's trained block is `planner.actor`, its
    exploration law `planner.exploration_policy`, and `ExplorationPolicy.controller` → `actor`.
  - *The solution's shape.* `PlanningSolution` is a keyword-only record `(problem, controller,
    record, trajectory, cost_to_go, evaluation, params)` with no function on it (the law as a
    function is the compiled face's `pv.command(z, x, t)`); `trajectory` is an open-loop
    planner's plan only, `None` for a feedback law; `cost_to_go` is a `Field` or `None`, read
    `J(x)` or `J(x, t=t)` (value iteration's and tabular learning's a `GridField`, LQR's a
    `QuadraticField`, the RL critic a `CriticField`); every planner's `nominal_trajectory` and
    `holds_policy` are gone.
  - *The evidence scores the controller as it is handed out* (scores moved once): a feedback law
    (value iteration, tabular and deep RL) is continuous, no longer held on its grid's `dt`, and
    is integrated on the planner's own `dt`, so `solve(evaluate=True)`, `evaluate(problem,
    sol.controller, dt=)` and `compare(..., dt=)` agree; `evaluate` on a continuous controller
    with no `dt` runs on the loop's automatic step. A step problem's planners score on its `run`.
  - *The planners' math.* Value iteration on a finite horizon runs exactly `N` sweeps, a law per
    stage and `J(x, t)` (it ran to convergence before; a script that means the stationary law
    solves `problem.replace(horizon=None)`); its record's `fixed_horizon` → `mode` (`"infinite"`,
    `"finite"`, `"receding"`) beside `converged`; `solve_steps(n_steps)` names its count;
    `bellman_backup` is gone (the backup is written in each engine); the interpolation
    `"spline"` → `"cubic"`. LQR solves a discounted problem as discounted (with a warning), its
    finite-horizon law applies the stationary gain past the horizon, and `LQRPlanner.lqr_matrices()`
    → `design_matrices()` (the module function `lqr_matrices` stays).
  - *The workflow on the verb.* `verbose`, `callback`, `live_plot`, `record_history` and RRT*'s
    `history_stride` leave every planner's constructor for `solve` (and `solve_steps`); RRT's
    `live_plot`, `live_plot_every`, `live_plot_pause`, `live_plot_after_goal_only` and
    `live_plot_ax` → the hook `LiveSearchPlot(every=, pause=, after_goal_only=, ax=)` given to
    `solve(callback=)`, which takes a callable or a list (`LiveSearchPlotCallback` →
    `LiveSearchPlot`); the training budgets `solve(episodes=)` → `n_episodes`,
    `solve(timesteps=)` → `n_timesteps`, the records' fields alike, and the RL planner's
    `num_timesteps` → `n_timesteps`.
  - *The problem.* `problem.compile(controller, dt=None, ...)` always takes the controller (the
    controller-less `problem.compile(dt=)` is gone); `problem.discretize(dt)` answers its own
    equations, `transition`, `terminal`, `run(controller)`, `cost_bound()` and
    `failure_cost_at()` (`StepPlanningProblem.compile` and `StepFace` are gone); the problem's
    and the planner base's helper methods are module functions (`require_cost(problem)`,
    `require_finite_horizon(problem)`, `default_x0_of`, `part_trees_of`, `sources_of`,
    `require_solution(planner)`; `require_goal` is gone).
  - *The systems.* `refresh()` is gone (a block's state size is fixed once built); the
    constructor's `n` → `state_dim` on `System`, `DynamicSystem`, `StepSystem` and their
    subclasses, `StateSpaceSystem`'s `n`, `m`, `p` → `state_dim`, `input_dim`, `output_dim` (the
    attributes `n`, `m`, `p` unchanged); a static `System` takes the port shortcuts and feeds its
    input through by default (`y_dependencies=("u",)`); `h` is the `"y"` port on every block and
    on the evaluators (`ev.h`), so `Sum`, `Gain`, `Mux`, `Saturation` and `MLP`, whose `h`
    returned zeros, return their output.
  - *Imports.* The path names (`PolylinePath`, `ReferencePath`, `circuit_waypoints`,
    `from_waypoints`) import from `minilink.core.geometry`, no longer from
    `minilink.planning.spatial` (the `minilink.planning` facade keeps them);
    `minilink.optimization.reporting` → `minilink.core.reporting`;
    `minilink.simulation.compile_backend` is gone (`build_on_auto` and `NotTraceableError` in
    `minilink.core.compile.compiler`). Added: the core's mathematical types on `minilink.core`,
    `QuadraticField` and `GridField` on the root, `LiveSearchPlot` on `minilink.planning`.
  - *Numbers and names that moved once.* `scipy_stiff` runs at `scipy`'s tolerances (`rtol=1e-4`,
    `atol=1e-7`) with the Jacobian on both backends; `PurePursuit`, `LineOfSight` and
    `HeadingController` are named by their class.
  - *Gone from the examples:* the topic notebooks `topics/reinforcement_learning/drone_ppo` and
    `pendulum_value_iteration_vs_lqr_vs_rl` (GRO860's own copies stay).
- 2026-11-08 (the API naming sweep, merged into `dev` by PR #138, for `0.4.0`;
  every notebook migrated in the same commits, no alias):
  - *Counts.* `n_steps` counts transitions, the textbook `N`; a count of time samples is
    `n_samples`: `compute_trajectory(n_steps=)` and `Simulator(n_steps=)`, the LQR gain
    schedule's (`lqr_gain_schedule`, `lqr_finite_horizon`, `LQRPlanner`) and `animate_modal`'s
    `n_steps=`, and `step_response`, `bode`, `nyquist`, `frequency_response` and their `plot_`
    twins' `n=` → `n_samples=`; `Source.show_signal(n_pts=)` → `plot_signal(n_samples=)`.
    Trajectory optimization's `n_steps` counts intervals: a plan has `n_steps + 1` knots, one
    more than before for the same number (its numbers moved once).
  - *One keyword per kind of choice.* A plot's or an animation's drawing library is
    `renderer=` (`backend=` is the array module only), `game(plot_backend=)` and the realtime
    simulator's → `plot_renderer=`; value iteration's `backend="loop"` → `method="loop"`;
    RRT's `nearest_backend=` → `nearest_method=`.
  - *Value iteration's policy evaluation.* The `PolicyEvaluator` class is gone:
    `planner.evaluate_policy(controller)` returns `J^π` as a `GridField` on the planner's own
    grid and numbers, read `J_pi(x)` and drawn by `planner.plot_cost_to_go(field=J_pi)`;
    `DynamicProgrammingResult.value_at` is gone.
  - *A step problem's run.* `step_problem.run(controller)`, which returned `xs, us, cs, J`, →
    `step_problem.rollout(controller)`, a `Trajectory` (`x` is `(n, N + 1)`) whose signal
    `"cost"` is the running cost, `J` its last value (`traj.signals["cost"][0, -1]`).
  - *Records.* `iterations` → `n_iterations`, the learning records' `algorithm` → `method`,
    `PlanningSolution.method` → `family`, RRT's `nodes` / `tree_nodes` → `n_nodes` and
    `RRSearchIteration` → `TreeSearchIteration`, a trajectory-optimization iterate's `max_eq` /
    `min_ineq` → `max_equality_violation` / `min_inequality_margin`; `**algorithm_kwargs` /
    `**method_kwargs` → `**method_numbers`.
  - *Keywords.* `evaluate(..., record=True)` → `trajectories=True`; `StateSpaceGrid(x_grid_shape=,
    u_grid_shape=)` → `x_grid=`, `u_grid=`; `KinodynamicExtender(n_substeps=)` → `n_steps=`;
    `Sys2Gym(tf=)` and `env.tf` → `episode_length`; the channel count of the sources, white
    noise and the PID family, `p=` / `dof=` → `dim=`; `show_3d=` → `is_3d=`; the grid plots'
    `axes=(i, j)` and `slice_2d(axis_x, axis_y)` → `x_axis=i, y_axis=j`; `plot_diagram(filename=)`
    → `file_name=`; `LyapunovCertificate.plot(limits=)` → `bounds=`;
    `animate(time_factor_video=)` → `speed=`; `region_of_attraction(samples=)` → `n_samples=`;
    `step_info(time, y)` → `step_info(t, y)`; `discretize(system=)` → `sys=`;
    `RRTPlanner.show_live_search()` is gone for `solve(callback=LiveSearchPlot())`. The RL
    planner's `solve(params=)` takes the planner's whole tree, as every planner's.
- 2026-11-10 (the API review's defects, [api-stabilization.md](../plans/api-stabilization.md)
  §1, merged into `dev` for `0.4.0`; no name breaks but one):
  - *Refused now:* `lqr`, `lqr_finite_horizon`, `kalman`, `lqg`, `trajectory_lqr` and
    `LQRPlanner` on a step or sampled model (they solved the continuous Riccati equation on
    the discrete matrices); `compute_trajectory` given both `n_samples=` and `dt=`; a
    first-order `Optimizer` method on a program without a gradient, at construction.
  - *Numbers that moved once:* `BoxSet.sample(None)` is the box's center (it drew fresh
    entropy); `field.as_set(upper=level)` is the sublevel set (it was the band
    `0 ≤ V ≤ level`); RRT and RRT* plans start at the problem's `t0` and their evidence runs
    the plan's own length.
  - *New warnings:* a design verb at a point the plant does not rest at (`u_bar=`,
    `find_equilibrium`); scoring a problem with no horizon (the 10 s it runs,
    `episode_length=`); every warning of a default points at the caller's line.
  - *Renamed:* `KinematicManipulator(effector_dim=)` → `task_dim=`, as `Manipulator`;
    `forward_differential_kinematic_effector` is gone.
  - Also: a one-line `repr` on systems, sets, fields, costs, laws and the `Optimizer`;
    refusals that name what to pass; `SumCost.g_max` / `h_max` index their terms as `g` does.
- 2026-11-10 (the API settled before the demo rebuild, branch `api-settle`, for `0.4.0`; every
  notebook migrated in the same commits, no alias):
  - *Subsystem, the one word for a system placed in a diagram.* `print(diagram)` lists
    `subsystems:` (it said `blocks:`); `get_subsys_input_port` → `get_subsystem_input_port`,
    `compute_subsys_output_port` → `compute_subsystem_output_port`. "Block" keeps one meaning,
    the standard components of `minilink.blocks`.
  - *Solver names.* `Simulator` and `compute_trajectory` take the step model's words, as
    `discretize`, the evaluators and the sampled loop do: `solver="rk4_fixedsteps"` →
    `"rk4"`, `"euler_fixedsteps"` → `"euler"`, both fixed-step; the variable-step Euler is
    gone, and a non-uniform grid on a fixed-step solver is refused, naming `solver="scipy"`
    or a uniform `dt`. The SciPy presets are unchanged.
  - *A set's `margin` takes its member first, on every set* (`X.margin(x, t)`,
    `U.margin(u, x, t)`), now written in CONSTITUTION §2; a box, a ball or a singleton refuses
    a member of another size (`Pendulum().input_box.margin(x, u)` returned four numbers).
  - *The author's reads.* `sys.get_port_values_from_u(u, "r", "y")` → `sys.ports(u, "r", "y")`
    (the same three return forms: every port as a dict, one port, a tuple of several);
    `traj.get_signal(name)` → `traj.signal(name)`.
  - *The transcriptions take their interval count.*
    `DirectCollocationTranscription(DirectCollocationOptions(n_steps=20))` →
    `DirectCollocationTranscription(n_steps=20)`, the same for `ShootingTranscription` and
    `MultipleShootingTranscription`; the option records are gone, and the grid
    (`t(problem)`, `dt(problem)`, `n_knots`) is read on the transcription.
  - *Every count is `n_<thing>`.* A compiled planner's `status["iterations"]` →
    `status["n_iterations"]` (trajectory optimization, MPPI, the MPC's verbose line);
    `MPPIRecord.effective_samples` → `effective_rollouts`; `StateSpaceGrid.nodes_n`,
    `actions_n` → `n_nodes`, `n_actions`; `LyapunovCertificate.verify(n=)` →
    `verify(n_rollouts=)`, and its plots' `n=` (`certificate.plot`,
    `plot_region_of_attraction`) → `n_samples=`.
  - *The plots' keywords.* `plot_sampled_futures(axes=("t", 0))` →
    `plot_sampled_futures(x_axis="t", y_axis=0)`; value iteration's
    `plot_control_law(axis=)` → `u_axis=`, as every other control-law plot, and its title
    reads "Control law u[0]"; the animations write a file with `file_name=` (RRT's
    `animate_search` and `animate_convergence` took `save=`, value iteration's
    `animate_cost_to_go` and `animate_control_law` took `save_path=`);
    `StateSpaceGrid.plot_value` → `plot_cost_to_go`.
  - *`tf` is an absolute end time.* `DynamicProgrammingPlanner.final_time` → `planner.tf` (the
    problem's `t0 + horizon`); where a length was called `tf`, it is `horizon` or
    `episode_length`: `RolloutEnvironment.tf` → `env.episode_length`, and DESIGN's scoring
    formula reads `e^{−ρ horizon} h(x(t_end), t_end)` with `t_end = t0 + horizon` (the
    internal `score_run(horizon=)`, `planner_evidence(episode_length=)`, `steps_of(horizon,
    dt)` with it).
  - *Exported.* `DiscreteLTISystem`, the pure difference equation (a step linearization's
    deviation model), is on `minilink.dynamics` beside `LTISystem`; its `period=` keeps its name (renaming
    it `sample_period` would put the model on a clock, an open question).
  - *One signature order, the model's `(x, u, t; p)`.* `RolloutEnvironment.step(x, t, u, key,
    params)` → `step(x, u, t, key, params=None)`; `LookupPolicy.action(x, params=None, *,
    t=0.0)` and `NeuralPolicy.action(x, params=None)` → `action(x, t=0.0, params=None)`
    (pass `params=` by keyword).
  - *A word per idea.* `MPPIPlanner.method` (its dict of numbers, `params["method"]`'s
    defaults) → `planner.method_numbers`; `TabularLearningPlanner(eta=)` and `.eta` →
    `learning_rate` (the `params["method"]` key with them), as the `Optimizer`'s;
    `ReinforcementLearningPlanner.train_time` → `train_time_s`, as its record;
    `TrajectoryOptimizationRecord.solver_success` → `optimizer_success`.
  - *The optimizer's native numbers are keywords.* `Optimizer(program, method="gd",
    options={"maxiter": 5, "gtol": 1e-6})` → `Optimizer(program, method="gd",
    max_iterations=5, tol=1e-6)` (Adam's `max_grad_norm` too); `options=` is refused on `gd`
    and `adam` and stays the external solver's dict (SciPy, Ipopt); trajectory optimization
    takes `max_iterations=` and `tol=` on a native method the same way.
    `OptimizationResult.n_iterations` reads `stats["nit"]`; `stats["algorithm"]` →
    `stats["method"]`.
  - *One word per idea, on the runs and the sources.* `step_sys.plot_rollout(rollout=)` →
    `plot_rollout(traj=)`; `compute_trajectory(u=…, input_port_id="left")` and
    `Simulator.solve(input_port_id=)` → `port="left"`, as the blocks say; the hold between
    samples is one keyword and two words on the three tools that read it:
    `TrajectorySource(t, u, interpolation="previous")` → `TrajectorySource(t, u, hold="zoh")`
    (`"linear"` the default, `from_trajectory(hold=)` with it, `source.hold` the attribute),
    and the step model's `evaluator.rollout(…, hold="foh")` → `hold="linear"`, as the noise
    sources already said.
  - *Keyword-only where every sibling already is.* `PlanningProblem(sys, *, x0=, cost=, …)`;
    a system's `compile(*, backend=, verbose=)` (`plant.compile("jax")` →
    `plant.compile(backend="jax")`, the module function `compile(system, *, backend=, …)`
    with it); `sys.animate(traj=None, *, speed=, …)` and `sys.render(x, u, t, *, is_3d=, …)`;
    `MLP(input_dim, output_dim, *, hidden=, …)`; `Sys2Gym(sys, cost, *, dt=, …)`;
    `NativeSampledSystem(state_dim=0, *, sample_period=, input_dim=, expose_state=)`, the
    port shortcuts of a `System` but the output, declared in seconds. The reverse:
    `evaluate(problem, controller, dt=None, *, …)` takes `dt` by position, as
    `problem.compile(controller, dt)` does.
  - *Removed: no caller, or against a rule.* `validate=` on `feedback`, `closed_loop` and
    `autowire` (it switched off the algebraic-loop check, which now always runs);
    `PlanningProblem.has_goal` and `has_cost` (write `problem.Xf is not None`,
    `problem.cost is not None`); the `COMPILE_BACKEND_AUTO` re-export of
    `minilink.simulation.simulator` (`backend="auto"`); `InputPort.get_default_value()` and
    `signal.set_nominal_value(v)` (read `port.nominal_value`, assign
    `signal.nominal_value = v`); `RRTPlanner.reached_goal` and `.iterations`, and RRT*'s,
    copies of the record (`solution.record.reached_goal`, `solution.record.n_iterations`);
    the comparison's record `GivenPolicy` → `GivenController`.
  - *Library-internal helpers, one name each.* The problem's five checks (`coerce_horizon`,
    `coerce_state`, `require_type`, `check_state_shape`, `require_member`) and the
    `Optimizer`'s `start_vector`, `decision_vector` and solve panel
    (`print_program_preamble`, `print_program_report`) are module functions, no longer
    methods; `steps_of`, `planner_parts` and `require_cost_to_go` have one body each (MPPI's
    refusal of a `dt` that does not divide the horizon now says `dt=… must divide the horizon
    …`); MPPI's episode tree is `episode_leaves` and the step problem's check
    `coerce_n_steps`, beside `problem_leaves` and `coerce_horizon`. A sampled block's
    predicates read as questions: `SampledSystem.counts` → `is_tick_law`, `resolves` →
    `is_integrated`; `minilink.core.sets.is_bounded(box)` → `has_finite_face(box)`.
  - *The drawing shortcuts write their keywords out.* `sys.plot_input_output_map(…)`,
    `sys.plot_control_law(…)` and a sampled loop's `animate(…)` take the named keywords of the
    function they draw with, so an editor shows them (a misspelt keyword is refused at the call);
    `plot_diagram(show_inline=True)` → `plot_diagram(show="inline")`, `show_pdf=True` →
    `show="pdf"`, `show="window"` the Matplotlib window; `show=True` still picks the place
    (inline in a notebook, a window in a script) and `show=False` builds the graph only.
  - *The impedance laws take the PID family's signature* (the maintainer's ruling 9):
    `Impedance(dof=1, *, Kp=10.0, Kd=1.0, …)` → `Impedance(Kp=10.0, Kd=1.0, *, dim=1,
    tracking_ref=False, …)` and `IntegralImpedance(Kp, Ki, Kd, *, dim=1, …)`, so
    `Impedance(10.0)` is a stiffness of 10, no longer a 10-axis block; `dof=` → `dim=` and the
    attribute `.dof` → `.dim`, as `PD(Kp, Kd, *, dim=1)`. `JointImpedance` and
    `TaskImpedance` take the plant and keep its `dof` (a manipulator's joints).
  - *The linear models* (the maintainer's ruling 11). `linearize(plant, x_bar)` returns a
    `LinearizedSystem`, `ẋ = f(x̄, ū) + A (x − x̄) + B (u − ū)`, `y = h(x̄, ū) + C (x − x̄) +
    D (u − ū)`, in the plant's own coordinates, ports, labels and units (it returned an
    `LTISystem` in the deviations whose `f` ignored the `x_bar` it stored, so
    `lqr(lin, Q, R) @ lin` settled the upright pendulum at 6.27 rad); its `params` hold
    `A, B, C, D` and the point (`x_bar`, `u_bar`, `y_bar`, the drift `dx_bar = f(x̄, ū)`),
    read as `lin.x_bar`; its nominal state is `x̄`, its `x0` the plant's; `lin.deviation_model()`
    is the pure `LTISystem` with Δ labels (they read `Delta θ`). A step or sampled system gives
    `LinearizedStepSystem` (`x_next_bar = F(x̄, ū)`), its deviation model a `DiscreteLTISystem`.
    `LTISystem(A, B, C, D)` and `DiscreteLTISystem` are pure: `x_bar=`, `u_bar=`, `y_bar=` and
    their attributes are gone (a model from arrays sits at the origin). The analysis verbs read
    `A, B, C, D` only, no number moved, and warn on a linearization off an equilibrium; a
    design verb reads the point from the linearization, and a point given with a linear model
    linearizes it there (it was refused). A linearization's step response is titled
    "Linearized" and drawn as the deviation `Δy`. The matrices stay methods, `lin.A()`; a
    method passed where its matrix goes (`lqr_gain(lin.A, …)`) is refused with the call to
    write. `sys.get_u_from_input_ports()` → the read-only `sys.u_nominal`.
  - *`Step` reads its final level first* (the maintainer's ruling 13):
    `Step(initial_value=None, final_value=None, step_time=1.0)` →
    `Step(final_value=1.0, initial_value=0.0, step_time=1.0, *, port="y")`, so `Step()` is the
    unit step at `t = 1 s` (it was zero) and `Step(1.0)` steps up (it stepped down); a scalar
    level takes the other level's size. The positional callers were flipped; a bare `Step()`'s
    numbers move once (no test asserted them).
  - *A plant's signals* (the maintainer's ruling 7): positions, then their rates; a rate
    `d<position>` unless the textbook names it; bare units, the plot adds the brackets; default
    labels counted from 1. `QuarterCar` and `ConstantSpeedHelicopterTunnel` were rates first,
    `[dy, y, x]`: their state is `[x, y, dy]` (`x[0]` the forward position, `x[2]` the vertical
    rate). `MechanicalSystem`'s default labels `"Angle 0"`, `"Velocity 0"`, `"Torque 0"` →
    `q1`, `dq1`, `u1` (units `"rad"`, `"rad/s"`, `"Nm"`, they read `"[rad]"`, `"[rad/sec]"`,
    `"[Nm]"`), `GeneralizedMechanicalSystem`'s → `q1`, `v1`, `u1`; a manipulator's task ports
    read `p1…`, `dp1…` in `"m"`, `"m/s"`, and the robotic and model-based laws' inputs
    `q1…`, `dq1…`.
  - *`ModelPredictiveController` is the class* (the maintainer's ruling 15): it was a function
    returning a `PredictiveController`; the class takes the function's signature,
    `ModelPredictiveController(planner, *, inputs=None, delay=False, warm_start=True,
    backend=None, verbose=False)`, so `isinstance(mpc, ModelPredictiveController)` holds and
    `PredictiveController` is gone. Import it from the teaching facade,
    `from minilink.control import ModelPredictiveController` (the
    `minilink.planning.trajectory_optimization.mpc` export stays).
  - *The optimizer's method names the algorithm* (the maintainer's ruling 12):
    `method="scipy_slsqp"` → `method="slsqp"`, `"scipy_trust_constr"` → `"trust-constr"`
    (SciPy's own spellings), on `Optimizer`, trajectory optimization (its default is
    `"slsqp"`) and the MPC presets; the old strings are refused. `backend=` (the array module)
    and the method select the implementation through one table keyed `(method, backend)`;
    every method runs as before on NumPy and JAX.
  - *A port and the box it builds say the same word* (the maintainer's ruling 5):
    `add_input_port(…, lower_bound=, upper_bound=)` → `add_input_port(…, lower=, upper=)`,
    `add_output_port` and `connect_new_output_port` with it; the attributes
    `port.lower_bound`, `state.upper_bound` → `port.lower`, `state.upper`, as the box
    (`port.box.lower`), the sets, `Saturation` and the program already said. A limit a law
    applies to a named signal keeps that signal's name (`u_min`, `u_max`, `delta_max`).
  - *Added, nothing breaks* ([api-stabilization.md](../plans/api-stabilization.md) §3):
    `QuadraticCost(Q, R)` (`S`, `x_bar`, `u_bar` zero by default); `lower`, `upper`, `dim` on
    `BoxInputSet` and `Uniform`, `dim` on every set and field that knows its size,
    `BallSet.sample` and `SingletonSet.sample` (so a ball is a start a problem draws from);
    `CostFunction.h` zero by default (a running cost writes `g` only); `Gain(2.0)` one channel
    (`dim=1`, it was refused), `Integrator(dim=1)`, `Saturation` with one end stop per channel;
    `poles(sys, x_bar)` on `minilink.analysis` and the root (the linear model's eigenvalues, in
    `z` on a step system); `print(lqr(…))` shows `K`, read-only as `ctrl.K`;
    `sys.plot_nyquist(…)` beside the other one-channel plots; `step_sys.linearize(…)` and
    `step_sys.transfer_function(…)` as on a continuous system; `diagram.add_subsystem(sys)`
    without an id (the operators' default, returned); `StepPlanningProblem.replace`; a scalar
    `u` on a one-input `compute_rollout`; a sampled `u` off the run's grid refused naming
    `n_samples=` and `dt=`; `verbose=False` on the LQR, RRT, RRT* and MPPI planners' `solve`
    (the record printed), `callback` a list of callables on trajectory optimization and
    `Optimizer.solve`, `params=` on value iteration's `solve_steps`, `show=` on
    `plot_signal` and the learning curves; `sensitivity(plant, controller, …)` (and its three
    siblings) positional, the plant read on `bode`'s channel with `of=` / `wrt=` (a plant of
    several channels was refused); on the facades: `LTISystem`, `LinearizedSystem`,
    `MechanicalSystem`, `step_info` and `poles` on the root, `Margins` and `StepInfo` on
    `minilink.analysis`, the shaping functions on `minilink.core`.
  - *The planning plots take one keyword set and return a `PlotResult`* (the maintainer's
    ruling 16): `sol.plot_cost_to_go(*, x_axis=0, y_axis=1, anchor=None, bounds=None,
    grid_shape=(51, 51), t=0.0, jmax=None, is_3d=False, trajectory=None, title=None, ax=None,
    renderer="matplotlib", show=True)` and `sol.plot_control_law` alike (with `u_axis=`, no
    `jmax=`), written out instead of `**kwargs`, on `PlanningSolution` and every planner (a thin
    call to the latest solution; value iteration keeps `field=`), a grid planner's table drawn
    on its own nodes. Value iteration's and the tabular learner's plots returned `(fig, ax)`
    and the comparison's `(fig, axes)`: every planning plot returns a `PlotResult`, read
    `.figure` and `.axes` (`fig, ax = planner.plot_cost_to_go()` →
    `result = planner.plot_cost_to_go(); result.axes`). `jmax=None` is now the failure cost on
    a grid planner, as its default was; `jmax=np.inf` gives Matplotlib's own scale (it was
    `jmax=None`). Keywords outside the set (`cmap=`, `vmin=`, `overlay_labels=`) are the grid's
    and the controller's own plots'. The tutorial's call table follows.
  - *A grid planner's tables, one name.* `DynamicProgrammingResult` → `ValueTables` (it holds
    `J` and `pi`), read as `solution.record.tables`; `planner.result` → `planner.tables` on
    value iteration and on the tabular learner, one meaning on both, the latest solution's
    (`None` before a solve). The tabular learner's `result` was the table as it stands: that is
    `planner.greedy_tables` (`J = min Q`, `pi = argmin Q` of the live `planner.Q`).
  - *A record's cost, named by its quantity.* RRT's (and RRT*'s) `record.cost` →
    `record.path_cost`; the tabular and the neural learners' `record.cost` →
    `record.episode_cost` (the mean episode cost at the end of training). The objective at
    the solution keeps `cost`: the `Optimizer`'s result, trajectory optimization's and MPPI's
    records.
  - *An evaluation's two costs.* `Evaluation.cost` → `raw_cost`: the run's discounted integral
    and terminal cost, before the failure charge; `J`, the score every statistic reads, is
    unchanged (the raw cost plus the failure cost of a failed or missed run). Its one-run
    `str` reads `raw cost = …`.
  - *Every record times its solve.* `solve_time_s` on every planner's record, as trajectory
    optimization's already had: value iteration's (`solve` and `solve_steps`), the LQR's
    Riccati record, RRT's and RRT*'s, MPPI's, the tabular and the neural learners'; the wall
    time of the user's verb without the evidence's scoring, `None` on a compiled face's solve.
    Each record's one-line `str` ends with "solved in … s". The neural learner's
    `train_time_s` stays beside it: it counts every training of the planner, its collections
    and updates only.
  - *The RL planner speaks minilink's words* (the maintainer's ruling 18):
    `ReinforcementLearningPlanner(n_envs=)` → `n_rollouts=` (the rollouts simulated in
    parallel, as MPPI says), its `n_steps=` → `n_steps_per_update=` (each rollout's control
    periods per collection, the data of one update; `n_steps` keeps one meaning, a horizon's
    transitions), read back as `planner.n_rollouts` and `planner.n_steps_per_update`.

## 3. Maturity (TRL)

Readiness levels are an internal maturity scale for planning and review — not
a release process by themselves.

| Level | Name | Description |
| --- | --- | --- |
| **TRL 1** | Agent MVP | Initial code exists and works |
| **TRL 2** | User-check MVP | User performs a high-level functional review |
| **TRL 3** | Architecture Validated | High-level architecture is approved |
| **TRL 4** | Integration Proposed | Final integration/refactor is proposed |
| **TRL 5** | Integration Validated | User approves main-codebase integration |
| **TRL 6** | Automated Tests Pass | Final pytest coverage exists and passes |
| **TRL 7** | Details Validated | Naming and implementation details are approved |
| **TRL 8** | Demo Released | Demo script is created and validated |
| **TRL 9** | Mission Complete | Tests, demo, and user approval are all complete |

Step ids (`S29`, `P4`, `T2`, …) are the rows of
[docs/plans/TODO.md](../plans/TODO.md).

| Area | Lane | TRL | State | Next |
| --- | --- | --- | --- | --- |
| Core + diagrams | teaching | 7 | Public API and diagram API stable; compile-vs-reference parity tested; wrong-shape `f` / `h` fails at `compile()`, and so does an `h`-computed output that moves with an input it does not declare (a port function is not probed, KNOWN_ISSUES); `connect` refuses a rewire (`disconnect` first; the v0.3 run, step 7); the modules by job (the operators, the closed loop, the roles, the port graph; the discrete-time pieces in `core/discrete/`, the `System` shortcuts in `core/facades/`); three owners of a start (the block, the problem, the evaluator's tree), a block's state size fixed once built; `System(state_dim, *, input_dim=, output_dim=)` on every system kind, a static one feeding its input through; `h` the `"y"` port on every block and evaluator. | Automation by convention, the rest (P10: plots and animation by role). |
| Compile (`core/compile/`) | teaching (frozen subset) | 4 | Integrated; the frozen evaluator subset is named in DESIGN §5; one evaluator class per system kind on either backend, one body per method compiled on JAX; the step model (`ev.step` / `ev.rollout`, `solver=`, `substeps=`, `hold=`) under every sampled tool, `discretize` on it (S75). Speed lives in batches (1000 rollouts × 1000 RK4 steps in 0.12–0.16 s on a cloud CPU); `rollout_batch` takes a `params` family; float64 on JAX by default. Step evaluators that trace and batch, one signal pass per tick, the sampled plan; the step problem answers its own equations (`transition`, `terminal`, `run`), on the internal `StepModel` (`planning/step_model.py`). One automatic-backend rule (`build_on_auto`, falling back on a missing JAX or a `NotTraceableError` only); the batch claim gated in CI, a parameter family's price measured (about 4–6× the plain batch on a one-joint plant, none where `H(q)` varies) and accepted, the catalog's readability first. | Diffrax (S27, v0.9). |
| Simulation | teaching | 7 | Mature; fixed 10 001-point default grid; `verbose` names unified; one automatic step from the blocks' physics (the linear blocks' eigenvalues, the catalog's `fastest_mode()`), a held block read at each RK4 stage like any block (first order at its switches); `scipy_stiff` at `scipy`'s tolerances with the Jacobian on both backends. | Two points of one owner per rule waiting for the maintainer's go (S65); the rest of the catalog declaring its `τ` (TODO §2). |
| Dynamics (abstraction + catalog) | teaching | 7 | Plants QA'd; `MechanicalSystem` / `Manipulator`; UR5 ABA/RNEA; **every catalog plant compiles on both backends** (contract test); four-rung vehicle ladder; UdeS racecar (kinematic, dynamic, 3-D); every plant states its own bounds, the mechanical bases invent none. | The catalog clean-up (K8, v0.4, after the demo rebuild); one mechanical family (S32; postponed until the maintainer reviews the mechanical hierarchy, TODO §2); catalog hygiene (S64) and the general serial chain (C6), v0.5. |
| Control | teaching | 6 | Linear, LQR (infinite horizon, finite horizon, along a trajectory), pole placement, reference scaling (`N`), `P` / `PI` / `PD` / `PID` and the rate-measured `Impedance` / `IntegralImpedance` as one family (what is measured decides the derivative; limits with infinite defaults, anti-windup once a limit is declared; the shared pieces three pure functions; the joint and task impedance laws on the same error rule), model-based SMC and computed torque, robotic impedance and kinematic laws, neural policy block; the geometric path-tracking laws `PurePursuit`, `Stanley` and `LineOfSight` on a `core/geometry` path, with the heading loop `HeadingController` (forward driving; unit-tested and JAX-traced, three racecar demos and their comparison, not yet in a course); a controller is a plain `System`; MPC's teaching name `ModelPredictiveController` on `minilink.control` (G3). | The robotic PID wrappers (C2, v0.5); the path-tracking laws on a `Track` (A3). |
| Analysis | teaching | 6 | Jacobians, linearize, structural, equilibria, modal; one-channel Bode with grid-free margins, pole-zero, root locus, Nyquist, the linear step response on matplotlib and plotly; the sensitivity functions `S` / `T` / `PS` / `CS`. Every verb takes any `System`, the linear model as is, the rest linearized at the operating point; a step or sampled system in `z`, the root locus and the step response included, and the four linear plots (`plot_bode`, `plot_pzmap`, `plot_root_locus`, `plot_step_response`) on continuous, step and sampled systems alike. | Nichols and overlays later; Bode, poles and margins traceable for sensitivity to params (idea to review, S73). |
| Blocks | teaching | 5 | Routing, nonlinear, filters, sources (with `Sine`; `NoiseSource`, sampled noise of any law, `WhiteNoise` its subclass), TF, `MLP`, `UnitDelay`. | `Ramp` / `Chirp` / `Delay` / `Switch` (C3, v0.5). |
| Planning / policy synthesis (DP) | teaching (GRO860) | 6 | Grid + value iteration (`method="loop"`, or the table on `numpy` / `jax`), lookup controller, policy evaluation (`evaluate_policy`), `LQRPlanner`; every planner returns a `PlanningSolution` (the controller as handed out, its cost-to-go a `Field`: a `GridField` for value iteration, a `QuadraticField` for LQR); value iteration's two horizons (an infinite one to convergence; a finite one with a law per stage and `J(x, t)`) and the receding `solve_steps`, LQR's finite law past the horizon, discounted LQR; the evidence on the planner's own `dt`; `compare()` reads solutions side by side, and `compare(problem, …, dt=)` scores them on the problem; an inadmissible pair priced by the problem's cost bound; `dp.py` is the textbook reference of the style; the grid reads the problem's step model. | The compiled faces of LQR and value iteration (v0.4's phase 7, [plans-as-policies.md](../plans/plans-as-policies.md)); policy iteration (S51), v0.5. |
| Planning / trajopt | teaching (GRO860) | 5 | Collocation, shooting, multiple shooting; live plot; `success` means defects satisfied to `feasibility_tol`; float64 by default on JAX. | Harden SciPy/Ipopt before TRL 6. |
| Optimization | teaching (via trajopt) | 5 | `MathematicalProgram` + `Optimizer`, SciPy/Ipopt; the method table (`gd`, `adam`, `l-bfgs`, `method="auto"`) with a first-order backend, `z0` optional (S74); the generic parametric program on the backend table, the stochastic program its case; a native fixed-step solver with `n_iterations=K` as a proof of concept. | Harden SciPy/Ipopt before TRL 6; the friction-aware MPC example from a port (S76's rest, TODO §1.7); native solvers beyond the proof of concept in v0.5 ([native-optimizers.md](../plans/native-optimizers.md)). |
| Interfaces / RL bridge | research lane (bridge) | 4 | `Sys2Gym` + `SB3Controller`; the env step is one compiled RK4 call. | Keep for external interop; courses solve with the native planner. |
| Analysis / Lyapunov certificates | provisional (research) | 4 | `region_of_attraction` → `LyapunovCertificate` (`V` a `QuadraticField`, its `region` an `EllipsoidSet`, sampled level, `verify`, `plot`, `contains`); demo and showcase §11. | Cohort validation; `V_dot` and the level search on `V.gradient` (A1's rest); SOS and discrete time later. |
| Planning / RL planner | teaching (GRO860) | 6 | `ReinforcementLearningPlanner` (REINFORCE, actor-critic, PPO, SAC in pure JAX), `TabularLearningPlanner` (Q-learning, SARSA, Monte Carlo control), the stochastic `PlanningProblem` (a law as its start, laws in `params`, by its parts), `evaluate` and its `Evaluation`; the discount from the problem, `gamma=` a training override; `NeuralPolicy` as `planner.actor`, the critic as the solution's `CriticField`; training on a native step problem too; on the root prelude; canonical demos in `examples/demos/rl/`, chapter 11 and the native teaching notebooks. | SAC actor step (S50); deep Q-learning (S52); automatic reward scaling ([rl-reward-scaling.md](../plans/rl-reward-scaling.md)). |
| Planning / evaluation (the path to the cost) | provisional until a course runs it | 4 | `problem.compile(controller, dt)` → the problem evaluator; `evaluate` (one run or `n_trials` draws of the continuous loop, integrated to `tf`, each trial realizing the loop's random blocks) and `compare(problem, …)` its facades, `evaluate_trajectory` for a held run; `jax.grad` reaches every gain, plant parameter, cost weight and start; `pe.program(wrt=)` to the `Optimizer`, `pe.live_plot(wrt)` its progress figure; the closing PID example (S77) by hand, by the JAX pipeline and by the native optimizers; the draws by path on both backends (`realize`), disturbances as source blocks, the problem stochastic by its parts, the sampled loop, the judge on the step problem's own `run`, a step problem realizing from its own tree with `t0` its tick offset. The maintainer's review held. | Identification's data cost (C4, v0.5). |
| Planning / search (RRT) | teaching (G3) | 5 | RRT / RRT*; `RRTPlanner(problem)` works from the problem's input box alone (bang-bang extender; an unbounded `U` refused with the line to write); returns a `PlanningSolution`; the live figure a `LiveSearchPlot` hook of `solve(callback=)`. | RRT-Connect later. |
| Planning / path integral (MPPI) | provisional (research), on `minilink.planning` | 5 | `MPPIPlanner` / `MPPIRecord` on the compiled planner face (`n_z = N m + 1`, the method's numbers and the seed in `params`), the MPC in every form on it, the closed-loop fans of sampled futures redrawn from a run on each solve's own key (`mpc_sampled_futures`, the animation overlay, `plot_sampled_futures`) and its account on the MPC's `verbose` line, `test_mppi.py`, two demos (`mppi_pendulum`, `mppi_racecar`: the kinematic racecar laps the UdeS circuit at a lower closed-loop cost than the collocation MPC); deterministic only. Contract: DESIGN §6; the rest is TODO's S72 row. | The stochastic branch (S72's rest, MP-3); the teaching decision (MP-6, V3). |
| Geometry / spatial | provisional (teaching) | 4 | SDF + `Scene` / fields / bodies under `planning/spatial/`, JAX twins tested; the shapes and the paths in `core/geometry/`, their one home; the planning facade's spatial names are provisional teaching names (§2). | Package move and course catalog (A3); bicubic SDF and CBF filter (research). |
| Graphics / animation | teaching | 5 | Frame-keyed `tf` / geometry / overlays; four renderers; auto-fit camera. | Camera and plot defaults from roles (P10); constructor-derived camera hints (S43); glyph/solid rename (S30, v0.9). |
| Hybrid / step / MPC | provisional (research); MPC's teaching name `ModelPredictiveController` (G3) | 4 | `StepSystem`; the sampled loop: `SampledSystem` (`sys % dt`, `discretize`), `SampledDiagramSystem` with periods on blocks, its compiled plan (divisors, held outputs, the time each block reads), `>>` / `+` / `@` on step operands, `compute_rollout` and `compute_trajectory` agreeing at the ticks, the host clock runner over the same blocks, `problem.compile(pid % dt)` differentiable; the loop's grammar: `%` the computer, the plant a loop closes on integrated at the continuous simulator's step (`dt=` at the runs, `substeps=` at the compiles), `linearize` in `z` on a step or sampled system; the MPC on the planner's compiled face (`planning/trajectory_optimization/mpc/`), one architecture: the box `solve`, `command`, `memory`, a visible `delay` when asked, inputs as bindings, `PlanFollower` for dual rate, its closed loop one scan on JAX on a native fixed-iteration planner and tuned by gradient. | Cohort validation. |
| Realtime simulation | provisional | 2 | `RealtimeSimulator` + pygame I/O. | Architectural review (v1.0); later the human-demonstration recorder of the behavior-cloning pipeline (after v1.0). |
| Estimation | teaching (GRO501) | 6 | `StateObserver` (innovation form, operating point on the block, hat labels), `luenberger` and `kalman` on arrays, a linear model or the plant (the duals of `place` and `lqr`); `lqg` returns the observer + gain box; an observer is refused as a controller; `closed_loop(gain, plant, observer=obs)` wires the LQG loop by role (AC-4). | the trajectory observer and the finite-horizon filter, the EKF, the estimation problem and its tools (v0.9, [the plan](../plans/estimation-and-state-feedback.md)); the sampled filters on the core's sampled loop. |
| Identification | planned | 1 | No library module (the `identification/` stub deleted; its home `minilink.learning` when C4 lands); the `params_gradient` demo fits plant parameters by `Optimizer` (L-BFGS) on the equation error. | V1's program export with a data cost (C4, v0.5). |
| C export (`experimental/c_export`) | research | 1 | JAX→C transpiler; two demos; flagship smoke in the JAX regression job. | Keep isolated; repo-only. |
| Experimental tier (`experimental/symbolic`, `experimental/engines`) | research | 1 | Not on the teaching path nor the API site. | Keep isolated; repo-only. |
| External multibody leaf (MJX) | research | 0 | Not started. | Spike later. |
| Pyro 2.0 overall | v0.5 (C1) | 3 | Catalog + core + search/DP/trajopt done; many demos unported. | Remaining rows in [pyro-port-remaining.md](../plans/pyro-port-remaining.md). |

## 4. Course objectives

Three courses drive the release contract: GRO860 (§4.1, v0.1, running now),
GRO501 (§4.2, v0.2) and GMC714 (§4.3, v0.3). A course is
"end to end" when every topic row is green on the teaching surface — in Colab
(git-clone cell) and in the conda env — and the cross-cutting gates hold.

### 4.1 v0.1 — GRO860 end to end

| Topic | Surface | Material | Gate |
| --- | --- | --- | --- |
| Value iteration / DP | `PlanningProblem`, `StateSpaceGrid`, `DynamicProgrammingPlanner`, `LookupPolicy`, `plot_cost_to_go` / `plot_control_law` | `pendulum_value_iteration`, `pendulum_value_iteration_vs_lqr`, `demos/value_iteration/` | `tf` reads the problem's `t0 + horizon`; `success` reports convergence; notebooks wire with `vi_ctl @ plant` |
| LQR + linearization | `linearize`, `lqr` (on arrays, a linear model or the plant), `plot_control_law` | `basics/02_analyze_and_control`, `demos/control/` | — (green today) |
| Trajectory optimization | `PlanningProblem`, `TrajectoryOptimizationPlanner` (`direct_collocation`, `multiple_shooting`, `single_shooting`), `QuadraticCost` | `basics/03_plan_and_sample`, `advanced/02_optimal_control`, `demos/trajopt/` | float64 by default on JAX; `success` means defects satisfied; canonical problems succeed with default optimizer |
| Reinforcement Learning (tabular on the grid, neural in native JAX) | `TabularLearningPlanner` (Q-learning, SARSA, Monte Carlo control; `EpsilonGreedy`, `UCB`), `PlanningProblem` with laws, `ReinforcementLearningPlanner` (REINFORCE / actor-critic / PPO / SAC), `NeuralPolicy`, `evaluate` | `advanced/04_reinforcement_learning.ipynb`, `pendulum_value_iteration_vs_lqr_vs_rl.ipynb`, `drone_ppo.ipynb`, `gymnasium_interface.ipynb`, `showcase_from_rl_to_bode.ipynb`, `demos/rl/` (pendulum, cart-pole, drone, rocket) | pure-JAX training on compiled plant (pendulum swing-up, 120k steps in 12.5 s on CPU, 0% failure over 50 trials); neural policy closed loop linearizes, simulates, and plots Bode; those names (plus `Evaluation`, `Gaussian` / `Uniform`) are on the root prelude. Notebooks solve with the native planner; Gymnasium stays taught as the domain standard and `Sys2Gym` + SB3 stay as the external bridge |

**Cross-cutting gates**

1. Wrong-shape `f` / `h` / port computes fail at `compile()` on both backends;
   a forgotten `super().__init__()` gives a named error; the README custom
   plant composes with `@`.
2. Default `compute_trajectory` returns a fixed output count (10 001, the fine
   reporting grid) and never selects a solver from the point count.
3. Every GRO860 notebook and every `examples/tutorial/`, `examples/teaching/`, and `examples/demos/`
   file imports only through the teaching surface (CI test).
4. The Basic tier (NumPy + SciPy + Matplotlib, nothing else) runs
   sim / plot / phase plane / animate / linearize / LQR / VI in a clean
   environment (CI test).
5. The Colab cell (git clone + path) and the conda environment from
   `environment.yml` both run every GRO860 notebook top to bottom.
6. `ruff` + `pytest` + notebook smoke green; nightly full demo sweep green.
7. No name a GRO860 notebook imports changes before v1.0. **Paused**: between
   terms the teaching design is unfrozen; a rename migrates every
   course notebook that uses the name in the same commit, with no alias. The freeze
   resumes when the next term's notebooks are pinned.
8. `pip install minilink` installs the teaching surface, published from a GitHub tag
   (Trusted Publishing in `.github/workflows/publish.yml`, the version read from the
   tag); extras (`full`, `dev`) match `pyproject.toml`. Conda remains
   the Full local stack (JAX, Ipopt, notebooks).

Out of the v0.1 checklist by decision: MPC/hybrid (provisional, lesson keeps
shipping), estimation, identification, frequency-domain tools (gated by §4.2).

### 4.2 v0.2 — GRO501 end to end

The classical-control course (*Systèmes asservis*, BSc Génie Robotique):
an APP problem on the UdeS-Racecar. APP2 is multi-physics modelling of the
DC-motor propulsion plus SISO speed and position loops designed to time- and
frequency-domain specifications and implemented as difference equations on an
Arduino; APP4 is the bicycle-model MIMO plant with LQR, nested loops, pole
placement and a Kalman observer. Student guide: `GRO501_Guide.pdf`
(maintainer's copy); references Dorf & Bishop and Åström & Murray.

| Topic | Surface | Status | Gate |
| --- | --- | --- | --- |
| Multi-physics modelling — nonlinear `f`/`h`, block diagram, linearize, `H(s)` | custom `DynamicSystem`, `plot_diagram`, `linearize`, `transfer_function` | green | a longitudinal-vehicle plant in the catalog (met: `LongitudinalTorqueCar` on `minilink.catalog`; a DC-motor plant is off the critical path, S71); pole/zero cancellation by default; order reduction by fast-mode neglect is out of v0.2 |
| Closed-loop analysis — poles, root locus, Bode, margins, step specs | `pzmap`, `root_locus`, `bode`, `margins`, `step_info`, `P` / `PI` / `PD` / `PID` | green | every compensator form reports the poles and zeros of the hand calculation, and margins are found wherever the crossover sits |
| Design to specification — rise time, overshoot, final error, phase margin | `PI`, `PD`, `PID`, `Lead`, `step_info`, `margins` (`Lag` removed) | green | the Table 2 specs of the guide are checkable in one notebook |
| Loop-shaping specs — disturbance and measurement-noise sensitivity in dB at a frequency | `sensitivity`, `complementary_sensitivity`, `load_sensitivity`, `noise_sensitivity` | green | the `sensitivity_functions` teaching notebook |
| Digital implementation — difference equations on the Arduino | `discretize` (Euler / RK4 step models) | green | the law is designed in continuous time, discretized, and the sampled loop validated by simulation; no z-domain analysis family |
| State-space MIMO — bicycle model, controllability at every nominal speed | `KinematicBicycle`, `controllability`, `observability` | green | — |
| Optimal control — LQR on the guide's cost, closed-loop poles, nonlinear check | `lqr(plant, Q, R, x_bar=)`, `StateFeedback` | green | — |
| Pole placement — `K_sta` for a prescribed pole set | `place` (on a linear model or the plant; `place_gain` on arrays) | green | — |
| Nested loops — inner speed loop, outer position loop | `@` composition | green for error-driven compensators (`PID @ (PID @ G)`); an outer `r, y` or state controller is refused today | P10 (TODO §3) nests the inner loop as the outer loop's `plant`; stays green with the observer in the loop |
| State estimation — Luenberger observer and Kalman filter | `StateObserver`, `luenberger`, `kalman`, `lqg` | green | the observer closes the loop as a standard diagram block: by role with `closed_loop(gain, plant, observer=obs)`, in the `lqg` box, or by hand |
| Reference scaling — the `N` matrix giving `y = r` at steady state | `OutputReference(K, N)` | green | the student writes `N = -(C (A - B K)⁻¹ B)⁻¹`; the block takes the output reference |

**Cross-cutting gates**

1. Every topic row above is green, with a demo or a notebook.
2. No APP notebook in the repository: the students write the APP work themselves. The demos and notebooks of gate 1 run Colab-first on the Basic tier
   (NumPy + SciPy + Matplotlib) — no JAX on the GRO501 path.
3. Every tool agrees with the hand calculation for the guide's §9 exercises;
   the closed-loop analysis exercises (§9.6, §9.7) are the acceptance test.
4. Student-facing GRO501 material imports only through the teaching surface.

Out of the GRO501 checklist by decision: hardware, ROS, and the Arduino
firmware itself (the course owns those); system identification from logged
runs.

### 4.3 v0.3 — GMC714 end to end

The robotics course (*Robotique : modélisation et commande*), winter term, week 1 on
January 9. Syllabus: [the course page](https://www.alexandregirard.com/courses/gmc714/) and its
notes (*Notes_Robotique_UdeS*). Its labs run today on pyro, through Colab
notebooks one per homework. **GMC714 needs no new feature.** Its labs
move from pyro to minilink as the maintainer's teaching work, outside this roadmap; the
release's part is that the teaching surface covers every lab, checked the same day:

| Week | Topic (notes chapters) | The lab's numerical part | Surface | Status |
| --- | --- | --- | --- | --- |
| 1 | Phase space, model types (1, 7) | lateral control of a bicycle model, the phase plane of the closed loop | `KinematicBicycle`, `sys.plot_phase_plane` | on the surface |
| 2–3 | Arm kinematics, then dynamics (2–6) | forward and inverse kinematics, Jacobian, manipulability; `M`, `C`, `g` (mostly by hand) | `TwoLinkManipulator`, `ThreeLinkManipulator`, `UR5Manipulator` | on the surface |
| 4 | Arm control I, quasi-static (7, 8) | impedance control of a 3-DoF drilling robot in simulation | `ThreeLinkManipulator`, `Impedance`, `TaskImpedance`, `JointImpedance` | on the surface; the drilling contact is the notebook's |
| 5 | Arm control II, dynamic (9, 20) | computed torque, sliding mode, adaptive control | `ComputedTorque`, `SlidingMode`, `DynamicSystem` | on the surface; the adaptive law built by hand in the lab as a custom `DynamicSystem` with the controller's ports (pyro's `DynamicController`) |
| 6 | Vehicle modelling (11) | longitudinal model with tire slip; the dynamic bicycle, oversteer, linearization | `LongitudinalTorqueCar`, `DynamicBicycle`, `linearize` | on the surface |
| 7 | Vehicle planning (13, 16, 21) | trajectory optimization of a pendulum (`Q`, `R`, the torque limit, `n`, `dt`); RRT then trajectory optimization on a cart-pole | `TrajectoryOptimizationPlanner`, `RRTPlanner`, `QuadraticCost`, `Pendulum`, `CartPole` | on the surface |
| 8 | To come | — | — | — |
| 10–13 | Projects | "a new system or controller integrated into the library" | the `System` contract | on the surface |

Not a week of the syllabus: MPC (its teaching name stable all the same, when v0.4 merges),
robust control as a topic of its own
(one section of week 5, the sliding mode), the draws and Monte Carlo, gradient tuning.

**The gate:** every row above stays on the teaching surface through the v0.3 and v0.4 changes,
so a lab written against the release its notebook pins needs nothing more; and the tools the labs use carry
one public, stable teaching name when v0.4 merges (RRT, MPC, trajectory optimization, computed
torque, sliding mode, the task- and joint-space impedance laws, task-space position control
with a nullspace posture, `Manipulator`), frozen for the term (TODO §2, G3). The lab notebooks themselves are the
maintainer's teaching design (AGENTS), written when he chooses; agents draft on request.
The course material on `main` runs on `0.3.0` until v0.4 merges.

## 5. The path to v1.0

**How the rungs work:**
- Each rung names its open steps, one line each. The steps themselves, with files, order and
  "done when", are the rows of [docs/plans/TODO.md](../plans/TODO.md).
- A rung is done when every step in it has landed or the maintainer has dropped it.
- Landed work leaves this section; its history is in git.
- Wave D's rows (the T rows, D1–D3, S43, S67–S69, A4, P4) are named in TODO only, beyond §5.2's
  summary.

**The order is the commitment; the course calendar is secondary** **No course calendar
shapes a step.**
- The class material on `main` runs on the latest tag, and the releases carry no dates.
- GRO860 runs in the fall term and GMC714 in the winter term.
- Two windows the plan cites:
  - *between terms*, late December 2026 to early January 2027 (V11 and the examples moves);
  - *between cohorts*, between GRO860 cohorts, whose material reads the Monte Carlo evaluator
    and the RL numbers.
- Neither window delays a step: a step that would move a running course's numbers ships behind
  the pinned release the course runs on (V6) rather than waiting.

**The shape the rungs build.** The spine is CONSTITUTION §1.1: a `System`, compiled,
rolled out to a `Trajectory`, scored by a cost, lowered to a `MathematicalProgram`, solved by the
`Optimizer`. There are two step paths on it:
- the continuous one (`Simulator`, the differentiable evaluator);
- the sampled one (the step model and the sampled loop, DESIGN §5).

The toolbox chapters on the spine are:
- classical and modern control;
- nonlinear control;
- trajectory optimization and MPC;
- reinforcement learning;
- identification and imitation;
- estimation.

**A direction to promote: the fair comparison across control approaches** (the maintainer,
2026-11-09). One `PlanningProblem` handed to every planner (dynamic programming, LQR, tree
search, trajectory optimization, MPC, reinforcement learning), each answer a `PlanningSolution`
whose controller is scored by the one judge, `evaluate` and `compare(problem, ...)`: the same
model, cost, sets, start law and disturbances for every approach. This is the contribution the
toolbox can make that the specialised tools do not, and it ranks above new planners:
- the `compare` path is hardened first (its maturity is TRL 4 in §3, "Planning / evaluation"),
  and the README's own comparison example is fixed (KNOWN_ISSUES, "The README's examples");
- a comparison is fair when it also reports what each approach was given and what it spent:
  the model access (equations, gradients, samples only), the compute time, the samples or
  iterations, and the numbers a user tuned;
- where the idea is stated (CONSTITUTION §1, the README's lead) is an open question for the
  maintainer: [QUESTIONS.md](QUESTIONS.md), "Where is the fair comparison stated?".

The releases: **v0.3 built the complete core** (§5.3), **v0.4 makes its surface clean**
(§5.4), **v0.5 adds the features built on it** (§5.5), v0.9 decides what can still break a name,
and v1.0 freezes when the API is mature.

| Rung | What | Rows |
| --- | --- | --- |
| v0.3 | The continuous core, two invariants and seven core pillars (shipped, `0.3.0`) | git history |
| v0.4 | The clean surface: one vocabulary, the core API proposals, the minimal surface, the core's leftovers; the clean-up lane | TODO §1, §2 |
| v0.5 | Features that read the core and never change it | TODO §4 |
| v0.9 | What could still break a name; the trust contract; the syllabus holes; the reference site | TODO §5 |
| v1.0 | A colleague can build a course on it | TODO §5 |

### 5.1 v0.1 — close-out

The GRO860 path is green (§4.1) and `0.1.1` is published; nothing is open.

### 5.2 v0.2 — shipped; the carry-over

`0.2.0` was tagged on 2026-10-17 with every §4.2 row green. The steps below were planned for
v0.2 and not needed for the tag; they carried into the later releases, each in the slot the order of
TODO §1 to §4 gives it, wave D standing. The core's rows among them (A5, RN-2,
AC-7, AC-8, S65, S66) joined v0.3's core (TODO §1).

**Wave A — the textbook objects finish speaking.**
- **A5** The later nouns, each with its first consumer: `log_prob`, sets and distributions
  over parameter dictionaries, `UnionSet`, `PlanningProblem.hamiltonian()` only if a course
  teaches Pontryagin (the library sets reading `params` and `Set.as_cost`, the v0.3 core).
- **RN-2**, the draws: distributions read `params`,
  `Gaussian(cov=)`, every `sample` takes a key (the v0.3 core).

**Wave B — GRO501 end to end** (§4.2; its steps landed).
- **P10** Automation by convention (TODO §3): standard port names drive the wiring, three reserved names (`plant`,
  `controller`, `observer`) are the roles every tool reads, and the cost is scored on the
  plant. Its composition steps landed in this wave; the rest: plots and
  animation by role (the plotting lane), the cost's draws (AC-7) and the sampled loop (AC-8),
  the v0.3 core; TODO §1 and §3, the plan's §7.

**Wave D — the code reads like the textbook, and the demos like the API.** Standing work,
behaviour-preserving, one module per step.
- The textbook pass landed; its behaviour and API
  leftovers are TODO §3's T rows (T1 and T2 in the core, TODO §1.1).
- **D1** Demos and notebooks to the minimal rule (RULES 6.1 / 6.10 / 6.11). **[ask per
  file]**
- **D2** Consolidation picks still open from the 2026-09-05 inventory.
- **D3** Hardening rows carried over (research lane).
- **S65** One owner per rule (the automatic `dt`, the backend fallback, the input-hold
  model, the Monte Carlo defaults), the core; **S66** wiring mistakes fail at wiring time
  (the go given), the core's hardening; **S67** one plot vocabulary; **S68** contracts as tests.

### 5.3 v0.3 — shipped

`0.3.0` was tagged on 2026-10-30 (the merge of `dev` into `main`, PR #131) and published on
PyPI: every workstream's main rows, the acceptance scripts as tests and the budget as a
report script (retired with the benchmark studies in v0.4); the maintainer's review held and its six release points ruled. The
core's small leftovers and the clean-up lane carry into v0.4 (§5.4).

### 5.4 v0.4 — the clean surface

v0.4 makes the surface of the v0.3 core one thing to learn: one vocabulary, one shape per kind
of object, the demos, notebooks, tests and CI redrafted on it, and the core's leftovers closed.
It is the release where public names break (no alias, every call site and notebook in the same
change), so that v0.5's features land on names that hold. Ruled 2026-11-04: v0.4 is this
release, ROADMAP's former v0.4 features become v0.5. The rows are TODO §1 and §2, in the order
of TODO's "Next, in order": what changes a public name or a behaviour first, the surface
redrafted once on the settled names, the numbers re-pinned once, additive work last.

**Where it stands (2026-11-10).** The first four phases of that order are on `dev`, merged by
PRs #135 and #136 on 2026-11-08, then the API naming sweep (PR #138, 2026-11-08), the
maintainer's PID autotuning demos with the evaluator's live plot (2026-11-09) and the API
review's defects (2026-11-10). Their contracts are DESIGN's; every name they changed is in
§2's breaking changes. Every feature branch is merged: `dev` holds all the work.
- *Before them, on `dev` after the tag:* the tutorials in three layers (basics, core,
  advanced); the vocabulary of RULES 2.7 and one last answer per object (RULES 2.4), held by
  `test_vocabulary.py` and `test_last_answer.py`; every planner's `params` tree and
  `solve(params=)` (the neural RL planner's tree a report); the one parameters model and no cache (DESIGN §4); the nine API surface
  rulings of 2026-11-05; a diagram run recording the plant's input by default (2026-11-06).
- *Phase 1, the planners' math and fields* (plans as policies, its first change): the pure
  cores first in their files, the cost-to-go of value iteration and LQR as fields (`GridField`,
  `QuadraticField`), discounted LQR.
- *Phase 2, value iteration's finite horizon:* a law per stage and `J(x, t)`, the law past the
  horizon for value iteration and LQR, the examples migrated.
- *Phase 3, the core API proposals as ruled:* static systems with the port shortcuts and
  `state_dim` (CA-1), `h` the `"y"` port everywhere (CA-2), the core's types on
  `minilink.core` and two fields on the root (CA-5); the controller is the block and the
  solution's shape; the workflow keywords on `solve`; `problem.compile(controller)`; the
  training budgets; the helper methods as module functions; `refresh()` removed.
- *Phase 4, the core's small leftovers:* the step problem's own tree and tick offset; the RL
  environment on a native step problem; the root locus and the step response in `z`; the
  linear-analysis plots on step and sampled systems; one error rule for the impedance laws;
  one owner per rule (the automatic backend, `scipy_stiff`, the report panels, the print
  text); the batch-rollout claim gated (a parameter family's price accepted as it is, the catalog's
  readability first); the first cut of the minimal surface (two topic notebooks
  that duplicated GRO860's deleted, the benchmarks reduced to the regression gate, the slow
  tests behind `--runslow`).
- *The maintainer's rulings of 2026-11-07 and 2026-11-08, built with them:* the step problem
  answers its own equations; the solution hands out its controller, scored as handed out, on
  the planner's own `dt`; value iteration's modes; the four vocabulary renames; RRT's live plot
  as a hook; the path names from `minilink.core.geometry`.
- *The API naming sweep (TODO §1.11's rulings of 2026-11-08, PR #138):* `n_steps` counts
  transitions and `n_samples` time samples, `renderer=` for the drawing library, the policy
  evaluation a `GridField`, the step problem's `rollout` a `Trajectory`, the records' `n_<thing>`.
- *The PID autotuning demos (the maintainer, 2026-11-09):* `pe.live_plot(wrt)`, a progress
  callback for `Optimizer.solve(callback=)`, and `pid_autotuning_native.py` beside the JAX one.
- *The API review before the demo rebuild* ([api-stabilization.md](../plans/api-stabilization.md),
  2026-11-10): four independent reviewers over the public surface; its defects section (§1)
  landed, its other sections wait for the maintainer.

**Left**, in order (TODO §1, "Next, in order"):
- **Phase 4b, the API settled before the demo rebuild** ([api-stabilization.md](../plans/api-stabilization.md)
  §2 to §5, on the maintainer's go): the names a rule already decides, the additions that break
  nothing, then 24 decisions one at a time, and the planned renames pulled forward (the car
  family, the root prelude, the catalog's words, the mechanical names), so the rebuilt demos are
  written once, on names that hold.
- **Phase 5, the surface redrafted on the settled names** (TODO §1.12 and §2): the
  maintainer's demo rebuild, a minimal set in place of every demo, then the tests rebuilt from
  a clean slate around it; the course notebooks, their content the maintainer's; the
  clean-up lane below. The cart-pole RL demo's budget waits for the rebuild.
- **Phase 6, the release** (TODO §1.10): the student-facing numbers re-pinned once, after the
  maintainer checks the demos on `dev`; the course notebooks on the new names; the merge to
  `main`; the `0.4.0` tag.
- **Phase 7, the compiled faces of LQR and value iteration and the MPC reading the state**
  (plans as policies, its third change; TODO §1.8): additive, so before the tag if ready, else
  a 0.4.x or v0.5 with the RL and RRT faces.

**The v0.3 core's leftovers.** One number per workstream, the same as TODO §1.N; each is small:

| § | Workstream | Left (TODO §1.N) |
| --- | --- | --- |
| 1 | Every object pure, every number a parameter (invariant) | A5's later nouns, each with its first consumer; the sets-as-costs demos (was V1 step 8) |
| 2 | The step algebra first-class (invariant) | two points of one owner per rule (S65) waiting for the maintainer's go: the Monte Carlo defaults, the empty-cache rule |
| 3 | The draws | nothing (the contract is DESIGN §4 and §6) |
| 4 | The seeded stochastic policy | nothing |
| 5 | The sampled loop: a step diagram, two runners | nothing (the realtime runner on the pure blocks is v0.5) |
| 6 | One score | nothing |
| 7 | The parametric program; the stochastic program its seed case | the friction-aware MPC example from a port |
| 8 | The native proof of concept | plans as policies' third change, the compiled faces (phase 7) |
| 9 | Fields | nothing |
| 10 | Hardening and the release | the release (phase 6) |

Beside them, from the clean surface's own rows: the parameters model's internal follow-up
(TODO §1.11: the problem's helpers folded into its held tree; no number moves).

**The clean-up lane** (TODO §2), part of phase 5:
- **One demo per feature** (K4), now the maintainer's end-of-v0.4 demo rebuild.
- **The catalog clean-up** (K8):
  - the car family to three models; the root prelude trimmed to the course set;
  - `minilink.catalog` the flat index of tested plants;
  - the unsupported plants moved to the experimental tier, chosen by rule after the demo
    rebuild (ruled 2026-11-07).
- **The experimental tier out of the CI gate** (K9). Its last items, the unsupported plants'
  tests, leave with the catalog clean-up.
- **GMC714's names** (G3), pinned once at the very end of v0.4, after the demo rebuild (ruled
  2026-11-07); the moves and renames landed.
- **The kinematic car family declares its time constant**, with the catalog clean-up.
- **One mechanical family** (S32), postponed: the maintainer first reviews the mechanical
  hierarchy and the car models' base, with the demo clean-up or later.

**GMC714**: no new feature. The teaching surface already covers every lab (§4.3). The
audit is done (G1).

### 5.5 v0.5 — features on the core

v0.5 adds features built on the finished core, not
intertwined with it; any order, each independent. The rows are TODO §4; a row that reads
a core pillar names it.
- The remaining compiled faces on the plans-as-policies contract
  ([plans-as-policies.md](../plans/plans-as-policies.md) §3): the RL and RRT planners (and
  the LQR and value-iteration faces, if they miss `0.4.0`).
- The realtime runner on the pure blocks: game mode and hardware in the loop on the
  sampled loop's blocks (TODO §4).
- **S72** The path-integral planner (MPPI) on the core's sampled loop and draws
  (TODO §4), a planner under the compiled-face contract (DESIGN §6).
- **C4** Identification as the path's first data consumer: the program export with a
  data-mismatch cost; trajectory generation and experiment design beside it.
- **LQGPlanner** with RN-4 (the core).
- **A3** Workspace geometry, `core/geometry/` ([geometry-module.md](../plans/geometry-module.md)),
  the path-tracking laws on a `Track` (the shapes and paths landed). **[ask — core]** The CBF safety filter after it (unscheduled).
- **C5** RL follow-ups: **S50** SAC actor step, **S51** policy iteration on the grid,
  **S52** deep Q-learning, approximate dynamic programming; the RL heads rebuilt on the seeded
  policy (with the block or here: a placement detail of its row).
- **Native solvers beyond the proof of concept** ([native-optimizers.md](../plans/native-optimizers.md)):
  the tolerance form, implicit differentiation, least squares, the augmented Lagrangian, the
  stochastic category, convergence as stability.
- **C6** General serial chain: one mechanism description (DH or URDF) feeding RNEA / ABA, the
  GMC714 arm topic's modelling rung; design first. **[ask — core]**
- The former wave G rows: **C1** pyro parity and the pyro → minilink migration guide,
  **C2**'s robotic PID wrappers, **C3** the blocks (`Ramp` / `Chirp` / `Delay` / `Switch`),
  **S64** catalog hygiene.
- **S73** Traceable linear analysis, any time, self-contained.

### 5.6 v0.9 — freeze candidate (spring 2027, indicative, set before 2026-10-26)

What v1.0 asks is adoption, not architecture: a colleague builds a course on minilink and
trusts it for three years. An
adoption review found the
engine ready and three things missing around it: a stability promise, the two syllabus
holes every course hits (estimation, discrete time), and material packaged for an
instructor. The discrete-time hole closed with S75 in v0.3 (§5.3); v0.9 closes the stability
promise and the estimation hole, and settles every question that could still break a
public name; v1.0 (§5.7) freezes.

**The foundations that can break names** — decided before the freeze:
- **S27** Diffrax as an optional JAX solve, beside the step formulas S75 kept in
  `core/compile/` (S37 is absorbed by S75). **[ask — core]**
- **S30** The glyph / solid rename; **S44** one posed-geometry hook. **[ask]**
- **V3** The frozen surface: which names the freeze covers. **[ask — public names]**

**The repository's shape** — between terms, after GMC714 and before the freeze:
- **K5** Course material and/or the research projects in their own repositories
  (§6). **[ask — student-facing]**
- **K6** The move, if K5 says so: history kept, the citing course documents updated, the new
  repositories pinned on a minilink release. **[ask — student-facing]**

**The trust contract:**
- **V4** The policy for changing a public name after v1.0: open (QUESTIONS; a one-release
  `DeprecationWarning` shim was declined), no alias meanwhile; a snapshot test pins the
  teaching-surface export list. The pending renames land before the freeze (ruled
  2026-11-03, the vocabulary of RULES 2.7); `JaxMechanicalSystem` is deleted instead. **[ask]**
- **V5** `CHANGELOG.md` for users; every release from v0.9 carries its entry.
- **V6** The Colab cell installs a pinned release (the install story itself landed:
  conda recommended, `pip install minilink` plus one `full` extra, no
  `install.md`).
- **S70** A `MinilinkError` family (wiring, shape, solver) under the plain
  `ValueError` / `TypeError`. **[ask — core]** (It lands before or with the core's S66, which
  raises its types.)
- **V11** A light history: the notebook outputs committed before `nbstripout` leave the
  git history, once, after the v0.3 tag. **[ask]**

**The syllabus holes** (beyond GRO501's P4):
- **Estimation follow-ups** after P4: EKF, time-varying and discrete Kalman; online parameter
  estimators (recursive least squares, gradient laws).
- **C4** Identification: moved to v0.5 (V1's program export with a data cost).
- **S71** The classic first plants (DC motor, tank / thermal process, ball and beam,
  differential drive, 3-D quadrotor); LQI beside the `N` matrix.

**The reference site:**
- **V7** The docs site renders the tutorials, a plant gallery with pictures, and a short
  concept guide.

### 5.7 v1.0 — a colleague can build a course on it (2027, indicative, set before 2026-10-26)

After two cohorts have run on v0.2 and the freeze candidate has held a term:
- **V8** A textbook-validation suite: Dorf / Ogata / Franklin worked examples asserted
  numerically; GRO501 gate 3 is its seed.
- **V9** Course-neutral labs (10–15 in `examples/teaching/topics/`) and a "For
  instructors" page. **[maintainer]**
- **V10** The public face: a teaching quickstart in the README; no internal vocabulary in
  user-facing files; `CONTRIBUTING.md`, issue templates, a code of conduct. **[ask]**
- **V2** Release hygiene: Zenodo archive and DOI, `CITATION.cff`; the JOSS decision.
- **S49** The four Stable-Baselines3 notebooks of GRO860: `drone_ppo_sb3` and
  `pendulum_value_iteration_vs_lqr_vs_ppo_sb3` retire; `particle_gymnasium_sb3` and
  `gymnasium_interface` are decided then. **[ask]**
- The realtime band's architectural review (`RealtimeSimulator`, the pygame I/O). **[ask —
  core]**
- The freeze: the V3 surface under the V4 policy; the GRO860 name freeze becomes the
  general one.

**After v1.0 (1.x features, additive by construction):** **S36** iLQR from parts (research lane); a
**behavior-cloning pipeline** end to end:
a game mode on `RealtimeSimulator` that records human demonstrations (keyboard or joystick
driving the plant live) as trajectory datasets, a planner that fits a policy to them
(imitation beside the RL planner, the neural policy block as its output), and the cloned
controller closed on the plant and compared against the demonstrator **[ask]**; diffusion
policy after it. Its design notes are its TODO row.

**Simplify and consolidate — a standing principle, not a rung.** The repo must stay
manageable by one maintainer, so every rung carries consolidation work, and the target is
*maintenance cost*, never features: text edited twice when code changes, dead API,
boilerplate a flag would replace, twins one `xp` body covers, research scenarios inside the
teaching tree, plan docs already self-marked complete. A deliberate ladder of
implementations (the DP planner's three backends) is not duplication. The maintainer picks
each item; agents execute only what was picked.

## 6. Decisions

The open design questions live in [QUESTIONS.md](QUESTIONS.md), one entry each; details and
the rows waiting for the maintainer's go are in TODO. A decision leaves it when it is taken: a ruling the maintainer takes updates the doc it
changes (DESIGN, ROADMAP, TODO or the plan doc it shapes) in the same change, and the commit
message carries its why.

## 7. Out of scope

By decision — see [CONSTITUTION.md](CONSTITUTION.md) §1.4:

Full Simulink parity (GUI, DAE, arbitrary multi-clock scheduling,
event-driven switching as a framework feature); becoming a multibody/contact
OS or a batched RL physics engine.

What *is* in scope but subsidiary: the sampled loop (`StepSystem`, `SampledSystem`,
`SampledDiagramSystem` and the host clock runner), so discrete control laws can close the loop on
continuous plants; it is provisional, and MPC is its exemplar in the tutorials (RULES 3.7).
