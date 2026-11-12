# Open design questions for the maintainer

The design questions that wait for the maintainer: the architecture, the API and its naming, the
vision, and what affects performance. Details, versions, process, tests and demos are not here:
they sit in their workboard rows, and the rows that wait for the maintainer's go are listed at
the head of [TODO.md](../plans/TODO.md#rows-waiting-for-the-maintainers-go). Known defects are in
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

A ruling the maintainer takes updates the doc it changes (DESIGN, ROADMAP, TODO or the plan
doc it shapes) in the same change; the commit message carries its why. It then leaves this
file. The core's architecture is of record: the v0.3 rulings and the v0.4 rulings of
2026-11-04 to 2026-11-11 live in DESIGN and RULES. Open here: four small names the API
settlement (PR #139) left, four questions on the car ladder, two on the architecture, and one on the vision.

## The API

### Four small names the API settlement left
- **Context:** the API settlement before the demo rebuild (PR #139, merged 2026-11-11) logged
  four names it did not touch, because no ruling listed them; each is a public name, so each
  waits for you. They are small and independent; the rebuilt demos would meet the
  last two.
  1. `SampledDiagramEvaluator.block_input(path, x, u, k)`, a public method of the compiled
     evaluator (DESIGN §4, one test, the MPC drawing helper): it reads a subsystem's input,
     where ruling 1 made "subsystem" the word for a system placed in a diagram.
  2. `TrajectoryOptimizationPlanner.transcription_options()`, the verbose print's line: it
     names the option records the settlement deleted (the transcription now takes
     `n_steps=` itself) and returns `{"n_steps": N}`; its only caller is the print.
  3. `animate_control_law(axis=0, ...)` (value iteration's planner and
     `planning/dynamic_programming/plotting.py`): the input channel drawn, the idea
     `plot_control_law` now names `u_axis=`.
  4. The planners' `compile(backend=None)` (trajectory optimization, MPPI): `backend` stays
     positional (the MPC calls `planner.compile(backend)`), where `System.compile(*, backend=)`
     is keyword-only.
- **Options**, per name:
  1. `block_input` → `subsystem_input` (one library caller, one test), or kept as is.
  2. `transcription_options()` → `transcription_numbers()`, folded into the print (no method
     left), or kept as is.
  3. `animate_control_law(axis=)` → `u_axis=`, as `plot_control_law`; or kept as is.
  4. `compile(*, backend=None)` keyword-only, as the system's (two callers in the MPC), or
     kept positional.
- **Recommendation:** rename 1 and 3 and make 4 keyword-only (one word, one shape per idea,
  RULES 2.7 and DESIGN §4); fold 2 into the print, since nothing else reads it.
- **Blocks:** nothing; each is one commit with its callers, no alias.
- **Answer:**

### The 3-D car's rear axle: locked, or an open differential?
- **Context:** the 3-D car (`SuspensionCar3D`, the last rung of the car ladder) drives both rear wheels at one rate `w_rear`, a locked axle (ruled
  2026-11-11). Measured on the built car against the drivetrain bicycle with the same numbers,
  steady yaw rate at a steer of 0.01 rad, 5 and 15 m/s: 65 % of the bicycle's on the default
  car, 79 % on the off-road preset, 88 % on the formula preset. The two rear tires slip against
  each other and resist the yaw, like about 1 m more wheelbase on a 2 m car. The reduction test
  passes because it narrows the track to 0.3 m. So "the two rungs are the same car" and "a
  planar MPC drives the 3-D car" do not hold at the default track.
- **Code, today and under option 1:**
  ```python
  # Today: both rear wheels at the axle's rate
  w_wheel = vx_w[i] / r_wheel if front else w_rear
  # Open: each rear wheel at its own rate, w_rear their mean (side = +1 left, -1 right)
  w_wheel = vx_w[i] / r_wheel if front else w_rear + side * 0.5 * dw_rear
  ```
- **Options:**
  1. *An open differential by default.* Each rear wheel turns at its own rate, the drive torque
     split evenly; `w_rear` stays the axle's mean rate, so the first nine states keep their
     meaning (plan §4 already designs for it). The 3-D car then reduces to the bicycle at any
     track. Costs: the two rates are either two more states, or solved from the even torque
     split inside `f`.
  2. *Keep the locked axle,* and say in the class docstring and the tests that the car steers
     less than the bicycle, with the numbers above. Costs nothing; the model-mismatch study
     starts with a mismatch that is not the suspension's.
  3. *A `locked_axle=` option,* open by default. Both behaviours, one more structural switch.
- **Recommendation:** option 1: it is what makes the 3-D car the bicycle's limit, which the
  ladder promises.
- **Blocks:** the planar-controller-drives-the-3-D-car demo, and the presets' final numbers.
- **Answer:**

### Which point does the kinematic bicycle's `(x, y)` follow?
- **Context:** `KinematicBicycle.f` writes `ẋ = v cos ψ`, `ψ̇ = v tan δ / L`, the equations of
  the rear axle's centre. Its frames draw `(x, y)` as the centre of gravity, `b` ahead of the
  rear axle, as on the dynamic rungs, where `(x, y)` is the centre of gravity. The ladder's
  rule is that a state keeps its meaning from rung to rung. This predates the vehicle branch.
- **Options:**
  1. *The centre of gravity,* with the sideslip `β = atan(b tan δ / L)`:
     `ẋ = v cos(ψ + β)`, `ψ̇ = v cos β tan δ / L`. The same point on every rung; one more line
     in the first model a student reads, and the numbers of every kinematic demo move.
  2. *The rear axle, drawn there:* `f` unchanged, the body frame's drawing moved back by `b`.
     The simplest textbook model stays; `(x, y)` changes meaning between rungs 2 and 3, said
     in both docstrings.
  3. *Leave it,* and state the approximation in the docstring.
- **Recommendation:** option 2: rung 2 is the no-slip lesson and its page should stay three
  lines; the drawing and the docstring then tell the truth.
- **Blocks:** nothing.
- **Answer:**

### The public face of the car's parts
- **Context:** the tires, the springs and the terrains are parts a car is built from. Three
  small inconsistencies, each a public name or call:
  1. tires and springs take their numbers at construction (`BrushTire(mu=1.6)`); terrains do
     not (`Bumps()`, then `car.params["terrain"]["amplitude"] = 0.8`);
  2. the parts are imported from their modules
     (`from minilink.dynamics.catalog.vehicles.terrain import Bumps`), which the demos'
     import check allowlists; the cars come from `minilink`;
  3. `TerrainBlock` (ruled: "also exported as a static block") has no car port to wire to:
     no car outputs its contact positions or takes ground heights.
- **Options,** per item:
  1. `Bumps(amplitude=0.8, wavelength=30.0)`, as the tires; or kept.
  2. the parts on `minilink.catalog` beside the cars; or kept in their modules.
  3. a `contacts` output on the 3-D car for the block; or the block waits for a diagram that
     needs it; or kept as it is.
- **Recommendation:** constructor numbers on the terrains and the parts on `minilink.catalog`
  (one way to build a car, one import line); the block waits.
- **Blocks:** nothing; each is one commit.
- **Answer:**

### Three small points the car ladder's review left
- **Context:** the independent review of the car ladder (PR #141) raised three points that
  change a public signature or name, so each waits for you. They are small and independent.
  1. *Where `t` sits on the bicycles' hooks.* To read a road that changes with time, the
     review added the time of the call as an optional last argument:
     `compute_tire_physics(v, u, params, q, t)`, `contact_friction(q, params, t)`,
     `generalized_d(q, v, u, params, t)`, `rear_wheel_ground_torque(v, w, delta, params, q, t)`.
     On the 3-D car it sits before `params`, as on `f`: `contact_forces(x, t, params)`. The
     trailing form keeps the project folders' positional calls working; those rungs
     (`BicycleDynTauRate`, `BicycleDynServo`, `BicycleDynEngine`) therefore still read the
     road at `t = 0`.
  2. *The numbers and the command limits written twice.* The 3-D car is a model of its own,
     so the 23 default numbers it shares with the drivetrain bicycle, `bound_commands`,
     `commands`, `power` and the command ports are written on both classes, source-identical,
     held together by a test.
  3. *Two new method names:* `set_numbers(numbers)` (a preset's numbers over the car's own,
     the command limits and the rest state following) on `DrivetrainBicycle` and
     `SuspensionCar3D`, and `rest_state()` on the 3-D car.
- **Options,** per point:
  1. `t` before `params` everywhere (CONSTITUTION, 'One signature'), with the project folders'
     calls changed in the same commit; or the trailing form kept and stated.
  2. One `DRIVETRAIN_PARAMS` constant read by both cars, as `PUBLIC_RACECAR_PARAMS` is, and
     `bound_commands` as one module helper; or each model's numbers kept on its own page.
  3. The names kept; or others of your choosing.
- **Recommendation:** `t` before `params` everywhere once the project folders move to the
  private repository (the consolidation); the shared constant for the numbers, since a
  number has one home, the equations staying on each page; the two names kept.
- **Blocks:** nothing.
- **Answer:**

## The architecture

### Does the planning framework get a package of its own?
- **Context:** `minilink.planning` is the largest package (19 093 lines, the core is 17 005).
  A third of it is not a planner: the 13 files at its top level (6 585 lines) are the
  framework every planner shares: the problem (`problems.py`, `step_problems.py`,
  `step_model.py`), the evaluation (`evaluator.py`, `evaluation.py`, `comparison.py`), the
  solution record (`results.py`), the planner base class and its helpers. The algorithms sit
  beside it in four subpackages (`trajectory_optimization`, `dynamic_programming`, `search`,
  `spatial`). CONSTITUTION §1 names `PlanningProblem` and `PlanningSolution` as a foundation
  beside `System`, and the RL planner in `minilink.learning` builds on them. The package
  layout was ruled by pillar on 2026-10-24.
- **Options:**
  1. *A subpackage inside `planning`* for the framework, beside the four algorithm ones,
     the facade unchanged (`from minilink.planning import PlanningProblem` still works). No
     user import moves; the folder shows what the structure already is.
  2. *Into `core`.* The problem and the solution are core objects by the constitution. Costs:
     the evaluator runs simulations and compares planners, tool-level code that would grow
     the core.
  3. *As it is:* the framework at the package's top level.
- **Recommendation:** option 1, decided and done during the consolidation's core cut, when
  it is known which of the 13 files stay.
- **Blocks:** nothing; it moves no public import.
- **Answer:**

### What the check of the core notebooks found in the API
- **Context:** to make the core notebooks exact (2026-11-12), every public signature of the
  core objects was resolved against the code. The list below is what looked inconsistent in
  the library itself, not in the notebooks; nothing was changed. It is input for the
  maintainer's API review, each item his to keep, change or drop. Three defects found the
  same way are in KNOWN_ISSUES ("The compiled evaluators").
- **Two names for one idea:**
  1. a diagram's member is `subsystem` (`subsystem_id`), `block` (`diagram.of`), `role`
     (`Trajectory.of`) and `of=` (`compute_cost`); `sys_id` on a diagram, a bare `id` on
     `add_input_port` and `add_output_port`;
  2. `OutputPort(function=...)` is read back as `port.compute`;
  3. `GridField` has `n` and `dim`; the other fields `dim` only;
  4. the program evaluator has `J` and `objective`, `h` and `equality_residual`, `g` and
     `inequality_margin`, but one name only for `grad_J`, `jac_h`, `jac_g`;
  5. `method=` is the optimizer's algorithm on trajectory optimization, the Jacobian's on
     `LQRPlanner`, the backward step (`"table"`, `"loop"`) on value iteration, the learning
     algorithm on the RL planners; its default is `"slsqp"` on one and `"auto"` on `Optimizer`;
  6. the default time is `t=0` on `System.f`, `h`, `tf` and `t=0.0` on the catalog plants, the
     sets, the costs and the fields.
- **Positional against keyword-only:**
  7. `pe.evaluate(params, key, n_trials, *, trajectories)` against
     `evaluate(problem, controller, dt, *, ..., n_trials, key)`; `pe.program(wrt, bounds, key,
     n_trials, *, params)`; `pe.trajectory(key=None)` against `pe.evaluate(key=0)`;
  8. `solve` is keyword-only on `Optimizer` and `Planner`, positional on the compiled
     trajectory planner, which returns `(z, status)` and not a result object;
  9. `backend` defaults to `"numpy"` on `System.compile`, `None` on a planner's `compile`
     (positional), `"auto"` on `problem.compile`;
  10. `Field.as_set(*, lower, upper)` is keyword-only with a sentinel; `FieldSet(field,
      lower=0.0, upper=None)` is positional with another default;
  11. `plot_diagram(file_name, show, *, expand)`: `show` is positional there and keyword-only
      on every other plot;
  12. `StepPlanningProblem.transition(x, u, k, params)` has no default for `k`, and
      `terminal(x, t, params)` takes `t` where its sibling takes `k`;
  13. `SingletonSet.residual(z, params)` has no `t`; `margin(z, t, params)` has.
- **Siblings whose surfaces differ:**
  14. the step `compute_trajectory` has no `port`, `n_samples` or `solver_warnings`, and
      another keyword order;
  15. `Planner.plot_trajectory`, `animate` and `plot_cost` take `**kwargs` where
      `plot_control_law` and `plot_cost_to_go` spell every keyword; `Comparison`'s plots have
      no `renderer`, `ax` or `title`;
  16. `StepPlanningProblem` has no `compile`, `nominal`, `goal_point`, `discount_factor` or
      `drawn_paths`, and a `disturbances` property that always returns `{}`;
  17. three draw verbs on the problem with three defaults: `realize(key=None, n=1)`,
      `sample_x0(key, n=None)`, `sample_params(key)`;
  18. the problem's parameter tree has an `"X0"` key only when the start is not the singleton
      at `x0`, so its keys depend on how the start was given;
  19. `Uniform.lower` and `upper` are read-only where `Gaussian.mean` and `BoxSet.lower` can
      be set; the base `Set` has no `dim`, so `MarginField(CallableSet(...)).dim` raises.
- **Public members that read as internal:** the wiring mixin's `init_wiring`,
  `new_subsystem_id`, `get_local_state`, `get_local_input`, `subsystem_params`,
  `compute_subsystem_output_port` and `check_algebraic_loops`; `SampledSystem`'s
  `source_params`, `write_source_params` and `update_with_cost` (it has no internal banner);
  `PlanningProblem.parts_view`, `Planner.build_kwargs` and `rebuilt_on`,
  `Optimizer.solve_count`, `SampledCost.laws_at`; `System.compute_state`,
  `get_input_port_slice` and `get_all_input_labels_and_units`.
- **Exports:** `ModelPredictiveController` is exported by `minilink.control` and defined under
  `planning/`; `MPPIPlanner` is the one planner not at the root; `array_module`, which a
  hand-written `f` needs to run on both backends, is exported by no facade.
- **Options:** take them one family at a time in the API review, with the core notebooks open
  beside the code; or rule a few now.
- **Recommendation:** the naming items 1, 5 and 7 to 9 first: they are what a reader of the
  core notebooks meets on every page.
- **Blocks:** nothing.
- **Answer:**

## The vision

### Where is the fair comparison stated?
- **Context:** the maintainer's direction of 2026-11-09 (ROADMAP §5): one problem handed to
  every planner and one judge for every answer is the toolbox's distinctive contribution, to be
  promoted in the vision. Today CONSTITUTION §1.1 names the three foundations and calls every
  approach "a tool on that path", without saying the approaches are compared on it; the README
  shows it in §7 of 9, after the model, the wiring, the graphics, JAX and the optimizer.
- **Proposed wording, CONSTITUTION §1** (a new item after "One toolbox, one model"):
  "**One problem, one judge.** Every approach takes the same `PlanningProblem` and returns a
  `PlanningSolution`; every answer is scored by the same evaluation, on the same model, cost,
  sets, start law and disturbances. A method is never scored by a judge of its own."
- **Options:**
  1. *CONSTITUTION §1 and the README's lead.* The item above, and the README's first example
     the comparison table. Costs: the README leads with the advanced tool, before a student's
     first simulation.
  2. *CONSTITUTION §1 only, the README's order kept.* The item above; README §1's backbone
     paragraph gains one sentence and §7 is fixed. The teaching order stays.
  3. *A governing principle (CONSTITUTION §4) instead of the vision.* "One judge for every
     planner" beside "One set of tools for every System": a rule for reviews, not an identity.
- **Recommendation:** option 2 with the principle of option 3 as well: the identity in §1, the
  review rule in §4, and the README's teaching order kept until its example wins.
- **Blocks:** nothing in the code; the README's wording waits on it.
- **Answer:**

## Toward v1.0

### How does a public name change once v1.0 is frozen?
- **Your answer (2026-10-27):** not a `DeprecationWarning` shim for one minor release, as
  proposed; the policy stays open. No alias, one sweep, holds meanwhile.
- **Your answer (2026-11-03):** the pending renames land before the freeze, one sweep per
  group, under the vocabulary of RULES 2.7; so the freeze itself needs no shim.
- **Blocks:** the trust contract (V4), for the names changed after v1.0.
- **Today:** no alias or shim ever. The frozen names are drawn at v0.9 from RULES 2.7.
- **Options still open (after v1.0 only):** a shim of some length; a major version per rename;
  a snapshot test of the export list either way.
- **Detail:** [TODO, V4](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**
