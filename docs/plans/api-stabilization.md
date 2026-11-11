# API stabilization before the example rebuild

The maintainer's request of 2026-11-10: every example is about to be deleted and rebuilt, and
the new ones must not be written on names that will move again. This is the review of the
public surface as it stands on `dev` (commit `0fb4878`), and the proposal to settle it first.
**Nothing here is approved yet**; the run starts on the maintainer's go, and section 4's
decisions are taken one at a time before their rows are touched.

**How it was reviewed.** Four independent reviewers, one area each (the system core and
wiring; the mathematical objects and the solver boundary; planning and learning; control,
analysis and the catalog), briefed with CONSTITUTION, RULES and DESIGN, not with conclusions.
Each ran its snippets and counted call sites. A census of every keyword on the 228 public
objects ran beside them. The behavioural claims of sections 1 and 4 were reproduced a second
time by hand (NumPy, a 4-core cloud machine). Usage counts read library / tests / examples.

**What the review says overall.** The core contracts hold: the one signature
(`f`, `h`, `margin`, `value`, `g`, `sdf`, `sample`), the operators and their refusals,
`Trajectory`, `PlanningSolution`, `Evaluation`, `solve(*, params, evaluate, n_trials)` on every
planner, the analysis calling pattern, and the 2026-11-04 to 11-08 renames, which landed almost
everywhere. What is left falls in four groups: defects that give a wrong or silent answer
(section 1), names an existing rule already decides (section 2), small additions that remove
friction (section 3), and choices only the maintainer can make (section 4). Section 5 is
planned work to pull forward, because the rebuilt examples would otherwise be migrated twice.

Steps are numbered `AS-n` (this doc's own prefix).

## 1. Defects: a wrong answer, a silent default, or a contract the docs state and the code breaks

No public name changes here; numbers move only where noted.

**Status (2026-11-10): landed on `dev`** (built on the branch `claude/zen-noether-sj2n6c`, one
commit per step, on the maintainer's go for this section; merged into `dev` on his
instruction the same day; the changes a user meets are in ROADMAP §2's breaking changes). Three steps came out smaller than written, each
because a recorded contract or a shape decision stood in the way, and their rest is a decision
of section 4:
- AS-6: the plan now starts at `t0` and its evidence runs the plan's own length; **the replay
  that misses the goal is decision 23**.
- AS-7: the refusal of a plan with signals is DESIGN §6's contract (a loop's state of the
  plant's size must not be read as the plant's), so it stays; its message now gives the line
  that works, `Trajectory(traj.t, traj.x, traj.u)`.
- AS-10: a solution that stores the planner's whole tree changes the shape of
  `PlanningSolution.params`, a core record: **decision 24**.

| Step | What happens today | Fix |
| --- | --- | --- |
| AS-1 | `lqr`, `lqr_finite_horizon`, `kalman`, `lqg` on a step or sampled model solve the continuous Riccati equation: `lqr(discretize(Pendulum(), 0.1), I, I, x_bar=[π, 0])` returns `K = [816.9, 40.7]`, closed-loop `|λ| = [1.25, 0.80]`, unstable, no message (`control/design.py`, `estimation/design.py`) | Refuse a discrete model with the line to write ("design on the continuous plant, then `% dt`"). The discrete equation is section 4, decision 14 |
| AS-2 | A design verb at a point that is not an equilibrium drops the affine term silently: `lqr(Pendulum(), I, I, x_bar=[π/4, 0])` returns `u_bar = 0` | Warn once in the design verbs (not in `linearize`), naming `find_equilibrium` and `u_bar=` (RULES 4.12) |
| AS-3 | `BoxSet.sample(None)` draws fresh entropy (two calls differ); `Uniform` and `EllipsoidSet` return the centre | The centre, as the rule "no key, no draw" (DESIGN §4, RULES 2.7) |
| AS-4 | `field.as_set(upper=level)` is the band `0 ≤ V ≤ level`, not the sublevel set DESIGN and the docstring promise; wrong for a field that goes negative (a cost-to-go, a critic, a clearance) | `lower` defaults to `0.0` only when `upper` is not given |
| AS-5 | `SumCost.g_max` / `h_max` accept a partial parameter tree (`.get`) that `g` refuses with a bare `KeyError` | Index, as `g` does (RULES 2.3, nothing merges) |
| AS-6 | RRT: the record says "goal reached", its own evaluation "goal missed" (plan ends at `[3.31, −0.15]`, the replayed controller at `[2.66, −1.58]` on the plan's `dt`); with `problem.t0 = 2` the plan runs `0…4.8` and the score `2…6.8` (`search/rrt.py:423` passes an absolute end time as a length) | The plan starts at `t0`; the replay holds each edge's input over its period; a length is passed. Numbers move once |
| AS-7 | `evaluate_trajectory(problem, solution.trajectory)` refuses a trajectory-optimization plan, and its advice (`traj.of("plant")`) fails too | Score a trajectory whose `x`, `u` have the plant's sizes; fix the message |
| AS-8 | `evaluate` on an infinite-horizon problem scores 10 s silently (two copies of the constant) | Warn once naming `episode_length`; one constant |
| AS-9 | `compute_trajectory(n_samples=11, dt=0.1)` logs through `logging.warning` and runs | Raise, as `Trajectory.resample` does |
| AS-10 | `planner.solve(params=solution.params)` is refused ("missing `method`, `seed`") | The solution stores the planner's complete tree |
| AS-11 | `control/robotic.py:476` probes `hasattr(plant, "task_dim")` (RULES 4.3) because `KinematicManipulator` says `effector_dim` | `task_dim` on both (10 / 1 / 0), the probe deleted; the dead `forward_differential_kinematic_effector` removed |
| AS-12 | Messages that do not name the fix: an unknown `solver` (no list); the Jacobian's `wrt` list (`'u'` twice); `lqr(p, [1, 1], [1])` → SciPy's "Matrix q should be square"; `FieldInputSet.margin(u)` without `x`; `CallableSet` with a one-argument callable; `compare(A=controller)` without a problem; `TaskImpedance(Pendulum())` (raw `AttributeError`); `max_iterations=` on trajectory optimization; a first-order method on a program without a gradient (fails at `solve`, not at construction); `as_cost(shaping="…")` | One domain sentence each, naming the valid values or the line to write |
| AS-13 | Value iteration's cost-bound warning points at `planning/step_model.py`, not the caller, and does not say the sweeps will not converge | `stacklevel`; the consequence in the sentence |
| AS-14 | No `__repr__` on `System`, sets, costs, fields, laws: a list or an error shows addresses | The one-line head each already prints |
| AS-15 | Docstrings and DESIGN lines the code has left behind: a merge promised on `Optimizer.solve`, `MathematicalProgram`, `SumCost` and the program evaluator; `with_signal`; `gradient` / `hessian` / `jacobian_h`; "its name in snake case"; "value iteration's α"; "the options' guess"; DESIGN's "`scipy_slsqp` (the default)"; `Distribution` "a duck type"; every record's `method`; `ValueIterationRecord.discount_rate` printing `None` | Corrected to the code and the rulings |

## 2. Names an existing rule or ruling already decides

Each breaks a name, with no decision to take: RULES 2.4, 2.5, 2.7, 3.4, 4.3 or a recorded
ruling says what the word is. One commit per row, every call site in the same commit (RULES
3.4), no alias.

**Status (2026-11-10): AS-20 to AS-32 landed on `api-settle`**, one commit per row, the seeded
baseline byte-identical after each (but for two renamed keys of AS-21); AS-24 came out smaller:
the export landed, `period` keeps its name (see the decisions below).

| Step | Today → proposed | The rule | Uses |
| --- | --- | --- | --- |
| AS-20 | `DirectCollocationTranscription(DirectCollocationOptions(n_steps=20))` → `DirectCollocationTranscription(n_steps=20)`; the same for shooting and multiple shooting; the four option records deleted. Landed on `api-settle`. | 2.5, 2.7; the 2026-11-04 sweep removed the other option records | 37 calls |
| AS-21 | The compiled planner's status key `"iterations"` → `"n_iterations"`; `MPPIRecord.effective_samples` → `effective_rollouts`; `StateSpaceGrid.nodes_n`, `actions_n` → `n_nodes`, `n_actions`; `LyapunovCertificate.verify(n=)` → `n_rollouts=`, its plots' `n=` → `n_samples=`. Landed on `api-settle`. | 2.7, every count is `n_<thing>` | 12; 10; 72; few |
| AS-22 | `plot_sampled_futures(axes=)` → `x_axis=`, `y_axis=`; value iteration's `plot_control_law(axis=)` → `u_axis=`; the animations' `save=` (RRT) and `save_path=` (value iteration) → `file_name=`; the plot title "Policy u[0]" and `plot_policy_field` → control law; `StateSpaceGrid.plot_value` → `plot_cost_to_go`. Landed on `api-settle`. | 2.7 | small |
| AS-23 | `tf` as a length in `score_run(tf=)`, `planner_evidence(tf=)`, `steps_of(tf, dt)`, `RolloutEnvironment.tf`, DESIGN's scoring formula → `horizon` or `episode_length`; `DynamicProgrammingPlanner.final_time` → `tf`. Landed on `api-settle`. | 2.7, `tf` is absolute | internal; 24 |
| AS-24 | `DiscreteLTISystem(period=)`, `.period` → `sample_period`; the class exported on `minilink.dynamics`. The export landed on `api-settle`; `period` stays (a question, below). | 2.7 | 11 |
| AS-25 | `RolloutEnvironment.step(x, t, u, key, params)` → `(x, u, t, key, params=None)`; `LookupPolicy.action(x, params=None, *, t=0.0)` and `NeuralPolicy.action` → `action(x, t=0.0, params=None)`. Landed on `api-settle`. | CONSTITUTION, one signature | about 15 |
| AS-26 | `MPPIPlanner.method` (a dict of numbers) → `method_numbers`; `TabularLearningPlanner(eta=)` → `learning_rate=` (TODO's judgment-calls row names it); `planner.train_time` → `train_time_s`; `TrajectoryOptimizationRecord.solver_success` → `optimizer_success`. Landed on `api-settle`. | 2.7 | 8 examples |
| AS-27 | The optimizer's native numbers: `options={"maxiter": 5}` on `gd` / `adam` → `max_iterations=5`, `gtol` → `tol`; `options=` refused for a native method; `OptimizationResult.n_iterations` added beside `stats["nit"]`, `stats["algorithm"]` → `stats["method"]`. Landed on `api-settle`. | 2.7 | few |
| AS-28 | `plot_rollout(rollout=)` → `traj=`; `input_port_id=` → `port=` (blocks already say `port=`); `TrajectorySource(interpolation="previous")` and `ev.rollout(hold="foh")` → `hold="zoh"` / `"linear"` on the three tools. Landed on `api-settle`. | one word per idea | 5; 25; 14 |
| AS-29 | Keyword-only where every sibling already is: `PlanningProblem(sys, *, x0, cost, …)` (0 callers pass a second positional); `compile(*, backend=, verbose=)`; `animate(traj=None, *, …)`, `render(x, u, t, *, …)`; `MLP(input_dim, output_dim, *, …)`; `Sys2Gym(sys, cost, *, …)`; `NativeSampledSystem(state_dim=0, *, sample_period, …)` with the port shortcuts. And the reverse: `evaluate(problem, controller, dt=None, *, …)`, as DESIGN §6 writes it and `problem.compile` takes it. Landed on `api-settle`. | DESIGN §4, sibling verbs agree | positional callers only |
| AS-30 | Removed, with no caller or against a rule: `validate=` on `feedback`, `closed_loop`, `autowire` (0 uses; it switches off the algebraic-loop check); `PlanningProblem.has_goal`, `has_cost` (0 library uses); the `COMPILE_BACKEND_AUTO` re-export "so legacy callers keep working" (3.4); `InputPort.get_default_value`, `set_nominal_value` (duplicates of `nominal_value`); `RRTPlanner.reached_goal`, `iterations` (copies of the record, RULES 2.4; 31 / 24 / 5); `GivenPolicy` → `GivenController`. Landed on `api-settle`. | 2.4, 3.4, 4.10 | as noted |
| AS-31 | The problem's five static helpers and the `Optimizer`'s four machinery methods → module functions; the duplicate helper names with different bodies (`require_cost_to_go`, `planner_parts`, `steps_of`, `problem_leaves`) → one each; `SampledSystem.counts`, `resolves` → `is_tick_law`, `is_integrated`; `is_bounded(box)` → `has_finite_face`. Landed on `api-settle`. | ROADMAP 2026-11-07; readable predicates | library only |
| AS-32 | The explicit-wiring drawings written out: `plot_input_output_map(**kwargs)`, `plot_control_law(**kwargs)`, `SampledDiagramSystem.animate(**options)` → their keywords; `plot_diagram(show_inline=, show_pdf=)` folded into `show`. Landed on `api-settle`. | DESIGN §4, "written out so every editor shows them" | 27 |

## 3. Additions that remove friction (nothing breaks)

| Step | Addition |
| --- | --- |
| AS-40 | `QuadraticCost(Q, R)` works: `S`, `x_bar`, `u_bar` default to zero (today five required positionals, so every call goes through `from_system`) Landed on `api-settle`. |
| AS-41 | `lower`, `upper`, `dim` on `BoxInputSet` and `Uniform` (`plant.input_box.lower` fails today, `plant.state.box.lower` works); `dim` on every set and field where it is known; `BallSet.sample`, `SingletonSet.sample` Landed on `api-settle`. |
| AS-42 | `CostFunction.h` defaults to `0.0` (a running cost no longer writes `return 0.0`) Landed on `api-settle`. |
| AS-43 | `Gain(2.0)` works (`dim=1`, as `Sum`, `Saturation`, `UnitDelay`); `Integrator(dim=1)`; `Saturation` takes vector bounds Landed on `api-settle`. |
| AS-44 | `poles(sys, …)` on `minilink.analysis`, the root and as a `System` shortcut (the examples write `np.linalg.eigvals` 22 times); `print(lqr(…))` shows `K` as an observer shows `L`, with a read-only `K` Landed on `api-settle`, but for the `System` shortcut (a question, below). |
| AS-45 | `plot_nyquist` as a `System` shortcut (the fifth of five one-channel plots); `linearize` and `transfer_function` on a `StepSystem` (the plots already work there) Landed on `api-settle`. |
| AS-46 | `diagram.add_subsystem(sys)` without an id (the operators' rule); `StepPlanningProblem.replace`; a scalar `u` on a one-input step rollout; the shape error of a sampled `u` naming `n_samples=` / `dt=` Landed on `api-settle`. |
| AS-47 | `verbose=False` on every planner's `solve` (missing on LQR, RRT, MPPI); `callback` a callable or a list everywhere; `params=` on `solve_steps`; `show=` on every plot Landed on `api-settle`. |
| AS-48 | `sensitivity(plant, controller, …)` positional, with `of=` / `wrt=` and the channel default of `bode` (today keyword-only, and it refuses the default pendulum the sibling verbs accept) Landed on `api-settle`. |
| AS-49 | On the facades: `LTISystem`, `MechanicalSystem`, `step_info` on the root (the design verbs' own error tells the student to build an `LTISystem`); `Margins`, `StepInfo` on `minilink.analysis`; the shaping functions on `minilink.core` Landed on `api-settle`, with `LinearizedSystem` beside `LTISystem` and `poles` on the root. |

## 4. Decisions for the maintainer

Each changes a public name, a shape or a number a student meets. They are put one at a time,
with the code before and after, once the plan is approved. The recommendation is the agent's;
TODO's "API sweep's judgment calls" row is folded in here.

| # | The question | Today | Recommendation |
| --- | --- | --- | --- |
| 1 | **Block or subsystem?** One idea, two words: `add_subsystem`, `.subsystems` beside `diagram.of(block)`, "blocks:" in the print, RULES and DESIGN | 22 / 206 / 114 calls | **Ruled 2026-11-10: `subsystem`**, the one word for a system placed in a diagram: the print (`subsystems:`), the internal helpers (`block_roles`, `block_run`, `blocks_params`, …) and DESIGN / RULES prose where "block" means that; `get_subsys_input_port` → `get_subsystem_input_port`, `compute_subsys_output_port` → `compute_subsystem_output_port`. "Block" keeps one meaning, the standard components of `minilink.blocks` (the package keeps its name). No other public rename. Landed on `api-settle`. |
| 2 | **The simulator's solver names.** `Simulator` takes `"rk4_fixedsteps"`, `"euler_fixedsteps"`, `"scipy"`…; `discretize`, the evaluators and the sampled loop take `"rk4"`, `"euler"`. `compute_trajectory(solver="rk4")` is refused | 83 uses | **Ruled 2026-11-10: `"rk4"`, `"euler"` everywhere**, both fixed-step (the step model's words); `rk4_fixedsteps` and `euler_fixedsteps` gone, no alias; the variable-step Euler dropped (a non-uniform grid on a fixed-step solver refused, naming `scipy` or a uniform `dt`), after a search confirms no caller; the SciPy presets unchanged. Landed on `api-settle`. |
| 3 | **`InputSet.margin(u, x, t)`** reverses the model's order; `U.margin(x, u)` returns numbers, silently wrong | about 10 library calls | **Ruled 2026-11-10: the member first, for every set** (`X.margin(x, t)`, `U.margin(u, x, t)`), written into CONSTITUTION's one-signature paragraph ("a set's `margin` takes its member first, then the arguments it depends on"), plus a shape check that refuses a member of the wrong size; no call site changes. Landed on `api-settle`. |
| 4 | **The `get_` methods a controller or plant author writes:** `get_port_values_from_u(u, "r", "y")`, `get_u_from_input_ports()`, `traj.get_signal(name)`, `get_kinematic_geometry`, `get_dynamic_geometry`; and `tf`, which is the pose map, a final time and almost a transfer function | 23; 36; 15; 93; 143 | **4a ruled 2026-11-10:** `get_port_values_from_u(u, …)` → `self.ports(u, …)` (the same three return forms), `get_u_from_input_ports()` → the read-only property `sys.u_bar` (each input port's default), `traj.get_signal(name)` → `traj.signal(name)`; no alias. Landed on `api-settle`, but for `u_bar` (see the decisions below: `get_u_from_input_ports` stays until the question is answered). **4b ruled 2026-11-10: no rename** — `tf`, `get_kinematic_geometry` and `get_dynamic_geometry` stay as they are (the drawing hooks keep their names; `tf` the pose map beside the final time `traj.tf` is a recorded exception of RULES 2.7); S44's one hook stays v0.9 |
| 5 | **Bounds: three spellings.** `lower` / `upper` (sets, `Saturation`, the program), `lower_bound` / `upper_bound` (ports), `u_min` / `u_max` (PID), `delta_max` (path tracking) | 833 for the ports | **Ruled 2026-11-10: option 2** — a port and the box it builds say the same word: `add_input_port(..., lower=, upper=)`, `port.lower` / `port.upper`, `state.lower` / `state.upper` (was `lower_bound` / `upper_bound`), no alias; a law's limit on a named signal keeps that signal's name (`u_min`, `u_max`, `delta_max`); the rule in RULES 2.7. Landed on `api-settle`. |
| 6 | **The catalog's vocabulary.** Mass is `"mass"` on 13 plants and `"m"` on 2; damping `d`, `b`, `damping`, `damping_coef`; drag `cda`, `CdA`, `cdA`; `Pendulum(length=, mass=)` writes keys `l`, `m`; `Pendulum()` has `I = 1` beside `m = l = 1` (ω = 2.21, not √(g/l) = 3.13) | every `params[…]` in a new example | **6a ruled 2026-11-10 — by judgment, not a hard rule:** a single textbook letter where it is clear in context (`m`, `l`, `k`, `d`); capital letters reserved for matrices in `params`; a short or full word when one letter is not enough (`CdA`) or clashes with a core name (`gravity`, `g` being the running cost); one spelling per quantity across the catalog; the constructor keyword writes the same key. `Pendulum`'s scalar inertia `I` (a capital, not a matrix) is the builder's judgment. **6b ruled 2026-11-10:** `Pendulum`'s default `I = 0` (the textbook point mass, `ω = √(g/l)`), built at the release with the student-facing re-pin (phase 6), not in the `api-settle` run; the scalar inertia keeps the letter `I` |
| 7 | **Labels and units.** `"Torque 0"`, `"[Nm]"` from the mechanical base against `"tau1"`, `"Nm"` on the plants; rates `dtheta`, `omega`, `yaw_rate`; two plants rate-first | plot axes | **Ruled 2026-11-10: option 1** — units bare (`"Nm"`, the plot adds brackets); positions then rates; a rate `d<position>` unless the textbook names it (`omega`, `yaw_rate`); the base's defaults `q1…`, `dq1…`, `u1…` from 1; the two rates-first plants reordered; the rule written in RULES. Landed on `api-settle`. |
| 8 | **The mechanical base's names** (S32's renames, pulled forward): `C(q, v)` on the base against `C(q, dq)` on 26 plants; `actuators=` a count; the mass matrix's one name | every plant | **Ruled 2026-11-10: none now** — `C(q, v)` against `C(q, dq)`, `actuators=` and the mass matrix's one name wait for the one-mechanical-family work (S32) and the maintainer's review of the hierarchy |
| 9 | **`Impedance(dof=1, *, Kp, Kd)`** against `PD(Kp, Kd, *, dim=1)`: `Impedance(10)` is a 10-axis block | 8 tests | **Ruled 2026-11-10: option 1** — `Impedance(Kp=10.0, Kd=1.0, *, dim=1, tracking_ref=False, …)`, `IntegralImpedance(Kp, Ki, Kd, *, dim=1, …)`: the PID family's signature, `dof=` → `dim=`, no alias. Landed on `api-settle`. |
| 10 | **The path-tracking laws** default to the UdeS racecar (`wheelbase=0.34`, `state_dim=9`) and close on no catalog car with `@` | 60 calls | **Ruled 2026-11-10: the laws take the plant** — `PurePursuit(plant, path, *, lookahead=…)` and its siblings read the wheelbase, the steering limit, the measurement size and the pose and heading states from the plant, so `PurePursuit(car, path) @ car` closes on every car; built with K8-0 (the car family to three models), not in the `api-settle` run |
| 11 | **The linear model.** `lin.A` is a method (`lin.A()`); the operating point lives in three places (`LTISystem` attributes, `StateFeedback`'s port, `params["x_bar"]` elsewhere) | KNOWN_ISSUES | **Ruled 2026-11-10** (after two independent design reviews; reproduced bug: `lqr(lin) @ lin` settled at 6.27 instead of 0): five linear classes — `StateSpaceSystem` unchanged; `LTISystem` and `DiscreteLTISystem` pure `ẋ = Ax + Bu` (`x_bar`, `u_bar`, `y_bar` removed; `TransferFunction` stops carrying zeros); **`LinearizedSystem`** (under `StateSpaceSystem`), returned only by `linearize`: `ẋ = f(x̄, ū) + A(x − x̄) + B(u − ū)`, `y = h(x̄, ū) + C(x − x̄) + D(u − ū)` in the plant's own coordinates, ports, labels and units, `params` `{A, B, C, D, x_bar, u_bar, y_bar, dx_bar}` (`dx_bar = f(x̄, ū)`, zero at an equilibrium), its nominal `x̄` / `ū`; **`LinearizedStepSystem`** its z twin; `lin.deviation_model()` the pure δ view, an `LTISystem` with Δ labels. The drift is kept and the analysis verbs warn off an equilibrium. The matrices stay methods `A(t, params)` on every class (`lin.A` without `()` gets a sentence). `sys.u_nominal` (read-only) replaces `get_u_from_input_ports()`; `u_bar` keeps one meaning. The design verbs read the point from the model; `require_own_point` goes; the hand subtraction in `cartpole_dynamic_controller` and the step response's "linear" title fixed. No gain, pole or Bode number moves. Landed on `api-settle`, but for the hand subtraction in `cartpole_dynamic_controller` (see the decisions below) |
| 12 | **The optimizer's method names:** `"scipy_slsqp"`, `"scipy_trust_constr"` beside `"l-bfgs"`, `"gd"`, `"ipopt"` | 55; 7 | **Ruled 2026-11-10: option 2** — `method=` names the algorithm, the existing `backend=` (the array module) picks the implementation through one `(method, backend)` table, a missing pair refused with the valid list; `scipy_slsqp` → `slsqp`, `scipy_trust_constr` → `trust-constr`; a `library=` keyword may come later without a break. Landed on `api-settle`. |
| 13 | **`Step(1.0)` steps down** (`initial_value` first) and `Step()` is zero | 9 positional, 16 bare | **Ruled 2026-11-10: option 1** — `Step(final_value=1.0, initial_value=0.0, step_time=1.0, *, port="y")`: `Step()` the unit step at `t = 1 s`; positional callers migrated; a bare `Step()`'s numbers move once. Landed on `api-settle`. |
| 14 | **Riccati on a discrete model:** refuse (AS-1) or solve the discrete equation | none | **Ruled 2026-11-10: refuse** (landed with AS-1); the discrete Riccati designs come with the sampled filters in v0.9 — not the time for new discrete-time tools |
| 15 | **MPC's name and place.** `ModelPredictiveController` is a function returning a `PredictiveController`, on no facade; examples import the deep path 12 times | 7 tests | **Ruled 2026-11-10: option 1** — `ModelPredictiveController` is the class (`PredictiveController` renamed, the factory's signature its constructor); the deep imports move to `minilink.control`. Landed on `api-settle`. |
| 16 | **The planner's own plots.** Value iteration and tabular learning override `plot_cost_to_go` / `plot_control_law` with other keywords and another return type than the solution's; the plots return three shapes (`PlotResult`, `(fig, ax)`, `ax`) | few | **Ruled 2026-11-10: option 1** — the planning plots take one keyword set, written out (`sol.plot_cost_to_go(*, x_axis=, y_axis=, is_3d=, ax=, renderer=, show=)`, `plot_control_law` alike); the planner's method a thin call to the solution's, keeping only its extra `field=`; every plot returns `PlotResult`; scoped to the planning plots, the rest of S67 its own row. Landed on `api-settle`. |
| 17 | **Records.** `DynamicProgrammingResult` / `record.tables` / `planner.result` (which means two things between value iteration and tabular learning); `cost` is five different quantities across records; timings on two records only | 60 | **Ruled 2026-11-10: option 1** — `DynamicProgrammingResult` → `ValueTables` (`sol.record.tables`); `planner.result` → `planner.tables` on both grid planners, one meaning (the latest solution's tables), tabular learning's live table its own name (`planner.q_table`); a record's `cost` named by its quantity (RRT's `path_cost`, the learners' `episode_cost`; the `Optimizer`'s `cost` stays `J(z*)`; `Evaluation` keeps one of `cost` / `J` after a count); `solve_time_s` on every record. Landed on `api-settle`. **17b ruled 2026-11-10 (after an audit of every `J` / `cost`):** four words, one meaning each — `cost` the cost object (`problem.cost`, `cost=`, the parameter tree's `"cost"` key); `J` a scalar, one run's cost or a program's at `z` (`J*` at the optimum); `objective` the criterion minimized over draws; `cost_to_go` the function `J(x, t)` (a `Field`; `J*` or `J_π` by its source, not its name; `ValueTables.J` the DP table, kept). Renames: the records' scalar `.cost` → `.J` (`OptimizationResult`, `TrajectoryOptimizationRecord`, an iterate's, MPC `status["cost"]`, `MPPIRecord`); the signals `"cost"` → `"J"` with `h` added at the last sample on the continuous run too (one number moves: that last sample gains `h`, so it equals the run's `J`), `"cost_rate"` → `"dJ"`; RRT/RRT* `node.cost` → `cost_to_come`, `best_cost` → `best_cost_to_come`, `best_goal_cost` → `best_path_cost`; MPPI's internal `path_cost(...)` → `rollout_cost(...)`; the learners' `episode_cost` → `mean_episode_cost`; `Evaluation.raw_cost` and tabular learning's `greedy_tables` kept. Exception recorded: `Manipulator.J(q)` the Jacobian |
| 18 | **The learners' words.** `n_envs`, `n_steps` (periods per update, where `n_steps` is the horizon everywhere else), `policy=` that becomes `planner.actor`, `learn` and `predict` beside `solve`; `Sys2Gym` | 67; 45 | **Ruled 2026-11-10: option 1** — `n_envs` → `n_rollouts`; the RL planner's `n_steps` → `n_steps_per_update`; `policy=` → `actor=` (the block's role in actor-critic, read back as `planner.actor`; the handed-out snapshot stays `solution.controller`); `learn` folds into `solve`; `predict` moves to the SB3 bridge; `Sys2Gym` kept. The blocks named `…Policy` go to decision 20. Landed on `api-settle`. |
| 19 | **Design verbs.** `trajectory_lqr` beside `lqr_*`; `lqr_finite_horizon(…, tf)` a length; `StateObserver(A, B, C, L, D)` re-taking what the linear model holds | 30; 24; 10 | `lqr_trajectory`; `horizon`; `StateObserver(model, L)` |
| 20 | **Class names.** `VanderPol`, `ThreeBodyProblem`, `HeadingController` (the one "Controller"), `TaskKinematic`, `OutputReference`, `StructuralResult`, `StepCost.c` (a third letter beside `g`, `h`) | 16; 11; 23; 21; 23; 9 | `VanDerPol`, `ThreeBody`; the rest checked against the course notes' words first |
| 21 | **The root list.** Names with no example use (`LinearApproximator`, `QuadraticFeatures`, `RadialBasisFeatures`, `StaticSimulator`, the `*_gain` four, `BoxInputSet`); half pairs (`lqr_gain` without `lqr_gain_schedule`) | maintainer-owned | Regenerated from what the rebuilt examples import (DESIGN §2's own rule) |
| 23 | **How a held open-loop plan is replayed.** RRT's and MPPI's controller is a `TrajectorySource` holding each sample; read at every RK4 stage, it takes the next sample at a step's last stage, so RRT's replay on its own `dt = 0.05` misses the goal its plan reaches (it reaches it at `dt = 0.002`), while the same samples as a held input (`compute_trajectory(u=…)`) reproduce the plan exactly | KNOWN_ISSUES | The evidence of an open-loop held plan runs the simulator's held-input rule; the handed-out block unchanged |
| 24 | **`solution.params`** is the problem's tree, so `planner.solve(params=solution.params)` is refused ("missing `method`, `seed`") | a core record's shape | The planner's complete tree on the solution, after the two-reviewer brief AGENTS asks for a core object's shape |
| 22 | **Smaller ones:** `CallableSet(contains_fn=)` a set without a margin; `EllipsoidSet(field, …)` not buildable from `(center, P, level)`; `path.sample(s)` using the draw word (`path.at(s)`); a `dt` that does not divide the run overshoots `tf` (`[0, 0.3, 0.6, 0.9, 1.2]` for `tf = 1`); `LowPassFilter(cutoff_hz=)` beside `Sine(omega=)`; `Source.plot_signal`; `connection_verbose`; the program's start in `metadata["z0"]`; RRT's `max_nodes` (a cap on iterations) | small | In the row |

## 5. Planned work to pull forward

Each is already a workboard row; each renames something every rebuilt example would import.
Doing them after the rebuild migrates the new examples a second time.

- **The car family to three models** (K8-0) and **the root prelude trimmed** (K8-1).
- **Catalog hygiene** (S64, planned for v0.5): its naming parts only, with decisions 6 and 7.
- **The mechanical family** (S32): its renames only, with decision 8.
- **The one posed-geometry hook** (S44, planned for v0.9), with decision 4.

## 6. How the run goes

On the maintainer's go, as an automatic run (AGENTS, "Automatic runs"):

1. One feature branch from `dev`, one commit per step, one pull request at the end, never merged
   by the agent.
2. Order: section 1, then section 2, then section 3 (no decision needed in any); section 4's
   rows as each ruling comes; section 5 last.
3. Before the first edit, seeded baselines of every path the run touches, run twice and
   compared; byte-identical after each rename step. A step meant to move a number (AS-3, AS-4,
   AS-6) gets its own test instead.
4. After each commit: `ruff`, the touched modules' tests, the vocabulary test. Once on the
   finished branch: the full suite, the regression gates, the notebook checks.
5. Every rename migrates the library, the tests, the docs (DESIGN, RULES 2.7's table, ROADMAP
   §2's list) and the examples that exist at that time in the same commit, with a new row in
   the vocabulary test for each retired word.
6. Stop and ask when: a rename step moves a number; a ruling would change a name not listed
   here; a rebase on `dev` conflicts; speed falls beyond noise.
7. The run ends with the review pass and two independent reviewers (correctness; architecture),
   then the gates on the final tree.

**The examples.** Sections 2, 4 and 5 are cheapest once the old examples are gone: about half
of every usage count above is example code. If the maintainer deletes them first, the run
migrates only the library and the tests.

## 7. Left alone, on purpose

The one signature and its defaults; the class families (`*Set`, `*Field`, `*Cost`, the
un-suffixed laws and shapes); `as_set`, `as_cost` and their siblings; `Gaussian(mean, std=,
cov=)`; `MathematicalProgram`'s `J`, `h`, `g`, `lower`, `upper`; `Optimizer(program, z0, *,
method=, backend=, options=)`; `Trajectory`; the constructor shortcut `(state_dim, *, input_dim,
output_dim, …)`; the operators; `closed_loop` and `feedback`; `record=`; `discretize`; the
design verbs' block-versus-`_gain` split; `of=` / `wrt=` on the analysis verbs; the gain keys
`Kp`, `Ki`, `Kd`, `K`, `L`, `N`; `"gravity"` as the key on all 18 plants; `PlanningSolution` and
`Evaluation`; the `…Planner` suffix; `x_grid`, `u_grid`, `dt` on the grid planners.

## Decisions taken during the run

The run of rulings 1 to 4a on the branch `api-settle` (2026-11-10), one commit per ruling.

- **Ruling 1, which helpers say subsystem.** Renamed where "block" meant a system placed in a
  diagram: `BlockRoles` / `block_roles` → `SubsystemRoles` / `subsystem_roles`, `block_run`,
  `blocks_params`, `every_block`, `block_under`, `plant_block_roles`, `blocks_of`,
  `solving_block_of`, `block_path_of`, `require_one_block`, `require_new_block`,
  `require_plant_block`, `stateful_blocks_of`, `block_params_of`, `blocks_by_seed`,
  `check_blocks_trace`, `f_and_block`, `loop_blocks`, the evaluator's `block_ids`
  (an attribute no caller outside the class reads), and the verbose compile line ("Checking
  JAX compatibility of 4 subsystems"). Kept: `command_block` and `jacobian_block` (a block of
  a matrix), `block_until_ready` (JAX) and `block_when_needed` (Matplotlib), the diagram
  drawing's `render_block`, `block_html`, `get_system_block_html` (a box of the drawn block
  diagram, and the last one an export of `minilink.graphical.diagrams`), `policy_from_block`
  (a controller read as its policy, not yet placed), the wire notation `"block:port"`, and
  `SampledDiagramEvaluator.block_input` (a public method of the compiled evaluator, named in
  DESIGN and a test: renaming it is a public rename the ruling did not list; a question if
  the maintainer wants it). Library docstrings were left as they are; DESIGN and RULES prose
  changed only where "block" plainly meant a diagram's member.
- **Ruling 2, the variable-step Euler.** The search found no caller that runs `euler` on a
  non-uniform grid (every call gives `dt=` or `n_samples=`), so the backend is gone. The
  fixed-step runs are byte-identical under their new names (a seeded baseline of
  PID @ Pendulum, a forced pendulum and a sliding-mode loop, NumPy and JAX). A run that used
  the variable-step Euler moves at round-off only, its per-interval step read off the grid
  now one `dt`: at most `1.2e-15` in the states and `1.2e-14` in `plant:u` (the sliding-mode
  loop on its automatic grid); no test moved. The retired names are refused by
  `test_vocabulary.py` (`RETIRED_SOLVERS`), since the word table there walks keywords, not
  values.
- **Ruling 3, which sets check.** The box (so the input box), the ball and the singleton
  refuse a member of another size, read off static shapes (`z.shape[-1]` against the bound's),
  so the check traces under `jit`; a scalar or a list member is left to broadcasting. Cost: a
  NumPy grid-loop step (`f`, then `U.contains` and `X.contains`) measured 33.9 µs with the
  check and 34.0 µs without (median of 10 alternated runs, the cloud VM). `FieldInputSet` has
  no check: a field declares no input size to compare with (AS-41, `dim` where it is known,
  is where it would come from).
- **Ruling 4a, `u_bar` collides: a question for the maintainer.** A read-only `System.u_bar`
  (each input port's default) would clash with the operating point `LTISystem` and
  `DiscreteLTISystem` already store as the attribute `self.u_bar`
  (`dynamics/abstraction/state_space.py`: the input the deviations are measured from, used
  for `y_bar = C x_bar + D u_bar`), and the name means that operating point on every design
  verb, `StateFeedback`'s `params["u_bar"]`, `QuadraticCost.u_bar` and `LQRPlanner.u_bar`
  (RULES 2.7: "Operating point, target"). The two are different numbers on a linear model
  (its port default is not its operating point), and assigning `self.u_bar` would fail under a
  read-only property. Decision 11 (the operating point readable as `x_bar`, `u_bar`, `y_bar`
  everywhere) touches the same word. So `get_u_from_input_ports()` stays, with its 40 callers;
  the options: another name for the port defaults (`sys.u_nominal`, the word the input port
  already uses, `port.nominal_value`, and the evaluator's `nominal_input`); or `u_bar`, with
  the linear models' operating point moved under decision 11 first.

The run of AS-20 to AS-26 on the branch `api-settle` (2026-11-10), one commit per row. The
seeded baseline (value iteration, finite and infinite; tabular learning; a trajectory
optimization per transcription, planner and compiled face; MPPI, planner and compiled face;
an `evaluate`; a rollout-environment step; a Lyapunov `verify`; a `DiscreteLTISystem` step,
timings out) was run twice and compared byte for byte, then after each row: identical, the
only change two keys renamed by AS-21 (`status["n_iterations"]`, `effective_rollouts`).

- **The vocabulary test reaches the deep modules.** The facade scan of `test_vocabulary.py`
  does not see a class of a deep module (the transcriptions, the grid plots, the
  environment), so three small tables join `RETIRED`: `RETIRED_NAMES` (a name, or a record's
  field, at its full path), `RETIRED_KEYWORDS` (a function's retired keyword) and
  `ONE_SIGNATURE` (a method's leading parameters in the model's order); and the compiled
  face's status keys are read off `STANDARD_STATUS`. An instance attribute set in
  `__init__` (`planner.train_time`, `env.tf`, `mppi.method`) has a `RETIRED` row but no
  check: none can be read without building the object.
- **AS-20, the grid on the transcription.** `n_steps` is keyword-only
  (`DirectCollocationTranscription(n_steps=20)`); the three share a `FixedGridTranscription`
  base that holds it with the grid the record carried, under its names (`t(problem)`,
  `dt(problem)`, `offsets(problem)`, `n_knots`). `TrajectoryOptimizationPlanner.
  transcription_options()` (the verbose print's line) now returns the transcription's own
  numbers, `{"n_steps": N}`, the same text as before; **a question**: its name names the
  record that is gone (rename it, fold it into the print, or keep it).
- **AS-21.** The MPC's verbose line prints `n_iterations=`, and `MPPIRecord`'s text
  "effective rollouts"; the local variable of `experimental/mppi/pendulum_mppi.py` followed.
- **AS-22.** The helper `plot_policy_field` is `plot_control_law_field` (internal), the title
  "Control law u[0]". Kept, not listed: **`animate_control_law(axis=)`** (and the internal
  `animate(axis=)`), the same idea as `u_axis` (a question: rename it with `plot_control_law`'s);
  the module helpers `plotting.plot_value` and `plot_value_3d`, which a GRO860 notebook
  imports; `System.animate(save=True)`, a flag beside `file_name`, not a path. Left as the
  maintainer wrote it: the commented-out line of `examples/demos/value_iteration/
  vi_cartpole_swingup.py` (`# planner.plot_control_law(axis=0, axes=(1, 3), …)`, already
  stale since the 2026-11-08 sweep) and the call table of `examples/tutorial/core/
  08_planners.ipynb`, which still lists `axes=(0, 1)` for `plot_cost_to_go` (student-facing:
  for the maintainer).
- **AS-23, `final_time` checked.** It is the problem's `t0 + horizon` (the test reads 3.0 for
  `t0 = 1`, `horizon = 2`), an absolute end time, so it is `planner.tf`; the vocabulary row
  that already retired `final_time` now names `tf`. `RolloutEnvironment.tf` is
  `episode_length`, as its constructor's keyword. The internal `failure_cost_of` and
  `diverged_cost_of` take `horizon` with `score_run`. **Two more lengths named `tf`, not
  listed, left as questions:** `ProblemEvaluator.tf` (the run's length, a public attribute of
  an exported class) and `LyapunovCertificate.verify(tf=)` (how long each state is
  integrated); both read `horizon` under the rule. The GRO860 gymnasium notebook's own
  environment class keeps its `self.tf` (the notebook's code, not the library's).
- **AS-24, `period` not renamed: a question.** On a `StepSystem`, `sample_period` is the
  clock contract (`StepSystem.sample_period = None`: a bare tick law; a number: on a clock),
  read by the step diagram, the compile plan, the simulation facade and the evaluator. A
  `DiscreteLTISystem` describes a period and puts the model on no clock (its
  docstring). Measured with `sample_period = 0.1` set on one: its `compute_rollout(n_steps=3)`
  runs `t = [0, 0.1, 0.2, 0.3]` instead of `[0, 1, 2, 3]`, and `model >> other` is refused
  ("is on a clock of 0.1 s"). The options: `sample_period`, the model then clocked (what
  `linearize(sampled_loop)` returns would roll out in seconds and compose only as a sampled
  diagram); another word for a period that is not a clock (`model_period`, or `Δ`'s name in
  the docs); or `period` kept, recorded as an exception in RULES 2.7. The export landed:
  `from minilink.dynamics import DiscreteLTISystem`, in the teaching surface's list.
- **AS-25.** `NeuralPolicy.action` gains `t` (unread: a time-invariant law). The three
  callers that passed `params` positionally (`ctl`, `setpoint_position`, the RL planner's
  jitted action, one test) now name it. The Gym bridge's `action(x)`
  (`interfaces/gymnasium.py`) reads `x` only and is unchanged.
- **AS-26.** The learner's update keeps the textbook's `η` (`eta` in `update(…, eta, gamma)`
  and the moving-average line), as the first-order optimizers read
  `eta = params["learning_rate"]`; the planner's keyword, attribute and `params["method"]`
  key are `learning_rate`. `TrajectoryOptimizationRecord`'s printed text keeps "solver ok"
  (display only).

The run of AS-27 to AS-32 and ruling 9 on the branch `api-settle` (2026-11-10), one commit per
row. The seeded baseline grew by the paths these rows touch (an `Optimizer` solve per method,
`gd` and `adam` with an iteration cap and a tolerance, a fixed-count Adam; an RRT solve and its
held controller; a PID on a clock closed on the pendulum; `Impedance` and `IntegralImpedance`
loops; a `TrajectorySource` held and linear; a forced port; the step model's rollout held and
linear; an `evaluate` of a sampled and of a continuous PID), each call written in the old
words before its row and in the new ones after: run twice and compared byte for byte, then
after each row: identical.

- **AS-27, the cap on trajectory optimization.** The planner passed `options=` through to the
  `Optimizer`, so a native method there took `options={"maxiter": …}` too; with `options=`
  refused on a native method, `TrajectoryOptimizationPlanner(…, method="adam",
  max_iterations=…, tol=…)` takes the two keywords the same way (structure, kept beside the
  law's numbers as `planner.method_limits`, never in `params["method"]`), and refuses
  `options=` on a native method at construction. Adam's `max_grad_norm` (it was given in
  `options=`) is a keyword too. The first-order backend's own option keys are `max_iterations`
  and `tol`; its constructor keeps `algorithm=` (internal, beside the presets' table). Nothing
  outside one demo (`examples/demos/compile/pid_autotuning_native.py`) and four tests used the
  old words.
- **AS-28, the hold's spelling.** The noise sources already said `hold="zoh"` / `"linear"`,
  so the two others follow them: `TrajectorySource(hold=)` (default `"linear"`, the attribute
  `source.hold`, `from_trajectory(hold=)`) and the step model's `rollout(hold="linear")`.
  Kept, another idea: the grid interpolation of value iteration and `LookupPolicy`
  (`interpolation="linear"` / `"nearest"`, a spatial interpolation of a table, not a hold in
  time). The compile plan's local loop variable `input_port_id` is internal and kept.
- **AS-29, the port shortcuts of a native law.** `NativeSampledSystem` takes
  `input_dim=` and `expose_state=`; **`output_dim=` is not added: a question.** The `y`
  shortcut wires the output to `h`, and a native law's tick face `h(x, u, k)` reads its first
  output port, so the shortcut would read itself; a native law declares its outputs in seconds
  with `add_output_port`. The options: leave it so (the docstring says it), or a seconds
  output map the subclass writes (`h_seconds`) that the shortcut wires. The planner's
  `compile(backend=None)` (trajectory optimization, MPPI) keeps `backend` positional: not
  listed, and the MPC calls it so; a question if the maintainer wants it keyword-only with
  the system's. The positional callers were found by an AST scan of every `.py` file and
  notebook code cell: about 15 `plant.compile("numpy")` in the tests and the four `MLP(…)`
  calls of the RL code.
- **AS-30, `set_nominal_value`.** It was the one checked way to set a nominal value (a flat
  float copy of `dim` entries); the constructor's check is now the module function
  `nominal_of`, and an assignment `signal.nominal_value = v` is not checked, as the direct
  assignments in the catalog and the demos already were. **A question:** a checked
  `nominal_value` property would restore the check on every assignment (a scalar assigned to a
  one-entry port, as `racecar.py`'s `__main__` does, would then become a vector). RRT*'s other
  run-state copies of its record, `converged`, `best_goal_cost` and `history`, stay (not
  listed; the history is what `animate_convergence` replays); **a question** with RULES 2.4.
  `solution_node` stays (the plotting hooks read it during the search).
- **AS-31, the names.** The optimizer's panel functions are `print_program_preamble` and
  `print_program_report`, not `print_solve_…`, which are the trajectory planner's own methods
  with other bodies (a fifth duplicate the row's rule would refuse). `require_cost_to_go`
  keeps the planner's message, which a test reads ("ReinforcementLearningPlanner's solution
  carries no cost-to-go field"); the comparison's refusal now reads "this value iteration
  solution carries no cost-to-go field (its cost_to_go is None)". MPPI's refusal of a `dt` that
  does not divide the horizon is the evaluator's sentence ("dt=… must divide the horizon …
  into a whole number of periods"). The program evaluator's own `decision_vector` method stays
  (a method of another class, the same body).
- **AS-32, `show` on `plot_diagram`.** `show=True` keeps today's choice (inline in a notebook, a
  Matplotlib window elsewhere), `False` builds only, and `"inline"`, `"window"`, `"pdf"` name the
  place; any other value is refused. The commented call in
  `experimental/compile/diagram_flatten.py` (`# flat.plot_diagram(…, show_pdf=True)`) is left as
  the maintainer wrote it. `PlanningSolution.plot_control_law(**kwargs)` and the planners' own
  plots keep their `**kwargs`: decision 16 (the planner's plots) decides their keyword set.
- **Ruling 9.** `Impedance(Kp=10.0, Kd=1.0, *, dim=1, tracking_ref=False, u_min, u_max, …)`
  and `IntegralImpedance(Kp=10.0, Ki=1.0, Kd=1.0, *, dim=1, …)`, the defaults kept; the
  attribute `.dof` → `.dim`, as on the PID family. `JointImpedance` and `TaskImpedance` take the
  plant and have no `dof=` keyword; their attribute `.dof` is the plant's joints, the meaning
  RULES 2.7 keeps, so they are unchanged. Eight test calls passed `dof=`; no example did.

The run of rulings 11, 13 and 7 on the branch `api-settle` (2026-11-10), one commit per step.
A new seeded baseline (`linearize` on the pendulum, the cart-pole and a sampled pendulum;
`lqr`, `place`, `lqr_finite_horizon`, `kalman`, `luenberger` and `lqg` on the plant and on its
linearization; `controllability`, `observability`, `trajectory_lqr`; `bode`, `margins`,
`pzmap`, `step_response`, `transfer_function`, `root_locus` and `nyquist` on the plant, the
linearization and the step linearization; the loops `lqr(plant) @ plant` and `lqr(lin) @
plant`), and one of the catalog (each plant's `f`, `h`, `tf` and a short run), each run twice
and compared, then after each step: byte-identical, with the earlier run's baseline.

- **Ruling 11, the analysis numbers.** A linearization at its own point is read as stored
  (`stored_matrices`), not differentiated again, so its numbers are exact on every
  `method`: with `method="fd"` a Jacobian about `x̄ = π` would move the Bode numbers by
  `1e-9` against the deviation model's about the origin. A point given (`bode(lin, x_bar)`)
  or a diagram wire selected takes the Jacobian path.
- **Ruling 11, a point given with a linear model.** `require_own_point` refused it; with it
  gone, the point linearizes the model there (`lqr(lin, Q, R, x_bar=x1)` is the same gain
  about `x1`, the drift `f(x1, ū)` warned when it is not zero), as for any system; no point
  reads the model as it is.
- **Ruling 11, the step twin's drift.** `LinearizedStepSystem` keeps `x_next_bar = F(x̄, ū)`
  in its `params` (not `dx_bar`, a rate), and warns off an equilibrium when
  `F(x̄, ū) ≠ x̄`; a plain step system's design is not warned, as before.
- **Ruling 11, the ports.** The linearization declares the plant's input ports selected by
  `wrt` (a port, its components, every input), each at its share of `ū`; a diagram wire, or a
  port named `x` or `y`, keeps the one port `u`. The output is always `y` (the selected rows,
  with their labels and units) and the state is always exposed as `x`, as `LTISystem` did, so
  `lqr(lin) @ lin` closes on a plant without an `x` port too. The state's bounds are not
  copied (the model is valid near `x̄`); the input ports' are.
- **Ruling 11, the warning's text.** One message for the designs and the analysis: "does not
  rest at the design point: |f(x_bar, u_bar)| = …. The linear model (A, B, C, D) drops that
  term: its poles and responses are those of the deviations, and a loop designed on it will
  not hold x_bar …". Warned once per calling line (Python's default filter, the caller's
  stack level).
- **Ruling 11, a forgotten `()`.** The library's own array verbs (`lqr_gain`, `place_gain`,
  `lqr_gain_schedule`, `kalman_gain`, `luenberger_gain`, `controllability_matrix`,
  `observability_matrix`, `analysis.linear`, the `LTISystem` constructors) refuse a method
  given for its matrix ("A is a method of the model, not its matrix: call it, sys.A()").
  NumPy's own `np.linalg.eigvals(lin.A)` cannot be reached; KNOWN_ISSUES keeps that entry,
  its fix "none", and TODO's row on the matrices left.
- **Ruling 11, the names.** The deviation model is named `"<name> (deviations)"`, its labels
  `Δθ` (they read `Delta θ` on the old model); an observer's labels drop no prefix any more.
  `LinearizedSystem` and `LinearizedStepSystem` are on `minilink.dynamics` beside
  `LTISystem`. The step response of a linearization reads "Linearized Step Response", its
  axis "Deviation Δy", its hover `Δy`.
- **Ruling 11, the GRO501 cart-pole notebook: a question.**
  `udes_gro501/cartpole_dynamic_controller.ipynb` uses no removed name (`linearize`, `ss.A()`
  … still work), so it is left as the maintainer wrote it: its observer subtracts
  `[0, π, 0, 0]` from `y` by hand and plots the deviation estimate beside the absolute plant
  state. With the new model the cell could design on `linearize(sys, x_bar)` and read
  `ss.x_bar`; that is a change of the teaching code, the maintainer's call.
- **Ruling 13.** A scalar level is broadcast to the other's size (`Step([1.0, 2.0])` starts
  at `[0, 0]`). Nine positional calls flipped (two demos, two tutorial notebooks, five
  tests). The bare `Step()` calls whose numbers move (a unit step at 1 s, was zero): the
  tests `test_diagrams` (the controller box's loop), `test_graphics` (a signal-recording
  loop), `test_simulation` (a static simulator's grid), `test_compile`, `test_core` and
  `test_feedback_composition` (a refusal or a name), and the demos `diagram_compiling.py`
  (its params overwritten, unchanged) and `diagram_shortcuts.py` (`Step()` and
  `Step(port="r")`, drawn only). None asserted a value, so no expected number changed.
- **Ruling 7, the order of the two plants.** `[dy, y, x]` → `[x, y, dy]` (the planar
  position in the drawing's order, then the rate) on `QuarterCar` and
  `ConstantSpeedHelicopterTunnel`; no demo or notebook uses either; one test's state and
  index followed. `ThreeBodyProblem` keeps its per-body `[r1, v1, r2, v2, r3, v3]` (not one of
  the two; **a question** whether the rule reads per body or for the whole state).
- **Ruling 7, the labels.** `GeneralizedMechanicalSystem`'s velocities read `v1…` (its `v`
  is not `dq`); the manipulator's task ports `p1…`, `dp1…` (they read `pdot0…`); the robotic
  and model-based laws' `q1…`, `dq1…`. Left: `DoublePendulum` inherits the base's `u1`, `u2`
  (its siblings say `tau1`, `tau2`, a catalog-hygiene row); the laws' own internal state labels
  (`e_int0`, `pos0`, `rate0`, `u0` in the PID and impedance families) are counted from 0, not a
  plant's signals; `DoubleIntegrator`'s `speed` and other plant-chosen rate names were not
  renamed (the rule allows the textbook's word; a catalog-hygiene pass decides). The
  plots already bracket a bare unit and leave a bracketed one alone.

The run of rulings 15, 12 and 5 on the branch `api-settle` (2026-11-10), one commit per ruling.
The seeded baseline grew by the paths they touch (each optimizer method on a NumPy and a JAX
program, the constrained class and `"auto"`, trajectory optimization on each SciPy method and
its default, MPC loops warm, cold and delayed, MPPI in the loop, ports declared with bounds, a
diagram's state box, a linearization's input bounds, a lookup policy's ports, a manipulator's
torque drawing), each call written in the old words before its ruling and in the new ones
after: run twice and compared byte for byte, then after each ruling: identical. No number
moved.

- **Ruling 15.** A plain `__init__` builds the three blocks the factory built: the loop's `%`
  takes a `copy.copy` of the box, which runs no constructor, so no `__new__` was needed. The
  class docstring is the factory's, with the old class's note on the pure blocks. Nine files
  imported it by the deep path (two projects, the pathtracking demo, three `experimental/`
  scripts, the MPC parity scenario, `docs/make_assets.py`, one test) and one notebook
  (`mpc_spatial_stack.ipynb`); the other callers already used `minilink.control`. Where
  `PlanFollower` sat in the same deep import (the dual-rate project, the parity scenario), it
  moved to `minilink.control` with it; the deep imports of the drawing helpers
  (`mpc_animation_overlays`, `plant_run`) stay, they are on no facade.
- **Ruling 12, the table.** `METHODS[(method, backend)]` holds every pair: each algorithm runs
  today on NumPy and JAX programs (SciPy and Ipopt on the host, the first-order loop on the
  program's array module), so the refusal of a missing pair ("the methods on 'x' are: …") is
  reached only by a backend with no row. The pair is read off the compiled evaluator's
  backend (normalized), so `backend="NumPy"` still works and an unknown backend still fails
  at the compile, with today's message. `make_optimizer_backend(method, *, backend="numpy")`
  defaults to NumPy for the callers that hold no program (trajectory optimization reading a
  native law's numbers, two tests). An unknown method keeps its sentence, now listing `slsqp`
  and `trust-constr` (the vocabulary test reads it for the retired strings).
- **Ruling 12, the benchmarks.** The solve-speed suite's ids
  `optimizer.<case>.scipy_slsqp.*` are `optimizer.<case>.slsqp.*`, in the suite and in
  `benchmarks/baselines/solve_speed.json` (ids and notes renamed, no number touched); the
  regression gates pass on them. A demo's commented-out `# method="scipy_slsqp"` line was
  renamed with the calls (a renamed call, not a tuning). `library=` is not added.
- **Ruling 5.** A word-boundary rename of `lower_bound` / `upper_bound` (905 sites), then a
  review: no other library's keyword used the words; the only bare names were the
  manipulators' local copy of the torque bound passed to `planar_torque_geometry`, renamed
  `upper` with it; an AST scan found no function where the renamed keyword collides with a
  local of another meaning. The signal's size check now names the field `lower` / `upper` in
  its message. The `System` constructor has no bound keyword, so nothing there changed.
  `Saturation`'s `lower` / `upper`, the sets' and the program's were already the word; the
  laws' signal limits (`u_min`, `u_max`, `delta_max`, `e_int_min`, `e_int_max`) are
  unchanged.

The run of AS-40 to AS-49 and ruling 16 on the branch `api-settle` (2026-11-10), one commit per
row. The seeded baselines of the earlier runs (`baseline_r.py` at rulings 5, 12 and 15, and
the ruling-11 analysis baseline) were run twice and compared before the first edit, then after
each row: byte-identical every time (the additions move no existing number).

- **AS-40.** `QuadraticCost(Q, R, S=None, x_bar=None, u_bar=None)`: the zeros are sized by `Q`
  (`n × n`) and `R` (`m × m`); a call that gives all five is unchanged.
- **AS-41, `dim` where it is known.** On `BoxInputSet` (with `lower`, `upper`), an intersection
  (its first member that states one), a field set and a margin field (what they wrap), the
  quadratic field (its center) and the grid field (its `n`); none on a callable set or a field's
  input set (no input size is declared). `BallSet.sample` draws as the ellipsoid does (a
  Gaussian direction, a radius `s^(1/n)`; a counter key refused, the ellipsoid's refusal now
  names both); `SingletonSet.sample` gives its point whatever the key. **A ball is now a start
  a problem draws from** (`x0=BallSet(...)` was refused as unable to draw): the test that pinned
  the refusal refuses an intersection instead, and the refusal names the sets that draw.
- **AS-42.** `CostFunction.h` returns `xp.zeros(())` on `x`'s array module. The library costs
  that write `return 0.0` (`TimeCost`, `FieldCost`) keep it, and the demos' own `return 0.0`
  are left for the rebuild.
- **AS-43.** `Gain(K, dim=1)` (the test that pinned `Gain(2.0)`'s refusal now pins the one
  channel). `Integrator(dim=1)` writes `ẋ = k u`, `y = x` on the whole vector (the same floats
  on one channel). `Saturation(lower, upper, dim=None)`: `dim` is the bounds' size, 1 for
  scalars; bounds of two sizes, or a `dim` the bounds contradict, are refused; a scalar stop
  stays a float in `params`.
- **AS-44, the `System` shortcut not added: a question.** `sys.poles(...)` would be shadowed
  where `poles` already means an array: `TransferFunction` (and `LowPassFilter`, `Lead`) store
  `self.poles` in `__init__`, and `StateObserver.poles` is a property (the error poles,
  `eig(A − L C)`). The options: a method on every system, the two attributes renamed or turned
  into it (a public change); a read-only property `sys.poles` at the nominal point, which the
  observer's and the transfer function's already are (`TransferFunction` would then stop
  assigning it); or the band function only, as landed. The function is `poles(sys, x_bar, …)`
  on `minilink.analysis.modal`, through `linearize_matrices`, so a linearization is read as
  stored and a step system gives its poles in `z`. `StateFeedback` and `OutputReference` print
  `K` under the block and read it as a read-only `K`; no caller assigned `.K` on them.
- **AS-45.** `linearize` and `transfer_function` moved to a `LinearAnalysis` mixin that
  `DynamicAnalysis` and `StepAnalysis` inherit (the band functions already read a step system;
  ruling 11 had given it `LinearizedStepSystem`). **Found, a question:** `transfer_function` on
  a step system returns a continuous `TransferFunction` block holding `G(z)`'s coefficients,
  printed `G(s)`; simulating it reads them in `s`. Logged in KNOWN_ISSUES; the options are a
  discrete transfer-function block, or the verb refused on a step system.
- **AS-46.** `add_subsystem(sys, sys_id=None)` on the flow, step and sampled diagrams, through
  `DiagramWiring.new_subsystem_id` (the operators' `default_subsystem_id`, `unique_id`, and the
  reserved-role check before numbering, so a second `plant` is refused, not renamed `plant2`);
  it returns the id. `StepPlanningProblem.replace` keeps `origin` (the continuous problem a
  discretized one samples, which gives its cost bound and design point) only when the change is
  the start, `X0`, the failure's price or the criterion. The sampled-input refusal names
  `n_samples=` and `dt=` only when the rows are right and the sample count is off the grid.
- **AS-47.** LQR, RRT, RRT* and MPPI print their record once when `verbose=True` (none has a
  progress of its own; the tabular learner already ends so). `callback` lists through one
  helper, `core/reporting.one_callback`, on `Optimizer.solve` and trajectory optimization (RRT's
  own list handling kept). `show=` added where a plot had none: `Source.plot_signal` and both
  learning curves; the learning curves now show a figure they make by default (they left it to
  the next plot), `show=False` builds only. The planners' `**kwargs` delegations
  (`plot_trajectory`, `plot_cost`, `plot_tree`, the scene and track plots) already reach a
  `show=`.
- **AS-48.** All four sensitivity functions take `(plant, controller, x_bar, u_bar, t, params,
  *, filter=, of=, wrt=, method=, eps=)`; the plant is read by `transfer_function(…, of=, wrt=)`
  (bode's channel), the controller and filter keep the SISO refusal. Keyword callers unchanged.
- **AS-49.** Root: `LTISystem`, `LinearizedSystem`, `MechanicalSystem`, `step_info` (and
  `poles`, AS-44); `minilink.analysis`: `Margins`, `StepInfo`; `minilink.core`: the four
  shaping functions. Each in the teaching-surface registry; their modules were already on the
  API pages.
- **Ruling 16, the keyword set.** Written out on `PlanningSolution`, the base `Planner` (so LQR,
  RRT, trajectory optimization, MPPI, RL and the tabular learner, whose overrides were deleted)
  and value iteration (plus `field=`): `plot_cost_to_go(*, x_axis=0, y_axis=1, anchor=None,
  bounds=None, grid_shape=(51, 51), t=0.0, jmax=None, is_3d=False, trajectory=None, title=None,
  ax=None, renderer="matplotlib", show=True)`; `plot_control_law` the same with `u_axis=0`
  after `y_axis`, `x_axis=None`, `y_axis=None` (the controller's own default sweep),
  `grid_shape=(101, 101)` and no `jmax`. Beyond the ruling's six keywords, the set keeps what
  the callers used today (`anchor`, `bounds`, `grid_shape`, `t`, `jmax`, `trajectory`, `title`):
  the course notebooks pass `jmax=`, `title=`, `trajectory=`, and a demo `anchor=`. Dropped from
  the planners' plots: `cmap`, `vmin`, `overlay_labels` (no planning caller; still on the grid's
  and the controller's own plots). `renderer` other than `"matplotlib"` is refused (the planning
  plots have no Plotly path).
- **Ruling 16, which drawing.** A grid planner's solution (its record carries `tables`) is drawn
  on the grid's own nodes, as value iteration drew it, unless the call asks for another mesh
  (`bounds`, `grid_shape`), a time, or a control law's surface: then the field (or the
  controller's law) is sampled, as for any other solution. So the planners' figures are the
  ones they drew before, and `sol.plot_*` of a grid solution now draws the table too (it
  sampled the field). `jmax=None` tops the scale at the field's `outside` value (the failure
  cost), as value iteration's and the learner's defaults did; `jmax=np.inf` is Matplotlib's own
  scale (it was `jmax=None` on the planners; one test migrated). A finite horizon's table is
  stage 0. `anchor` on a law the controller draws is refused (it pins the other coordinates at
  their nominal values). A batch of slices (lists of axes, the grid path only) returns a list
  of `PlotResult`.
- **Ruling 16, the returns.** `PlotResult` as the library builds it (`.figure`, `.axes`; the
  ruling's `.fig` is `.figure` on today's record, no alias added). The comparison's three plots
  return one too (`.axes` its list of panels): they were the third shape. Out of scope, as
  ruled: the grid's own `StateSpaceGrid.plot_cost_to_go` and the module functions of
  `dynamic_programming/plotting.py` still return `(fig, ax)` (a GRO860 notebook unpacks them),
  and the learning curves return their axis. Callers migrated: four tests and the call table of
  `examples/tutorial/core/08_planners.ipynb` (its planner, solution and comparison rows); no
  example or course notebook unpacked a planning plot.

The run of ruling 17 on the branch `api-settle` (2026-11-10), one commit per step. A new seeded
baseline (every field of every planner's record, timings left out, under the pre-ruling names:
value iteration infinite, finite, receding and on the loop engine with its tables; the tabular
learner's records, its live table and its solution's tables after more learning; the LQR,
infinite and finite; RRT and RRT*; trajectory optimization with its history; MPPI; the neural
learner; `evaluate` on a stochastic problem), run twice and compared, then after each step:
byte-identical.

- **Ruling 17, the tabular learner's live table.** Its `result` returned a `ValueTables` built
  from `Q` (`J = min Q`, `pi = argmin Q`), not the table: the table itself is already
  `planner.Q`. Named for what it is, `planner.greedy_tables`, rather than the ruling's
  `q_table`, which would read as `Q`. `planner.tables` reads the latest solution's, as on value
  iteration; the last-answer test lists both grid planners' `tables` as shortcuts.
- **Ruling 17, the learners' `episode_cost`.** Neither record held the last episode's cost: the
  tabular one is the mean over the last tenth of the episodes, the neural one the mean episode
  cost of the last collection. Both keep the ruling's name, `episode_cost`, documented as that
  mean (their `str` already said "mean episode cost"); `mean_episode_cost` is the alternative.
  The per-episode and per-iteration `history` dicts keep their `"cost"` key.
- **Ruling 17, MPPI's and trajectory optimization's `cost`.** Both are the objective at the
  returned plan, the quantity of the `Optimizer`'s `cost`, so they keep it.
- **Ruling 17, the evaluation's count.** `J`: every statistic, the objective and about thirty
  reads in tests, examples and benchmarks; `cost`: two test reads and a tutorial's table. They
  differ (`J` adds the failure cost of a failed or missed run under `penalize_failure`), so both
  stay and `cost` is renamed by its quantity, `raw_cost` (the tutorial's own "raw integral +
  terminal cost"), in the record, `score_run`'s row and formula and the step problem's scoring;
  the one-run `str` reads "raw cost = …".
- **Ruling 17, what `solve_time_s` times.** The wall time of the user's verb without the
  evidence's scoring: value iteration's sweeps, the Riccati design, the search, MPPI's compile
  and solve until the plan is on the host, the learners' training (the tabular one with its
  greedy law). A first JAX call's compilation is inside it (value iteration on JAX, MPPI, the
  neural learner); trajectory optimization keeps its own, the optimizer's backend solve after
  the warm-up. `None` on a compiled face's solve. The neural learner's `train_time_s` stays: it
  counts every training of the planner, collections and updates only, with `n_timesteps`.
  Each record's `str` ends with "; solved in … s" (the LQR's before "; U ignored").

The run of ruling 18 on the branch `api-settle` (2026-11-10), one commit per step. A new seeded
baseline (PPO, REINFORCE, actor-critic and SAC on the bounded pendulum, two solves each with an
evaluation, the records, weights and laws; a given actor and its rebuild at another tree; a
tabular run, twice), name-agnostic, run twice and compared, then after each step:
byte-identical.

- **Ruling 18, the collector's words.** The internal `rollout(…, n_steps, n_envs)` and
  `reset_carry(…, n_envs)` take `n_rollouts`; the collector's `n_steps` stays, since there it is
  the rollout's own length, a horizon's transitions. PPO's refusal reads "n_steps_per_update *
  n_rollouts must be a multiple of batch_size". The algorithms' parameter tree keeps its
  `"policy"` key: it is the exploration policy's `theta` (`mlp` and `head`), not the actor.
- **Ruling 18, `learn` into `solve`.** `learn` trained without handing out a solution; `solve`
  already trained through it, continuing where the last training stopped. The loop moved into
  `solve`, with no private `learn` left (a renamed helper would keep two verbs);
  `learn(n)` → `solve(n_timesteps=n)`, its one cost a copy of the law per call. Checked on the
  base: `learn(64)` then `solve(64)` and the new `solve(64)` twice give the same weights,
  records and evaluation (PPO and SAC). A test asserts that a second solve continues the count.
- **Ruling 18, `predict`.** Nothing native called it but the tests. Its deterministic branch
  is `solution.controller` or `planner.actor.action(x)`; its `deterministic=False` branch, an
  exploration draw, is reachable as `planner.exploration_policy.sample(theta, x, key)` then
  `.input(a)` (the test now says so). The bridge needed no change: `SB3Controller` already
  reads an SB3 model's `predict`. The jitted `action_jit` it alone used is gone.
- **Ruling 18, scope.** The tabular learner's own `learn(n_episodes)` was left, the ruling
  naming the RL planner; a question for the maintainer whether it folds into `solve` the same
  way. Course notebooks: the code cells only (`drone_ppo.ipynb`'s constructor and two training
  cells, `pendulum_value_iteration_vs_lqr_vs_rl.ipynb`'s constructor); SB3's own `learn` and
  `predict` in the SB3 notebooks are SB3's and stay.
