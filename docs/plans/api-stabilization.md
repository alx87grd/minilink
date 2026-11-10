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

| Step | Today → proposed | The rule | Uses |
| --- | --- | --- | --- |
| AS-20 | `DirectCollocationTranscription(DirectCollocationOptions(n_steps=20))` → `DirectCollocationTranscription(n_steps=20)`; the same for shooting and multiple shooting; the four option records deleted | 2.5, 2.7; the 2026-11-04 sweep removed the other option records | 37 calls |
| AS-21 | The compiled planner's status key `"iterations"` → `"n_iterations"`; `MPPIRecord.effective_samples` → `effective_rollouts`; `StateSpaceGrid.nodes_n`, `actions_n` → `n_nodes`, `n_actions`; `LyapunovCertificate.verify(n=)` → `n_rollouts=`, its plots' `n=` → `n_samples=` | 2.7, every count is `n_<thing>` | 12; 10; 72; few |
| AS-22 | `plot_sampled_futures(axes=)` → `x_axis=`, `y_axis=`; value iteration's `plot_control_law(axis=)` → `u_axis=`; the animations' `save=` (RRT) and `save_path=` (value iteration) → `file_name=`; the plot title "Policy u[0]" and `plot_policy_field` → control law; `StateSpaceGrid.plot_value` → `plot_cost_to_go` | 2.7 | small |
| AS-23 | `tf` as a length in `score_run(tf=)`, `planner_evidence(tf=)`, `steps_of(tf, dt)`, `RolloutEnvironment.tf`, DESIGN's scoring formula → `horizon` or `episode_length`; `DynamicProgrammingPlanner.final_time` → `tf` | 2.7, `tf` is absolute | internal; 24 |
| AS-24 | `DiscreteLTISystem(period=)`, `.period` → `sample_period`; the class exported on `minilink.dynamics` | 2.7 | 11 |
| AS-25 | `RolloutEnvironment.step(x, t, u, key, params)` → `(x, u, t, key, params=None)`; `LookupPolicy.action(x, params=None, *, t=0.0)` and `NeuralPolicy.action` → `action(x, t=0.0, params=None)` | CONSTITUTION, one signature | about 15 |
| AS-26 | `MPPIPlanner.method` (a dict of numbers) → `method_numbers`; `TabularLearningPlanner(eta=)` → `learning_rate=` (TODO's judgment-calls row names it); `planner.train_time` → `train_time_s`; `TrajectoryOptimizationRecord.solver_success` → `optimizer_success` | 2.7 | 8 examples |
| AS-27 | The optimizer's native numbers: `options={"maxiter": 5}` on `gd` / `adam` → `max_iterations=5`, `gtol` → `tol`; `options=` refused for a native method; `OptimizationResult.n_iterations` added beside `stats["nit"]`, `stats["algorithm"]` → `stats["method"]` | 2.7 | few |
| AS-28 | `plot_rollout(rollout=)` → `traj=`; `input_port_id=` → `port=` (blocks already say `port=`); `TrajectorySource(interpolation="previous")` and `ev.rollout(hold="foh")` → `hold="zoh"` / `"linear"` on the three tools | one word per idea | 5; 25; 14 |
| AS-29 | Keyword-only where every sibling already is: `PlanningProblem(sys, *, x0, cost, …)` (0 callers pass a second positional); `compile(*, backend=, verbose=)`; `animate(traj=None, *, …)`, `render(x, u, t, *, …)`; `MLP(input_dim, output_dim, *, …)`; `Sys2Gym(sys, cost, *, …)`; `NativeSampledSystem(state_dim=0, *, sample_period, …)` with the port shortcuts. And the reverse: `evaluate(problem, controller, dt=None, *, …)`, as DESIGN §6 writes it and `problem.compile` takes it | DESIGN §4, sibling verbs agree | positional callers only |
| AS-30 | Removed, with no caller or against a rule: `validate=` on `feedback`, `closed_loop`, `autowire` (0 uses; it switches off the algebraic-loop check); `PlanningProblem.has_goal`, `has_cost` (0 library uses); the `COMPILE_BACKEND_AUTO` re-export "so legacy callers keep working" (3.4); `InputPort.get_default_value`, `set_nominal_value` (duplicates of `nominal_value`); `RRTPlanner.reached_goal`, `iterations` (copies of the record, RULES 2.4; 31 / 24 / 5); `GivenPolicy` → `GivenController` | 2.4, 3.4, 4.10 | as noted |
| AS-31 | The problem's five static helpers and the `Optimizer`'s four machinery methods → module functions; the duplicate helper names with different bodies (`require_cost_to_go`, `planner_parts`, `steps_of`, `problem_leaves`) → one each; `SampledSystem.counts`, `resolves` → `is_tick_law`, `is_integrated`; `is_bounded(box)` → `has_finite_face` | ROADMAP 2026-11-07; readable predicates | library only |
| AS-32 | The explicit-wiring drawings written out: `plot_input_output_map(**kwargs)`, `plot_control_law(**kwargs)`, `SampledDiagramSystem.animate(**options)` → their keywords; `plot_diagram(show_inline=, show_pdf=)` folded into `show` | DESIGN §4, "written out so every editor shows them" | 27 |

## 3. Additions that remove friction (nothing breaks)

| Step | Addition |
| --- | --- |
| AS-40 | `QuadraticCost(Q, R)` works: `S`, `x_bar`, `u_bar` default to zero (today five required positionals, so every call goes through `from_system`) |
| AS-41 | `lower`, `upper`, `dim` on `BoxInputSet` and `Uniform` (`plant.input_box.lower` fails today, `plant.state.box.lower` works); `dim` on every set and field where it is known; `BallSet.sample`, `SingletonSet.sample` |
| AS-42 | `CostFunction.h` defaults to `0.0` (a running cost no longer writes `return 0.0`) |
| AS-43 | `Gain(2.0)` works (`dim=1`, as `Sum`, `Saturation`, `UnitDelay`); `Integrator(dim=1)`; `Saturation` takes vector bounds |
| AS-44 | `poles(sys, …)` on `minilink.analysis`, the root and as a `System` shortcut (the examples write `np.linalg.eigvals` 22 times); `print(lqr(…))` shows `K` as an observer shows `L`, with a read-only `K` |
| AS-45 | `plot_nyquist` as a `System` shortcut (the fifth of five one-channel plots); `linearize` and `transfer_function` on a `StepSystem` (the plots already work there) |
| AS-46 | `diagram.add_subsystem(sys)` without an id (the operators' rule); `StepPlanningProblem.replace`; a scalar `u` on a one-input step rollout; the shape error of a sampled `u` naming `n_samples=` / `dt=` |
| AS-47 | `verbose=False` on every planner's `solve` (missing on LQR, RRT, MPPI); `callback` a callable or a list everywhere; `params=` on `solve_steps`; `show=` on every plot |
| AS-48 | `sensitivity(plant, controller, …)` positional, with `of=` / `wrt=` and the channel default of `bode` (today keyword-only, and it refuses the default pendulum the sibling verbs accept) |
| AS-49 | On the facades: `LTISystem`, `MechanicalSystem`, `step_info` on the root (the design verbs' own error tells the student to build an `LTISystem`); `Margins`, `StepInfo` on `minilink.analysis`; the shaping functions on `minilink.core` |

## 4. Decisions for the maintainer

Each changes a public name, a shape or a number a student meets. They are put one at a time,
with the code before and after, once the plan is approved. The recommendation is the agent's;
TODO's "API sweep's judgment calls" row is folded in here.

| # | The question | Today | Recommendation |
| --- | --- | --- | --- |
| 1 | **Block or subsystem?** One idea, two words: `add_subsystem`, `.subsystems` beside `diagram.of(block)`, "blocks:" in the print, RULES and DESIGN | 22 / 206 / 114 calls | **Ruled 2026-11-10: `subsystem`**, the one word for a system placed in a diagram: the print (`subsystems:`), the internal helpers (`block_roles`, `block_run`, `blocks_params`, …) and DESIGN / RULES prose where "block" means that; `get_subsys_input_port` → `get_subsystem_input_port`, `compute_subsys_output_port` → `compute_subsystem_output_port`. "Block" keeps one meaning, the standard components of `minilink.blocks` (the package keeps its name). No other public rename |
| 2 | **The simulator's solver names.** `Simulator` takes `"rk4_fixedsteps"`, `"euler_fixedsteps"`, `"scipy"`…; `discretize`, the evaluators and the sampled loop take `"rk4"`, `"euler"`. `compute_trajectory(solver="rk4")` is refused | 83 uses | **Ruled 2026-11-10: `"rk4"`, `"euler"` everywhere**, both fixed-step (the step model's words); `rk4_fixedsteps` and `euler_fixedsteps` gone, no alias; the variable-step Euler dropped (a non-uniform grid on a fixed-step solver refused, naming `scipy` or a uniform `dt`), after a search confirms no caller; the SciPy presets unchanged |
| 3 | **`InputSet.margin(u, x, t)`** reverses the model's order; `U.margin(x, u)` returns numbers, silently wrong | about 10 library calls | **Ruled 2026-11-10: the member first, for every set** (`X.margin(x, t)`, `U.margin(u, x, t)`), written into CONSTITUTION's one-signature paragraph ("a set's `margin` takes its member first, then the arguments it depends on"), plus a shape check that refuses a member of the wrong size; no call site changes |
| 4 | **The `get_` methods a controller or plant author writes:** `get_port_values_from_u(u, "r", "y")`, `get_u_from_input_ports()`, `traj.get_signal(name)`, `get_kinematic_geometry`, `get_dynamic_geometry`; and `tf`, which is the pose map, a final time and almost a transfer function | 23; 36; 15; 93; 143 | **4a ruled 2026-11-10:** `get_port_values_from_u(u, …)` → `self.ports(u, …)` (the same three return forms), `get_u_from_input_ports()` → the read-only property `sys.u_bar` (each input port's default), `traj.get_signal(name)` → `traj.signal(name)`; no alias. **4b ruled 2026-11-10: no rename** — `tf`, `get_kinematic_geometry` and `get_dynamic_geometry` stay as they are (the drawing hooks keep their names; `tf` the pose map beside the final time `traj.tf` is a recorded exception of RULES 2.7); S44's one hook stays v0.9 |
| 5 | **Bounds: three spellings.** `lower` / `upper` (sets, `Saturation`, the program), `lower_bound` / `upper_bound` (ports), `u_min` / `u_max` (PID), `delta_max` (path tracking) | 833 for the ports | `lower` / `upper` on ports too, only if done now; otherwise record the exception |
| 6 | **The catalog's vocabulary.** Mass is `"mass"` on 13 plants and `"m"` on 2; damping `d`, `b`, `damping`, `damping_coef`; drag `cda`, `CdA`, `cdA`; `Pendulum(length=, mass=)` writes keys `l`, `m`; `Pendulum()` has `I = 1` beside `m = l = 1` (ω = 2.21, not √(g/l) = 3.13) | every `params[…]` in a new example | One rule (constructor keyword = `params` key); the textbook letters `m`, `l`, `I`, `d`, with `gravity`; `Pendulum`'s `I = 0` |
| 7 | **Labels and units.** `"Torque 0"`, `"[Nm]"` from the mechanical base against `"tau1"`, `"Nm"` on the plants; rates `dtheta`, `omega`, `yaw_rate`; two plants rate-first | plot axes | Bare units; positions then rates; `d<position>`; the base's defaults `q1`, `dq1` |
| 8 | **The mechanical base's names** (S32's renames, pulled forward): `C(q, v)` on the base against `C(q, dq)` on 26 plants; `actuators=` a count; the mass matrix's one name | every plant | The renames now, the class hierarchy later as planned |
| 9 | **`Impedance(dof=1, *, Kp, Kd)`** against `PD(Kp, Kd, *, dim=1)`: `Impedance(10)` is a 10-axis block | 8 tests | `Impedance(Kp, Kd, *, dim=1)`, one family as DESIGN says |
| 10 | **The path-tracking laws** default to the UdeS racecar (`wheelbase=0.34`, `state_dim=9`) and close on no catalog car with `@` | 60 calls | Take the plant, as the robotic laws do; decided with the car family (K8-0) |
| 11 | **The linear model.** `lin.A` is a method (`lin.A()`); the operating point lives in three places (`LTISystem` attributes, `StateFeedback`'s port, `params["x_bar"]` elsewhere) | KNOWN_ISSUES | `x_bar`, `u_bar`, `y_bar` readable as properties everywhere; `A` kept a call, or a property at the block's own `params` |
| 12 | **The optimizer's method names:** `"scipy_slsqp"`, `"scipy_trust_constr"` beside `"l-bfgs"`, `"gd"`, `"ipopt"` | 55; 7 | `"slsqp"`, `"trust-constr"`: named by algorithm, as DESIGN says |
| 13 | **`Step(1.0)` steps down** (`initial_value` first) and `Step()` is zero | 9 positional, 16 bare | `Step(final_value=1.0, initial_value=0.0)`: `Step()` the unit step |
| 14 | **Riccati on a discrete model:** refuse (AS-1) or solve the discrete equation | none | Refuse now; the discrete designs with the sampled filters (v0.9) |
| 15 | **MPC's name and place.** `ModelPredictiveController` is a function returning a `PredictiveController`, on no facade; examples import the deep path 12 times | 7 tests | The class itself; on `minilink.planning` if the rebuilt examples show MPC |
| 16 | **The planner's own plots.** Value iteration and tabular learning override `plot_cost_to_go` / `plot_control_law` with other keywords and another return type than the solution's; the plots return three shapes (`PlotResult`, `(fig, ax)`, `ax`) | few | One keyword set and `PlotResult` everywhere |
| 17 | **Records.** `DynamicProgrammingResult` / `record.tables` / `planner.result` (which means two things between value iteration and tabular learning); `cost` is five different quantities across records; timings on two records only | 60 | `ValueTables`, `planner.tables`; `path_cost` on RRT; `solve_time_s` on every record |
| 18 | **The learners' words.** `n_envs`, `n_steps` (periods per update, where `n_steps` is the horizon everywhere else), `policy=` that becomes `planner.actor`, `learn` and `predict` beside `solve`; `Sys2Gym` | 67; 45 | `n_rollouts`, `n_steps_per_update`, `actor=`; `predict` to the SB3 bridge; `Sys2Gym` kept (research lane) |
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
