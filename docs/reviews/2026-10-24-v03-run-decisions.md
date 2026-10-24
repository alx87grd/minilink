# The v0.3 overnight run: decisions to review (2026-10-24)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** the long run of TODO §4 on `dev-v0.3` (the names and structure, the step model,
the path to the cost, the program export, the optimizer, the closing PID example), asked for on
2026-10-23 with the instruction: where a review would have been asked, decide as the maintainer
would, write the decision here, and continue. Rulings R1–R90 are in
[the rulings record](2026-10-22-rulings-session-1.md).

**How to read it.** §1 is the morning summary. §2 lists every decision taken without the
maintainer, each with the step, the choice, the reason, the alternative and the commit to revert.
§3 is the ledger of intended number changes (R80: the same problem and cost before and after).
§4 is what was not done or is blocked. §5 lists the tests skipped in this container.

**Decision rules used** (in order): the rulings, read literally; the Constitution's precedence
(purity, closure, continuous first, textbook readability); the course notes' vocabulary; the
shortest clear name; an existing pattern over a new one; one path, no alias; when torn, the
reversible and smaller option. Never without the maintainer: removing what no ruling removes,
tuned constants in examples, CONSTITUTION, the demo pass (K4).

## 1. Morning summary

**What works.** The pipeline you asked for runs end to end on `dev-v0.3`, every step pushed:

```python
problem = PlanningProblem(plant, cost=cost, tf=TF)          # the task, a plain class
pe = problem.compile(loop_or_controller, dt)                 # the problem evaluator
report = pe.evaluate()                                       # one run (or n_trials draws), the Evaluation
jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)      # every gain, plant parameter, cost weight, start
program = pe.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"])
result = Optimizer(program, method="gd").solve()             # or "adam", "l-bfgs", "auto"
tuned = pe.unpack(result.z, wrt)                             # written back, judged by pe.evaluate
```

The closing example (`examples/demos/compile/pid_autotuning_jax.py`) has your two sections on
one cost. The hand-written gradient loop lands at Kp 6.68, Ki 9.67, Kd 3.74. The pipeline's
`Optimizer(program, method="gd")`, with the same learning rate and number of steps, lands on the
same gains to 1.8e-15 (the same cost, gradient and step). Then `method="auto"` (L-BFGS, 16
iterations) finishes the descent that 1000 hand steps leave halfway: J drops from 0.0701 to 0.0555
at gains (13.4, 21.2, 8.0). The demo's tuned constants are kept. Its loss had to become the
problem's cost: the tracking error plus an effort term measured from the torque that holds the
setpoint (OD-20), because the old penalty on the gains cannot be a cost on the run.

**What landed, in plan order.**
- The clean-up's renames and moves: the removals, the package moves (dynamic programming,
  `minilink.learning`, MPC under trajectory optimization), the short names, GMC714's teaching
  names (K2, K3, K10, A6, G3).
- The step model, one compiled `ev.step` / `ev.rollout` / `ev.jit` for every consumer, the old
  integrator grid gone (SM-0 to SM-8).
- The path to the cost (V1.1 to V1.5, A2):
  - the native `Trajectory` and the cost's quadrature;
  - the problem's fields with one meaning each: `U` unconstrained unless declared,
    `x_goal` never creating `Xf`, the discount on the problem, the price switch and
    `cost_bound()`;
  - the one scoring formula, and `evaluate_trajectory` for a held run;
  - the costs' parameters, and the problem's parameters as a live view;
  - the problem evaluator, `evaluate` / `compare`, and the program export.
- The optimizer's method table with the first-order backend (S74).
- The closing example (S77).

**Checks.**
- Level 2 (the full suite minus the six slowest): 1692 passed.
- The six slow ones pass. The RRT flagship demo timed out once under load and runs in 51 s
  alone.
- The 70-entry step-model baseline agrees at rtol 1e-6 through every step, apart from SM-7's
  intended change.
- The regression gate passes apart from speed rows on this slow VM.
- On the final code commit (852a3018): the full `pytest` (1704 passed, 9 skipped), the notebook gate (34 of 34), the flagship demos (10 of 10) and the regression gate all pass.

**Please look first at:**
1. The number changes in §3. Value iteration's default price changed from 1e6 to the cost bound,
   so the course notebooks' cost-to-go colour scale now tops at the bound. The evaluation
   scores moved from a held law to the continuous loop: a few percent, 8% for a stiff PD at
   dt = 0.05.
2. The U sweep (OD-18). About 90 constructions now write `U=plant.input_box`, the course
   notebooks among them, so their numbers stay the same. You may prefer fewer such lines in
   lessons where U never mattered (LQR).
3. The planners' reported trajectory stays their held-input rollout on their grid, while
   `evaluate` scores the continuous loop (OD-27).
4. `compare` keeps the names-only form beside `compare(problem, ..., dt=)` (OD-28).
5. Cost term ids follow your R53 (the class name verbatim), not the plan doc's snake case
   (OD-22).
6. The three demos that still teach a hand gradient loop (OD-32), and the LQR warning (OD-33),
   are yours to call.

## 2. Decisions

| Id | Step | Question | Choice and reason | Alternative | Commit |
| --- | --- | --- | --- | --- | --- |
| OD-1 | K2 | R54 deletes `ProblemEnv`, but it is the class behind `Sys2Gym.from_problem`, which the `gymnasium_interface` notebook teaches | Kept as internal machinery of the gym module, off the `interfaces` facade; `to_gymnasium` deleted. The ruling's aim (no public name without a caller) holds | Fold its reset/step into `Sys2Gym` (more branches in the plain env) | — |
| OD-2 | K2, K8-1 | R64 takes names off the root but the teaching-surface test required every band name on the root | The test now reads the root prelude as a subset of the band facades: a band name need not be on the root, and where it is, it is the same object | Keep the test and leave the names on the root (contradicts R64) | — |
| OD-3 | K2 | Folding `NeuralNetwork` into `MLP` changes the neural-controller demo's initial weights (Gaussian × 0.2 → orthogonal) and so its numbers | `MLP(2, 1, hidden=(8,), seed=2)`, every tuned constant of the demo kept; it still trains (batch loss 0.32 → 0.0011). No `scale=` option added to `MLP` for one demo | Give `MLP` a Gaussian `scale=` initialisation to keep the old numbers | — |
| OD-4 | Order | Within the clean-up, which items must precede the new core? | K10 (the package moves) and A6 (the renames) go first, since the path to the cost edits those files; the catalog items (K8's cars, root trim, experimental plants, twins), S32 and K3b touch nothing the new core writes, so they run after it. R68 lands with V1.2, which reads the plants' boxes | Every clean-up item first, as planned (the evaluation pipeline would then land later in the night) | — |
| OD-5 | K10 | Where MPC lives after R66 and how students import it | `minilink/planning/trajectory_optimization/mpc/` (the package moved whole, its `__init__` the MPC sub-facade); `minilink.control` re-exports `ModelPredictiveController` (R66) and drops its `mpc` submodule alias (no alias, R82); the demos import from the new sub-facade, which the teaching-import check accepts | Keep a `minilink.control.mpc` alias module | — |
| OD-6 | G3 | Where the teaching name `ModelPredictiveController` lives, and its frozen constructor | A module of its own, `planning/trajectory_optimization/mpc/predictive.py` (teaching lane); the MPC classes, utilities and plots stay research lane by module. The frozen constructor is today's: `(planner, *, dt_mpc, warm_start=True, verbose=False, t0=0.0, debug=False)`. Students import it from `minilink.control`; it is not on the root (GMC714 does not teach MPC, R48) | Put it on the root prelude; or drop `debug=` before freezing | — |
| OD-7 | SM-1, SM-2 | The plan has discretize 'on the compiled step', but a StepSystem's step must stay an equation path on both backends | The RK4/Euler formula is written once (core/compile/evaluators/stepping.py) and used by the compiled evaluators' step/rollout and by discretize, which applies it to the source's f; compiling the discretized system traces through it. Same numbers, one formula | discretize compiles its source per backend and caches the evaluator (a cache on a System) | — |
| OD-8 | SM-1, V1.3 | Should rollout return (t, xs, signals) as the plan writes? | rollout returns the states (N+1, n) only; t is t0 + k dt, and the recorded signals come after the scan by evaluating the pure output map on the samples (vmap on JAX), which is identical, traceable and costs nothing to callers that do not need them (RL, DP) | Record the signals inside the scan, changing rollout's return type for every caller | — |
| OD-9 | SM-1, SM-6 | The evaluator's port map: rename outputs to h as the plan's four bodies say? | Kept as `outputs`, which returns the dict of output ports; h is a system's vector output, and a dict named h would mislead. (Corrected 2026-10-24: at the time `outputs(x, u, t)` still had its `outputs_p` twin; the fold of §9 gave it `params=None`, and the maintainer confirmed the name.) | Rename to h | — |
| OD-10 | SM-7 | The NumPy evaluator has no jit tier; how does a speed-minded caller write one line for both backends? | On NumPy `ev.jit` is the evaluator itself (nothing to compile), so `ev.jit.rollout(...)` runs on both backends; the fixed-step solvers use it (the JAX one stays as fast as the old jitted rollouts) | Raise on NumPy and branch on the backend in every caller | — |
| OD-11 | SM-6 | Move the integrators to simulation/solvers/ (S37) as the row says? | No: the step model's formulas (stepping.py) stay in core/compile, since the compiled evaluator, a core object, calls them and core never imports simulation (dependency law); simulation/solvers/ keeps the solver backends, which now call ev.jit.rollout. The relocation the row asks for has nothing left to move | Move stepping.py to simulation/solvers and have core import it (breaks the dependency law) | — |
| OD-12 | Order (V1) | The path to the cost bundles semantic migrations (V1.2 problem fields, A2 params view, V1.4b Monte Carlo fold) with the new engine; which goes first? | The engine first, additive, in its own module planning/engine.py (ProblemEvaluator, the scoring formula score_run, the new Evaluation record, the evaluate facade, PlanningProblem.compile), so the PID pipeline the run must finish works end to end on a green suite; the semantic migrations then land one at a time against a working engine, and the old MonteCarloEvaluator module is deleted when its callers move (V1.4b). | The plan's order (V1.2, A2, then the engine): every step breaks callers before anything new runs. | — |
| OD-13 | V1.3 | What name does the recorded plant input take in the loop's trajectory (the role 'plant' points at it)? | plant.u: a block output is recorded as 'id:port' (the existing compute_internal_signals_dict convention), so the plant's gathered input gets a different separator and can never collide with a plant output named u; roles = {'plant': {'x': (a, b), 'u': 'plant.u'}}. | 'plant:u' as the plan writes, which collides with an output port named u. | — |
| OD-14 | V1.5 | pe.program(wrt) reads its base point from pe.params, i.e. the objects; how does a caller tune with a free input (the reference r of an error-driven PID) at a value other than its port's nominal? | program(wrt, bounds, key, n_trials, *, params=None): params overrides entries of pe.params at the base point, as in pe.evaluate(params); the leaves not tuned keep those values and z0 reads the tuned ones from the same tree; z0 is also in program.metadata['z0']. | Require the caller to set loop.inputs['r'] nominal before program(): state on the loop the caller never built. | — |
| OD-15 | S74 | The preset names and the class table: algorithm names with the scipy_* keys as aliases (D23), and which methods each class admits? | No aliases (R82): the new presets are named by algorithm, gd, adam and l-bfgs (SciPy's L-BFGS-B), and the existing keys scipy_slsqp, scipy_trust_constr and ipopt stay as they are; the class (unconstrained, bounded, constrained) is read from g, h and the finite box faces; method='auto' picks l-bfgs for the first two and scipy_slsqp for constrained; gd and adam refuse constraint rows. The default stays scipy_slsqp (§4.5). Differential evolution and scipy_bfgs are not added (no caller); the stochastic class waits for a program that declares itself stochastic. | Rename the scipy_* keys to algorithm names: a sweep of the trajopt planner, demos and baselines with no gain this release. | — |
| OD-16 | S74 | The plan writes Optimizer(program).solve() with no z0, but MathematicalProgram stores no initial guess | Optimizer's z0 is optional: unset, it reads program.metadata['z0'], which pe.program fills with the packed current leaves; a program without one is refused with that message. The program stays a pure description (metadata is its diagnostic slot). | A z0 field on MathematicalProgram: solver state on the pure NLP, which its docstring rules out. | — |
| OD-17 | S74 | The first-order backend: move RL's Adam 'unchanged', but the backend steps a NumPy z | Adam and clip_by_global_norm moved verbatim to optimization/optimizers/first_order.py (the RL algorithms import them from there; the RL baselines run unchanged); FirstOrderOptimizer jits Adam.update once per solve (eager JAX dispatch cost 37 ms a step) and keeps gd in NumPy, so gd needs no JAX. Adam's eps in the Optimizer preset is 1e-8 (the textbook value; RL keeps its 1e-5 default), max_grad_norm None. | Write a NumPy twin of Adam for the backend: two copies of one formula. | — |
| OD-18 | V1.2 | U unbounded by default (R21, R37) and x_goal no longer creating Xf (R33): about 90 constructions relied on the old defaults; how to keep every number? | One scripted sweep (R82): every construction whose plant input box may be finite (its class default, or the file sets input bounds; unresolvable cases conservatively) gains U=<plant>.input_box, a new System property; every x_goal-only problem in a trajectory-optimization file has x_goal=E replaced by Xf=SingletonSet(E), x_goal then read from the singleton; RRT keeps x_goal (its goal ball). SingletonSet joins the root's teaching band names, since students now write that line. Value iteration's grid and RRT's bang-bang refuse an unbounded U with the line to write. The 70-entry step-model baseline and the regression gate agree unchanged. | Keep the plant box as U's default: the ruled design makes the problem the only owner of its constraints. | — |
| OD-19 | V1.2 | examples/projects/car_trajopt/car_trajopt.ipynb cell 16 had a syntax error on dev (a missing comma after cost=tracking_cost + obstacle_cost, from an earlier sweep) | Fixed in the U sweep (the comma, and U=sys.input_box beside X=sys.state.box): a broken cell in a project notebook, outside the smoke set. | Leave it: the notebook cannot run. | — |
| OD-20 | S77 | The closing example: the hand loop's loss (mean squared error plus 1e-4·\|gains\|², a penalty on the parameters the program cannot carry, R46) differs from any problem cost; and plain gradient descent at the demo's LR=1 for 1000 steps does not reach the optimum, so 'Adam and the class default reach the hand loop's gains' cannot hold literally | The hand loss is rewritten as the problem's cost, J = (1/T)∫((r−θ)² + ρ(u−u_ss)²)dt with ρ = 0.01 and u_ss = m g l sin r (the torque that holds the setpoint: without it the effort term fights the integral action), by the trapezoid on the same RK4 run; every tuned constant of the demo stays (TF, DT, N_STEPS, N_LEARN_STEPS, LR, the plot settings, the start gains). Section 2 states the problem, pe.program(wrt, params=start), and Optimizer(program, method='gd') with the hand loop's LR and steps lands on the hand loop's gains to 1.8e-15; then method='auto' (L-BFGS, 16 iterations) finishes the descent the hand loop leaves halfway (J 0.0701 → 0.0555; gains (6.68, 9.67, 3.74) → (13.43, 21.20, 8.04)), judged by pe.evaluate. Adam is not shown (OD-17 tests it). | Keep the gain penalty in the hand loss and leave it out of the problem: the two sections would then optimize different costs and could not agree. | — |
| OD-21 | V1.2 (price) | The price switch: infeasible_cost becomes None/True/number (R32) and DP/RL price by problem.cost_bound(); how do the planners' own price knobs and the bound's discount fit? | PlanningProblem coerces infeasible_cost to None, True or a float and refuses a callable, False or a string; cost_bound(tf=None, *, discount_rate=None) is g_max·(1−e^{−ρT})/ρ + e^{−ρT}·h_max over the declared boxes, else the plant's declared ranges, always finite (1e6 with a warning naming infeasible_cost when no bound exists); infeasibility_price() is the number when one is set, else the bound, read by DP, tabular learning, the RL environment and the Gym bridge. CostFunction gains g_max/h_max (sampled by default; exact for QuadraticCost on a box, TimeCost, sums and nonnegative scalings). Value iteration's own out_of_bound_cost knob keeps its callable form (its JAX tables test it) and now defaults to None, the problem's price; grid_price uses the rate (1−α)/dt so the bound covers the discrete sum Σαᵏdt (the continuous −ln(α)/dt undercounts it); problem.infeasible_penalty is gone. RL's feasible_cost_bound is deleted (one owner). | Keep DP's 1e6 default and only price by the bound when infeasible_cost=True: the plan rules the bound as the default price. | — |
| OD-22 | A2 | cost-params.md names a cost's default term id by snake-casing its class (quadratic, swing_up), but R53 (2026-10-23) made a block's default id its class name verbatim | Cost term ids follow R53, so costs and diagrams share one rule: QuadraticCost, QuadraticCost2 for a repeat, a scaled cost the id of the cost it scales, cost.id overriding; SumCost.of(**named) keeps names; anonymous + and sum() flatten. The library costs (QuadraticCost, TimeCost, SumCost, ScaledCost, FieldCost) are plain classes with params on the System pattern; their attributes are views (param_view) so in-place writes such as cost.R[0, 0] = … still reach g. | Snake-case term ids as the plan doc writes: two naming rules for one mental model. | — |
| OD-23 | A2 (R89) | A plain-class PlanningProblem loses dataclasses.replace, which tabular learning and the MPC tick used | problem.replace(**fields) builds a new problem of the same class (FIELDS lists the constructor's keywords; a stochastic problem adds its laws); a new x_start drops a singleton X0 so it follows. Writing problem.params['x_start'] (or the setter) moves a singleton X0 with the start, else requires the start to lie in X0. | A copy-and-mutate in each caller: three spellings of one operation. | — |
| OD-24 | A2 (R87) | 'Each set owns its parameters': the library sets are frozen dataclasses with margin(z, t, params=None) and no params of their own | Set and InputSet gain a class attribute params = None; problem.params shows each set's params by role, and writing one to a set that holds none (a frozen library set) is refused with TypeError; every planner that passed problem.params.sets now passes nothing, so a set reads its own. A user set that keeps parameters stores self.params and reads it when params is None (the System pattern); the tests' Band and ShrinkingInputs show it. Library sets reading params stay A5's. | Keep a shared sets dictionary on the problem: the bundle R87 retires. | — |
| OD-25 | A2 (R87) | The bundle carried a random plant's nominal draw (sys.realize(None)); R87 says it is 'taken where a tool needs it, not stored' | nominal_plant_params(problem) in planning/problems.py: the mean draw of a random plant, else None (the plant's own); the planners that read problem.params.system (DP, policy evaluation, the grid, LQR, the transcriptions, the MPC tick) call it. The engine keeps the plant's own params in pe.params (its draws are the stochastic problem's). | A problem method: a second home for a derived value. | — |
| OD-26 | A2, V1.4 | Which of the problem's leaves join the run's tree pe.params? | pe.params = the loop's blocks by id, then the problem's own leaves (cost, the parameters of the sets that hold any by role, x_start, discount_rate), then the free inputs; score_run reads cost, X, U, Xf and discount_rate from it, so jax.grad reaches Q, R and rho (tested against a central difference) and pe.program(wrt=['cost.R']) works. A block or free input named like one of these leaves is refused. | Keep the cost out of the tree until a consumer asks: V1.4's gate names the Q gradient. | — |
| OD-27 | V1.4b | The feedback planners' solution.trajectory was the old evaluator's held-input rollout on the planner's grid; with the evaluator gone, does it become the continuous run? | No: nominal_trajectory (now in the engine) keeps the held-input rollout, u_k = π(x_k, t_k) held while the plant steps (rk4) to x_{k+1}: the PlanningSolution docstring calls it the planner's evidence on its own model, and its tests pin it to 1e-10. evaluate scores the continuous loop. The sampled loop of v0.4 is where the two meet. | The continuous run for both: the planners' reported trajectories would change and no longer show the law the planner designed (sampled at its dt). | — |
| OD-28 | V1.4b | compare: today's compare(VI=sol, LQR=sol) lines solutions up for the overlays with no problem; the plan writes compare(problem, PID=pid, ..., dt) scoring every entry | Both, by the first argument: compare(**named) as today (solutions only); compare(problem, **entries, dt=, n_trials=, key=, solver=, backend=) also scores each entry (a PlanningSolution, a controller block, or an Evaluation already computed) on one key and one backend (NumPy for all when one entry does not trace, so the trials are paired). Comparison gains evaluations {name: Evaluation} and difference(a, b), the per-trial J_a − J_b as an array; a bare controller is wrapped as a solution with a 'given' record so the overlays read it. | Two verbs (compare and a scoring variant): one way to compare controllers is R84. | — |
| OD-29 | V1.4 | The plan's evaluate takes solver='scipy' (dt the reporting grid), and 'auto' must fall back to NumPy when the cost or a set does not trace (today's Monte Carlo did) | The engine integrates an adaptive solver label (the Simulator's 'scipy*' table) with SciPy's solve_ivp on the NumPy evaluator, reported on the dt grid; an explicit JAX backend with it is refused. Under 'auto', after the loop compiles on JAX, one abstract trace of a trial's run and score (jax.eval_shape, no compile) decides: a NumPy-only cost or set falls back to NumPy for the whole run. | Refuse an untraceable cost under 'auto': a regression for every NumPy-written student cost. | — |
| OD-30 | V1.4b | A planned schedule (TrajectorySource, the open-loop policy) did not trace on JAX (scipy's interp1d per channel), so a trajectory-optimization solution could not be evaluated on JAX in the loop | TrajectorySource.h interpolates on the array module (xp.interp for 'linear', searchsorted for 'previous', endpoints held), so it traces; its interpolator memory and refresh() override are gone. Numbers unchanged (the same linear and previous rules). | Pre-tabulate the source on the evaluation grid as the old evaluator did: a second path for one block kind. | — |
| OD-31 | V1.5 | The row asks for the program's leaves scaled by their nominal magnitude | Not done: with z in the leaves' own units, gradient descent on the program takes exactly the hand loop's steps (the closing example's two sections agree to 1e-15), and pe.unpack(result.z) needs no scale; L-BFGS and Adam absorb the scaling. Scaling can come back as an option of program() if a tuning needs it. | Scale by \|leaf\| (or 1 at zero): the gd section would then differ from the hand loop, and every unpack would need the scale. | — |
| OD-32 | S74 | The row asks that no hand descent loop remain in examples/demos/compile/ but the closing example's first section and tutorial 07's | Left for the maintainer: cartpole_rollout_gradients (an open-loop schedule, which the engine cannot tune until a source's samples are parameters), neural_controller_jax and params_gradient (identification) each teach the hand loop, and R90 kept that loop in two places on purpose; converting them changes what they teach (student-facing). | Convert all three now: three lessons rewritten without the maintainer. | — |
| OD-33 | V1.2 | 'LQR warns once' when U is declared | Deferred: the U sweep (OD-18) wrote U=plant.input_box on every lesson whose plant has a finite input box, LQR lessons included, to keep their numbers; a warning would now fire in the GRO860 LQR notebooks although nothing changed for them. Whether those lessons keep the line (and see the warning) is the maintainer's call. | Warn now: new warnings in course notebooks overnight. | — |
| OD-34 | V1.2 | The README's planning snippet builds a problem without U and hands it to value iteration and RRT, which now refuse an unbounded U | Its API line gains U=plant.input_box beside X=plant.state.box (as examples/demos/core/readme_examples.py, which the sweep updated); no other README change. | Leave the README stale until the maintainer's pass: its snippet would raise. | — |
| OD-35 | R68 | R68 names MechanicalSystem; GeneralizedMechanicalSystem invents bounds the same way (±10 on the state, ±5 on the inputs) | Both bases invent none: their bound-writing lines are gone, and the 23 catalog plants on them (the experimental tier's included) write their range in their own constructor, at the class that calls the base (a subclass inherits it), as np.full(self.n, ±2π) / np.full(dim, ±5) or ±10 / ±5; a snapshot of every plant's state and input bounds before and after is byte-identical, as are the step-model baseline and the regression gate. A user's own subclass starts unbounded (examples/demos/robotic/custom_one_link_arm.py runs as it was). | MechanicalSystem only, the literal ruling: the generalized family would keep inventing bounds until S32 merges it. | — |
| OD-36 | V1.1 | The row folds StepRollout into Trajectory and derives Set.contains from margin on xp | contains is derived on the array module (a NumPy boolean on NumPy, a traceable JAX boolean under jit; tested), numbers unchanged. The StepRollout fold waits for S31 in v0.4: StepRollout is the hybrid simulator's computer record and MPC's dual-rate view, and R86 moved every hybrid piece to v0.4. | Fold it now: a rewrite of the hybrid simulator's records in the release that defers the hybrid work. | — |

## 3. Intended number changes (R80)

| Step | What changed | Problem and cost | Before | After | Relative difference |
| --- | --- | --- | --- | --- | --- |
| SM-7 | compute_forced on rk4_fixedsteps: a sampled input matrix is held (zero-order) over each period instead of interpolated; a callable u(t) is read at the RK4 stage times instead of sampled and interpolated | Pendulum, x0 = [0.4, −0.2], u = 0.5 sin 2t on [0, 1], 20 steps (SM-0 entries forced.rk4_fixedsteps.*) | linear input between samples | held / at the stages | sampled 4.4e-3, callable 9.5e-5 (max over the run, relative) |
| V1.2 (price) | Value iteration's default price of an inadmissible pair: 1e6 → the problem's cost bound on the grid's own sum (g_max dt/(1−α), exact for a quadratic on the boxes); tabular learning prices its grid the same way; the RL environment and the Gym bridge price by cost_bound over the episode (was a sampled bound, Halton 4096, over the training zone and the action port) | DoubleIntegrator test problem (tests/unittest/test_dynamic_programming.py make_problem), QuadraticCost Q=I R=I, grid 31×31×7, dt 0.1, α 0.95 | price 1e6; J on the 203 cells with J < 5 as solved | price 38.0 (= the bound); on those 203 cells J lower in 99%, median \|ΔJ\| 5e-4, max 3.07 (cells next to the boundary, where 1e6 leaked through the interpolation), the same action in 89% (ties) | median 1e-4 relative; up to 0.64 at boundary-adjacent cells. The step-model baseline (its DP entries price explicitly) and the regression gate agree unchanged; the value-iteration course notebooks' cost-to-go colour scale now tops at the bound instead of 1e6: worth a look |
| V1.4b | Every evaluation score: the old Monte Carlo trial held the law over each period dt and cut the integral at the first exit; evaluate runs the continuous loop (the law at every RK4 stage) and integrates to tf | Pendulum, state box ±1 rad × ±6 rad/s, QuadraticCost Q=I R=0.01, tf=2 s, 20 starts ~ U([−0.5, 0.5]²) from seed 1 (the same starts on both paths), dt=0.05, price 100; a PD law K=[30, 8] and a weak one K=[2, 0.5] | MonteCarloEvaluator(backend='numpy', seed=1): mean J 0.2946 (PD), 0.4509 (weak) | evaluate(n_trials=20, key=1, backend='numpy'): 0.2713 (PD), 0.4385 (weak); no run failed on either path | per run, median 7.7e-2 (max 1.0e-1) for the PD law, 2.8e-2 (max 3.0e-2) for the weak one: the difference between a law held over 50 ms and the continuous law, larger for the stiffer gain, as expected |

## 4. Not done, or blocked

The rest of v0.3's clean-up, not started (each needs a pass of its own, with its numbers kept):

- the catalog clean-up (K8): the car family to three models, the root prelude trimmed to the
  course plants (R60), `minilink.catalog` as the index of tested plants, the unsupported plants
  to the experimental tier, the twins merged, the README's catalog line;
- one mechanical family (S32): `H` the one mass-matrix name, the boat and the plane under
  `MechanicalSystem`, `DynamicBicycle` on the generalized child;
- the contract tests for the kept prototypes (K3b).

What the landed rows left (each written as a "rest" row in TODO §4):

- the evaluator's internals deduplicated and the `f` / `f_p` tiers folded (OD-9; SM-6's rest);
- `StepRollout` folded into the native `Trajectory` (V1.1's rest; OD-36);
- the LQR warning (OD-33; V1.2's rest);
- the three hand-loop demos (OD-32; S74's rest);
- the program's leaves scaled by their nominal magnitude (OD-31, decided against);
- the library sets reading their own `params` (A5, by design after A2).

## 5. Tests skipped in this container

No Ipopt (`cyipopt`) in the cloud container: the `ipopt` tests skip and trajectory optimization
falls back to SciPy where a test or demo asks for Ipopt.

The other skips in `pytest tests/unittest` (9 in all): the six Ipopt tests; the notebook smoke
inside pytest (run instead by `tests/demo_checks/run_notebook_checks.py`, the "Notebook change"
row); one hybrid test needing the graphviz `dot` binary; one SciPy run on a discontinuous
sliding-mode loop that is skipped by design. Speed rows of the regression gate fail on this VM
(for example `f.dual_rate.nlp_s` at 0.22 s against a 0.15 s gate); every value row passes.

## 6. The review pass after the run

Three independent reviewers read the run's diff: the engine and the problem classes; the costs,
sets and optimizer; the step model and its callers. Each finding was checked by its own script
before a fix. A seeded baseline of 19 evaluations (NumPy and JAX, the price off, on and a number,
deterministic and stochastic, one program gradient) stayed byte-identical through every fix.

Fixed:

- A cost under the user's own `jax.jit` crashed in a fresh process: the per-sample cost
  switched JAX to 64-bit floats in the middle of the trace. It no longer touches the config, and
  the `Trajectory` pytree is registered on the first JAX array (it was registered by the first
  minilink JAX tool only).
- `problem.compile` crashed on a plant with an input nothing wires (`NoisyPendulum`'s `d`, `n`):
  the port now reads its nominal value.
- On JAX, a compiled evaluator kept the price and the sets of its first trace, so a changed
  `infeasible_cost` or `Xf` gave NumPy and JAX different numbers. The price is now an argument
  of the compiled trial, worked out once per evaluation (NumPy no longer recomputes it per
  trial), and a replaced cost or set traces anew.
- `problem.params = problem.params` refused the `None` entries of sets without parameters;
  `replace(x0_distribution=...)` kept the old start and support; the CVaR counted one run too
  many at `α n` = 7.000000000000001; `Comparison.difference` broadcast mismatched trial counts.
- The first-order optimizer tested `‖∇J‖`, so an optimum on a face of the box never converged,
  and it reported `success` after running out of steps. It now tests the projected step
  `‖z − Π(z − ∇J)‖` at the returned `z` and reports `success` only when converged.
- `SumCost` keys depended on the bracketing (`a + (b + c)` gave `QuadraticCost22`).
- `quadratic_max` returned less than the maximum for an indefinite weight; it now returns the
  bound `dᵀ |W| d` there. The sampled `g_max` / `h_max` docstrings now say "estimate".
- `PolicyEvaluator` built without options ignored the problem's discount and horizon (about
  50× the optimal cost on a ρ = 1 problem); it now resolves them as the planner does.
- The failure price of the stepped tools (value iteration at a finite horizon, the RL
  environment, the Gym bridge) used the continuous `∫ e^{−ρt} dt`, below their discrete sum
  `Σ_{k<N} γᵏ dt`, so a failure could cost less than staying feasible (a regression from the
  removed `feasible_cost_bound`). `cost_bound(dt=)` now takes the discrete sum.
- The Gym bridge charged a failure on the last step of a finite horizon the price but not `h`,
  unlike the RL environment and `evaluate`.
- The RL reward read `g` on the action alone; a plant with more input ports (a `d`, an `n`) was
  charged the action on every port. It now reads the plant's whole input, as `evaluate` does.
- The JAX rollout refused an integer `x0`; `substeps=2.0` passed the check, then failed in
  `range`.
- The U sweep of the run had written `U=plant.input_box` into a comment in the particle
  Gymnasium notebook (GRO860) and one RL test, so the argument was never passed.
- `evaluate` dropped a problem's disturbance draws silently; it now warns (open item 1).
- Smaller: the terminal-cost discount in `CostFunction`'s
  docstring, "in this release" in a docstring, a ruling id in a test comment, a stale DESIGN
  sentence on a callable price, the MPC module pointer, the `Discretization` docstring.

Gates on the fixed code: ruff and the format check repo-wide; `pytest` 1714 passed, 9 skipped
(the container skips of §5); the notebook checks 34/34; the regression gate (CI flags) passes.
The math edits were read next to `planning/dynamic_programming/dp.py`.

Number changes (R80): a stepped tool's price at a finite horizon with `α < 1` rises to the
discrete sum (pendulum, ρ = 0.5, tf = 2 s: 171.36 → 172.06 at α = 0.99, dt = 0.05; 89.89 →
91.32 at α = 0.9, dt = 0.1); the infinite horizon is unchanged. The RL reward changes only on a
plant whose cost reads more input ports than the action.

Open, for the maintainer (not changed):

1. **Disturbance draws in `evaluate`.** The run turned the old Monte Carlo evaluator (which
   stepped the RL environment and drew `d` every step) into the continuous loop, which holds the
   problem's `disturbances` ports at nominal. The evaluator now warns instead of silently
   dropping the draws. Per-step draws in the continuous loop are the noise-signal work planned
   with the randomness steps (RN-4); bringing them forward is a design choice.
2. **A diverged run scores better than a failing one** under the price: a run that blows up
   is charged `price`, a finite run that leaves `X` is charged `cost + price`, and the
   integral runs to `tf` with no cut (the ruling). Options: charge a diverged run
   `cost_bound + price`, or include `isfinite(cost)` in `finite` and keep the cost of the
   finite prefix. Its gradient through the overflowed rollout is NaN (the docstring now says so).
3. **The tabular learner's model.** Its environment steps with RK4 while its grid defaults
   to Euler, so "the same grid solved by value iteration is the exact answer the learner
   approaches" holds only with `solver="euler"` passed. Making the grid's solver the default
   changes the course results.
4. **The Gym bridge runs one step too many** on an infinite horizon (`t_new > tf` on summed
   floats: 11 steps for `tf = 1`, `dt = 0.1`), unlike the RL environment. The GRO860
   Gymnasium notebook's hand-written class mirrors this line, so the fix touches the notebook.
5. **The same cost added twice** (`q + 10 * q`) shares its params between both terms. Refusing
   it would refuse `cost + cost`.
6. **The price ignores a run's cost overrides**: a program over cost weights keeps the price
   of the problem's own weights. Worked out once per evaluation, it costs 4096 calls of `g`
   for a cost with no exact bound (`QuadraticCost` has one) at every `evaluate`, a program's
   every iteration included; a cache keyed on the cost's parameters would remove that.

## 7. The second review pass

A fresh reviewer read the first pass's two commits adversarially; two more read the parts the
first pass did not cover (the blocks, control, diagrams and facades, learning, trajectory
optimization and MPC, graphics; and the documents, demos and student-facing material).

Fixed in the first pass's own work:

- The first-order optimizer reported `success=False` for a gradient descent converged to
  machine precision (the default `gtol = 0` asks for an exact zero). A step that no longer moves
  `z` is now a fixed point, hence converged, for gradient descent; Adam keeps its step until its
  moments settle, so it stops on the projected gradient alone.
- Value iteration's `solve()` sweeps to convergence whatever the problem's horizon, so its sum is
  `Σ_{k≥0} αᵏ dt = dt / (1 − α)`; the price took the sum cut at `tf` (below the converged cost
  of a feasible cell). `grid_price` now bounds the converged sum; `α = 0` no longer warns.
- `PolicyEvaluator` uses a passed options bag as given; its docstring now says to pass the
  planner's resolved `planner.options` (a fresh bag is the user's explicit choice).
- The substep check now refuses every non-integer type (`np.float32(2.0)`, `True`), not only
  Python floats.
- The JAX evaluator's compiled-trial cache grew one entry per replaced set or cost; it now
  keeps one entry per batch layout, replaced when the problem's parts change.
- `cost_bound(dt=)` counts at least one period, as the stepped tools run at least one.

Number change (R80): value iteration's price at a finite horizon with `α < 1` is the converged
sum's bound (pendulum, ρ = 0.5, dt = 0.05: 71.7 → 113.4 at tf = 2 s); at an infinite horizon it
is unchanged. The 19-evaluation baseline stays byte-identical.

From the library reviewer:

- A block id holding `.`, `/` or `:` silently broke the parameter paths (`"motor.v2.A"`) and the
  port names (`linearize(loop, of="g:1:y")`): the run's verbatim ids (R53) dropped the old
  normalization. `add_subsystem` now refuses such an id and names the separators; no id in the
  library, tests, examples or benchmarks holds one.
- DESIGN's package map and dependency exception named the deleted `planning/evaluation.py`
  (now `engine.py`) and promised a Mermaid export (deleted, R67); ROADMAP §3 listed
  `MonteCarloEvaluator` (now `evaluate`). Stale wording of the renames fixed in a test comment,
  a benchmark docstring, the dynamic-programming package docstring, the symbolic export and the
  RL planner's discount warning. `TrajectorySource.h` names its result before one return; the RRT
  planner's dead `set_params = None` bindings are gone.

Open, for the maintainer: `minilink.closed_loop` and `minilink.feedback` are on the root
prelude but on no band facade (true at the base commit; the facade is a public name, so it waits
for a nod); `A >> B >> C` is named `A >> B` (the left operand's name carried, true at the base).

From the documents and demos reviewer:

- A stray 80 KB file named `--compare` sat at the repository root since the run's V1.2 (sets)
  commit: `benchmarks/step_model_baseline.py` wrote to `sys.argv[1]` whatever it was. Removed;
  the script now refuses a first argument that is a flag.
- DESIGN corrected where the code says otherwise: a diverged run's gradient (not zero, not
  defined) and a finite run whose cost overflows (`+inf`, price or not); `cost_bound(dt=)` and
  value iteration's converged sum `dt/(1 − α)` with `ρ = −ln α / dt`; a feedback planner's
  `trajectory` (filled by `solve(evaluate=True)`, else `None`); `ev.rollout`'s `n_steps=`;
  `ModelPredictiveController` (a `StepSystem` by default). ROADMAP §3 and §5.3 no longer say the
  integrators moved to `simulation/solvers/` (OD-11 kept them in `core/compile/`); TODO's frozen
  surface and blocks rows name `UnitDelay` and `ModelPredictiveController`; the API page's
  section is "Dynamic programming".
- The changed demos: 46 of 51 ran headless; the five that did not finish in the reviewer's
  7-minute cap (four MPC and value-iteration demos on SciPy without Ipopt, one value iteration
  killed for memory with four demos in parallel) showed no error before the cut. The closing PID
  example runs in 61 s; its pipeline descent matches the hand loop to 1.8e-15.

Open, for the maintainer:

- RULES 6.15 names tutorial 07 as the one place gradient descent is written out; tutorial 07 has
  no descent loop, and the closing PID example now holds one. Which place is canonical?
- ROADMAP §4.2 says "beyond what `identification/` already offers": the package was deleted
  (R67).
- Demo style, as questions: `pid_autotuning_jax.py` keeps its two helpers (as before the run),
  its new prints carry arithmetic, and its last figure is hand-made Matplotlib;
  `diagram_shortcuts.py` labels its noisy loop `Pendulum()` while it builds `NoisyPendulum()`
  (as at the base commit); `custom_one_link_arm.py` reads module constants, not `params`, and
  its `# H = m l²` comment restates the line below.

Gates on the second pass (at 2e468130, in this container): ruff and the
format check repo-wide; `pytest` 1721 passed, 9 skipped; the regression gate (CI flags) passes;
the notebook checks 33 of 34. The one that fails, `showcase_from_rl_to_bode`, times out on its
Meshcat animation: in this container `animate(renderer="meshcat")` hangs for a bare `Pendulum`
at the base commit e9f69dff too (no server left running), while the notebook passed in the first
pass's container. Its other cells run in seconds (the UR5 rollout 4 s on JAX).


## 8. K3b, the contract tests for the kept prototypes

The K1 inventory's lost script is rebuilt as `benchmarks/public_names.py`: the facades'
`__all__` by identity, one caller per file by AST (imports, attributes from an imported module,
the nine System verbs called as methods), notebooks' code cells included. "A contract test" is
read as the row's done-when reads it: a test file that uses the name. 21 public names had none:
the five result records (`StructuralResult`, `VerificationReport`, `OptimizationResult`,
`Comparison`, `HybridSimResult`), `angle_features`, the `Distribution` base, twelve graphics
primitives and skins, `plot_track` and `TrackCorridorOverlay`. Each now has a test of what its
docstring states, in the domain's test file (a skin draws only on frames its plant's `tf`
places; a pose is a rotation at `p0` along `p1 − p0`; a user `Distribution` drives a stochastic
problem). `tests/unittest/test_public_names.py` holds the list empty (6 s; it passes with JAX,
Meshcat, Plotly and pygame blocked, as the main CI job runs), so a new public name needs a test.

## 9. R12 finished: one body per evaluator method, compiled on JAX

The run had folded only the step model (`step`, `rollout`); R12 asked the same of every
method. The maintainer's rulings (2026-10-24, in conversation):

- **`outputs` keeps its name** (OD-9 confirmed), folded like the rest.
- **No `ev.frozen.*`.** `params=None` already reads the compiled parameters as constants:
  measured, the same call cost as today's frozen jitted `f` (10 µs on the pendulum leaf,
  12–13 µs on `PID @ Pendulum`).
- **One compiled name per method, marked `@compiled`** (`core/compile/evaluators/compiled.py`),
  and no `ev.jit`. A traced body called from Python on JAX is 120–640× slower than its
  compiled form (1 223 µs and 7 860 µs against 10 and 12 µs), so a backend-agnostic
  `ev.f(x, u, t)` would have been slow on JAX; a compiled method also composes inside an outer
  `jit`, `vmap`, `scan` and `jit(grad)` with bit-identical numbers (measured on both systems),
  so the trace tier and the `jit` namespace had no remaining job.

What changed: `f`, `outputs`, `signals`, the continuous `step` / `rollout` and the discrete
`step` / `outputs` / `rollout` are each one method taking `params=None` (`None` the compiled
parameters, block by block on a diagram, as the old frozen tier). Gone: `f_p`, `f_trace`,
`f_trace_p`, `f_jit`, `f_jit_p`, `f_ivp`, `f_ivp_p`, `f_scipy`, `f_ivp_scipy`,
`as_scipy_rhs`, `as_scipy_rhs_forced`, `as_scipy_jac`, `outputs_p`, `outputs_trace`,
`outputs_trace_p`, `outputs_jit`, `outputs_jit_p`, `step_p`, `step_trace`, `step_trace_p`,
`step_jit`, `step_jit_p`, `rollout_p`, `compute_internal_signals`, `step_block`,
`has_trace_tier`, `tiers.py` and its generated aliases, and `ev.jit`. Each diagram evaluator
keeps one signal sweep for both parameter conventions (it was two). The SciPy solver adapts
`f` itself, its analytic Jacobian compiled once per evaluator as before. Every caller moved in
the same change (R82): the library, the tests, the benchmarks, the demos
(`params_gradient`, `neural_controller_jax`, `pid_autotuning_jax`, `mpc_path`), the
experimental scripts, and the notebooks `07_compile`, `showcase_jax` and `showcase_minilink`
(their trace-tier paragraphs rewritten to the one rule).

Numbers (R80, R81): a seeded baseline of 164 evaluator entries (the five evaluator kinds on
both backends: `f`, `outputs`, the step model on both solvers and holds, `jacobian` with
respect to `x`, `u`, `params` and a wire, the discrete rollouts; the simulator on three solvers
and both backends, forced runs, `linearize` of a wire, a Lyapunov certificate) is identical in
157 entries; the 7 others are JAX calls from Python that ran op by op before and run compiled
now, at most 9e-16 absolute (4e-14 relative). The step-model baseline agrees on all 70 entries
(one RL entry at 7e-16) and the engine baseline on 18 of 19 (the program's gradient at 9e-16).

One semantic note: a diagram's `params=None` reads each block's compiled parameters; a
partial dict still reads the live parameters of the blocks it leaves out, as `f_p` did.

