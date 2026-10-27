# The v0.3 core: the plan

**Read first for the core.** This is the one design home of v0.3's workstreams. Each gets a
goal, a status, what the code does today, what is still open and its rows. Where the pieces live:
- **The rows** (files, "done when") are [TODO.md](TODO.md) §1.N, with the same N as the section
  here.
- **The rulings** are preliminary choices, open to readjustment with the final architecture:
  [R124–R140](../reviews/2026-10-27-design-questions-rulings.md) and
  [R141–R153](../reviews/2026-10-27-core-vision-decisions.md) (the latter opens with a one-screen
  index and what it revises among the former). How they were reached:
  [the consolidated core vision](../reviews/2026-10-27-consolidated-core-vision.md), whose fourteen
  decisions they take.
- **The open design questions** are [QUESTIONS.md](../dev/QUESTIONS.md).
- **The design detail** is in the topic plan doc each section links.
- **The landed contracts** are [DESIGN.md](../dev/DESIGN.md) §4–§6.

**Status words:** **landed** (in the code); **ruled** (an R-id in a dated record under
[docs/reviews/](../reviews/)); **direction** (the maintainer's words of 2026-10-26,
[record](../reviews/2026-10-26-core-consolidation.md)); **open** (not decided); **proposal** (the
agent's, not ruled). Nothing in this doc rules anything.

## 0. The target

**The direction (2026-10-26).** v0.3 is the complete core: one architecture from the plant to the
evaluation and the optimizer, supporting stochastic problems and the sampled loop, built in the
natural order of the code, with no date. v0.4 adds features built on it, not intertwined with it;
v0.9 stays the freeze candidate; v1.0 comes when the API feels mature. Class material stays on
`main` until the new core is ready.

**The spine** (ruled, R119, CONSTITUTION §1.1). A `System`, compiled, is rolled out to a
`Trajectory`, scored by a problem's cost into an `Evaluation`, lowered to a `MathematicalProgram`
and solved by the `Optimizer`. Every approach is a tool on that path. Its continuous half landed
2026-10-24/25 (DESIGN §5–§6). Where each workstream enters it:

```text
System  f(x, u, t; p)                     StepSystem  step(x, u, k; p)          §1 every number a leaf
  └─ % Δ, discretize → SampledSystem → SampledDiagramSystem (computer @ plant)  §2, §5 the step algebra, the sampled loop
PlanningProblem / StepPlanningProblem (stochastic by their parts)              §3 the draws   §4 the seeded policy
  └─ problem.compile(dt=) → transition(x, u, k) → (x_next, c_k)                §6 one score
  └─ problem.compile(controller) → evaluate → Evaluation; program(wrt)         §6, §7
MathematicalProgram  J(z, params) → Optimizer(method, iterations=K)             §7, §8
Planner → planner.compile() → pv: solve, shift, policy → the MPC                §8
Field  value(x, u, t; p): as_set, as_cost, as_terminal_cost                    §9
```

**The workstreams.** The maintainer named eight items on 2026-10-26; the plan adds two invariants
under them ("invariants", since ROADMAP's "foundations" also names the CONSTITUTION's three
objects).

| § | Workstream | Kind | The maintainer's item |
| --- | --- | --- | --- |
| 1 | Every object pure, every number a parameter | invariant | (under all) |
| 2 | The step algebra first-class | invariant | (under all) |
| 3 | The draws | pillar | stochastic problems |
| 4 | The seeded stochastic policy | pillar | the seeded stochastic policy |
| 5 | The sampled loop: a step diagram, two runners | pillar | `HybridDiagram` and `Computer`, two runners |
| 6 | One score | pillar | one discrete judge |
| 7 | The parametric program, the stochastic program its seed case | pillar | the generic parametric `MathematicalProgram`; the stochastic program |
| 8 | The native proof of concept: the planner's pure face and an MPC on it | pillar | a native fixed-step solver and an MPC on it |
| 9 | Fields | pillar | Fields |

"Pillar" means a core workstream; ROADMAP §5's six application families are "the toolbox
chapters".

**What a user writes at the end of v0.3** (a proposal, illustrative, in the ruled names):

```python
problem = PlanningProblem(plant, cost=cost, tf=5.0,                              # §3: laws where numbers go
    x_start=Gaussian(x_bar, std), params={"plant": {"m": Uniform(0.9, 1.1)}},
    disturbances={"d": WhiteNoise(1, psd=0.1, sample_period=0.01)})
pe = problem.compile(pid % 0.01)                     # §5: a digital PID, the loop a SampledDiagramSystem
print(pe.evaluate(n_trials=100, key=1))              # §3, §6: one key, one J
best = Optimizer(pe.program(wrt=["controller.Kp", "controller.Kd"])).solve()        # §7

planner = TrajectoryOptimizationPlanner(problem.replace(tf=0.5), n_knots=10,
                                        method="adam", iterations=20)              # §8: a fixed count
mpc = ModelPredictiveController(planner)             # §8: the delayed form (R123)
tuned = Optimizer(problem.compile(mpc % 0.05).program(wrt=["controller.cost.Q"])).solve()
terminal = QuadraticField(P, x_bar).as_terminal_cost()   # §9: the LQR value ending a short horizon
```

## 1. Every object pure, every number a parameter (invariant)

- **Goal: one contract every core math object converges back to** (CONSTITUTION §2, "One
  signature"; R140):
  - its mathematics is a function of its arguments and `params`, `params=None` reading the frozen
    defaults: `f(x, u, t, params=None)`, `g(...)`, `margin(...)`, `value(...)`, `J(z, params=None)`;
  - every core math object has a `params` field (`{}` when it has none), unless a written reason
    makes it an exception;
  - a composed object's `params` is a dict of its parts' dicts, reached by a dotted path
    (`cost.Q` on a problem, `controller.Kp` on a diagram);
  - leaves are numbers and numeric arrays; integer leaves (the `seed`, a step problem's tick
    offset) are allowed and never in `wrt` (R148, amending R118);
  - every function is deterministic given its parameters, the seed among them.
- **Ruled:** R140 (the contract, the first row of the core); an override is `None` or a complete
  tree (R121); a sampled controller's memory is its state, its draws from seed and tick (R104);
  the three owners of a start (R124): the block owns its `x0`, a diagram's `x0`, `n` and `state`
  are views of its blocks (setting `diagram.x0` writes through), the problem owns the plant's
  start (`x_start`, a leaf), and each stateful block's start is an `x0` leaf of the evaluator's
  tree; a realization is one tree (R144); a problem's tree gains `t_start` and a planner's tree is
  its problem's plus `"method"` and `seed` (R148).
- **Today:** `Set.params = None` (OD-24); distributions keep `mean` / `std` as attributes;
  `TrajectorySource` and `LookupPolicy` keep their tables as attributes; only the plant's slice of
  the start is a leaf, a diagram stores stacked copies of `x0`, `n` and `state` restacked by
  `refresh()`; `Computer` blocks read `op.bound_params`, frozen at compile; the MPC's
  `MPCTickLatch` is a hidden cache; `StepSystem.rollout` and `HybridDiagram.last_result` cache
  runs (D13); the path lookup and merge of parameter trees are written four times
  (`planning/evaluator.py`, `planning/problems.py`, `core/compile/evaluators/jacobian.py`,
  `core/assembly.py`).
- **Open:** the CONSTITUTION's amendments (invariants 1 and 3, §6's `%` sentence, §2's
  disturbance sentence): their text is proposed in QUESTIONS.md, to rule before the loop lane
  starts.
- **Rows:** [TODO §1.1](TODO.md#11-every-object-pure-every-number-a-parameter). Design:
  [randomness.md](randomness.md) (RN-2), [differentiable-problem.md](differentiable-problem.md) §8.

## 2. The step algebra first-class (invariant)

- **Goal.** `StepSystem` and `StepDiagramSystem` form a closed algebra on an integer tick
  (CONSTITUTION invariant 1), with the same grammar as the continuous one:
  - one same-tick semantic, propagation within the tick; a delay is a block (R125);
  - `@`, `>>` and `+` on step systems; mixing a continuous and a step operand refused, naming `%`,
    except `sampled @ continuous`, the one place the two worlds meet (R126, R146);
  - a pure `StepSystem` has no clock: `step(x, u, k)`, a chess game or a grid world; a clock is
    applied from outside (§5, R146);
  - `compute_rollout` on every step system returns a `Trajectory` (`StepRollout` folded), `t = k`
    on a pure step system, `t = k·Δ` on a sampled one (R147);
  - step rollouts trace end to end and batch over starts, seeds and parameter families
    (native-optimizers.md §3.1); the step tools linearize at a fixed point and read eigenvalues
    against the unit circle (R122).
- **Ruled:** R125, R126, R146, R147.
- **Today:** the discrete evaluators' `rollout` (`core/compile/evaluators/discrete.py`) returns
  host NumPy, with no step `rollout_batch`; `compile_step_diagram(flatten=True)` raises, and step
  diagrams refuse a continuous leaf; two same-tick semantics (`StepDiagramEvaluator` propagates
  within a tick, `Computer.tick` reads the last tick's buffer); `@`, `>>`, `+` refuse step
  systems; `jacobian` but no `linearize` or poles; the step evaluators compute the port signals
  twice per tick (in `step` and in `outputs`); one step formula (`stepping.py`'s `step_body`)
  under eight hand-written loops (`nominal_trajectory`, `RolloutEnvironment`, `Sys2Gym`, the
  realtime simulator, `HybridSimulator`, shooting, multiple shooting, the DP discretizer).
- **Open:** nothing at the design level.
- **Rows:** [TODO §1.2](TODO.md#12-the-step-algebra-first-class).

## 3. The draws (pillar)

- **Goal.** Stochastic problems on one convention ([randomness.md](randomness.md)):
  - a `Distribution` has no time; a problem is stochastic by its parts: where a number goes, a
    law may go (`x_start=` a point, a `Set` or a law; `params=` laws in the tree's own shape);
    `criterion=` and `nominal()` on the class; no `StochasticPlanningProblem` (R143);
  - a disturbance is a source block wired onto a port, never a bare law; a noise signal holds its
    draws over a sample period, its seed in `params`, drawn by a counter-based cipher, the same on
    NumPy and JAX (R128);
  - `problem.realize(key, n_trials) -> params`, one tree: a law in a leaf is drawn to a value
    from a sub-key derived by the leaf's path, a source block gets an integer seed leaf, every
    drawn leaf with a trial axis (R144);
  - Monte Carlo is the closed loop simulated on draws realized once from one key.
- **Ruled:** RD-D1–RD-D24 (in part superseded by R10, R15, R19, R27, R44); each trial realizing
  the loop's blocks (R92, landed); R128, R143, R144.
- **Today:** `StochasticPlanningProblem(sys, x0_distribution=, params_distribution=,
  disturbances=)`; `problem.realize(key, n)` returns `(x0s, {"params", "disturbances"})`, the key
  split by position, one held value per disturbance port; `evaluate` holds disturbances at
  nominal and warns; one integer key draws different values on NumPy and JAX (`trial_key`;
  KNOWN_ISSUES); `Sys2Gym.from_problem` applies no draw, although DESIGN §6 says it does; the free
  loop inputs are constant over a run; there is no `NoiseSource`.
- **Open:** nothing at the design level.
- **Rows:** [TODO §1.3](TODO.md#13-the-draws).

## 4. The seeded stochastic policy (pillar)

- **Goal** (direction 2026-10-22, a pillar 2026-10-26).
  `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)` is a plain `System` built like the noise block:
  `GaussianPolicy(mean, log_std, *, sample_period, seed=None)`, its noise internal and
  deterministic given the seed, `seed` an integer leaf; `seed=None` gives the mean; a seed per
  experiment from `realize(key)`; `log_prob` and `entropy` methods of the block; one draw
  convention, the sample index from the block's own period (R129). Its deterministic face is
  `NeuralPolicy` (renamed from `NeuralPolicyController` in the v0.3 sweep, R151).
- **Ruled:** R129, R151.
- **Today:** the RL Gaussian heads draw from their own keys.
- **Open:** whether the RL heads are rebuilt on the block in v0.3 or in v0.4 (a placement detail).
- **Rows:** [TODO §1.4](TODO.md#14-the-seeded-stochastic-policy).

## 5. The sampled loop: a step diagram, two runners (pillar)

- **Goal** ([sampled-path.md](sampled-path.md)).
  - **A clock is applied, never built in** (R145, R146): `sys % Δ` is
    `discretize(sys, Δ, solver="euler")` for any system (a law is implemented, its difference
    equation); `discretize(sys, Δ)` keeps RK4 (a plant is simulated). Both return a
    `SampledSystem(source, sample_period, *, solver, substeps)`: any system on a clock, a view of
    its source (its `params`, ports and state the source's), the period structural. It replaces
    and renames `Discretization`.
  - **Periods on blocks; the schedule derived** (R146): a diagram with a clocked block is a
    `SampledDiagramSystem`, its base period the gcd of its blocks'; a block of divisor `d > 1`
    fires on `k mod d = 0`, its held outputs state slots; a multi-rate computer is composition,
    `(filt % 0.01) >> (ctl % 0.1)`. No `Computer`, `StepSchedule` or `HybridDiagram` class:
    `computer @ plant` is a `SampledDiagramSystem` whose plant block is the plant on the base
    clock, with substeps.
  - **Two run verbs that agree** (R147): `compute_rollout` gives the ticks, what `evaluate`
    scores; `compute_trajectory` is the same scan with the plant's substeps kept (one new evaluator
    method beside `rollout`), bit for bit at the ticks; `solver="scipy"` is the host clock runner
    with an adaptive plant between samples, the cross-check.
  - **Two runners over the same pure blocks:** the step diagram (the algebra, the analysis,
    `evaluate`, `jit`, `grad`, `vmap`) and the host clock runner (realtime and game mode, hardware
    in the loop, host-only blocks such as an Ipopt MPC). Neither owns a copy of a block's
    equations; the step diagram is the runner's test oracle.
  - **The MPC** is a block of this loop, designed in [planner-and-mpc.md](planner-and-mpc.md) (§8).
- **Ruled:** R13, R14 (ticks), R104, R105 (`problem.compile` dispatches on the policy's kind),
  R123 (the MPC's one-period computational delay), R100 (the lazy `core` → `simulation.discrete`
  import), R125, R145, R146, R147. R146 revises R120 and R127: the loop is a `System` of the step
  algebra; CONSTITUTION invariants 1 and 3 and §6's `%` sentence are amended when it lands.
- **Today:** `Computer` is mutable and NumPy-only; `HybridSimulator` is a Python tick loop with no
  `params`, no roles and no problem; `StepRollout` is the computer's record; `problem.compile`
  refuses a `StepSystem` controller but accepts the stateless MPC, whose latch then runs at the
  RK4 stages (KNOWN_ISSUES); `%` refuses a law with continuous states.
- **Open:** whether the realtime simulator is re-based on the pure blocks in v0.3 or with the
  realtime review (v1.0); AC-8a's port names (automation-by-convention.md); D15, whether
  `problem.compile(controller, dt=)` on a sampled controller must equal the loop's base period
  or reads it (sampled-path.md §8.7); whether the host clock runner keeps the lazy
  `core` → `simulation` import once the hybrid modules go (R100); `dt` at the calls against
  `sample_period` on the object (QUESTIONS, the names entry).
- **Rows:** [TODO §1.5](TODO.md#15-the-sampled-loop-a-step-diagram-two-runners).

## 6. One score (pillar)

- **Goal** ([differentiable-problem.md](differentiable-problem.md) §9). A policy has one `J`,
  whatever tool produced it and whatever loop runs it:
  - **two problem kinds** (R141): `PlanningProblem` (continuous: `tf`, `discount_rate`, a cost
    whose `g` is a rate) and `StepPlanningProblem` (a `StepSystem`, a horizon `N`, a per-step
    `gamma`, a `StepCost` `c(x, u, k)`, `h(x, k)`), each stochastic by its parts;
  - **a problem is a description; its step face is compiled:** `problem.compile(dt=)` (no
    controller) gives `transition(x, u, k, params) -> (x_next, c_k)`, `N = tf/dt`,
    `gamma = e^{−ρ dt}`, the sets at `t_k`, the price; `problem.compile(controller)` scans it, the
    loop compiled with its plant block augmented to `(x_next, c_k)` inside its period body, so a
    stateful controller's loop is still one scan with one plant evaluation per period;
    `discretize(problem, dt)` (facade `problem.discretize`) is the explicit `StepPlanningProblem`,
    compiling to the same face; value iteration, the RL environment, the judge, shooting and the
    native MPC read only that face;
  - **the stage cost** is integrated with the state inside the step, in the stationary form
    `c_k = ∫₀^dt e^{−ρτ} g dτ`, `J = Σ γ^k c_k + γ^N h` (R142); under forward Euler it is
    today's `g·dt` of value iteration and RL; a logged run scored by `evaluate_trajectory` keeps
    the trapezoid;
  - **the RL discount** is the problem's `γ` by default; `gamma=` on the RL planner is a training
    hyperparameter only, never the scored discount (R150);
  - the continuous description stays primary for collocation, the continuous LQR, the linear
    analysis and `solver="scipy"`.
- **Ruled:** R141, R142, R150.
- **Today:** one policy gets three or four close `J`s (a left Riemann sum `g dt` in `dp.py` and
  the RL reward; a trapezoid in `score_run`; shooting's trapezoid with a first-order hold); the
  failure rules differ (`score_run` integrates to `tf` and checks `X` and `U` at every sample; the
  RL environment stops at the first exit from `X` and checks no `U`; value iteration prices the
  cells that leave its grid), written twice (the environment's step and `score_run`; R108 gives
  them one owner); the hold differs too: the planners' `nominal_trajectory` holds `u_k` over the
  period while `evaluate` closes the law as a continuous diagram and reads it at the RK4 stages,
  so a lookup policy is scored on two loops today.
- **Open** (QUESTIONS, the one-score entry): time-varying sets and the margins between samples;
  one failure rule (absorbing or not) and the input hold; shooting's reported score. And the
  discretized problem's cost, which R141 leaves with no object (QUESTIONS).
- **Rows:** [TODO §1.6](TODO.md#16-one-score).

## 7. The parametric program; the stochastic program its seed case (pillar)

- **Goal** (direction 2026-10-26; [parametric-program.md](parametric-program.md)).
  - One noun, `MathematicalProgram(..., params=None)`: its functions take `(z, params=None)`, the
    problem's tree, compiled once; the solve takes the values (`Optimizer(program).solve(params=)`),
    so the MPC and every other user rebinds without recompiling; no `ParametricProgram`, no
    `bind` (R130). One evaluator body and one backend table for the offline and compiled paths
    (OPW-1–OPW-5); a performance gate: if carrying `params` costs speed, an export option bakes
    them in.
  - The tree carries the start and its time: `x_start` (the start a planner pins, `X0` derived
    from it) and `t_start` (the horizon `[t_start, t_start + tf]`; the discount and the horizon
    counted from it, costs, sets and references reading absolute time) (R148).
  - Every program is deterministic given its parameters, the seed one of them; complete trees
    inside, one merge at the boundary, integers never in `wrt`, sizes structural (R131). The core
    keeps the sample-average export (`pe.program(wrt, key=, n_trials=)`); how a solver uses the
    seed (stochastic approximation) is an `Optimizer` strategy for later
    ([native-optimizers.md](native-optimizers.md) §7).
- **Ruled:** R26 (the key `plant`), R130, R131, R148.
- **Today:** `ParametricMathematicalProgram` varies `x0` only; collocation closes over the plant,
  cost and set parameters, and `require_online_params` refuses every other key; the MPC port
  functions drop `params`; `JaxParametricProgramEvaluator` repeats about half of the plain
  evaluator; the compiled path takes SciPy only; problems have no start time; shooting and
  collocation pin the start through `X0`; `pe.program(wrt, key, n_trials)` refuses `"x_start"` in
  `wrt` on a stochastic problem.
- **Open:** the name of `program.params_at(z)` (the names sweep); a performance budget for the
  gate (QUESTIONS).
- **Rows:** [TODO §1.7](TODO.md#17-the-parametric-program-the-stochastic-program-its-seed-case).

## 8. The native proof of concept: the planner's pure face and an MPC on it (pillar)

- **Goal** ([native-optimizers.md](native-optimizers.md) §1; [planner-and-mpc.md](planner-and-mpc.md)).
  - Native algorithms are pure array code, like a `System`; the solve is a rollout: a pure
    `init` / `step` kernel run by `Optimizer`, with a step-system view (R132);
    `Optimizer(program, method="adam", iterations=K)` is exactly `K` steps, one `lax.scan`, pure
    and differentiable; `iterations=None` runs to a tolerance (R149).
  - **A planner is a description; `planner.compile()` is its pure face** (R148): `n_z`,
    `initial`, `solve(params, z0) -> (z, status)`, `shift(z, dt)`, `policy(z, x, t)`,
    `prediction`, `solution`, `pack`, `program`; `planner.solve()` stays the convenience verb.
    Every planner fits: what differs is what its solution vector `z` holds.
  - **The MPC** is one block in two forms on that face (R148, R123): the delayed form, a
    `StepSystem` whose state is `(z, t_last)`, shifting and solving once per tick and applying the
    stored plan; the pure form, a static block. Its inputs are bindings to leaves of the planner's
    tree (`{"x_start": "y", "t_start": "t"}` by default), time on a `t` port wired by convention,
    its tree the planner's `params`.
  - **In v0.3, a proof of concept** (the direction): a fixed-step solver, the planner's face on
    trajectory optimization, the gradient MPC on it, its closed loop in one scan tuned by
    gradient. State-of-the-art solvers and MPPI are v0.4.
- **Ruled:** R132, R135 (answered by R148), R148, R149.
- **Today:** `gd` and `adam` are a Python loop on NumPy (`optimizers/first_order.py`), Adam's
  update jitted; nothing differentiates through a solve; the RL learners already run their steps
  in one `lax.scan`; the planners have no pure face, `solve` mutates them (`last_solution`); the
  MPC is two classes with `dt_mpc`, `t0`, `warm_start` and a tick latch, probing the planner.
- **Open:** the names in planner-and-mpc.md §11; one owner of the plan's time (`t_last` beside
  the origin in `z`) and the clock port `t` (QUESTIONS).
- **Rows:** [TODO §1.8](TODO.md#18-the-native-proof-of-concept-the-planners-pure-face-and-an-mpc-on-it).

## 9. Fields (pillar)

- **Goal** ([fields.md](fields.md); CONSTITUTION §2 lists `Field` as planned). A scalar function
  on `(x, u, t; p)` with `value` and `gradient`; the certificate's `V`, the LQR value, the
  cost-to-go and the critics speak it.
- **The v0.3 scope** (R133, R153): `Field` (today's `StateField` moved to `core/`),
  `QuadraticField` (its sublevel set the ellipsoid), the shaping functions in `core/`, and the
  conversions named by what they return: `as_set`, `as_input_set`, `as_cost` (the running cost)
  and `as_terminal_cost` (the terminal cost); `Set.as_cost` is `Field.as_cost` on the set's
  margin. Later, with their consumers (v0.4): `GridField`, `CallableField`, the approximator, and
  value iteration, the certificates and the critics rebuilt on the noun; value iteration's
  `cost_to_go` stays a callable until then.
- **Ruled:** R133, R153.
- **Today:** `StateField` lives in `planning/spatial/state_fields.py`, its `as_cost` a running
  cost only; `V(x)` and `cost_to_go` are not fields.
- **Open:** nothing at the design level.
- **Rows:** [TODO §1.9](TODO.md#19-fields).

## 10. The order: three lanes, two joins (a proposal, not ruled)

The lanes follow the dependencies (they replaced the phases 0–8 of 2026-10-26, whose two
backward edges reviewer C found; [the simplification record](../reviews/2026-10-26-plan-simplification.md)).
Every lane is ready to start once step 0 lands; only join 1 waits on the one-score question's
three open points.

| Lane | Workstreams, in order | Waits on |
| --- | --- | --- |
| **Step 0** (serial, before the lanes) | §1: the names sweep (R134, R151) and the one parameter-tree module (`core/params.py`): they touch every lane's files (`planning/evaluator.py`, `problems.py`, `jacobian.py`, `assembly.py`; `compile_backend` → `backend` across the API), so no lane starts before them | nothing |
| **Objects and draws** | §1: the contract, the textbook leftovers, distributions and sets reading `params`, the shaping to `core/`, `Set.as_cost` → §3: one draw by path on both backends, disturbances as source blocks, the one-tree `realize`, the problem stochastic by its parts, `NoiseSource` → §4: the seeded policy | step 0 |
| **The loop** (the critical path; it starts first) | §2: one same-tick semantic, the operators on step systems, step evaluators that trace and batch, one signal pass per tick → §5: `SampledSystem` (renamed from `Discretization`), periods on blocks with divisors and held outputs, `SampledDiagramSystem`, the two run verbs and the dense method, the host clock runner over the pure blocks, the hybrid demos re-pinned once | step 0 |
| **The program** | §7: one backend table, the program with parameters, `t_start` and the start from `x_start`, collocation reading the plant, the online call → §8: the native kernel, byte-identical, then `iterations=K`; the planner's compiled face on trajectory optimization | step 0 |
| **Join 1** | §6: `StepPlanningProblem`, `problem.compile(dt=)` and `transition`, the judge on the scan, then value iteration and the RL environment on `transition`, the RL discount derived | the objects and loop lanes, and the one-score question |
| **Join 2** | §8: the MPC rebuilt on the planner's face (both forms, the `t` port), its closed loop in one scan tuned by gradient; §7: the stochastic program as a convention and a test | all three lanes |
| **Beside, from §1 on** | §9: Fields, additive to the path | §1's `as_cost` signature |
| **Last** | hardening (S66, with the error family if moved in); the notebooks (R47); the merge | everything |

**The clean-up lane runs beside them** (TODO §2).

**Number changes** (R80–R82): an intended change is checked by closeness on the same problem and
cost; a restructure agrees to `rtol 1e-6`; a move or rename stays byte-identical; no alias, one
sweep. The intended changes, each a commit of its own with a ledger line, in this sequence so
each number moves once (reviewer C):
1. the sampled loop: the same-tick semantic (R125), then the run verbs' defaults (R147); the MPC
   demos, which `Computer` runs today, are not re-pinned here: they move again at 5 and are
   pinned once there;
2. the one-score `J` (R142), before any RL re-pin;
3. the cross-backend draws and live disturbances (R128, R144);
4. the RL re-pin with the seeded policy (R129) and the derived discount (R150);
5. the MPC's one-period delay (R123), with the MPC rebuilt on the planner's face (join 2): the
   MPC demos pinned once, here.

Each re-pins the regression baselines it moves, in its own commit; the student-facing numbers at
the merge (TODO §1's gates).

**Spikes before a lane starts** (what the vision's prototypes did not verify,
[its §10](../reviews/2026-10-27-consolidated-core-vision.md)): the hold slots and the divisors
inside the compiled step plan (`plan.py`) and the plant block augmented to `(x_next, c_k)` inside
the loop's evaluator, before the loop lane; the host clock runner with an adaptive plant between
held inputs (`solver="scipy"`), with it; the DP sweep on `transition` under JAX, before join 1;
the program's constraint rows on the compiled path with Ipopt inside a tick, before join 2;
`jax.checkpoint` on long loops under `grad`, when a demo needs it.

## 11. Definition of done (a proposal; the maintainer confirms)

From reviewer C §3.3, in the ruled names. v0.3 is done when these scripts run as tests:
1. **An LQG loop** with noise on `d` and `n`: the Monte Carlo mean is within the covariance
   prediction's sampling error, and one key gives the same draws on NumPy and JAX; the same loop
   with its computer sampled (`% dt`) scores with its noise blocks realized per trial (the
   draws × loop join).
2. **A digital PID** (`problem.compile(pid % 0.01)`): `evaluate(p)` runs and `jax.grad` matches
   central differences.
3. **The same digital PID** run by `compute_rollout` and scored by `evaluate` gives one `J`;
   `compute_trajectory` agrees with it bit for bit at the ticks, and `solver="scipy"` within its
   integration tolerance.
4. **One compiled MPC program**, rebound with two plant `mu` values and a moved target, matches
   the rebuilds; `method="ipopt"` works on the compiled path.
5. **The fixed-iteration MPC's closed loop:** `problem.compile(mpc % Δ).evaluate(p)` runs, `vmap`s
   over starts, and its gradient tunes a cost weight.
6. **A native step problem** (a grid world as a `StepPlanningProblem`) solved by value iteration
   on `transition`, and a continuous problem's value iteration and the judge, which scores the
   lookup policy through the sampled face (`policy % dt`, R105), giving one `J`.
7. **A `QuadraticField`** serves as a terminal cost (`as_terminal_cost`) and as the certificate's
   `V`, with the DP and RoA baselines unchanged.
8. **Every catalog plant and notebook** is green on the merge gate.

## 12. The core and the features

v0.4 reads the core and never changes it.

| Feature (TODO §4 unless named) | Reads | Plan |
| --- | --- | --- |
| MPPI (S72, MP-1–MP-6), a planner under the compiled-face contract | §3, §5, §6, §8 | [mppi.md](mppi.md), [planner-and-mpc.md](planner-and-mpc.md) |
| Identification and trajectory generation (C4) | §7 (the export with a data cost) | [differentiable-problem.md](differentiable-problem.md) §1 |
| `LQGPlanner` | §3 | [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §5 |
| Workspace geometry (A3), then the CBF safety filter (unscheduled; research lane until a course asks) | §9 | [geometry-module.md](geometry-module.md), [cbf-safety-filter.md](cbf-safety-filter.md) |
| The RL follow-ups (C5), the RL heads (open: here or §4), reward scaling (TODO §6) | §4 | [future-policies.md](future-policies.md), [rl-reward-scaling.md](rl-reward-scaling.md) |
| `GridField`, `CallableField`, the fields' consumers | §9 | [fields.md](fields.md) |
| Native solvers beyond the proof of concept (NS-2's tolerance form, NS-3–NS-7) | §7, §8 | [native-optimizers.md](native-optimizers.md) |
| Traceable linear analysis (S73) | the Jacobian | [differentiable-problem.md](differentiable-problem.md) §4.6 |
| The serial chain (C6), pyro parity (C1), C2, C3, S63, S64 | — | [pyro-port-remaining.md](pyro-port-remaining.md) |
| The estimation follow-ups (v0.9, TODO §5): the sampled filters | §5 | [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §6 |
| After v1.0: behavior cloning (a `fit` verb and a dataset noun, open), diffusion policy | §4, §5 | [future-policies.md](future-policies.md) §3–§5 |
