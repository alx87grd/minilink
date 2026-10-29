# The consolidation run (2026-10-29): the log

**Audience:** Prof. Alexandre Girard (Maintainer), for the next conversation.
**Context:** the run that implements [the consolidation plan](../plans/consolidation-after-the-runs.md)
(rulings R168–R179 in [the record](2026-10-29-consolidation-rulings.md)), with the amendments the
maintainer gave in the session that started it (§1). Where a review would have been asked, the
decision is taken as the maintainer would, written in §2 to be raised with him, and the run
continues.

**How to read it.** §1 is the amendments of the session and the summary. §2 lists every decision
taken without the maintainer (C-1…), each with the step, the choice, the reason and the
alternative. §3 is the ledger of intended number changes. §4 is the textbook-check statement of
every math diff. §5 is what was not done. §6 is the progress record, one line per pushed row. §7 holds the
drafts for the maintainer's own sections of ROADMAP §2.

## 1. The amendments of the session, and the summary

The maintainer's answers when the plan was checked (2026-10-29), each overriding the plan where
they differ:

1. **No refusal of `x >> (loop)`** (plan §5): composition stays simple; `>>` into a loop is
   always legal ("feed the loop's reference"); the docs warn that `@` binds before `>>`.
2. **The cost bound** (plan §4.2): one formula owned by the problem, `problem.cost_bound(tf,
   dt=None)`: the integral for the continuous judge, the sum at a tool's `dt` for every tool
   that steps on a grid (the face reads the problem's at its own `dt`).
3. **The failure flag** (plan §4.2): `penalize_failure: bool` (default off) and `failure_cost:
   float | None` (`None`: the cost bound) replace `infeasible_cost`; `cost_bound` is always
   computed; the word `price` goes.
4. **The plant's step** (plan §5): one rule, the continuous simulator's, no exception; `dt=` at
   the runs, `substeps=` at the compiles; every demo, notebook and baseline loop gets an
   explicit step so its numbers do not move.
5. **The MPC** (plan §6): `ModelPredictiveController(planner, delay=, warm_start=, …)` keeps both
   switches, independent; underneath it always builds the one three-block architecture (solve,
   command, memory), `delay=True` adds a visible `UnitDelay`; the default becomes
   `delay=False, warm_start=True`; `lookahead` goes; every demo sets the flags that keep its
   behaviour.
6. **The default start** (plan §7): the start region's bounding-box centre when finite and
   inside, else `sys.x0` when inside, else a refusal asking for `x_start`.
7. **The cart-pole RL demo** (plan §4.1): its reliability is not re-measured and it is not
   retuned; the maintainer handles it.

**The summary.** Steps 1 to 8 of the plan landed in order, one commit (or two) each, on
`dev-v0.3` (§6); step 9 is these records. Every step kept the seeded v0.3 baseline
byte-identical except the ledger's lines (§3): the RL reward (step 4), the shooting guess
(step 5), the delayed MPC on the box (step 7). The regression gates moved twice, both
intended and re-pinned with their reason: the bicycle problems' first guess (step 5, C-19) and
the delayed MPC (step 7, C-34). Nothing of the cart-pole RL demo was re-run (§5). The
decisions to raise first: C-30 (the judge's step on an undeclared plant), C-23 (the
continuous simulator's held-signal grid), C-19 with §3's gate line (the guess aiming at a
tracking target moves the MPC demos), C-33 (the dual rate's late plan), C-22, C-31.

## 2. Decisions taken without the maintainer

- **C-1 (step 1) The trajectory-optimization planner's offline key.** The plan said "built at
  construction, rebuilt by the user, or a key on the parts' identity". The key is the problem's
  parts by identity (plant, cost, `X`, `U`, `X0`, `Xf`, held with the program so no identity is
  reused), its horizon `tf` and its numbers' digest: a part replaced or a new horizon builds a
  new program (the test "a new horizon before a solve is a new program" keeps passing); an edit
  inside a part needs a new planner. Alternative: rebuild every solve (a JAX re-trace per solve).
- **C-2 (step 1) A transcription holds the plant it compiled** (`Transcription.plant_evaluator`,
  by the plant's identity and the backend), so a held planner's repeat solve compiles nothing
  (the reconstruction read the plant through the cache before). Alternative: pass the evaluator
  through every transcription method.
- **C-3 (step 1) The compile-once tests.** The cache's own cases go (a rebuilt loop reuses, a
  copy shares nothing, the eviction, the fingerprint, the port bound in the key, the run
  records no edit); the edit cases stay as the proof that a convenience call sees every edit
  (a function swapped, a helper edited, a class method replaced, an embedded model's numbers);
  the repeats are now a held tool's (a `Simulator`, a compiled sampled loop), and one test
  pins that a convenience call compiles each time.

- **C-4 (step 2) The spatial lane's numbers.** The workspace fields' trees are their parts':
  `ClearanceField.params = {"scene": scene.params}`, `CorridorMarginField.params = {"track":
  track.params}` (`{"path": {"waypoints"}, "half_width"}`); a `Scene` is `{"obstacles": …,
  "workspace_fields": …}`, each shape and each density its own (`Sphere {"center", "radius"}`,
  `Box {"lower", "upper"}`, `Union` by position, `Inflated {"base", "radius"}`, `GaussianField
  {"center", "amplitude", "sigma"}`), each with `with_params`. The body's probes are placed by
  the plant's own kinematics on the plant's numbers (one owner: the plant), so the field no
  longer hands its tree to the plant's `tf`. Alternative: the field's tree holding a copy of
  the plant's numbers too (a second home).
- **C-5 (step 2) No shared class-level `params = {}`.** The base classes (`CostFunction`,
  `StepCost`, `Set`, `InputSet`, `Field`, `Distribution`, `Shape`, `ReferencePath`,
  `WorkspaceField`, `Shaping`) read `params` through `core/params.py`'s `own_params()`: each
  instance's own dict, a new `{}` read from an instance that holds none, nothing written on
  read. Alternative: a read-only mapping that refuses a write; it is not a JAX pytree node,
  so a tree holding one (a field cost's `{"field": {}}`) would fail to trace.
- **C-6 (step 2) The numbers kept out of `params`, each with its structural reason** (the
  audit, `tools/audit_numbers.py` over every public system plus the core objects by hand):
  the drawn geometry of a skin read in `tf` (`DoublePendulum`/`Acrobot.l2` drawn, the cart's
  box, the cars' tracks and widths: rendering, never the dynamics); a summing junction's signs
  (`Sum.signs`, the wiring); a neural policy's action box (`u_mid`, `u_half`, the plant's input
  bounds; as leaves the RL algorithms would train them); a clock's period (it decides the
  plan); an LTI model's operating point (`xbar`, `ubar`, `ybar`, never in `f`); a
  `SampledCost`'s period, rate and origin (the face passes the problem's own leaves). Moved
  into `params`: the fields' (under `"field"`), the shapings' thresholds and scales (under
  `"shaping"`, the factories now return small `Shaping` objects; a plain function still works),
  `QuadraticField`'s schedule times `t`, `TrajectorySource`'s times (`{"t", "values"}`),
  `TimeCost.eps`, an LTI block's matrices (`{"A", "B", "C", "D"}`, also `TransferFunction`,
  `Lead`, `LowPassFilter`: the realization's matrices; the coefficients stay for display),
  `LookupPolicy`'s table and levels (`{"u_table", "levels"}`, now a traceable multilinear or
  nearest lookup; it equals SciPy's to the bit on a 2-D grid), the shapes', paths' and
  densities' numbers (C-4), a model-based law's model (C-7).
- **C-7 (step 2) A model-based law's model numbers** (the TODO §1.1 row, landed here):
  `ComputedTorque`, `SlidingMode`, `JointImpedance`, `TaskImpedance`, `TaskKinematic`,
  `TaskKinematicNullspace` hold `params["model"]`, a deep copy of the plant's numbers taken at
  construction, and call the model hooks with it. **Behaviour change:** an edit of
  `plant.params` after the law is built no longer reaches the law's model (it did, through the
  shared object); a mismatch is `law.params["model"]["m"] = …`. DESIGN's "Embedded-model params
  rule" is rewritten. **To raise:** the deep copy rather than an alias of the plant's dict (an
  alias would make "the model's numbers" and "the plant's numbers" one edit again).

- **C-8 (step 3) The method's numbers through the optimizer.** The native laws hold
  `params` (`GradientDescentLaw {"learning_rate"}`, `AdamLaw {"learning_rate", "b1", "b2",
  "eps"}` and `"max_grad_norm"` when clipped, the clip's presence structure);
  `Optimizer.params` is the program's tree with `"method"`, and `Optimizer.solve(params=)`
  splits `"method"` off for the law (an argument of the jitted run, so a new learning rate
  never recompiles). A host method (SLSQP, trust-constr, Ipopt, L-BFGS) has `"method": {}`:
  its options are its stopping rule, kept as constructor options. The trajectory-optimization
  planner's tree is the program's leaves, `"method"` and `"seed": None`; `planner.solve(params)`
  and `pv.solve(params)` split the method's numbers off before the problem copy.
- **C-9 (step 3) The RL planner's tree is a report.** `planner.params = {"method":
  algorithm.method_params, "seed": seed}`: the discount and the algorithm's numbers (PPO's
  `gae_lambda`, `clip_range`, `ent_coef`, `vf_coef`; SAC's `tau`) and its Adam's under
  `"optimizer"`. Their home stays the algorithm's attributes, read when the update is first
  traced (set them before learning); no `solve(params=)` rebinds them. **To raise:** a
  training run with a tree argument (every update reading the numbers as arguments) is
  possible, and a larger change of the four algorithms.
- **C-10 (step 3) MPPI's tree** gains `cost`, `discount_rate` and `X`'s numbers (when `X` has
  some); the environment's step merges a given cost and discount into the face's tree and reads
  a given `X` (the input box the samples are clipped to stays structure, read at build). The
  failure price is the problem's at build (a moved weight does not move it; §4 rewrites it).
  An MPPI MPC on a traced loop (`problem.compile(mpc % dt, backend="jax")`) fails because the
  face's `initial(params)` is host-only; that is plan §6's traced first guess, landed there.

- **C-11 (step 4) One cost-bound formula, each tool over its own sum's horizon.** The
  amendment's formula is `problem.cost_bound(tf, dt=)`; the face reads it at its period
  (`face.cost_bound` over its `N`, `face.cost_bound_over(N, γ)` for a tool). The RL
  environment takes it over its episode (an infinite-horizon problem trains in episodes of
  `episode_length`; the geometric limit does not exist undiscounted) and value iteration over
  its sweeps (to convergence: the geometric limit when `α < 1`). So the plan's "three tools
  give one number" holds where their horizons agree (a finite undiscounted problem: the test);
  on one pendulum with a discount they still answer their own sums (the review's 193.8, 198.7,
  314.3 stay three numbers: the judge's integral, the episode's sum, the sweeps' limit), and
  the discretized problem now answers the face's number (its fourth number is gone). **To
  raise:** whether value iteration should bound over the problem's horizon instead (it sweeps
  to convergence, so its cost-to-go can exceed the finite sum).
- **C-12 (step 4) The failure switch and cost** (the amendment): `penalize_failure` (bool) and
  `failure_cost` (number or `None`); `infeasible_cost=True` became `penalize_failure=True`,
  `infeasible_cost=x` became `penalize_failure=True, failure_cost=x` in every test, demo,
  notebook and `docs/make_assets.py` (the old number charged both the judge and the tools, so
  both keywords keep the behaviour). `failure_cost` is a field of the problem, not a leaf of
  its tree (a failure cost is a step, with no gradient). The nouns: `failure_cost_at`,
  `failure_cost_of`, `failure_cost_and_bound`, `grid_failure_cost`, the environment's
  `failure_cost`; `price` is gone from `minilink/` (notebook prose keeps its own words).
- **C-13 (step 4) The RL reward** `r_k = −g(x_k, u_k, t_k) dt`, one call of `self.cost.g` on the
  plant's whole input, zero after a step that blew up (as `c_k` was); `reward="stage"` on the
  environment and the RL planner gives `−c_k`. MPPI's environment and the tabular learner use
  `reward="stage"` (MPPI scores its rollouts on the problem's stage cost; the tabular table
  approaches value iteration's). The record and `describe()` say "trained on −g dt, scored on
  the problem's cost". Acceptance script 6 builds its environment with `reward="stage"` (the
  score's reward), so its "one J" still holds. A native step problem has no `g`: its reward is
  `−c_k`.
- **C-14 (step 4) A tool's own discount.** Value iteration with `alpha=` on a continuous
  problem plans on `problem.replace(discount_rate=−ln α / dt)` (the grid rebuilt on it, a given
  grid's shape kept); `ValueIterationRecord.discount_rate` names it and its `__str__` says
  "planned at rho=…, scored at the problem's". The RL planner with `reward="stage"` and its own
  `gamma` builds its environment at `ρ = −ln γ / dt` the same way (the default reward needs
  none: `−g dt` carries no discount inside the period); its record's `scored_gamma` stays the
  problem's.

- **C-15 (step 5) The goal point.** `x_goal` is gone from `PlanningProblem`,
  `StepPlanningProblem` and every caller; `problem.goal_point()` is the plan's heuristic end
  (`Xf`'s point: a singleton's, a ball's centre, else a finite bounding box's centre; else the
  cost's `params["xbar"]`; else the start). It feeds the default initial guess, the DP grid's
  plot anchor (before: the goal, else the grid's centre), the comparison's anchor, RRT's goal
  bias and best effort, and the planner's verbose report (`terminal_error` only with an `Xf`).
- **C-16 (step 5) `start_law` stays** the name of the law a problem draws its start from (the
  plan's "the `start_law` attribute becomes the law held in `x_start`" is read as: the law is
  given through `x_start=` and is the tree's `"x_start"` entry, as it already was);
  `problem.x_start` is a property (the point, a law's mean) whose setter is `move_start`, so
  `problem.x_start = v` moves a singleton `X0` (the review's desync); the start's singleton
  `X0` is no longer an entry of `problem.params` (one home: `"x_start"`).
- **C-17 (step 5) RRT's goal region is `Xf`; `goal_tolerance` is removed** from `RRTOptions`
  and `RRTStarOptions` (it only made a ball around `x_goal`): `Xf=BallSet(goal, tol)` says it;
  a singleton or missing `Xf` is refused with that line. The goal bias now always draws (it was
  skipped when no `x_goal` was given), toward `goal_point()`. **To raise:** the option's
  removal (a public keyword) follows from R177 but was not named in it.
- **C-18 (step 5) The per-demo `x_goal` lines** ([ask] for the maintainer, plan §9): deleted
  where the cost's `xbar` (or the `Xf` already given) is the same point, so nothing moves: the
  five value-iteration demos, `readme_examples.py` and README, the showcase notebook,
  `docs/make_assets.py`, `trajopt_holonomic_corridor.py`, `rrt_pendulum_swingup.py`, the eight
  GRO860 and RL-topic value-iteration notebooks (their 2-D grids pin no axis), the
  benchmarks' value-iteration and parity problems; turned into `Xf=BallSet(goal, tol)` where
  RRT needed a region (`rrt_holonomic_obstacles.py` with its two tolerances,
  `rrt_star_live.py`, `benchmarks/planning_rrt.py`, the test fixtures). No demo turned an
  `x_goal` into a hard `Xf`.
- **C-19 (step 5) The default initial guess now aims at the cost's target.** With no `Xf`, the
  straight-line guess ran from the start to the start (a constant); it now runs to the cost's
  `xbar` (the plan's rule). A problem with no `Xf` and an `xbar` away from its start starts
  its solve from another guess: the v0.3 baseline's `trajopt.shooting.*` moved (6e-4
  relative, re-pinned); the regression gates say whether any other plan moved (§3).
- **C-20 (step 5) A region start's default** (the amendment): the bounding box's centre when
  finite and inside, else `sys.x0` when inside, else a refusal naming `x_start` (before:
  `sys.x0`, refused when outside).
- **C-21 (step 5, found in step 6) The guess's target inside `X`.** C-19 aimed the straight-line
  guess at the cost's `xbar` even when `xbar` lies outside the state constraint `X`; an MPC test
  (`X` a ball of radius 0.5, `xbar = 2`) then left the solver infeasible. `goal_point()` now
  takes `xbar` only when `X` contains it, else the start. Landed in the step 6 commit.
- **C-22 (step 6) What a clock made of its source is the view's `kind`.** `SampledSystem.kind`
  is `"law"` (`block % Δ`), `"model"` (`discretize`) or `"plant"` (the view `closed_loop`
  wraps on the loop's base clock; never the user's). The plan's `plant_op` reads `kind ==
  "plant"`; `resolves` is false on a law, so `pid % Δ` alone is never integrated inside its
  period and a chain of two Euler laws is a computer with no plant. A loop wired by hand
  (`SampledDiagramSystem.add_subsystem`) on a `discretize`d model is a loop of step systems,
  ticks only: there is no public way to hand-wire a continuous plant into a sampled diagram
  (`@` or `closed_loop` wraps it). **To raise:** whether a hand-wired loop needs one
  (`SampledDiagramSystem.add_plant(plant, "plant")`, say).
- **C-23 (step 6) The continuous simulator's held-signal grid stays.** The plan removes
  `HELD_SIGNAL_STEPS_PER_SAMPLE`, the loop's exception (one step per period for a plant that
  declares no time constant); it is gone from the sampled loop. The continuous `Simulator`
  read the same constant for a held source (a `WhiteNoise` or a sampled policy in a continuous
  diagram): removing it there too would move the numbers of the noise demos and notebooks
  (`lqg_cartpole`, `lqg_monte_carlo`, the state-space notebooks) and make them about 100
  times slower (`Δt = 0.1 ms`). The Simulator keeps it. **To raise:** one rule there too,
  with the demos given an explicit `dt`?
- **C-24 (step 6) Long substep runs are rolled under JAX.** At the automatic step, a plant
  that declares no time constant takes `⌈Δ / 0.1 ms⌉` RK4 steps per period (1000 at
  `Δ = 0.1`), which unrolled would make a JAX trace of every step. More than
  `UNROLLED_SUBSTEPS = 64` run in one `lax.scan` (`stepping.rolled_steps`); up to 64 stay
  unrolled, so every existing configuration traces as before.
- **C-25 (step 6) The knobs:** `dt=` on `compute_trajectory` and `compute_rollout` (the plant's
  step, `⌈Δ / dt⌉` per period), `substeps=` and `solver=` on `loop.compile` and
  `problem.compile` (`substeps=None` the automatic step; a continuous loop's default stays 1),
  `solver` on `compute_trajectory` widened to `None | "rk4" | "euler" | "scipy"`. A diagram
  with no plant refuses them by name. `compute_rollout(record=)` keeps the ticks of the dense
  run (the same states bit for bit), and a loop with a plant always records `"plant:u"` and
  its role there too.
- **C-26 (step 6) The demos' loop lines.** Every `@ discretize(sys, Δ, substeps=round(Δ /
  SIM_DT))` became `@ sys` with `dt=SIM_DT` at the run (the same substeps, so the same
  numbers): 12 demos and projects, 3 notebooks, the v0.3 baseline (byte-identical: all 116
  entries agree at rtol 0), `f_mpc_parity`, `docs/make_assets.py`; the compile demos'
  `problem.compile(ctl % DT)` (plants with no declared time constant, one step a period
  before) name `substeps=1`: `digital_pid_tuning`, `lqg_monte_carlo`, `native_mpc_tuning`,
  `terminal_field_mpc`, `benchmarks/v03_budget.py`, `benchmarks/compile_once.py`. A loop on
  an undeclared plant that relied on the old one-step exception (the showcase notebook's
  `(ctl % 0.1) @ plant`) now integrates at the automatic step: its numbers move (more
  accurate, slower); left without a `dt` since the old step was the exception the plan removes.
- **C-27 (step 6) `NativeSampledSystem`'s period at construction** (rule 1) lands with step 7:
  its one subclass is the delayed MPC, which step 7 deletes.
- **C-28 (step 6) z-domain analysis** (R175): `DiscreteLTISystem` (a `StepSystem`, params
  `{"A", "B", "C", "D"}`, `period` a description, not a clock); `linearize` on a step or
  sampled system returns it; `bode`, `margins`, `nyquist`, `frequency_response` read
  `G(e^{jωΔ})` on a grid three decades up to `π/Δ` (per tick for a tick law); `plot_pzmap`
  draws the unit circle. The root-locus and step-response tools stay continuous-only (not in
  the plan). The demo: `examples/demos/analysis/analysis_digital_pid_poles.py`.

- **C-29 (step 6, then step 8) No `>>` refusal** (amendment 1). Step 6 first landed the
  plan's refusal of a law chained into a closed loop; it contradicted the maintainer's
  amendment and was removed in step 8: `>>` into a loop is always legal (it feeds the
  reference), and DESIGN and `sampled-path.md` §8.4 say that `@` binds before `>>`.
- **C-30 (step 6) The judge integrates at the continuous simulator's step.** A held law
  scored by `problem.compile(law % dt)` (and so every planner's `evaluate`, the evidence of
  value iteration, the RL and MPPI planners) integrates a plant that declares no time
  constant at `Δt = 0.1 ms`, not in one RK4 step a period: it no longer equals the step face
  to the bit (script 6: 7e-5 relative), and `substeps=1` gives the face's number back (the
  test checks both). The compile demos and budgets that relied on the old step name
  `substeps=1` (C-26). **To raise:** whether a planner's evidence should score on its own
  step model instead.
- **C-31 (step 6) `experimental/` is left as it is.** Three scripts there close loops on
  `discretize(...)` (`experimental/boat/parking_compare.py`, `parking_mpc.py`,
  `experimental/mppi/racecar_mppi.py`); under the grammar they become loops of step systems
  (ticks only). They are the maintainer's scratch (AGENTS.md: never without an ask).
  **To raise:** move them to `@ plant` with `dt=`?

- **C-32 (step 7) One box, `PredictiveController`.** `ModelPredictiveController(planner, *,
  inputs=, delay=False, warm_start=True, backend=, verbose=)` always builds the box `solve`,
  `command`, `memory`, plus `delay` (a `UnitDelay` of `n_z` from the default guess, between
  the solve and the command) when `delay=True`. `lookahead`, `DelayedPredictiveController`,
  `PurePredictiveController`, `WarmStartedPredictiveController`, `predicted_tree`,
  `predicted_start`, `pinned_shift` and the evaluator's stateless-MPC refusal are gone (the
  box with no clock is a tick diagram, refused by name). Every caller of the old default
  names `delay=True`: the two compile demos, `docs/make_assets.py`, `v03_baseline`,
  `v03_budget`, `compile_once`, `f_mpc_parity`'s zero-order hold, `test_v03_core`'s script 5.
- **C-33 (step 7) The `z` port is the plan solved at this tick**, in both forms (the old
  delayed form's `z` was its stored, one-period-old plan). A dual-rate computer that wants the
  plan one slow period late writes the unit delay itself:
  `(mpc % slow) >> (UnitDelay(dim=n_z).rename_ports(u="z") % slow) >> (PlanFollower(planner) %
  fast)` (`f_mpc_parity`'s dual rate and script 5's dual-rate test now do; the rename makes
  `>>` take the plan, not the command). **To raise:** a `PlanDelay(planner)` block would read
  better than the rename.
- **C-34 (step 7) The delayed form's warm start is the box's.** The old delayed law warmed its
  solve with the plain shift of the stored plan, and its tick 0 from the shift of the default
  guess; the box pins the first state knot at the measurement (`pv.shift(z, t, x_start=)`)
  and starts tick 0 from the default guess at the measured start. The plan expected this
  ("the delayed-form demos move once"): `mpc.integrator.x` of the v0.3 baseline moved 4e-7
  relative (re-pinned), the two compile demos and the delayed regression entries moved (§3).
- **C-35 (step 7) `x_ref` and `u_ref` on every form**, from the applied plan
  (`PlanCommand`, when the plan carries states); before, on the delayed form only.
- **C-36 (step 7) The traced default guess** (the review's D-4): `pv.initial(params)` under a
  trace is `z(x̄) + M (x̂ − x̄)` with the traced `t_start`, `M` the guess's derivative in the
  start read once on the host by unit steps (exact: both default guesses, the straight line
  and the cubic, are affine in their start); the tree's other numbers are the compiled
  defaults. MPPI's `initial` concatenates a traced `t_start`. The test: a jitted `initial`
  equals the host's at a moved start.
- **C-37 (step 7) The tuning path** is `controller.solve.cost.R`: `native_mpc_tuning`'s
  `wrt` and script 5's gradient read it there.
- **C-38 (step 7) `NativeSampledSystem(n, sample_period=)`**: the period is required at
  construction and `%` on it is refused (rule 1; C-27).

- **C-39 (step 8) `hold_x0` is a top-level leaf** of the evaluator's tree
  (`pe.params["hold_x0"]`, present when the loop holds a slow block's output): the loop is the
  tree's root, so the plan's `params["<loop>"]["hold_x0"]` has no `<loop>` key to live under.
  A layout change under a start that is not zero is refused (the zero start of a fresh
  layout is still reset silently, as every block's is). The slot reads `controller:u
  (memory)`.
- **C-40 (step 8) The compile probe on every port.** A port whose function is not the
  shortcut's `h` is evaluated only when it has an undeclared input (once, then once per such
  input), so an MPC's solve, which declares every input it reads, never runs at compile; a
  function that raises at the probe point is not judged. The KNOWN_ISSUES entry is gone.
- **C-41 (step 8) `RolloutEnvironment` on a step problem is refused by name** (the plan's
  first option): a native step problem has no `dt` and is stepped on its face; a discretized
  one names its origin and period (`RolloutEnvironment(origin, dt=0.1)`). Reading the face's
  `N`, `γ` and bound instead is a larger change (the sets, laws and sources of a step
  problem); not done.
- **C-42 (step 8) The planners' evidence carries the face's running cost.**
  `nominal_trajectory` (value iteration, LQR, the RL planners, tabular) steps the face's
  `transition_at` and records `C_{k+1} = C_k + γᵏ c_k` with the plant role, so
  `evaluate_trajectory(problem, evidence)` is the face's own `J`; the states are the same
  bits as before (the face's `transition` is its `step`'s to the bit).
- **C-43 (step 8) One Euler stage-cost rule** (`stepping.euler_stage_cost`), used by
  `step_with_cost`, `stage_cost` and `SampledCost.stage_cost_at` (which had a copy of its
  own reading `g` directly: `e^{−ρ·0} = 1` exactly, so the same bits; the v0.3 baseline
  agrees at rtol 0).
- **C-44 (step 8) A law's `with_params`** builds the law from its numbers,
  `type(law)(**params)`: a library law's `params` name its constructor's arguments (its
  `params` is a view of its attributes, not a stored dict, so the set pattern's copy cannot
  hold them). `BoxSet.sample` already read `params`; `StepCost` and the shared class-level
  dicts landed in step 2 (`own_params`, C-5).
- **C-45 (step 8) MPPI's key on the solve's time.** The tick key stays
  `fold_in(seed, round((t0 − t_origin)/dt))` on the planner's grid (so every run on that grid
  keeps its draws); a solve between two knots folds in its offset in millionths of `dt`, so
  an MPC faster than the planner's `dt` no longer reuses one key on two solves.
- **C-46 (step 8) `randomness.md` §2** states the cipher's reach (uniforms on a 2⁻²⁴ grid,
  normals bounded by `√(50 ln 2) ≈ 5.89`) and the CRC-32 stream names. `function_identity`
  went with the cache in step 1.

## 3. Ledger of intended number changes

| Row | What moved | Why |
| --- | --- | --- |
| Step 4, the RL reward | `rl.ppo.*` and `rl_env.*` of the v0.3 baseline (`rl_env` 1.5e-3 relative, the PPO history 1e-8 to 4e-3); re-pinned in `benchmarks/baselines/v03_core.json` (those keys only) | the training reward is `−g dt` (R171) |
| Step 5, the default initial guess | `trajopt.shooting.u`, `trajopt.shooting.x` of the v0.3 baseline (6e-4 relative), re-pinned | the guess aims at the cost's `xbar` when there is no `Xf` (C-19) |
| Step 5, RRT's goal bias | an RRT problem with an `Xf` region and no `x_goal` draws its goal bias now (none of the demos: each had an `x_goal` at the region's centre) | C-17 |
| Step 6, the loop's automatic step | a sampled loop on a plant that declares no time constant, run with no `dt`: before one RK4 step per period, now `⌈Δ / 0.1 ms⌉` (the showcase notebook's `(ctl % 0.1) @ plant`); every demo, project and baseline loop that named its step keeps its numbers (C-26) | R174, one rule |
| Step 6, the judge on an undeclared plant | `problem.compile(law % dt)` and the planners' `evaluate` (value iteration, RL, MPPI evidence) on a plant that declares no time constant: the plant at the automatic step, not one RK4 step a period (script 6: 7e-5 relative) | C-30 |
| Step 5, the default guess on a tracking cost | the MPC parity gates (`f.hybrid_zoh`, `f.hand_loop`, `f.dual_rate`: up to 0.36 in the bicycle's states) and `e4.bicycle_parametric` (0.12): the bicycle problems have no `Xf` and a cost target `x_ref`, so the first guess now aims at it (C-19); found by the step 6 regression run, re-pinned after step 7 | C-19 |
| Step 7, the delayed MPC on the box | `mpc.integrator.x` (4e-7 relative, re-pinned), `f.hybrid_zoh` (the delayed zero-order hold), `native_mpc_tuning` and `terminal_field_mpc` | C-34 |
| Step 4, value iteration's `alpha` | no baseline entry moves (the baseline's value iteration runs Euler, where the two discounts agree); under RK4 the backed-up `J` is the consistent one (the new test) | R173 |

## 4. Textbook check

Every math diff was read beside `planning/dynamic_programming/dp.py`.

- Step 1: no equation line changed.
- Step 7: `CompiledTrajectoryPlanner.initial`'s traced guess (`z(x̄) + M (x̂ − x̄)`), the
  pinned `shift`, `model_prediction` (RK4 a quarter knot apart) and MPPI's `initial`, read
  beside `dp.py`.
- Step 8: `euler_stage_cost` (`c = dt ċ(x, u, t)`), the evidence's running sum
  (`C_{k+1} = C_k + γᵏ c_k`), MPPI's `tick_key`, read beside `dp.py`.
- Step 6: `steps_per_period` (`n = ⌈Δ / Δt⌉`), `rolled_steps` (the RK4 substep in a scan, the
  input line `u0 + (i/s)(u1 − u0)`), `DiscreteLTISystem.step`/`h`, the discrete response
  `z = e^{jωΔ}`, read beside `dp.py`.
- Step 4: the reward line (`g = cost.g(...)`, `reward = −g dt`), `sampled_cost_bound` (ρ from
  γ), `tool_rate_of`, the score's charged term, read beside `dp.py`.
- Step 2: the fields' margins and costs, the shapes' `sdf`, the paths' distance and arc
  lengths, the densities, the LTI matrices, `TimeCost`, `TrajectorySource` and the new
  `LookupPolicy.action` were read beside `dp.py`: bindings (`params`, then `xp`), the
  parameters one per line, the equations as named lines with their textbook comments, a
  name returned. `arc_lengths` (ℓᵢ and sᵢ) is a generic building block shared by four
  methods (RULES 7.8).

## 5. Not done

- The cart-pole RL demo's reliability (the maintainer's, §1 item 7): the reward change moves
  every RL demo's training; none was re-run for its numbers.
- The three loops in the top-level `experimental/` folder still close on `discretize(...)`
  (C-31, yours to move).
- The RL environment on a native step problem refuses (C-41); reading the face's `N`, `γ`
  and bound is a later row.
- The z-domain root locus and step response (C-28): `linearize`, `bode`, `margins`,
  `nyquist` and `pzmap` read `z`; `root_locus` and `step_response` stay continuous.
- ROADMAP §2 is not edited: the MPPI sentence and the breaking-changes entry are drafts (§7).

## 6. Progress

| Step | Commit | Row |
| --- | --- | --- |
| 1 | `dd7822d4` | the compile cache out (R168) |
| 2 | `09bf4b71`, `c342acbb` | every number a parameter (R169) |
| 3 | `5bbf5058` | one tree per planner (R170) |
| 4 | `0a79f9de` | the one score's edges (R171–R173) |
| 5 | `fcd1da56` | the problem's boundary dials (R177) |
| 6 | `52be6eb2` | the loop's grammar (R174, R175) |
| 7 | `a6c16fff`, `51b452b9` | one MPC architecture (R176); the moved gates re-pinned |
| 8 | `920aeedb` | the details the review owns (R178, R179); the `>>` refusal removed |
| 9 | this record's commit | the records: DESIGN, ROADMAP §3, TODO, core-architecture, the drafts of §7 |

## 7. Drafts for the maintainer (ROADMAP §2 is his)

**MPPI's lane** (plan §8): beside the provisional spatial names, in ROADMAP §2:

> **Provisional planning names:** `MPPIPlanner` and `MPPIRecord` sit on `minilink.planning`
> as a research-lane planner with no stability promise, provisional until the teaching
> decision (MP-6, V3) says whether a course uses it.

**The breaking-changes entry** for the two runs and this consolidation:

> - 2026-10-29 (on `dev-v0.3`, the consolidation of the two night runs; every demo, project,
>   lesson and baseline migrated in the same commits): no compile cache, a held tool compiles
>   once (R168); every number of a field, a shape, a path, a scene, an LTI block and a
>   model-based law is a `params` leaf (R169); a planner's tree holds its method's numbers and
>   its seed (R170); the RL training reward is `−g dt` and the stage reward its option, one
>   cost bound `problem.cost_bound(tf, dt=None)`, `penalize_failure` and `failure_cost`
>   replace `infeasible_cost` (R171–R173); `x_goal` is gone, `X0` and `Xf` are the sets,
>   `x_start` the evaluation start, RRT's goal is an `Xf` region (`goal_tolerance` gone) and a
>   guess aims at `Xf`'s point, else the cost's target (R177); `%` is the computer, the plant
>   a loop closes on is integrated at the continuous simulator's step (`dt=` at the runs,
>   `substeps=` at the compiles, no loop on `discretize`), `linearize` on a step or sampled
>   system is a `DiscreteLTISystem` (R174, R175); the MPC is one box, `delay=False,
>   warm_start=True` by default, `lookahead` gone, its tuning path `controller.solve.cost`
>   (R176).
