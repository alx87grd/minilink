# Rulings session 1: the shape, the convergence, the step model (2026-10-22)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** The first of the two rulings sessions ROADMAP §5.3 schedules before wave K and
wave S code, run item by item over [end-state-api.md](../plans/end-state-api.md) D1–D10. This
record is appended as the session proceeds; a ruling leaves ROADMAP §6 when it lands here and
in the plan doc it shapes.

## Rulings

- **R1 (D1) The shape: two foundations, six tooling pillars.** Modelling is the primary
  foundation (the `System` and its algebra, the catalog plants, compile and the rollout);
  optimization the secondary one (the `MathematicalProgram` and the `Optimizer`, the solver
  boundary every pillar lowers to); on them six tooling pillars: classical and modern control,
  nonlinear control, trajectory optimization and MPC, reinforcement learning, identification
  and imitation, estimation. RRT a side pillar kept small; deployment experimental until a
  vehicle runs it. The constitution sentence is the maintainer's to place. Lives in:
  end-state-api.md D1; core-consolidation.md §2 reads with this amendment (plants are
  foundation, not optional content).
- **R2 (D2) The catalog clean-up: two import layers, support by test.** No catalog split by
  architecture: a plant is a leaf. The root prelude carries the course set (at most 15
  plants); `minilink.catalog` is the one flat index of every supported plant, each passing the
  both-backends contract test with a docs entry; unsupported plants the maintainer names
  from K1's table move to `minilink/experimental/catalog/`, untested, off the docs site, out
  of the wheel, free to be abandoned; twins merge; the dead helpers go. The defining module
  is not a documented layer. Lives in: end-state-api.md D2; TODO §4 K8 (steps K8-1 to K8-6);
  core-consolidation.md §6.1.
- **R1 amended: the problem statement is foundation too.** Beside the model (primary) and the
  optimizer (secondary), `PlanningProblem` / `StochasticPlanningProblem` with
  `PlanningSolution` (and the estimation problem as its dual) is the third fixed object every
  pillar reads: model, problem, program — what, what for, how.
- **R3 (D3) `experimental/engines/` stays, the tier goes untested.** The engines stay in
  `minilink/experimental/` as a demo of a contact-engine leaf, clearly not core. The
  experimental tier (`minilink/experimental/` for importable prototypes; a top-level
  `experimental/` folder for prototype scripts, moved out of `examples/`) is repo-only,
  untested by default (nightly smoke at most), off the docs site, never cited by a course, and
  the staging area for what may leave into its own project at v1.0 (with K5). `c_export`'s
  flagship smoke test is the one gate exception. Lives in: ROADMAP §2 (lane row); TODO §4 K9;
  end-state-api.md D3.
- **R4 (D4) No exit rule.** The maintainer prunes by hand at each release from a K1-style
  table. core-consolidation.md §8's rule is not adopted; the seeds have no clock.
- **R5 (D5) The prelude trim is silent.** Names leave the root prelude in the release, one
  ledger line per release in ROADMAP §2 naming the new import; no shim before V4 (v1.0).
- **R6 (D6) The step model: `discretize` returns a `StepSystem` on one compiled step.**
  `discretize(plant, dt, *, method, substeps, hold) -> StepSystem`, `step(x, u, k, params)` the
  one body with `params` always an argument and the parameterless call a binding;
  `integrator=` becomes `method=` (one release accepted with a ledger line); every sampled
  consumer (RL, DP, the gym bridge, the hybrid and realtime simulators, RRT, MPPI, later
  identification and imitation) calls this step. Lives in: sampled-path.md §3; TODO §4 S75.
- **R7 (D7) The four knobs and their defaults.** `dt` required and owned by the step model;
  `method` in `rk4` / `euler` / `exact`, default `rk4`; `substeps` default 1; `hold` in `zoh` /
  `foh`, default `zoh`; `substeps` exposed on every consumer that takes `dt`
  (`ReinforcementLearningPlanner`, `Sys2Gym`, `MPPIPlanner`, `HybridSimulator` in place of
  `dt_inner`). Today's numbers everywhere.
- **R8 (D8) DP stays on Euler through the step model.** `method="euler"` the default,
  byte-identical; `DynamicProgrammingPlanner(problem, dt=, method="rk4")` available; whether
  the default flips is V3's question.
- **R9 (D16, D20) The cost of a trajectory: one formula, the cost owns `g` and `h`, the
  problem owns the rest.** `evaluate_trajectory(problem, traj)` is the one body (today's
  `score_trajectory` and `CostFunction.evaluate_trajectory`): trapezoid on the trajectory's
  grid, the problem's discount, cut at the first exit and charged `infeasible_cost`, `h` at a
  finite `tf`; byte-identical on NumPy, traceable on JAX; an infinite horizon with a nonzero
  `h` warns once. `discount_rate` moves from `CostFunction` to `PlanningProblem` (one ledger
  line; RL's γ and DP read it there). No scoring modes. Lives in: end-state-api.md D16, D20;
  differentiable-problem.md §4.3; cost-params.md.
- **R10 (D9, D17) One evaluator family; the ODE solver is the judge.** `evaluate(problem,
  controller, *, n_trials=, key=)` is the one verb (deterministic: one run; Monte Carlo: the
  draws), `Evaluation` its record, `compare` a plot of Evaluations; `MonteCarloEvaluator` folds
  into it. The adaptive ODE solver produces every reported trajectory; the fixed-step compiled
  run (`problem.compile`, `ProblemEvaluator`) is the engine under `tune` and the researcher's
  `jax.grad`, never the reported number. Continuous controllers first: a sampled controller is
  refused in v0.3 and runs in v0.4 with the sampled loop; no sampled replay before then.
  `infeasible_cost` on the problem is the one price of an exit (DP, RL, the evaluator), its
  default `cost.g_max(X, U) · tf` (or `/ ρ`); no "infinity" on the cost. Lives in:
  end-state-api.md D9, D16, D17; differentiable-problem.md §4.1, §4.5 to amend at step 0.
- **R11 (D10b) The compiled problem is the traceable twin of `evaluate`.** One body with the
  run as its one varying piece: `closed_loop_of(problem, controller)`, `problem.realize(key,
  n)`, `evaluate_trajectory`, `feasibility_margin` and `Evaluation` are each one function;
  `evaluate` runs the `Simulator`, `problem.compile(controller)` runs the compiled fixed-step
  scan, and its verbs are thin calls into the same functions plus the program bridge. A
  pinned test keeps the two runs at round-off on one grid. A built loop is accepted when it
  carries the role `plant` (that block the problem's system); otherwise the argument is a
  controller, wired by the problem; roles are never inferred; the cost is scored on the
  `plant` role; `tune` names leaves by the diagram's params paths. Lives in: end-state-api.md
  D10b; differentiable-problem.md §4.1, §4.5 to rewrite at step 0.
- **R12 (D10) One body per integrator; the frozen name is a binding (B).** The traced
  parametric body is the only code; `ev.jit.*` and `ev.frozen.*` are generated and cached once,
  never rebuilt in a loop; the four-body surface of sampled-path.md §3.6 follows (91 names to
  about 12); the stable names become aliases for one release. Lives in: sampled-path.md §3.5,
  §3.6; end-state-api.md D10; TODO §4 S75 (SM-1, SM-6).
- **D10's measurement.** `benchmarks/params_binding.py` (2026-10-22, CPU): inside a
  scan or `vmap` the frozen and `params` forms are equal; from Python the dict costs about
  3 µs of dispatch per call and the bound form equals the frozen call; only the `params` form
  differentiates (2.2× the value). The table is in sampled-path.md §3.5. Session 1 stopped
  here; D10 is the first item of the next session.

## Where session 1 stands

Ruled: D1–D10, D10b, D16, D17, D20 (R1–R12). Session 1 is closed. Open: session 2 (D11–D15,
D18, D19, D21–D24) and the v0.4 names (D25–D30).

## Addendum, the same evening: the evaluator's surface and the speed trade-offs (discussion)

Measured and discussed, not ruled (they shape D10): no real speed difference between the
frozen and the `params` step once the frozen name is a binding; the traced body is the
contract and the jitted and bound forms are generated and cached once (re-jitting per call
costs 5×, re-binding per call 5000×); the evaluator's 91 method names across the two backends
reduce to four traced bodies (`f`, `h`, `step`, `rollout`) plus `rollout_batch`, with `step`
the same word for a continuous system's integrator step and a discrete system's own map;
batches map any leaf with a leading `B` axis and share the rest; a parameter family costs up
to 5× on a memory-bound plant and nothing on a compute-bound one, the S53 row's subject. All
of it in sampled-path.md §3.6; the numbers from `benchmarks/params_binding.py`.

## Session 2 (started the same day; the maintainer's order: rule the whole plan now, then
## implement the continuous-time core first, the sampled loop's code after)

- **R13 (D11) The sampled loop is a tool; `HybridDiagram` keeps its name for now.** One
  kernel, a scan over clock ticks on the step model with the computer's `step` between samples,
  NumPy and JAX faces, tracing decided by the computer (MPC runs the NumPy face). Consumers:
  `HybridSimulator`, the MPC export and replay, later the sampled face of `evaluate` / `tune`
  (v0.4). The RL environment depends on the step model only, not on the loop. Lives in:
  end-state-api.md D11; sampled-path.md §8.
- **R14 (D12) Ticks.** `step(x, u, k, params)` keeps the integer tick; the loop converts to
  seconds where it calls the plant. **D13–D15 parked** until the continuous architecture and
  API are done and implemented (the hybrid tool's details come in that later phase).
- **R15 (D9, D17 revised, 2026-10-23) One evaluator, one engine.** The fixed-step compiled
  rollout is the judge and the engine: `problem.compile(controller, dt)` is the one
  implementation (`trajectory`, `cost`, `margin`, `value`, `evaluate`, `program`), and
  `evaluate(problem, controller, *, dt=None, method="rk4", n_trials=1, key=0)` its facade;
  `method="scipy"` swaps the run for the adaptive `Simulator` and nothing else; `dt=None` is
  the Simulator's automatic rule. This supersedes R10's "the ODE solver is the judge"; the
  rest of R10 stands. Lives in: end-state-api.md D9, D17.
- **R16 `compare` (2026-10-23).** `compare(problem, **entries, dt=, n_trials=, key=)` wraps
  `evaluate` on several controllers (or solutions, or evaluations) with one key, returns a
  `Comparison` of `Evaluation`s with the table and the plot verbs (trajectory, cost, phase
  plane, control law, cost-to-go, paired difference). Lives in: end-state-api.md D9.
- **R17 (D9b, 2026-10-23) `compute_forced` on the same engine.** Keeps its name and argument
  kinds, gains `method="rk4" | "scipy"`; the fixed-step backends call the one `rollout` with
  the step model's `hold`; the `euler` loop backend goes; SciPy keeps its interpolant;
  `input_interp` becomes `hold`. Open: the `rk4_fixedsteps` forced default, `foh`
  (byte-identical) or `zoh`. Lives in: end-state-api.md D9b; sampled-path.md §7 (S65).
- **R18 (2026-10-23) The forced hold is `zoh` everywhere.** `rk4_fixedsteps` forced runs stop
  interpolating the input linearly; their numbers change once (baseline, ledger line). The
  knobs `method`, `substeps`, `hold` are static Python values, never traced (sampled-path §3.6).
- **R19 (D16 revised, 2026-10-23) One evaluation of a run: no event, no mask, no stop.** Every
  run is integrated to `tf`; `evaluate_trajectory` returns `TrajectoryEvaluation(cost, value,
  feasibility_margin, failed, reachability_margin, reached)` with `cost` the plain discounted
  integral plus `e^{−ρ tf} h(x(tf))`, `failed` from the minimum margin to `X` and `U` over the
  run, `reached` from `Xf.margin(x(tf))` (`Xf=None` means reached), and `value = cost +
  infeasible_cost · (failed or not reached)`. Minimum time is `TimeCost`. A diverging failed run
  is handled later by a saturating cost wrapper, not a mask. The discount applies to `g` and `h`
  everywhere; `x_goal` and `cost.xbar` are reconciled at construction. This replaces R9's "cut
  at the first exit". Lives in: end-state-api.md D16; the field inventory
  (docs/reviews/2026-10-23-planning-problem-field-inventory.md).
- **R20 (D16b, 2026-10-23) The cost object.** `params`, `g`, `h`, `g_max(X, U)` (the scaling
  hint for `infeasible_cost`), `integrate(traj, *, discount_rate, params)` (the one quadrature,
  the running integral without `h`), and the algebra; `evaluate_trajectory(problem, traj)` reads
  `integrate`'s last value plus the discounted `h`; a plotting wrapper attaches the `cost_rate`
  and `cost` signals. `discount_rate`, `evaluate_trajectory`, `terminal_cost`, `total_cost` leave
  the cost. Lives in: end-state-api.md D16b; cost-params.md to amend at step 0.
- **Time-dependent sets (2026-10-23).** `X(t)` and `U(x, t)` are the contract and nearly the
  practice; the one gap is RRT, which checks `X` and the goal without `t` (a hardening row for
  D3 in TODO §3). `evaluate_trajectory` reads the margins at every sample's time.
- **R20 amended (2026-10-23):** the cost signals are attachable from a cost alone,
  `traj.with_cost(cost, discount_rate=0.0)`, no problem object needed; the problem form reads
  its cost and discount; one body, `cost.integrate`.
  The problem form `traj.with_cost(problem)` is dropped (2026-10-23): one signature, a cost
  and an optional discount rate.
- **R21 (D31a, D31b, 2026-10-23) The sets are explicit.** `X` and `U` default to unbounded; DP
  and RL refuse an unbounded one with the line to write (`X=plant.state.box`,
  `U=plant.input_box`); RL's action space and policy scaling read `U`'s box; LQR ignores `U`
  and warns once when it is declared; the lessons that relied on the silent fallback gain the
  explicit line in the same change. Lives in: end-state-api.md D16.
- **R22 (D16c, 2026-10-23) The loop the pipeline reads.** A controller (the loop built by
  `closed_loop` under the automation-by-convention rules, the reference from the problem, no
  command-port argument, ambiguity refused) or a built loop (a diagram whose `plant` key is
  the problem's system or a box containing it; roles never inferred). `pe.params` is the
  loop's params plus the problem's leaves; `wrt` names leaves by path. Lives in:
  end-state-api.md D16c.
- **R23 (2026-10-23) This phase exports the program only.** `pe.program(wrt, bounds)` returns a
  `MathematicalProgram` whose objective is the problem's value; the user runs `Optimizer` and
  `pe.unpack(z, wrt)` writes the leaves back. No `tune` / `optimize` facade, no
  `OptimizationSolution`, no `PlanningSolution` from it until a lesson needs one. Lives in:
  end-state-api.md D16c; differentiable-problem.md §4.6 and step 5 to amend at step 0.
- **R24 (2026-10-23) The criterion lives on the problem.** `criterion` in `"expectation"`,
  `("cvar", α)`, `"worst_case"`, read by `value`; `Evaluation.value(criterion)` loses its
  argument; the report keeps the statistics (`mean`, `worst`, `cvar(α)`) free. Lives in:
  end-state-api.md D18.
- **R25 (D23, 2026-10-23) The optimizer upgrade is deferred.** Program classes hinting the
  solver, `auto` selection, algorithm-named presets and the first-order backend (S74) come
  after the continuous core; this phase exports the program to today's `Optimizer`.
- **R26 (D19, 2026-10-23) The names.** `problem.compile -> ProblemEvaluator`; its verbs all
  take `(params=None, key=...)` first: `trajectory`, `evaluate_trajectory` (one run's record;
  the verb formerly `cost`), `value`, `evaluate` (the report), `program`, `pack` / `unpack`;
  `wrt=`; the key `plant`; `TrajectoryEvaluation` and `Evaluation` the two records. Lives in:
  end-state-api.md D19.
- **R27 (D18, D19, D22, 2026-10-23) One record, two verbs.** `Evaluation` is the one pytree
  record (one row per run; `n_trials = 1` for one run) with `.value` the criterion scalar the
  program minimizes, the statistics on `xp`, and three rates: `failure_rate` (left `X` or `U`),
  `reachability_rate` (ended in `Xf`), `success_rate` (reached and never failed). The engine has
  `trajectory`, `evaluate`, `program` (plus `pack` / `unpack`), each named by what it returns,
  each taking `(params=None, key=...)` first. `evaluate_trajectory(problem, traj)` returns a
  one-row `Evaluation`. `TrajectoryEvaluation`, `mean_cost`, `final_state` and the engine verbs
  `cost`, `value`, `margin` do not exist. This supersedes R26's verb list. Lives in:
  end-state-api.md D18, D19, D22.
  Amended the same day: the three rates are `feasibility_rate` (never left `X` or `U`),
  `reachability_rate`, `success_rate`; no `failure_rate` (it would read as `1 − success_rate`).
- **R28 (D21, D24, 2026-10-23) Confirmed.** The evaluator's numbers change once (the real loop,
  the integral to `tf`), re-pinned with a note. The lane: `evaluate`, `compare`,
  `evaluate_trajectory`, `discretize` teaching surface; `problem.compile`, `ProblemEvaluator`,
  `program` provisional on the planning facade.

## Where the plan stands after 2026-10-23

Ruled for this phase (the continuous core): D1–D12, D16 (with 16b, 16c), D17–D22, D24, D31
(R1–R28). Deferred: D23 (the optimizer upgrade, S74) to after the continuous core; D13–D15
(the hybrid tool's details) to the hybrid phase; D25–D30 (the parametric program's details,
MPPI's name and contract, the seeded stochastic policy) to v0.4. Next: fold the rulings into
the plan docs (differentiable-problem.md, sampled-path.md, cost-params.md, randomness.md) and
turn them into workboard rows, then implement.
- **Consistency pass (2026-10-23).** The sketch (end-state-api.md) and the diagram were
  re-read against R1–R28: `tune`, the engine verbs `cost` / `margin` / `value`,
  `TrajectoryEvaluation`, `SampledLoop`, a separate `feasibility_margin` function and the
  "cut at exit" wording in D20 were leftovers and are gone. The per-run `value` is an `(n,)`
  array; the criterion over the runs is the one scalar **`criterion_value`** (proposed name,
  the maintainer to confirm), the objective of the program and the target of `jax.grad`.
- **R29 (2026-10-23) Three corrections.** The problem never reads the cost's target (`xbar` is
  not a generic attribute): `x_goal` is a point of `Xf` for the tools that need one, and a loop
  whose controller reads `r` is refused when `x_goal` is `None`. `Xf=None` means no terminal
  constraint (`reached` by definition, `h` the soft end), not "regulation". Three trajectories
  named once: the loop's run (what the engine and the Simulator return, signals recorded, what
  the plots show), the plant's run (`traj.of("plant")`, what the sets and the cost are written
  over), the record (`evaluate_trajectory` scores the plant's run, projecting a loop trajectory
  by its `plant` role; a plant-dimension trajectory without roles is taken as the plant's).
  Lives in: continuous-core.md §3.4–§3.6, §4.8; end-state-api.md D16, D16c.
- **Checked (2026-10-23): `traj.of("plant")` traces once signals are recorded.** State a static
  slice; each plant input a static index into the recorded signals, the boundary `u` or a
  nominal constant; no recomputation. Today's recompute loop is the NumPy fallback for a
  signal-less trajectory. Lives in: continuous-core.md §3.5.

## Handoff (2026-10-23, the session closes)

The design of record is [continuous-core.md](../plans/continuous-core.md); the decision ledger
[end-state-api.md](../plans/end-state-api.md); the picture `docs/plans/pipeline.mmd`; the
numbers `benchmarks/params_binding.py`; the audit
[2026-10-23-planning-problem-field-inventory.md](2026-10-23-planning-problem-field-inventory.md).
ROADMAP §5.3 and TODO §4 point here. Next, in order: (1) rewrite differentiable-problem.md,
sampled-path.md, cost-params.md and randomness.md's evaluator step to the design; (2) the
design as workboard rows with gates; (3) implement, the step model's baseline (SM-0) first.
Open by the maintainer's choice: the hybrid tool's details (D13–D15), the optimizer upgrade
(D23, S74), the v0.4 names (D25–D30).

## Session 3 (2026-10-23, evening): the second opinion

Taken one by one over the points the alignment pass and the independent second opinion raised
([the record](2026-10-23-v03-alignment-and-second-opinion.md) §3, whose item numbers are cited).
Where a ruling here differs from an earlier one, it supersedes it; the superseded ones are
named.

- **R30 (item 6) The GMC714 audit now.** The agent runs G1 this week on the teaching surface
  and writes "what each GMC714 lesson needs by January 5"; the course page
  (alexandregirard.com/courses/gmc714) is the syllabus, to be read once the environment allows
  the host or the maintainer pastes it.
- **R31 (item 1) A run that is not finite is failed.** `failed = ~finite | (feasibility_margin
  < 0)`; its raw `cost` is `+inf`, the gradient kept finite by a safe `where`; its `J` is the
  price when the price is on (R32).
- **R32 (items 2, 4; the price, one rule for every pipeline).** The cost carries its
  upper-bound hint, `CostFunction.g_max(X, U)` and `h_max(X)`, always defined;
  `problem.infeasible_cost_bound()` resolves it to a finite number: `g_max(X_b, U_b) · tf +
  h_max(X_b)` (`/ ρ` on an infinite horizon), `X_b`, `U_b` the declared boxes, else the plant's
  own, else `1e6` with one warning (the name to confirm). `infeasible_cost` is the switch:
  `None` (the default), `True` (the bound) or a number. Then, read the same way by `evaluate`,
  `compare`, `jax.grad` and `pe.program`: `J = cost` when off, `J = cost + price · (failed or not
  reached)` when on; `objective` is the criterion over `J`. `pe.program` adds the declared `X`,
  `U`, `Xf` as hard margin rows beside it; its docstring states the two caveats (the price is a
  step with no gradient; with the switch on and sets declared it counts beside the rows) and
  the smooth alternative, `cost + w · X.as_cost()`. DP, RL and later MPPI read `infeasible_cost`
  when set, else `infeasible_cost_bound()`; nothing refuses. Supersedes R10's default
  (`g_max · tf`) and R19's always-priced `value`.
- **R33 (item 3) `x_goal` never creates `Xf`.** A terminal set is written (`Xf=BallSet(x_goal,
  r)`); trajectory optimization's terminal equality needs `Xf=SingletonSet(...)` explicitly (its
  demos and lessons gain the line; a ledger line).
- **R34 (the criterion)** keeps its three values, `"expectation"`, `("cvar", α)`,
  `"worst_case"`, on `StochasticPlanningProblem`, reducing `J`; the mean over successful runs,
  the plain cost's mean and the rates are report fields, never criteria.
- **R35 (the settings map).** The problem says *what* is optimal (the sets, the horizon, the
  discount, the price switch, the laws, the criterion); the engine says *how a run is computed*
  (`dt`, `substeps`, `solver`, `backend`); the program says *how it is lowered* (`wrt`, `bounds`,
  the sample set).
- **R36 (item 5) `dt` is required** on `evaluate` and `compare`, as on `problem.compile`; with an
  adaptive `solver` it is the reporting grid. Supersedes R15's `dt=None`.
- **R37 (item 11) `U` unbounded unless declared** (R21 kept); `plant.input_box`, the input
  ports' `BoxInputSet`, is added so the refusal's line works; the trajectory-optimization
  numbers are re-pinned once (a ledger line); the lessons gain `U=plant.input_box`.
- **R38 (item 13) The problem never fills `r`.** Every free input of the loop stays at its
  port's nominal value; the setpoint is set on the controller (`inputs["r"].set_nominal_value`)
  or by a source block; the free inputs stay leaves of `pe.params`. `x_goal` serves RRT's bias and
  trajectory optimization's initial guess only. Supersedes R22's reference filling and R29's
  refusal.
- **R39 (item 14) A role-less trajectory** is taken as the plant's run only when `x` has the
  plant's `n`, `u` its `m`, and it carries no recorded signals; a loop trajectory carries its
  roles (a static layout, kept by save and load) and is projected; signals without roles are
  refused. Supersedes R29's dimension rule.
- **R40 (item 12) A step map holds its input by definition.** `step` and `discretize` have no
  `hold` (their knobs: `dt`, `solver`, `substeps`); `hold="zoh" | "foh"` lives on `rollout` and
  `compute_forced`, which hold a sequence; a callable `u(t)` given to `compute_forced` is
  evaluated at the RK4 stage times; a sampled matrix is held (`zoh` by default, R18 for sampled
  inputs only). Supersedes R7's `hold` on the step and R18 for callables.
- **R41 (item 15) One home per number.** Each object owns its parameters (`plant.params`,
  `cost.params`, a set's); `ProblemParameters` is retired, its readers migrating with A2;
  `pe.params` is the tree assembled from the objects, plus `x0` and the free inputs (`"sets"` once
  A5 lands); `pe.evaluate(p)` takes an override tree.
- **R42 (item 16) One keyword per kind of choice.** `method=` is the verb's own numerical method
  (the derivative, the optimizer's algorithm, the interpolation, the certificate family);
  `solver=` is the time integration on every tool that integrates (`Simulator`,
  `compute_trajectory`, `compute_forced`, `evaluate`, `compare`, `discretize`, the RL planners,
  `RolloutEnvironment`, `Sys2Gym`); `integrator=` is retired (one release accepted, a ledger line;
  it clashes with the `Integrator` block). Values: `"rk4"`, `"euler"`, `"exact"` for a step,
  `"rk4"` accepted for `"rk4_fixedsteps"`, the `"scipy*"` family adaptive. RULES gains the rule.
  Supersedes R6's `integrator=` → `method=` and the `method=` of R15 and R17.
- **R43 (item 17) The RL episode is a training setting.** `RolloutEnvironment` owns it and reads
  the problem's fields as hints (`X` the exit, `infeasible_cost` else the bound its price,
  `discount_rate` its γ, `tf` or `episode_length`, `U` the action box), keeping its own rules
  (termination at an exit, truncation at the training zone, the left-Riemann reward); the learned
  law is scored by `evaluate` like any other. `RolloutEnvironment` stops serving the Monte Carlo
  trials.
- **R44 (items 10, 20) The Monte Carlo fold is v0.3,** right after `evaluate` exists (V1's step
  4): `evaluate` and `compare` take `n_trials` and `key` over `problem.realize(key, n)` (the start
  and parameter laws; RN-4's per-block noise seeds join in v0.4 under the same verb);
  `MonteCarloEvaluator` becomes a one-line facade, then goes; the RL lessons and tutorial 11
  migrate once. Supersedes the v0.4 slot of V1's step 7 for its fold.
- **R45 (item 19) The record's names: `cost`, `J`, `objective`.** `cost` the plain integral per
  run, `J` the run's score (the price added when on), `objective` the criterion over `J`, the one
  scalar the program minimizes and `jax.grad` targets. Supersedes R27's `value` and the proposed
  `criterion_value` (no clash with `V(x)`, `critic.value`, `Field.value`).
- **R46 (item 22) No hook for a term on the parameters.** The autotuning demo drops its `1e-4`
  gain regularizer (its numbers re-pinned once) and ends in the `Optimizer`; a `jax.grad` user adds
  any term to `pe.evaluate(p).objective`.
- **R47 (item 7) No pin; v0.3 stays on `dev`** until every notebook has moved to the new
  interface; `main` stays what the running course clones.

Left open from the record: items 8, 9 and 18 (v0.3's scope and the slots of S31 and S74, after
G1), 21 (`discretize`'s `params["dt"]` fallback, proposed dropped), 23 and 24 (agent-managed: V1.1
split when it starts; this section marks the reversals).
- **R48 (2026-10-23, after G1) GMC714 needs no new feature.** The course page and its notes
  were read (8 homework labs: the bicycle's phase plane, arm kinematics and dynamics, impedance
  on a 3-DoF arm, computed torque, sliding mode and adaptive control, the longitudinal and
  dynamic bicycle models, trajectory optimization and RRT); every lab's ingredients are on the
  teaching surface (ROADMAP §4.3). The labs move from pyro to minilink as the maintainer's
  teaching work, outside the roadmap; the adaptive law is a custom `DynamicSystem` written in
  the lab (pyro's `DynamicController`). MPC, the draws and robust control as a topic are not in
  the syllabus, which answers the second opinion's item 9. The former wave G rows (C1, C2, C3,
  S63, S64) move to v0.4. This closes G1 (R30).
- **R49 (2026-10-23) GMC714's names stable by January.** The tools the labs use carry a public,
  stable teaching name on the teaching surface before the term (week 1 on January 9): RRT,
  MPC and trajectory optimization; `ComputedTorqueController`, `SlidingModeController`; the
  impedance laws in task and joint space (`ImpedanceController`, `TaskImpedance`,
  `JointImpedance`, `ModelJointImpedance`); task-space position control with a nullspace
  posture (`TaskKinematicNullspace`); the `Manipulator` and `MechanicalSystem` classes. Checked
  the same day: `RRTPlanner` is on the surface but provisional; `ModelJointImpedance`,
  `TaskKinematic`, `TaskKinematicNullspace` are on the `control` facade only; `Manipulator` and
  `MechanicalSystem` are a deep import; MPC is two research-lane classes
  (`MPCStatelessController`, `MPCStatefulController`). A v0.3 row (TODO §4, G3) carries it.
- **R50 (2026-10-23) G3's three choices.** (1) MPC's teaching name is `ModelPredictiveController`,
  over today's `MPCStatelessController` / `MPCStatefulController`, its constructor frozen for the
  term. (2) `Manipulator` and `MechanicalSystem` go on a new band facade, `minilink.dynamics`
  (the package exists; its `__init__` becomes the facade of the modelling abstractions, beside
  `minilink.catalog` for the plants), registered with the other bands in the teaching-surface
  test; its exact list is ruled with G3's table. (3) A6's short catalog names land **before** the
  GMC714 term (by January 9), with every course notebook migrated in the same commit while the
  name freeze is paused; v0.3 stays on `dev` until then (R47).
- **R51 (2026-10-23, K9) The two notebooks on the experimental tier.**
  `examples/teaching/topics/robotics/manipulator_eom.ipynb` is deleted (it imported the
  symbolic mechanics); `examples/tutorial/showcase_jax.ipynb` keeps its quick C-export demo, the
  second documented gate exception beside the C-export flagship smoke.
- **R52 (2026-10-23, A6) The short names: drop every display name, rename seventeen classes.**
  The 68 explicit display names go, so the class name shows. Seventeen classes are renamed
  (the K1 inventory's §4.1 options A and B, as drafted): `LongitudinalTorqueCar`,
  `LongitudinalSlipCar`, `ThreeLinkManipulator`, `FiveLinkManipulator`,
  `KinematicManipulator` (was `SpeedControlledManipulator`), `QuarterCar`, `NoisyCartPole`,
  `NoisyPendulum`, `TimeVaryingFeedback`, `IntegralImpedance`, `ComputedTorque`, `SlidingMode`,
  `Impedance`, `StateFeedback`, `OutputReference`, `TrajectoryFeedback`, `LookupPolicy`. `UdeS`
  is an acronym (Université de Sherbrooke) and stays whole: `UdeSRacecar`, `UdeSRacecarDyn`,
  `UdeSRacecarDyn3D` are unchanged. Breaking names is accepted: every notebook, demo and test
  migrates in the same commit, no alias. The GMC714 names G3 freezes are these new ones.
- **R53 (2026-10-23) A block's id defaults to its class name, verbatim.** No snake-case
  conversion (`Pendulum`, `Pendulum2`, `Drone2D`, `UdeSRacecar`, `ComputedTorque`), so digits
  and acronyms are never mangled. An id the user gives (`add_subsystem(sys, "arm")`,
  `sys.id = …`) still wins, and a loop the library builds (`@`, `closed_loop`) still keys its
  blocks by role (`plant`, `controller`, `observer`), so loop code survives swapping the
  controller and the tools find the plant by its key (R22).
- **R54 (2026-10-23, K2) The tools with no caller are deleted; the demos are the maintainer's pass.**
  Deleted, one commit per feature, callers, tests and docs with them, no alias: `ProblemEnv` and
  `to_gymnasium`, `fixed_camera` and `follow_frame_camera` (`camera.py`), `merge_skins`,
  `debug_state_skin`, `Replay`, `Sampler`, `RateLimiter`, the `SimpleIntegrator` plant (the
  `Integrator` block covers it); `NeuralNetwork` folds into `MLP`. `Lag` stays and gains its demo
  beside `Lead` (classical control). `ProblemParameters` is not deleted here: the planners read it
  internally, so it goes with its migration (R41, V1.2 / A2). The demo removals and merges of the
  K1 inventory (§3) are not ruled: the maintainer runs his own pass on the demos (K4).
- **R54 amended (2026-10-23): `Lag` is deleted too; `Lead` stays** (used by the frequency-response
  topic notebook, the root-locus demo and the tutorial's showcase).
- **R55 (2026-10-23, K8) Three cars on the teaching surface.** The car family has three distinct
  models: the kinematic bicycle, the dynamic bicycle with tires, and the real 1/10 racecar. The
  root prelude carries those three: `KinematicBicycle`, `DynamicBicycle`, `UdeSRacecarDyn`. The
  variants fold in: `KinematicCar` and `DynamicBicycleCar3D` merge away (a drawing is the plant's
  `skin`, set on the instance; the car's size is the bicycle's parameters); `UdeSRacecar` becomes a
  parameter preset of `KinematicBicycle` (`KinematicBicycle(**UDES_RACECAR)`); `UdeSRacecarDyn3D`
  folds into `UdeSRacecarDyn(wheel_angles=True)` (the two wheel angles exist only for the 3-D
  animation). `BicycleDynRate` (the MPC reformulation), the two longitudinal cars and
  `HolonomicMobileRobot` stay on `minilink.catalog`. The tutorial, the GRO501 `racecar_toward_mpc`
  notebook and the racecar demos migrate in the same commit.
- **R56 (2026-10-23, K8) Three arms on the root.** `TwoLinkManipulator` (the planar textbook arm),
  `KinematicManipulator` (joint-speed inputs, kinematic control; was `SpeedControlledManipulator`)
  and `UR5Manipulator` (the real robot). `ThreeLinkManipulator` (the GMC714 drilling lab) and
  `FiveLinkManipulator` (the nullspace demo) stay on `minilink.catalog`; `OneLinkManipulator` goes
  to the experimental tier (`Pendulum` teaches one link). The root then holds 14 plants: `Pendulum`,
  `InvertedPendulum`, `NoisyPendulum`, `CartPole`, `NoisyCartPole`, `DoubleIntegrator`, `SingleMass`,
  `Drone2D`, the three cars of R55, the three arms.
- **R57 (2026-10-23, K8) The pendulum and cart-pole variants stay for now.** `Pendulum`,
  `InvertedPendulum`, `NoisyPendulum`, `CartPole`, `NoisyCartPole` stay on the root as five classes
  (no `noise_ports=` option: it would put two input layouts in one `f`). A plant that declares its
  own `d` and `n` is a use case the library must show, but today's two noisy variants add the
  disturbance at the actuator, which `closed_loop(disturbance=, noise=)` already does. Once a better
  example exists (an unmatched disturbance the loop cannot add, e.g. a push on the cart-pole's tip
  or a lateral wind on `Drone2D`, with noise on some sensors only), the two variants are revisited:
  one becomes that example, the other folds into `closed_loop`. A Later row on the workboard.
- **R58 (2026-10-23, K8) The classical examples stay in the catalog; the research tests go.**
  On `minilink.catalog` (supported, tested, documented): `MountainCar`, `Acrobot`, the two Furuta
  pendulums (`RotatingCartPole`, `UnderactuatedRotatingCartPole`), `Plane2D`, `Boat2D` and
  `Boat2DWithCurrent`, `ThreeBodyProblem`, `Lorenz`, `VanderPol`. To the experimental tier (research
  tests): `TwoIndependentPendulums`, `ConstantSpeedHelicopterTunnel`, `Drone2DWithSideThruster`,
  `Plane3D` with its 3-D skin and airframe (`OneLinkManipulator` already, R56).
- **R59 (2026-10-23, K8) The mass family on the root; the rest of the plants placed.** `SingleMass`,
  `TwoMass`, `ThreeMass` are on the root prelude (many lessons use one, two and three masses); the
  floating variants merge into them (`FloatingSingleMass` is `SingleMass` with `k1 = 0`, and so on:
  one class per chain, the floating case a parameter value). `TripleIntegrator` and `QuarterCar` stay
  on `minilink.catalog`. `SpeedControlledDrone2D` is deleted (`HolonomicMobileRobot` has the same
  `ẋ = u`). The root then holds 16 plants, one over R2's cap of 15 (to settle, R59's note below).
- **R60 (2026-10-23) The root's plants have no cap.** The rule is the course set: the root prelude
  holds exactly the plants a course notebook, the tutorial or a ruling (R55–R59) puts there, pinned
  by the teaching-surface test; R2's cap of 15 goes. Today: 16 plants.
- **R61 (2026-10-23, G3) The `minilink.dynamics` facade, and one mechanical base.** It exports the
  textbook model forms a student subclasses: `MechanicalSystem`, `Manipulator`,
  `GeneralizedMechanicalSystem`, `StateSpaceSystem`, `LTISystem`. `JaxMechanicalSystem` is deleted:
  `MechanicalSystem`'s default matrix builders are written once with the `xp` idiom (as every catalog
  plant's are), the experimental symbolic export reads `MechanicalSystem`, and its one test moves
  with it (CONSTITUTION §6: one equation path for NumPy and JAX).
- **R62 (2026-10-23) One mechanical family, one vocabulary.** `GeneralizedMechanicalSystem` becomes a
  child of `MechanicalSystem`, beside `Manipulator`: the base keeps the textbook manipulator equation
  `H(q) q̈ + C(q, q̇) q̇ + d + g(q) = B(q) u`; the child adds `N(q)` (`q̇ = N(q) v`, body-frame
  velocities) and the state split when `q` and `v` differ in size. The mass matrix is `H` everywhere,
  as in the course notes (ch. 6, eq. 6.14; ch. 11, eqs. 11.5 and 11.8, where `M_b` is a body's own
  inertia): `GeneralizedMechanicalSystem.M`, the boat's and the plane's become `H`, the docstring
  noting "often written `M` (Craig, Lynch & Park)"; `qv2x` / `x2qv` align with `q2x` / `x2q`. The
  model-based controllers (`ComputedTorque`, `SlidingMode`, the joint impedance laws) read one set of
  matrices and accept every mechanical plant, working in `v`. Both forms stay for teaching; S32's
  single class (`N = I` by default) is not adopted.
- **R63 (2026-10-23, G3) Joint and task impedance are two plain classes.** `ModelJointImpedance` is
  renamed `JointImpedance` (the class: the law on a mechanical plant, optional `gravity_comp`), and the
  `JointImpedance(...)` factory that returned either it or a generic `ImpedanceController` is deleted;
  the generic spring-damper is `Impedance(dof=…)` (R52). `JointImpedance` and `TaskImpedance` are the
  pair on the root; the one `JointImpedance(dof=2)` call (a test) becomes `Impedance(dof=2)`.
- **R64 (2026-10-23, K8-1 / K2) The root's other names, and the basic blocks.** Off the root prelude,
  kept on their band facade (the silent trim of R5): `Error`, `Mux`, `Source`, `MLP`, `UnitDelay`,
  `TimeVaryingFeedback`, `TrajectoryFeedback`, `modal_analysis` (the method stays),
  `StaticSimulator`, `LinearApproximator`, `QuadraticFeatures`, `RadialBasisFeatures`, `LookupPolicy`,
  `RRTStarPlanner`. Kept on the root as course-relevant: `LowPassFilter`, `PurePursuit`. The block
  library keeps the basic blocks only, a special block existing when a lesson needs it: `DeadZone`,
  `Relay`, `NotchFilter` and `Washout` are deleted (their one user is the `signal_blocks.py`
  showcase); `ZOHHold`, a discrete unit delay `x_{k+1} = u_k` and not the continuous zero-order hold
  (that is the sampled loop's and the sequences' `hold`, R40), is renamed `UnitDelay`.
