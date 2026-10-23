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
