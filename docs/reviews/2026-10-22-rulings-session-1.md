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
