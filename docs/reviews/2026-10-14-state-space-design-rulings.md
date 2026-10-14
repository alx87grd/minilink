# State feedback and state estimation as one pattern: the rulings (2026-10-13 and 2026-10-14)

**Date:** 2026-10-13 (design conversation) and 2026-10-14 (the first landings)
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Step P4 (`estimation/`) was about to start on top of `control/lqr.py` as it stood.
The maintainer asked for a fresh design of state feedback and state estimation on the core
alone, against what pyro 2 did well (`StateObserver` + a `kalman()` design; designs made on a
state-space object), and for the long view: the estimation problem and its tools, and where
an extended Kalman filter along a trajectory fits. The design is
[the P4 plan](../plans/estimation-and-state-feedback.md); this file is the record of what
was ruled, one entry each, and where each ruling now lives.

---

## Rulings

- **R1 The nominal state is the operating point; `x0` is the start of a simulation.**
  `VectorSignal.nominal_value` was documented as "the default or operating point value" and
  the input ports' nominal values already served as ū, yet `operating_point()` read `x0` for
  x̄, so a plant whose `x0` was a disturbed start (the catalog pendulum at 0.7 rad, the
  cart-pole at `[1, 0.25, 0, 0]`) was linearized away from its equilibrium. Ruling:
  `state.nominal_value` is x̄, pyro's `xbar`; every analysis and design verb defaults to it
  and keeps its `x_bar=` / `u_bar=` keywords; `x0` keeps one meaning. No warning when the
  point is omitted: the default is a declared quantity. Lives in: `analysis/derivatives.py`
  (`operating_point`), DESIGN §4 (`System`, "Nominal and start"), TODO S29 (reworded: two
  quantities, one owner each; the scan's core#11 "fold `nominal_value` into `x0`" is not taken).
  Landed 2026-10-14.
- **R2 The linear model carries its operating point.** `LTISystem(..., xbar, ubar, ybar)`;
  `linearize()` fills them (the inputs of the selected `B` columns, the outputs of the selected
  `C` rows). A model from arrays sits at the origin. Lives in: `state_space.py`,
  `linearization.py`, DESIGN §4. Landed 2026-10-14.
- **R3 Designs take a model or a plant, MATLAB style, in two arities.** `lqr(A, B, Q, R)`
  on arrays; `lqr(sys, Q, R, *, x_bar=None, u_bar=None, t, params, method, eps)` on an
  `LTISystem` (as is, with its point; a point given here is refused) or on any plant,
  linearized first about its **command port alone** (RULES 4.3; a plant with `d` / `n` ports
  gives a gain over `u`). Same for `place`, `lqr_finite_horizon`, and the estimation verbs.
  The weights may be keywords. `lqr_at_operating_point` and `place_at_operating_point` leave
  with no alias (the GRO860 freeze is paused between terms; the notebooks that used them
  migrated in the same commit). Lives in: `control/design.py`, `analysis/linearization.py`
  (`design_model`), DESIGN §2 and §3. Landed 2026-10-14.
- **R4 One design module per band, no module/function shadow.** `control/design.py` folds
  `lqr.py` and `place.py`; the control facade exports `lqr` and `place` directly (S62's
  module part closes; `P` joining the siso family stays open). `estimation/design.py` will
  be its mirror. The DESIGN §2 exception "a module name may match a factory" is gone: a
  module never takes the name of a function it defines. Landed 2026-10-14.
- **R5 The Riccati equations have one home, `analysis/riccati.py`** (`care`, the exact
  Hamiltonian step), shared by control, estimation and `LQRPlanner`. The backward step on the
  dual pair `(Aᵀ, Cᵀ)` is the filter covariance stepped forward: no direction parameter is
  needed. Matrices tier, like `minreal`: not on the analysis facade. Dependency law: `control/`
  and `estimation/` import `analysis` (`design_model`, `riccati`, `structural`). Landed
  2026-10-14.
- **R6 The block is `StateObserver`; the designs are `luenberger` and `kalman`.** A block is
  named for what it is, a design verb for how it was obtained (pyro's lesson); both verbs
  return a `StateObserver`, named "Kalman Filter" or "Luenberger Observer" for `print` and
  `plot_diagram`. Innovation form, `f` written out, with the operating point on the block
  (`xbar`, `ubar`, `ybar`), as `StateFeedbackController` carries `xbar`, `ubar`. Ports `u`,
  `y` -> `x` (the estimate, declared through its own port function `estimate`, so a later
  filter whose state is larger than the estimate keeps the port); labels with a hat (`θ̂`),
  copied from the plant by the plant-taking verbs. P4.
- **R7 `Q_n`, `R_n` are the noise intensities everywhere in the estimation band**, gains
  included (`kalman_gain(A, C, Q_n, R_n)`), so `Q`, `R` keep one meaning, the LQR weights
  (RULES 5.4). `Q_n` is the n×n state-noise intensity; no pyro-style `G=` whose presence
  changes `Q`'s shape. The plant-taking form reads `B_w = ∂f/∂d`, `D_v = ∂h/∂n` from the
  ports when noise blocks are given (RN-4). P4 / RN-4.
- **R8 `kalman_gain` returns `L` only**, mirroring `lqr_gain -> K`; the error covariance is
  the evidence layer's, never the block's (pyro stored `obs.P`). No estimation planner now.
- **R9 The LQG compensator is a design verb, `lqg(model, Q, R, Q_n, R_n)`**, in
  `control/design.py`, returning the controller box with keys `observer` and `gain` built by
  `connect` (the `autowire` recipe warns about the open `y` and `r`). The ROADMAP §6 question
  "a `compensator(observer, K)` helper or the `expose` recipe" closes as **the recipe by hand,
  `lqg()` for the design**: no third wiring entry beside the operators and `closed_loop`.
  `closed_loop(observer=)` stays AC-4's. P4.
- **R10 An observer is not a controller.** A block that reads a measurement and commands
  nothing is refused as a controller by `@`, `closed_loop`, the sampled loop's boundary ports
  and `plot_control_law`, through one `require_command` in `core/feedback.py`; today all three
  readers fell back to the first output, the estimate, and would have wired it into the
  plant's `u`. P4, with the block.
- **R11 The role name is `observer`**, reversing the 2026-10-11 record's rejection
  ("`observer`: Luenberger only; a Kalman filter is an estimator"). In control texts an
  observer is any state reconstructor, Kalman's being the optimal one, and the Layer 3 tool
  family that mirrors `Planner` is naturally `Estimator` (`KalmanEstimator(problem)` beside
  `LQRPlanner(problem)`), which the role name would have collided with. Reserved ids are
  `plant`, `controller`, `observer`; `Roles.observer`; the planned keyword is
  `closed_loop(observer=)`. Renamed before any observer shipped: `core/feedback.py`,
  `composition.py`, `wiring.py`, the tests, DESIGN, ROADMAP, TODO and the automation plan.
  Landed 2026-10-14.
- **R12 The long view: `EstimationProblem`, `EstimationSolution`, the `*Estimator` tools.**
  The estimation problem shares the `StochasticPlanningProblem` spine (system, `disturbances`
  on `d` / `n`, the prior `x0_distribution`) and adds the measurement record and the measured
  ports; its solution mirrors `PlanningSolution` field for field, with the covariance as the
  dual of the cost-to-go. `LQGPlanner` (v0.3, with RN-4) reads the same problem. The
  linearized Kalman filter along a trajectory (`trajectory_kalman` -> `TrajectoryObserver`,
  the dual of `trajectory_lqr`) and the extended Kalman filter (`A(x̂)`, `C(x̂)` taken on the
  model's `f`, `h` inside its own `f`, `P` a state) have their slots named in the plan's
  stress test; the sampled filters wait for the sampled seam (AC-8a, S31) and `c2d` (P6).
  Not built; the plan is their design note.
- **R13 The GRO501 from-scratch notebooks are not migrated.** `numpy_state_space.ipynb` and
  `cartpole_dynamic_controller.ipynb` hand-write LQR, Kalman and LQG on purpose; the library's
  evidence is the demos (an LQR demo updated, a Kalman observer demo and an LQG demo added).
- **R14 The design verbs take a system; arrays go to the gains** (reverses R3's arrays
  form). `lqr(sys, Q, R, *, x_bar, u_bar, t, params, method, eps)`, and the same for
  `place`, `lqr_finite_horizon`, `lqg`, `kalman` and `luenberger`: `sys` is an `LTISystem`,
  used as is with its operating point, or any plant, linearized about its command port.
  The `*args` parser that read the arrays form leaves; on arrays, the gains `lqr_gain`,
  `place_gain`, `kalman_gain` and `luenberger_gain` join the root prelude, and
  `design_model` refuses a non-system argument by naming them. Course notebooks change only
  the lines that already call the minilink API (`03_control`, GRO860 `cartpole_lqr`, and one
  line of GRO501 `numpy_state_space`, an exception to R13); their from-scratch code stays.
  Lives in: `control/design.py`, `estimation/design.py`, `analysis/linearization.py`,
  DESIGN §2–§3.
- **R15 One law per state-feedback block** (reverses the P8 `N=` keyword of 2026-10-10).
  `StateFeedbackController(K, xbar, ubar)` is the state reference, `u = ū − K (x − r)` with
  `r` defaulting to x̄; `OutputReferenceController(K, N, xbar, ubar)` is the output
  reference, `u = ū − K (x − x̄) + N r`, with x̄ in `params` because its law reads it.
  Each block has one home for x̄ and no mode switch in its law; `lqr` and `place` still
  return the first. Lives in: `control/state.py`, DESIGN `state.py` row, ROADMAP §4.

## Where the earlier records and this one differ

The 2026-10-11 record and its 2026-10-13 second opinion are read-only history. Where they
say the role `estimator` or `closed_loop(estimator=)`, read `observer` (R11); where they
say `LuenbergerObserver` or `kalman(A, B, C, Q, R)`, read `StateObserver` and `kalman(…,
Q_n, R_n)` (R6, R7); where the gro501 plan says `place_at_operating_point`, read `place(sys,
poles, x_bar=)` (R3). The plan carries the live rules.
