# State feedback and state estimation as one pattern: the design for P4 and beyond

**Status:** decided 2026-10-13; P4 (the control sweep, the observer, the designs, `lqg`, the demos) landed 2026-10-14, and the review that followed made the verbs system-only, split the output reference out of `StateFeedbackController` and put `LQRPlanner` on the command port (rulings R14–R17), the open follow-ups are TODO rows (P4 follow-ups, `LQGPlanner`, the estimation follow-ups); §3, §5 and §6 are the live design for what comes next. The rulings and their reasons are in
[2026-10-14-state-space-design-rulings.md](../reviews/2026-10-14-state-space-design-rulings.md).
Workboard row P4 (v0.2 wave B). Items marked **[ruled]** were decided in the design
conversation; the role name was ruled `observer` (R11) and `analysis/riccati.py` taken (R5).

## 1. Vision

Minilink has one object, the `System`, and one algebra to wire systems by name. Control
design and state estimation are the two halves of the textbook's state-space chapter, and
they are mathematical duals. The library should show that duality rather than hide it: the
same layering, the same verbs, the same signatures with the pair `(A, B)` on one side and
`(A, C)` on the other, the same way of reading an operating point, the same evidence objects.
A student who has learned `lqr` knows `kalman`; a student who has built `ctl @ plant` can
read `lqg(plant, …) @ plant`; a researcher who used `LQRPlanner(problem)` finds the estimation
mirror in the same shape. Every later estimator (time-varying, along a trajectory, extended,
unscented, particle, moving horizon) must land in a slot this layout already has.

Pyro 2 showed the two lessons the layout must keep: a block is named for what it is and a
design for how it was obtained (`StateObserver` + `kalman()`), and a design is made on a
model object, not on loose arrays (`synthesize_lqr_controller(ss, cf)`). Pyro also showed
what to avoid: a second class hierarchy for controllers, evidence stored on the block
(`obs.P`), an argument whose presence changes another argument's shape (`G=`), and the
operating point subtracted by hand in a demo.

## 2. Principles, from the core

1. **A block is what its ports say** (`core/feedback.py`): a controller reads `y` or `x`
   and `r` and gives `u`; an observer reads `u` and `y` and gives `x`, the estimate. Role
   keys `plant`, `controller`, `observer` name blocks in a diagram. No controller or
   observer base class.
2. **Arrays at the boundary, objects inside.** Gains are array functions; blocks carry their
   gain in `params`; designs take a model or a plant; problems and solutions are the nouns
   researchers hold.
3. **Linearize, then design.** A design on a nonlinear plant is `linearize` followed by the
   linear design, and the operating point travels on the linear model.
4. **The operating point is the nominal state and the nominal inputs** **[ruled]**:
   `state.nominal_value` is x̄ (pyro's `xbar`), the input ports' `nominal_value` is ū. `x0`
   is the start of a simulation (and the planners' default `x_start`), not an operating
   point. Every analysis and design verb takes `x_bar=` / `u_bar=` and defaults to the
   nominal when they are omitted, silently: the default is a declared quantity, not a guess,
   so RULES 4.12 does not ask for a warning. Spelling: tools say `x_bar`, blocks say `xbar`
   (RULES 5.20, the neighbourhood; the V4 list unifies later).
5. **Evidence lives in the solution record**, never on the block: `P` is the planner's or
   the estimator tool's, `print(block)` shows the gain and the poles.
6. **Textbook bodies** (RULES 5.3): the Riccati equation, the duality transpose, the
   innovation form are named lines with the equation as a comment; plumbing goes to helpers.

## 3. The layers, both families side by side

```
Layer 0  gains on arrays            lqr_gain(A, B, Q, R) -> K                 kalman_gain(A, C, Q_n, R_n) -> L  (= lqr_gain(Aᵀ, Cᵀ, Q_n, R_n)ᵀ)
                                    place_gain(A, B, poles) -> K              luenberger_gain(A, C, poles) -> L  (= place_gain(Aᵀ, Cᵀ, poles)ᵀ)
                                    lqr_gain_schedule(..., tf) -> (t, K, S)   kalman_gain_schedule(..., P0, tf) -> (t, L, P)   [later]

Layer 1  blocks                     StateFeedbackController(K, xbar, ubar)            x, r -> u    u = ū − K (x − r)
                                    TimeVaryingStateFeedbackController(t, K, ...)    x, r -> u    u = ū − K(t)(x − r)
                                    TrajectoryFeedbackController(traj, K)            x -> u       u = u_d(t) − K(t)(x − x_d(t))
                                    StateObserver(A, B, C, L, D, xbar, ubar, ybar)    u, y -> x    dx̂ = A(x̂−x̄) + B(u−ū) + L(y−ŷ)
                                    TimeVaryingStateObserver(t, L, A, B, C, ...)      u, y -> x    L(t)                                 [later]
                                    TrajectoryObserver(traj, L, A, B, C schedules)    u, y -> x    δx̂ about x_d(t), y_d(t)              [later]
                                    ExtendedKalmanFilter(model, Q_n, R_n, x0, P0)     u, y -> x, P  A(x̂), C(x̂) online, P a state        [later]
                                    UnscentedKalmanFilter / ParticleFilter (StepSystem) u, y -> x   sampled, in a Computer              [later]

Layer 2  designs on a model         lqr(model, Q, R)  place(model, poles)       kalman(model, Q_n, R_n)  luenberger(model, poles)
         or a plant                 lqr_finite_horizon(model, Q, R, S_f, tf)  kalman_finite_horizon(model, Q_n, R_n, P0, tf)  [later]
                                    trajectory_lqr(sys, traj, Q, R)           trajectory_kalman(sys, traj, Q_n, R_n, P0)       [later]
                                    lqg(model, Q, R, Q_n, R_n) -> controller box {observer, gain}
                                    one signature (R14): lqr(sys, Q, R, *, x_bar=None, u_bar=None, t=0.0, params=None, method, eps):
                                      an LTISystem as is, with the point it carries; any other System linearized at (x̄, ū),
                                      about its command port only; arrays go to the gains, lqr_gain(A, B, Q, R) -> K

Layer 3  problems and solutions     PlanningProblem / StochasticPlanningProblem        EstimationProblem                                  [later]
                                    Planner(problem).solve() -> PlanningSolution      Estimator-tool(problem).solve() -> EstimationSolution
                                      policy (a controller block)                       the estimator block
                                      solver record (RiccatiRecord: K, P, poles)        record (L, P, poles; or the P(t) schedule)
                                      trajectory (nominal run), evaluation (J draws)    trajectory (x̂ along the data), evaluation (error statistics)
                                      cost_to_go J(x)                                   covariance P(t), the dual of J
                                    LQRPlanner, LQGPlanner (v0.3)                      KalmanEstimator, ExtendedKalmanEstimator, ParticleEstimator, MovingHorizonEstimator

The loop  ctl @ plant       lqg(…) @ plant       a hand-built box: add_subsystem(observer, "observer"), add_subsystem(gain, "gain"),
                                                 connect x and u, expose("r", "y", "u"), then box @ plant       closed_loop(ctl, plant, observer=obs)  (AC-4)
```

(`(observer + gain).autowire()` also builds the box, but warns about the open `y` and `r`
before they are exposed, so the demos and `lqg` wire it with `connect`.)

**The three rungs of each family, and where the EKF sits.**

| Rung | Control | Estimation | Linearized | Gain |
| --- | --- | --- | --- | --- |
| About an equilibrium | `lqr`, `place` -> constant `K` | `kalman`, `luenberger` -> constant `L` | once, at x̄ | algebraic Riccati, or placement |
| Along a nominal trajectory, offline | `trajectory_lqr(sys, traj, Q, R)` -> `K(t)`, backward Riccati from `S_f` | `trajectory_kalman(sys, traj, Q_n, R_n, P0)` -> `L(t)`, forward Riccati from `P0` (the linearized Kalman filter about `x_d(t)`, `u_d(t)`, `y_d(t)`) | at every sample of the trajectory, `A_k`, `B_k`, `C_k` | the same exact Hamiltonian step, dual pair, opposite direction |
| At the current point, online | iLQR / MPC (research lane) | `ExtendedKalmanFilter`: `A(x̂)`, `C(x̂)` inside `f`, `P` integrated as state | every instant, at x̂ | the Riccati differential equation integrated with the estimate |

The EKF along a trajectory is the middle rung: the same block shape as `TrajectoryFeedbackController` (schedules in `params`, `interpolate_schedule` between samples, held past the end), with the plant's nominal `y_d(t) = h(x_d, u_d)` as a third schedule and the estimate written as `x̂ = x_d(t) + δx̂`. Nothing in the core changes for it; it needs the Riccati machinery to run forward on the dual pair (the backward map already exists in `riccati_step`) and `interpolate_schedule` to have one home both bands import.

## 4. Names **[ruled unless marked]**

- Blocks: `StateFeedbackController`, `StateObserver` (returned by both `kalman` and
  `luenberger`; its `name` reads "Kalman Filter" or "Luenberger Observer" for `print` and
  `plot_diagram`). Later: `TrajectoryObserver`, `ExtendedKalmanFilter`, `UnscentedKalmanFilter`,
  `ParticleFilter`. Blocks are named for what they are; a design verb for how it is obtained.
- Verbs: `lqr`, `place`, `lqg`, `kalman`, `luenberger`; later `trajectory_kalman`,
  `kalman_finite_horizon`. Gains `*_gain`, band-level. The `_at_operating_point` twins leave:
  every verb takes a plant and `x_bar=`.
- Modules: `control/design.py` (the control verbs and gains, folding `lqr.py` and `place.py`;
  the module shadow S62 closes), `estimation/observer.py`, `estimation/design.py`. Shared
  math in the analysis band, where RULES 4.3's adapters live: `analysis/riccati.py` (`care`,
  `hamiltonian_matrix`, `riccati_step(…, direction)`, `riccati_map`; used by control,
  estimation and `LQRPlanner`) **[ask: new module]**, and the model adapter
  `design_model(sys, *, x_bar, u_bar, t, params, method, eps) -> (A, B, C, D, x̄, ū, ȳ)` in
  `analysis/linearization.py` (an `LTISystem` returns its stored point and refuses `x_bar=`;
  any other system is linearized with `wrt=command_port(sys)` and `of="y"`, `u_bar` sliced to
  the command, so a plant with `d` / `n` ports gives a gain over `u` alone; the name avoids
  `LQRPlanner.linear_model`). `interpolate_schedule` moves to `core/trajectory.py`. The
  dependency law gains two lines: "`control/` and `estimation/` import `analysis`
  (`design_model`, `riccati`, `structural`): the designs are adapters on a linearization"
  (today's lazy imports in `lqr.py:87`, `place.py:14,100` already do this unrecorded), and
  "`estimation.design` imports `control.design.place_gain`: the dual placement". Later
  `estimation/extended.py`, `unscented.py`, `particle.py`, `problems.py`, `estimators.py`.
- Ports: estimator `u`, `y` -> `x`; labels with a hat, copied from the plant by the plant-taking
  verbs (`θ̂`), `x̂_i` from arrays. Signals: `plant:x` against `controller/estimator:x`.
- Noise: `Q_n`, `R_n` everywhere in the estimation band, gains included
  (`kalman_gain(A, C, Q_n, R_n)`), so `Q`, `R` keep their one meaning as LQR weights (RULES
  5.4): `Q_n` the n×n state-noise intensity, `R_n` the p×p measurement intensity
  (randomness.md: `Q_n = B_w W B_wᵀ`, `R_n = D_v V D_vᵀ`). No `G=`. A plant-taking form
  reading `B_w = ∂f/∂d`, `D_v = ∂h/∂n` and the noise blocks' `psd` was planned here and
  dropped (rulings R20): the design takes the textbook `Q_n`, `R_n`.
- **A block that reads a measurement and commands nothing is an estimator, not a
  controller**: `require_command(block)` in `core/feedback.py` refuses it with one message
  ("… reads y but gives no u: an estimator; put it in a controller box"), used by
  `default_control_out` (`@`, `closed_loop`), `default_computer_boundary_ports` (`%`),
  `feedback()` and `plot_control_law`'s `law_roles_of`. Today all three fall back to the first
  output, `x`, and would wire the estimate into the plant's `u`. The two messages that name
  `estimator=` (`composition.py:216`) and `lqg()` are worded for what exists at each commit.
- **The role name, `estimator` or `observer`** **[ask]**. The 2026-10-11 record rejected
  `observer` as the role ("Luenberger only; a Kalman filter is an estimator"). Layer 3 brings
  the collision: the tool that mirrors `Planner` is naturally `Estimator`, and that is the
  block's role name. Two ways out, decide once, before any estimator exists:
  (a) keep the role `estimator`; name the tool family by its product, `KalmanDesign`,
  `ParticleFilterDesign` (no `Estimator` noun);
  (b) rename the role to `observer` (plant / controller / observer; in control texts an
  observer is any state reconstructor, Kalman's being the optimal one) and keep `Estimator`
  for the Layer 3 tools (`KalmanEstimator(problem)` beside `LQRPlanner(problem)`; blocks stay
  `StateObserver`, `ExtendedKalmanFilter`, …). Cost of (b) today: `RESERVED_IDS`,
  `Roles.estimator`, two messages, the plan docs and tests, no shipped code. Recommended: (b).

## 5. Layer 3, the long view (not built now; the architecture must allow it)

`EstimationProblem(sys, disturbances, x0_distribution, measured=("y",), data=None, tf=None)`
shares the `StochasticPlanningProblem` spine (system, `disturbances={"d": WhiteNoise(psd=W),
"n": WhiteNoise(psd=V)}`, the prior `x0_distribution` = x̂₀ and `P0`) and adds the record of
measurements for offline runs (a `Trajectory` of `(t, u, y)`) and which ports are measured.
A `StochasticPlanningProblem` converts to it (`from_problem`), which is how `LQGPlanner` gets
its filter. `EstimationSolution(problem, estimator, record, trajectory, evaluation,
covariance)` mirrors `PlanningSolution` field for field: the block, the tool's own account
(`L`, `P`, poles, or the `P(t)` schedule), the estimate along the data, Monte Carlo error
statistics over the problem's draws (the dual of `J`), and `covariance` as the dual of
`cost_to_go`. Tools: `KalmanEstimator` (steady-state or time-varying by the horizon, as
`LQRPlanner` picks by `tf`), `ExtendedKalmanEstimator`, `UnscentedKalmanEstimator`,
`ParticleEstimator`, `MovingHorizonEstimator` (the dual of MPC: a trajopt over a window of
past data). The evaluator's dual scores estimation error, consistency (NEES) and innovation
whiteness. Where the pieces come from today: `Planner` base (`planning/planner.py`),
`PlanningSolution` (`planning/results.py`), `evaluate` on the compiled problem (today's
`MonteCarloEvaluator`, folding into it in v0.3, R44), `nominal_trajectory`.

## 6. Stress test: the slots the later estimators take

| Estimator | Shape | Ports / state | Entry | Verdict |
| --- | --- | --- | --- | --- |
| Time-varying Kalman (continuous) | `TimeVaryingStateObserver`, `L(t)` from the forward Riccati schedule | `u`, `y` -> `x`; x̂ only | `kalman_finite_horizon(model, Q_n, R_n, P0, tf)` | holds: the dual of `lqr_finite_horizon` line for line |
| Linearized Kalman along a trajectory | `TrajectoryObserver`: schedules `t, x_d, u_d, y_d, A, B, C, L` in `params`; `x̂ = x_d(t) + δx̂` | `u`, `y` -> `x` | `trajectory_kalman(sys, traj, Q_n, R_n, P0)` | holds: the dual of `trajectory_lqr`; needs `riccati_step` forward and `interpolate_schedule` in `core` (A1, A2) |
| Extended Kalman filter | `ExtendedKalmanFilter(model, Q_n, R_n, x0, P0)`: state `[x̂; vec(P)]`; `A(x̂) = ∂f/∂x`, `C(x̂) = ∂h/∂x` taken **directly on `model.f` / `model.h`** inside `f` (`jax.jacfwd` when tracing, central differences on NumPy), never through `analysis.jacobian`, which compiles an evaluator and coerces to NumPy per call | `u`, `y` -> `x` (a slice), `P` | the block is the algorithm; later `ExtendedKalmanEstimator(problem)` | holds: `estimate` is already a port function, not `compute_state`; the model-based params rule (DESIGN §4) covers `model.params` |
| Unscented Kalman, discrete Kalman | `StepSystem`s in a hand-built `StepDiagramSystem` (the flow `+` refuses a `StepSystem`, `%` refuses a flow diagram): `computer = StepDiagramSystem(); add_subsystem(ukf, "observer"); add_subsystem(gain, "gain"); connect…; computer % dt @ plant` | `u`, `y` -> `x`, `P`; `k` | `UnscentedKalmanFilter(model, Q_d, R_d, dt)`; `kalman(…, dt=)` or `kalman_discrete` with `c2d` (P6) | strains the sampled seam: `expose_computer_boundary_ports` auto-exposes one block only and `as_step_diagram` names it `controller` regardless of `id` (AC-8a, S31), and the numbers need P6; P4 keeps `kalman` continuous in name |
| Particle filter | `StepSystem(N·n + N)`, counter-based draws from `(seed, k)`, `is_random`, `realize(key)` | `u`, `y` -> `x` (weighted mean) | `ParticleFilter(model, process_noise: Distribution, measurement_noise: Distribution, n_particles, seed)` | holds the port convention; needs RN-2 (`sample(key)`) and the `realize` tree |
| Moving-horizon estimator | a trajopt over a window of past data | `u`, `y` -> `x` | `MovingHorizonEstimator(problem)` (Layer 3) | holds: the dual of MPC; needs `EstimationProblem.data` |
| Parameter estimators (RLS, gradient laws) | `u`, `y` -> `theta` | not `x` | `identification/` online, or a new role | the role means a *state* estimate; decide the noun when it lands |

**What P4 does so these fit**: `estimate` as an explicit port function; `kalman` continuous
and steady-state in name; `interpolate_schedule` and the Riccati machinery in shared homes,
the backward step on the dual pair being the forward filter step (no direction parameter);
the role name `observer` settled before the first observer shipped; `require_command` in the
resolver so no reader takes an estimate for a command. **Noted for AC-4**: once a plant
exposes `x` and an observer gives `x` in one diagram, `autowire` finds two candidates for any
`x` input and `expose("x")` refuses; the `observer=` keyword wires by role, not by name
(landed 2026-10-14).

## 6b. Ask-first items, by name (AGENTS "Ask first")

Core: `operating_point()`'s default (A0); the `LTISystem` constructor (A1); `require_command`
in `core/feedback.py` and its callers in `composition.py` (B1); `interpolate_schedule` to
`core/trajectory.py` (A1). Public names: `lqr_at_operating_point` and
`place_at_operating_point` removed, the `control.lqr` / `control.place` module paths removed,
`StateObserver`, `luenberger`, `kalman`, `lqg` added to the root prelude (A3, B5). New
dependency: none. New modules: `analysis/riccati.py` (A2). Student-facing prose touched by
the rename: README `:143`, ROADMAP §4 `:116`, `:170`, `:171`, the markdown cells named in A3,
the gro501 plan P4 section. The freeze: ROADMAP §4.1 gate 7 is paused (`ROADMAP.md:136-138`);
ROADMAP §2 `:33` ("names change only with a deprecation note") is read with that pause.
Role rename (b), if taken: reverses the 2026-10-11 ruling at `2026-10-11-automation-by-convention.md:499`
and needs its own dated record.

## 7. Verification

- Before A0: seeded baseline in the scratchpad of every `K` (`lqr`, `place`,
  `lqr_finite_horizon`, `trajectory_lqr`, `LQRPlanner`), every `linearize` matrix the tests
  build with an explicit `x_bar`, and the closed-loop rollouts of `test_control_analysis.py`
  and `test_lqr_planner.py`; run twice, `cmp`.
- After each Part A commit: the baseline `cmp`s byte-identical; `pytest` on the
  "Cross-cutting or handoff" row; after A3 the notebook smoke check ("Notebook change") on
  the five migrated notebooks; `ruff check . && ruff format --check .`.
- After each Part B commit: `pytest tests/unittest/test_estimation.py` plus "Narrow module
  change"; after B5 also `test_teaching_surface.py`, `test_public_imports.py`,
  `test_teaching_imports.py`, `test_repo_contract.py`.
- Textbook check, stated in the report: `StateObserver.f`, `kalman_gain`, `luenberger_gain`,
  `care`, `lqg` read next to `dp.py` and `lqr_gain`.
- After C1: the demo headless (`MPLBACKEND=Agg`); the estimate converges to the true state;
  `plot_diagram` shows controller{estimator, gain} and plant.
- Before every push: fetch and compare with origin; the "Always before push" row.

