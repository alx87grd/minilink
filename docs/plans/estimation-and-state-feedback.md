# State feedback and state estimation as one pattern: the design beyond P4

**Status:** decided 2026-10-13. P4 landed 2026-10-14: the gains on arrays, the blocks
(`StateFeedback`, `StateObserver`), the designs on a model or a plant (`lqr`, `place`,
`kalman`, `luenberger`, `lqg`), the observer role, and the review that made the verbs
system-only, split the output reference out of `StateFeedback` and put `LQRPlanner` on the
command port (rulings R14–R17). Its contract is DESIGN §3 (the package map and the
dependency law) and §4 (the observer role); the rulings and their reasons are in
[2026-10-14-state-space-design-rulings.md](../reviews/2026-10-14-state-space-design-rulings.md).
This doc holds the design of what comes next: `LQGPlanner` (v0.4; §5) and the estimation
follow-ups of v0.9, each in its slot (the time-varying and trajectory filters, the extended
Kalman filter, the sampled filters, and Layer 3, the estimation problem and its tools; §3,
§5, §6). The open work is in TODO.md (the P4 follow-ups, `LQGPlanner`, the estimation
follow-ups). P4's build plan, its ask-first list and its verification were cut once it
landed (the history is in git). **Where it sits:** `LQGPlanner` stays a v0.4 feature
([TODO.md](TODO.md) §4), the follow-ups v0.9 rows; the sampled filters read the core's hybrid
loop and the draws (TODO §1.3, §1.5).

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

A block is what its ports say (`core/roles.py`): an observer reads `u` and `y` and gives `x`,
the estimate, and the role keys `plant`, `controller`, `observer` name blocks in a diagram,
with no controller or observer base class. Gains are array functions, blocks carry their gain
in `params`, designs take a model or a plant, and problems and solutions are the nouns
researchers hold. A design on a nonlinear plant is `linearize` followed by the linear design,
and the operating point (`state.nominal_value` is x̄, the input ports' `nominal_value` is ū;
every verb takes `x_bar=` / `u_bar=`) travels on the linear model. Evidence lives in the
solution record, never on the block. The bodies are textbook (RULES 5.3): the Riccati
equation, the duality transpose and the innovation form are named lines with the equation as
a comment. DESIGN §3 and §4 state these rules as built.

## 3. The layers, both families side by side

```
Layer 0  gains on arrays            lqr_gain(A, B, Q, R) -> K                 kalman_gain(A, C, Q_n, R_n) -> L  (= lqr_gain(Aᵀ, Cᵀ, Q_n, R_n)ᵀ)
                                    place_gain(A, B, poles) -> K              luenberger_gain(A, C, poles) -> L  (= place_gain(Aᵀ, Cᵀ, poles)ᵀ)
                                    lqr_gain_schedule(..., tf) -> (t, K, S)   kalman_gain_schedule(..., P0, tf) -> (t, L, P)   [later]

Layer 1  blocks                     StateFeedback(K, xbar, ubar)            x, r -> u    u = ū − K (x − r)
                                    TimeVaryingFeedback(t, K, ...)    x, r -> u    u = ū − K(t)(x − r)
                                    TrajectoryFeedback(traj, K)            x -> u       u = u_d(t) − K(t)(x − x_d(t))
                                    StateObserver(A, B, C, L, D, xbar, ubar, ybar)    u, y -> x    dx̂ = A(x̂−x̄) + B(u−ū) + L(y−ŷ)
                                    TimeVaryingStateObserver(t, L, A, B, C, ...)      u, y -> x    L(t)                                 [later]
                                    TrajectoryObserver(traj, L, A, B, C schedules)    u, y -> x    δx̂ about x_d(t), y_d(t)              [later]
                                    ExtendedKalmanFilter(model, Q_n, R_n, x0, P0)     u, y -> x, P  A(x̂), C(x̂) online, P a state        [later]
                                    UnscentedKalmanFilter / ParticleFilter (StepSystem) u, y -> x   on a clock, `% dt` (R146)           [later]

Layer 2  designs on a model         lqr(model, Q, R)  place(model, poles)       kalman(model, Q_n, R_n)  luenberger(model, poles)
         or a plant                 lqr_finite_horizon(model, Q, R, S_f, tf)  kalman_finite_horizon(model, Q_n, R_n, P0, tf)  [later]
                                    trajectory_lqr(sys, traj, Q, R)           trajectory_kalman(sys, traj, Q_n, R_n, P0)       [later]
                                    lqg(model, Q, R, Q_n, R_n) -> controller box {observer, gain}
                                    one signature (R14): lqr(sys, Q, R, *, x_bar=None, u_bar=None, t=0.0, params=None, method, eps):
                                      an LTISystem as is, with the point it carries; any other System linearized at (x̄, ū),
                                      about its command port only; arrays go to the gains, lqr_gain(A, B, Q, R) -> K

Layer 3  problems and solutions     PlanningProblem (stochastic by its parts, R143)    EstimationProblem                                  [later]
                                    Planner(problem).solve() -> PlanningSolution      Estimator-tool(problem).solve() -> EstimationSolution
                                      policy (a controller block)                       the estimator block
                                      solver record (RiccatiRecord: K, P, poles)        record (L, P, poles; or the P(t) schedule)
                                      trajectory (nominal run), evaluation (J draws)    trajectory (x̂ along the data), evaluation (error statistics)
                                      cost_to_go J(x)                                   covariance P(t), the dual of J
                                    LQRPlanner, LQGPlanner (v0.4)                      KalmanEstimator, ExtendedKalmanEstimator, ParticleEstimator, MovingHorizonEstimator

The loop  ctl @ plant       lqg(…) @ plant       a hand-built box: add_subsystem(observer, "observer"), add_subsystem(gain, "gain"),
                                                 connect x and u, expose("r", "y", "u"), then box @ plant       closed_loop(ctl, plant, observer=obs)  (AC-4)
```

**The three rungs of each family, and where the EKF sits.**

| Rung | Control | Estimation | Linearized | Gain |
| --- | --- | --- | --- | --- |
| About an equilibrium | `lqr`, `place` -> constant `K` | `kalman`, `luenberger` -> constant `L` | once, at x̄ | algebraic Riccati, or placement |
| Along a nominal trajectory, offline | `trajectory_lqr(sys, traj, Q, R)` -> `K(t)`, backward Riccati from `S_f` | `trajectory_kalman(sys, traj, Q_n, R_n, P0)` -> `L(t)`, forward Riccati from `P0` (the linearized Kalman filter about `x_d(t)`, `u_d(t)`, `y_d(t)`) | at every sample of the trajectory, `A_k`, `B_k`, `C_k` | the same exact Hamiltonian step, dual pair, opposite direction |
| At the current point, online | iLQR / MPC (research lane) | `ExtendedKalmanFilter`: `A(x̂)`, `C(x̂)` inside `f`, `P` integrated as state | every instant, at x̂ | the Riccati differential equation integrated with the estimate |

The EKF along a trajectory is the middle rung: the same block shape as `TrajectoryFeedback` (schedules in `params`, `interpolate_schedule` between samples, held past the end), with the plant's nominal `y_d(t) = h(x_d, u_d)` as a third schedule and the estimate written as `x̂ = x_d(t) + δx̂`. Nothing in the core changes for it; it needs the Riccati machinery to run forward on the dual pair (the backward map already exists in `riccati_step`); `interpolate_schedule` already has the one home both bands import (`core/trajectory.py`).

## 4. Names

The names P4 shipped (`StateFeedback`, `StateObserver`, `lqr`, `place`, `lqg`, `kalman`,
`luenberger`, the `*_gain` functions, `analysis/riccati.py`, `design_model`, the role
`observer`, ruled R11) are in DESIGN §3 and §4. Blocks are named for what they are, a design
verb for how it is obtained. The names reserved for later:
- blocks: `TimeVaryingStateObserver`, `TrajectoryObserver`, `ExtendedKalmanFilter`,
  `UnscentedKalmanFilter`, `ParticleFilter`;
- verbs: `trajectory_kalman`, `kalman_finite_horizon`;
- modules: `estimation/extended.py`, `unscented.py`, `particle.py`, `problems.py`,
  `estimators.py`;
- noise: `Q_n`, `R_n` everywhere in the estimation band, gains included, so `Q`, `R` keep
  their one meaning as LQR weights (RULES 5.4): `Q_n` the n×n state-noise intensity, `R_n`
  the p×p measurement intensity (DESIGN §4, "Randomness": `Q_n = B_w W B_wᵀ`, `R_n = D_v V D_vᵀ`);
- the Layer 3 tools take the noun `Estimator` (`KalmanEstimator(problem)` beside
  `LQRPlanner(problem)`), which the role name `observer` left free.

## 5. Layer 3, the long view (not built now; the architecture must allow it)

`EstimationProblem(sys, x_start=Gaussian(x̂₀, P0), measured=("y",), data=None, tf=None)`
shares the `PlanningProblem` spine of a problem stochastic by its parts (R143): the system, its
noise as source blocks (`WhiteNoise(psd=W)` on the process input, `WhiteNoise(psd=V)` on the
measurement, R128), the prior as the law of `x_start`; it adds the record of measurements for
offline runs (a `Trajectory` of `(t, u, y)`) and which ports are measured. A stochastic
`PlanningProblem` converts to it (`from_problem`), which is how `LQGPlanner` gets its filter.
`EstimationSolution(problem, estimator, record, trajectory, evaluation, covariance)` mirrors `PlanningSolution` field for field: the block, the tool's own account
(`L`, `P`, poles, or the `P(t)` schedule), the estimate along the data, Monte Carlo error
statistics over the problem's draws (the dual of `J`), and `covariance` as the dual of
`cost_to_go`. Tools: `KalmanEstimator` (steady-state or time-varying by the horizon, as
`LQRPlanner` picks by `tf`), `ExtendedKalmanEstimator`, `UnscentedKalmanEstimator`,
`ParticleEstimator`, `MovingHorizonEstimator` (the dual of MPC: a trajopt over a window of
past data). The evaluator's dual scores estimation error, consistency (NEES) and innovation
whiteness. Where the pieces come from today: `Planner` base (`planning/planner.py`),
`PlanningSolution` (`planning/results.py`), `evaluate` on the compiled problem (`planning/evaluator.py`;
`MonteCarloEvaluator` folded into it in v0.3, R44), `nominal_trajectory`.

## 6. Stress test: the slots the later estimators take

| Estimator | Shape | Ports / state | Entry | Verdict |
| --- | --- | --- | --- | --- |
| Time-varying Kalman (continuous) | `TimeVaryingStateObserver`, `L(t)` from the forward Riccati schedule | `u`, `y` -> `x`; x̂ only | `kalman_finite_horizon(model, Q_n, R_n, P0, tf)` | holds: the dual of `lqr_finite_horizon` line for line |
| Linearized Kalman along a trajectory | `TrajectoryObserver`: schedules `t, x_d, u_d, y_d, A, B, C, L` in `params`; `x̂ = x_d(t) + δx̂` | `u`, `y` -> `x` | `trajectory_kalman(sys, traj, Q_n, R_n, P0)` | holds: the dual of `trajectory_lqr`; needs `riccati_step` run forward (`interpolate_schedule` is in `core/trajectory.py`) |
| Extended Kalman filter | `ExtendedKalmanFilter(model, Q_n, R_n, x0, P0)`: state `[x̂; vec(P)]`; `A(x̂) = ∂f/∂x`, `C(x̂) = ∂h/∂x` taken **directly on `model.f` / `model.h`** inside `f` (`jax.jacfwd` when tracing, central differences on NumPy), never through `analysis.jacobian`, which compiles an evaluator and coerces to NumPy per call | `u`, `y` -> `x` (a slice), `P` | the block is the algorithm; later `ExtendedKalmanEstimator(problem)` | holds: `estimate` is already a port function, not `compute_state`; the model-based params rule (DESIGN §4) covers `model.params` |
| Unscented Kalman, discrete Kalman | `StepSystem`s on a clock, `ukf % dt` and `gain % dt` wired as the `lqg` box is (`x` and `u` connected) into a `SampledDiagramSystem`, then `@ plant` (R126, R146); today a hand-built `StepDiagramSystem` (`add_subsystem`, `connect`, `computer % dt @ plant`), as the flow `+` refuses a `StepSystem` | `u`, `y` -> `x`, `P`; `k` | `UnscentedKalmanFilter(model, Q_d, R_d, dt)`; `kalman(…, dt=)` or `kalman_discrete` with a `c2d` | strains today's sampled seam: `expose_computer_boundary_ports` auto-exposes one block only and `as_step_diagram` names it `controller` regardless of `id` (AC-8a, S31); the discrete numbers need a `c2d`, which has no row since P6 was dropped (2026-10-17); `kalman` stays continuous in name |
| Particle filter | `StepSystem(N·n + N)`, counter-based draws from `(seed, k)`, its `seed` an integer leaf of `problem.realize(key, n_trials)`'s tree (R144) | `u`, `y` -> `x` (weighted mean) | `ParticleFilter(model, process_noise: Distribution, measurement_noise: Distribution, n_particles, seed)` | holds the port convention; needs RN-2 (`sample(key)`) and the realized tree |
| Moving-horizon estimator | a trajopt over a window of past data | `u`, `y` -> `x` | `MovingHorizonEstimator(problem)` (Layer 3) | holds: the dual of MPC; needs `EstimationProblem.data` |
| Parameter estimators (RLS, gradient laws) | `u`, `y` -> `theta` | not `x` | online identification in `minilink.learning` (R66, R67), or a new role | the role means a *state* estimate; decide the noun when it lands |

P4 left these slots ready: `estimate` is an explicit port function, `kalman` is continuous
and steady-state in name, the Riccati machinery (`analysis/riccati.py`) and
`interpolate_schedule` (`core/trajectory.py`) have shared homes, and `require_command` in the
resolver keeps any reader from taking an estimate for a command.
