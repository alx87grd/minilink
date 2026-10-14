# State feedback and state estimation as one pattern: the design for P4 and beyond

**Status:** decided 2026-10-13; §6 (the control sweep, the observer, the designs, `lqg`, the demos) landed 2026-10-14 in nine commits, the open follow-ups are TODO rows (P4 follow-ups, `LQGPlanner`, the estimation follow-ups); §3, §5 and §7 are the live design for what comes next. The rulings and their reasons are in
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
                                    two signatures, one boundary test isinstance(first, System) (MATLAB: lqr(sys, Q, R) / lqr(A, B, Q, R)):
                                      lqr(A, B, Q, R, *, xbar=None, ubar=None)                                  arrays, today's form
                                      lqr(sys, Q, R, *, x_bar=None, u_bar=None, t=0.0, params=None, method, eps)  an LTISystem as is, with the
                                      point it carries; any other DynamicSystem linearized at (x̄, ū), about its command port only

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
  (randomness.md: `Q_n = B_w W B_wᵀ`, `R_n = D_v V D_vᵀ`). No `G=`. The plant-taking form
  reads `B_w = ∂f/∂d`, `D_v = ∂h/∂n` from the ports when `disturbance=` / `noise=` blocks are
  given (RN-4).
- **A block that reads a measurement and commands nothing is an estimator, not a
  controller**: `require_command(block)` in `core/feedback.py` refuses it with one message
  ("… reads y but gives no u: an estimator; put it in a controller box"), used by
  `_default_control_out` (`@`, `closed_loop`), `default_computer_boundary_ports` (`%`),
  `feedback()` and `plot_control_law`'s `_law_roles`. Today all three fall back to the first
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
`PlanningSolution` (`planning/results.py`), `MonteCarloEvaluator`, `nominal_trajectory`.

## 6. What changed on 2026-10-14 (P4 + the control sweep), in the order it landed

Behaviour-preserving steps follow the AGENTS recipe: a seeded baseline of every gain and
closed-loop rollout the tests and demos build, `cmp` byte-identical after.

### Part A: the operating point and the control band (4 commits)

A0. **The nominal state is x̄** (core). `operating_point()` in `analysis/derivatives.py`
    defaults `x_bar` to `sys.state.nominal_value` (ū already comes from the ports). Every
    verb that linearizes keeps its `x_bar=` / `u_bar=` keywords: `jacobian`, `linearize`,
    `linearize_matrices`, `transfer_function`, `bode`/`pzmap`/… , `modal_analysis`,
    `find_equilibrium`'s guess, `region_of_attraction`, `plot_control_law`'s pins; their
    docstrings say "the nominal state" instead of "`sys.x0`". `x0` is the start of
    `compute_trajectory` only. Diagrams already stack both (`wiring.py`). DESIGN §4: one
    paragraph, two quantities with one owner each. S29 / scan core#11 reworded (no folding of
    `nominal_value` into `x0`). Catalog plants whose natural operating point is not zero may
    declare `state.nominal_value` (none needs it for the courses today). Impact: every bare
    `linearize()` in examples and tests is on a loop whose `x0` is zero; plant linearizations
    pass `x_bar`; `test_derivatives.py:326` (`plant.linearize()`) is checked. Tests:
    `operating_point` reads the nominal state; a plant with `x0 ≠ nominal` linearizes at the
    nominal. **Found by review**: `test_derivatives.py:128-136` (`test_defaults_are_the_nominal_point`
    sets `plant.x0` and expects the Jacobian there) is rewritten to set
    `state.nominal_value`; `plot_control_law` keeps pinning internal states at `x0`
    (`graphical/port_map.py`), unchanged; the comments "the operating point every tool
    defaults to" in `demos/analysis/analysis_frequency.py:10-12`, `analysis_root_locus.py:9`,
    `frequency_response.ipynb` cells 4 and 23 and `showcase_minilink.ipynb` cells 18/20 are
    true only because those `x0` are zero: reworded in the two demos (agent lane), flagged
    for the maintainer in the two notebooks. Catalog plants with a nonzero default `x0`
    (`vehicles/propulsion.py:161`, `astro/three_body.py:96`) are never linearized bare.
A1. **The model carries its operating point.** `LTISystem(A, B, C, D, *, xbar=None,
    ubar=None, ybar=None)` as plain attributes, block spelling (zeros when `None`; `ybar = C
    x̄ + D ū` by default). `linearize()` fills them from the point it used and `ybar = h(x̄,
    ū)`. The linearized model's own state is `Δx`, so its `state.nominal_value` stays zero
    and no tool reads it for the point: `design_model` reads `model.xbar`. `interpolate_schedule`
    moves from `control/state.py` to `core/trajectory.py` (old import kept until A3).
A2. **`analysis/riccati.py`** **[ask: new module]**: `care(A, B, Q, R) -> P` (one
    `solve_continuous_are`, the equation as a comment), `hamiltonian_matrix`,
    `riccati_transition(…, direction=-1)` (the sign of `expm(±H dt)` a parameter, not a flip
    inside), `riccati_step`, `riccati_map`, moved from `control/lqr.py` with their tests.
    `LQRPlanner` imports `care` from here instead of calling SciPy itself. Baseline
    byte-identical (`lqr_gain_schedule`, `trajectory_lqr`, `LQRPlanner`). And
    `analysis/linearization.py` gains `design_model` (§4), tested on an `LTISystem`, a plant,
    and a plant with `d` / `n` ports (`B` has the command's width only).
A3. **`control/design.py` replaces `control/lqr.py` and `control/place.py`.** `lqr_gain`,
    `place_gain`, `lqr`, `place`, `lqr_gain_schedule`, `lqr_finite_horizon`, `trajectory_lqr`,
    `requested_poles` / `require_controllable` / `robust_gain` / `require_placed`. The two
    signatures of §3 behind `isinstance(first, System)`; arrays never enter `design_model`;
    the legacy order `lqr(plant, x_bar, Q, R)` is caught by the type test and refused with the
    new signature in the message. `lqr_at_operating_point` and `place_at_operating_point`
    are deleted, no alias; every call site migrates in this commit (freeze paused; course
    notebooks in the same commit): library `minilink/__init__.py`, `control/__init__.py`
    (exports `lqr`, `place` directly: S62 closes), `control/state.py` Sphinx refs
    (`:func:minilink.control.lqr.lqr` and three more), `analysis/lyapunov.py`,
    `planning/policy_synthesis/lqr.py`; demos `cartpole_lqr.py`,
    `pole_placement_pendulum.py`, `trajectory_lqr_cartpole.py`,
    `analysis_region_of_attraction.py`; notebooks `03_control` (code and its markdown cell
    4), `showcase_minilink` (code and markdown cell 22), `udes_gro860/cartpole_lqr`,
    `pendulum_value_iteration_vs_lqr_vs_ppo_sb3`
    (`lqr_at_operating_point(plant, x_bar, Q, R)` -> `lqr(plant, Q, R, x_bar=x_bar)`, the
    sentence naming the function adjusted, nothing else in the cell). **Not touched, by
    ruling**: the GRO501 notebooks `udes_gro501/numpy_state_space.ipynb` and
    `cartpole_dynamic_controller.ipynb` hand-write LQR, Kalman and LQG from scratch on
    purpose; `numpy_state_space` imports only the array form `lqr(A, B, Q, R)`, which keeps
    working, and its prose stays as it is. Tests
    `test_control_analysis`, `test_lqr_planner`, `test_analysis_lyapunov`, `test_core`,
    `test_graphics`, `test_random`, `test_derivatives` (`:18`, `:450-454` the `method=` /
    `eps=` test moves to `lqr(plant, …, method=, eps=)`), `test_public_imports`,
    `test_teaching_imports`, `test_teaching_surface` (registry rows and the
    `TEACHING_MODULE_FUNCTIONS` rows for `control.lqr` / `control.place` go); docs README
    verb table (`:143`), ROADMAP §4 rows `:116`, `:170`, `:171` (maintainer-owned prose:
    the function name only), DESIGN `:35` (facade example), `:44-46` (the "module name
    matches a factory" rule loses its only example: the rule is deleted or re-exampled),
    `:89`, `:142`, `:637`, `:944`, `docs/api/control.rst`, TODO (S62 closes), the gro501
    plan (P3 row), `fields.md`, `randomness.md`, `pyro-port-remaining.md` (`:44`, `:359`,
    `:360` and the name map). Review files are history and stay. Baseline: every `K`
    byte-identical.

### Part B: estimation (5 commits)

B1. **`estimation/observer.py`: `StateObserver(A, B, C, L, D=None, xbar=None, ubar=None,
    ybar=None, labels=None)`.** `DynamicSystem(n)`; ports `u` (m), `y` (p) ->
    `add_output_port("x", n, function=self.estimate, labels=hats, dependencies=())`;
    `self.id = "estimator"` (or `"observer"`, §4); `name = "State Observer"`; `x0` = x̂₀.
    Params `A, B, C, D, L, xbar, ubar, ybar`. `f`, three beats on `xp`, `u` by position:

        # dx̂ = A (x̂ − x̄) + B (u − ū) + L (y − ŷ),   ŷ = ȳ + C (x̂ − x̄) + D (u − ū)
        e_x, e_u = x_hat - xbar, u_in - ubar
        y_hat = ybar + C @ e_x + D @ e_u
        dx_hat = A @ e_x + B @ e_u + L @ (y - y_hat)

    `estimate` returns the state (the slice the EKF narrows later). `__str__` adds `L` and
    `eig(A − L C)`. **With it, in the same commit, the guard** (§4): `require_command` in
    `core/feedback.py`, called by `_default_control_out`, `default_computer_boundary_ports`,
    `feedback()`'s entry and `_law_roles`, so `observer @ plant`, `observer % dt @ plant` and
    `plot_control_law(observer)` refuse instead of taking `x` as the command. Tests
    (`tests/unittest/test_estimation.py`, after `TestPolePlacement`): JAX trace of `f`;
    convergence from a wrong x̂₀, with and without an operating point; `block_roles` /
    `roles`; `print`; the three refusals.
B2. **`estimation/design.py`: the gains and `luenberger`.** `luenberger_gain(A, C, poles)`:
    `observability(A, C)` with its own refusal, then `L = place_gain(Aᵀ, Cᵀ, poles)ᵀ` under the
    duality comment. `kalman_gain(A, C, Q_n, R_n)`: `P = care(Aᵀ, Cᵀ, Q_n, R_n)` under
    `# A P + P Aᵀ − P Cᵀ R_n⁻¹ C P + Q_n = 0`, then `# L = P Cᵀ R_n⁻¹`. `luenberger(A, B, C,
    poles, *, xbar, ubar, ybar)` and `luenberger(sys, poles, *, x_bar, u_bar, t, params, method,
    eps)` behind `isinstance(first, System)`, the `System` form through `design_model`
    (`analysis.linearization`), returning a `StateObserver` with the model's point and hat
    labels copied from the plant's state labels. Dependency law: the two lines of §4. Tests:
    `np.poly(A − L C)` equals the request; duality with `place_gain`; the unobservable
    refusal; `kalman_gain(A, C, Q_n, R_n) == lqr_gain(Aᵀ, Cᵀ, Q_n, R_n)ᵀ`; the GRO501
    `numpy_state_space` hand numbers reproduced.
B3. **`kalman(A, B, C, Q_n, R_n, *, …)` / `kalman(sys, Q_n, R_n, *, …)`**, same two forms,
    a `StateObserver` named "Kalman Filter".
B4. **`lqg(A, B, C, Q, R, Q_n, R_n, *, …)` / `lqg(sys, Q, R, Q_n, R_n, *, …)` in
    `control/design.py`**: `gain = lqr(...)`, `observer =
    kalman(...)` (lazy import), the box built explicitly and silently (RULES 4.6:
    `autowire` would warn about the open `y` and `r`): `DiagramSystem`, `add_subsystem(observer,
    "estimator")`, `add_subsystem(gain, "gain")`, two `connect`s, `expose("r", "y", "u")`,
    `name = "LQG Controller"`. The recipe stays what a student writes by hand. Dependency
    law: "`control.design` imports `estimation.design` inside `lqg`: an LQR gain on a Kalman
    estimate" (no cycle). Tests:
    keys `estimator`/`gain`, edge `r`, `y` -> `u`; `roles(box @ plant)`; the separation
    principle on the double integrator (`eig(linearize(box @ plant).A()) = eig(A − B K) ∪
    eig(A − L C)`); `lqg(CartPole(), …, x_bar=x_up) @ CartPole()` converges from a disturbed
    start.
B5. **Facades and contracts.** `estimation/__init__.py` as a `lazy_facade`; root prelude adds
    `StateObserver`, `luenberger`, `kalman`, `lqg`; `test_teaching_surface.py` registry;
    `docs/api/estimation.rst` (the API-page test `test_every_exported_name_has_an_api_page`
    requires it); DESIGN §3 estimation row and §4 observer contract; ROADMAP §4.2
    "`KalmanFilter`" -> "`kalman()`"; the gro501 plan's P4 section rewritten to this plan's
    names (it still specifies `LuenbergerObserver` and `kalman(A, B, C, Q, R)`); the
    `composition.py:216` message ("use estimator=") reworded until AC-4 lands.

### Part C: evidence and housekeeping (2 commits)

C1. **Demos, the library's evidence** (the GRO501 from-scratch notebooks are not the place for
    it). Two new scripts beside the updated LQR demos of A3:
    - `examples/demos/control/kalman_observer_pendulum.py`: `plant = Pendulum()` measuring
      the angle only (`Pendulum() >> Gain([[1, 0]])` as the plant box, or the catalog's
      position-output variant if one exists), `observer = kalman(plant, Q_n, R_n)`, the
      observer driven by the plant in one diagram (hand-wired `u`, `y`), a wrong x̂₀, then
      `plot_trajectory(signals=("plant:x", "estimator:x"))`: the estimate converging on the
      truth, and `print(observer)` for `L` and the observer poles.
    - `examples/demos/control/lqg_cartpole.py`: `plant = CartPoleWithNoisePort()`,
      `ctl = lqg(plant, Q, R, Q_n, R_n, x_bar=x_up)`, `loop = closed_loop(ctl, plant)`, noise by
      port `(WhiteNoise(1, psd=…, port="d") + WhiteNoise(4, psd=…, port="n") + loop).autowire()`,
      disturbed start, `plot_trajectory`, `plot_diagram` (the guide's Figure 12; pyro's twin
      is `cartpole_LQG.py`).
    RULES 6.1 / 6.10 / 6.13; both join `examples/README.md` and the nightly sweep.
C2. **Docs**: TODO P4 row leaves; new rows: `LQGPlanner` (v0.3, with RN-4), the plant-taking
    `kalman(…, disturbance=, noise=)` reading `psd` (RN-4), `trajectory_kalman` +
    `TrajectoryObserver` and `kalman_finite_horizon` (estimation follow-ups), the EKF, the
    `EstimationProblem` / `EstimationSolution` / estimator tools (v0.9 estimation follow-ups,
    §5 of this plan as their design note); ROADMAP §3 estimation TRL 1 -> 6, control row
    names the model-taking verbs; ROADMAP §6: "compensator helper or recipe" closed as the
    recipe + `lqg()`, the role-name question added if (b) is not taken now; the 2026-10-13
    decision record gains the rulings of this plan (nominal state, model-taking verbs,
    `StateObserver`, both arities, `Q` n×n, `lqg`, Layer 3 shape); gro501 plan P3/P4 marked
    landed; `pyro-port-remaining.md` rows `StateObserver`, `ObservedSystem`, `kalman(G=)`;
    automation plan §10 note stays for the EKF review. The GRO501 from-scratch notebooks
    (`numpy_state_space.ipynb`, `cartpole_dynamic_controller.ipynb`) stay as they are, by
    ruling: their hand-written Riccati solve and `LQG_Controller` are the lesson; the
    library's own evidence is the demos of C1 (P11 decides what the APP notebooks show).

**Not in this plan**: `closed_loop(estimator=)` (AC-4); `trajectory_kalman`, the EKF, UKF,
particle and discrete filters; `EstimationProblem` and the estimator tools; `LQGPlanner`;
`sys.xbar` as a property (the nominal state is reached as `state.nominal_value`).

## 7. Stress test: the slots the later estimators take

| Estimator | Shape | Ports / state | Entry | Verdict |
| --- | --- | --- | --- | --- |
| Time-varying Kalman (continuous) | `TimeVaryingStateObserver`, `L(t)` from the forward Riccati schedule | `u`, `y` -> `x`; x̂ only | `kalman_finite_horizon(model, Q_n, R_n, P0, tf)` | holds: the dual of `lqr_finite_horizon` line for line |
| Linearized Kalman along a trajectory | `TrajectoryObserver`: schedules `t, x_d, u_d, y_d, A, B, C, L` in `params`; `x̂ = x_d(t) + δx̂` | `u`, `y` -> `x` | `trajectory_kalman(sys, traj, Q_n, R_n, P0)` | holds: the dual of `trajectory_lqr`; needs `riccati_step` forward and `interpolate_schedule` in `core` (A1, A2) |
| Extended Kalman filter | `ExtendedKalmanFilter(model, Q_n, R_n, x0, P0)`: state `[x̂; vec(P)]`; `A(x̂) = ∂f/∂x`, `C(x̂) = ∂h/∂x` taken **directly on `model.f` / `model.h`** inside `f` (`jax.jacfwd` when tracing, central differences on NumPy), never through `analysis.jacobian`, which compiles an evaluator and coerces to NumPy per call | `u`, `y` -> `x` (a slice), `P` | the block is the algorithm; later `ExtendedKalmanEstimator(problem)` | holds: `estimate` is already a port function, not `compute_state`; the model-based params rule (DESIGN §4) covers `model.params` |
| Unscented Kalman, discrete Kalman | `StepSystem`s in a hand-built `StepDiagramSystem` (the flow `+` refuses a `StepSystem`, `%` refuses a flow diagram): `computer = StepDiagramSystem(); add_subsystem(ukf, "observer"); add_subsystem(gain, "gain"); connect…; computer % dt @ plant` | `u`, `y` -> `x`, `P`; `k` | `UnscentedKalmanFilter(model, Q_d, R_d, dt)`; `kalman(…, dt=)` or `kalman_discrete` with `c2d` (P6) | strains the sampled seam: `expose_computer_boundary_ports` auto-exposes one block only and `_as_step_diagram` names it `controller` regardless of `id` (AC-8a, S31), and the numbers need P6; P4 keeps `kalman` continuous in name |
| Particle filter | `StepSystem(N·n + N)`, counter-based draws from `(seed, k)`, `is_random`, `realize(key)` | `u`, `y` -> `x` (weighted mean) | `ParticleFilter(model, process_noise: Distribution, measurement_noise: Distribution, n_particles, seed)` | holds the port convention; needs RN-2 (`sample(key)`) and the `realize` tree |
| Moving-horizon estimator | a trajopt over a window of past data | `u`, `y` -> `x` | `MovingHorizonEstimator(problem)` (Layer 3) | holds: the dual of MPC; needs `EstimationProblem.data` |
| Parameter estimators (RLS, gradient laws) | `u`, `y` -> `theta` | not `x` | `identification/` online, or a new role | the role means a *state* estimate; decide the noun when it lands |

**What P4 does so these fit**: `estimate` as an explicit port function; `kalman` continuous
and steady-state in name; `interpolate_schedule` and the Riccati machinery in shared homes,
the backward step on the dual pair being the forward filter step (no direction parameter);
the role name `observer` settled before the first observer shipped; `require_command` in the
resolver so no reader takes an estimate for a command. **Noted for AC-4**: once a plant
exposes `x` and an observer gives `x` in one diagram, `autowire` finds two candidates for any
`x` input and `expose("x")` refuses; the `observer=` keyword wires by role, not by name.

## 7b. Ask-first items, by name (AGENTS "Ask first")

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

## 8. Verification

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

## 9. Waves and open items

Eleven commits over core, analysis, control, planning, a new package, five notebooks and the
docs is more than one wave. Four waves, each green on its own:

1. **A0 + A1 + A2** (behaviour-preserving, baseline-checked; no public name moves).
2. **A3**, one commit, after the names are signed off: the twins deleted, the modules folded,
   the five notebooks, README, ROADMAP §4 rows, DESIGN lines, `control.rst`, nine tests.
3. **The role record + B1 + B2 + B3** (additive; the guard with B1).
4. **B4 + B5 + C1 + C2** (`lqg` with its facade page and the demo).

Open for the maintainer:

1. The role name: `estimator` (a) or `observer` (b), §4. Recommended (b), ruled before wave 3
   with its own dated record. Under (b) the tool `Estimator` and the retired role differ
   only in case in prose; under (a) `*Design` tools share a word with `design.py`; name the
   collision accepted.
2. `analysis/riccati.py` as the shared home of the Riccati machinery (A2), or keep it in
   `control/design.py` and let estimation import control for it. Recommended: the new module.
