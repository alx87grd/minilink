# The textbook pass's open questions and recorded bugs: the rulings (2026-10-17)

**Date:** 2026-10-17 (six rounds of questions, after PR #125 merged)
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** The second textbook pass ([record](2026-10-16-textbook-pass.md)) changed no
behaviour, so it left about 40 questions and 52 recorded bugs. Each bug was re-checked on `dev`
(60f54ef): 46 reproduced and two did not (`generalized_force` forwards `params=None` to a `B`
that resolves it; `FixedGridOptions(n_steps=1.5)` is refused). This file records what was ruled,
one entry each, and where each ruling lives.

---

## Rulings

- **R1 The equation-path style lives in RULES.** The pattern the maintainer set before the pass
  (the params line, then `xp`, then one parameter per line, the pre-computation, the core
  equations, the assembly of a complex array, the return, each block behind a blank line), the
  banners, the verb families and the lazy-import reasons move from the pass's record into RULES
  §5 and §7.8. Lives in: RULES 5.2, 5.3, 5.8, 5.10, 5.22, 5.25–5.27, 7.8.
- **R2 The MPC blocks take plain names and one constructor.** `ModelPredictiveControllerMixin`
  gets an `__init__` holding the state both concrete blocks duplicated, and its hooks and
  attributes lose the underscore (`compute_x_ff`, `measurement`, `replan_k`,
  `z_warm_for_command`, `planner`, `latch`, …). A subclass override is honoured: the ports reach
  the hook through the instance when they are evaluated. The `mpc_v1` example moves in the same
  change. Lives in: `control/mpc/controller.py`.
- **R3 No shadow state in the wiring core, the minimal way.** `init_wiring` loses its dead
  `hasattr` guards, a helper returns the state layout that its callers assign, and
  `add_error_ports` returns the port layout that the constructors assign.
  `ErrorDriven.error_port`, never read, goes. Lives in: `core/wiring.py`, `core/feedback.py`.
- **R4 Every helper name says its role, public names included (RULES 5.25), with no alias.**
  `validate_diagram_params` → `check_diagram_params`; `check_algebraic_loops` splits into a
  check and `evaluation_order_of`; `validate_overlay` → `require_overlay`;
  `reject_unknown_online_params` → `require_online_params`;
  `validate_output_dependencies` → `check_output_dependencies`; the two `validate_x0` → one
  `require_x0`; `observer_of` → `build_observer`; `goal_biased_sample` → `sample_state`; the
  phase-plane helpers → `require_axis`, `require_grid_shape`, `x_ref_of`, `u_of`, ….
- **R5 Unused or half-wired names.** `identity_matrix`, `empty_transform` and
  `wheel_rectangle_pts` go. `Drone2D` draws with its own `width` and `height`.
  `CartPoleWithNoisePort` is named "Cart Pole with noise ports". `aligned()` loses its unread
  argument, and the package version is computed without a public helper name.
- **R6 The nominal state is every cost's default target.** `TimeCost.from_system` targets
  `state.nominal_value`, as `QuadraticCost.from_system` does (no catalog result changes: their
  nominal states are zero). `CallableSet` flattens `z` before `contains_fn`, as every set does.
- **R7 The auto time step keeps its scale, with one owner.** Smooth and discontinuous systems
  both auto-step at 0.1 τ_min. DESIGN no longer promises a finer step, and the three copies of
  the constant become one.
- **R8 `TwoMass` and `ThreeMass` keep their absolute (sky-hook) dampers**, and their docstrings
  say so.
- **R9 Refusals that agree with their siblings.** `Saturation` refuses crossed limits, as
  `RateLimiter` does. RK4 refuses a non-uniform grid, as fixed-step Euler does. `sensitivity()`
  takes the error-form controller and says so, and a reference-form controller meets a refusal
  that names the fix.
- **R10 Planner defaults that agree.** The tabular planner refuses `grid=` beside levels, as
  value iteration does. `RRTStarPlanner` has `RRTPlanner`'s constructor defaults. There is one
  "cyipopt missing" message, with its install hint. `nominal()` documents what certainty
  equivalence drops, and the planner warning says what a deterministic planner does with a
  stochastic problem.
- **R11 Messages are timeless too.** Refusal and warning messages never name the design process
  ("pipeline B", "for now", "currently", a ROADMAP section); RULES 5.24 now says so. Lives in:
  RULES 5.24.
- **R12 Reinforcement learning.** `predict(x, deterministic=False)` draws from the policy. The
  latch of a reused algorithm on its first discount is left as it is (recorded, not ruled).
- **R13 A value function is read with the interpolation it was swept with.** `value_at` follows
  `options.interpolation`, and the DP result's `value_at` takes `method=`, default linear.
- **R14 Bug semantics.**
  - `SE3(R, p)` refuses a scalar translation other than a literal `0`.
  - An empty intersection of boxes is refused ("the set is empty"), because `None` means
    unbounded.
  - The settling band is 2% of the peak deviation `max|y − y_final|`, so it is defined when the
    response settles at zero.
  - `PolylinePath.sample` and `tangent` take a vector of arc lengths.
  - A Monte Carlo episode shorter than half a control period is refused.
  - An explicit `initial_guess` bypasses the MPC tick latch.
  - `slip2force` keeps its name and documents that it returns μ.
- **R15 One owner for logic written twice (RULES 7.8).** Identical copies merge. Copies that
  differ align to the stricter twin: the forced-input coercion refuses NaN in both simulators;
  `ensure_jax_x64` gives `require_jax`'s install hint; the two JAX-backend checks become one.
  `GeneralizedMechanicalSystem` stays beside `MechanicalSystem` (recorded).
- **R16 The project code (`examples/projects/`).** These fixes were approved:
  - the vehicle physics: the front-wheel rolling sign, Pacejka's combined-slip κ, `f` mutating
    its input, and the friction circle on arrays;
  - the path helpers, including `rounded_rectangle_path`'s closed flag;
  - JAX-traceable allocation and MPC reference blocks, and `ConstantSpeedKinematicCar.tf`
    reading its params;
  - the `car_trajopt` notebook's import.
- **R17 Delivery.** One PR into `dev`, one commit per bug or ruling, each with its regression
  test.
