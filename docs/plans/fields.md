# The field object

Status: core contract — agreed 2026-09-15 (FD-C12, time and action arguments included); not started, step 4.7 done 2026-10-12; the field signature `value(x, u=None, t=0.0, params=None)` already on `StateField`.
Where it sits: the core's Fields pillar ([core-architecture.md](core-architecture.md) §9), row A1 of [TODO.md](TODO.md) §1.9. Depends on the sets-and-distributions pass (landed 2026-09-15: `bounding_box`, `.box`, the draw convention); the Lyapunov rulings it cites were open items of ROADMAP §6, now QUESTIONS.md (R111 ruled the certificates a research-lane tool).
**Preliminary choice, 2026-10-27 (R133, [record](../reviews/2026-10-27-design-questions-rulings.md)):** the core builds
steps 4.1–4.3 (a minimal `Field`, `QuadraticField`, the shaping in `core/`); 4.4–4.8 come later with
their consumers; one `as_cost` (`Set.as_cost` reuses it on a set's margin); the conversions are named
`as_set`, `as_input_set`, `as_cost`, so `as_constraint` below reads `as_set` once it lands.
**Preliminary choice, 2026-10-27 (R153, [record](../reviews/2026-10-27-core-vision-decisions.md)):**
a separate `as_terminal_cost(weight=, shaping=)` beside `as_cost` (the terminal cost `h`, no
`terminal=` keyword); `GridField` (step 4.4) stays with its v0.4 consumers.
Name-preserving (`V(x)`, `value_at`, `cost_to_go` keep working). Finding FD-F12 of the foundations review, docs/reviews/2026-09-15-foundations-review.md (`FD-` marks that review's findings and comments: its F12, C6, C12).

The scalar function on `(x, u, t)` becomes a core noun, `Field`, with the same verbs
everywhere: `value(x, u=None, t=0.0, params=None)` (native-array, bare signature, RULES 5.1 as
amended; a field reads the arguments it depends on and ignores the rest, as `f` ignores `t`),
`as_constraint(lower=, upper=)` → a `Set` over `x`, `as_input_constraint(lower=, upper=)` → an
`InputSet` over `u` at a given `x` (for fields that read `u`), `as_cost(weight=, shaping=)` → a
`CostFunction` (the running cost `g`), `as_terminal_cost(weight=, shaping=)` → the terminal cost
`h` (R153). No new verb for the sublevel set: `V.as_constraint(upper=level)` *is*
`{x : V(x) ≤ level}`. Time enters through `t`; a schedule (`S(t)` from the Riccati sweep, the
per-step DP tables) is held on the concrete field and interpolated in `t`.
Baseline first: the region-of-attraction level, extent and
`verify` report of `analysis_region_of_attraction.py`; the DP `J` table and
`PolicyEvaluator.value_at` on a grid of points; the `LinearApproximator` fitted weights of
`tests/unittest/test_dynamic_programming.py`; run twice, `cmp`.

- [ ] **4.1 Promote the noun** **[ask — file move, rename].** `StateField` → `Field`, with
  `FieldSet` and `FieldCost`, moves from `planning/spatial/state_fields.py` to
  `core/fields.py`. The four spatial fields (`ClearanceField`, `CostDensityField`,
  `PathDistanceField`, `CorridorMarginField`) subclass that ABC in
  `core/geometry/fields.py` — **not** `planning.spatial` ([geometry-module.md](geometry-module.md);
  they are workspace→state maps that a controller eval needs with no planner).
  Shaping (`quadratic_hinge`, …) lives next to `Field.as_cost` and `Field.as_terminal_cost`
  (R153) in `core/fields.py`.
  RULES 3.4: call sites updated (`geometry` fields, `scene.py`, `track.py`, `test_planning.py`,
  DESIGN §6 twice), no shim; `minilink.core._EXPORTS` gains `Field`. New: `FieldInputSet
  (InputSet)` with `margin(u, x, t, params) = value(x, u, t, params) − lower` (and `upper −
  value`), built by `Field.as_input_constraint()`; and `gradient(x, u, t, params)` on `Field`:
  `jax.grad` of `value` under JAX, central differences on NumPy through `analysis.derivatives`
  (the one place finite differences already live). Done when the corridor and RRT demos and
  `test_planning.py` are byte-identical.
- [ ] **4.2 `QuadraticField(center, P)`** — `value = (x − c)ᵀ P (x − c)`, `gradient = 2 P (x − c)`,
  written once with `xp`; frozen dataclass (FD-C6). `P` is a matrix, or a schedule `(t, S)` taken from
  the `(t, K, S)` that `lqr_gain_schedule` returns, interpolated in `t` the way `TimeVaryingFeedback`
  interpolates `K(t)`: then `value(x, t)` is the finite-horizon LQR value `xᵀ S(t) x`, and
  `lqr_finite_horizon` can hand it out beside the controller (additive; the cart-pole LQR
  notebook is its consumer). Its sublevel set is the one set that knows how
  to sample: `QuadraticField.as_constraint(upper=level)` returns an `EllipsoidSet(FieldSet)`
  with `sample` (uniform in the ellipsoid, moved from `lyapunov.sample_in_ellipsoid`) and
  `bounding_box` (`sqrt(level · diag(P⁻¹))`). Tests: value and gradient against NumPy on random
  points, jit parity, `sample` inside the set, `bounding_box` contains every sample.
- [ ] **4.3 `LyapunovCertificate` speaks the field** **[ask — `V` becomes an attribute].**
  `certificate.V` is the `QuadraticField(x_bar, P)`; `StateField.__call__ = value` so the
  existing `certificate.V(x)` calls (`test_analysis_lyapunov.py`, the certificate's own `verify` and
  plot in `analysis/lyapunov.py`) keep working and
  `certificate.V.as_constraint(upper=certificate.level)` reads as math; `region` property
  returns that set; `contains`, `extent`, `slice_extent` and `verify` delegate to it
  (`sample_in_ellipsoid`, `as_box`, `clip_box` go — the window is a `BoxSet`); `V_dot` is `gradient · f`; `search_level` reads `V.value` / `V.gradient`. Public
  names unchanged. Done when the RoA baseline `cmp`s and `test_analysis_lyapunov.py` passes.
- [ ] **4.4 `GridField(grid, values)`** — `value(x)` interpolates a node-indexed table on a
  `StateSpaceGrid` (wraps `grid.interpolate`; the same `fill_value=0` outside the grid, so the
  DP numbers hold). `values` may carry a leading time axis (the per-sweep tables of
  `record_history=True` / `solve_steps`, one per `dt`), and then `value(x, t)` picks the table
  of `t` — the finite-horizon `J(x, t)`; or a trailing action axis (the tabular `Q` table over
  `grid.actions`), and then `value(x, u)` reads `Q(x, u)`, `min_over_u()` returns the `J` field
  and `greedy()` the `LookupPolicy` — the two lines the `TabularLearningPlanner` docstring
  already states. DP's `PlanningSolution.cost_to_go` becomes `GridField(grid, J)` (until then
  it stays today's callable, R153; `sol.cost_to_go(x)` works either way);
  `TabularLearningPlanner`'s result the same; `PolicyEvaluator` holds a `GridField` and
  `value_at(x)` delegates (the name stays: `double_integrator_policy_evaluation.ipynb` uses it);
  `plot_cost2go` / `StateSpaceGrid.plot_value` accept a field or a table. Done when the DP
  baseline `cmp`s and `test_rl_tabular.py`, `test_planning_solution.py` pass.
- [ ] **4.5 The approximator is a field.** `LinearApproximator(StateField)`: `value(x) = w @ φ(x)`,
  `fit` and `sgd_step` update `w` in place (it is a learner, so a plain class, not frozen — FD-C6);
  `QuadraticFeatures.quadratic_form(w)` returns a `QuadraticField` beside the `(c, b, S)`
  triple it returns today (additive). Done when the approximator tests of
  `test_dynamic_programming.py` pass with identical fitted weights.
- [ ] **4.6 `CallableField(fn)`** — the escape hatch, mirror of `CallableSet`; the RL
  planner's `critic_cost_to_go()` returns one (`V_w(x)`, and `Q_w(x, a)` for the SAC family), so
  `PlanningSolution.cost_to_go` is typed `Field | None` on every planner. Done when
  `test_rl_planner.py` passes.
- [x] **4.7 CBF plan amendment** (done 2026-10-12, `cbf-safety-filter.md` §3 and CBF-2): the barrier is a `Field`
  (`scene.clearance_field(body)`), its safe set `h.as_constraint(lower=0)` a `Set`; the barrier
  condition `∇h · f(x, u) + α h(x)` is a second `Field` on `(x, u)` built from `h.gradient` and
  `sys.f`, and its superlevel set `.as_input_constraint(lower=0)` is the `InputSet` `U(x)` the
  safety QP enforces — no `Scene.as_cbf`, no barrier type of its own.
- [ ] **4.8 DESIGN §4** gains a fields bullet (the noun on `(x, u, t)`, `value` / `gradient`, the
  four conversions, schedules in `t`, the ellipsoid as a quadratic field's sublevel set); ROADMAP
  §3 Lyapunov and DP rows note it.

## Verification

The tests and demos each step's "done when" names, plus `tests/unittest/test_analysis_lyapunov.py`, `tests/unittest/test_planning_solution.py`, the demo `analysis_region_of_attraction.py`, the approximator tests of `tests/unittest/test_dynamic_programming.py`, and the notebook `double_integrator_policy_evaluation.ipynb` through the notebook checks; the baseline captured before step 4.1 (RoA level and `verify` report, DP table, fitted weights) `cmp`s after every step.
