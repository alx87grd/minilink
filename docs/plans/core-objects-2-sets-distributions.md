# Core objects, phase 2: sets and distributions spoken everywhere

Status: applied 2026-09-15 (uncommitted). Every step byte-identical to the seeded baseline
except the two intended changes of 2.5 (collocation on an intersection set carries finite
bounds; an intersection set is gridable on the JAX precompute). Verified: full `pytest` 1180
passed / 2 skipped, ruff clean, the five affected demos, notebook smoke 26/27 (the one
failure, `gymnasium_interface`, is a pre-existing `stable_baselines3` import abort in this
environment, identical at the last commit), regression gates all pass.
Findings (F), comments (C) and rulings (Q): docs/reviews/2026-09-15-foundations-review.md.
**[ask]** marks core API, public shapes and file moves.

Steps sized for about one agent-hour; **[ask]** marks public API / file moves. Baseline first
(AGENTS recipe): a scratchpad script capturing, seeded, to JSON — every library set's `margin`
on 50 random points on both backends; each `sample` for one seed; `rrt_car_parking` node count
and path length; the pendulum DP `J` table; `MonteCarloEvaluator` on the pendulum stochastic
problem, three backends; 2 000 RL steps of `pendulum_swing_up_rl`. Run twice, `cmp`.

- [x] **2.1 `core/sets.py` textbook pass.** Bare signatures on `margin` / `residual`; unpack
  before math in `BoxSet`, `SingletonSet`, `BallSet`; one step comment on the singleton margin;
  frozen dataclasses for `CallableSet` and `IntersectionSet` (C6); `IntersectionSet` flattens
  (`a & b & c` one level, as `SumCost.of`); docstring to three lines. Tests: `test_backends.py`
  (nested `&` flattens; margins byte-identical). Done when the baseline `cmp`s.
- [x] **2.2 The `box` property and the input twin** (ruled Q1). `VectorSignal.box` in
  `core/signals.py` returns `BoxSet(lower_bound, upper_bound)` on each read (`signals.py` imports
  `core.sets` lazily inside the property to keep `sets.py` free of a port dependency);
  `BoxSet.from_system_state(sys)` returns `sys.state.box`; new
  `BoxInputSet.from_system_inputs(sys)` concatenates the input ports' boxes and replaces
  `PlanningProblem._default_input_set`. Tool reads switch from the array pairs to the property:
  `lyapunov.state_box`, `discretizer._state_bounds`, `environment.allowed_box` (until 3.1
  removes it), `neural.py:116-119`, `port_map.py`, `phase_plane.py`, `gymnasium._state_bounds` /
  `_input_bounds`. Plant constructors that *declare* bounds are untouched (they are the
  student's line). Test in `test_core.py`: the property tracks a later `upper_bound`
  assignment. Done when the baseline `cmp`s.
- [x] **2.3 Distributions move to `core/distributions.py`** **[ask — file move].** RULES 3.4: every
  call site updated, no shim (`problems.py:20`, the `evaluation.py` docstring,
  `test_planning_stochastic.py`, `test_rl_planner.py`); `minilink.planning._EXPORTS` keeps the
  four names (S46 later promotes them to the root); `minilink.core._EXPORTS` gains them. Fold
  `split_keys`, `is_jax_key`, `generator` into the module's internal machinery, the JAX check
  through `array_module`. Done when `test_teaching_surface.py`, `test_teaching_imports.py` and
  both stochastic test files pass and no `planning.distributions` string remains outside
  `docs/reviews/`.
- [x] **2.4 One draw convention** **[ask — `BoxSet.sample` shape; `mean` attribute].**
  `Set.sample(key, n=None, params=None)` / `InputSet.sample(...)` follow the distribution:
  `(dim,)` for one draw, `(n, dim)` with `n`; `key` a `Generator`, an int or a JAX key;
  `BoxSet.sample` traces. `Uniform(lower, upper)` keeps its constructor, holds `box =
  BoxSet(lower, upper)` as `support`, delegates `mean` and `sample`; `lb/ub` → `lower/upper`.
  `mean` an attribute on every distribution (`problems.py:367, 432`, tests). Update the four
  `[0]` sites (`rrt.py:392, 399`, `extenders.py:70`, `tabular.py:326`). Done when the RRT and
  tabular baselines `cmp` and the distribution tests pass with the new shapes.
- [x] **2.5 `Set.bounding_box()` / `InputSet.bounding_box()`** **[ask — core API].** `BoxSet` →
  self; `BallSet` → its box; `SingletonSet` → the degenerate box; `IntersectionSet` → the
  intersection of members' boxes (`None` if none has one); `CallableSet`, `FieldSet` → `None`.
  Consumers replace their probes: `direct_collocation.decision_bounds` and `shooting` (the box
  part of any `X` lowers to bounds, the rest stays margins), `discretizer._state_bounds` /
  `_input_bounds` / the JAX precompute guard (`bounds & free` becomes gridable; validity still
  from `X.contains`), `rrt._sample_box`, `lyapunov.state_box` / `as_box`,
  `environment.allowed_box`, `search/plotting.py:396`. The two `SingletonSet` lowerings stay
  with a comment naming the boundary. **Ratchet test** (C7) in `test_planning.py`: an AST walk
  asserting `isinstance(..., BoxSet | BoxInputSet)` appears only in `core/sets.py` and the two
  transcription lowering methods. Done when the baseline `cmp`s and a new test shows
  collocation bounds on `bounds & FieldSet` equal those on `bounds` alone.
- [x] **2.6 `StateSpaceGrid.X`, `grid.U`** (the `BoxSet` / `BoxInputSet` the grid was built on);
  `TabularLearningPlanner.grid_box` → `grid.X`; `LookupTableController` ports from `grid.X`.
  Done when `test_rl_tabular.py` and the DP baseline hold.

## Sequencing across the phases

| When | Steps | Why |
| --- | --- | --- |
| Now | Phase 1 | docs only; unblocks the rest |
| During the term | 2.1, 2.2, 2.5, 2.6, 3.1, 3.2, 3.3 | name-preserving for every GRO860 notebook; 3.1 closes the live gap first |
| During the term, with the ruling | 2.3 (move), 2.4 (draw convention, `mean`) | `minilink.planning` re-exports hold; no notebook calls `Set.sample` or `Distribution.mean()` |
| After Phase 2, with the Lyapunov rulings D1–D5 | Phase 4 (the field object) | three consumers exist today; name-preserving (`V(x)`, `value_at`, `cost_to_go` keep working) |
| After Phase 1, alongside Phases 2–4 | 6.1, 6.2, then 6.3 file by file | each native plot removes hand-rolled code from several demos; the sweep follows the natives |
| With `estimation/` P4 (v0.2) | 5.1, the F9 decision | the consumer |
| When a consumer appears | 5.2, 5.3, 5.4 | RULES 6.3: no code without a use |

## Verification (Phases 2–4)

Phase 4 adds to the list below: `tests/unittest/test_analysis_lyapunov.py`,
`tests/unittest/test_planning_solution.py`, the demo `analysis_region_of_attraction.py`, and the
two notebooks `double_integrator_policy_evaluation.ipynb` and
`cost_to_go_function_approximation.ipynb` through the notebook checks; the Phase 4 baseline
(RoA level and `verify` report, DP table, fitted weights) `cmp`s after every step.

1. `ruff check . && ruff format --check .`
2. Targeted: `pytest tests/unittest/test_backends.py tests/unittest/test_planning.py
   tests/unittest/test_planning_stochastic.py tests/unittest/test_jax_planning.py
   tests/unittest/test_rl_planner.py tests/unittest/test_rl_tabular.py
   tests/unittest/test_analysis_lyapunov.py tests/unittest/test_interfaces_gymnasium.py
   tests/unittest/test_teaching_surface.py tests/unittest/test_teaching_imports.py`
3. Seeded baseline `cmp` after every Phase 2 step; 3.1 asserts the box-problem baseline
   byte-identical and the non-box problem changed as its test states.
4. Full `pytest` before handoff; demos `rrt_car_parking.py`, `trajopt_holonomic_corridor.py`,
   `vi_pendulum_lqr.py`, `pendulum_swing_up_rl.py` (short `TRAINING_TIMESTEPS`),
   `analysis_region_of_attraction.py`; `MPLBACKEND=Agg python
   tests/demo_checks/run_notebook_checks.py` after 2.3 / 2.4.
5. Once after 2.5 (collocation bounds change for intersection sets): `PYTHONPATH=. python
   benchmarks/run_regression_check.py --suite all --tiny --factor 10 --speed-gate-suffixes
   solve_s,nlp_s,speedup`.
