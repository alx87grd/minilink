# Evening session log — 2026-09-05 (autonomous, branch `dev-fable`)

Mandate from the maintainer: implement everything in the plan that is
straightforward, commit in small steps, no architecture or user-API changes,
postpone any design fork for a joint session. Plan of record:
[ROADMAP.md](../../ROADMAP.md); step specs: [docs/plans/TODO.md](../plans/TODO.md).

## What landed (32 commits, `5c41f83..HEAD`, incl. this log)

**Phase D — docs as plan of record (all seven).** ROADMAP rewritten
(milestones, two lanes, TRL ledger with lane column, GRO860 checklist,
phases, review queue, consolidation principle); DESIGN (two-lane stability,
wheel scope, float64 policy, unconnected-input contract, control-block
decision record); AGENTS (student-facing import rule, demo-header rule,
consolidate-never-strip, delegation split); TODO re-based; two self-marked
complete plan docs deleted and the parity audit shrunk 49 → 36 KB with a
landed-name map; README (custom-plant example composes, two-lane table);
examples README labels the research lane.

**Phase 0 — first-hour safety (all approved steps).**

| Step | Commit | Effect a student sees |
| --- | --- | --- |
| S40 | `2bc97de` | `DynamicSystem(output_dim=n)` outputs `y = x` by default (pyro convention); a textbook plant composes with `controller @ plant` |
| S05 | `77acd2e` | JAX runs float64 by default — the canonical trajopt problems succeed without `configure_jax` |
| S01 | `08080e5` | default grid is 1 001 points with the solver chosen first; JAX default is `scipy` (200 RHS evals, was 400 000) |
| S07 | `1b69825` | one `verbose` flag on planner / optimizer / MPC (panels unchanged) |
| S08 | `30d6df7` | `nbstripout` hook matches notebooks again |
| S04 | `dae31d3`, `b2e3727` | missing `'y'` port names the fix instead of "dim None" |
| S06 | `4fc6e98` | forgotten `super().__init__()` gives a named error |
| S42 | `9a56f0e` | multiple shooting refuses the parametric build instead of silently building collocation defects |
| S02 | `ffb0a4a` | wrong-shape `f` / `h` fail at `compile()` on both backends with the block, hook, and shapes named |
| S09 | `e568098` | trajopt `success` = the plan satisfies the constraints to `feasibility_tol`; violations recorded on the metadata |

**Phase 1 — teaching contract and the GRO860 path.**

| Step | Commit | What it is |
| --- | --- | --- |
| S18–S20 | `90f6fbf` | wheel excludes `symbolic/`, `dynamics/engines/`, `c_export`; `c_export` demos in the JAX regression job; nightly demo + notebook workflow |
| S21 | `51544f2` | branch-cleanup script for you to run ([2026-09-05-branch-cleanup.md](2026-09-05-branch-cleanup.md)) |
| S41 | `de2ccfb` | ranked consolidation inventory ([2026-09-05-consolidation-inventory.md](2026-09-05-consolidation-inventory.md)) |
| S13 | `699b497` + follow-up | `minilink.planning`, `minilink.core`, `minilink.optimization` band facades (lazy `_EXPORTS`, additive) |
| S11 | `fb6222b` | teaching-surface registry test (resolves, docstring, teaching lane) + Basic-tier clean-environment smoke (sim / plots / animate / linearize / LQR / VI with every optional package blocked) |
| S12 | `b7e6760` | import-layer test over `examples/learn` + `examples/demos` with a per-file allowlist that can only shrink |
| S14, S15 | `d6a547c`, `a0a2215`, `9e6bc3a` + follow-up | every student-facing file imports through the teaching surface; allowlist 339 → ~100 rows, all research-lane names (`core.backends.configure_jax` — now redundant after S05 — spatial scenes, geometry, JAX vehicle ladder, extenders) |
| — | `acc299e` | `plot_diagram()` warns instead of raising when the Graphviz binary is missing (bare Colab) |

## Verification at the end of the session

Full unit suite, catalog checks (49/49), demo sweep (60 pass / 3 interactive
skips), notebook smoke (15/15 with `dot` on `PATH`). Regression gates ran green
after S01 (the `Simulator` change).

**Final state of `dev-fable`:** `ruff` clean · `pytest` **915 passed, 2 skipped**
(was 884 on `dev-alex`) · catalog checks 49/49 · demo sweep **60 passed, 0 failed**
(3 interactive skips) · notebook smoke **15/15**.

## Postponed on purpose — need you

| Item | Why it waits |
| --- | --- |
| S16 `_jit` aliases, S17 28 unreferenced evaluator methods | deletions of public attribute names — your pick (inventory rows 2–3) |
| S33 `Sys2Gym` compiled step | value is unclear without vectorized envs (the SB3 loop, not `f`, dominates); needs a measurement first |
| S38 DP metadata (`final_time` reads `problem.tf`, honest `success`) | changes result semantics of a GRO860 tool |
| S39 demo headers → one-line | a dry run showed the headers mix run instructions (drop) with *lesson* prose (move next to the code); editorial, one joint pass |
| S22–S26 catalog `xp` sweep, twins, four-rung ladder, `rollout_batch` | catalog names and evaluator API — Phase 2, with you |
| `configure_jax(enable_x64=True)` calls in 34 demos/tests | now no-ops after S05; removing them is a demo edit — trivial once you say so |
| `examples/learn/teaching/mpc.ipynb` | your uncommitted work; only the `step_disp → verbose` kwarg was renamed in the working tree, never staged |

## Gotchas recorded for future sessions

`dot` needs the env's `bin` on `PATH` (the one "failing" notebook was that);
zsh needs `${=var}` splitting and `set -o pipefail` for gated chains; the
project's `addopts` already carries `-q`.

## Addendum — textbook pass on the student-facing API (same day, with the maintainer)

Measured first: 63 demos, 3 504 code lines, of which 293 import lines,
193 `np.array([...])`, 56 bound assignments, 101 `.params[...] =`. Four API
proposals were put to the maintainer; **A** (scalar/list bounds and `x0`) and
**B** (scalar `Q`/`R`/`S` in `QuadraticCost.from_system`) were **declined** —
bounds and cost matrices stay explicit arrays. **C** and **D** landed:

| Commit | Change |
| --- | --- |
| `3c3ce90` | `DynamicProgrammingPlanner(problem, x_grid=, u_grid=, dt=, ...)` builds its own grid (one object); `grid=` stays for custom grids |
| `7296cb6` | the root prelude **is the teaching surface** — one `from minilink import ...` line; catalog delegated to `minilink.catalog`; tested as a set |
| `7f13f79` | 80 student-facing files rewritten to one root import statement |
| `6defb17` | no-op `configure_jax` lines out of the `.py` demos; trajopt demo uses `plot_solution()` / `animate_solution()` and loses two unused flags; VI demos and the three VI notebooks use the one-object DP setup; the VI-vs-LQR(-vs-PPO) notebooks wire their loops with `controller @ plant` |
| follow-ups | allowlist regenerated (93 rows, all research-lane); merged root imports collapsed to one physical line where they fit |

Corpus effect: import lines **329 → 228**, `configure_jax` calls **7 → 0**.
Verification after the pass: `pytest` 919 passed / 2 skipped, demo sweep
60/60, notebook smoke 15/15.

### Textbook pass 2 (the five follow-ups, approved together)

| Commit | Change |
| --- | --- |
| `c623a87` | `DynamicProgrammingOptions.clean_infeasible` (default on) runs the cleanup after every solve; `TrajectoryOptimizationPlanner(live_plot=True)` builds the matplotlib live callback itself; `Pendulum.camera_scale` defaults to `2 × length` |
| `ab383f3` | demos/notebooks: explicit `S=zeros` / `ubar=zeros` (the `from_system` defaults) removed; `live_plot=LIVE_PLOT` replaces eight lines of callback setup; `clean_infeasible_set()` lines, `x0.copy()`, redundant `diagram.name` and camera hints gone from the VI material; `applied_u()` replaced by `reconstruct_internal_signals` (`ctl:u`) |
| follow-up | allowlist shrunk again (the live-plot deep import is gone) |

Result on the two GRO860 demos: `vi_pendulum_swingup.py` 48 → 40 non-comment
lines, `trajopt_cartpole_collocation_jax.py` 58 → 40 — with the bounds and
cost matrices deliberately left explicit (A/B declined).

## Review pass — 2026-09-06 (branch review + pre-merge punch list)

Ten-angle review of `dev-alex..dev-fable` (seven finder passes ran; the
line-by-line, removed-behaviour, and cross-file angles were redone inline after
a rate-limit stop), merged with the maintainer's punch list
([2026-09-06-dev-fable-pre-merge.md](2026-09-06-dev-fable-pre-merge.md)).
Fifteen findings reported; all fixed in one pass:

| Fix | Where |
| --- | --- |
| trajopt `success` **is** feasibility (solver flag stays in `message`/`stats`); report reads the stored metadata; `live_plot` composes with a user callback; iterates reuse one dynamics callable | `planner.py` |
| `Transcription` docstring restored (was displaced by `supports_parametric`) | `transcription.py` |
| every JAX entry point gets `jax` through `ensure_jax_x64()` — DP / discretizer no longer force x64 (`MINILINK_JAX_X64=0` honoured), collocation / shooting knot grids are float64 in a fresh process | `dp.py`, `discretizer.py`, `direct_collocation.py`, `multiple_shooting.py`, `shooting.py`, `transcription.py` |
| shape probe: wrong-length `x0` raises instead of zeros; a bare scalar is accepted for `n = 1` (dev-alex ran it); step diagrams are probed too | `compiler.py`, `step_compiler.py` |
| `System.__getattr__`: sentinel (`inputs` missing ⇒ `super().__init__` message for any name, incl. `p`/`m`/`h`), and a failing property is re-run so the real missing name shows | `system.py` |
| `FIXED_STEP_SOLVERS` derived from the solver table; the automatic-grid fallback lives once in `build_time_grid` | `simulator.py`, `time_grid.py`, `static_simulator.py` |
| root prelude: one `_EXPORTS` table (catalog names merged in) | `minilink/__init__.py` |
| nightly installs `coinor-libipopt-dev` + the `ipopt` extra (trajopt flagship uses Ipopt) | `nightly.yml` |
| stale `disp=True` docstring, test name, `optional` markers, `__main__` guard, DP opt-out test exercises the wiring; catalog full-mode helper called `Simulator.run()` (dead since the `solve()` rename — pre-existing) | tests |
| DESIGN: success wording, DP one-object setup + `clean_infeasible`, compile-time probe contract; README trajopt block on the one-line import; examples README lists `grid_world_exact_dp`; tests README stale link; `STEP_DISP` → `VERBOSE` in the two MPC demos | docs, demos |

Verification: `pytest` 929 passed / 2 skipped, catalog 49/49 (fast **and**
full mode), demo sweep 60/60, notebook smoke 15/15.

## Organisation pass — 2026-09-06 (rulings applied)

Rulings: online MPC ticks keep the solver flag; band names the demos use are
promoted to the root, option objects stay off the root and out of demos;
`configure_jax` stripped from the notebooks; DP history frames left as they
are; one facade helper; `minilink/experimental/`; the demo regroup; every
dedupe item; one-line headers ("super minimalist"). Proposal:
[2026-09-06-examples-organisation-proposal.md](2026-09-06-examples-organisation-proposal.md).

| Commit | Change |
| --- | --- |
| `experimental:` | `minilink/experimental/{symbolic,engines,c_export}` — the import path states the maturity; wheel exclude is one glob; duplicated UR5 EoM project notebook removed |
| `facades:` | `minilink/core/facade.py` (`lazy_facade`) replaces nine copies; root gains `TimeCost`, `StepDiagramSystem`, `BallSet`, `closed_loop_qdq`, `ZOHHold`, `Source`, `ImpedanceIntegralController`, `discretize`, `RRTStarPlanner` (`discretize` stays off the analysis band: it shadows its own module); 15 demos on the root line; `DynamicProgrammingOptions` out of the student path |
| `examples: regroup` | demo folders 19 → 13 keyed to the intro chapters (`core`, `blocks`, `dynamics`, `control`, `analysis`, `hybrid`, `compile`, `optimization`, `planning/{rrt,value_iteration,trajopt}`, `graphical`, `realtime`, `robotic`, `mpc`); `sandbox/` → `experimental/` (+ `c_export/`); two animation demos merged; internal-signals demo on the shortcut loop; sampled-SMC demo without its dead block; `readme_examples.py`; `projects/pathtracking/common/` (three identical `vehicle.py` → one); scratch rollout-gradients notebook → `learn/teaching/` (smoked); spatial-MPC notebook → `projects/mpc/`; empty `tooling/scripts/` gone; the `run_all_demos` special case for `game_bicycle` gone |
| `notebooks:` | `configure_jax` out of every `learn/` notebook (and the moved MPC one); `06_hybrid` rewritten as a real chapter (`StepSystem` → `StepDiagramSystem` → `block % dt` → `computer @ plant`); rollout-gradients twin on root imports |
| `headers:` | S39 — one-line docstrings on all 68 demo / experimental scripts; key maps kept as comments in the games; flag explanations next to the constants |

Kept as two lessons: `pendulum_swing_up_vi_vs_lqr` and `…_vs_lqr_vs_ppo` share
4 of 25 cells — not duplicates. Kept in `experimental/robotic/`: the two UR5
impedance scripts (JAX + meshcat, not headless).

Verification: demo sweep 57 passed / 3 interactive skips (60 scripts under
`demos/`), notebook smoke 15/15 (incl. the new `06_hybrid` and the promoted
`cartpole_rollout_gradients`), the three `projects/pathtracking` runs from
`common/`, `pytest` and catalog checks green.

## S33 — compiled `Sys2Gym` step (2026-09-06)

Ruled: keep the evaluator methods (S16/S17 closed as keep); land S33.
`Sys2Gym(sys, cost, ..., integrator="rk4", compile_backend=None)` compiles the
plant once and steps it with one call per env step — `jax.jit` of the
evaluator's RK4 step when JAX is installed and the plant traces, the NumPy
evaluator otherwise (`Drone2D` and the other NumPy-only catalog plants fall
back silently; `compile_backend` records the choice). `integrator="euler"`
reproduces the historical `x + f dt`. Per step on this machine: pendulum
9.6 µs (jitted RK4) vs 7 µs (Python Euler); UR5 56 µs vs 263 µs. Parity tests
cover RK4 vs the explicit formula, RK4 vs Euler at small `dt`, JAX vs NumPy,
and the fallback. The two PPO notebooks were not re-trained here:
stable-baselines3 is not in the dev env.

## Phase 2 — the JAX claim (2026-09-06, ruled "full phase 2")

| Step | Change |
| --- | --- |
| S38 | DP `success` = converged to `tol` (`solve_steps` always completes); sweep count and last delta in `message` / `stats`; `final_time` reads `problem.tf` |
| S23 | `tests/unittest/test_catalog_backends.py`: every `minilink.catalog` plant compiles on NumPy and JAX and agrees on random points; strict-xfail `NUMPY_ONLY` list, now empty |
| S22 | thirteen modules swept to `xp = array_module(...)` (oscillators, mountain car, suspension, rocket, three-body, steering, drone, propulsion, noise-port pendulum, cart-poles, boat, arms, dynamic bicycle); in-place writes became `concatenate` / `column_stack` / `where` |
| S24 | `JaxCartPole` retired — `CartPole` traces; every user says `CartPole` |
| S26 | `rollout_batch(x0s, u_sequences=None, *, t0, dt, n_steps, params)` on the JAX evaluator: one `vmap` of the RK4/ZOH rollout over families of initial states, inputs, and params (a leaf with one extra dimension is swept); demo `demos/compile/rollout_param_family.py` (pendulum lengths, Buckingham-π collapse) |
| S25 | catalog ladder `HolonomicMobileRobot` → `KinematicBicycle` / `KinematicCar` → `DynamicBicycle` → `BicycleDynRate` (now in `dynamic_bicycle.py`, numerically the JAX rate variant to 1e-15, dual-backend, on the root prelude); `named_ports=` replaces the six `*Ports` twins; `Holonomic`, `HolonomicAccel`, `BicycleKin`, `BicycleAcc`, the torque / servo / engine rungs, the four extra variants and `CarProfile` live in `examples/projects/car_trajopt/vehicles/`; `jax_vehicles.py` is gone |
| — | `Sys2Gym` steps on the compiled plant (S33, earlier the same day); S16/S17 closed as keep |

Verification after each step: full `pytest`, catalog checks (fast + full),
`test_catalog_backends` (51 cases), demo sweep, notebook smoke, regression
gates; a parity probe of the new ladder against the retired `jax_vehicles`
module reported differences of 1e-13 or below on random points.

### Review pass on Phase 2 (2026-09-06, inline — the finder agents hit the session limit)

Twelve findings; four fixed: `rollout_batch` caches one `jit(vmap)` per input
/ family layout (repeat calls 65 ms → 4 ms on the seven-length pendulum
sweep); `Sys2Gym` falls back to NumPy only on the "not JAX-traceable" verdict
and re-raises any other JAX compile error; the catalog-check registry gains
the five plants it never covered (`DoublePendulum`, `Plane3D`,
`PendulumWithNoisePort`, `DynamicBicycle`, `DynamicBicycleCar3D`) plus a test
that it covers every `minilink.catalog` name; the 3-D collision tests use a
three-line test plant instead of importing the research-lane `extras`.
Recorded, not changed: NumPy `f` of the swept `DynamicBicycle` costs 15 µs
(was 10) from the `xp.array` / `where` forms; `named_ports` exists in three
shapes (two inline, one project mixin); `BicycleDynRate` rebuilds its state
and ports after the base constructor; the family-axis heuristic for params
leaves without a compiled twin; `test_dynamics_catalog` still imports the
research rungs it tests. Note: the S22 / S23 / S24 / S26 changes landed inside
the S25 commit `9554a48` (their own gated chains stopped on a test failure at
the time).

## Derivatives facade and analysis family (2026-09-06, `c9372a5` + review fixes)

Plan [docs/plans/derivatives-facade.md](../plans/derivatives-facade.md) v4
implemented with its recommended rulings: `evaluator.jacobian(of, wrt)` on
all ten evaluators (eager `jax.jacfwd` on JAX, central differences on NumPy;
`f` / `step` / output ports / diagram wires `"block:port"` against `x` / `u` /
input ports / `t` / `params` / wires), `System.jacobian(...)` returning NumPy
values over a cached compiled evaluator, and one calling pattern
`tool(<what>, x_bar, u_bar, t, params, *, method="auto", eps)` for
`linearize`, `bode`, `pzmap`, `plot_*`, `modal_analysis`, `find_equilibrium`,
`lqr_at_operating_point`, the new `transfer_function`, `discretize(integrator=)`
and `controllability(lti)`; all of them methods on `DynamicSystem`.
`jacobian_f_params` removed; demos, five intro notebooks, README and DESIGN
updated. Verification: unit suite 1022 passed / 2 skipped, demo sweep 58/58,
notebook smoke 5/5 edited notebooks.

Inline review of the commit (diff scan, removed behaviour, callers, pitfalls,
wrapper, cleanup, altitude, AGENTS conventions, sweep) found and fixed:

1. `backends.py`: the `lru_cache` decorator had slid from `require_jax_numpy`
   onto the new `jax_installed` — both now cached.
2. Evaluator cache in a `WeakKeyDictionary` leaked every leaf system (the
   evaluator holds the system, so the weak key never dies); replaced by
   `sys.compiled_evaluators`, initialised in `System.__init__`, tagged with a
   structural signature (ports, dims, subsystems by identity, connections) and
   reset by `__getstate__` so copies and pickles never carry jitted closures.
3. `refresh()` invalidation dropped the cache on every `Simulator` solve, and a
   port added to a block already inside a diagram never invalidated the
   diagram — both covered by the signature; the mutator hooks are gone.
4. `_jac_resolve_of(None)` on static evaluators selected the missing
   evolution; static-block `linearize_matrices` could raise `AttributeError`
   on a wire selector and iterate `None` when a static block had no outputs;
   the old `(sys_id, port_id)` tuple and a list passed as a channel produced
   `int()` errors; `controllability(array)` raised `AttributeError` — all now
   clear `TypeError` / `ValueError` messages.
5. `central_difference` evaluated `g(z)` once more than needed; stale
   "under `jax.jit`" docstring; AGENTS rules: no leading-underscore facade
   methods (`compiled_evaluator`), no hasattr-or-create state, `__main__`
   hello-worlds restored in `linearize.py` / `derivatives.py`.

Left as is, for the maintainer: `transfer_function` imports the
`TransferFunction` block lazily from `blocks/` (a tool returning a library
block, like `lqr_at_operating_point` returns `StateFeedbackController`);
`analysis.linearize` keeps six small selector helpers; eager `jacfwd` costs
0.5–5 ms per Jacobian against 0.03–0.08 ms for finite differences.

Follow-up (2026-09-07): the maintainer questioned the evaluator cache on the
system. Measured with the eager JAX path, a JAX compile costs 1.3–1.6 ms
(NumPy 0.02 ms) against 1.8–5.4 ms for one Jacobian evaluation, so the cache
saved about 1.5 ms per call at the price of an attribute, a method, a
structural-signature heuristic and a pickling hook on every `System`. Removed:
`compiled_evaluators`, `compiled_evaluator`, `structure_signature`,
`__getstate__`; `analysis.derivatives.compiled(sys, method)` compiles per
call and the value tier stores nothing on the system. Suite 1020 passed / 2
skipped, analysis demos 8/8, two notebooks re-smoked.

Streamlining pass on the continuous-time teaching material (2026-09-07):
demos no longer redefine blocks the prelude ships (`diagram_nested_loop.py`
uses `Integrator` / `ProportionalController`), the flat scripts lost their
helper functions (`signal_blocks.py`, `optim_plot.py`) and demo-control flags,
`try`/`except` around optional renderers is gone (`animation_renderers.py`),
sources and controllers are built with their constructor kwargs instead of
three `params[...]` lines where the parameters are not the lesson, redundant
`show=False` / unused variables are dropped, and the notebooks lost their
defensive `getattr` probes (`05_simulation`, `08_optimization`,
`09_planning`). Hybrid / step, compile, planning and mpc demos untouched by
request. Demo sweep 58/58, five notebooks re-smoked, teaching-import test
green after two stale allowlist rows were removed.
