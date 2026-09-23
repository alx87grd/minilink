# Improvement scan — consolidated propositions (2026-09-22)

**Status:** proposal for the maintainer. Nothing here is scheduled until a
row lands on [docs/plans/TODO.md](../plans/TODO.md) or a decision on
[ROADMAP §6](../../ROADMAP.md#6-decisions).

**How it was produced.** Ten band scans of the whole library (core,
compile/simulation, dynamics, control, analysis, planning,
graphics/blocks/interfaces, examples, tests/CI, docs governance), one finder
per band, each asked for the fifteen most valuable improvements and every bug
it met. The finders reported 148 suggestions and 46 bugs. The independent
verification pass did not run (session limits), so every item is
single-finder unless the **Reproduced** column says otherwise: those I
re-ran myself on `dev` at `8a7226c`. The full title list is in the
[appendix](#appendix-every-finding-by-band); the finders' full text (what,
why, evidence, reproduction) is in
[2026-09-22-improvement-scan-ledger.md](2026-09-22-improvement-scan-ledger.md).

**Companion:** the same-day
[consolidation review](2026-09-22-consolidation-review.md) (architecture,
state, roadmap, textbook pass).

## 1. Verdict

The library is sound; the scan found no broken equation in a catalog plant
or a control law that the tests cover. What it found, again and again, is
**drift between owners of one rule**: three computations of the automatic
`dt`, three keyword sets for the four feedback wires, two names for the
cost-to-go plot, two default sets for the Monte Carlo score, 56 hand-copied
label lines, four copies of the CI regression command. The second theme is
**traps that fail late or not at all**: a diagram whose input feeds `h` with
`y_dependencies=()` simulates a wrong fixed point silently, `discretize()`
on any closed loop returns an object that cannot step, and the CI merge gate
never executes a JAX-dependent test.

Three findings change what the close-out should contain:

1. **The merge gate is dark on JAX.** `test.yml`'s `test` matrix installs no
   `jax`, and the `regression` job (which does) never calls `pytest`. Every
   `@pytest.mark.jax` test and six whole modules (mid-file `importorskip`)
   skip in CI. A broken JAX backend merges green. This belongs with R2.
2. **`discretize()` is broken on diagrams and lossy on leaves.** The flat
   `dt` key written into the source's params is rejected by the diagram
   validator; on a leaf the wrapper drops `x0`, labels and bounds, so the
   shipped demo rolls out from rest. One small change (dt on the wrapper,
   params passed through, metadata copied) fixes three reported bugs.
3. **Six small confirmed bugs** (§2) are each an afternoon with a test and do
   not need a design decision.

Recommendation: add the §3.1 items to the v0.1 close-out, fold the §3.2
items into the wave rows they name, and rule on the §3.3 decisions. Work in
small commits, one bug or one row per push.

## 2. Bugs

Severity is the finder's. **Reproduced** = re-run by me with the finder's
snippet on `dev`.

### 2.1 Reproduced

| # | Sev. | What | Fix size | Proposed home |
|---|------|------|----------|---------------|
| B1 | high | `discretize(ctl @ plant, dt).step(...)` raises `Unknown subsystem ids in diagram params: 'dt'` (`analysis/discretize.py` merges `dt` into the source params). | S | close-out bug fix; the P6 plan then builds on a working verb |
| B2 | medium | `discretize(plant, dt)` drops `x0`, state labels and input bounds; `analysis_discretize.py` plots a flat zero trajectory. | S (same change as B1) | with B1 |
| B3 | high | CI merge gate runs no JAX test: `test` job has no `jax`, `regression` job never runs `pytest`. | S (workflow) | R2 |
| B4 | medium | Module-level `pytest.importorskip("jax")` mid-file skips the NumPy tests above it (`test_dynamics_catalog.py:533`, `test_mechanical_robotics.py:274`, `test_geometric_control.py:11`, racecar tests). | M (tests only) | R2 |
| B5 | medium | Flagship manifest lists `requires: []` for `mpc_integrator_numpy` and `mpc_car_minimal`, which call `plot_diagram()`; `pytest` fails without the `diagrams` extra. | S | close-out |
| B6 | medium | `dt`-based time grids overshoot `tf` by one sample: `Simulator(Integrator(), tf=1.1, dt=0.1)` gives 13 points ending at 1.2 (`np.arange(t0, tf + dt, dt)`). | S, but changes regression baselines and every `dt=` notebook | decision D-a |
| B7 | medium | `L @ plant` with `L = Gain >> Gain` appends the plant to the user's `L` (`closed_loop` routes through `series`, which extends a diagram left operand in place). | S | close-out bug fix (P10 documents `@`) |
| B8 | medium | `discount_rate` cannot be set on an instance (frozen dataclass) and `2 * cost`, `a + b` reset it to 0, so composite costs are scored undiscounted. | S–M | A2 (cost parameters) |
| B9 | medium | `lqr_gain_schedule` on a stiff pair with a coarse grid returns a wrong schedule, a `LinAlgError`, or NaN (`n_steps=2`); `riccati_step` has the substep guard, `riccati_transition` does not. | S | close-out bug fix |
| B10 | low | `Plane2D.d` / `Plane3D.d` index `u`, so `inverse_dynamics(q, v, a)` and the model-based controllers crash on the base's `u=None` contract. | S | close-out bug fix |
| B11 | low | `q` / `dq` output-port labels read `Angle 0` / `Velocity 0` on every `MechanicalSystem` plant (copied before the plant renames its states). | S now; the real fix is proposition 3.2-C3 | close-out |
| B12 | low | RULES 4.1's canonical example `from minilink.control import lqr` binds the **module**, not the function (shadowing). | S (docs) now; rename in 3.2-B4 | close-out |
| B13 | medium | `examples/demos/trajopt/trajopt_cartpole_collocation_jax.py` hard-codes `optimizer_method="ipopt"`; fails on the pip Full tier and Colab. Every sibling probes `find_spec("cyipopt")`. | S (maintainer's example) | close-out |
| B14 | medium | `docs/plans/pyro-port-remaining.md` marks `ss2tf`, `TrajectoryLQRController` and dedicated PI/PD as TODO; all are landed. | S (docs) | C1 re-audit (3.2-C5) |

### 2.2 Reported, not yet reproduced

Plausible from the cited code; each needs a five-minute check before a fix.

- **`JaxDiagramEvaluator` casts `dx` to the dtype of `x`** (medium): an
  integer `x0` on a diagram truncates derivatives to 0; the leaf evaluator
  is right. `core/compile/evaluators/jax_evaluators.py:921-926`.
- **`QuarterCarOnRoughTerrain` damper term** (medium): `b * (dy - dz/dx)`
  where the road's vertical velocity is `dz/dx * vx`; the default `vx = 1`
  hides it. Physics check for the maintainer (decision D-g).
- **MPC tick latch memoizes by `k` only** (medium): two
  `compute_command(y, k=0)` calls with different `y` return the first plan.
  Simulation is safe only because `HybridSimulator` resets per run.
  `control/mpc/controller.py:630-642`.
- **`score_trajectory` drops `problem.params.sets`** (medium): NumPy and
  simulator Monte Carlo backends disagree with JAX on a parametric
  constraint set. `planning/evaluation.py:30`.
- **`MonteCarloEvaluator` default backend is `jax`** (medium): it cannot
  score a DP or tabular law (`RegularGridInterpolator`), and the error
  blames the block. `planning/evaluation.py:136-139`.
- **DP replaces a callable `infeasible_cost` with 1e6 silently** (medium):
  `options_of` tests `isinstance(..., float)`. `policy_synthesis/dp.py:648`.
- **`Sys2Gym.reset` on a plant without finite state bounds** (medium):
  `OverflowError` or an `inf` observation, also through `from_problem`,
  which has `sample_x0`. `interfaces/gymnasium.py:134-136`.
- Low: `IntersectionSet.margin` on a scalar member margin;
  `DiagramSystem.add_subsystem` accepts a `StepSystem` and `f` returns a
  short vector; `DiscretizedDynamicSystem.step` rejects the plant's own
  params and exposes `dt` as a parameter sensitivity; `step_info` rise time
  on an undershooting response; Bode margins vs `margins()` grid mismatch;
  DP plot verbs raise `AttributeError` before `solve`; animation frame
  schedule never draws the last sample; `animate(save=True,
  file_name='clip.gif')` writes `clip.gif.gif`; Plotly y-labels double the
  unit brackets; `HybridDiagram.plot_trajectory` dead abscissa argument;
  `mpc % dt` bypasses the `dt_mpc` cross-check; `gravity_feedforward` masks
  a hook's own `TypeError`; TwoMass/ThreeMass clamp an out-of-range
  `output_mass`; tutorial 00 §2 prints state labels on the "Diagram" line;
  notebook-check ids collide; `tests/run` factor differs from CI; DESIGN §6
  documents `Distribution.mean()` as a method (it is an attribute); DESIGN §5
  points at a script that does not exist; RULES 7.6 says CI has no link
  checker (it does); `tests/README.md` module census (22 vs 45); AGENTS "CI
  runs exactly three things" (four jobs); `estimation/` and
  `identification/` docstrings point at a retired ROADMAP section.

## 3. Propositions

Ordered by where the work lands. Each line names the finder id (band#n) so
the ledger entry can be found.

### 3.1 Add to the v0.1 close-out (ROADMAP §5.1)

Small, agent-lane unless marked, and each removes a first-hour trap or a
false green.

1. **R2 grows into "the gate runs what it claims"** (tests-ci#0, #1, #2,
   #6): the `regression` job runs `pytest` after installing `jax` (and
   `ipopt`, as `nightly.yml` already does); per-test `jax` markers replace
   the mid-file `importorskip` calls and the `jax` marker skips itself when
   `jax` is absent; the flagship manifest lists every optional import a demo
   makes and a contract test checks it; `ruff` and `ruff format` hooks join
   `.pre-commit-config.yaml`. Fixes B3, B4, B5.
2. **R1 grows into "the tag is tested"** (tests-ci#12, #13; examples#1,
   #4): `publish.yml` runs the suite before uploading and every long job
   gets `timeout-minutes`; `showcase_jax.ipynb` stops importing
   `experimental.c_export`, which the wheel does not ship; `plot_diagram()`
   warns instead of raising when the `graphviz` wrapper is missing
   (maintainer: or put the pure-Python wrapper in the base dependencies);
   one canonical Colab setup cell per tier, pinned by a test (38 notebooks
   carry 11 variants today).
3. **Confirmed small bug fixes**, one commit each with a test: B1+B2
   (`discretize`: dt on the wrapper, params passed through, x0/labels/bounds
   copied); B7 (`closed_loop` copies a diagram left operand); B9 (substep
   guard in `lqr_gain_schedule`); B10 (`Plane2D.d` tolerates `u=None`);
   B11 (relabel `q`/`dq` after the state); B12 (RULES 4.1 example); B13
   (maintainer: the trajopt demo probes `cyipopt` like its siblings).
4. **Docs drift bundle** (docs-gov#4, #5, #7 and the low docs bugs): CI
   commands written once (tests/README) and named jobs in AGENTS; DESIGN's
   inline TODOs moved to the workboard and its retired pointers fixed;
   RULES 3.3's teaching-surface definition aligned with ROADMAP §2; the
   `Distribution.mean` sentence, the RULES 7.6 link-checker sentence, the
   tests/README census, and the `estimation/` docstring pointers.
5. **Registry walk over the band facades** (graphics#2): the teaching-surface
   test walks each band's `__all__` like it walks the root prelude, then
   `NotchFilter`, `Washout`, `MLP` join the blocks row and `blocks.step`,
   `blocks.neural`, `graphical.port_map` join the API pages.

### 3.2 Fold into scheduled v0.2 steps (ROADMAP §5.2)

**Wave A — textbook objects**

- **A1** `PolicyEvaluator` gets the evaluators' verb and a field-shaped
  result (planning#4).
- **A2** `discount_rate` becomes a constructor field (or a `params` entry)
  that `ScaledCost` forwards and `SumCost` reconciles (core#4, fixes B8);
  `validate_diagram_params` checks each per-block dict against the block's
  keys so a partial dict fails at the setter, not inside `f` (core#3).
- **A4** one name, `plot_cost_to_go`, and one colour keyword, `jmax`,
  across DP, tabular RL, `PolicyEvaluator` and `Comparison`; `plot_cost2go`
  kept as an alias for one release (planning#3, examples#5, with S54).
- **A5** `Distribution.sample(key=None)` like the sets (core#5);
  `NoiseSource(distribution, sample_period)` replaces `WhiteNoise`'s
  `refresh()` cache, with a guard test now so S29 cannot land first
  (examples#14); `Trajectory.from_rollout(x0, xs, us, dt)` so scanned
  rollouts return the object, not arrays (examples#9; V1's first consumer).

**Wave B — analysis and control verbs**

- **B1 / P6** `discretize` on the fixed verb of §3.1 item 3; add an exact
  zero-order-hold option shared with `step_response` (analysis#0, #1).
- **B2 / P7, P8** a `poles(sys)` verb with a `sys.poles()` facade, and
  `LTISystem.__str__`, `TransferFunction.__str__`, `StructuralResult.__str__`
  so tutorial 01's `print(tf)` shows the transfer function (analysis#3,
  examples#2). Fourteen sites stop writing `np.linalg.eigvals(lin.A())`.
- **B3 new row** `controllability` / `observability` take any `System`
  through `linearize()` like every sibling, with facades (analysis#2);
  `step_info` and `find_equilibrium` take the same defaults (analysis#4, #8).
- **B4 new row** rename `control/lqr.py` so the LQR family can sit on the
  band facade and in the teaching-surface registry (control#0); `P` joins
  the siso family (control#1). Do it before P11 writes notebooks on the band
  layer.
- **B5 / P10** one keyword vocabulary for the four feedback wires across
  `closed_loop`, `hybrid_closed_loop` and `StandardFeedbackWiring` (core#12).
- **B6 / P11** `Controller(feedback="state" | "output", n_y, m)` declares
  its own ports from `feedback_profile`, so a GRO501 student writes gains
  and `ctl` only (examples#0).

**Wave C — catalog and planners**

- **C1** re-audit `pyro-port-remaining.md` against the code before the
  migration guide (docs-gov#11, fixes B14).
- **C2 new row** DP: a finite `problem.tf` selects the finite-horizon
  recursion (or warns once), matching `LQRPlanner`; a callable
  `infeasible_cost` is honoured in the table (planning#1, #2).
- **C3 new row** `MechanicalSystem` stops inventing ±2π / ±5 bounds; each
  plant states its own (dynamics#0; also removes the `Sys2Gym` `inf` trap
  and the three `NormalizedDrone2D` subclasses in notebooks). Output ports
  that are the state or a slice of it resolve labels and units from the
  state on read; the 56 copy lines go (dynamics#3, fixes B11 for good).
- **C4** `Sys2Gym` takes a `Distribution` for the start state (graphics#1).

**Wave D — one owner per rule (D3 grows)**

- **D3** `auto_dt(sys)` in `time_grid.py` is the one owner (Simulator,
  realtime, solver notes, hybrid default), and `refresh_solver_info`
  bubbles `smallest_time_constant` like it bubbles
  `discontinuous_behavior`; either the discontinuous scale becomes finer or
  DESIGN stops promising it (core#0, compile-sim#1). One
  try-JAX-then-NumPy policy in `compile_auto` with a typed
  `NotTraceableError` (compile-sim#0). Monte Carlo score: one default set
  shared by `MonteCarloEvaluator` and `Planner.evaluate`, and a backend
  that fits the law (planning#0). The forced-input hold model becomes one
  `input_interp` option every solver obeys (compile-sim#2). The one-tick lag
  inside a `Computer` is tested and stated, or removed (compile-sim#6).
- **D new row "wiring-time errors"** (RULES 4.10): `add_subsystem` rejects
  an evolution-kind mismatch (core#1); `connect` refuses to rewire a
  connected subsystem input (core#13); the compile probe detects undeclared
  feedthrough (core#2, decision D-b).
- **T6** one record per MPC tick: `Command` and `MPCTickSolve` fold into
  the `PlanningSolution` (control#4); the latch memoizes by `(k, y)`
  (bug control#1).
- **D1 / D2** one plot keyword set (`title`, `ax`, `backend`, `show`) and
  one `PlotResult.axes` shape across the verbs (graphics#4); `docs/api`
  generated from the teaching-surface registry, or a test that every
  prelude name's module has a page (docs-gov#0); the RULES 5.8 underscore
  check becomes a repo-wide ratchet with a shrinking allowlist
  (tests-ci#11, docs-gov#1).

### 3.3 Decisions requested (candidates for ROADMAP §6)

| Id | Question | My recommendation |
|----|----------|-------------------|
| D-a | Time-grid endpoint rule: `n = round((tf - t0) / dt)`, `t[-1] <= tf`, warn when `(tf - t0) / dt` is not an integer. Changes regression baselines and every `dt=` notebook by one sample. | Yes, in the close-out with one baseline update; a grid that ends past `tf` is a bug students will hit. |
| D-b | Undeclared feedthrough (`h` reads `u` with `y_dependencies=()`): raise or warn at compile. | Raise. A silent wrong fixed point is the worst outcome. |
| D-c | Retire the `JaxMechanicalSystem` twin (76 lines of `jnp`; the `xp` base already traces). Alias for one release. | Retire; it is the standing simplify-and-consolidate principle applied. |
| D-d | Sampled seam time argument: ticks (today) or seconds. Seconds removes three copies of `t0` / `dt_mpc` and `_replan_divisor`. | Seconds, decided with S31 and T6 together. |
| D-e | Mechanical bases' default bounds: keep the invented ±2π / ±5 or make each plant state its own. | Each plant states its own; `StateSpaceGrid` already errors on infinite bounds. |
| D-f | `PlanningProblem.metadata`: document or retire. | Retire unless a consumer is named. |
| D-g | `QuarterCarOnRoughTerrain` damper: is `b * (dy - dz/dx)` the intended model, or should it be `b * (dy - vx * dz/dx)`? | Physics check by the maintainer; the finder's derivation is the standard one. |
| D-h | `plot_diagram()` without the `graphviz` wrapper: warn, or move the pure-Python wrapper into the base dependencies. | Base dependency; the binary stays optional and already warns. |

## 4. Next small steps

1. This document (this commit).
2. Workboard rows for §3.1 items 1–5 under TODO §1 and the §3.3 rows on
   ROADMAP §6 (one commit, docs only).
3. Confirmed bug fixes B1+B2, B7, B9, B10, B11, B12, one commit each with a
   test, after the maintainer's go on the close-out list.
4. The §2.2 reproductions, five minutes each, before any of those fixes.

## Appendix: every finding, by band

Kind / effort / owner / rung as the finder proposed them; `planned` names an
existing step the finder judged the item belongs to. Full text in the
[ledger](2026-09-22-improvement-scan-ledger.md).


### core

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| core#0 | bug | S | maintainer | v0.2 wave D | — | Bubble smallest_time_constant to the diagram root |
| core#1 | trap | S | maintainer | v0.2 wave D | — | Reject evolution-kind mismatches in add_subsystem |
| core#2 | trap | M | maintainer | v0.2 wave D | — | Probe undeclared feedthrough at compile time |
| core#3 | trap | S | maintainer | v0.2 wave A | A2 | Check per-block params dicts against the block's keys |
| core#4 | api | M | maintainer | v0.2 wave A | A2 | Carry discount_rate as a field that composites keep |
| core#5 | api | S | maintainer | v0.2 wave A | A5 | Give Distribution.sample the sets' key=None default |
| core#6 | consolidation | S | agent | v0.2 wave A | A3 | Flatten Shape unions like IntersectionSet and SumCost |
| core#7 | consolidation | S | agent | v0.2 wave D | D1.1 | Give StepRollout and HybridSimResult a print, one validator with Trajectory |
| core#8 | consolidation | M | agent | v0.2 wave D | — | Let each textbook object own its print text |
| core#9 | docs | S | maintainer | v0.2 wave A | — | Export the operators' named forms together |
| core#10 | api | S | maintainer | v0.2 wave D | — | One verbose default and one empty-cache rule across compute_* facades |
| core#11 | consolidation | M | maintainer | v1.0 | S29 | One owner for the initial state: x0 or state.nominal_value |
| core#12 | api | S | maintainer | v0.2 wave B | P10 | One keyword vocabulary for the four feedback wires |
| core#13 | trap | S | maintainer | v0.2 wave D | — | Refuse to rewire a connected subsystem input |
| core#14 | consolidation | L | maintainer | v1.0 | S31 | Decide the core → simulation import with S31 |

Bugs:

- **bug core#0** (medium) `L @ plant` extends a shortcut-built left diagram in place — `minilink/core/composition.py:333-338`
- **bug core#1** (medium) `discount_rate` cannot be set on an instance and composite costs drop it — `minilink/core/costs.py:29-36`
- **bug core#2** (low) `refresh_solver_info` drops `smallest_time_constant`, so a loop ignores its plant's solver hint — `minilink/core/wiring.py:250-257`
- **bug core#3** (low) `IntersectionSet.margin` fails on a member whose margin returns a scalar — `minilink/core/sets.py:377-386`
- **bug core#4** (low) `DiagramSystem.add_subsystem` accepts a StepSystem and the reference `f` returns a short vector — `minilink/core/wiring.py:242-248`

### compile-simulation

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| compile-simulation#0 | consolidation | S | agent | v0.2 wave D | — | Give the try-JAX-then-NumPy policy one owner and a typed error |
| compile-simulation#1 | trap | S | maintainer | v0.2 wave D | D3 | Give the automatic dt one owner and make the discontinuous scale mean something (or drop the claim) |
| compile-simulation#2 | api | M | maintainer | v0.2 wave D | — | Make the forced-input hold model one option honoured by every solver |
| compile-simulation#3 | trap | S | maintainer | v0.2 wave D | — | State and test what `n_steps` counts on each simulation verb |
| compile-simulation#4 | trap | M | maintainer | v0.2 wave D | D3 | Derive the hybrid plant sub-step from the plant time constant instead of one RK4 step per tick |
| compile-simulation#5 | consolidation | S | agent | v0.2 wave D | D3 | Align HybridSimulator with its siblings: shared input coercion, positional `input_port_id`, the framed verbose panel |
| compile-simulation#6 | trap | S | maintainer | v0.2 wave D | — | Pin (or reconcile) the one-tick lag between chained blocks inside a Computer |
| compile-simulation#7 | api | S | agent | v0.2 wave D | — | Make `has_trace_tier` a plain boolean on both backends and correct the frozen-subset sentence |
| compile-simulation#8 | consolidation | S | agent | v0.2 wave D | — | Stop `simulation` importing from `optimization` for three display constants |
| compile-simulation#9 | consolidation | M | agent | v1.0 | S37 | Deduplicate the evaluator internals that S37 will otherwise carry: ZOH sugar, the JAX step rollout, the four `_jac_probe`s, the undeclared batch cache |
| compile-simulation#10 | trap | S | maintainer | v0.2 wave D | — | Make the `scipy_stiff` preset honest: explicit tolerances and a Jacobian on both backends |
| compile-simulation#11 | tooling | S | agent | v0.2 wave D | S53 | Gate the quoted batch-rollout claim and give diagrams a frozen-params reference for the family rule |
| compile-simulation#12 | performance | S | agent | v0.2 wave D | — | Batch the static-leaf time grid instead of dispatching 10 001 times |
| compile-simulation#13 | trap | S | agent | v0.2 wave D | — | Let `compile(verbose=True)` read in order on every path |
| compile-simulation#14 | consolidation | S | agent | v0.2 wave D | T4 | Share one uniform-grid check between the fixed-step backends and the simulator |

Bugs:

- **bug compile-simulation#0** (medium) `dt`-based time grids overshoot `tf` by one sample (float `arange`) — `minilink/simulation/time_grid.py:47`
- **bug compile-simulation#1** (medium) `JaxDiagramEvaluator` casts `dx` and the signal buffer to the dtype of `x`, truncating derivatives for integer states — `minilink/core/compile/evaluators/jax_evaluators.py:921-926`
- **bug compile-simulation#2** (low) `compile_step_diagram` probes every subsystem's `step` and `h` twice — `minilink/core/compile/step_compiler.py:56-57`

### dynamics

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| dynamics#0 | trap | M | maintainer | v0.2 wave C | — | Drop the invented default bounds from the mechanical bases; each plant states its own |
| dynamics#1 | api | S | maintainer | v0.2 wave C | — | Constructor hygiene in the pendulum and mass-spring-damper families |
| dynamics#2 | trap | S | maintainer | v0.2 wave C | — | One port layout across the bicycle rungs: the `named_ports` default and the `x` port |
| dynamics#3 | consolidation | M | maintainer | v0.2 wave D | — | Derive `y` / `q` / `dq` port labels from the state instead of copying them |
| dynamics#4 | consolidation | M | maintainer | v0.2 wave C | — | One owner for the wheelbase: `a`, `b` in params; `length` and the `.a` / `.b` attributes go |
| dynamics#5 | consolidation | M | maintainer | v1.0 | S32 | Put `DynamicBicycle` on `GeneralizedMechanicalSystem` |
| dynamics#6 | trap | S | maintainer | v0.2 wave C | — | Name the hidden `0.01` linear damping in `Drone2D.d` and `Rocket.d` |
| dynamics#7 | trap | S | maintainer | v0.2 wave D | T3 | `Manipulator` kinematics defaults raise instead of returning zeros; `link_lengths` becomes the base contract |
| dynamics#8 | api | S | maintainer | v0.2 wave C | — | Give `VanderPol` a real input or none |
| dynamics#9 | trap | S | maintainer | v0.2 wave D | S43 | Make the physics own the drawn lengths: `DoublePendulum.l2`, `CartPole.pole_length`, `Boat2D.body_width` |
| dynamics#10 | trap | S | agent | v1.0 | S29 | The racecar solver hint must not depend on the pre-read `refresh()` that S29 removes |
| dynamics#11 | test | S | agent | v0.2 wave D | T3 | Extend the both-backends catalog test with a perturbed-`params` case |
| dynamics#12 | docs | S | agent | v0.1 close-out | — | Docs: the Sphinx dynamics page covers 7 of 23 catalog modules, and DESIGN points at a `car_profile` that is not in the catalog |
| dynamics#13 | consolidation | S | maintainer | v0.2 wave D | D2 | Three catalog rows for the D2 consolidation pick list |
| dynamics#14 | consolidation | S | maintainer | v0.2 wave D | — | Retire the `JaxMechanicalSystem` twin: the `xp` base already traces |

Bugs:

- **bug dynamics#0** (medium) QuarterCarOnRoughTerrain damps against the road slope instead of the road's vertical velocity — `minilink/dynamics/catalog/vehicles/suspension.py:82`
- **bug dynamics#1** (low) Plane2D.d / Plane3D.d index `u`, so `inverse_dynamics` (and the model-based controllers) crash on the base's `u=None` contract — `minilink/dynamics/catalog/aerial/plane.py:156`
- **bug dynamics#2** (low) `q` / `dq` output-port labels are stale on every MechanicalSystem plant — `minilink/dynamics/abstraction/mechanical.py:66-71`
- **bug dynamics#3** (low) TwoMass / ThreeMass silently clamp an out-of-range `output_mass` — `minilink/dynamics/catalog/mass_spring_damper/linear.py:36-41`

### control

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| control#0 | api | M | maintainer | v0.2 wave B | — | Put the LQR family on the control band facade |
| control#1 | api | S | maintainer | v0.2 wave B | — | Add `P` to the siso family and align `ProportionalController`'s default layout |
| control#2 | consolidation | M | maintainer | v0.2 wave D | T2 | One time-interpolation for trajectories and gain schedules |
| control#3 | api | L | maintainer | v1.0 | S31 | Decide the time argument on the sampled seam once: ticks or seconds |
| control#4 | consolidation | M | maintainer | v0.2 wave D | T6 | One record per MPC tick: fold `Command` and `MPCTickSolve` into the `PlanningSolution` |
| control#5 | consolidation | S | maintainer | v0.2 wave D | T2 | One warm-start helper on the plan `Trajectory` |
| control#6 | api | M | maintainer | v0.2 wave C | C2 | One constructor contract for the robotic laws |
| control#7 | consolidation | M | agent | v0.2 wave D | T2 | Consolidate the impedance and robotic law bodies |
| control#8 | consolidation | S | maintainer | v0.2 wave D | — | One `ctl` for the model-based laws |
| control#9 | trap | S | maintainer | v0.2 wave A | A3 | Size PurePursuit's measurement from the vehicle, not a `state_dim=9` default |
| control#10 | feature | S | agent | v0.2 wave D | D1.1 | `print(controller)` shows the gains |
| control#11 | test | S | agent | v0.2 wave D | — | One parameterized both-backends test over every control law |
| control#12 | docs | S | agent | v0.1 close-out | T2 | Control band docs housekeeping |
| control#13 | consolidation | S | maintainer | v0.2 wave D | — | Move `LookupTableController` beside the other laws |

Bugs:

- **bug control#0** (medium) `lqr_gain_schedule` returns a wrong or NaN schedule on stiff pairs with a coarse grid (no substep guard, unlike `riccati_step`) — `minilink/control/lqr.py:120-128`
- **bug control#1** (medium) MPC tick latch memoizes by `k` only and returns a plan solved for a different measurement — `minilink/control/mpc/controller.py:630-642`
- **bug control#2** (low) `mpc % dt` bypasses the `dt_mpc` cross-check that `export_to_computer(dt)` enforces — `minilink/core/system.py:412-416`
- **bug control#3** (low) RULES 4.1's canonical band import `from minilink.control import lqr` raises — `RULES.md:139`
- **bug control#4** (low) `gravity_feedforward` masks a gravity hook's own `TypeError` — `minilink/control/robotic.py:432-437`

### analysis

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| analysis#0 | api | M | maintainer | v0.2 wave B | P6 | Move dt off the params dict in discretize and copy the source's x0, labels and bounds |
| analysis#1 | feature | S | maintainer | v0.2 wave B | P6 | Add an exact zero-order-hold option to discretize and share it with step_response |
| analysis#2 | api | M | maintainer | v0.2 wave B | — | Let controllability and observability take any System, with facades and a __str__ |
| analysis#3 | api | S | maintainer | v0.2 wave B | P8 | Add a poles verb on a System so notebooks stop writing np.linalg.eigvals(lin.A()) |
| analysis#4 | api | S | maintainer | v0.2 wave B | — | Let step_info take a System like every other verb in time_response |
| analysis#5 | consolidation | S | agent | v0.2 wave B | — | Refine the margin crossings by a root solve so plot_bode and margins() report one number |
| analysis#6 | trap | S | agent | v0.2 wave B | — | Check the operating point's size and warn on an unstable step channel at the boundary |
| analysis#7 | test | S | agent | v0.2 wave B | P7 | Pin the band's calling pattern with a signature test |
| analysis#8 | api | S | maintainer | v0.2 wave B | — | Default find_equilibrium's guess to sys.x0 like the rest of the band |
| analysis#9 | api | S | agent | v0.2 wave D | D2 | Give the five control plots one keyword set and the region plot the same return type |
| analysis#10 | trap | S | agent | v0.2 wave D | T4 | Overlay trajectories on the region plot without touching sys.x0 or sys.traj |
| analysis#11 | trap | M | maintainer | v0.2 wave B | P7 | Free linearize and discretize from the band-facade name collision |
| analysis#12 | docs | S | agent | v0.1 close-out | P4 | Put the seven missing analysis modules on the API page and trim the two placeholder docstrings to their step ids |
| analysis#13 | api | M | maintainer | v0.2 wave B | P4 | Give the state-space base named input ports so observers and sensitivity blocks share one f |
| analysis#14 | feature | S | maintainer | v0.2 wave C | C4 | Log dx and y on the simulated Trajectory so an equation-error fit has its data |

Bugs:

- **bug analysis#0** (high) discretize() on a diagram produces a StepSystem that cannot step: the flat dt key is rejected as an unknown subsystem id — `minilink/analysis/discretize.py:119-135`
- **bug analysis#1** (medium) discretize() drops the source's x0 (and state labels and input bounds); the shipped demo rolls out from rest — `minilink/analysis/discretize.py:13-29`
- **bug analysis#2** (low) DiscretizedDynamicSystem.step rejects the plant's own params and exposes dt as a parameter sensitivity — `minilink/analysis/discretize.py:66-79`
- **bug analysis#3** (low) step_info rise time uses \|y\| thresholds, so an undershooting (non-minimum-phase) response reports the wrong 10-90 % time — `minilink/analysis/time_response.py:138-145`
- **bug analysis#4** (low) The margins printed on plot_bode differ from margins() because they are read off grids of different resolution — `minilink/analysis/frequency.py:45`

### planning

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| planning#0 | trap | S | maintainer | v0.2 wave D | — | Give the Monte Carlo score one set of defaults and a backend that fits the law |
| planning#1 | trap | M | maintainer | v0.2 wave C | — | Let a finite problem.tf pick the finite-horizon recursion in DynamicProgrammingPlanner.solve |
| planning#2 | bug | S | maintainer | v0.2 wave C | — | Honour a callable infeasible_cost in the value-iteration table |
| planning#3 | api | S | maintainer | v0.2 wave A | A4 | One name and one keyword for the cost-to-go picture across the band |
| planning#4 | api | S | maintainer | v0.2 wave A | A1 | Give PolicyEvaluator the evaluators' verb and a field-shaped result |
| planning#5 | consolidation | S | agent | v0.2 wave D | — | Derive the flat-kwarg key tuples from the option dataclasses, one overlay helper, and mirror the RRT pair's constructors |
| planning#6 | performance | S | agent | v0.2 wave D | — | Track RRT* goal nodes incrementally instead of rescanning the tree every extension |
| planning#7 | consolidation | S | agent | v0.2 wave D | D1.4 | Make Planner.plot_solution the solution's own plot_trajectory |
| planning#8 | consolidation | S | agent | v0.2 wave D | — | One state-axis label helper for the three planning plot modules |
| planning#9 | api | S | agent | v0.2 wave D | T5 | Route the parametric (MPC) optimizer through Optimizer's backend table |
| planning#10 | test | S | agent | v0.2 wave D | — | Pin the scoring contract with a params.sets parity test across the three backends and the grid |
| planning#11 | trap | S | maintainer | v0.2 wave D | — | One owner of the sets between StateSpaceGrid and the planner that uses it |
| planning#12 | performance | M | agent | v0.2 wave D | D3 | Lower the state set member-wise in the transcriptions |
| planning#13 | trap | S | agent | Later | — | Do not hand Ipopt the objective Hessian alone |
| planning#14 | docs | S | maintainer | v0.2 wave D | — | Decide PlanningProblem.metadata: document it or retire it |

Bugs:

- **bug planning#0** (medium) score_trajectory drops problem.params.sets, so the NumPy and simulator Monte Carlo backends disagree with the JAX backend on a parametric constraint set — `minilink/planning/evaluation.py:30`
- **bug planning#1** (medium) MonteCarloEvaluator's default backend cannot score a DP or tabular solution and the error blames the block — `minilink/planning/evaluation.py:136-139`
- **bug planning#2** (medium) DynamicProgrammingPlanner silently replaces a callable infeasible_cost with the 1e6 default — `minilink/planning/policy_synthesis/dp.py:648-649`
- **bug planning#3** (low) DynamicProgrammingPlanner.value_at, plot_cost2go, plot_policy and animate_* raise AttributeError before solve instead of the planner's 'No solution' error — `minilink/planning/policy_synthesis/dp.py:386-438`

### graphics-blocks-interfaces

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| graphics-blocks-interfaces#0 | api | S | agent | v0.2 wave B | — | Let plot_trajectory select a leaf's named output ports |
| graphics-blocks-interfaces#1 | api | M | agent | v0.2 wave C | — | Give Sys2Gym a Distribution for the start state |
| graphics-blocks-interfaces#2 | test | S | agent | v0.1 close-out | — | Register the block names the facade exports and walk every band facade in the registry test |
| graphics-blocks-interfaces#3 | api | S | maintainer | v0.2 wave D | — | Make minilink.graphical.catalog the one shapes-and-skins facade students import |
| graphics-blocks-interfaces#4 | api | M | agent | v0.2 wave D | — | One keyword vocabulary across the plot verbs (title, ax, backend) and one PlotResult.axes shape |
| graphics-blocks-interfaces#5 | consolidation | S | agent | v0.2 wave D | T3 | One axis-label formatter and one unit convention (no brackets in the catalog) |
| graphics-blocks-interfaces#6 | consolidation | S | agent | v0.2 wave D | — | One backend resolver and one optional-import helper for the graphical band |
| graphics-blocks-interfaces#7 | trap | M | agent | v0.2 wave D | S43 | Publish a renderer capability table and give each cell an honest fallback |
| graphics-blocks-interfaces#8 | trap | S | agent | v0.1 close-out | — | Silence the renderers (RULES 4.6) |
| graphics-blocks-interfaces#9 | trap | S | agent | v0.2 wave D | — | Warn on the +-10 phase-plane window and stop duplicating the default bounds |
| graphics-blocks-interfaces#10 | api | S | agent | v0.2 wave C | — | A lazy facade for minilink.interfaces |
| graphics-blocks-interfaces#11 | api | S | maintainer | v0.2 wave C | C3 | Give Integrator and ZOHHold a dim like every static block |
| graphics-blocks-interfaces#12 | consolidation | S | maintainer | v0.2 wave D | — | Fold NeuralNetwork into MLP (one owner of the one-hidden-layer map) |
| graphics-blocks-interfaces#13 | docs | M | agent | v0.2 wave D | T1 | Add the graphical band to the textbook pass (T7) |
| graphics-blocks-interfaces#14 | consolidation | S | maintainer | v0.2 wave D | D2 | Retire the sources demo and __main__ that only exist for show_signal |

Bugs:

- **bug graphics-blocks-interfaces#0** (medium) Sys2Gym.reset fails or returns inf on any plant without finite state bounds (also through from_problem) — `minilink/interfaces/gymnasium.py:134-136`
- **bug graphics-blocks-interfaces#1** (low) Animation frame schedule never draws the last sample and crashes on a one-sample trajectory — `minilink/graphical/animation/renderers/timing.py:30`
- **bug graphics-blocks-interfaces#2** (low) animate(save=True, file_name='clip.gif') writes clip.gif.gif — `minilink/graphical/animation/renderers/matplotlib_renderer.py:573-575`
- **bug graphics-blocks-interfaces#3** (low) Plotly time-signal y-labels double the brackets of '[m]'-style units — `minilink/graphical/signals/plotly_backend.py:294`
- **bug graphics-blocks-interfaces#4** (low) HybridDiagram.plot_trajectory accepts a dead abscissa argument — `minilink/core/hybrid_diagram.py:230`

### examples-teaching

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| examples-teaching#0 | api | M | maintainer | v0.2 wave B | P11 | Declare controller ports from `feedback_profile` so a student writes only `ctl` |
| examples-teaching#1 | trap | S | maintainer | v0.1 close-out | R1 | Make `plot_diagram()` warn, not raise, when the `graphviz` wrapper is missing |
| examples-teaching#2 | api | S | maintainer | v0.2 wave B | P7 | Let the linear records print themselves and add a `poles` verb |
| examples-teaching#3 | docs | M | maintainer | v0.2 wave D | D1.3 | Fill the placeholder sections of tutorials 01, 02, 03, 05, 08 and 09 |
| examples-teaching#4 | test | S | agent | v0.1 close-out | R1 | Pin one Colab setup cell per tier with a test, and align install.md's tiers to it |
| examples-teaching#5 | api | S | maintainer | v0.2 wave D | S54 | One name and one keyword for the cost-to-go plot across planner, evaluator and comparison |
| examples-teaching#6 | feature | S | agent | v0.2 wave D | D1.1 | Overlay several trajectories on one phase plane and two fields on one grid surface |
| examples-teaching#7 | trap | M | maintainer | v0.2 wave D | D1.2 | Stop teaching code from rebinding `sys`; add it to the flatness ratchet |
| examples-teaching#8 | test | S | maintainer | v0.1 close-out | — | Keep each course pin byte-identical to its topic twin with a `cmp` test |
| examples-teaching#9 | api | S | maintainer | v0.2 wave A | V1 | `Trajectory.from_rollout(x0, xs, us, dt)` for compiled and scanned rollouts |
| examples-teaching#10 | api | S | maintainer | v0.2 wave A | A5 | Give the double-integrator homework its two missing verbs: entry time into a set and the Bellman residual |
| examples-teaching#11 | api | S | maintainer | v0.2 wave D | — | Accept `optimizer_method="auto"` that picks Ipopt when installed |
| examples-teaching#12 | api | M | maintainer | v0.2 wave D | T6 | Let `mpc @ inner_loop` close on a multi-block plant diagram |
| examples-teaching#13 | api | S | maintainer | v0.2 wave A | A3 | Close the facade gaps the import allowlist records |
| examples-teaching#14 | trap | M | maintainer | v0.2 wave A | A5 | `WhiteNoise` samples are rebuilt only by Simulator's pre-read `refresh()`; S29 would silently break the noise demos |

Bugs:

- **bug examples-teaching#0** (low) Tutorial 00 §2 prints the plant's state labels on the "Diagram" line — `examples/tutorial/00_core.ipynb (code cell 11)`
- **bug examples-teaching#1** (medium) `trajopt_cartpole_collocation_jax.py` hard-codes `optimizer_method="ipopt"` and fails without cyipopt — `examples/demos/trajopt/trajopt_cartpole_collocation_jax.py:42`

### tests-ci-tooling

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| tests-ci-tooling#0 | tooling | S | agent | v0.1 close-out | — | Run pytest with JAX and Ipopt in the CI regression job so the optional-extra tests are gated |
| tests-ci-tooling#1 | test | M | agent | v0.1 close-out | — | Guard optional extras per test instead of per module: move mid-file importorskip into the JAX classes and fix marker misuse |
| tests-ci-tooling#2 | bug | S | agent | v0.1 close-out | — | Make the flagship manifests honest: `requires` must list every optional import a demo makes, and drop or validate dead `demo_id`s |
| tests-ci-tooling#3 | performance | S | agent | v0.2 wave D | — | Gate the subprocess demo-check bridge tests behind a marker so the unit suite stays a unit suite |
| tests-ci-tooling#4 | consolidation | S | agent | v0.2 wave D | — | One owner for the CI regression-gate flags shared by tests/run, tests/README and test.yml |
| tests-ci-tooling#5 | tooling | S | agent | v0.1 close-out | — | Give every teaching notebook a job that executes it: nightly `--all`, unique ids, and drop the stale intro branch |
| tests-ci-tooling#6 | tooling | S | agent | v0.1 close-out | — | Add ruff check and ruff format hooks to pre-commit so the 'always before push' gate runs itself |
| tests-ci-tooling#7 | consolidation | S | agent | v0.2 wave D | D1.2 | One owner for the teaching-facade list shared by test_teaching_imports and test_teaching_surface |
| tests-ci-tooling#8 | docs | M | agent | v0.1 close-out | — | Pin the Sphinx API pages to the teaching surface with a test and build docs with -W |
| tests-ci-tooling#9 | test | S | agent | v0.2 wave D | — | Move wall-clock speed claims out of unit tests into the regression gates |
| tests-ci-tooling#10 | api | S | agent | v0.2 wave D | — | Expose `Animator.resolve_frame` so the graphics harness stops calling a private method |
| tests-ci-tooling#11 | test | M | agent | v0.2 wave D | — | Turn the RULES 5.8 underscore check into a repo-wide ratchet with a shrinking allowlist |
| tests-ci-tooling#12 | tooling | S | maintainer | v0.1 close-out | R1 | Run the test suite before publishing to PyPI and put timeouts on the long CI jobs |
| tests-ci-tooling#13 | trap | S | maintainer | v0.1 close-out | — | Trap: showcase_jax.ipynb imports minilink.experimental.c_export, which the wheel does not ship |
| tests-ci-tooling#14 | consolidation | M | agent | v0.2 wave D | — | Stop unit tests from importing examples/projects and benchmarks/systems so the suite runs from the sdist and the Basic tier |

Bugs:

- **bug tests-ci-tooling#0** (high) CI merge gate never executes any JAX-dependent test: no job installs jax and runs pytest — `.github/workflows/test.yml:31`
- **bug tests-ci-tooling#1** (medium) Module-level pytest.importorskip("jax") placed mid-file (or atop mostly-NumPy files) skips the NumPy tests too — `tests/unittest/test_dynamics_catalog.py:533`
- **bug tests-ci-tooling#2** (medium) Flagship manifest omits graphviz for the two MPC demos, so `pytest` fails without the diagrams extra — `tests/demo_checks/flagship_manifest.json`
- **bug tests-ci-tooling#3** (low) run_notebook_checks.py assigns the same id to two different notebooks, so overrides and --notebook filters are ambiguous — `tests/demo_checks/run_notebook_checks.py:48-57`
- **bug tests-ci-tooling#4** (low) tests/run regression launcher claims CI parity but uses --factor 6 where CI uses --factor 10 — `tests/run/_common.py:57-66`
- **bug tests-ci-tooling#5** (low) tests/README.md and the flagship-graphics manifest carry stale facts (file count, removed folders, dead demo_id links) — `tests/README.md:135`

### docs-governance

| id | kind | eff. | owner | rung | planned | title |
|----|------|------|-------|------|---------|-------|
| docs-governance#0 | tooling | M | agent | v0.2 wave D | — | Generate docs/api from the teaching-surface registry |
| docs-governance#1 | test | S | agent | v0.2 wave D | T2, T3, T5, T6 | Make the RULES 5.8 check a ratchet over every System-family class |
| docs-governance#2 | test | S | agent | v0.2 wave D | — | Test that ROADMAP §5 step ids and TODO rows agree |
| docs-governance#3 | docs | S | agent | v0.2 wave D | — | Give plan-doc steps ids that cannot collide with the workboard's |
| docs-governance#4 | docs | S | agent | v0.1 close-out | — | Align RULES 3.3's teaching-surface definition with ROADMAP §2 |
| docs-governance#5 | consolidation | S | agent | v0.1 close-out | — | Keep the CI commands in one place and name every job |
| docs-governance#6 | consolidation | S | agent | v0.2 wave D | — | Delete DESIGN §8's Package roles table (a stale copy of §3) |
| docs-governance#7 | docs | S | agent | v0.1 close-out | — | Move DESIGN's inline TODOs to the workboard and fix its retired pointers |
| docs-governance#8 | docs | S | agent | v0.2 wave D | — | Scrub retired phase numbers and line pointers from the plan docs; settle the optimizer-wiring rung |
| docs-governance#9 | api | S | maintainer | v0.2 wave B | P4 | Name the estimation API once, in the P4 plan |
| docs-governance#10 | consolidation | M | maintainer | v0.2 wave D | D2 | Add the DESIGN research-lane trim to D2 as a row with its three moves |
| docs-governance#11 | docs | M | agent | v0.2 wave C | C1 | Re-audit pyro-port-remaining against the code before the migration guide |
| docs-governance#12 | docs | S | agent | v0.1 close-out | — | Drop tests/README's stale module census and the duplicated smoke-policy lines |
| docs-governance#13 | docs | S | maintainer | v0.1 close-out | — | Name who owns ROADMAP §5 and §6 in the AGENTS lanes |

Bugs:

- **bug docs-governance#0** (low) DESIGN §6 documents `Distribution.mean()` as a method; it is an attribute and the call raises — `DESIGN.md:925`
- **bug docs-governance#1** (low) DESIGN §5 points at a diagnostics script that does not exist — `DESIGN.md:824-825`
- **bug docs-governance#2** (low) RULES 7.6 says there is no link checker in CI; `test_repo_contract.py` checks links in the CI `test` job — `RULES.md:456-458`
- **bug docs-governance#3** (low) tests/README.md claims 22 domain test modules; 45 exist — `tests/README.md:135-141`
- **bug docs-governance#4** (low) AGENTS.md says CI runs "exactly" three things; the workflow also has a `packaging` job — `AGENTS.md:86`
- **bug docs-governance#5** (medium) pyro-port-remaining marks landed features as TODO (`ss2tf`, `TrajectoryLQRController`, dedicated PI/PD) — `docs/plans/pyro-port-remaining.md:34, 43, 44, 288, 293`
- **bug docs-governance#6** (low) `estimation/` and `identification/` package docstrings point at a ROADMAP section that no longer exists and pre-name an API the plan contradicts — `minilink/estimation/__init__.py:8, 11`
