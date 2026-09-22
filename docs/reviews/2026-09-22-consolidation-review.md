# Consolidation review: goal, architecture, state, and the path to v1.0

**Date:** 2026-09-22
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Branch `dev` @ `1f0f7ce`, after the GRO860 term release work and the GRO501
and racecar additions of 2026-09-16 to 2026-09-22. The maintainer asked for four things: a
review of the goal and the high-level architecture, a status of development, one coherent
plan to v1.0 in place of the phase logs and independent plan docs, and a first
behaviour-preserving textbook pass on the modules that do not yet read like `dp.py`.
Evidence: CONSTITUTION, RULES, AGENTS, DESIGN, ROADMAP, TODO, every plan doc, the eight
dated reviews, the library (261 modules, 56 k lines outside `experimental/`), the 48 test
files, and a five-band line-level style audit run for this review (its findings are the
ledger in §4).
**Applied the same day:** the roadmap and workboard of §3, and the T0 style pass of §4.

---

## 1. The goal and the architecture

### 1.1 Verdict

**The goal is sharp and the architecture serves it.** Minilink's one idea is that a model
is a pure `f(x, u, t; p)` with two companion maps, and that every tool is a verb on that
object, so a course model is a research model without a rewrite. The constitution states
it in one page and, unusually, gives a decision procedure (the precedence ladder: purity,
closure, continuous primary, readability, single path) rather than a list of virtues. The
2026-09-15 addition of the mathematical objects (set, distribution, field, cost, problem,
solution) as the nouns the tools carry is the second good idea: it is what lets value
iteration, LQR, trajectory optimization, RRT and reinforcement learning all take one
`PlanningProblem` and return one `PlanningSolution` that `compare()` can read. Nothing in
this review argues for a new core type or for re-cutting an existing one.

**What is strong, and should not be touched:**

- The `System` → ports → `DiagramSystem` → `compile` → `Simulator` spine, with
  compile-versus-reference parity tested. The composition grammar `+`, `>>`, `@` is nicer
  for teaching than anything comparable, and it is frozen.
- Purity as the top law. It is what makes the same equations batch under `vmap`, trace
  under `jit`, and differentiate: the catalog now compiles on both backends in full.
- Governance as tests: the teaching-import check, the no-harness-in-demos check, the
  set-probe ratchet, the link check, the underscore check. Prose rules drift in days here;
  the rules that became tests have not.
- The two-lane contract with the wheel scope. A student can be told exactly what is
  promised.
- `dp.py` after 2026-09-18 is a genuine textbook chapter: the reference every other module
  is now measured against.

### 1.2 Risks, in the order they matter

1. **Surface against one maintainer.** 101 names on the root prelude, 261 modules,
   1 236 tests, 71 demos, 38 notebooks, and a governance stack of about 2 000 lines. The
   2026-09-05 review said this and it is still the strategic risk. The two courses are the
   right discipline: v0.2 should add only what GRO501 and the three foundation objects
   need, and no new catalog plant. The roadmap in §3 is written that way.
2. **The one seam in "everything is a System".** `Computer` and `HybridDiagram` are not
   Systems, and MPC, the intro surface's hybrid exemplar, lives on them. The seam is
   correctly parked at v1.0 (S31), but every new hybrid feature deepens it: dual-rate MPC,
   the racecar MPC pair, `HybridDiagram`'s hand-copied facades. Recommendation: no new
   hybrid *features* before the S31 decision, only fixes; the roadmap says so.
3. **The `params` gap.** The constitution now promises "one signature" traceable in
   `params` for every object; today no library cost, set or shape reads `params`. The
   promise is right, and the cost-params plan is the first step, but until wave A lands
   the sentence is aspirational. Keep it scheduled, not implied.
4. **Two modules carry the complexity the core is supposed to keep out.**
   `core/composition.py` (1 351 lines, 45 underscore helpers, three semantics of `@`) is
   the operator a student meets first; `control/mpc/controller.py` (908 lines, 17
   underscore methods, attributes set from outside `__init__`) is the exemplar of the
   hybrid path. Both need a conversation before a pass, which is why they are T6 rather
   than part of the pass below.
5. **DESIGN.md is the next drift risk.** At 84 kB it carries about 250 lines of
   research-lane mechanism (realtime, dual-rate MPC, the spatial pipeline) inside the
   frozen contract. The 2026-09-05 inventory already named it (item 15); it stays a D2
   pick because it is the maintainer's document.
6. **CI does not run on the working branch.** `test.yml` triggers on `main`,
   `refactor-v4` and `dev-alex`; the work is on `dev`. Nothing gates a push until a PR,
   which is how `ruff` came to fail on 11 counts in the racecar and value-iteration demos
   between 2026-09-20 and 2026-09-22 (fixed in this pass). One line per workflow (R2).
7. **Two semantic decisions block v0.2 code and are still open:** the terminal cost
   `h` per tool and the disturbance convention for the estimation band. They are cheap to
   decide and expensive to retrofit; the roadmap puts them at the head of waves B and C.

### 1.3 What I would not change

The precedence ladder; the four-document governance split; the length of RULES; the
decision to solve courses with the native RL planner and keep `Sys2Gym` as the taught
bridge; the three DP backends as a teaching ladder; conda as the Full local install.

## 2. State of development

Measured on `dev` @ `1f0f7ce` in a clean Python 3.11 environment (`pip install -e
".[dev,rl,diagrams]"` plus CPU JAX).

| Signal | State |
| --- | --- |
| `ruff check .` / `ruff format --check .` | **red** on arrival (11 errors, 1 unformatted file, all in `examples/demos/udes_racecar/`, `examples/demos/value_iteration/vi_cartpole_swingup.py`, `examples/projects/racecar/`); green after the lint-only commit of this pass |
| `pytest` | 1 273 passed, 30 skipped after the pass (§4.3); on arrival the same suite passed up to the link check, which failed only on the review document this file now is |
| Regression gates, flagship demos, notebook smoke | not run here (the `regression` job needs Ipopt, plotly, meshcat and pygame); last green on the maintainer's machine per the commits of 2026-09-18 to 2026-09-21 |
| GRO860 path (§4.1 of ROADMAP) | every topic row green; gates 1–7 met; gate 8 (PyPI) waits on the `0.1.0` tag and the trusted-publisher registration (R1) |
| GRO501 path (§4.2) | seven of eleven topic rows green; `minreal`, `place`, the sensitivity shortcuts, the `N` matrix and the estimation band are the v0.2 work (P2–P8); estimation is held since 2026-09-07 |
| Both-backends catalog contract | 49 plants, all pass |
| Root prelude | 101 names, tested as a set |
| Library size | 261 modules, 55 977 lines outside `experimental/` |
| Tests | 48 files, 1 236 test functions |
| Examples | 71 demos, 31 project scripts, 9 experimental scripts, 38 tutorial and teaching notebooks |
| Plan docs | 17 on arrival; 12 after this pass (5 landed and deleted, 2 renamed) |

Maturity by band is the TRL table of ROADMAP §3, rewritten today to state the current
state rather than the landing history. In one line: the core, simulation and dynamics
bands are at TRL 7; control, analysis, DP and RL at 6; trajopt, optimization, RRT and
graphics at 5; compile (frozen subset), the RL bridge, Lyapunov, geometry and hybrid at 4;
realtime at 2; estimation and identification are placeholders.

## 3. The roadmap readjustment

### 3.1 What was wrong

The plan of record had grown by accretion. ROADMAP §5 was a phase table from
2026-09-05 (D, 0, 1, 2, 3) with three dated status paragraphs appended; phases D–2 were
complete. §6 mixed struck-through settled items with open questions for three different
milestones. `docs/plans/TODO.md` carried every landed row since 2026-09-05 as history and
scattered the open ones across six sections. The plans folder held 17 documents: five were
landed or folded (`lyapunov-certificates`, `planning-solution-comparison`,
`standard-planning-problems`, `core-objects-5`, `core-objects-6`), two carried
numbering from retired schemes (`core-objects-4-fields`, `phase4-fidelity-maps`), and the
July research designs still pointed at "ROADMAP §5 Phase 3". The GRO501 plan, the
foundations plan and the geometry plan were each internally coherent but nothing said in
which order they run or which is blocked on what.

### 3.2 What changed

- **ROADMAP §5 is one ladder to v1.0**: the v0.1 close-out (R1–R3, S49, S54), v0.2 in
  four waves (A the textbook objects: fields, cost parameters, geometry, naming; B GRO501;
  C catalog, parity and new bands; D textbook code, minimal demos, consolidation), and
  v1.0 (the foundation questions). Every step has an id and lives in the workboard.
- **ROADMAP §6 lists only open decisions**, grouped by the rung they block; settled
  decisions are in DESIGN and the reviews.
- **ROADMAP §3** states the current state per band and names the next step ids.
- **ROADMAP §1** gained three short edits (the close-out pointer, the foundation objects in
  the v0.2 row, the v1.0 row pointing at §5.3). §1, §2 and §4 are maintainer-owned; these
  are for your confirmation.
- **`docs/plans/TODO.md` is the workboard**: every open step by rung, step ids kept
  (S29, P3, …) so cross-references hold; landed rows removed (the last full board is the
  file at `1f0f7ce`).
- **`docs/plans/README.md`** indexes the twelve remaining docs as *scheduled* (a rung names
  them) or *research lane, unscheduled*, and records the deletions.
- **Deleted:** `lyapunov-certificates.md` (implemented; the open rulings D5 and D6 are in
  ROADMAP §6), `planning-solution-comparison.md` (landed; the demo rewrites are D1.4),
  `standard-planning-problems.md` (the deterministic / stochastic pair shipped; the robust
  class is a Later idea), `core-objects-5-later-nouns.md` (A5), `core-objects-6-demos.md`
  (D1). **Renamed:** `core-objects-4-fields.md` → `fields.md`, `phase4-fidelity-maps.md`
  → `fidelity-maps.md`. Every inbound link and the `lyapunov.py` message were updated;
  the link test passes.
- **Kept and relabelled:** `gro501-classical-control.md` (wave B), `cost-params.md` (A2),
  `geometry-module.md` (A3), `naming.md` (A4), `pyro-port-remaining.md` (C1),
  `cbf-safety-filter.md`, `optimizer-parametric-wiring.md`, `fidelity-maps.md`,
  `articulated-mechanism.md` (research lane, unscheduled).

### 3.3 Sequencing rationale

Wave A before B where they touch: `fields.md` is what makes the Lyapunov certificate,
the DP table and the LQR value one object, and `geometry-module.md` is what lets
`PurePursuit` and the racecar demos stop rebuilding the oval by hand; both are designed
and agreed, so they are the cheapest large wins. Wave B's correctness steps (P2, P3) come
before any GRO501 notebook is written, as the plan already said; P4 (estimation) needs the
disturbance-convention decision first, so that decision heads §6. Wave C follows A because
identification and the robust problem need the parameter-dictionary rule (A5). Wave D is
standing work. v1.0 holds only questions that need two cohorts of evidence.

## 4. The textbook pass

### 4.1 Method

A five-band audit (core; blocks and control; dynamics; analysis and simulation; planning)
read every module against RULES §5 with `dp.py` as the reference and reported
line-level findings in seven categories: `self.` in math lines or the equation on the
`return` line (5.3), `np` where `xp` is bound (5.2), hints on equation signatures (5.1),
underscore methods on System and tool classes (5.8), preambles and first-screen order
(5.9, 5.10, 5.23), copy comments and equations living in docstrings (5.22), and shadow
state (5.7). The T0 pass below took the findings that are mechanical and
behaviour-preserving; the rest is the ledger of §4.4.

Every edit followed the AGENTS recipe: a seeded baseline of 864 probes (every touched
equation path on NumPy and on JAX where it traces, every catalog plant's `f`, `h`, `tf`,
ports and matrices, the controllers' laws, the analysis verbs, simulations on both
backends, the hybrid loop, trajectory optimization on both backends, LQR, DP, RRT, Monte
Carlo on both backends and four RL algorithms at a fixed seed) was captured twice and
`cmp`-identical before any edit, and re-captured after each band: **byte-identical after
all four bands**. `ruff` and the bands' test files pass after each band.

### 4.2 What landed (T0)

| Band | Files | What changed |
| --- | --- | --- |
| core | `costs`, `sets`, `geometry`, `kinematics`, `system`, `feedback`, `diagram`, `wiring`, `signals`, `composition` | named equations with `self.` unpacked (SumCost, ScaledCost, SingletonSet, Union.sdf, `apply`, `e = r − y`, the stacked `dx`); seven preambles to a title; the hello-world binds `xp`; `init_wiring`, `refresh_solver_info`, `subsystem_params` |
| blocks + control | `siso`, `lqr`, `modelbased`, `robotic`, `impedance`, `neural`, `geometric`; `sources`, `routing`, `nonlinear`, `filters`, `step`, `transfer_function`, `neural`, `basic` | the PID law readable in `f` / `ctl`, its helpers plain-named with the equations as comments; the Riccati equations beside the solves; the robotic and model-based laws unpack the plant and state their equation once; `as_dof_vector` public across its four importers; ten more renames; Step, ZOH, rate limiter, pure pursuit, neural policy named; seven titles; interpolators declared in `__init__` |
| dynamics | the two mechanical bases, `manipulator`, 19 catalog modules, one test line | 14 title docstrings added, 7 preambles trimmed; named equations in pendulum, cart-pole, three-body, suspension, mountain car, propulsion, boat, plane (coefficients unpacked), dynamic bicycle, racecar (drive and servo equations as comments), UR5 (RNEA/ABA, Jacobian), arms; `trig`, `lengths`, `link_lengths`, `absolute_trig`, `slip_ratio`, `kinematic_chain`, `rnea`, `aba` |
| analysis + simulation + planning | `analysis/__init__`, `linear`, `linearize`, `discretize`, `lyapunov`; `simulator`, `hybrid_simulator`; `transcription`, `policy_synthesis/lqr`, `spatial/state_fields`, `scene`, `track`, `evaluation`, `problems` | `P` named at the Lyapunov equation, `V`, `V̇`, extents and rate unpacked; poles and horizon named; Euler and RK4 steps as three beats with `source` public; `A = ∂f/∂x …` beside the code; `t` / `n_pts` / `solver_mode` before the branch; six hybrid names and eight `PlanningProblem` helpers plain; trapezoid, defect, RK4 knot step, cost-to-go, path and corridor fields, clearance, margin named; six titles |

Also in this pass, outside style: the six example files that failed `ruff` (lint only:
unused imports, one of them python-control, a duplicate mid-file import, an import sort, a
trailing blank line).

### 4.3 Verification

`ruff check .` and `ruff format --check .` clean on the whole repository. Full `pytest`
after the pass (Python 3.11, JAX, no plotly / meshcat / pygame / sympy / Ipopt / dot):
1 273 passed, 30 skipped, 3 packaging tests deselected; the one failure was a renamed
ROADMAP anchor in a plan doc, fixed in the follow-up commit. The regression gates
and the notebook checks were not run in this environment (no Ipopt, plotly, meshcat,
pygame); the touched paths are all covered by the baseline above and by the unit files.

### 4.4 The ledger: what the audit found and T0 did not do

Everything below is behaviour-preserving in intent but needs either a move that makes a
diff hard to read, a rename that ripples into examples or tests, a decision, or a fix that
is not style. It is the content of T1–T6 in the workboard, in priority order per band.

**core (T1)**

- `system.py`: the four default stubs (`System.h`, `DynamicSystem.f` / `h`, `StepSystem.h`)
  use `np.zeros(self.p)`; `StepSystem.step` returns `np.asarray(x).reshape(self.n).copy()`
  on the return line (would fail on a tracer). Changing them to `xp` changes the returned
  array type under JAX, so it is a small decision, not a mechanical edit. No `# Public API`
  / `# Internal machinery` sections; `validate_output_dependencies` sits among port
  methods.
- `feedback.py`: `ErrorDriven.add_error_ports` creates `port_layout` and `error_port`
  outside `__init__`; `error_input` probes with `getattr(block, "error_port", None)`;
  `blocks/transfer_function.py` sets `port_layout` by hand. Interleaved order (roles,
  registry, functions, classes). **[ask — core]**
- `wiring.py`: hasattr-and-create of `subsystems` / `connections` (both constructors
  already set them); `init_wiring` and `compute_state_properties` assign attributes
  instead of returning them; `_composition_entry` / `_composition_output` written from
  `composition.py` at eleven sites; the flag-guarded `print` in `connect`.
- `diagram.py`: **bug, not style**: `StepDiagramSystem.step` assigns in place
  (`x_new[start:end] = …`) on an array built with `xp`, which breaks under JAX. Needs its
  own test and fix.
- `signals.py`: four private helpers before `VectorSignal` (~115 lines to move).
- `kinematics.py`: the `SE3` and `inv` docstrings carry the block forms; fine as API text,
  but the module's `identity` / `translation` could name their results too.
- `hybrid_diagram.py`: `_simulator`, `_cache_result`; `last_result` / `traj` created in
  `__post_init__` without `field(init=False)`. `hybrid_composition.py`: two public
  functions under `# Internal machinery`, one private cross-module import.
- `composition.py`: see T6.

**blocks and control (T2)**

- `control/mpc/controller.py`: see T6. `control/mpc/__init__.py`: 12-line run recipe
  docstring. `control/mpc/utilities.py`: `_shift_plan_trajectory` opens the module (and is
  imported by `test_mpc.py`), `_finite_diff_knots`, `_clamp_tau`; a stale line pointer in
  a docstring.
- `control/robotic.py`: the joint law still lives in `impedance_joint_torque` rather than
  in `ctl` (kept, one call); `TaskImpedance`'s class docstring carries the law.
- `control/impedance.py` and `control/lqr.py`: docstrings of `lqr_gain_schedule`,
  `trajectory_lqr` and `riccati_step` still state equations; harmless as API text.
- `blocks/sources.py`: `WhiteNoise.h` and `TrajectorySource.h` cannot trace
  (`scipy.interp1d`, `float(t)`); `Source.show_signal` is inventory item 7 (D2).
- `blocks/transfer_function.py`: `tf` uses `np` and `float` and drops `params`; the
  build-then-clear port construction is P9.
- `blocks/nonlinear.py` and `blocks/routing.py`: the `# weighted sum …` comment restates
  `y = signs @ stacked`.
- `control/neural.py`: `action_port_of` sits above the class (imported by four modules).
- `control/geometric.py`: `np.shape` on `params["waypoints"]` (shape metadata; harmless).

**dynamics (T3)**

- `manipulators/arms.py`: six module helpers above the first class (~85 lines to move);
  `_set_planar_reach_camera` and `from_manipulator` set attributes from outside
  `__init__`; `link_lengths` probes with `hasattr` (and `self.l` is never set); every
  `forward_kinematics` / `J` reads `self.params` and ignores its `params` argument (a
  behaviour change to fix, with a test); `np` in `_planar_joint_positions` and in the
  3-link `tf`; 18 inline `array_module(...)` calls.
- `pendulum/cartpole.py`: `_configure_cartpole` sets nine attributes on the plant from
  outside `__init__` (one caller; inline it); `RotatingCartPole.tf` on `np`; the comments
  above `xp = array_module(...)` in `RotatingCartPole.C` / `g` belong above the equation.
- `mass_spring_damper/linear.py`: every `A` / `B` / `C` / `D` builder uses `np.array`
  with `params` values, so a traced `params` family cannot reach the course's first plant.
  This is a JAX-safety fix as much as style; it needs the both-backends test extended.
- `vehicles/dynamic_bicycle.py`: `_u_in` (overridden in `racecar.py` and in two project
  files) and `_contact_fields` (called from a project file and read by
  `graphical/catalog/skins.py` through `getattr`); `_visual_*` attributes; dead
  `_wheel_rectangle_pts`; the class starts at line 71.
- `vehicles/racecar.py`: `_u_in`; `PUBLIC_RACECAR_PARAMS` and `_frames` above the class;
  the chassis hooks inside the core block of `f`.
- `manipulators/ur5.py`: six helpers and the two algorithms above the public API (~200
  lines to move).
- `aerial/plane.py`: `Plane2D.tf` on `np` and `params = self.params`; a 55-line `__main__`
  with a pygame session (5.18).
- `abstraction/state_space.py`: four helpers above `StateSpaceSystem`; `LTISystem` stores
  `_A` … `_D`.
- 37 inline `array_module(q).array(...)` calls across the catalog instead of a bound `xp`.
- `vehicles/mountain_car.py` and `pendulum/double_pendulum.py`: `np` inside `tf` /
  forward kinematics.

**analysis and simulation (T4)**

- `analysis/lyapunov.py`: `region_of_attraction` at line 266 behind the two records; the
  method validation, `u_bar` coercion and default resolution inline before the math (an
  `options_of`-style helper); `verify` and the rollout still read `self.rate`,
  `self.extent`, `self.P`, `self.level`, `self.poles` in their math; `EVEN` / `ODD`
  constants mid-file; bare `import jax` (5.12); `draw_region` temporarily changes the
  user's `sys.x0`; `sample_in_ellipsoid` returns its equation on the return line.
- `analysis/linearize.py`: the 20-line nested comprehension that builds `D`; the early
  return that builds `C = I`, `D = 0`; no section headers.
- `analysis/linear.py`: `_interpolate`, `frequency.py` and `time_response.py`'s remaining
  return-line expressions (`w, linear.frequency_response(...)`, the overshoot formula,
  the rise time); the docstring equations that repeat the comments.
- `analysis/modal.py`: `return np.linalg.eig(A)` (naming it would change the returned
  `EigResult` into a tuple, so it is a small decision); `animate_modal` repeats the
  operating-point defaults inline.
- `analysis/structural.py`: rank computed inside the `return`.
- `analysis/derivatives.py`: no section headers; the finite-difference fallback inline.
- `analysis/discretize.py`: `# Public API` after the three classes; the `isinstance` /
  integrator checks inline in `discretize()`.
- `simulation/simulator.py`: the 47-line solver table and `_time_grid_is_uniform` above
  the class; `select_time_vector` / `select_solver` before `solve`; the detached comments
  in `solve`; a ~90-line constructor that does not read as named steps.
- `simulation/computer.py`: `as_computer` before the classes, with 20 lines of
  `isinstance` coercion inline; `reset` probes `_latch` with `getattr`.
- `simulation/solvers/scipy_ivp.py`: `_finalize_solution` above the public methods; six
  restating or mislabelled step comments. `rk4_fixed.py`: two copy comments.
  `euler_fixed.py`: helper above the class.
- Two owners of one constant: `solver_warnings._DISCONTINUOUS_AUTO_DT_SCALE` and
  `simulator.DISCONTINUOUS_AUTO_DT_SCALE` (5.6).
- `static_simulator.py` and `simulator.py` reach into the evaluator's `_u_nominal`.
- Doc bug: `time_response.step_response` says five time constants, `settling_horizon`
  uses eight.

**planning (T5)**

- `trajectory_optimization/planner.py`: the primary class at line 163 behind three
  dataclasses and a helper; eleven underscore methods (`_make_callback` is referenced by
  three test lines); `getattr(self.transcription, "supports_parametric", False)` and
  five more probes that `Transcription` makes redundant; `_USER_OPTIMIZER_METHODS`
  duplicates `optimization/optimizer.py`; the `self.problem` swap-and-restore in
  `solve_trajectory_from`.
- `trajectory_optimization/transcription.py`: `trajectory_cost` still returns the Bolza
  sum on the return line; the primary class at line 182; `getattr(evaluator,
  "has_trace_tier", False)`.
- `direct_collocation.py` / `shooting.py` / `multiple_shooting.py`: 7 + 7 + 3 underscore
  methods; `self.options.t(problem)` / `dt(problem)` inside the objective and defect
  math; the JAX closures duplicated verbatim in collocation; a redundant
  `isinstance(X0, SingletonSet)` in shooting; `ShootingTranscription` referenced by no
  preset.
- `policy_synthesis/discretizer.py`: fourteen underscore methods on the exported
  `StateSpaceGrid`; `x_next` / `action_ok` / `x_next_ok` only created under
  `precomputed` (start them as `None`); dead `_validity_masks` with an `isinstance` on the
  sets (25 lines); `on_grid` with `self.` on the return line; the JAX closure's
  `self.sys.f`.
- `policy_synthesis/policy_eval.py`: two underscore methods, `_policy_from_block`; the
  fixed-policy Bellman update has no step comment. `lookup_policy.py`: `action` on the
  return line. `approximation.py`: three return-line equations, `self.pairs` in the math.
- `search/rrt.py`: thirteen underscore methods (four referenced by tests);
  `_search_callback` set in `solve_trajectory` and read back through `getattr`;
  `isinstance(Xf, SingletonSet)`; the class at line 120. `search/rrt_star.py`: eight
  underscore methods (one in a test), five dead option probes, `isinstance` on the
  extenders, the rewiring-radius formulas on return lines; private cross-module imports
  from `rrt`. `search/extenders.py`: `getattr(U, "box", None)` instead of
  `U.bounding_box()`. `search/plotting.py`: `hasattr(planner, "_path_states")` across
  modules; `# Internal machinery` twice. `search/steering.py`: `self.speed`,
  `self.radius` in the math.
- `evaluation.py`: the primary class at line 140; the scoring contract has no step
  comments in `score_trajectory`; `sys.x0` mutated at the call site (restored in
  `finally`); the public `nominal_trajectory` after the helpers.
- `problems.py`: `ProblemParameters` above the primary class; no section headers;
  `isinstance(X0, SingletonSet)` in the defaults. `comparison.py`: `compare` before
  `Comparison`; `_maybe_show` duplicated in three modules. `initial_guess.py`: helpers
  before the public functions.
- `reinforcement_learning/policy.py`: seven return-line equations (`mu + sigma * eps`, the
  log-densities, the entropy); `over_states` above the class. `critics.py`, `optim.py`,
  `collect.py`, `algorithms/sac.py` / `ppo.py` / `actor_critic.py` / `reinforce.py`:
  `self.gamma`, `self.tau`, `self.vf_coef`, `self.learning_rate` inside the loss and
  update math; `algorithms/base.py` fills `policy` / `critic` / `gamma` in `bind()`.
  `environment.py`: `self.` throughout `step`. `planner.py`: `verbose=True` default
  prints (ruled to stay 2026-09-15; recorded).
- `spatial/paths.py`: `object.__setattr__` of three undeclared cached fields on a frozen
  dataclass (5.6, 5.7); return-line equations; `distance` and `project` share ten lines.
  `spatial/collision.py`: `_BoundCollisionBody`, `_normalize_shapes` between the class
  and the functions, `getattr(self._sys, "m", 0)`. `spatial/plotting.py`: helpers
  interleaved with public functions; `plot_track` duplicates `overlays._track_boundaries`.
  `shaping.py`, `workspace_fields.py`, `grid.py`, `dubins.py`, `steering.py`, `tree.py`,
  `metric.py`, `edge.py`, `live_plot.py`: docstring trims and module-level underscore
  helpers only.

**the two big ones (T6, a conversation first)**

- `core/composition.py`: 24-line section-map docstring (trimmed today to five lines);
  five private helpers before the public API; `_return_signal` and the alias
  `_close_with_junction = feedback` between public functions; `StandardFeedbackWiring`,
  `resolve_standard_feedback` and `default_computer_boundary_ports` buried under
  `# Internal machinery`; 45 module-level underscore helpers, one imported by
  `hybrid_composition.py` and by `test_graphics.py`; eleven writes to
  `_composition_entry` / `_composition_output`; and the three semantics of `@` that P10
  documents.
- `control/mpc/controller.py`: seventeen underscore methods on the mixin and the three
  System classes; eleven pseudo-private attributes declared twice; `hasattr` / `getattr`
  probes and `block._replan_divisor = d` set from `export_mpc_dual_rate_computer` (and
  from `examples/projects/pathtracking/mpc_v1/mpc_dual_rate.py`); the factory
  `ModelPredictiveController` at line 340; six ad-hoc section headers and no
  `# Internal machinery`.

## 5. Decisions requested

1. ROADMAP §1's three edits (the close-out pointer, the foundation objects in v0.2, the
   v1.0 row) — confirm or amend; §1 is yours.
2. R2: add `dev` to the trigger branches of `test.yml` and `docs.yml`.
3. R1: register the PyPI trusted publisher and tag `0.1.0`.
4. Lift the 2026-09-07 hold on P4 (estimation) for v0.2, after the disturbance-convention
   decision (§6 of ROADMAP).
5. T6: whether `composition.py` and `mpc/controller.py` get their pass now or after wave A.
6. The `StepDiagramSystem.step` in-place write under JAX: a fix with a test, agent lane,
   unless you want to look first.
