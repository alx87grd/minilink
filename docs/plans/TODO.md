# Minilink workboard

Every open step of the path to v1.0 in [ROADMAP.md §5](../../ROADMAP.md#5-the-path-to-v10),
by rung, in order. Strategy, milestones and maturity stay in ROADMAP; design writeups in this
folder; rulings and audits in [docs/reviews/](../reviews/); pyro parity rows in
[pyro-port-remaining.md](pyro-port-remaining.md). **Landed rows leave this file**: the
history is the git log and the dated reviews.

Each step is sized for roughly one agent-hour to one agent-day and ends with a "done when"
gate. **[ask]** marks maintainer-owned territory (student-facing material, core
architecture, main-tool APIs, any feature or public-name removal): propose, get a yes, then
land. Unmarked steps are agent-managed: land, then report. Every step runs
`ruff check . && ruff format --check .` and the relevant `pytest
tests/unittest/test_<domain>.py`, and updates DESIGN / README only where a public contract
changes. Behaviour-preserving steps follow the AGENTS refactor recipe (seeded baseline,
byte-identical after). A rename migrates every course notebook in the same commit, with no
alias, while the name freeze is paused between terms (ROADMAP §4.1 gate 7).

`scan: band#n` cites an entry of the 2026-09-22 improvement scan: the evidence is in
[docs/reviews/2026-09-22-improvement-scan-ledger.md](../reviews/2026-09-22-improvement-scan-ledger.md),
the triage in [2026-09-22-improvement-suggestions.md](../reviews/2026-09-22-improvement-suggestions.md).

| Section | Rung |
| --- | --- |
| [§1](#1-v02-wave-a--the-textbook-objects) | v0.2 wave A — the textbook objects |
| [§2](#2-v02-wave-b--gro501) | v0.2 wave B — GRO501 |
| [§3](#3-v02-wave-d--textbook-code-minimal-demos-consolidation) | v0.2 wave D (into v0.3) — textbook code, minimal demos, consolidation |
| [§4](#4-v03--gmc714-catalog-parity) | v0.3 — GMC714, catalog, parity |
| [§5](#5-v09-freeze-candidate-and-v10) | v0.9 freeze candidate and v1.0 |
| [§6](#6-later-ideas) | Later ideas |

---

## 1. v0.2 wave A — the textbook objects

Design: [naming.md](naming.md). Runs in parallel with wave B.

- [ ] **A4 Naming quick wins** of [naming.md](naming.md), each a few lines plus a test: the
  class name as the default `name`; informative default names where a shortcut says
  `Diagram`; `id` documented on `System`. (The closed-loop name and the role names are P10's
  AC-3 and AC-2.) Done when composed diagrams' keys, params dictionaries and trajectories are
  byte-identical before and after.
  Also: one colour keyword, `jmax`, across DP, tabular RL, `PolicyEvaluator` and
  `Comparison`, with S54's clipping (scan: planning#3, examples#5); `plot_cost2go` keeps its
  name to v1.0 and its rename to `plot_cost_to_go` (the name the solution and `Comparison`
  already use) is on V4's list; the operators' named forms exported together (scan: core#9).
- [ ] **A5 Later nouns, each with its first consumer.** `log_prob` on the distributions;
  parameter-dictionary support on sets and distributions (one flatten rule, with
  identification or the robust problem); `UnionSet` via `|` when reachability or multi-goal
  work needs it; vector bounds on `Saturation`; library sets, shapes and fields reading
  `params` (same rule as A2, after it); `PlanningProblem.hamiltonian()` only if the course
  teaches Pontryagin; the RL critic as a `Field` once training is over.
  Also: `Trajectory.from_rollout(x0, xs, us, dt)` for scanned rollouts, V1's first consumer
  (scan: examples#9); the double-integrator homework's two verbs, entry time into a set and
  the Bellman residual (scan: examples#10).
- [ ] **RN-2** of [randomness.md](randomness.md): distributions read `params`,
  `Gaussian(cov=)`, every `sample` takes a key.

---

## 2. v0.2 wave B — GRO501

Plan: [gro501-classical-control.md](gro501-classical-control.md). Release contract:
[ROADMAP §4.2](../../ROADMAP.md#42-v02--gro501-end-to-end).

**Order:** TB-b next (S61, S62 and S73 landed 2026-10-15); P10's AC-0, AC-2a, AC-OP, AC-3,
AC-2b/c and AC-4 have landed, P4 with them; TB-b (its loop part after AC-3). P11, the two
GRO501 notebooks, was dropped on 2026-10-15: the students write the APP work themselves. P6
stays held for GRO501; its row is in §5 (v0.9).

- [ ] **P10 Automation by convention** ([automation-by-convention.md](automation-by-convention.md),
  decided 2026-10-11, amended 2026-10-13 by
  [the second-opinion record](../reviews/2026-10-13-automation-by-convention-second-opinion.md)):
  standard port names drive the wiring, three reserved names (`plant`, `controller`,
  `observer`) are the roles every tool reads, and the cost is scored on the plant. Its
  steps, in build order, are the plan's §7, each with its "done when":
  - AC-0 no controller classes, AC-2a the resolver, AC-OP the operators, AC-3
    composition on standard names and AC-2b/c the one-time rename and the ports `d` / `n`
    (all landed 2026-10-13; the MPC's `u_ff` → `u` trails with AC-8a, plan §10). Wave B
    of P10 is done.
  - AC-4 the loop inputs and `observer=` (landed 2026-10-14; rulings R18, R19); AC-5
    plots and AC-6 animation, in the plotting lane; AC-7 the cost on the plant, v0.3; AC-8a
    the sampled loop keeps in step, alongside; AC-8b with S31, v0.9.

  Related rows: TB-b's loop part and T6's composition half follow AC-3; S66 lands after
  AC-4; RN-4 and RN-5 build on AC-2, AC-4 and AC-7; S70 turns P10's refusals into
  `WiringError`; S31 brings AC-8b. The automatic `dt` and input-hold findings stay with S65.
- Later, not v0.2: `sensitivity(T)` reading the roles of one loop diagram instead of
  `plant=, controller=` ([the 2026-10-15 analysis record](../reviews/2026-10-15-linear-analysis-rulings.md),
  last note).
- [ ] **TB-b The control objects and the loop read like the textbook.** The audit, review
  and pass of TB-a on `control/siso.py` (P / PI / PD / PID / Lead / Lag), `control/state.py`,
  `control/design.py`, `blocks/transfer_function.py`,
  `dynamics/abstraction/state_space.py` (`LTISystem`) and the loop-building path students use
  (`feedback`, `closed_loop`, `@`; after AC-3). Before v0.2 ships. Done when every module in
  scope reads like `dp.py` and the baseline `cmp`s.
- [ ] **P4 follow-ups** (P4 landed 2026-10-14; design and rulings in
  [estimation-and-state-feedback.md](estimation-and-state-feedback.md)): named input ports
  on the state-space base so observers and sensitivity blocks share one `f` (scan:
  analysis#13); the review of the `observer` role against the real blocks when the EKF
  lands ([automation-by-convention.md](automation-by-convention.md) §10). `closed_loop(observer=)`
  landed with AC-4 (2026-10-14). Dropped, by decision (rulings R20): a `kalman(...,
  disturbance=, noise=)` reading `B_w`, `D_v` and the noise blocks' `psd`; the design verb
  takes the textbook `Q_n`, `R_n`.
- **P6** held for GRO501 (teach the Arduino law with `discretize` + simulation, unless the
  sommatif examines z-plane analysis); its row is in §5, v0.9.

---

## 3. v0.2 wave D — textbook code, minimal demos, consolidation

Standing work through v0.2 and into v0.3, behaviour-preserving, one module per step.

**Order:** the S69 follow-up now; T1–T7 in order, with T6 after P10's AC-3; D1.1, D1.2, then D1.3
file by file; D2, D3 and S65–S68 as picked.

**T — the textbook pass.** The AGENTS recipe (seeded baseline in the scratchpad,
byte-identical after; ruff and the module's tests). The findings, file by file, are in
[docs/reviews/2026-09-22-consolidation-review.md](../reviews/2026-09-22-consolidation-review.md);
`planning/policy_synthesis/dp.py` is the reference.
**Status (2026-10-16):** the textbook layout of TB-b and T1–T7 landed in the second-run pass
(PR #125, [record](../reviews/2026-10-16-textbook-pass.md)), except `control/mpc/controller.py`
(T6), which waits on the MPC mixin's subclass contract. The pass changed no behaviour and no
backend. What remains in the rows below are their behaviour, backend and API items, each
recorded in the PR with its question or bug.

- [ ] **T1 core**: `system.py` default stubs on `xp` (a returned-type decision under JAX) and
  section comments; `feedback.py`'s `ErrorDriven` shadow state and order **[ask — core]**
  (`ErrorDriven` stays a port-layout mixin when the controller classes go, P10 AC-0);
  `wiring.py`'s hasattr-and-create, attribute-assigning helpers and the `_composition_*`
  writes; `signals.py` helpers below the class; `hybrid_diagram.py` /
  `hybrid_composition.py` names and sections. Bug found by the audit, its own test first:
  `StepDiagramSystem.step` assigns in place on a JAX array.
- [ ] **T2 blocks and control**: `control/mpc/utilities.py` order and names
  (`_shift_plan_trajectory` is imported by `test_mpc.py`), the MPC package docstring;
  `blocks/transfer_function.py`'s `tf` on `np` without `params`; `TrajectorySource.h`
  cannot trace (`interp1d`); `action_port_of` above the class; the joint-impedance law
  still in a helper.
  Also (scan: control#2, control#5, control#7): one time-interpolation for trajectories and
  gain schedules; one warm-start helper on the plan `Trajectory`; the impedance and robotic
  law bodies consolidated; `mpc/viz.py`'s `HybridSimResult` import under `TYPE_CHECKING`
  (RULES 3.2; scan: control#12).
- [ ] **T3 dynamics catalog**: `manipulators/arms.py` (helpers below,
  `_set_planar_reach_camera` and `from_manipulator` setting attributes from outside
  `__init__`, `link_lengths` probing with `hasattr`, FK / J reading `params` — a fix with a
  test), `pendulum/cartpole.py` (`_configure_cartpole` inlined into `CartPole.__init__`,
  `RotatingCartPole.tf` on `xp`), `mass_spring_damper/linear.py` (`A` / `B` / `C` / `D` on
  `xp` — a JAX-safety fix; extend the both-backends test), `vehicles/dynamic_bicycle.py`
  (`_u_in` / `_contact_fields` renames ripple to `racecar.py`, two project files and
  `graphical/catalog/skins.py`; dead `_wheel_rectangle_pts`), `manipulators/ur5.py` (helpers
  and the two algorithms below the public API), `aerial/plane.py` (`tf` on `xp` reading
  `params`; the 55-line pygame `__main__`), `abstraction/state_space.py` (helpers below), the
  37 inline `array_module(q).array(...)` calls.
  Also: `Manipulator` kinematics defaults raise instead of returning zeros, with
  `link_lengths` the base contract (scan: dynamics#7).
- [ ] **T4 analysis and simulation**: `analysis/lyapunov.py` (primary function first,
  ceremony in a helper, `verify` / rollout unpacked, `EVEN` / `ODD` beside `METHODS`,
  `require_jax`, `sample_in_ellipsoid` named), `simulation/simulator.py` (solver table and
  helper below the class, `solve` before its helpers, the detached comments, a constructor of
  named steps), `simulation/computer.py` (`as_computer` below the classes, its coercion in a
  helper), the solver backends' comments and `_finalize_solution`, the duplicated
  `DISCONTINUOUS_AUTO_DT_SCALE`.
  Also: one uniform-grid check shared by the fixed-step backends and the simulator (scan:
  compile-sim#14).
- [ ] **T5 planning**: `trajectory_optimization/planner.py` (primary class first, eleven `_`
  methods — `_make_callback` in three test lines —, the `getattr` probes
  `Transcription.supports_parametric` makes redundant, the duplicated optimizer-method
  tuple), `transcription.py` (`trajectory_cost` named, primary class first),
  `direct_collocation.py` / `shooting.py` / `multiple_shooting.py` (17 `_` methods,
  `self.options.*` out of the math, the duplicated JAX closures),
  `policy_synthesis/discretizer.py` (fourteen `_` methods, dead `_validity_masks`,
  branch-only attributes start as `None`), `policy_eval.py`, `lookup_policy.py`,
  `approximation.py`, `search/rrt.py` and `rrt_star.py` (21 `_` methods, five in tests;
  `_search_callback` set in `__init__`; the dead option probes; `extenders.py` asking
  `U.bounding_box()`), `evaluation.py` (primary class first, the scoring contract as step
  comments), `problems.py` and `comparison.py` order, `reinforcement_learning/policy.py`,
  `environment.py`, `critics.py`, `optim.py`, `algorithms/*` (`self.` out of the loss and
  update math; `base.py` shadow state), `spatial/paths.py` (undeclared cached fields),
  `collision.py`, `plotting.py`. `ReinforcementLearningPlanner(verbose=True)` keeps printing
  by default (ruled). The parametric optimizer's backend table (OPW-1–OPW-5) changes
  behaviour, so it is a D3 row of its own.
- [ ] **T6 the two big ones** **[ask first]**: `core/composition.py` (after P10's AC-3: the
  section-map docstring, the `_` helpers, the public API buried under `# Internal
  machinery`) and `control/mpc/controller.py` (seventeen `_` methods, hasattr-and-set from
  `export_mpc_dual_rate_computer`, the factory at line 340). Each is a conversation, then a
  step ladder of its own.
  Also: one record per MPC tick, `Command` and `MPCTickSolve` folded into the
  `PlanningSolution` (scan: control#4). The MPC port rename (`x` + `r` → `u`, extras `x_ff`
  and `z`) and the dual-rate composite's `u` are P10's AC-3 and AC-8a. The shape proposed for
  the planner contract (three `Planner` verbs, `z` on the tick's record,
  `validate_mpc_planner` reading them) is [mppi.md](mppi.md) §6; MP-4 there is this step's
  second consumer, and the trajopt MPC baselines are its safety net.
- [ ] **T7 graphical** (scan: graphics#13): the graphical band joins the textbook pass
  (renderers, catalog skins, signal plots), same recipe.
- [ ] **Rename pass, the rest**: the remaining `_method` names on `System` subclasses not
  covered by T1–T6 → plain names (maintainer style rule). **[ask per module]**

**D1 — demos to the rule** (RULES 6.1 / 6.10 / 6.11; the census is comment FD-C14 of the
foundations review, docs/reviews/2026-09-15-foundations-review.md).

- [ ] **D1.1 Native reporting the demos hand-roll** (agent lane, plotting): an `Optimizer`
  convergence plot (`optim_plot.py`); two planners' trees or solutions side by side
  (`rrt_holonomic_obstacles.py`); a `rollout_batch` family plot (`rollout_param_family.py`);
  a 3-D phase plot (`lorenz_attractor.py`). Each one `plot_*` or `__str__` with a test;
  DESIGN §7 lists them.
  Also: `print` on `StepRollout` and `HybridSimResult`, one validator shared with
  `Trajectory` (scan: core#7); `print(controller)` shows the gains (scan: control#10);
  several trajectories on one phase plane and two fields on one grid surface (scan:
  examples#6).
- [ ] **D1.2 The flatness ratchet** in `test_teaching_imports.py`: no top-level `def` or
  utility class in `examples/demos/`, no notebook cell mixing an API call with matplotlib
  code; today's offenders allowlisted, the list shrinking with D1.3.
  Also: teaching code stops rebinding `sys` (scan: examples#7); one owner for the
  teaching-facade list the two teaching tests share (scan: tests-ci#7).
- [ ] **D1.3 The sweep**, one file per step, maintainer reviews each diff **[ask per file]**:
  the tutorial chapters (`11_reinforcement_learning.ipynb` and `showcase_jax.ipynb` first;
  `showcase_jax` rewritten, awaiting review), then the non-clean demos, then the teaching
  notebooks; the three `compile/` demos wait for V1 (their helpers *are* that feature);
  `manipulator_eom.ipynb` reviewed for which of its 26 helpers the text teaches. Done when
  the D1.2 allowlist is empty and the notebook checks pass.
  Also: the placeholder sections of tutorials 01, 02, 03, 05, 08 and 09 filled (scan:
  examples#3); the facade gaps the import allowlist records closed (scan: examples#13).
- [ ] **D1.4 The planning demos and notebooks on the solution verbs** **[ask]**: the
  `planner.solve().trajectory` one-liners, the hand-rolled comparisons and the per-file
  `@dataclass` rows become `solution.plot_*` and `compare(...)` (a runnable draft of
  `vi_pendulum_lqr.py` exists).
  Also: `Planner.plot_solution` is the solution's own `plot_trajectory` (scan: planning#7).

**D2 — consolidation picks** still open from
docs/reviews/2026-09-05-consolidation-inventory.md (nothing removed without the maintainer's
pick):

- [ ] `Source.show_signal` (80 lines of bespoke matplotlib; the same picture is
  `source.plot_trajectory(tf=…)`), with the sources demo and `__main__` that exist only for
  it (scan: graphics#14) **[ask — user-callable]**.
- [ ] Dead modules `planning/spatial/overlays.py` (retires with A3) and
  `experimental/symbolic/mechanics/utils.py` **[ask — importable names]**.
- [ ] MPC debug figure → `control/mpc/viz.py`.
- [ ] `HybridDiagram` hand-copied facades (with S31).
- [ ] Plotting in eight homes: write the placement rule in DESIGN first, then move
  `graphical/port_map.py` next to `control/` if agreed.
- [ ] DESIGN §8's package-roles table (a stale copy of §3) and the DESIGN research-lane trim
  (scan: docs-gov#6, docs-gov#10).
- [ ] The three catalog rows the dynamics scan names (scan: dynamics#13).
- [ ] `JaxMechanicalSystem` retired, the `xp` base already traces (scan: dynamics#14)
  **[ask — importable name]**: the name stays to v1.0 and leaves on V4's list, with no alias
  before.
- [ ] `NeuralNetwork` folded into `MLP` (scan: graphics#12) **[ask — importable name]**.
- [ ] One `ctl` for the model-based laws; `LookupTableController` beside the other laws
  (scan: control#8, control#13).

**D3 — hardening rows** (research lane, small):

- [ ] RRT `KinodynamicExtender` ignores `problem.params.system`; `extenders.py` probes
  `getattr(U, "box", None)` instead of `U.bounding_box()`.
- [ ] MPC port computes drop `params`; dual online-params façades.
- [ ] `ShootingTranscription` orphaned from the string presets.
- [ ] `ParametricMathematicalProgram` / `JaxParametricProgramEvaluator` placement (54 %
  duplicate of `optimization/evaluators/jax_evaluator.py`;
  [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md)).
- [ ] `HybridSimulator` conventions drift (shared input coercion, positional
  `input_port_id`, the framed verbose panel; the plant sub-step from the plant time constant
  instead of one RK4 step per tick; scan: compile-sim#4, compile-sim#5).
- [ ] **OPW-1–OPW-5** The parametric (MPC) optimizer goes through `Optimizer`'s backend table
  ([optimizer-parametric-wiring.md](optimizer-parametric-wiring.md); scan: planning#9). Out of
  T5, which is byte-identical, because it changes behaviour. Done when
  `optimizer_method="ipopt"` reaches the parametric MPC path and the plan's OPW-4 tests pass.
- [ ] `CostDensityField` / `WorkspaceField` export decision (with A3).
- [ ] **S43** Constructor-derived `camera_scale` hints (boat, steering, propulsion, plane,
  arms, rotating cart-poles) → auto-fit or params-derived; raw ground `CustomLine`s in catalog
  skins → `ground_line()`. Also: a renderer capability table with honest fallbacks (scan:
  graphics#7); the physics owns the drawn lengths (scan: dynamics#9).
- [ ] **S53** Profile `rollout_batch` with a `params` family: 278 ms vs 27 ms for the plain
  batch (pendulum, 1000 × 1000 RK4 steps). Done when the family path is within 2× of the
  plain batch. Also: gate the quoted batch-rollout claim; batch the static-leaf time grid
  (scan: compile-sim#11, compile-sim#12).
- [ ] RRT* tracks goal nodes incrementally; the transcriptions lower the state set
  member-wise (scan: planning#6, planning#12).
- [ ] **Third-round notes from the 2026-09-23 fix reviews** (small, or design questions;
  details in docs/reviews/2026-09-22-improvement-suggestions.md §2.4): `step_diagram % dt`
  adds boundary ports to a user `StepDiagramSystem`; the sampled `@` copy of a one-block
  plant diagram drops a diagram-level function output port; a `%` override that ends in
  `as_computer(self, schedule)` would recurse, and `System.__mod__` calls a private helper;
  `RealtimeSimulator`'s offline automatic `dt` reads the hint without `refresh()`; an
  explicit `backend='jax'` on a NumPy-only set or cost raises a raw trace error;
  `discretize(diagram, params={'dt': v})` is refused although a diagram would keep its
  blocks' params, and a raising params setter can leave a diagram half-updated;
  `JaxMechanicalSystem` keeps the integer-state dtype defect (it retires on V4's list);
  assigning into `q.labels[i]` in place no longer sticks; the gravity hook's signature is
  read on every call; the collocation demo's SLSQP fallback is not run in CI (its manifest
  entry still requires Ipopt).
- [ ] Bare `import jax` in `analysis/lyapunov.py` and
  `trajectory_optimization/parametric_evaluator.py` → `require_jax()` (RULES 5.12).

**S65–S69 — one owner per rule, contracts as tests, students' laptops:**

- [ ] **S65 One owner per rule** (scan: compile-sim#0, compile-sim#1, compile-sim#2,
  compile-sim#3, compile-sim#6, compile-sim#7, compile-sim#8, compile-sim#10, compile-sim#13,
  planning#0, planning#5, planning#8, planning#11, core#8, core#10). One item per step: the
  automatic `dt` as `auto_dt(sys)` in `time_grid.py`, with the discontinuous scale made
  finer or DESIGN's promise dropped (the duplicated constant goes with it); the
  try-JAX-then-NumPy policy in `compile_auto` with a typed `NotTraceableError`; the
  forced-input hold as one `input_interp` option every solver obeys (the automatic solver
  choice changes the hold model today); what `n_steps` counts on each verb; the one-tick lag
  inside a `Computer` tested and stated, or removed; `has_trace_tier` a boolean;
  `simulation` stops importing `optimization` for display constants; `scipy_stiff` with
  explicit tolerances and a Jacobian; `compile(verbose=True)` in order; one set of Monte
  Carlo defaults for `MonteCarloEvaluator` and `Planner.evaluate`; flat-kwarg keys derived
  from the option dataclasses; one owner of the sets between `StateSpaceGrid` and its
  planner; one print-text rule for the textbook objects; one verbose and empty-cache rule
  across the `compute_*` facades.
- [ ] **S66 Wiring mistakes fail at wiring time** **[ask — core]** (RULES 4.10; scan:
  core#2, core#13): `connect` refuses to rewire a connected subsystem input; the compile
  probe detects an output that moves with an input it does not declare (raise or warn:
  ROADMAP §6). After P10's AC-4 (landed 2026-10-14): the loop-input junctions still rewire
  the connected input they sum into (`composition.sum_into`), so S66 changes them first.
- [ ] **S67 One plot vocabulary** (scan: graphics#0, graphics#4, graphics#5, graphics#6,
  graphics#8, graphics#9, analysis#9, analysis#10): `title`, `ax`, `backend` and `show`
  honoured by every plot verb, one `PlotResult.axes` shape; `plot_trajectory` selects a
  leaf's named output ports; one axis-label formatter, one unit convention, one backend
  resolver; the renderers silent (RULES 4.6); a warning on the ±10 phase-plane window; one
  keyword set for the five control plots; the region plot overlays trajectories without
  touching `sys.x0`.
- [ ] **S68 Contracts as tests** (RULES 6.12; scan: tests-ci#3, tests-ci#5, tests-ci#9,
  tests-ci#10, tests-ci#11, tests-ci#14, docs-gov#1, docs-gov#2, examples#8, control#11,
  planning#10, dynamics#11): the RULES 5.8 underscore check a repo-wide ratchet with a
  shrinking allowlist; ROADMAP §5 step ids and TODO rows agree, and no plan doc numbers its
  steps with a bare workboard-style id (scan: docs-gov#3); the CI regression flags agree
  across `test.yml`, `tests/run/_common.py` and the tests/README Agent table (scan:
  tests-ci#4); the subprocess demo-check tests behind a marker; wall-clock claims out of
  unit tests; unit tests stop importing
  `examples/projects` and `benchmarks/systems`; `Animator.resolve_frame` public for the
  graphics harness; course pins byte-identical to their topic twins; one both-backends test
  over every control law, the catalog with perturbed `params`, and the `params.sets`
  scoring parity.
- [ ] **S69 follow-up**: the `laptops` job (Windows and macOS legs) and the Linux coverage
  report landed; confirm the first remote CI run is green before the GRO501 cohort installs.

---

## 4. v0.3 — GMC714, catalog, parity

**Order:** A1–A3 (the textbook objects) before wave C; G1 first in wave C; RN-4 and RN-5
together with P10's AC-7, between cohorts; S72 after them; V1 after A2, its Monte Carlo part
after RN-5.

**The textbook objects.** Design: [fields.md](fields.md), [cost-params.md](cost-params.md),
[geometry-module.md](geometry-module.md). Baselines first, as each plan states.

- [ ] **A1 `core/fields.py`** **[ask — core]**. Steps 4.1–4.8 of [fields.md](fields.md):
  promote `StateField` → `Field` on `(x, u, t)` with `as_constraint` / `as_input_constraint`
  / `as_cost` and `gradient`; `QuadraticField` (Lyapunov `V`, LQR value, `S(t)` schedule)
  with the ellipsoid as its sublevel set; `GridField` (DP and tabular tables, per-sweep time
  axis); `CallableField` (RL critics); `LinearApproximator` as a field;
  `LyapunovCertificate.V` a field; DESIGN §4 fields bullet (step 4.7, the CBF plan, is
  done). Done when the RoA and DP baselines `cmp` and `test_analysis_lyapunov.py`,
  `test_planning_solution.py`, `test_rl_tabular.py`, `test_rl_planner.py` pass.
  Also: `PolicyEvaluator` takes the evaluators' verb and returns a field-shaped result
  (scan: planning#4).
- [ ] **A2 Cost parameters** **[ask — core]**. [cost-params.md](cost-params.md): `params` on
  every library cost (`QuadraticCost`, `TimeCost`, `FieldCost`, `SumCost`, `ScaledCost`),
  composite costs nested like diagram params with `cost.id`, attributes as views so the
  notebooks' in-place writes keep working; then the trajopt, DP, evaluator and RL paths pass
  `problem.params.cost` through. Done when every planner baseline `cmp`s and `jax.grad` of a
  trajopt objective with respect to `Q` runs.
  Also (scan: core#3, core#4): `discount_rate` a constructor field (or a `params` entry)
  that `ScaledCost` forwards and `SumCost` reconciles; `validate_diagram_params` checks each
  per-block dict against the block's keys, so a partial dict fails at the setter instead of
  inside `f`. The Monte Carlo evaluator and the gym reward never apply `problem.params.cost`
  while the DP table does, so the two disagree on a parametric cost.
- [ ] **A3 Workspace geometry** **[ask — core]** (S57 was its earlier id).
  [geometry-module.md](geometry-module.md) implementation steps 1–6: `core/geometry/`
  package (shapes, paths, track, scene, probes, `bind`, spatial fields, grid, catalog
  `oval_circuit` / `racecar_circuit` / `holonomic_forest`); shaping next to `Field.as_cost`;
  plots and overlays under `graphical/` with lazy methods on `Scene` / `Track`;
  `planning.spatial` re-exports then deletes in the same change as the call sites;
  `PurePursuit` takes a Path / Track. Done when the UdeS racecar trio and the RRT forest demo
  use the catalog, `control` imports geometry with no planning import, and the suite is
  green. Glyph rename stays S30.
  Also: `Shape` unions flatten like `IntersectionSet` and `SumCost` (scan: core#6);
  `PurePursuit` sizes its measurement from the vehicle, not `state_dim=9` (scan: control#9);
  retarget `control/mpc/viz.py`'s `TrackCorridorOverlay` import in the change that deletes
  `planning.spatial.overlays` (scan: control#12).

**Wave C.**

- [ ] **G1 GMC714 baseline audit** **[ask — course scope]**. Run every topic of ROADMAP §4.3
  (vehicle models, robotic arm, nonlinear control, robust control, trajectory optimization,
  MPC) on the teaching surface, as the GRO501 audit did
  ([docs/reviews/2026-09-07-gro501-coverage.md](../reviews/2026-09-07-gro501-coverage.md));
  write the gaps as §4.3's gates and as rows here. Decide whether MPC joins the teaching
  surface. Done when every §4.3 row has a status and a gate.
- [ ] **C1 Pyro parity** open rows — [pyro-port-remaining.md](pyro-port-remaining.md); then
  the pyro → minilink migration guide in README from the name map there. **[ask]** for the
  README. Re-audit the table against the code first (scan: docs-gov#11).
- [ ] **C2 GMC714 modelling ladder**: manipulators + the four-rung vehicle ladder as a
  `02_dynamics` lesson; robotic PID wrappers (`JointPD`, `EndEffectorPD` twins).
  Also: one constructor contract for the robotic laws (scan: control#6).
- [ ] **C3 Blocks**: `Ramp` / `Chirp` / `Delay` / `Switch`; `Integrator` and `ZOHHold` take
  a `dim` like every static block (scan: graphics#11).
- [ ] **C4 Identification and generation**: `identification/fitting.py` on `rollout_batch`
  (physical params and NN weights are one verb); `trajectory_generation/` port (polynomial,
  min-snap); SMC trajectory-following demo; trajectory post-filter; experiment-design
  helpers (the PRBS / chirp inputs live in `blocks/sources`).
  Also: the simulated `Trajectory` logs `dx` and `y`, so an equation-error fit has its data
  (scan: analysis#14).
- [ ] **C5 RL follow-ups**:
  - [ ] **S50 SAC actor step** **[ask]**: pre-update critic (today, kept for bit-identity) or
    updated critic (SB3, CleanRL). One line in `algorithms/sac.py`; re-measure
    `pendulum_swing_up_rl.py` if changed.
  - [ ] **S51 Policy iteration on the grid** (course ch. 12) as a `DynamicProgrammingPlanner`
    mode or sibling. **[ask — planner API]**
  - [ ] **S52 Deep Q-learning** (course ch. 17): a discrete-action head over the grid's input
    levels, `QFunction` with replay and a target network. Research lane first.
  - [ ] **Approximate dynamic programming** on `policy_synthesis/approximation.py` (fitted
    value iteration); design first, it touches the planner API. **[ask]**
  - [ ] `Sys2Gym` takes a `Distribution` for the start state (scan: graphics#1;
    randomness.md RD-R5, step RN-5).
- [ ] **S63 DP reads the horizon** **[ask — planner API]** (scan: planning#1): a finite
  `problem.tf` runs `round(tf / dt)` backward sweeps as `solve_steps` does, the way
  `LQRPlanner` picks the Riccati ODE, or warns once. Done when LQR and DP agree on the
  finite-horizon double integrator.
- [ ] **S64 Catalog hygiene** **[ask — catalog]** (scan: dynamics#0, dynamics#1,
  dynamics#2, dynamics#3, dynamics#4, dynamics#6, dynamics#8): the mechanical default
  bounds as ROADMAP §6 decides (keep the invented ±2π / ±5, or each plant states its own); output ports that are
  the state or a slice of it read labels and units from the state (the 56 copy lines go);
  one port layout across the bicycle rungs; one owner for the wheelbase; the hidden `0.01`
  damping in `Drone2D.d` and `Rocket.d` named; `VanderPol` with a real input or none;
  constructor hygiene in the pendulum and mass-spring-damper families.
- [ ] **RN-3 / RN-4 / RN-5** of [randomness.md](randomness.md) (its §9 holds the files and
  "done when" per step): RN-4 `realize(key)` by name and signals on `disturbances=` (the
  frozen-noise defect fixed), RN-5 the Monte Carlo evaluator as a batched simulation of the
  closed-loop diagram on its test set; together, between cohorts, before the v0.9 freeze,
  with P10's AC-7. RN-3 `NoiseSource` with its first consumer. RN-6, the contract in DESIGN,
  goes with each step.
  Also: the evaluator's `simulator` backend fails on a diagram plant with a nested-diagram
  error (randomness.md §1).
- [ ] **AC-7 of P10: the cost scored on the plant** across every evaluator and design tool
  ([automation-by-convention.md](automation-by-convention.md) §5, §7), with RN-4 and RN-5.
- [ ] **S72 The path-integral planner (MPPI)** **[ask — name, MPC block contract]**, after
  RN-4 and RN-5, on the evaluator's batched rollout over realizations. Design:
  [mppi.md](mppi.md); the standalone prototypes `examples/experimental/mppi/pendulum_mppi.py`
  and `racecar_mppi.py` run today. MP-1 `planning/trajectory_optimization/path_integral.py`:
  `PathIntegralPlanner` / `PathIntegralRecord` (working names; the name is the ask), `solve`
  and `solve_trajectory_from` on
  `RolloutEnvironment` (`jit(vmap(scan))` over `K` samples), the six-beat body of the plan's
  §5, `decision_dimension` and `warm_start_guess`; MP-2 the hand-loop demo
  `examples/experimental/mppi/pendulum_mppi.py`; MP-3 the stochastic branch
  (`n_plant_samples` realizations per control sample, mean or CVaR by the problem's
  criterion) and the fan of sampled futures as `solution.plot_samples()`. Done when the
  seeded-determinism, LQR-agreement and swing-up tests pass, the deterministic twin of the
  stochastic branch is byte-identical, and the body reads next to `dp.py`. MP-4 (the MPC
  block reads the planner verbs: T6) and MP-5 (demos, `compare(MPC=, MPPI=)`) before v0.9;
  MP-6 (the `loop` backend, facade names, DESIGN §6) with V3.
- [ ] **LQGPlanner** **[ask — planner API]**: `LQGPlanner(problem)` on a
  `StochasticPlanningProblem` with a `QuadraticCost` and `disturbances` on `d` / `n`, beside
  `LQRPlanner`: the policy is the `lqg` box, the record holds `K`, `L`, both Riccati matrices
  and the poles, the cost-to-go the certainty-equivalent form; `Q_n = B_w W B_wᵀ`,
  `R_n = D_v V D_vᵀ` from `linearize_matrices(sys, wrt="d")` / `wrt="n"` and the blocks'
  `psd`. With RN-4. Design: [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §5.
- [ ] **V1 The differentiable closed-loop cost** **[ask — core]** (API and architecture not
  decided). One fully differentiable JAX path: controller params → closed-loop dynamics → RK4
  step → the run on a horizon → the cost `J` → the mean of `J` over stochastic rollouts, every
  parameter an input (controller, plant, cost (A2), sets, `x0`), behind one standard entry
  point; teaching surface once mature. Today tutorial 11 and `pid_autotuning_jax` write that
  scan by hand. In place: the params-traced closed-loop run (`rk4_integrate_zoh_trace_p`; the
  grad of a PID gain through `PID() @ Pendulum()` matched finite differences to 8 digits on
  2026-10-12). Missing: the plant's `(x, u)` along a traced run, one traced cost integral, the
  Monte Carlo path with the controller's params as an input (RN-5 and AC-7 are its
  neighbours). Done when the pathwise gradient of the mean closed-loop cost with respect to
  the controller's params matches a central difference on the same draws, through the
  standard entry point.

---

## 5. v0.9 freeze candidate and v1.0

ROADMAP §5.4 and §5.5: v1.0 means a colleague builds a course on minilink and trusts it for
three years. The foundation rows are design conversations before code.

**Order:** the foundations (S31, S29, S37, S44, S30, V3) before the trust contract; V11 once,
in December 2026 after the v0.3 tag; the v1.0 rows after two cohorts.

### v0.9 — what could still break a name

- [ ] **S31** `HybridDiagram` → a `System` (state `[plant; computer]`, periodic discrete
  update) or an honest `HybridLoop` rename; `%` stays the one hybrid operator either way; the
  hand-copied facades go with it; P10's AC-8b lands with it.
  Also: the sampled seam's time argument, ticks or seconds (ROADMAP §6; scan: control#3); the
  run-state caches, decided here: RULES 2.4 allows only `self.traj`, while
  `StepSystem.rollout` and `HybridDiagram.last_result` cache a run too (ROADMAP §6); the
  `core` → `simulation` import (scan: core#14).
- [ ] **S29** `DiagramSystem.x0` / `n` / `state` as derived properties (mirror the live
  `params` view); `Simulator` drops its pre-read `refresh()`.
  Also (scan: core#11, dynamics#10, examples#14): `x0` (the simulation start) and
  `state.nominal_value` (the operating point x̄, ruled 2026-10-13) are two quantities with
  one owner each; a diagram derives both from its blocks; the racecar solver hint, `WhiteNoise` and the diagrams' bubbled
  solver hints stop depending on the pre-read `refresh()`.
- [ ] **S37** Evaluator / solver re-layering: evaluators keep pure maps and one scannable
  step; integrators move to `simulation/solvers/`. Then **S27** Diffrax as an optional JAX
  solver backend.
  First, the evaluator internals deduplicated (ZOH sugar, the JAX step rollout, the four
  Jacobian probes, the undeclared batch cache; scan: compile-sim#9).
- [ ] **S44** A single posed-geometry hook so "two functions" (`f` + drawing) is literal;
  today `tf` + skin.
- [ ] **S30** Rename the graphical `Sphere` / `Box` glyphs so no two importable public types
  share a name with the geometry solids. **[ask — public names]**
- [ ] **V3 The frozen surface** **[ask — public names]**: list the names the v1.0 freeze
  covers. `StepSystem`, `StepDiagramSystem`, `ZOHHold` (root prelude), `control.mpc`
  (control facade) and the spatial names A3 moves to `core/geometry/` (provisional teaching
  names, ROADMAP §2) join it or move behind a visibly provisional name. Done when the
  teaching-surface registry (`tests/unittest/test_teaching_surface.py`) marks each name
  frozen or provisional and the API site shows the mark.

### v0.9 — the trust contract

- [ ] **V4 Deprecation policy** **[ask]**: from v1.0, a teaching-surface removal or rename
  ships one minor release as a `DeprecationWarning` shim (RULES "no deprecated aliases" is
  amended for v1.0 on). A snapshot of the export list is a test; changing it fails until the
  snapshot and the CHANGELOG line move together. Done when RULES, ROADMAP §2 and the
  snapshot test agree.
  The rename list, applied with shims under the policy (the names hold until then, with no
  alias before): `xbar` / `ubar` → `x_bar` / `u_bar`, the name everywhere else, on the costs
  (`QuadraticCost.from_system(xbar=, ubar=)`, `cost.xbar`, `cost.ubar`, `TimeCost`'s
  `xbar`); `plot_cost2go` → `plot_cost_to_go` (A4); `JaxMechanicalSystem` retired into the
  `xp` base (D2).
- [ ] **V5 `CHANGELOG.md`**: user-facing, one entry per release from v0.9 (back-filled for
  0.1.0 / 0.1.1 from ROADMAP §2's breaking changes); `pyproject.toml` `Changelog` URL points
  there. Done when `publish.yml` refuses a tag without its CHANGELOG heading.
- [ ] **V6 The pinned Colab cell** **[ask — student-facing]**: conda from `environment.yml`
  stays the default supported install and README leads with it; `pip install minilink` is the
  Basic alternative in `install.md` (both in place). What remains: the Colab setup cell
  installs a pinned release (`%pip install minilink==X.Y`) instead of `git clone main`;
  `COLAB_CLONE` in `tests/unittest/test_repo_contract.py` follows. `showcase_jax.ipynb`
  imports `minilink.experimental.c_export`, which the wheel does not ship: settle it here.
  Done when every notebook carries the pinned cell and runs in Colab.
- [ ] **S70 `MinilinkError` family** **[ask — core]**: `WiringError`, `ShapeError`,
  `SolverError` subclassing the `ValueError` / `TypeError` raised today (no caller breaks);
  S66's wiring-time checks and P10's refusals raise them. Done when the wiring and compile
  paths of `core/` raise only the family.
- [ ] **V11 Rewrite the history without notebook outputs** **[ask — force-push]**, in
  December 2026 after the v0.3 tag, between terms. A fresh clone is 176 MB for 19 MB of
  files: 169 MB of the history is `.ipynb` outputs committed before the `nbstripout` hook
  worked, mostly five notebooks deleted by May (`examples/notebooks/demo*.ipynb`,
  `animation_colab.ipynb`). `main` carries it all, so deleting branches or `archive/*` tags
  does not help. Steps: keep a bare mirror clone as the backup; `git filter-repo` stripping
  outputs from every `.ipynb` blob (or dropping the deleted notebooks' paths); re-point the
  release tags `0.0.1`–`0.1.1` and the `archive/*` tags; force-push branches and tags; the
  re-clone notice to students and collaborators, and the line in the CHANGELOG (V5). PyPI
  releases are untouched. GitHub keeps the old objects behind `refs/pull/*` until its support
  purges them; clones shrink regardless. Done when `git clone` of the repo is under 25 MB and
  CI is green on the rewritten `dev`.

### v0.9 — the syllabus holes

- [ ] **P6 Discrete (z) tier** **[ask — public names]**, if v0.9 takes it (proposed yes,
  ROADMAP §6); held for GRO501, which teaches with `discretize` + simulation unless the
  sommatif examines z-plane analysis. Exact ZOH and Tustin `c2d` on `LTISystem`, z-plane
  `pzmap`, discrete `step_response` and `bode` (`analysis/`), checked against
  `scipy.signal.cont2discrete`; an exact zero-order-hold option on `discretize`, shared with
  `step_response` (scan: analysis#1). Done when a digital PID and a discrete state feedback
  run in a topic notebook. Two follow-ups:
  - **[ask]** keep the remaining `params["dt"]` fallback (`dt=` omitted, read from the
    `params=` dict given to `discretize`) or make `dt` required;
  - a source whose `y` feeds through from a port other than `u` does not discretize
    (`PendulumWithNoisePort`), because the wrapper stacks every port into one `u`: map `y`'s
    dependencies to `("u",)`, or keep the source's ports.

  **Future work, sampled controllers with continuous states (`PID % dt`)** **[ask]**: `%`
  raises `NotImplementedError` for them (P10). Supporting them means choosing the
  discretization method (Euler, Tustin, RK4) and whether `%` discretizes or the user calls
  `discretize` first. It needs `discretize` to keep the controller's ports first.
- [ ] **Estimation follow-ups** after P4, each in the slot
  [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §7 names: the
  finite-horizon filter (`kalman_finite_horizon` → a time-varying observer, the dual of
  `lqr_finite_horizon`, the forward sweep on the dual pair with `riccati_step`); the
  linearized Kalman filter along a trajectory (`trajectory_kalman` → `TrajectoryObserver`,
  the dual of `trajectory_lqr`); the extended Kalman filter (`ExtendedKalmanFilter(model, Q_n,
  R_n, x0, P0)`, `P` a state, Jacobians taken on the model's `f`, `h` inside its own `f`);
  the sampled filters (unscented, discrete Kalman, particle) after AC-8a / S31 and P6; the
  `EstimationProblem` / `EstimationSolution` and the `*Estimator` tools (§5 of the plan);
  online parameter estimators (recursive least squares, gradient laws) under a role of their
  own. Each revisits the observer conventions of
  [automation-by-convention.md](automation-by-convention.md) §10.
- [ ] **S71 The classic first plants**: DC motor, tank / thermal first-order-plus-delay
  process, ball and beam, differential drive, 3-D quadrotor, each with `tf`, a skin and the
  catalog both-backends contract test; LQI beside the `N` matrix of P8. Done when each has a
  one-line demo in `examples/demos/dynamics/`.
- C4 (identification, trajectory generation) moves here if GMC714 does not land it (§4).

### v0.9 — the reference site

- [ ] **V7 The docs site as a guide**: myst-nb (or nbsphinx) renders `examples/tutorial/`
  from stored outputs; a plant gallery (one image per catalog plant from
  `docs/make_assets.py`); a concept page (System, composition, tools as verbs). Done when
  the Sphinx `-W` build on `docs.yml` publishes all three.

### v1.0 — a colleague can build a course on it

- [ ] **V8 Textbook-validation suite**: `tests/unittest/test_textbook_examples.py`, worked
  examples from Dorf & Bishop, Ogata, Franklin–Powell asserted numerically (margins, step
  specifications, root locus, LQR and Kalman gains), each citing its example number; GRO501
  gate 3 is the seed. Done when the README links it as the correctness evidence.
- [ ] **V9 Course-neutral labs** **[maintainer]**: `examples/teaching/topics/` grows from 7
  to 10–15 labs (objectives, prerequisites, time, starter and solution), classical control
  and robotics first; a "For instructors" page (pin a version, Colab, adapt a lab, report a
  bug).
- [ ] **V10 The public face** **[ask]**: the README opens on a teaching quickstart; TRL,
  lanes, wave and step ids leave README and `examples/README.md`; the agent and governance
  files stay at the root or move under `docs/dev/` (ROADMAP §6); `CONTRIBUTING.md`, issue
  templates, a code of conduct.
- [ ] **V2** Zenodo archive and a citable DOI (`CITATION.cff`) after PyPI; the JOSS entry
  and the RULES 6.9 carve-out for its state-of-the-field section, decided then.
- [ ] **S49 The Stable-Baselines3 teaching notebooks** **[ask]**, at the end of the term.
  Four, in `teaching/courses/udes_gro860/`: retire `drone_ppo_sb3.ipynb` and
  `pendulum_value_iteration_vs_lqr_vs_ppo_sb3.ipynb` (delete them, their notebook overrides
  and allowlist entries, and the README rows marked Stable-Baselines3) once the course notes
  point only at the native twins; decide `particle_gymnasium_sb3.ipynb` and
  `gymnasium_interface.ipynb` then.
- [ ] **Realtime architectural review** **[ask — core]**: `RealtimeSimulator` and the pygame
  I/O (`simulation/realtime/simulator.py`, `io.py`, each marked `TODO: User Architectural
  Review`) reviewed before the freeze; the band joins the V3 surface or stays provisional.
  Done when both markers are gone.

### After v1.0 — 1.x features, additive

- [ ] **S32** Unify `MechanicalSystem` / `GeneralizedMechanicalSystem` (`N = I` special
  case); `Boat2D` / `Plane3D` on the mechanical base, with `y` and `x` like every mechanical
  plant (no `q` / `dq` ports); `DynamicBicycle` on `GeneralizedMechanicalSystem` (scan:
  dynamics#5).
- [ ] **S36** iLQR planner from parts (`jacfwd` of `f_trace`; idea, research lane).
- [ ] **General serial chain**: one mechanism description (DH or URDF) feeding RNEA / ABA
  beyond the UR5 and the symbolic path, the shared serial-chain stack and ABA on the other
  RNEA arms included — [articulated-mechanism.md](articulated-mechanism.md).
- [ ] **Behavior-cloning pipeline** **[ask]**: a game mode on `RealtimeSimulator` that records
  human demonstrations (keyboard or joystick driving the plant live) as trajectory datasets; a
  planner that fits a policy to them (imitation beside the RL planner, `NeuralPolicyController`
  as its output); the cloned controller closed on the plant and compared against the
  demonstrator. The pyro game demos, if ported (ROADMAP §6), are its first scenes.

---

## 6. Later ideas

One line each; open a plan doc only when a design needs a writeup.

- Order reduction by neglecting fast modes (out of v0.2 by decision): keep the slow modes and
  the DC gain, a band function in `analysis/modal.py`. Separately, swap the guide's §9.13
  realization into `TestMinreal` and its §1.4.2 parking model into `TestPolePlacement` once
  the matrices are in hand (stand-ins today).
- Automatic reward scaling for the RL planner (`reward_scale="auto"` = `1 / (g_max dt)`): raw
  costs throttle PPO through the joint gradient clip; the scaled planner reaches LQR on every
  seed tested — [rl-reward-scaling.md](rl-reward-scaling.md) (RS-1–RS-4, rung for the
  maintainer).
- The automatic `d` / `n` plant wrapper of P10, deferred until someone needs it
  ([automation-by-convention.md](automation-by-convention.md) §4.3, §10).
- Vehicle view ports (`pose` / `bodyvel` on `DynamicBicycle` for impedance / PID).
- **[ask]** Simon's curve speed scaler in the path-tracking project: `PathTracking` in
  `examples/projects/pathtracking/bicycle_los_v2/path_tracking.py` holds `vx_ref` constant;
  Simon's LOS caps it by the heading change ahead along the path (the weighted sum of
  `|Δψ|` over a look-ahead window, mapped to a speed limit through lookup tables). Source:
  PR #99, `minilink/control/guidance_los_coupled.py` (`compute_curve_demand_norm`,
  `speed_limit_from_curve_demand`). Weigh the tables against the one-parameter textbook cap
  `v ≤ sqrt(a_y_max / κ)` on the sharpest curvature ahead.
- Scene params / `J(z, p)` bind (DESIGN §4 planning-params pipeline B; moving obstacles
  online without rebuilding the NLP).
- `SimulationOptions`, one record for the `Simulator` solver presets (DESIGN §5), weighed
  against RULES 2.5 (no option bags) before it is built.
- A sample stride on the fixed-step solvers (`compute_trajectory(..., substeps=k)`: `k` RK4
  steps per stored sample, the scan keeping only the samples): a stiff model stepped at
  0.5 ms over 40 s stores 80k states today, which is what pushed jaxnet to keep its own
  decimated `lax.scan` rollout beside `Simulator`.
- Cross-fidelity `lift` / `project` maps on the car ladder —
  [fidelity-maps.md](fidelity-maps.md).
- `RobustPlanningProblem` (set-bounded uncertainty, minimax criterion) only when a minimax
  consumer exists; the path-integral planner's `"worst_case"` branch (CVaR over plant draws,
  [mppi.md](mppi.md) §4) would be that consumer; the two decisions go together.
- `RolloutEnvironment` may move from `reinforcement_learning/` to `planning/environment.py`
  once a second band reads it (open in ROADMAP §6) (the path-integral planner; the evaluator and the tabular learners
  already do): one import rewrite, no behaviour change.
- `MjxPlant` (`interfaces/mjx.py`) and `torch` / `flax` model wrappers in `interfaces/`;
  Pacejka tire; stochastic forcing; ROS2 / FMI; sparse long-horizon trajopt; RRT-Connect.
- Ipopt given the Lagrangian Hessian, not the objective's alone (scan: planning#13);
  `optimizer_method="auto"` picking Ipopt when installed (scan: examples#11);
  `PlanningProblem.metadata` documented or retired (scan: planning#14; ROADMAP §6).
- `LowPassFilter(tau=)`: the shaping filter of the colored-noise recipe parameterized by its
  time constant, beside `cutoff_hz` (randomness.md RD-D23, RN-4).
- A fixed-step RK4 that samples held sources once per step, so the four stages read one
  sample and the block route equals the port route (randomness.md RD-A8, RD-D18): fourth order
  with no boundary error at `dt = Δ`, one draw per step instead of four. Worth it only if a
  twin test shows the first-order bias.
- A key check on leaf params (a `validate_params` hook on `System`, no-op by default, called
  where diagram params are validated): today a typo or a retired key in any block's params is
  ignored silently; a question for the whole library.
- A 3-D still of any animation: `save_frame(t)` on the meshcat renderer writes the static
  HTML of one frame. The headless screenshot side exists: `docs/asset_export.py::html_to_gif`
  seeks an exported page frame by frame, so a still is one `seek` and one screenshot. With it,
  two camera notes for S43: the default meshcat eye sits low so the viewer's grid reads as
  horizon lines, and an overlay (a track corridor) drives the auto-fit camera, so a follow
  camera cannot zoom on the car; overlays should opt out of the fit as the ground lines
  already do.
- Declined (do not re-propose): scalar / list signal bounds and a coercing `x0`; scalar `Q` /
  `R` / `S` in `QuadraticCost.from_system`.
