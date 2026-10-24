# Minilink workboard

Every open step of the path to v1.0 in [ROADMAP.md §5](../dev/ROADMAP.md#5-the-path-to-v10),
by rung, in order. Strategy, milestones and maturity stay in ROADMAP; design writeups in this
folder; rulings in [docs/reviews/](../reviews/); pyro parity rows in
[pyro-port-remaining.md](pyro-port-remaining.md). **Landed rows leave this file**: the
history is the git log and the dated reviews.

Each step is sized for roughly one agent-hour to one agent-day and ends with a "done when"
gate. **[ask]** marks maintainer-owned territory (AGENTS "Ask first"); unmarked steps are
agent-managed: land, then report. The gates and the behaviour-preserving recipe are AGENTS'.
A rename migrates every course notebook in the same commit, with no alias, while the name
freeze is paused between terms (ROADMAP §4.1 gate 7).

`scan: band#n` cites an entry of the 2026-09-22 improvement scan; its ledger and triage are in
git history (`docs/reviews/2026-09-22-improvement-*.md`, before 2026-10-17).

| Section | Rung |
| --- | --- |
| [§1](#1-wave-a--the-textbook-objects-carried-from-v02-into-v03) | Wave A — the textbook objects, carried into v0.3 |
| [§2](#2-wave-b--gro501-the-carry-over-020-shipped-2026-10-17) | Wave B — GRO501, the carry-over |
| [§3](#3-wave-d--textbook-code-minimal-demos-consolidation) | Wave D — textbook code, minimal demos, consolidation |
| [§4](#4-v03--a-cleaner-core-and-the-differentiable-path-then-gmc714) | v0.3 — a cleaner core and the differentiable path, then GMC714 |
| [§5](#5-v04--the-draws-and-the-textbook-objects) | v0.4 — the draws and the textbook objects |
| [§6](#6-v09-freeze-candidate-and-v10) | v0.9 freeze candidate and v1.0 |
| [§7](#7-later-ideas) | Later ideas |

---

## 1. Wave A — the textbook objects (carried from v0.2 into v0.3)

Their slots in the order: A4 any time (agent); A5's sets reading `params` with A2 in v0.3 (V1's
step 6), `Set.as_cost` with V1's step 8 in v0.4, its other nouns each with its first consumer; A6
in v0.3's clean-up, after K2, K8 and K10 and before G3 (TODO §4's order, R78); RN-2 before V1's step 7 in v0.4.

- [ ] **A4 Naming, the rest** (the naming quick wins landed 2026-10-17:
  the class name as the default `name`, shortcuts named as the expression they build, `id`
  documented): one colour keyword, `jmax`, across DP, tabular RL, `PolicyEvaluator` and
  `Comparison`, with S54's clipping (scan: planning#3, examples#5); `plot_cost2go` keeps its
  name to v1.0 and its rename to `plot_cost_to_go` (the name the solution and `Comparison`
  already use) is on V4's list; the operators' named forms exported together (scan: core#9).
- [ ] **A5 Later nouns, each with its first consumer.** `log_prob` on the distributions;
  parameter-dictionary support on sets and distributions (one flatten rule, with
  identification or the robust problem); `Set.as_cost(weight=, shaping=)` as the set-to-cost
  dual of `Field.as_cost()` (its first consumer the program bridge's `"soft"` lowering, step 8
  of differentiable-problem.md, after `shaping.py` moves to `core/`; MPPI reads it next);
  `UnionSet` via `|` when reachability or multi-goal
  work needs it; vector bounds on `Saturation`; library sets, shapes and fields reading
  `params` (same rule as A2, after it); `PlanningProblem.hamiltonian()` only if the course
  teaches Pontryagin; the RL critic as a `Field` once training is over.
  Also: the double-integrator homework's two verbs, entry time into a set and
  the Bellman residual (scan: examples#10).
- [ ] **RN-2** of [randomness.md](randomness.md): distributions read `params`,
  `Gaussian(cov=)`, every `sample` takes a key.

---

## 2. Wave B — GRO501, the carry-over (0.2.0 shipped 2026-10-17)

Baseline audit: [the 2026-09-07 coverage record](../reviews/2026-09-07-gro501-coverage.md). Release contract:
[ROADMAP §4.2](../dev/ROADMAP.md#42-v02--gro501-end-to-end).

**Order:** P10's remaining steps (AC-5, AC-6 in the plotting lane; AC-7 v0.4; AC-8 with S31),
then the P4 follow-ups. P11 and P6 were dropped (2026-10-15, 2026-10-17); TB-b landed with the
textbook pass.

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
    plots and AC-6 animation, in the plotting lane; AC-7 the cost on the plant, v0.4 (§5);
    AC-8a the sampled loop keeps in step, alongside; AC-8b with S31, after the continuous path (§5, R86).

  Related rows: S66 lands after
  AC-4; RN-4 and RN-5 build on AC-2, AC-4 and AC-7; S70 turns P10's refusals into
  `WiringError`; S31 brings AC-8b. The automatic `dt` and input-hold findings stay with S65.
- Later, not v0.2: `sensitivity(T)` reading the roles of one loop diagram instead of
  `plant=, controller=` ([the 2026-10-15 analysis record](../reviews/2026-10-15-linear-analysis-rulings.md),
  last note).
- [ ] **P4 follow-ups** (design: [estimation-and-state-feedback.md](estimation-and-state-feedback.md)):
  named input ports on the state-space base so observers and sensitivity blocks share one
  `f` (scan: analysis#13); the review of the `observer` role against the real blocks when
  the EKF lands ([automation-by-convention.md](automation-by-convention.md) §10).

---

## 3. Wave D — textbook code, minimal demos, consolidation

Standing work into v0.3, behaviour-preserving, one module per step.

**Order:** the S69 follow-up now; the T rows' leftovers as picked; D1.1, D1.2, then D1.3
file by file; D2, D3 and S65–S68 as picked.

**T — the textbook pass.** Landed: the textbook layout of TB-b and T1–T7 in the second-run
pass (PR #125, 2026-10-16, [record](../reviews/2026-10-16-textbook-pass.md)), behaviour- and
backend-preserving; its style sheet lives in RULES §5 (5.3's six blocks, 5.8, 5.10, 5.22,
5.25–5.27) and §7.8. The pass's recorded bugs and open questions were ruled the next day and
fixed one commit each with a regression test (PR #126, 2026-10-17,
[rulings](../reviews/2026-10-17-textbook-pass-follow-up-rulings.md)): the plants' `tf` and
geometry read their `params`, the MPC blocks take plain hook names and one mixin
constructor, the wiring core keeps no shadow state, every helper name says its role. The
rows below hold what neither PR touched, each a behaviour, backend or API item verified open
on `dev` at 638215e; the recipe and the reference are AGENTS'.

- [ ] **T1 core**: `system.py`'s default `h` stub builds `np.zeros(p)` (a returned-type
  decision under JAX) **[ask — core]**; `wiring.py`'s `refresh` still guards with
  `hasattr(self, "solver_info")`. Bug found by the audit, its own test first:
  `StepDiagramSystem.step` (`core/diagram.py`) writes `x_next[start:end] = …` in place, which
  a JAX array refuses.
- [ ] **T2 blocks and control**: `blocks/transfer_function.py`'s `tf` calls `self.h(x, u, t)`
  on `np` and drops `params`; `TrajectorySource.h` cannot trace (`interp1d`); `mpc/viz.py`
  imports `HybridSimResult` at module level instead of under `TYPE_CHECKING` (RULES 3.2;
  scan: control#12).
  Also (scan: control#2, control#5, control#7): one time-interpolation for trajectories and
  gain schedules; one warm-start helper on the plan `Trajectory`; the impedance and robotic
  law bodies consolidated.
- [ ] **T3 dynamics catalog**: `manipulators/arms.py` (`from_manipulator` sets `_kin_arm`,
  `name`, `camera_scale` and `camera_target` on the instance from outside `__init__`;
  `link_lengths` probes with `hasattr`), `mass_spring_damper/linear.py` (`A` / `B` / `C` / `D`
  on `np`, so they do not trace — a JAX-safety fix; extend the both-backends test),
  `aerial/plane.py` (the 55-line pygame `__main__`, a demo in disguise, RULES 5.18).
  Also: `Manipulator` kinematics defaults (`forward_kinematic_effector`, `J`) return zeros
  instead of raising, with `link_lengths` the base contract (scan: dynamics#7).
- [ ] **T4 analysis**: `analysis/lyapunov.py`'s `EVEN` / `ODD` slices sit under Internal
  machinery, apart from `METHODS` at the top.
- [ ] **T5 planning**: `trajectory_optimization/planner.py` probes
  `getattr(transcription, "supports_parametric", False)` and keeps the optimizer-method
  table as a `_USER_OPTIMIZER_METHODS` class attribute; `direct_collocation.py` /
  `shooting.py` / `multiple_shooting.py` read `self.options.*` inside their bodies;
  `search/extenders.py` asks `getattr(U, "box", None)` instead of `U.bounding_box()` (also a
  D3 row); `spatial/paths.py`'s `PolylinePath` sets `_seg_lengths`, `_s_knots` and
  `_total_length` through `object.__setattr__` without declaring them as fields.
  `ReinforcementLearningPlanner(verbose=True)` keeps printing by default (ruled). The
  parametric optimizer's backend table (OPW-1–OPW-5) changes behaviour, so it is out of this
  row: it lands with S74 in v0.3 (R74, §4).
- [ ] **T6 the MPC block's contract** **[ask first]**: `export_mpc_dual_rate_computer`'s
  single-rate path still undoes the dual-rate hooks with `hasattr(block, "replan_divisor")`
  and `hasattr(latch, "set_after_solve")`; `validate_mpc_planner` probes the planner's
  verbs with `hasattr`.
  Also: one record per MPC tick, `Command` and `MPCTickSolve` folded into the
  `PlanningSolution` (scan: control#4). The MPC port rename (`x` + `r` → `u`, extras `x_ff`
  and `z`) and the dual-rate composite's `u` are P10's AC-3 and AC-8a. The shape proposed for
  the planner contract (three `Planner` verbs, `z` on the tick's record,
  `validate_mpc_planner` reading them) is [mppi.md](mppi.md) §6; MP-4 there is this step's
  second consumer, and the trajopt MPC baselines are its safety net.

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
  notebooks; the three `compile/` demos wait for V1 (their helpers *are* that feature). Done when
  the D1.2 allowlist is empty and the notebook checks pass.
  Also: the placeholder sections of tutorials 01, 02, 03, 05, 08 and 09 filled (scan:
  examples#3); the facade gaps the import allowlist records closed (scan: examples#13).
- [ ] **D1.4 The planning demos and notebooks on the solution verbs** **[ask]**: the
  `planner.solve().trajectory` one-liners, the hand-rolled comparisons and the per-file
  `@dataclass` rows become `solution.plot_*` and `compare(...)` (a runnable draft of
  `vi_pendulum_lqr.py` exists).
  Also: `Planner.plot_solution` is the solution's own `plot_trajectory` (scan: planning#7).

**D2 — consolidation picks** still open from
the 2026-09-05 consolidation inventory, in git history (nothing removed without the maintainer's
pick):

- [ ] `Source.show_signal` (80 lines of bespoke matplotlib; the same picture is
  `source.plot_trajectory(tf=…)`), with the sources demo and `__main__` that exist only for
  it (scan: graphics#14) **[ask — user-callable]**.
- [ ] Dead modules `planning/spatial/overlays.py` (retires with A3) and
  `experimental/symbolic/mechanics/utils.py` **[ask — importable names]**.
- [ ] MPC debug figure → `planning/trajectory_optimization/mpc/viz.py`.
- [ ] `HybridDiagram` hand-copied facades (with S31).
- [ ] Plotting in eight homes: write the placement rule in DESIGN first, then move
  `graphical/port_map.py` next to `control/` if agreed.
- [ ] The DESIGN research-lane trim (scan: docs-gov#10).
- [ ] The three catalog rows the dynamics scan names (scan: dynamics#13).
- [ ] `JaxMechanicalSystem` retired, the `xp` base already traces (scan: dynamics#14)
  **[ask — importable name]**: the name stays to v1.0 and leaves on V4's list, with no alias
  before.
- [ ] `NeuralNetwork` folded into `MLP` (scan: graphics#12) **[ask — importable name]**.
- [ ] One `ctl` for the model-based laws; `LookupPolicy` beside the other laws
  (scan: control#8, control#13).

**D3 — hardening rows** (research lane, small):

- [ ] RRT `KinodynamicExtender` ignores `problem.params.system`; `extenders.py` probes
  `getattr(U, "box", None)` instead of `U.bounding_box()`.
- [ ] RRT checks `X.contains(sample)` and the goal region without the time: a time-varying `X` or
  `Xf` is read at `t = 0` (`rrt.py`, `rrt_star.py`); the tree nodes need a time (the sum of the
  edge durations) and the checks pass it (found 2026-10-23).
- [ ] `ShootingTranscription` orphaned from the string presets.
- [ ] `HybridSimulator` conventions drift (shared input coercion, positional
  `input_port_id`, the framed verbose panel; the plant sub-step from the plant time constant
  instead of one RK4 step per tick; scan: compile-sim#4, compile-sim#5).
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
  details in the 2026-09-22 improvement suggestions §2.4, in git history): `step_diagram % dt`
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
- [ ] Bare `import jax` in `simulation/compile_backend.py` and `from jax.errors` in
  `analysis/derivatives.py` → `require_jax()` (RULES 5.12); the two rows the textbook-style
  allowlist still carries.

**S65–S69 — one owner per rule, contracts as tests, students' laptops:**

- [ ] **S65 One owner per rule** (scan: compile-sim#0, compile-sim#1, compile-sim#2,
  compile-sim#3, compile-sim#6, compile-sim#7, compile-sim#8, compile-sim#10, compile-sim#13,
  planning#0, planning#5, planning#8, planning#11, core#8, core#10). One item per step: the
  automatic `dt` as `auto_dt(sys)` in `time_grid.py`, with the discontinuous scale made
  finer or DESIGN's promise dropped (the duplicated constant goes with it); the
  try-JAX-then-NumPy policy in `compile_auto` with a typed `NotTraceableError`; the
  forced-input hold as one `input_interp` option every solver obeys (its sampled side: S75) (the automatic solver
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
  planning#10, dynamics#11; the RULES 5.8 ratchet landed as
  `tests/unittest/textbook_style.py` with its allowlist): ROADMAP §5 step ids and TODO rows agree, and no plan doc numbers its
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

## 4. v0.3 — a cleaner core and the differentiable path, then GMC714

**Status (2026-10-24, the overnight run on `dev-v0.3`, pending the maintainer's review):** the
clean-up's renames and moves (K2, K3, K10, A6, G3's moves), the step model (SM-0 to SM-8), the
path to the cost (V1.1 to V1.5 with A2), the optimizer's method table (S74) and the closing PID
example (S77) landed; what each row left is below, and every decision taken without the
maintainer is in [the run's log](../reviews/2026-10-24-v03-run-decisions.md). Left of the
clean-up: K8, S32, K3b (R68 landed: every plant states its own bounds).

**Start here (2026-10-23, after session 5):** every decision the code needs is ruled (R1–R89,
[the record](../reviews/2026-10-22-rulings-session-1.md)). v0.3 is the continuous path: the new
names and the differentiable evaluation of the cost and the criterion; every hybrid piece (the
sampled loop S31, the hybrid and realtime simulators, the parametric MPC solve) waits for v0.4
(R86). The design of record is [continuous-core.md](continuous-core.md),
and [the final review](../reviews/2026-10-23-v03-final-review.md) checked the plan against the
code. The work happens on `dev-v0.3`, a temporary branch the maintainer merges into `dev` after
his review (R79); when it reaches `main` is his call, by the courses (R78).

**The gates of this pass** (R80–R82): where a step restructures a computation, agreement to
`rtol 1e-6` with the baseline is the bar (a pure move or rename stays identical); an intended
number change is checked by running the same problem and the same cost before and after, the
relative differences in the step's report; every rename and keyword change lands with all its
callers, with no alias, shim or facade.

**Order** (the agent's, R78): the names and the structure first, so the new core is written once,
on its final names and paths; then the step model, which the path to the cost builds on; then the
path to the cost; then the optimizer; the closing example last.

1. **The clean-up:** K2, K3 (with K3b's script added to the repository), K8 (K9's last items with
   K8-3), K10 without `spatial/`, A6, S32, G3, R68 (its own commit). K4 is the maintainer's pass,
   any time.
2. **The step model** (S75): SM-0, SM-1, SM-2, SM-7, SM-3, SM-4, SM-5, SM-6, SM-8.
3. **The path to the cost** (V1): the one-day spike of the scoring formula on the pendulum, V1.1,
   V1.2, A2 (with the sets reading `params`; it needs nothing from the engine, so `problem.compile`
   assembles its parameters from the objects on day one), V1.3, V1.4, V1.4b, V1.5.
4. **The optimizer** (S74; its OPW part, the parametric MPC solve, waits with S76, R86).
5. **The closing example** (S77, R85), which needs V1.5 and S74 only.

GMC714 needs no new feature (R48): the former wave G rows (C1, C2, C3, S63, S64) are §5 rows
beside C6 and S76. The rows below are grouped by kind; the order is the list above.

**Wave K — convergence** (November 2026, first, before the continuous core adds new code; K7's
analysis is the ruling table K1 asks for). The goal: a
smaller library that is easier to read and keep — fewer names, fewer demos, prototypes
labelled as such. It extends D2 (whose open picks join K1's table) and runs before D1.3, so
the demo sweep touches fewer files. Nothing leaves without the maintainer's ruling (RULES 7.1).

- K1, the inventory, landed 2026-10-23: [the record](../reviews/2026-10-23-k1-inventory.md)
  (every public name with its callers, maturity and a proposal; the plants; the demos and their
  twins; the draft A6 and G3 rename tables). K2, K3, K4, K8, A6 and G3 rule from it.
- [ ] **K3b Contract tests for the kept prototypes** (agent, R69): every name the K1 inventory labels
  "prototype" and keeps gets a test of its contract (the sets, the blocks, the sensitivity functions,
  `discretize`, the graphics primitives, …). The K1 inventory's script lived in a lost scratchpad: it
  lands in `benchmarks/` first. Done when that script finds no kept public name without a
  contract test.
- [ ] **K4 One demo per feature** **[maintainer]** (R54): the maintainer's own pass on the demos,
  with the K1 inventory's §3 (twins and proposals) as the input; agents update `examples/README.md`'s
  map and the demo checks after each of his changes.
- [ ] **K8 The catalog clean-up** (ruled 2026-10-22, [end-state-api.md](end-state-api.md) D2;
  [record](../reviews/2026-10-22-rulings-session-1.md)): two import layers and support by
  test. Steps, one commit each: **K8-0** the car family to three models (R55: `KinematicBicycle`,
  `DynamicBicycle`, `UdeSRacecarDyn` on the root; `KinematicCar` and `DynamicBicycleCar3D` merged as
  skins, `UdeSRacecar` a parameter preset, `UdeSRacecarDyn3D` a `wheel_angles=True` option, the
  notebooks and demos migrated); **K8-1** the root prelude trimmed to the course set (the
  plants of R55–R60, 16 by R60, no cap; all 50 catalog plants today; the other names per R64; ledger line in ROADMAP §2);
  **K8-2** `minilink.catalog` the one flat index of every supported plant, each passing the
  both-backends contract test, with a docs entry; **K8-3** the unsupported plants the
  maintainer names from K1's table moved to `minilink/experimental/catalog/` with their skins
  and demos (the top-level `experimental/`), their tests and docs entries removed, out of the
  wheel **[ask — per plant]**; **K8-4** twins merged (R55, R59) and the dead helpers deleted
  (R54, R67: the Mermaid exporters, `camera.py`, `ProblemEnv` / `to_gymnasium`, the
  `identification/` stub; the pygame renderer stays, the real-time default); **K8-6** the README's catalog line by domain and the API
  site's plant page. Done when the root prelude lists exactly R60's plants, `minilink.catalog`
  lists only tested plants, no two supported plants differ by parameters alone, and
  `test_teaching_surface.py` pins both lists.
- [ ] **K9 The experimental tier, untested** (ruled 2026-10-22, R3; [end-state-api.md](end-state-api.md)
  D3; [record](../reviews/2026-10-22-rulings-session-1.md)). Landed 2026-10-23: the prototype
  scripts moved from `examples/experimental/` to a top-level `experimental/` (K9-1); the tier's
  tests (`test_symbolic.py`, `test_engine_jax.py`) moved to `tests/experimental/`, off the merge
  gate, run by the nightly job (`pytest tests/experimental`), `c_export`'s flagship smoke kept in
  the JAX regression job (K9-2); the status line on every `minilink/experimental/` module
  docstring (K9-3). Ruled 2026-10-23: `manipulator_eom.ipynb` deleted (it imported the symbolic
  tier); `showcase_jax.ipynb` keeps its quick C-export demo, the one documented exception beside the
  flagship smoke (both on `minilink.experimental.c_export`). Left: the unsupported plants' test rows
  leave with K8-3. Done when CI's gate imports nothing under the tier but those two exceptions,
  and `pip wheel` ships nothing under it.
- K7, what is core and what is a side feature, closed 2026-10-23 (R69): answered by R1–R5 and
  R52–R67; its moves are the K2, K3, K8 and K10 rows.

**The continuous core — the step model** (S75, ruled R6–R8, R12, R17, R18, R40, R42;
[sampled-path.md](sampled-path.md) §1–§7, the API in [continuous-core.md](continuous-core.md)
§3.1–§3.2). **[ask — core]**, ruled; every step agrees with the baseline (R81: `rtol 1e-6` where
the computation is restructured, identical for a move) unless it names its intended change
(AGENTS' refactor recipe; the baseline JSON and the script that writes it under `benchmarks/`,
compared after each step).

- [ ] **SM-6, the rest** (the trim landed 2026-10-23; the integrators stay in `core/compile/`, OD-11 of
  [the run's log](../reviews/2026-10-24-v03-run-decisions.md)): the evaluator internals
  deduplicated (the ZOH sugar, the JAX step rollout, the four Jacobian probes, the undeclared batch
  cache) and the `f` / `f_p` tiers folded (OD-9). Done when the SM-0 baseline agrees (R81).

**The continuous core — the path to the cost** (V1, steps 1–6 and 7a of
[differentiable-problem.md](differentiable-problem.md); the API is
[continuous-core.md](continuous-core.md) §3.3–§3.6 and §4). **[ask — core]**, ruled (R9–R11,
R15, R16, R19–R29, R30–R47). The names are ruled (R26–R28, R45: `cost`, `J`, `objective`). A
sampled controller waits for the hybrid phase; RN-4's per-block noise seeds for v0.4. Each step
reads its math next to `dp.py` before it lands. Nothing merges to `main` until every notebook has
moved to the new interface (R47).

- [ ] **V1.1, the rest** (the native `Trajectory`, the cost's quadrature and `contains` on `xp` landed
  2026-10-24): `StepRollout` folded in as `t = k·dt`, with S31 in v0.4 (it is the hybrid
  simulator's computer record, R86). Done when it lands byte-identical.
- [ ] **V1.2, the rest** (the evaluation of a run, the problem's fields, the price switch and the cost
  bound landed 2026-10-24): LQR warns once when `U` is declared (the U sweep wrote
  `U=plant.input_box` on lessons that never needed it, so the warning waits for the maintainer's
  pass on those lines).
- [ ] **S74, the rest** (the method table and the first-order backend landed 2026-10-24): the hand
  descent loops of `examples/demos/compile/` other than the closing example's first section
  (`cartpole_rollout_gradients`, `neural_controller_jax`, `params_gradient`) moved onto
  `Optimizer` **[ask — student-facing]**: each teaches the hand loop today, and R90 kept that
  loop in two places on purpose; the open-loop schedule needs a source whose samples are
  parameters first.
- Second opinion and final review: every point ruled (R30–R89).

**GMC714** (R48, 2026-10-23): no new feature; the teaching surface covers every lab (ROADMAP
§4.3). The labs move from pyro to minilink as the maintainer's teaching work, outside this
workboard; the former wave G rows are §5 rows.

- [ ] **G3 GMC714's names stable by January** **[ask — public names]** (R49): every tool a lab
  uses has one public teaching name on the teaching surface, frozen for the term (week 1 on
  January 9), after A6's renames (R52: `ComputedTorque`, `SlidingMode`, `Impedance`,
  `ThreeLinkManipulator`, `LongitudinalTorqueCar`). Moves: `RRTPlanner` from provisional to the teaching lane; `JointImpedance` (R63: the class,
  was `ModelJointImpedance`; the factory deleted), `TaskKinematic`, `TaskKinematicNullspace` onto the
  root prelude; a new band facade,
  `minilink.dynamics` (R50, R61): `MechanicalSystem`, `Manipulator`, `GeneralizedMechanicalSystem`,
  `StateSpaceSystem`, `LTISystem`, with `JaxMechanicalSystem` deleted (`MechanicalSystem`'s default
  builders on `xp`); MPC as one teaching name, `ModelPredictiveController` (R50), over today's
  `MPCStatelessController` / `MPCStatefulController`, its constructor frozen while S76 and the
  planner contract (D27) reshape its internals in v0.4, and the loop it runs in (`mpc @ plant`,
  as every demo and lesson writes it) kept as the user's syntax through S31. `ModelPredictiveController`
  already exists as a factory over the two classes, in a research-lane module the teaching-surface
  test rejects: it moves to a teaching-lane module. A6's catalog renames land before the term
  (R50), so G3's list is pinned after them. Each name gets a docstring, a demo and the teaching-surface test
  (ROADMAP §2's soft rule). Done when `test_teaching_surface.py` pins the list, each name has a
  demo under `examples/demos/`, and no GMC714 lab needs a deep import.

- [ ] **S32 One mechanical family** **[ask — core]**, ruled 2026-10-23 (R62), v0.3: `GeneralizedMechanicalSystem` a child of `MechanicalSystem` beside `Manipulator`, `H` the
  one mass-matrix name (`M` renamed in the class, the boat and the plane), `qv2x` / `x2qv` aligned, the
  model-based controllers accepting every mechanical plant; `Boat2D` (and `Plane3D` in the
  experimental tier) with `y` and `x` like every mechanical plant (already so today);
  `DynamicBicycle` on the generalized child (scan: dynamics#5); also `DynamicBicycle.M` (read by the racecar and
  `BicycleDynRate`) and `examples/projects/car_trajopt`'s ladder, and `DynamicBicycle`'s base
  moves from `DynamicSystem` to the generalized child. Done when the boat and the plane pass the
  catalog contract test as children of `MechanicalSystem`, `ComputedTorque` runs on one of them in a
  test, and no class names the mass matrix `M`.

---

## 5. v0.4 — the draws and the textbook objects

**Order** (winter 2027, the window between GRO860 cohorts; the GMC714 term runs meanwhile
and does not gate it, ROADMAP §5's calendar rule): S31 first (moved from v0.3 by R86), then OPW-1–OPW-5; V1's
steps 7–9 of [differentiable-problem.md](differentiable-problem.md) (its steps 0–6 and the
Monte Carlo fold land in v0.3, R44) with RN-3 / RN-4 / RN-5 and AC-7 (step 7 is now RN-4's
per-block seeds under the same verb; RN-2 before it), its step 7b
the sampled face (after S31), C4 its first data consumer; S74 before S76 (S76 builds on its
backend factory), S76 beside them (its names ruled: `ParametricProgram`, the key `plant`); then A1,
then A3; S72 on the sampled kernel after RN-4 / RN-5; C6, C5, the former wave G rows (R48) and
LQGPlanner as picked; S73 any time, self-contained.

**The draws on the path.**

- [ ] **S31 The sampled loop** **[ask — core]**, ruled a tool with ticks (R13, R14; the name
  `HybridDiagram` kept for now; [sampled-path.md](sampled-path.md) §8): one tick-scan kernel on
  the step model, a NumPy and a JAX face, tracing decided by the computer; its callers
  `HybridSimulator` and the MPC export and replay; P10's AC-8b and the hand-copied facades with
  it. Parked until the continuous path lands (the maintainer, 2026-10-22; R86): the run-state caches
  (RULES 2.4), controller memory and the computer's draws on the tick, the evaluator's dispatch
  of a sampled controller (D13–D15), the `core` → `simulation` import (scan: core#14). In v0.4, after the continuous path (R86). Done when
  D13–D15 are ruled and the kernel's steps are rows here.
- [ ] **OPW-1–OPW-5** (after S74, with S76; R86) The parametric (MPC) optimizer goes through `Optimizer`'s
  backend table ([parametric-program.md](parametric-program.md) §6; scan: planning#9). Done
  when `optimizer_method="ipopt"` reaches the parametric MPC path and the plan's OPW-4 tests
  pass.
- [ ] **V1, step 7: the noise seeds on the draws** **[ask — core]** (the fold itself is v0.3,
  V1.4b, R44): `problem.realize(key, n)` gains one seed per random block (RN-4), so a noise block
  inside the loop or on a port draws per trial under the same `evaluate(..., n_trials, key)`; with
  RN-5 and AC-7. Done when two trials from one key draw distinct noise, adding a random block
  leaves every other stream unchanged, and RN-5's LQG loop scores within the covariance
  prediction's sampling error.
- [ ] **V1, step 7b: the sampled face** **[ask — core]** (after S31; D15 parked): `problem.compile(arg)`
  reads the controller's kind; a continuous law runs the loop as a diagram, a sampled controller
  (a digital PID, a held neural policy, MPPI, the RL policies) runs S31's kernel on its JAX face,
  so `trajectory`, `evaluate` and `program` serve both; the v0.3 refusal goes. Done when the
  gradient of a digital PID's `objective` through the kernel matches a central difference,
  and a continuous law's numbers are unchanged.
- [ ] **S76 The parametric program binds every problem parameter** **[ask — core]**,
  proposed 2026-10-22: [parametric-program.md](parametric-program.md), steps PP-0–PP-5
  (PP-0, the names, ruled in V1's step 0 in v0.3). Today the compiled MPC program binds the
  measured `x0` only: direct collocation freezes the plant, cost and set parameters in its
  closures, the online call refuses every other key, and the MPC block's port functions drop
  `params` (this row absorbs that D3 row, the parametric evaluator's placement and the scene
  bind of the Later list). The recommendation (the plan's §2, option C): one family, the
  library building `J(z, p)`, `h(z, p)`, `g(z, p)` on the problem's parameter tree (`x0`,
  plant, cost, sets; the scene with A3), the frozen `MathematicalProgram` its binding, so
  hand-written programs stay `J(z)`, one evaluator body serves both, and the optimizer still
  sees a flat `z`. Direct collocation and the MPC block first (optional live-parameter ports,
  unwired ports at their nominal). Order: PP-1, PP-2 and PP-4 on S74's backend factory
  (OPW-1; S74 lands in v0.3, R74); PP-3, the cost and set leaves, on A2 (v0.3). Done when
  one compiled MPC program, bound with two values of a plant parameter and of a cost target,
  gives the solutions a rebuild gives, the MPC baselines are byte-identical with only `x0`
  bound, and the friction-aware MPC example runs from a port.

- [ ] **RN-3 / RN-4 / RN-5** of [randomness.md](randomness.md) (its §9 holds the files and
  "done when" per step): RN-4 `realize(key)` by name and signals on `disturbances=` (the
  frozen-noise defect fixed), RN-5 the Monte Carlo evaluator as a batched simulation of the
  closed-loop diagram on its test set (step 7 of differentiable-problem.md is that one path;
  RN-4's seeds join it under the same verb when they land); together, between cohorts, before
  the v0.9 freeze, with P10's AC-7. RN-3 `NoiseSource` with its first consumer. RN-6, the contract in DESIGN,
  goes with each step.
  Also: the evaluator's `simulator` backend fails on a diagram plant with a nested-diagram
  error (randomness.md §1).
- [ ] **AC-7 of P10: the cost scored on the plant** across every evaluator and design tool
  ([automation-by-convention.md](automation-by-convention.md) §5, §7). Its deterministic half
  (the plant's run scored, controllers with state, one price) lands with V1's steps 2–4 in v0.3;
  this row is the rest, with the draws (V1's step 7, RN-4, RN-5).
- [ ] **C4 Identification and generation**: identification is V1's program export
  (`pe.program(wrt=["plant.…"])` and the `Optimizer`) with a data-mismatch cost (`identification/fitting.py` on `rollout_batch`; physical params and NN
  weights are one verb), its first data consumer; `trajectory_generation/` port (polynomial,
  min-snap); SMC trajectory-following demo; trajectory post-filter; experiment-design
  helpers (the PRBS / chirp inputs live in `blocks/sources`).
  Also: the simulated `Trajectory` logs `dx` and `y`, so an equation-error fit has its data
  (scan: analysis#14).
- [ ] **LQGPlanner** **[ask — planner API]**: `LQGPlanner(problem)` on a
  `StochasticPlanningProblem` with a `QuadraticCost` and `disturbances` on `d` / `n`, beside
  `LQRPlanner`: the policy is the `lqg` box, the record holds `K`, `L`, both Riccati matrices
  and the poles, the cost-to-go the certainty-equivalent form; `Q_n = B_w W B_wᵀ`,
  `R_n = D_v V D_vᵀ` from `linearize_matrices(sys, wrt="d")` / `wrt="n"` and the blocks'
  `psd`. With RN-4. Design: [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §5.

**The textbook objects.** Design: [fields.md](fields.md),
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
  retarget `planning/trajectory_optimization/mpc/viz.py`'s `TrackCorridorOverlay` import in the change that deletes
  `planning.spatial.overlays` (scan: control#12).

**On the spine.**

- [ ] **S72 The path-integral planner (MPPI)** **[ask — name, MPC block contract]**, after
  S31's sampled kernel (v0.3) and RN-4 and RN-5, its plant the step model (S75), on the evaluator's batched rollout over realizations. Design:
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
- [ ] **C5 RL follow-ups**:
  - [ ] **S50 SAC actor step** **[ask]**: pre-update critic (today, kept for bit-identity) or
    updated critic (SB3, CleanRL). One line in `algorithms/sac.py`; re-measure
    `pendulum_swing_up_rl.py` if changed.
  - [ ] **S51 Policy iteration on the grid** (course ch. 12) as a `DynamicProgrammingPlanner`
    mode or sibling. **[ask — planner API]**
  - [ ] **S52 Deep Q-learning** (course ch. 17): a discrete-action head over the grid's input
    levels, `QFunction` with replay and a target network. Research lane first.
  - [ ] **Approximate dynamic programming** on `dynamic_programming/approximation.py` (fitted
    value iteration); design first, it touches the planner API. **[ask]**
  - [ ] `Sys2Gym` takes a `Distribution` for the start state (scan: graphics#1;
    randomness.md RD-R5, step RN-5).
  - [ ] **The standard stochastic policy as a seeded block** **[ask — core]** (the
    maintainer's direction, 2026-10-22; [future-policies.md](future-policies.md) §5 point 4):
    `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)` with the noise block's counter-based draw, the mean
    at `seed=None`, a seed per experiment from `realize(key)`, the log-probability and the
    entropy as methods beside `h`; the RL Gaussian and squashed-Gaussian heads rebuilt on it,
    the collectors moved from their own keys to per-episode seeds (the RL numbers change once:
    a baseline and a re-pin, in this window). Done when the RL demos re-pin and a policy block
    vmaps over seeds on both backends.
- [ ] **C6 General serial chain** **[ask — core]**: one mechanism description (DH table or
  URDF: geometry, inertia, topology) feeding RNEA / ABA beyond the UR5, the shared serial-chain
  stack, ABA on the other RNEA arms, the symbolic path reading the same description; the GMC714
  arm topic's modelling rung. Design first (the July 2026 articulated-mechanism draft is in git
  history; it is re-planned here, the optional `KinematicModel` delegate decided with it, ROADMAP
  §6). Moved here from wave G on 2026-10-22; the GMC714 audit (2026-10-23) found the labs need no
  second arm description (`ThreeLinkManipulator` and the UR5 cover them). Done when a second arm runs on the shared
  stack with the catalog both-backends test.
- **The former wave G rows** (moved from v0.3 by R48; none is needed by GMC714):
  - [ ] **C1 Pyro parity** open rows — [pyro-port-remaining.md](pyro-port-remaining.md); then
    the pyro → minilink migration guide in README from the name map there. **[ask]** for the
    README. Re-audit the table against the code first (scan: docs-gov#11).
  - [ ] **C2 Modelling ladder**: manipulators + the four-rung vehicle ladder as a
    `02_dynamics` tutorial lesson (not a GMC714 lab); robotic PID wrappers (`JointPD`, `EndEffectorPD` twins).
    Also: one constructor contract for the robotic laws (scan: control#6).
  - [ ] **C3 Blocks**: `Ramp` / `Chirp` / `Delay` / `Switch`; `Integrator` and `ZOHHold` take
    a `dim` like every static block (scan: graphics#11).
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
- [ ] **S73 Traceable linear analysis** **[idea to review — ask — core]**, proposed by the
  maintainer 2026-10-18, not scheduled: `poles`, `frequency_response` and `margins` in
  `analysis/linear.py` through the array module, so the sensitivity of closed-loop poles,
  damping ratios, Bode magnitude and phase, and gain and phase margins to any plant or
  controller param is `jax.grad` of the existing verbs on `jacobian("f", "params")`. The
  math lines stay the textbook ones (`C (jωI − A)⁻¹ B + D` as one batched solve instead of
  the Python loop; eigenvalues through `xp.linalg.eigvals`, whose derivative is the classical
  `vᴴ (∂A/∂p) u / (vᴴ u)` sensitivity). Brent's method on the crossovers becomes a fixed number
  of Newton steps from the grid bracket, so the crossover frequency carries the
  implicit-function derivative. Zeros stay NumPy (a generalized eigenproblem JAX lacks; the
  companion-matrix route for `D = 0` is a later step); root-locus branch matching stays NumPy
  (a plotting concern); the root locus generalizes to any parameter (`root_locus(wrt="m")`).
  Documented non-differentiable spots: repeated poles, the switch of the minimum margin
  between crossovers; JAX's nonsymmetric eigensolver is CPU-only. Recommendation from the
  2026-10-18 architectural review: alongside crossover `margins`, expose the maximum
  sensitivity peaks $M_s = \max_\omega |S(j\omega)|$ and $M_t = \max_\omega |T(j\omega)|$
  (and their smooth `logsumexp` surrogates over $\omega$) as traceable metrics on
  `sensitivity` / `complementary_sensitivity`, providing a smooth, crossover-free robustness
  bound for NLP gain tuning. Independent of V1 (needs
  only the jacobian), self-contained for a cloud agent. Done when `jax.grad` of the phase
  margin and of each closed-loop pole of `PID() @ Pendulum()` with respect to a gain and to
  the mass matches a central difference, and the NumPy callers see no change. Design:
  [differentiable-problem.md](differentiable-problem.md) §4.8.

---

## 6. v0.9 freeze candidate and v1.0

ROADMAP §5.5 and §5.6: v1.0 means a colleague builds a course on minilink and trusts it for
three years. The foundation rows are design conversations before code.

**Order:** the foundations (S29, S27, S44, S30, V3; S31 and S75 moved to v0.3, S37 absorbed by S75) before the trust contract; V11 once,
in December 2026 after the v0.3 tag; the v1.0 rows after two cohorts.

### v0.9 — what could still break a name

- [ ] **K5 Where the course material and the projects live** **[ask — student-facing]**
  (ROADMAP §6): `examples/teaching/` and/or `examples/projects/` in their own repositories, or
  here. Decided between terms (after GMC714). Options, a recommendation and the asks:
  [repository-structure.md](repository-structure.md). Done when the ruling is recorded.
- [ ] **K6 The move**, if K5 says so **[ask — student-facing]**: the history kept
  (`git filter-repo` per folder), every citing document updated (the GRO860 notes and webpage
  links, GRO501's `\colab{}`), the new repositories' CI pinned on a minilink release, the
  notebook smoke check moved with them, this repo's README and `examples/README.md` pointing
  there. Done when every course link resolves and both CIs pass.
- [ ] **S29** `DiagramSystem.x0` / `n` / `state` as derived properties (mirror the live
  `params` view); `Simulator` drops its pre-read `refresh()`.
  Also (scan: core#11, dynamics#10, examples#14): `x0` (the simulation start) and
  `state.nominal_value` (the operating point x̄, ruled 2026-10-13) are two quantities with
  one owner each; a diagram derives both from its blocks; the racecar solver hint, `WhiteNoise` and the diagrams' bubbled
  solver hints stop depending on the pre-read `refresh()`.
- [ ] **S27** Diffrax as an optional JAX solver backend, on the integrators S75 moved to
  `simulation/solvers/`. S37 (evaluators keep pure maps and one scannable step; the integrators'
  move; the evaluator internals deduplicated: ZOH sugar, the JAX step rollout, the four
  Jacobian probes, the undeclared batch cache; scan: compile-sim#9) is absorbed by S75's SM-1
  and SM-6 (ruled 2026-10-22).
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
  is the recommended install and README leads with it; `pip install minilink` is the light
  alternative, one `full` extra beside it (one install story, ruled 2026-10-17,
  [record](../reviews/2026-10-17-one-install-story.md)). What remains: the Colab setup cell
  installs a pinned release (`%pip install minilink==X.Y`) instead of `git clone main`;
  `COLAB_CLONE` in `tests/unittest/test_repo_contract.py` follows. `showcase_jax.ipynb`
  imports `minilink.experimental.c_export`, which the wheel does not ship: settle it here.
  The "pin the course notebooks on `0.2.0` now" option of
  [repository-structure.md](repository-structure.md) §4 waits here (ruled 2026-10-22: course
  sync is secondary to the order of the waves; the course notebooks keep tracking `main` and
  migrating in the same commit until then).
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

- [ ] **Estimation follow-ups** after P4, each in the slot
  [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §6 names: the
  finite-horizon filter (`kalman_finite_horizon` → a time-varying observer, the dual of
  `lqr_finite_horizon`, the forward sweep on the dual pair with `riccati_step`); the
  linearized Kalman filter along a trajectory (`trajectory_kalman` → `TrajectoryObserver`,
  the dual of `trajectory_lqr`); the extended Kalman filter (`ExtendedKalmanFilter(model, Q_n,
  R_n, x0, P0)`, `P` a state, Jacobians taken on the model's `f`, `h` inside its own `f`);
  the sampled filters (unscented, discrete Kalman, particle) after AC-8a / S31; the
  `EstimationProblem` / `EstimationSolution` and the `*Estimator` tools (§5 of the plan);
  online parameter estimators (recursive least squares, gradient laws) under a role of their
  own. Each revisits the observer conventions of
  [automation-by-convention.md](automation-by-convention.md) §10.
- [ ] **S71 The classic first plants**: DC motor, tank / thermal first-order-plus-delay
  process, ball and beam, differential drive, 3-D quadrotor, each with `tf`, a skin and the
  catalog both-backends contract test; LQI beside the `N` matrix of P8. Done when each has a
  one-line demo in `examples/demos/dynamics/`.
- C4 (identification, trajectory generation) is a v0.4 row (§5), as V1's program export with
  a data cost; it moves here only if v0.4 does not land it.

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
  lanes, wave and step ids leave README and `examples/README.md`; `CONTRIBUTING.md`, issue
  templates, a code of conduct (the governance docs moved under `docs/dev/` on 2026-10-17).
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


- [ ] **S36** iLQR planner from parts (`jacfwd` of `f_trace`; idea, research lane).
- [ ] **Behavior-cloning pipeline** **[ask]**: a game mode on `RealtimeSimulator` that records
  human demonstrations (keyboard or joystick driving the plant live) as trajectory datasets; a
  planner that fits a policy to them (imitation beside the RL planner, `NeuralPolicyController`
  as its output); the cloned controller closed on the plant and compared against the
  demonstrator. The pyro game demos, if ported (ROADMAP §6), are its first scenes.

---

## 7. Later ideas

One line each; open a plan doc only when a design needs a writeup.

- **A plant that owns its noise ports, as a real example** (R57): a disturbance the loop cannot
  add (unmatched, `B_w ≠ B`: a push on the cart-pole's tip, a lateral wind on `Drone2D`) and noise
  on some sensors only, with a demo and its LQG twin test. When it lands, `NoisyPendulum` and
  `NoisyCartPole` are revisited: one becomes that example, the other folds into
  `closed_loop(disturbance=, noise=)` (the tutorial cells and GRO501 `cartpole_dynamic_controller`
  migrate). **[ask — student-facing]**
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
- `SimulationOptions`, one record for the `Simulator` solver presets (DESIGN §5), weighed
  against RULES 2.5 (no option bags) before it is built.
- A sample stride on the fixed-step solvers (`compute_trajectory(..., substeps=k)`: `k` RK4
  steps per stored sample, the scan keeping only the samples): a stiff model stepped at
  0.5 ms over 40 s stores 80k states today, which is what pushed jaxnet to keep its own
  decimated `lax.scan` rollout beside `Simulator`.
- Cross-fidelity `lift` / `project` maps on the car ladder (a July 2026 draft, in git history).
- `RobustPlanningProblem` (set-bounded uncertainty, minimax criterion) only when a minimax
  consumer exists; the path-integral planner's `"worst_case"` branch (CVaR over plant draws,
  [mppi.md](mppi.md) §4) would be that consumer; the two decisions go together.
- `RolloutEnvironment` may move from `reinforcement_learning/` to `planning/environment.py`
  once a second band reads it (open in ROADMAP §6) (the path-integral planner; the evaluator and the tabular learners
  already do): one import rewrite, no behaviour change.
- Diffusion policy (research lane, JAX-only, after behavior cloning): a sampled controller with
  memory and a key, trained by `fit`; see [future-policies.md](future-policies.md).
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
