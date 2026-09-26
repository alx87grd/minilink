# Minilink workboard

Every open step of the path to v1.0 in [ROADMAP.md §5](../../ROADMAP.md#5-the-path-to-v10),
by rung. Strategy, milestones and the TRL ledger stay in ROADMAP; rulings and
audits in [docs/reviews/](../reviews/); pyro parity rows in
[pyro-port-remaining.md](pyro-port-remaining.md). Landed rows leave this file:
the history is the git log and the dated reviews (the last workboard that
carried them is `docs/plans/TODO.md` at commit `1f0f7ce`).

Each step is sized for roughly one agent-hour to one agent-day and ends with a
"done when" gate. **[ask]** marks maintainer-owned territory (student-facing
material, core architecture, main-tool APIs, any feature or public-name
removal): propose, get a yes, then land. Unmarked steps are agent-managed:
land, then report. Conventions for every step: `ruff check . && ruff format
--check .`, the relevant `pytest tests/unittest/test_<domain>.py`, and
DESIGN / README updates only where a public contract changes. Behaviour-
preserving steps follow the AGENTS refactor recipe (seeded baseline, byte-
identical after).

`scan: band#n` cites an entry of the 2026-09-22 improvement scan: the evidence is in
[docs/reviews/2026-09-22-improvement-scan-ledger.md](../reviews/2026-09-22-improvement-scan-ledger.md),
the triage in [2026-09-22-improvement-suggestions.md](../reviews/2026-09-22-improvement-suggestions.md).

| Section | Rung |
| --- | --- |
| [§1](#1-v01-close-out) | v0.1 close-out |
| [§2](#2-v02-wave-a--the-textbook-objects) | v0.2 wave A — the textbook objects |
| [§3](#3-v02-wave-b--gro501) | v0.2 wave B — GRO501 |
| [§4](#4-v02-wave-c--catalog-parity-new-bands) | v0.2 wave C — catalog, parity, new bands |
| [§5](#5-v02-wave-d--textbook-code-minimal-demos-consolidation) | v0.2 wave D — textbook code, minimal demos, consolidation |
| [§6](#6-v10-foundations) | v1.0 foundations |
| [§7](#7-later-ideas) | Later ideas |

---

## 1. v0.1 close-out

- [ ] **R1 Publish `0.1.1` from a tag** **[ask]**. `minilink 0.1.0` is on PyPI (2026-09-16)
  with no git tag on its commit; tag that commit `0.1.0` for the record if it is known (the
  tag push starts `publish.yml`, whose upload PyPI refuses for an existing version).
  Confirm the GitHub Environment `pypi` and the PyPI trusted publisher (`alx87grd` /
  `minilink` / `publish.yml` / env `pypi`), then `git tag 0.1.1 && git push origin 0.1.1`.
  Done when `pip install minilink==0.1.1` installs the teaching surface from PyPI.
  Before `0.1.1` (scan: tests-ci#12, tests-ci#13, examples#1, examples#4): `publish.yml`
  runs ruff and `pytest` before uploading, and every long CI job gets `timeout-minutes`;
  `showcase_jax.ipynb` stops importing `experimental.c_export`, which the wheel does not
  ship; `plot_diagram()` without the `graphviz` wrapper (ROADMAP §6); one canonical Colab
  setup cell per install tier, pinned by a test (38 notebooks carry 11 variants today).
- [ ] **R3 Ruff runs itself**: `ruff` and `ruff-format` hooks in `.pre-commit-config.yaml`,
  pinned to the dev extra's ruff (scan: tests-ci#6). Done when `pre-commit run --all-files`
  matches the CI lint steps.
- [ ] **S59 Docs drift** (scan: docs-gov#3, docs-gov#4, docs-gov#5, docs-gov#7, docs-gov#8,
  docs-gov#12, docs-gov#13, tests-ci#4, control#12, analysis#12): the CI commands written
  once (the tests/README agent table), AGENTS naming the jobs and the regression flags given
  one owner; DESIGN's inline TODOs moved here and its retired pointers fixed; RULES 3.3's
  teaching surface stated as ROADMAP §2 states it; plan-doc step ids that cannot collide
  with the workboard's; retired phase numbers scrubbed from the plans; AGENTS naming who
  edits ROADMAP §5 and §6; the control band's docs housekeeping; the analysis API page. Done
  when `test_repo_contract.py` passes and no doc names a retired section.
- [ ] **S60 The teaching surface, checked and documented** (scan: graphics#2, graphics#3,
  graphics#10, tests-ci#8, docs-gov#0, dynamics#12): the registry test walks every band
  facade's `__all__` as it walks the root prelude, then `NotchFilter`, `Washout` and `MLP`
  join the blocks row; `docs/api` generated from the registry (or a test that every prelude
  name's module has a page) and Sphinx built with `-W`; the dynamics page covers the catalog;
  `minilink.graphical.catalog` and `minilink.interfaces` as lazy facades. Done when a prelude
  name without a page fails a test.
- [ ] **S49 Retire the Stable-Baselines3 teaching notebooks** **[ask]**. Delete
  `teaching/courses/udes_gro860/drone_ppo_sb3.ipynb` and
  `pendulum_value_iteration_vs_lqr_vs_ppo_sb3.ipynb`, their notebook overrides and allowlist
  entries, and the README rows marked Stable-Baselines3, once the course notes point only at
  the native twins.
- [ ] **S54 `plot_cost2go` colour scale** clipped at `out_of_bound_cost` by default (the
  showcase passes `jmax` by hand); review the DP `out_of_bound_cost` default. It can be a
  callable `price(x, t)` (the problem's `infeasible_cost`), which has no single level to clip
  at, so the default needs a rule for that case too.

---

## 2. v0.2 wave A — the textbook objects

Design: [fields.md](fields.md), [cost-params.md](cost-params.md),
[geometry-module.md](geometry-module.md), [naming.md](naming.md). Baselines first, as each plan
states; every step is name-preserving for the GRO860 notebooks.

- [ ] **A1 `core/fields.py`** **[ask — core]**. Steps 4.1–4.8 of [fields.md](fields.md):
  promote `StateField` → `Field` on `(x, u, t)` with `as_constraint` / `as_input_constraint` /
  `as_cost` and `gradient`; `QuadraticField` (Lyapunov `V`, LQR value, `S(t)` schedule) with the
  ellipsoid as its sublevel set; `GridField` (DP and tabular tables, per-sweep time axis);
  `CallableField` (RL critics); `LinearApproximator` as a field; `LyapunovCertificate.V` a
  field; CBF plan amended; DESIGN §4 fields bullet. Done when the RoA and DP baselines `cmp`
  and `test_analysis_lyapunov.py`, `test_planning_solution.py`, `test_rl_tabular.py`,
  `test_rl_planner.py` pass.
  Also: `PolicyEvaluator` takes the evaluators' verb and returns a field-shaped result
  (scan: planning#4).
- [ ] **A2 Cost parameters** **[ask — core]**. [cost-params.md](cost-params.md): `params` on
  every library cost (`QuadraticCost`, `TimeCost`, `FieldCost`, `SumCost`, `ScaledCost`),
  composite costs nested like diagram params with `cost.id`, attributes as views so the
  notebooks' in-place writes keep working; then the trajopt, DP, evaluator and RL paths pass
  `problem.params.cost` through. Done when every planner baseline `cmp`s and `jax.grad` of a
  trajopt objective with respect to `Q` runs.
  Also (scan: core#3, core#4): `discount_rate` a constructor field (or a `params` entry) that
  `ScaledCost` forwards and `SumCost` reconciles (the composites keep it since 2026-09-23);
  `validate_diagram_params` checks each per-block dict against the block's keys, so a partial
  dict fails at the setter instead of inside `f`. The Monte Carlo evaluator and the gym reward
  never apply `problem.params.cost` while the DP table does, so the two disagree on a
  parametric cost (found 2026-09-23).
- [ ] **A3 Workspace geometry (S57)** **[ask — core]**. [geometry-module.md](geometry-module.md)
  implementation steps 1–6: `core/geometry/` package (shapes, paths, track, scene, probes,
  `bind`, spatial fields, grid, catalog `oval_circuit` / `racecar_circuit` /
  `holonomic_forest`); shaping next to `Field.as_cost`; plots and overlays under
  `graphical/` with lazy methods on `Scene` / `Track`; `planning.spatial` re-exports then
  deletes in the same change as the call sites; `PurePursuit` takes a Path / Track. Done when
  the UdeS racecar trio and the RRT forest demo use the catalog, `control` imports geometry
  with no planning import, and the suite is green. Glyph rename stays S30.
  Also: `Shape` unions flatten like `IntersectionSet` and `SumCost` (scan: core#6);
  `PurePursuit` sizes its measurement from the vehicle, not `state_dim=9` (scan: control#9).
- [ ] **A4 Naming quick wins** 1–4 and 6 of [naming.md](naming.md), each a few lines plus a
  test: class name as the default `name`; one closed-loop name on both `@` paths; informative
  default names where a shortcut says `Diagram`; the sampled loop's plant wrapper honours
  `plant.id`; `id` documented on `System`. Done when composed diagrams' keys, params
  dictionaries and trajectories are byte-identical before and after.
  Also: one name, `plot_cost_to_go`, and one colour keyword, `jmax`, across DP, tabular RL,
  `PolicyEvaluator` and `Comparison`, with `plot_cost2go` an alias for one release and S54's
  clipping (scan: planning#3, examples#5); the operators' named forms exported together
  (scan: core#9).
- [ ] **A5 Later nouns, each with its first consumer.** `Gaussian(mean, std=None, cov=None)` +
  `log_prob` with the disturbance convention (ROADMAP §6; before P4); parameter-dictionary
  support on sets and distributions (one flatten rule, with identification or the robust
  problem); `NoiseSource(distribution, sample_period)` replacing the hand-rolled `WhiteNoise`
  draw; `UnionSet` via `|` when reachability or multi-goal work needs it; vector bounds on
  `Saturation`; library sets and fields reading `params` (the "one signature" gap, same rule
  as A2, after it); `PlanningProblem.hamiltonian()` only if the course teaches Pontryagin; the
  RL critic as a `Field` once training is over.
  Also: `Distribution.sample(key=None)` like the sets (scan: core#5); a guard test that
  editing `WhiteNoise` params changes the next simulation without `refresh()`, so S29 cannot
  land before `NoiseSource` (scan: examples#14); `Trajectory.from_rollout(x0, xs, us, dt)` for
  scanned rollouts, V1's first consumer (scan: examples#9); the double-integrator homework's
  two verbs, entry time into a set and the Bellman residual (scan: examples#10).

---

## 3. v0.2 wave B — GRO501

Plan: [gro501-classical-control.md](gro501-classical-control.md) (P1, F1, F2, F4 landed
2026-09-07). Release contract: [ROADMAP §4.2](../../ROADMAP.md#42-v02--gro501-end-to-end).

- [ ] **P2 `minreal`** on the matrices tier + `sys.minreal()` returning an `LTISystem`; a
  `tol` keyword on `transfer_function` / `pzmap` / `root_locus` (default: no cancellation);
  order reduction by neglecting fast modes as a separate verb. Done when the pre-P1 pure-P
  loop reduces to `200 / (s² + 0.5 s + 4.905)` and the guide's §9.13 realization survives
  unchanged.
- [ ] **P3 `place()` / `place_gain()` / `place_at_operating_point()`** mirroring `lqr`;
  `scipy.signal.place_poles` plus an honest error when the pole set is not reachable; a demo
  in `examples/demos/control/`. Done when `eigvals(A − BK)` on the guide's parking model
  match `{−1 ± 0.5i, −1}` to 1e-9 and the block closes the loop with `@`.
- [ ] **P5 Named sensitivity functions** `sensitivity` (`S = e/r`), `complementary_sensitivity`
  (`T = y/r`), `PS`, `CS` on a closed-loop diagram, each an `LTISystem`; `disturbance=` /
  `noise=` injection points on `feedback()`. Done when `S + T = 1` holds to 1e-12 on a SISO
  loop and each Table 2 spec is one call plus a comparison.
- [ ] **S61 Analysis verbs on any `System`** (scan: analysis#2, analysis#4, analysis#5,
  analysis#6, analysis#8): `controllability` / `observability` linearize like every sibling
  and gain facades; `step_info` takes a `System`; `find_equilibrium` defaults its guess to
  `sys.x0`; the operating point's size is checked; margin crossings refined by a root solve so
  `plot_bode` and `margins()` report one number. Done when P7's signature test covers them.
- [ ] **S62 The LQR family on the control band** **[ask — public names]** (scan: control#0,
  control#1): rename the `control/lqr.py` module so `lqr`, `lqr_at_operating_point`,
  `lqr_finite_horizon`, `trajectory_lqr` and `lqr_gain_schedule` join the band facade and the
  registry; `P` joins the siso family. After the GRO860 term (gate 7), before P11 writes
  notebooks on the band layer.
- [ ] **P7 Generate the analysis facades**: one helper builds a delegating method from the
  target function (signature and docstring copied), explicit form kept only where the facade
  differs; a test asserts every generated `__signature__` matches its target. Done when the
  13 hand-copied methods are generated and `help(sys.bode)` is unchanged.
  Also: a signature test pinning the band's calling pattern (scan: analysis#7); `linearize` and
  `discretize` freed from the band-facade name collision (scan: analysis#11).
- [ ] **P8 Small teaching helpers**: ζ and ω_n from a complex pole pair (fields on the `pzmap`
  result or a `damping(sys)` verb); the `N` reference-scaling matrix giving `y = r` at steady
  state; the `settling_horizon` docstring (five vs eight). Done when §9.5 and §9.11 of the
  guide are each a short notebook cell.
  Also: a `poles(sys)` verb with a `sys.poles()` facade, and `__str__` on `LTISystem`,
  `TransferFunction` and `StructuralResult`, so tutorial 01's `print(tf)` shows the transfer
  function and fourteen `np.linalg.eigvals(lin.A())` sites go (scan: analysis#3, examples#2).
- [ ] **P4 `estimation/`** **[held 2026-09-07 — ask to lift]**. `LuenbergerObserver(A, B, C,
  L)` as a `DynamicSystem` with ports `u`, `y` → `x_hat`; `luenberger(A, B, C, poles)` on the
  dual pair (needs P3); `kalman(A, B, C, Q, R)` from the filter Riccati equation; the
  observer + state-feedback composition ruled first (ROADMAP §6). Done when LQR + Kalman
  stabilizes the guide's cart-pendulum from a disturbed start with noise on `u` and `y`, and
  `plot_diagram` shows the Figure 12 topology.
  Also: named input ports on the state-space base so observers and sensitivity blocks share
  one `f` (scan: analysis#13); the plan names the estimation API once (scan: docs-gov#9).
- [ ] **P9 `TransferFunction` builds its ports once** (no clear-and-re-add); the four
  `ports` values stay. Done when no `self.inputs = {}` remains in the constructor and the
  `TransferFunction` / `Lead` / `Lag` tests pass untouched.
- [ ] **P10 Document the three `@` dispatch paths** in DESIGN (operand shape → what `@`
  builds); pin the `e`-input autowire heuristic with a test; collapse the `PROFILE_PORTS`
  rows that differ only in `plot_space` if that reads better.
  Also: one keyword vocabulary for the four feedback wires across `closed_loop`,
  `hybrid_closed_loop` and `StandardFeedbackWiring` (scan: core#12).
- [ ] **P6 Discrete (z) tier** **[held 2026-09-07]**. Default: teach with `discretize` +
  simulation. Reopen only if the sommatif examines z-plane analysis.
  When it reopens: an exact zero-order-hold option on `discretize`, shared with
  `step_response` (scan: analysis#1). The `discretize` bugs (dt in params, dropped `x0`) were
  fixed 2026-09-23: the sample time is `disc.dt`, `params` reach the source untouched, and
  the wrapper keeps the source's `x0` and signal metadata. The sample time has one owner:
  a `"dt"` key in the params that reach `f` is refused at construction and on
  `disc.params = ...`, so the fallback to the source's own `params["dt"]` is gone;
  `params={"dt": 0.05}` alone is refused for a source with other params (pass `dt=`), and
  `dt=` with a different `params["dt"]` is refused. Two follow-ups remain.
  **[ask]** keep the remaining `params["dt"]` fallback (`dt=` omitted, read from the
  `params=` dict given to `discretize`) or make `dt` required.
  A source whose `y` feeds through from a port other than `u` does not discretize
  (`PendulumWithNoisePort`: unknown input dependency `v`), because the wrapper stacks every
  port into one `u`; map `y`'s dependencies to `("u",)` or mirror the source's ports.
- [ ] **P11 Two GRO501 notebooks** **[maintainer — student-facing]**:
  `teaching/topics/classical_control/dc_motor_propulsion.ipynb` (APP2) and
  `bicycle_autopilot.ipynb` (APP4), Basic tier, Colab-first; drafted for review. Done when
  both run top to bottom in Colab and the conda env, import only through the teaching
  surface, and agree with the guide's worked exercises.
  First, **[ask — core]**: `Controller(feedback="state" | "output", ...)` declares its ports
  from `feedback_profile`, so the student writes gains and `ctl` only (scan: examples#0).

---

## 4. v0.2 wave C — catalog, parity, new bands

- [ ] **C1 Pyro parity** open rows — [pyro-port-remaining.md](pyro-port-remaining.md); then
  the pyro → minilink migration guide in README from the name map there. **[ask]** for the
  README.
  Re-audit the table against the code first; the three landed rows the scan found were
  corrected 2026-09-23 (scan: docs-gov#11).
- [ ] **C2 GMC714 modelling ladder**: manipulators + the four-rung vehicle ladder as a
  `02_dynamics` lesson; robotic PID wrappers (`JointPD`, `EndEffectorPD` twins).
  Also: one constructor contract for the robotic laws (scan: control#6).
- [ ] **C3 Blocks**: `Sine` / `Ramp` / `Chirp` / `Delay` / `Switch`; `Integrator` and `ZOHHold`
  take a `dim` like every static block (scan: graphics#11).
- [ ] **C4 Identification and generation**: `identification/fitting.py` on `rollout_batch`
  (physical params and NN weights are one verb); `trajectory_generation/` port (polynomial,
  min-snap); SMC trajectory-following demo; trajectory post-filter.
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
  - [ ] `Sys2Gym` takes a `Distribution` for the start state (scan: graphics#1).
- [ ] **S63 DP reads the horizon** **[ask — planner API]** (scan: planning#1): a finite
  `problem.tf` runs `round(tf / dt)` backward sweeps as `solve_steps` does, the way
  `LQRPlanner` picks the Riccati ODE, or warns once. Done when LQR and DP agree on the
  finite-horizon double integrator.
- [ ] **S64 Catalog hygiene** **[ask — catalog]** (scan: dynamics#0, dynamics#1, dynamics#2,
  dynamics#3, dynamics#4, dynamics#6, dynamics#8): the mechanical bases stop inventing
  ±2π / ±5 bounds and each plant states its own (ROADMAP §6); output ports that are the state
  or a slice of it read labels and units from the state (the 56 copy lines go); one port
  layout across the bicycle rungs; one owner for the wheelbase; the hidden `0.01` damping in
  `Drone2D.d` and `Rocket.d` named; `VanderPol` with a real input or none; constructor hygiene
  in the pendulum and mass-spring-damper families.
- [ ] **Estimation follow-ups** after P4: EKF, time-varying and discrete Kalman as
  `estimation/` rows.

---

## 5. v0.2 wave D — textbook code, minimal demos, consolidation

**T — the textbook pass.** Behaviour-preserving, one module per step, the AGENTS recipe
(seeded baseline in the scratchpad, byte-identical after; ruff and the module's tests). The
findings, file by file and line by line, are in
[docs/reviews/2026-09-22-consolidation-review.md](../reviews/2026-09-22-consolidation-review.md);
`planning/policy_synthesis/dp.py` is the reference. T0 landed 2026-09-22 (see that review).

- [ ] **T1 core** (what T0 left): `system.py` default stubs on `xp` (a returned-type decision
  under JAX) and section comments; `feedback.py`'s `ErrorDriven` shadow state and order
  **[ask — core]**; `wiring.py`'s hasattr-and-create, attribute-assigning helpers and the
  `_composition_*` writes; `signals.py` helpers below the class; `hybrid_diagram.py` /
  `hybrid_composition.py` names and sections. Bug found by the audit, its own test first:
  `StepDiagramSystem.step` assigns in place on a JAX array.
- [ ] **T2 blocks and control** (what T0 left): `control/mpc/utilities.py` order and names
  (`_shift_plan_trajectory` is imported by `test_mpc.py`), the MPC package docstring;
  `blocks/transfer_function.py`'s `tf` on `np` without `params` (P9 rebuilds its ports);
  `WhiteNoise.h` / `TrajectorySource.h` cannot trace (`interp1d`); `action_port_of` above
  the class; the joint-impedance law still in a helper.
  Also (scan: control#2, control#5, control#7): one time-interpolation for trajectories and
  gain schedules; one warm-start helper on the plan `Trajectory`; the impedance and robotic
  law bodies consolidated.
- [ ] **T3 dynamics catalog** (what T0 left): `manipulators/arms.py` (helpers below,
  `_set_planar_reach_camera` and `from_manipulator` setting attributes from outside
  `__init__`, `link_lengths` probing with `hasattr`, FK / J reading `params` — a fix with a
  test), `pendulum/cartpole.py` (`_configure_cartpole` inlined into `CartPole.__init__`,
  `RotatingCartPole.tf` on `xp`), `mass_spring_damper/linear.py` (`A` / `B` / `C` / `D` on
  `xp` — a JAX-safety fix; extend the both-backends test), `vehicles/dynamic_bicycle.py`
  (`_u_in` / `_contact_fields` renames ripple to `racecar.py`, two project files and
  `graphical/catalog/skins.py`; dead `_wheel_rectangle_pts`), `manipulators/ur5.py`
  (helpers and the two algorithms below the public API), `aerial/plane.py` (`tf` on `xp`
  reading `params`; the 55-line pygame `__main__`), `abstraction/state_space.py` (helpers
  below), the 37 inline `array_module(q).array(...)` calls.
  Also: `Manipulator` kinematics defaults raise instead of returning zeros, with
  `link_lengths` the base contract (scan: dynamics#7).
- [ ] **T4 analysis and simulation** (what T0 left): `analysis/lyapunov.py` (primary
  function first, ceremony in a helper, `verify` / rollout unpacked, `EVEN` / `ODD` beside
  `METHODS`, `require_jax`, `sample_in_ellipsoid` named), `analysis/linearize.py` (the `D`
  comprehension unrolled, the early return), `analysis/linear.py` / `frequency.py` /
  `time_response.py` / `structural.py` remaining return-line expressions and the
  five-vs-eight settling docstring, `analysis/modal.py` (naming `eig`'s result is a
  returned-type decision), `simulation/simulator.py` (solver table and helper below the
  class, `solve` before its helpers, the detached comments, a constructor of named
  steps), `simulation/computer.py` (`as_computer` below the classes, its coercion in a
  helper), the solver backends' comments and `_finalize_solution`, the duplicated
  `DISCONTINUOUS_AUTO_DT_SCALE`.
  Also: one uniform-grid check shared by the fixed-step backends and the simulator (scan:
  compile-sim#14).
- [ ] **T5 planning** (what T0 left): `trajectory_optimization/planner.py` (primary class
  first, eleven `_` methods — `_make_callback` in three test lines —, the `getattr` probes
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
  `collision.py`, `plotting.py`. `ReinforcementLearningPlanner(verbose=True)` prints by
  default (RULES 4.6): ruled to stay 2026-09-15; recorded.
  Also: the parametric (MPC) optimizer goes through `Optimizer`'s backend table (scan:
  planning#9).
- [ ] **T6 the two big ones** **[ask first]**: `core/composition.py` (24-line section-map
  docstring, 45 `_` helpers, public API buried under `# Internal machinery`, eleven external
  writes to `_composition_*`; and the three `@` semantics of P10) and
  `control/mpc/controller.py` (seventeen `_` methods, hasattr-and-set from
  `export_mpc_dual_rate_computer`, the factory at line 340). Each is a conversation, then a
  step ladder of its own.
  Also: one record per MPC tick, `Command` and `MPCTickSolve` folded into the
  `PlanningSolution` (scan: control#4); `mpc @ inner_loop` closing on a multi-block plant
  (scan: examples#12).
- [ ] **T7 graphical** (scan: graphics#13): the graphical band joins the textbook pass
  (renderers, catalog skins, signal plots), same recipe.
- [ ] **Rename pass, the rest**: the remaining `_method` names on `System` subclasses not
  covered by T1–T6 → plain names (maintainer style rule). **[ask per module]**

**D — demos to the rule** (RULES 6.1 / 6.10 / 6.11; the census is comment C14 of
docs/reviews/2026-09-15-foundations-review.md).

- [ ] **D1.1 Native reporting the demos hand-roll** (agent lane, plotting): an `Optimizer`
  convergence plot (`optim_plot.py`); two planners' trees or solutions side by side
  (`rrt_car_parking.py`, `rrt_holonomic_obstacles.py`); learning curves of several learners on
  one axis (`double_integrator_sarsa_vs_q_learning_rl.py`); a `rollout_batch` family plot
  (`rollout_param_family.py`); a 3-D phase plot (`lorenz_attractor.py`). Each one `plot_*` or
  `__str__` with a test; DESIGN §7 lists them.
  Also: `print` on `StepRollout` and `HybridSimResult`, one validator shared with `Trajectory`
  (scan: core#7); `print(controller)` shows the gains (scan: control#10); several
  trajectories on one phase plane and two fields on one grid surface (scan: examples#6).
- [ ] **D1.2 The flatness ratchet** in `test_teaching_imports.py`: no top-level `def` or
  utility class in `examples/demos/`, no notebook cell mixing an API call with matplotlib
  code; today's offenders allowlisted, the list shrinking with D1.3.
  Also: teaching code stops rebinding `sys` (scan: examples#7); one owner for the
  teaching-facade list the two teaching tests share (scan: tests-ci#7).
- [ ] **D1.3 The sweep**, one file per step, maintainer reviews each diff **[ask per file]**:
  the tutorial chapters (`11_reinforcement_learning.ipynb` and `showcase_jax.ipynb` first;
  `showcase_jax` rewritten 2026-09-18, awaiting review), then the non-clean demos, then the
  teaching notebooks; the three `compile/` demos wait for V1 (their helpers *are* that
  feature); `manipulator_eom.ipynb` reviewed for which of its 26 helpers the text teaches.
  Done when the D1.2 allowlist is empty and the notebook checks pass.
  Also: the placeholder sections of tutorials 01, 02, 03, 05, 08 and 09 filled (scan:
  examples#3); the facade gaps the import allowlist records closed (scan: examples#13).
- [ ] **D1.4 The planning demos and notebooks on the solution verbs** **[ask]**: the
  `planner.solve().trajectory` one-liners, the hand-rolled comparisons and the per-file
  `@dataclass` rows become `solution.plot_*` and `compare(...)` (the runnable draft of
  `vi_pendulum_lqr.py` was shown 2026-09-17).
  Also: `Planner.plot_solution` is the solution's own `plot_trajectory` (scan: planning#7).

**D2 — consolidation picks** still open from
docs/reviews/2026-09-05-consolidation-inventory.md (nothing removed without the maintainer's
pick):

- [ ] `Source.show_signal` (80 lines of bespoke matplotlib; the same picture is
  `source.plot_trajectory(tf=…)`), with the sources demo and `__main__` that exist only for it
  (scan: graphics#14) **[ask — user-callable]**.
- [ ] Dead modules `planning/spatial/overlays.py` (retires with A3) and
  `experimental/symbolic/mechanics/utils.py` **[ask — importable names]**.
- [ ] Deprecated benchmark shims (`run_pendulum_f_speed.py`, `run_diagram_f_speed.py`; check
  `run_study.py` presets cover the `run_step_*` / `run_simulator_*` scripts).
- [ ] MPC debug figure → `control/mpc/viz.py`.
- [ ] `HybridDiagram` hand-copied facades (with S31).
- [ ] Plotting in eight homes: write the placement rule in DESIGN first, then move
  `graphical/port_map.py` next to `control/` if agreed.
- [ ] DESIGN §8's package-roles table (a stale copy of §3) and the DESIGN research-lane trim
  (scan: docs-gov#6, docs-gov#10).
- [ ] The three catalog rows the dynamics scan names (scan: dynamics#13).
- [ ] `JaxMechanicalSystem` retired, the `xp` base already traces (scan: dynamics#14;
  ROADMAP §6) **[ask — importable name]**.
- [ ] `NeuralNetwork` folded into `MLP` (scan: graphics#12) **[ask — importable name]**.
- [ ] One `ctl` for the model-based laws; `LookupTableController` beside the other laws (scan:
  control#8, control#13).

**D3 — hardening rows** (research lane, small):

- [ ] RRT `KinodynamicExtender` ignores `problem.params.system`; `extenders.py` probes
  `getattr(U, "box", None)` instead of `U.bounding_box()`.
- [ ] MPC port computes drop `params`; dual online-params façades.
- [ ] `ShootingTranscription` orphaned from the string presets.
- [ ] `ParametricMathematicalProgram` / `JaxParametricProgramEvaluator` placement (54 % duplicate
  of `optimization/evaluators/jax_evaluator.py`; [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md)).
- [ ] `HybridSimulator` conventions drift (shared input coercion, positional `input_port_id`,
  the framed verbose panel; the plant sub-step from the plant time constant instead of one RK4
  step per tick; scan: compile-sim#4, compile-sim#5); realtime `TODO: User Architectural
  Review`.
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
  details in the review records of docs/reviews/2026-09-22-improvement-suggestions.md §2.4):
  `step_diagram % dt` adds boundary ports to a user `StepDiagramSystem` (DESIGN §4 names the
  gap); the sampled `@` copy of a one-block plant diagram drops a diagram-level function
  output port; a `%` override that ends in `as_computer(self, schedule)` would recurse, and
  `System.__mod__` now calls a private helper; `RealtimeSimulator`'s offline automatic `dt`
  reads the hint without `refresh()`; an explicit `backend='jax'` on a NumPy-only set or cost
  raises a raw trace error; `discretize(diagram, params={'dt': v})` is refused although a
  diagram would keep its blocks' params, and a raising params setter can leave a diagram
  half-updated; `JaxMechanicalSystem` keeps the integer-state dtype defect (retires with the
  ROADMAP §6 decision); assigning into `q.labels[i]` in place no longer sticks; the gravity
  hook's signature is read on every call; the collocation demo's SLSQP fallback is not run in
  CI (its manifest entry still requires Ipopt).
- [ ] Bare `import jax` in `analysis/lyapunov.py` and
  `trajectory_optimization/parametric_evaluator.py` → `require_jax()` (RULES 5.12).

**S65–S68 — one owner per rule, contracts as tests** (from the 2026-09-22 scan):

- [ ] **S65 One owner per rule** (scan: compile-sim#0, compile-sim#1, compile-sim#2,
  compile-sim#3, compile-sim#6, compile-sim#7, compile-sim#8, compile-sim#10, compile-sim#13,
  planning#0, planning#5, planning#8, planning#11, core#8, core#10). One item per step: the
  automatic `dt` as `auto_dt(sys)` in `time_grid.py`, with the discontinuous scale made
  finer or DESIGN's promise dropped (the duplicated constant goes with it); the
  try-JAX-then-NumPy policy in `compile_auto` with a typed `NotTraceableError`; the
  forced-input hold as one `input_interp` option every solver obeys; what `n_steps` counts on
  each verb; the one-tick lag inside a `Computer` tested and stated, or removed;
  `has_trace_tier` a boolean; `simulation` stops importing `optimization` for display
  constants; `scipy_stiff` with explicit tolerances and a Jacobian; `compile(verbose=True)`
  in order; one set of Monte Carlo defaults for `MonteCarloEvaluator` and `Planner.evaluate`;
  flat-kwarg keys derived from the option dataclasses; one owner of the sets between
  `StateSpaceGrid` and its planner; one print-text rule for the textbook objects; one verbose
  and empty-cache rule across the `compute_*` facades.
- [ ] **S66 Wiring mistakes fail at wiring time** **[ask — core]** (RULES 4.10; scan: core#2,
  core#13): `connect` refuses to rewire a connected subsystem input; the compile probe
  detects an output that moves with an input it does not declare (raise or warn: ROADMAP
  §6).
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
  shrinking allowlist; ROADMAP §5 step ids and TODO rows agree; every teaching notebook
  executed nightly; the subprocess demo-check tests behind a marker; wall-clock claims out of
  unit tests; unit tests stop importing `examples/projects` and `benchmarks/systems`;
  `Animator.resolve_frame` public for the graphics harness; course pins byte-identical to
  their topic twins; one both-backends test over every control law, the catalog with
  perturbed `params`, and the `params.sets` scoring parity.

---

## 6. v1.0 foundations

After two cohorts; each is a design conversation before code. **[ask — core]** throughout.

- [ ] **S31** `HybridDiagram` → a `System` (state `[plant; computer]`, periodic discrete
  update) or an honest `HybridLoop` rename with `%` → `on_schedule()`; the hand-copied
  facades go with it.
  Also: the sampled seam's time argument, ticks or seconds (ROADMAP §6; scan: control#3); the
  `core` → `simulation` import (scan: core#14).
- [ ] **S29** `DiagramSystem.x0` / `n` / `state` as derived properties (mirror the live
  `params` view); `Simulator` drops its pre-read `refresh()`.
  Also (scan: core#11, dynamics#10, examples#14): one owner for the initial state (`x0` or
  `state.nominal_value`); the racecar solver hint, `WhiteNoise` and the diagrams' bubbled
  solver hints (re-bubbled by `refresh()` since 2026-09-23) stop depending on the pre-read
  `refresh()`.
- [ ] **S37** Evaluator / solver re-layering: evaluators keep pure maps and one scannable
  step; integrators move to `simulation/solvers/`. Then **S27** Diffrax as an optional JAX
  solver backend.
  First, the evaluator internals deduplicated (ZOH sugar, the JAX step rollout, the four
  Jacobian probes, the undeclared batch cache; scan: compile-sim#9).
- [ ] **S44** A single posed-geometry hook so "two functions" (`f` + drawing) is literal;
  today `tf` + skin.
- [ ] **S30** Rename the graphical `Sphere` / `Box` glyphs so no two importable public types
  share a name with the geometry solids. **[ask — public names]**
- [ ] **S32** Unify `MechanicalSystem` / `GeneralizedMechanicalSystem` (`N = I` special
  case); `Boat2D` / `Plane3D` gain `q` / `dq` ports; `DynamicBicycle` on
  `GeneralizedMechanicalSystem` (scan: dynamics#5).
- [ ] **V1 The differentiable closed-loop cost**: `J = F(problem params)` as one traced
  scalar — simulate the closed loop and integrate the plant's cost with every parameter an
  input (`system` nested by subsystem id, `cost` (A2), `sets`, `x0`), so `jax.grad` and `vmap`
  reach plant, controller, cost, sets and `x0` alike; today tutorial 11 and
  `pid_autotuning_jax` write that scan by hand.
- [ ] **S36** iLQR planner from parts (`jacfwd` of `f_trace`; idea, research lane).
- [ ] **V2** Zenodo archive and a citable DOI (`CITATION.cff`) after PyPI; the JOSS entry
  and the RULES 6.9 carve-out for its state-of-the-field section, decided then.

---

## 7. Later ideas

One line each; open a plan doc only when a design needs a writeup.

- Vehicle view ports (`pose` / `bodyvel` on `DynamicBicycle` for impedance / PID).
- Scene params / `J(z, p)` bind (DESIGN §4 planning-params pipeline B; moving obstacles
  online without rebuilding the NLP).
- `SolverFactory` — [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md).
- Cross-fidelity `lift` / `project` maps on the car ladder —
  [fidelity-maps.md](fidelity-maps.md).
- Articulated mechanism layer (one mechanism description feeding RNEA/ABA and the symbolic
  path) — [articulated-mechanism.md](articulated-mechanism.md).
- `RobustPlanningProblem` (set-bounded uncertainty, minimax criterion) only when a minimax
  consumer exists; the deterministic / stochastic pair is the taxonomy that shipped.
- `MjxPlant` (`interfaces/mjx.py`); Pacejka tire; stochastic forcing; ROS2 / FMI; sparse
  long-horizon trajopt; RRT-Connect; shared RNEA serial-chain stack; ABA on other RNEA arms.
- Ipopt given the Lagrangian Hessian, not the objective's alone (scan: planning#13);
  `optimizer_method="auto"` picking Ipopt when installed (scan: examples#11);
  `PlanningProblem.metadata` documented or retired (scan: planning#14; ROADMAP §6).
- Declined 2026-09-05 (do not re-propose): scalar / list signal bounds and a coercing `x0`;
  scalar `Q` / `R` / `S` in `QuadraticCost.from_system`.
