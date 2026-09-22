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

- [ ] **R1 Tag `0.1.0`** **[ask]**. Register the GitHub Environment `pypi` and the PyPI pending
  publisher (`alx87grd` / `minilink` / `publish.yml` / env `pypi`), then
  `git tag 0.1.0 && git push origin 0.1.0`. Done when `pip install minilink` installs the
  teaching surface from PyPI.
- [ ] **R2 CI on the working branch** **[ask]**. `.github/workflows/test.yml` and `docs.yml`
  trigger on `main`, `refactor-v4`, `dev-alex`; add `dev` (or whichever branch carries the
  work) so the merge gate runs before a PR. One line each.
- [ ] **S49 Retire the Stable-Baselines3 teaching notebooks** **[ask]**. Delete
  `teaching/courses/udes_gro860/drone_ppo_sb3.ipynb` and
  `pendulum_value_iteration_vs_lqr_vs_ppo_sb3.ipynb`, their notebook overrides and allowlist
  entries, and the README rows marked Stable-Baselines3, once the course notes point only at
  the native twins.
- [ ] **S54 `plot_cost2go` colour scale** clipped at `out_of_bound_cost` by default (the
  showcase passes `jmax` by hand); review the DP `out_of_bound_cost` default.

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
- [ ] **A2 Cost parameters** **[ask — core]**. [cost-params.md](cost-params.md): `params` on
  every library cost (`QuadraticCost`, `TimeCost`, `FieldCost`, `SumCost`, `ScaledCost`),
  composite costs nested like diagram params with `cost.id`, attributes as views so the
  notebooks' in-place writes keep working; then the trajopt, DP, evaluator and RL paths pass
  `problem.params.cost` through. Done when every planner baseline `cmp`s and `jax.grad` of a
  trajopt objective with respect to `Q` runs.
- [ ] **A3 Workspace geometry (S57)** **[ask — core]**. [geometry-module.md](geometry-module.md)
  implementation steps 1–6: `core/geometry/` package (shapes, paths, track, scene, probes,
  `bind`, spatial fields, grid, catalog `oval_circuit` / `racecar_circuit` /
  `holonomic_forest`); shaping next to `Field.as_cost`; plots and overlays under
  `graphical/` with lazy methods on `Scene` / `Track`; `planning.spatial` re-exports then
  deletes in the same change as the call sites; `PurePursuit` takes a Path / Track. Done when
  the UdeS racecar trio and the RRT forest demo use the catalog, `control` imports geometry
  with no planning import, and the suite is green. Glyph rename stays S30.
- [ ] **A4 Naming quick wins** 1–4 and 6 of [naming.md](naming.md), each a few lines plus a
  test: class name as the default `name`; one closed-loop name on both `@` paths; informative
  default names where a shortcut says `Diagram`; the sampled loop's plant wrapper honours
  `plant.id`; `id` documented on `System`. Done when composed diagrams' keys, params
  dictionaries and trajectories are byte-identical before and after.
- [ ] **A5 Later nouns, each with its first consumer.** `Gaussian(mean, std=None, cov=None)` +
  `log_prob` with the disturbance convention (ROADMAP §6; before P4); parameter-dictionary
  support on sets and distributions (one flatten rule, with identification or the robust
  problem); `NoiseSource(distribution, sample_period)` replacing the hand-rolled `WhiteNoise`
  draw; `UnionSet` via `|` when reachability or multi-goal work needs it; vector bounds on
  `Saturation`; library sets and fields reading `params` (the "one signature" gap, same rule
  as A2, after it); `PlanningProblem.hamiltonian()` only if the course teaches Pontryagin; the
  RL critic as a `Field` once training is over.

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
- [ ] **P7 Generate the analysis facades**: one helper builds a delegating method from the
  target function (signature and docstring copied), explicit form kept only where the facade
  differs; a test asserts every generated `__signature__` matches its target. Done when the
  13 hand-copied methods are generated and `help(sys.bode)` is unchanged.
- [ ] **P8 Small teaching helpers**: ζ and ω_n from a complex pole pair (fields on the `pzmap`
  result or a `damping(sys)` verb); the `N` reference-scaling matrix giving `y = r` at steady
  state; the `settling_horizon` docstring (five vs eight). Done when §9.5 and §9.11 of the
  guide are each a short notebook cell.
- [ ] **P4 `estimation/`** **[held 2026-09-07 — ask to lift]**. `LuenbergerObserver(A, B, C,
  L)` as a `DynamicSystem` with ports `u`, `y` → `x_hat`; `luenberger(A, B, C, poles)` on the
  dual pair (needs P3); `kalman(A, B, C, Q, R)` from the filter Riccati equation; the
  observer + state-feedback composition ruled first (ROADMAP §6). Done when LQR + Kalman
  stabilizes the guide's cart-pendulum from a disturbed start with noise on `u` and `y`, and
  `plot_diagram` shows the Figure 12 topology.
- [ ] **P9 `TransferFunction` builds its ports once** (no clear-and-re-add); the four
  `ports` values stay. Done when no `self.inputs = {}` remains in the constructor and the
  `TransferFunction` / `Lead` / `Lag` tests pass untouched.
- [ ] **P10 Document the three `@` dispatch paths** in DESIGN (operand shape → what `@`
  builds); pin the `e`-input autowire heuristic with a test; collapse the `PROFILE_PORTS`
  rows that differ only in `plot_space` if that reads better.
- [ ] **P6 Discrete (z) tier** **[held 2026-09-07]**. Default: teach with `discretize` +
  simulation. Reopen only if the sommatif examines z-plane analysis.
- [ ] **P11 Two GRO501 notebooks** **[maintainer — student-facing]**:
  `teaching/topics/classical_control/dc_motor_propulsion.ipynb` (APP2) and
  `bicycle_autopilot.ipynb` (APP4), Basic tier, Colab-first; drafted for review. Done when
  both run top to bottom in Colab and the conda env, import only through the teaching
  surface, and agree with the guide's worked exercises.

---

## 4. v0.2 wave C — catalog, parity, new bands

- [ ] **C1 Pyro parity** open rows — [pyro-port-remaining.md](pyro-port-remaining.md); then
  the pyro → minilink migration guide in README from the name map there. **[ask]** for the
  README.
- [ ] **C2 GMC714 modelling ladder**: manipulators + the four-rung vehicle ladder as a
  `02_dynamics` lesson; robotic PID wrappers (`JointPD`, `EndEffectorPD` twins).
- [ ] **C3 Blocks**: `Sine` / `Ramp` / `Chirp` / `Delay` / `Switch`.
- [ ] **C4 Identification and generation**: `identification/fitting.py` on `rollout_batch`
  (physical params and NN weights are one verb); `trajectory_generation/` port (polynomial,
  min-snap); SMC trajectory-following demo; trajectory post-filter.
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
- [ ] **T6 the two big ones** **[ask first]**: `core/composition.py` (24-line section-map
  docstring, 45 `_` helpers, public API buried under `# Internal machinery`, eleven external
  writes to `_composition_*`; and the three `@` semantics of P10) and
  `control/mpc/controller.py` (seventeen `_` methods, hasattr-and-set from
  `export_mpc_dual_rate_computer`, the factory at line 340). Each is a conversation, then a
  step ladder of its own.
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
- [ ] **D1.2 The flatness ratchet** in `test_teaching_imports.py`: no top-level `def` or
  utility class in `examples/demos/`, no notebook cell mixing an API call with matplotlib
  code; today's offenders allowlisted, the list shrinking with D1.3.
- [ ] **D1.3 The sweep**, one file per step, maintainer reviews each diff **[ask per file]**:
  the tutorial chapters (`11_reinforcement_learning.ipynb` and `showcase_jax.ipynb` first;
  `showcase_jax` rewritten 2026-09-18, awaiting review), then the non-clean demos, then the
  teaching notebooks; the three `compile/` demos wait for V1 (their helpers *are* that
  feature); `manipulator_eom.ipynb` reviewed for which of its 26 helpers the text teaches.
  Done when the D1.2 allowlist is empty and the notebook checks pass.
- [ ] **D1.4 The planning demos and notebooks on the solution verbs** **[ask]**: the
  `planner.solve().trajectory` one-liners, the hand-rolled comparisons and the per-file
  `@dataclass` rows become `solution.plot_*` and `compare(...)` (the runnable draft of
  `vi_pendulum_lqr.py` was shown 2026-09-17).

**D2 — consolidation picks** still open from
docs/reviews/2026-09-05-consolidation-inventory.md (nothing removed without the maintainer's
pick):

- [ ] `Source.show_signal` (80 lines of bespoke matplotlib; the same picture is
  `source.plot_trajectory(tf=…)`) **[ask — user-callable]**.
- [ ] Dead modules `planning/spatial/overlays.py` (retires with A3) and
  `experimental/symbolic/mechanics/utils.py` **[ask — importable names]**.
- [ ] Deprecated benchmark shims (`run_pendulum_f_speed.py`, `run_diagram_f_speed.py`; check
  `run_study.py` presets cover the `run_step_*` / `run_simulator_*` scripts).
- [ ] MPC debug figure → `control/mpc/viz.py`.
- [ ] `HybridDiagram` hand-copied facades (with S31).
- [ ] Plotting in eight homes: write the placement rule in DESIGN first, then move
  `graphical/port_map.py` next to `control/` if agreed.

**D3 — hardening rows** (research lane, small):

- [ ] RRT `KinodynamicExtender` ignores `problem.params.system`; `extenders.py` probes
  `getattr(U, "box", None)` instead of `U.bounding_box()`.
- [ ] MPC port computes drop `params`; dual online-params façades.
- [ ] `ShootingTranscription` orphaned from the string presets.
- [ ] `ParametricMathematicalProgram` / `JaxParametricProgramEvaluator` placement (54 % duplicate
  of `optimization/evaluators/jax_evaluator.py`; [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md)).
- [ ] `HybridSimulator` conventions drift; realtime `TODO: User Architectural Review`.
- [ ] `CostDensityField` / `WorkspaceField` export decision (with A3).
- [ ] **S43** Constructor-derived `camera_scale` hints (boat, steering, propulsion, plane,
  arms, rotating cart-poles) → auto-fit or params-derived; raw ground `CustomLine`s in catalog
  skins → `ground_line()`.
- [ ] **S53** Profile `rollout_batch` with a `params` family: 278 ms vs 27 ms for the plain
  batch (pendulum, 1000 × 1000 RK4 steps). Done when the family path is within 2× of the
  plain batch.
- [ ] Two owners of one constant: `solver_warnings._DISCONTINUOUS_AUTO_DT_SCALE` duplicates
  `simulator.DISCONTINUOUS_AUTO_DT_SCALE` (RULES 5.6).
- [ ] Bare `import jax` in `analysis/lyapunov.py` and
  `trajectory_optimization/parametric_evaluator.py` → `require_jax()` (RULES 5.12).

---

## 6. v1.0 foundations

After two cohorts; each is a design conversation before code. **[ask — core]** throughout.

- [ ] **S31** `HybridDiagram` → a `System` (state `[plant; computer]`, periodic discrete
  update) or an honest `HybridLoop` rename with `%` → `on_schedule()`; the hand-copied
  facades go with it.
- [ ] **S29** `DiagramSystem.x0` / `n` / `state` as derived properties (mirror the live
  `params` view); `Simulator` drops its pre-read `refresh()`.
- [ ] **S37** Evaluator / solver re-layering: evaluators keep pure maps and one scannable
  step; integrators move to `simulation/solvers/`. Then **S27** Diffrax as an optional JAX
  solver backend.
- [ ] **S44** A single posed-geometry hook so "two functions" (`f` + drawing) is literal;
  today `tf` + skin.
- [ ] **S30** Rename the graphical `Sphere` / `Box` glyphs so no two importable public types
  share a name with the geometry solids. **[ask — public names]**
- [ ] **S32** Unify `MechanicalSystem` / `GeneralizedMechanicalSystem` (`N = I` special
  case); `Boat2D` / `Plane3D` gain `q` / `dq` ports.
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
- Declined 2026-09-05 (do not re-propose): scalar / list signal bounds and a coercing `x0`;
  scalar `Q` / `R` / `S` in `QuadraticCost.from_system`.
