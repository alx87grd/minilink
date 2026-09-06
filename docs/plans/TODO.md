# Minilink operational backlog

Step-level workboard for the phases in [ROADMAP.md §5](../../ROADMAP.md#5-phases).
Strategy, milestones, and the TRL ledger stay in ROADMAP; audit trail and
interview decision records in [docs/reviews/](../reviews/). Pyro parity rows
stay in [pyro-port-remaining.md](pyro-port-remaining.md).

Each step is sized for roughly one agent-hour and ends with a "done when"
gate. **[ask]** marks maintainer-owned territory (student-facing material,
core architecture, main-tool APIs, any feature or public-name removal):
propose, get a yes, then land. Unmarked steps are agent-managed: land, then
report. Conventions for every step: `ruff check . && ruff format --check .`,
the relevant `pytest tests/unittest/test_<domain>.py`, and DESIGN/README
updates only where a public contract changes.

| Section | Phase |
| --- | --- |
| [§1](#1-phase-d--docs-as-plan-of-record) | D — docs (in progress) |
| [§2](#2-phase-0--first-hour-safety) | 0 — first-hour safety |
| [§3](#3-phase-1--teaching-contract-and-the-gro860-path) | 1 — teaching contract + GRO860 path |
| [§4](#4-phase-2--the-jax-claim-and-the-research-facade) | 2 — JAX claim + research facade |
| [§5](#5-phase-3-and-later) | 3 / Later |
| [§6](#6-v02-pulls) | v0.2 pulls (pyro parity, GMC714, new modules) |

---

## 1. Phase D — docs as plan of record

- [x] **D1** `ROADMAP.md` rewritten (milestones, two lanes, TRL ledger with lane column, GRO860 checklist, phases, review queue, consolidation principle).
- [x] **D2** `README.md` **[ask]** — custom-plant example composes with `@` (depends on step S40 below); band-facade imports in every code block where a facade exists today; "API stability" table → the two-lane table; install section unchanged (conda recommended).
- [x] **D3** `DESIGN.md` — two-lane section, wheel scope, float64 policy, unconnected-input contract, control-block decision record, duplicated line removed.
- [x] **D4** `AGENTS.md` — student-facing import rule, demo-header rule, consolidate-never-strip, two lanes, delegation split, float64 / unconnected / verbose reminders.
- [x] **D5** this file.
- [x] **D6** `docs/plans/` — delete `control-block-contract.md` (Implemented) and `test-benchmark-consolidation.md` (Complete); shrink `pyro-port-remaining.md` to open rows plus a compact pyro→minilink name map, converting the two `*WithPositionInputs` "Done" rows (classes do not exist; DESIGN rejects the branch) to Drop and fixing the three stale paths; relabel the five draft plans "research lane — Later"; update `README.md` index.
- [x] **D7** `examples/README.md` — `projects/` and `sandbox/` (now `experimental/`) labelled research lane (outside the release contract, not CI-checked). `install.md` unchanged.

---

## 2. Phase 0 — first-hour safety

- [x] **S01 Default output grid** **[ask — Simulator API]** (direction approved 2026-09-05: fixed count).
  Touch `simulation/simulator.py` (`select_time_vector`, `select_solver`), `simulation/time_grid.py`, `simulation/static_simulator.py`, tests asserting `100001`.
  When neither `n_steps` nor `dt` is given: `n_steps = 1001` for adaptive solvers; `dt` from `smallest_time_constant` only for `euler` / `euler_fixedsteps` / `rk4_fixedsteps`; auto-RK4 under JAX keyed on the requested solver or `discontinuous_behavior`, never on `n_pts`.
  Done when `Pendulum().compute_trajectory(tf=10)` returns 1 001 samples on both backends, the JAX default still picks `scipy` (`nfev ≈ 200`), suite green.
- [x] **S02 Shape validation at compile** **[ask — core]**.
  Touch `core/compile/compiler.py` (leaf + diagram entry), `numpy_evaluators.py` constructors, `jax_evaluators.py` (probe *before* `check_jax_compatible`).
  Probe `f(x0, u_nom, 0, params)` and every `port.compute(...)`; raise `ValueError("f() of 'Name' returned shape (1,); expected (2,) for n=2")` and the port analogue; skip `f` for `n == 0`.
  Done when a wrong-shape `f` / `h` raises the message from `compute_trajectory`, `compile("numpy")`, and `compile("jax")`; test in `test_compile.py`.
- [x] **S40 `DynamicSystem` default output** **[ask — core API]**. Today `output_dim=n` with no `h` override yields `y ≡ 0` (the `00_core` custom-plant form). Decide: default `h` returns `x` when `output_dim == n` (pyro semantics), or require an explicit `h`, or keep zeros and make the README/notebook example define `h`. D2 depends on this.
- [x] **S04 README example + `@` message** **[ask — README]**. `composition.py` `_feedback_mismatch_message`: when the plant lacks the expected output port, say *"plant 'X' has no 'y' output port; pass `output_dim=…` or wire ports explicitly"* instead of "dim None". Test in `test_diagrams.py`.
- [x] **S05 Float64 policy** (approved).
  Touch `core/backends.py` (`ensure_jax_x64()` or inside `require_jax_numpy`), all JAX evaluator constructors in `core/compile/evaluators/jax_evaluators.py`, `optimization/evaluators/jax_evaluator.py`, `planning/trajectory_optimization/parametric_evaluator.py`; DP's existing `configure_jax(enable_x64=True)` calls become no-ops.
  Done when the canonical pendulum / cart-pole trajopt problems report `success=True` on `compile_backend="jax"` without the caller touching `configure_jax`; `MINILINK_JAX_X64=0` restores float32; test in `test_jax_planning.py`.
- [x] **S06 `super().__init__()` guard** **[ask — core]**. In the facade entry points (`compile`, `compute_trajectory`, `plot_*`, `animate`) one helper: if `not hasattr(self, "inputs")` raise `TypeError(f"{type(self).__name__}.__init__ must call super().__init__(n=…) before use")`. Test in `test_core.py`.
- [x] **S07 Unify verbose flag names** (approved; panel stays). `verbose` / `disp` → `verbose` on planners, optimizer, MPC; framed panel unchanged. Update call sites and docs in the same change (no aliases).
- [x] **S08 nbstripout hook**. `.pre-commit-config.yaml` → `files: ^examples/.*\.ipynb$`; strip stored outputs. Done when `pre-commit run nbstripout --all-files` is clean.
- [x] **S09 Trajopt `success` = defects satisfied** **[ask — planner API]**. `SolveMetadata` gains `max_defect` / `max_violation`; transcriptions expose `defects(z)` / `constraint_violation(z)`; `success = solver_ok or (defects ≤ tol and violation ≤ tol)`; both numbers in the solve summary. Tests in `test_planning.py`.

---

## 3. Phase 1 — teaching contract and the GRO860 path

- [x] **S11 Teaching-surface registry + Basic-tier clean-env test**. Registry lives in `tests/unittest/test_teaching_surface.py` (no new library module — the contract is test-only): every registered name imports and has a docstring; every name lives under a teaching-lane path; a subprocess smoke that blocks `graphviz` / `jax` / `meshcat` / `plotly` / `pygame` / `sympy` and runs sim + plot + phase plane + animate + linearize + LQR + VI.
- [x] **S12 Import-layer CI check**. Extend `test_public_imports.py`: AST-walk `examples/learn/` and `examples/demos/`; each `from minilink… import` is root-prelude, band-facade, or allowlisted; start with today's deep imports allowlisted, shrink through S13–S15.
- [x] **S13 Band facades** **[ask — public names]**. `simulation/__init__.py` (`Simulator`, `StaticSimulator`), `planning/__init__.py` (`PlanningProblem`, `TrajectoryOptimizationPlanner`, `DynamicProgrammingPlanner`, `StateSpaceGrid`, `LookupTableController`, `RRTPlanner`, `RRTStarPlanner`), `core/__init__.py` (`Trajectory`, `DiagramSystem`, `QuadraticCost`, sets) — same lazy `_EXPORTS` pattern as `control/`. Deep imports keep working.
- [x] **S14 Rewrite imports in `examples/learn/intro/`** **[ask — notebooks]**, two steps (`00`–`05`; `06`–`10` + showcases). Done when notebook smoke passes and the S12 allowlist shrinks.
- [x] **S15 Rewrite imports in `examples/learn/teaching/` and `examples/demos/`** **[ask]**, four steps by folder. Done when `run_all_demos.py` passes 60/60.
- [x] **S39 Demo-script headers → one-line title** (ruled 2026-09-06: "super minimalist"). All 68 demo / experimental scripts carry a one-line docstring; run instructions, section maps and wiring maps dropped; key maps kept as comments in the games; flag explanations sit next to the constants. Notebooks untouched.
- [ ] **S33 `Sys2Gym.step` on the compiled evaluator** (GRO860 perk; agent lane). Compile once in `__init__`; step via `integrate_zoh` (RK4); `backend=` kwarg for JAX. Done when the drone PPO notebook trains to the same qualitative policy and a parity test against the Euler path at small `dt` passes.
- [ ] **S38 DP metadata honesty** **[ask — planner API]**. `DynamicProgrammingOptions.final_time` reads `problem.tf` when set; `SolveMetadata.success` reports convergence, not always `True`; VI notebooks wire with `vi_ctl @ plant` (notebook edits **[ask]**).
- [ ] **S16 Delete the `_jit` aliases** **[ask — evaluator names]**. `register_jit_aliases`, `_TRACE_TIER_SUFFIXES`, six call sites, `test_f_jit_alias_identity`, DESIGN §5 sentence. 24 alias methods, zero call sites.
- [ ] **S17 Delete the 28 unreferenced evaluator methods** **[ask — evaluator names]**: `euler_integrate_ivp_p`, `euler_integrate_ivp_trace`, `euler_integrate_ivp_trace_p`, `euler_integrate_zoh_p`, `euler_integrate_zoh_trace_p`, `euler_step_ivp_p`, `euler_step_ivp_trace`, `euler_step_ivp_trace_p`, `euler_step_trace`, `euler_step_trace_p`, `f_ivp_scipy`, `f_scipy`, `integrate_zoh_p`, `outputs_trace_p`, `rk4_integrate_ivp_p`, `rk4_integrate_ivp_trace`, `rk4_integrate_ivp_trace_p`, `rk4_integrate_linear_trace`, `rk4_integrate_linear_trace_p`, `rk4_integrate_zoh_trace_p`, `rk4_step_ivp`, `rk4_step_ivp_p`, `rk4_step_ivp_trace`, `rk4_step_ivp_trace_p`, `rk4_step_trace_p`, `rollout_p`, `step_block`, `step_trace_p`. Keep the frozen subset DESIGN §5 names.
- [x] **S18 Wheel excludes the research lane** (approved). `pyproject.toml` `[tool.hatch.build.targets.wheel]` excludes `minilink/experimental/symbolic/**`, `minilink/experimental/engines/**`, `minilink/experimental/c_export.py`; `examples/` were never shipped. Done when `python -m build` produces a wheel without those paths and the suite (run from the repo) stays green.
- [x] **S19 `c_export` in the nightly sweep**. Add both `examples/experimental/c_export/` scripts to the nightly manifest with `requires: ["jax"]` (TRL row already in ROADMAP).
- [x] **S20 Nightly full demo sweep**. `.github/workflows/nightly.yml` running `run_all_demos.py --timeout 120 --continue-on-error` and `run_notebook_checks.py` on a schedule + `workflow_dispatch`.
- [x] **S21 Branch hygiene** **[ask — you run the script]** — script generated: [../reviews/2026-09-05-branch-cleanup.md](../reviews/2026-09-05-branch-cleanup.md). List the 31 local branches merged into `main` and the 42 `cursor/*` remotes with dates; produce the delete script.
- [x] **S41 Consolidation inventory** (report only) — [../reviews/2026-09-05-consolidation-inventory.md](../reviews/2026-09-05-consolidation-inventory.md). Ranked table of maintenance-cost items — text edited twice, dead API, boilerplate classes, twin plants, plotting homes, research code in the teaching tree — each with lines, blast radius, and what it does *not* remove. Maintainer picks; picked items become steps in §4/§5.
- [x] **S42 Multiple-shooting parametric guard** (carried over; cheap). `MultipleShootingTranscription` inherits collocation `transcribe_parametric` and silently builds wrong defects; override to `NotImplementedError` and replace the planner's `hasattr` check with an explicit capability flag.

---

## 4. Phase 2 — the JAX claim, and the research facade

- [ ] **S22a–h `xp` sweep**, one module per step **[ask per module — catalog is student-facing]**: `vehicles/steering.py`; `pendulum/cartpole.py`; `aerial/drone.py`; `manipulators/arms.py` (two steps); `marine/boat.py`; `mass_spring_damper/linear.py`; `vehicles/dynamic_bicycle.py`; then `rocket`, `mountain_car`, `suspension`, `oscillators`, `propulsion`. Pattern: `xp = array_module(x, u)` after params unpack; `np.` → `xp.` in `f` / `h` / port computes / `H` / `C` / `g`; no in-place writes; `np` stays for constructor metadata. Done when `compile("jax")` succeeds and `f` matches NumPy on three random points.
- [ ] **S23 Catalog both-backends contract test**. Parametrize over `minilink.catalog.__all__`; assert both backends compile and agree; `xfail` list = modules not yet swept, shrinking to empty.
- [ ] **S24 Retire `JaxCartPole`** **[ask — public name]** after S22b/S23.
- [ ] **S25 Vehicle teaching ladder** **[ask — catalog]** (ladder approved 2026-09-05): keep holonomic point → kinematic bicycle (car skin) → dynamic bicycle (linear tires) → `BicycleDynRate`; `named_ports=` constructor flag replaces the six `*Ports` twins; `BicycleKin` / `BicycleAcc` / `BicycleDyn` / `TauRate` / `Servo` / `Engine`, `ConstantSpeedKinematicCar`, `DynamicHolonomicMobileRobot`, `HolonomicMobileRobot3D`, `UdeSRacecar`, `CarProfile` move under `examples/projects/pathtracking/` with their scenarios. `DynamicBicycle`'s named `w_rear` / `delta` ports are a settled decision and stay.
- [ ] **S26 `rollout_batch` research facade** **[ask — evaluator API]**. `JaxDynamicsEvaluator.rollout_batch(x0s, u_sequences=None, t0, dt, n_steps, params=None)` on `jax.vmap` of the trace-tier rollout, `params` optionally batched; demo `examples/demos/compile/rollout_param_family.py` (sweep `m`, `l` — the Buckingham-π experiment); test against a loop of single rollouts.

---

## 5. Phase 3 and Later

- **Declined 2026-09-05 (do not re-propose):** scalar/list signal bounds and a coercing `x0`; scalar `Q`/`R`/`S` in `QuadraticCost.from_system`. Bounds and cost matrices stay explicit arrays in student-facing code.
- [x] **Textbook pass** — root prelude = teaching surface (one import line); `DynamicProgrammingPlanner` one-object setup + automatic infeasible-set cleanup; trajopt `live_plot=`; pendulum camera default; demo/notebook ceremony removed (see the session-log addendum).

After the term, in the order the cohort's questions suggest:

- [ ] **S29** `DiagramSystem.x0` / `n` / `state` as derived properties (mirror the live `params` view); `Simulator` drops its pre-read `refresh()`. **[ask — core]**
- [ ] **S30** Rename the graphical `Sphere` / `Box` glyphs so no two importable public types share a name with `core.geometry`. **[ask — public names]**
- [ ] **S31** `HybridDiagram` → `HybridLoop`, `%` → `on_schedule()` — or promotion to a `System` (ROADMAP §6, v1.0). **[ask — core]**
- [ ] **S32** Unify `MechanicalSystem` / `GeneralizedMechanicalSystem` (`N = I` special case); `Boat2D` / `Plane3D` gain `q` / `dq` ports. **[ask — core]**
- [ ] Frequency analysis — minimal NumPy `pole_zero_map` / `nyquist` / `margin` / `ss2tf` **or** a python-control bridge; decision postponed (ROADMAP §6). **[ask]**
- [ ] PyPI publication as a third install option (conda stays recommended). **[ask]**
- [ ] **S36** iLQR planner from parts (`jacfwd` of `f_trace`; idea, research lane).
- [ ] **S27** Diffrax as an optional JAX solver backend (later; not short-term).
- [ ] **S37** Evaluator / solver re-layering (v1.0; after S27).
- [ ] Rename pass: the 43 `_method` names on `System` subclasses → plain names (maintainer style rule). **[ask per module]**
- [ ] Carried-over hardening rows (research lane): RRT `KinodynamicExtender` ignores `problem.params.system`; MPC port computes drop `params`; `ShootingTranscription` orphaned from string presets; `ParametricMathematicalProgram` / `JaxParametricProgramEvaluator` placement (54% duplicate of `optimization/evaluators/jax_evaluator.py`); dual online-params façades; `HybridSimulator` conventions drift; `HybridDiagram` hand-copied facades; realtime `TODO: User Architectural Review`; `CostDensityField` / `WorkspaceField` export decision.
- [ ] Later ideas: scene params / `J(z, p)` bind ([planning-pipeline-architecture.md](planning-pipeline-architecture.md)); `SolverFactory` ([optimizer-parametric-wiring.md](optimizer-parametric-wiring.md)); `MjxPlant`; Pacejka; stochastic forcing; neural MLP; ROS2 / FMI; sparse long-horizon trajopt; parametric `Shape` / `Set` / `Cost` overrides; trajectory post-filter; RRT-Connect; shared RNEA serial-chain stack; ABA on other RNEA arms.

---

## 6. v0.2 pulls

- [ ] Pyro parity open rows — [pyro-port-remaining.md](pyro-port-remaining.md).
- [ ] GMC714 modelling ladder: manipulators + the four-rung vehicle ladder as a `02_dynamics` lesson; robotic PID wrappers; trajectory LQR.
- [ ] Estimation — Luenberger, Kalman. Identification — `fitting.py` (after S26).
- [ ] `trajectory_generation/` port.
- [ ] Blocks: `Sine` / `Ramp` / `Chirp` / `Delay` / `Switch`.
- [ ] SMC trajectory-following demo; remaining pyro game demos → `simulation/realtime/` or explicit drop.
- [ ] Pyro → minilink migration guide in README (uses the name map kept in the parity doc).
