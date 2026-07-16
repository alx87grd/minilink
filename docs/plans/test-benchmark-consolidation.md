# Test & benchmark consolidation plan

Status: **Complete** — six-layer vision landed; optional G1 perceptual PNG hash remains a nightly/local enhancement only.

**Current names** (see [tests/README.md](../../tests/README.md#entry-points)): contract tests · regression gates · benchmark study · graphics contract · graphics visual · demo checks. Paths: `tests/demo_checks/`, `tests/run/run_contract_tests.py`, etc.

---

## Testing vision (authoritative)

Six layers, bottom to top. **Unit tests belong to library modules (`minilink/`).**
Demo and catalog scripts are **demo-check only** (not duplicated in pytest).

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Demo checks           catalog + demo scripts — must not throw         │
├─────────────────────────────────────────────────────────────────────────┤
│  Graphics visual       you look: Meshcat, Matplotlib, Plotly live      │
├─────────────────────────────────────────────────────────────────────────┤
│  Graphics contract     agent/CI: runs, saves PNG, finite draw-lists    │
├─────────────────────────────────────────────────────────────────────────┤
│  Benchmark study       new machine / GPU — tables, not CI gates        │
├─────────────────────────────────────────────────────────────────────────┤
│  Regression gates      accuracy JSON goldens + guarded compute time      │
├─────────────────────────────────────────────────────────────────────────┤
│  Contract tests        pytest: API types, shapes, compile/sim behavior   │
└─────────────────────────────────────────────────────────────────────────┘
```

### Contract tests (pytest, `tests/unittest/`)

**Purpose:** Core library correctness without running demos. “This function returns
the right type/shape; this public API honors DESIGN.”

| What | How | CI |
| --- | --- | --- |
| Types, shapes, params override, wiring | Parametrized pytest on `minilink/` | `pytest` every PR |
| Compile / evaluator / sim / hybrid / MPC **behavior** | Domain test modules (~22 after consolidation) | `pytest` |
| Foundational catalog **math** only | 2–3 plants with reference `H`, `g`, matrices (not 40-class loop) | `pytest` |

**Not here:** demo scripts, wall-clock speed, pixel appearance, “every catalog class.”

**Command:** `pytest` (optional: `pytest -m "not optional"` for minimal deps)

**Target files:** see §Target layout — `test_core.py`, `test_compile.py`, `test_simulation.py`, …

---

### Layer 2 — Regression (accuracy + performance gates, `benchmarks/`)

**Purpose:** Catch broken **simulation**, **trajectory optimization**, **NLP/MPC**
paths — both **solution accuracy** and **compute time** on key scenarios.

Two gate types (never mix them up):

| Gate | Stored truth | Compare rule | Cross-machine safe? |
| --- | --- | --- | --- |
| **Accuracy** | Committed vectors in `benchmarks/baselines/*.json` (`x_tf`, `checkpoint_t*`, `eq_inf`, success flags) | Absolute tolerance / vector match | **Yes** — physics truth, not host-specific |
| **Speed** | Baseline seconds recorded on a reference host | Current ≤ baseline × **factor** (4× default; 6× in CI for `solve_s`/`nlp_s`) | **Partially** — never hard-code “must finish in 0.05 s”; use factor + gate only solver wall time (`solve_s`, `nlp_s`, speedup ratios), not end-to-end `wall_s` in CI |

**Accuracy truth pipeline:**

1. Live “truth” where needed (e.g. `scipy_ultra` sim) → compare candidate vs truth (`rel_err_l2`).
2. Truth vs **committed golden** (`truth.x_tf`) — catches broken sim even if candidate still matches truth.
3. Checkpoints along trajectory vs JSON — catches drift mid-horizon.

**Per-module coverage (where it makes sense):**

| Module / path | L2 suite | Accuracy | Speed |
| --- | --- | --- | --- |
| Compile / `f()` | `core_perf` | diagram `dx` residual, sim final state | speedup vs native |
| Simulator | `core_perf`, `integration` | checkpoint goldens, `rel_err_l2` | `solve_s` (sim wall) |
| Optimizer / NLP | `solve_speed` | success flag | `optimizer.*.solve_s` |
| Trajopt (NumPy) | `solve_speed` | success | `trajopt.pendulum.numpy_slsqp.solve_s` |
| Trajopt (JAX) | `integration`, `e4` | traj + checkpoints | `solve_s` |
| MPC product | `f_mpc` | hybrid/hand-loop/dual-rate states | `nlp_s` |

**Command (local pre-handoff):**

```bash
python benchmarks/run_regression_check.py --suite all
```

**Command (CI — already wired):**

```bash
python benchmarks/run_regression_check.py --suite all --tiny --factor 6 \
  --speed-gate-suffixes solve_s,nlp_s,speedup
```

**Refresh baselines (intentional change only):** `--update` + review JSON diff.

**Status:** ✅ unified runner + 5 suites + CI job landed.

---

### Layer 3 — Benchmark utilities (machine exploration, not testing)

**Purpose:** “I have a new computer / GPU — how fast are backends here?”
Utility scripts, **not** CI gates. Output human-readable tables for comparison
across machines you own.

| Script family | Question it answers |
| --- | --- |
| `run_pendulum_f_speed.py`, `run_diagram_f_speed.py`, … | Native vs NumPy vs JAX `f()` / `step()` |
| `run_simulator_speed_matrix.py` | Solver × backend matrix vs ultra truth |
| `run_trajopt_backends.py`, `run_optimizer_backends.py` | Which backend/solver wins **on this host** |
| `run_dp_backends.py`, `run_rrt_nearest_backends.py` | DP / RRT backend sweep |
| `run_pyro_minilink_parity.py` | External Pyro env comparison |

**Planned:** `python benchmarks/run_study.py --preset …` ✅ landed.

**Command:**

```bash
python benchmarks/run_study.py --list
python benchmarks/run_study.py --preset sim --mode matrix
```

Legacy `run_*_speed.py` scripts are deprecation shims forwarding to `run_study.py`.

**Rule:** Results are **for you on your machine** — do not commit as gates unless
folding a representative scenario into L2 with factor tolerance.

---

### Layer 4 — Graphics automated (agent + headless CI)

**Purpose:** Agent (or CI) verifies graphics **pipelines run** without you looking.

| Check | Pass criterion |
| --- | --- |
| Draw-list contract | Finite transforms, expected primitive keys (`geometry_smoke`) |
| Static render | Agg PNG written, finite pixels, non-empty variance |
| Demo/flagship smoke | Script completes; figure saved to temp path |
| Plotly (optional) | Figure object builds (`@pytest.mark.plotting`) |

**Not here:** “does it look pretty” — that is L5.

**Commands (planned):**

```bash
pytest tests/unittest/test_graphics.py tests/unittest/test_flagship_graphics_contract.py
python examples/scripts/_smoke/run_flagship_graphics.py --headless --out /tmp/ml-gfx
```

**Status:** ✅ kinematic manifest smoke + **`test_flagship_graphics_contract.py`** (G0 draw-list); G1 perceptual hash still optional.

---

### Layer 5 — Graphics visual (you confirm on your machine)

**Purpose:** One command opens **each major output channel** so **you** can confirm
Meshcat, Matplotlib animation, Plotly live traj, etc. Agent prepares; you judge.

| Pipeline | Script opens | You verify |
| --- | --- | --- |
| Matplotlib kinematic / animate | short flagship animate, `show=True` or saved + open | geometry, camera |
| Meshcat 3D | engine / tire demo smoke | scene loads in browser |
| Plotly live trajopt | cartpole live plot demo | interactive plot updates |
| pygame (optional) | viz demo | window / headless dummy in CI only |

**Command (planned):**

```bash
python examples/scripts/_smoke/run_graphics_visual_check.py
# blocking prompts: "Press Enter after confirming Meshcat…"
```

**CI:** skip L5 entirely. **Agent workflow:** run L4, then tell user “run L5 locally.”

**Status:** ✅ L5 checklist script (`run_graphics_visual_check.py`); local-only, not CI.

---

### Layer 6 — Smoke runners (catalog + demos, no per-script tests)

**Purpose:** “At least these scripts **don't throw**.” No unittest per demo file.

| Scope | Mechanism | Depth |
| --- | --- | --- |
| **Catalog** | `catalog/<pkg>/smoke.py` + `run_catalog_smokes.py` | instantiate, `f` finite, optional 3-step sim, `geometry_smoke` |
| **Demos** | whitelist manifest + `run_smoke()` in each flagship demo | short `tf`, Agg, no `show` |
| **All demos (optional)** | `run_all_demos.py --continue-on-error` | subprocess each script, report table — nightly only |

**Pytest surface:** one subprocess test per aggregator, not 78 test files.

```bash
python examples/scripts/_smoke/run_catalog_smokes.py --fast
python examples/scripts/_smoke/run_flagship_demos.py
# optional nightly:
python examples/scripts/_smoke/run_all_demos.py --timeout 120
```

**Status:** ✅ catalog registry + aggregators; 11 flagship demos in manifest.

---

## Quick reference — what runs where

| Your requirement | Layer | Command | CI? |
| --- | ---: | --- | ---: |
| API / type / module contracts | L1 | `pytest` | ✅ |
| Sim + trajopt accuracy goldens | L2 | `run_regression_check.py --suite all` | ✅ accuracy |
| NLP/trajopt solve time regression | L2 | same + speed suffixes | ✅ partial |
| New GPU / machine benchmarks | L3 | `run_study.py` (+ legacy shims) | ❌ |
| Graphics pipelines run, PNG exists | L4 | pytest graphics + smoke scripts | ✅ |
| You eyeball Meshcat / Plotly / MPL | L5 | `run_graphics_visual_check.py` | ❌ |
| Catalog + demos don't crash | L6 | `run_catalog_smokes.py`, `run_flagship_demos.py` | ✅ |

---

## Implementation roadmap (order)

| Phase | Layer | Deliverable | Status |
| ---: | ---: | --- | --- |
| 0 | — | This vision doc + inventory table | ✅ |
| 1 | L2 | Unified `run_regression_check`, `solve_speed`, CI regression job | ✅ |
| 2 | L1 | Merge 91 pytest files → ~22 domain modules | ✅ (21 files) |
| 3 | L3 | `run_study.py` replaces scattered benchmark CLIs | ✅ |
| 4 | L6 | Catalog smokes + demo whitelist aggregators | ✅ |
| 5 | L4 | Flagship graphics headless manifest + pytest | ✅ |
| 6 | L5 | Interactive visual check script for local use | ✅ |
| 7 | L1 | Shrink catalog unittest to 3-plant contracts only | ✅ (with Phase 4) |

---

## Problem statement (inventory)

The repo has grown test and benchmark surface area without periodic cleanup:

| Area | Count today | CI default |
| --- | ---: | --- |
| `tests/unittest/test_*.py` | **22 files**, ~**780** `test_*` functions | `pytest` (all collected; optional deps skip at runtime) |
| `benchmarks/run_*.py` | **2** primary + shims | L2 in CI; L3 manual |
| Gated regression baselines | **5** JSON suites + CI job | ✅ L2 in CI |
| `examples/scripts/` | **78** demos | L6 smokes (11 flagship whitelist) |

`tests/README.md` already states the philosophy (contracts over trivia, table-driven over duplicate files). The **six layers above** are the organizing principle; sections below retain the file-level inventory and migration tables from the first draft.

---

## Legacy section labels (mapped to L1–L6)

Older draft used Layer A–D. Mapping:

| Old | New |
| --- | --- |
| Layer A (contracts) | **L1** + part of **L4** (draw-list) |
| Layer B (regression JSON) | **L2** |
| Layer C (manual perf sweeps) | **L3** |
| Layer D (demos) | **L6** (+ L5 for visual) |
| Phase 5 catalog smokes | **L6** |
| Phase 6 flagship graphics | **L4** auto + **L5** visual |

---

## What we actually need to validate (detail — L1 domains)

Stable public APIs that break user code or DESIGN contracts (**L1**):

1. **System model** — `System` / `DynamicSystem` / `StepSystem`, signals, params override, evolution maps, composition, feedback wiring.
2. **Compile & evaluators** — diagram compile, algebraic loops, execution plan, NumPy/JAX evaluator tiers, parametric compile, math-program evaluators, static/step leaf dispatch.
3. **Continuous simulation** — `Simulator` solvers/backends, discontinuous closed-loop, ZOH integration, `StaticSimulator`, scheduled `Computer`.
4. **Discrete & hybrid** — `StepDiagramSystem`, rollouts, hybrid diagram wiring, `HybridSimulator`, multi-rate, fine recording, SMC hybrid parity.
5. **Dynamics catalog** — deep equation contracts on **2–3 representative plants** only; broad plant coverage moves to **headless catalog smokes** (§Catalog & demo smokes), not per-class unittest loops.
6. **Mechanical & robotics** — `MechanicalSystem`, `Manipulator`, task ports, IK, model-based controllers.
7. **Blocks & control** — signal blocks, linear control laws, standard feedback resolution.
8. **Analysis** — linearize, structural, equilibria, LQR, modal, frequency, phase plane, discretization.
9. **Planning** — problem/transcription architecture, UI constructors, RRT/RRT\*, spatial collision/fields, reference paths, DP policy synthesis.
10. **Optimization & costs** — `Optimizer` backends, cost primitives, set helpers.
11. **MPC product path** — stateless block, warm-start controller, planner compile, hybrid closed-loop parity vs hand loops, NumPy rebuild mode, `export_to_computer`.
12. **Graphics contract** — camera transform, implicit `world` frame, draw-list keys; **flagship demo pipeline smokes** (§Flagship graphics validation); catalog kinematic render via smoke registry (not a 40-class unittest loop).

**Non-goals for L1:** wall-clock performance ratios, third-party print formatting, demo script execution, pixel appearance.

### Layer 2 detail — numerical regression (committed JSON)

End-to-end trajectories, checkpoint goldens, and loose speed gates:

| Suite | Baseline | Independent checks |
| --- | --- | --- |
| `core_perf` | `benchmarks/baselines/core_perf.json` | `f()` compile speed ratios; dense diagram `dx` residual; sim final-state vs truth; truth vector vs golden |
| `integration_check` | `benchmarks/baselines/integration_check.json` | double-pendulum & showcase-pendulum checkpoints; cart-pole JAX trajopt accuracy + `solve_s` |
| `solve_speed` | `benchmarks/baselines/solve_speed.json` | standalone NLP + NumPy pendulum trajopt `solve_s` |
| `e4_trajopt_parity` | `benchmarks/baselines/e4_trajopt_parity.json` | TOP rebuild + MPC parametric on cartpole/pendulum/bicycle |
| `f_mpc_parity` | `benchmarks/baselines/f_mpc_parity.json` | hybrid ZOH, hand-loop, dual-rate MPC trajectories + `nlp_s` |

Single entry point target: `python benchmarks/run_regression_check.py --suite all`.

### Layer 3 detail — performance studies (manual utilities)

Backend and solver sweeps with human-readable tables — informative, not gated:

- `f()` / `step()` micro-benchmarks (native vs NumPy vs JAX evaluators).
- Simulator solver × backend matrix vs ultra truth.
- Trajopt transcription × backend × solver presets.
- Optimizer NLP presets on textbook cases.
- DP loop vs NumPy vs JAX.
- RRT nearest brute_force vs kd_tree (dense scene).
- Pyro vs Minilink DP/RRT parity (optional external env).

Single entry point target: `python benchmarks/run_study.py --preset <name>` (new), replacing many `run_*_speed.py` / `run_*_backends.py` clones.

### Layer 6 detail — demos (smoke only)

`examples/scripts/` remain user-facing workflows; overlap with tests should **shrink** (MPC hybrid parity tests duplicating demo logic is a consolidation opportunity — extract shared scenario builders into `tests/` or `benchmarks/scenarios/` helpers, not delete checks).

---

## Target layout (after consolidation)

### Pytest modules (~22 files, down from 91)

| New file | Domains merged |
| --- | --- |
| `test_core.py` | core system types, composition, standard feedback, evolution maps |
| `test_backends.py` | array module dispatch + native math helpers |
| `test_compile.py` | full compile pipeline, static/step leaves, evaluator API/tiers, math-program evaluators |
| `test_diagrams.py` | flow diagrams, wiring mixin, façade split (diagram-facing) |
| `test_simulation.py` | Simulator, static sim, discontinuous solvers, ZOH integrate, Computer |
| `test_step_discrete.py` | StepSystem, StepDiagram, rollouts, topology, JAX step compile smoke |
| `test_hybrid.py` | HybridDiagram, HybridSimulator, multi-rate, recording, SMC hybrid |
| `test_dynamics_catalog.py` | catalog migration smoke, deep plant contracts, manipulator catalog cross-check |
| `test_mechanical_robotics.py` | mechanical abstraction, manipulator ports, robotics wrappers, IK, model-based control, JAX mechanical smoke |
| `test_blocks.py` | routing/nonlinear/filters, signal colors, sources, neural blocks |
| `test_control_analysis.py` | control blocks, analysis verbs, modal/frequency/phase plane, discretize, state-space |
| `test_costs_optimizer.py` | costs, sets usage, Optimizer backends |
| `test_planning.py` | planning architecture, UI constructors, RRT/RRT\*, spatial, reference paths, DP |
| `test_mpc.py` | all MPC blocks/planners/hybrid parity scenarios (parametrized) |
| `test_graphics.py` | camera, world frame, kinematic manifest regression, overlays, plotly, optional viz |
| `test_geometry.py` | core geometry primitives (unchanged scope) |
| `test_engine_jax.py` | physics engine wrapper, contact JAX, ANCF tire (optional) |
| `test_jax_planning.py` | JAX trajopt smoke, kinematic bicycle twin (optional) |
| `test_symbolic.py` | SymPy mechanics (optional) |
| `test_benchmark_smoke.py` | import/API smoke for benchmark helpers + baseline compare unit tests |

Shared helpers stay as modules (not test files): `conftest.py`, `planning_helpers.py`, `graphics_contract_helpers.py`.

Fixtures stay: `tests/fixtures/kinematic_baseline/`, `tests/fixtures/benchmark_baseline/minimal_core_perf.json`.

### Benchmark entry points (~2 CLI + library)

| New entry | Replaces |
| --- | --- |
| `run_regression_check.py --suite {core_perf,integration,e4,f_mpc,all}` | `run_regression_check.py`, `run_e4_trajopt_parity.py`, `run_f_mpc_parity.py` |
| `run_study.py --preset …` | all tier-2 `run_*_speed.py`, `run_*_backends.py`, `run_pyro_minilink_parity.py` |

Library modules (`f_evaluators.py`, `simulation.py`, `suites/`, `scenarios/`, `systems/`) remain; runners become thin wrappers or are deleted after `run_study.py` lands.

---

## Master mapping table

**Columns:** current script → what it validates → consolidated owner → notes / merge strategy.

### Pytest — core & compile

| Current file | Validates | New file | Notes |
| --- | --- | --- | --- |
| `test_core.py` | `VectorSignal`, `System`/`DynamicSystem` ports, params | `test_core.py` | absorb related core-adjacent files |
| `test_composition.py` | diagram composition, autowire, cascade helpers | `test_core.py` | same layer |
| `test_standard_feedback.py` | `resolve_standard_feedback` | `test_core.py` | |
| `test_system_evolution_maps.py` | evolution map typing on hierarchy | `test_core.py` | |
| `test_backends.py` | `array_module`, JAX configure, backend labels | `test_backends.py` | keep |
| `test_backend_native_math_helpers.py` | native set/cost equation helpers | `test_backends.py` | merge |
| `test_compile_pipeline.py` | algebraic loops, execution plan, NumPy/JAX diagram evaluators, params contract | `test_compile.py` | largest compile owner |
| `test_compile_static.py` | static leaf compile dispatch (NumPy/JAX) | `test_compile.py` | |
| `test_compile_step_leaf.py` | StepSystem leaf compile dispatch | `test_compile.py` | |
| `test_evaluator_api.py` | compiled evaluator public surface | `test_compile.py` | |
| `test_evaluator_tiers.py` | fast vs trace tier behavior | `test_compile.py` | |
| `test_mathematical_program_evaluators.py` | pure math programs + compiled evaluators | `test_compile.py` | |
| `test_diagrams.py` | `DiagramSystem` wiring, interpreted `f` | `test_diagrams.py` | keep name |
| `test_wiring_mixin.py` | `WiredDiagramMixin` parity, topology export | `test_diagrams.py` | |
| `test_facades_split.py` | façade MRO, sim guards, compile types | `test_diagrams.py` | diagram-related façade only |
| `test_facades_rollout.py` | step rollout façade | `test_step_discrete.py` | move to discrete module |

### Pytest — simulation & hybrid

| Current file | Validates | New file | Notes |
| --- | --- | --- | --- |
| `test_simulator.py` | `Simulator` backends/solvers, compile-once, JAX paths | `test_simulation.py` | |
| `test_static_simulator.py` | `StaticSimulator` boundary output recording | `test_simulation.py` | |
| `test_discontinuous_solvers.py` | SMC/discontinuous closed-loop regression | `test_simulation.py` | |
| `test_integrate_zoh.py` | `IntegrationMixin.integrate_zoh` | `test_simulation.py` | |
| `test_computer.py` | scheduled `Computer` runtime | `test_simulation.py` | |
| `test_as_computer.py` | `as_computer`, `%` operator | `test_simulation.py` | |
| `test_hybrid_boundary_connect.py` | `HybridDiagram` boundary wiring | `test_hybrid.py` | |
| `test_hybrid_closed_loop.py` | `hybrid_closed_loop` helper | `test_hybrid.py` | |
| `test_hybrid_simulator.py` | `HybridSimulator` stepping/recording | `test_hybrid.py` | |
| `test_hybrid_topology.py` | hybrid topology export/render | `test_hybrid.py` | |
| `test_hybrid_multi_rate.py` | multi-rate computer + plant | `test_hybrid.py` | |
| `test_hybrid_fine_recording.py` | fine plant trajectory recording | `test_hybrid.py` | |
| `test_smc_hybrid.py` | hybrid SMC vs hand ZOH loop | `test_hybrid.py` | |

### Pytest — step / discrete

| Current file | Validates | New file | Notes |
| --- | --- | --- | --- |
| `test_step_system.py` | `StepSystem` leaf contract | `test_step_discrete.py` | |
| `test_step_diagram.py` | `StepDiagramSystem` wiring, interpreted step | `test_step_discrete.py` | |
| `test_step_diagram_topology.py` | step-diagram topology export | `test_step_discrete.py` | |
| `test_step_diagram_rollout.py` | compiled step diagram rollout parity | `test_step_discrete.py` | |
| `test_step_rollout.py` | `StepRollout`, `gather_u`, JAX rollout | `test_step_discrete.py` | |
| `test_step_diagram_jax.py` | JAX compile smoke for step diagrams | `test_step_discrete.py` | optional-marked section |

### Pytest — dynamics & mechanical

| Current file | Validates | New file | Notes |
| --- | --- | --- | --- |
| `test_catalog_migration.py` | broad catalog import/instantiate smoke | `test_dynamics_catalog.py` | |
| `test_catalog_plant_contracts.py` | deep dynamics + graphics on representative plants | `test_dynamics_catalog.py` | keep as parametrized deep block |
| `test_manipulators.py` | all catalog manipulator classes cross-check | `test_dynamics_catalog.py` | |
| `test_dynamic_bicycle_uy.py` | `JaxDynamicBicycleRateInputsUY` | `test_dynamics_catalog.py` | or `test_jax_planning.py` if only JAX |
| `test_mechanical.py` | `MechanicalSystem` dimensions, `f`, ports | `test_mechanical_robotics.py` | |
| `test_generalized_mechanical.py` | generalized coordinates variant | `test_mechanical_robotics.py` | |
| `test_mechanical_jax.py` | JAX mechanical smoke | `test_mechanical_robotics.py` | `@pytest.mark.jax` section |
| `test_manipulator.py` | `Manipulator` joint/task ports | `test_mechanical_robotics.py` | |
| `test_robotic.py` | task ports, robotic wrappers | `test_mechanical_robotics.py` | |
| `test_inverse_kinematics.py` | numerical IK on manipulator | `test_mechanical_robotics.py` | |
| `test_modelbased.py` | model-based mechanical controllers | `test_mechanical_robotics.py` | |

### Pytest — blocks, control, analysis

| Current file | Validates | New file | Notes |
| --- | --- | --- | --- |
| `test_blocks.py` | core block types | `test_blocks.py` | keep |
| `test_signal_blocks.py` | routing, nonlinear, filters | `test_blocks.py` | merge |
| `test_signal_colors.py` | signal color registry | `test_blocks.py` | |
| `test_sources_white_noise.py` | white noise source block | `test_blocks.py` | |
| `test_neural_blocks.py` | neural controller blocks | `test_blocks.py` | |
| `test_control_linear.py` | PID, lead/lag, etc. | `test_control_analysis.py` | |
| `test_analysis_control.py` | linearize, structural, equilibria, LQR | `test_control_analysis.py` | |
| `test_modal.py` | modal analysis | `test_control_analysis.py` | |
| `test_frequency.py` | frequency response | `test_control_analysis.py` | |
| `test_phase_plane.py` | phase plane utilities | `test_control_analysis.py` | |
| `test_discretize.py` | continuous→step discretization | `test_control_analysis.py` | |
| `test_state_space_system.py` | state-space wrapper | `test_control_analysis.py` | |

### Pytest — optimization, planning, MPC

| Current file | Validates | New file | Notes |
| --- | --- | --- | --- |
| `test_costs.py` | cost function primitives | `test_costs_optimizer.py` | |
| `test_optimizer.py` | `Optimizer` SciPy/JAX/IPopt backends | `test_costs_optimizer.py` | |
| `test_dynamic_programming.py` | DP policy synthesis | `test_planning.py` | |
| `test_planning_architecture.py` | transcriptions, callbacks, problem wiring | `test_planning.py` | large file — keep scenarios, dedupe with UI tests |
| `test_planning_ui_constructors.py` | flat planning UI constructors | `test_planning.py` | |
| `test_rrt.py` | RRT/RRT\*, tree, steering, holonomic scene | `test_planning.py` | table-driven already; keep one owner |
| `test_spatial.py` | collision bodies, fields, shaping, grid sampling | `test_planning.py` | |
| `test_reference_paths.py` | reference paths, corridor fields | `test_planning.py` | |
| `test_jax_direct_collocation.py` | JAX direct collocation smoke | `test_jax_planning.py` | optional |
| `test_jax_kinematic_bicycle.py` | JAX kinematic bicycle twin | `test_jax_planning.py` | optional |
| `test_model_predictive_controller.py` | `ModelPredictiveController` unit API | `test_mpc.py` | |
| `test_mpc_algebraic_controller.py` | stateless MPC block | `test_mpc.py` | |
| `test_mpc_warm_start_controller.py` | warm-start stateful controller | `test_mpc.py` | |
| `test_mpc_planner.py` | parametric `TrajectoryOptimizationPlanner` | `test_mpc.py` | |
| `test_mpc_solve_trajectory_from.py` | `solve_trajectory_from` | `test_mpc.py` | |
| `test_mpc_export_computer.py` | `export_to_computer` | `test_mpc.py` | |
| `test_mpc_numpy_rebuild.py` | NumPy rebuild-mode MPC | `test_mpc.py` | |
| `test_mpc_hybrid_straight_line.py` | stateless hybrid straight-line parity | `test_mpc.py` | parametrized `scenario=` |
| `test_mpc_hybrid_warm_start_parity.py` | warm-start hybrid parity | `test_mpc.py` | same |
| `test_mpc_hybrid_demo_parity.py` | demo-aligned hybrid parity | `test_mpc.py` | extract shared scenario builder with demo |

### Pytest — graphics, geometry, engine, symbolic, benchmarks

| Current file | Validates | New file | Notes |
| --- | --- | --- | --- |
| `test_geometry.py` | `Sphere`, core geometry | `test_geometry.py` | keep |
| `test_camera_transform.py` | camera transform contract | `test_graphics.py` | |
| `test_world_frame_contract.py` | implicit `world` frame injection | `test_graphics.py` | |
| `test_kinematic_regression.py` | kinematic render manifest smoke | `test_graphics.py` | |
| `test_overlays.py` | overlay drawables, animate overlays | `test_graphics.py` | |
| `test_advanced_plotting.py` | advanced plotting helpers | `test_graphics.py` | |
| `test_plotly_renderer.py` | plotly renderer | `test_graphics.py` | `@pytest.mark.plotting` |
| `test_visualization_optional.py` | pygame/meshcat optional smoke | `test_graphics.py` | |
| `test_engine_world_system.py` | physics `DynamicSystem` wrapper | `test_engine_jax.py` | |
| `test_contact_engine_jax.py` | JAX contact engine core | `test_engine_jax.py` | |
| `test_ancf_tire_jax.py` | ANCF tire prototype | `test_engine_jax.py` | |
| `test_symbolic.py` | SymPy mechanics | `test_symbolic.py` | keep |
| `test_benchmark_smoke.py` | benchmark helper import smoke + baseline compare units | `test_benchmark_smoke.py` | trim suite runners duplicated by Layer B |

### Test fixtures & helpers (not merged into test modules)

| Current path | Validates / role | After consolidation | Notes |
| --- | --- | --- | --- |
| `tests/unittest/conftest.py` | shared pytest fixtures | unchanged | |
| `tests/unittest/planning_helpers.py` | minimal RRT holonomic scene | unchanged | distinct from benchmark dense scene |
| `tests/unittest/graphics_contract_helpers.py` | draw-list resolution helpers | unchanged | |
| `tests/fixtures/kinematic_baseline/manifest.json` | golden kinematic render keys | unchanged | |
| `tests/fixtures/kinematic_baseline/regenerate_manifest.py` | regen manifest after plant set changes | unchanged | dev tool |
| `tests/fixtures/kinematic_baseline/render.py` | render helper for manifest | unchanged | |
| `tests/fixtures/benchmark_baseline/minimal_core_perf.json` | tiny baseline for unit compare tests | unchanged | |

### Benchmark runners (tier-1 regression)

| Current runner | Validates | New entry | Notes |
| --- | --- | --- | --- |
| `run_regression_check.py` | `core_perf` + `integration_check` vs JSON | `run_regression_check.py` | extend `--suite` |
| `run_e4_trajopt_parity.py` | E4 trajopt/MPC parametric parity | `run_regression_check.py --suite e4` | delete thin wrapper after merge |
| `run_f_mpc_parity.py` | F MPC hybrid/hand-loop/dual-rate parity | `run_regression_check.py --suite f_mpc` | factor 2× stays |

### Benchmark runners (tier-2 manual studies)

| Current runner | Validates | New entry | Notes |
| --- | --- | --- | --- |
| `run_pendulum_f_speed.py` | pendulum `f()` native/numpy/jax timing table | `run_study.py --preset f_eval --plant pendulum` | |
| `run_diagram_f_speed.py` | dense diagram `f()` timing | `run_study.py --preset f_eval --plant diagram_dense` | |
| `run_step_speed.py` | leaf `step()` timing | `run_study.py --preset step_eval --plant leaf` | |
| `run_step_diagram_speed.py` | step diagram `step()` timing | `run_study.py --preset step_eval --plant step_diagram` | |
| `run_simulator_standard.py` | three standard sim cases, one variant | `run_study.py --preset sim --mode standard` | |
| `run_simulator_speed_matrix.py` | solver×backend matrix vs truth | `run_study.py --preset sim --mode matrix` | |
| `run_simulator_speed_manual.py` | ad-hoc sim matrix | `run_study.py --preset sim --mode manual` | config file or inline preset |
| `run_optimizer_backends.py` | NLP solver presets on textbook problems | `run_study.py --preset optimizer` | |
| `run_trajopt_backends.py` | trajopt transcription×backend sweep | `run_study.py --preset trajopt --mode backends` | |
| `run_trajopt_solver_presets.py` | trajopt solver preset sweep | `run_study.py --preset trajopt --mode solvers` | |
| `run_dp_backends.py` | DP loop/numpy/jax backends | `run_study.py --preset dp` | |
| `run_rrt_nearest_backends.py` | RRT nearest brute_force vs kd_tree | `run_study.py --preset rrt_nearest` | |
| `run_pyro_minilink_parity.py` | Pyro vs Minilink DP/RRT parity | `run_study.py --preset pyro_parity` | keep env-var story |

### Benchmark library modules (keep, not counted as scripts)

These are imported by suites and smoke tests — **no deletion**, only stop exposing each as its own top-level CLI:

`baseline.py`, `common.py`, `f_evaluators.py`, `step_evaluators.py`, `simulation.py`, `trajopt.py`, `optimization.py`, `dynamic_programming.py`, `planning_rrt.py`, `pyro_parity.py`, `pyro_parity_worker.py`, `suites/*`, `scenarios/*`, `systems/*`.

---

## Overlap hotspots (priority merges)

1. **MPC (11 files → 1)** — Three hybrid parity files share straight-line bicycle setup; merge into parametrized scenarios with shared fixtures; keep distinct assertions (stateless vs warm-start vs demo-aligned).
2. **Hybrid (7 files → 1)** — All test `HybridSimulator` subgraph; single module with nested `Test*` classes preserves readability.
3. **Step (7 files → 1)** — Rollout parity appears in both `test_step_rollout` and `test_step_diagram_rollout`; one table of `(system, backend)` cases.
4. **Compile (6 files → 1)** — `test_compile_pipeline.py` already has 46 tests; absorb leaf/static/evaluator files without splitting further.
5. **JAX optional sprawl** — `@pytest.mark.jax` sections inside domain files (`test_mechanical_robotics`, `test_step_discrete`, `test_engine_jax`, `test_jax_planning`) instead of one-file-per-plant JAX smoke.
6. **Benchmark CLI duplication** — Four nearly identical `run_*_f_speed` / `run_*_step_speed` scripts differ only in fixture; one `run_study.py` with presets.
7. **Parity duplication across layers** — MPC hybrid parity in pytest (Layer A) vs `f_mpc_parity` JSON (Layer B): keep both but **share scenario code** in `benchmarks/scenarios/f_mpc_parity.py` imported by `test_mpc.py` for fast smoke subset.

---

## Proposed migration phases

### Phase 0 — Inventory lock (this doc)

- [x] Review mapping table for missing independent checks.
- [x] Layer B includes NLP/trajopt solve-speed gates (`solve_speed` suite + CI job).

---

## Decisions (applied)

1. **CI scope:** GitHub CI runs `run_regression_check.py --suite all --tiny` with
   `minilink[jax]`, factor **6×**, and `--speed-gate-suffixes solve_s,nlp_s,speedup`.
   Full `--suite all` (per-baseline factors, all speed metrics) stays the local
   pre-handoff gate.

2. **Pyro parity:** **Manual Layer C only** — not gated in CI (external Pyro env not
   reproducible on ubuntu runners). Keep `run_pyro_minilink_parity.py` as a dev study.

3. **`test_planning_architecture.py`:** **Keep monolithic** during consolidation — nested
   `Test*` classes inside one module; extract shared scenario builders to
   `benchmarks/scenarios/` when tests and Layer-B parity overlap (Phase 1), not
   separate data-table files yet.

4. **Layer B vs C (solve speed):** New **`solve_speed`** suite gates standalone
   `Optimizer.solve_s` and NumPy pendulum trajopt `solve_s`. Existing integration /
   e4 / f_mpc baselines gate JAX trajopt **`solve_s`** and MPC **`nlp_s`**. Tier-2
   backend sweeps (`run_trajopt_backends.py`, etc.) remain Layer C.

5. **Examples overlap:** **Tests + Layer-B scenarios are authoritative** for MPC hybrid
   parity; demos may diverge for pedagogy — shared builders live in
   `benchmarks/scenarios/f_mpc_parity.py` (Phase 1 extraction).

---

## Catalog & demo smokes (Phase 5)

Today catalog coverage is spread across four pytest modules doing overlapping work:

| Current test file | What it does | ~cost | Replace with |
| --- | --- | --- | --- |
| `test_catalog_migration.py` | Reference values for integrators/MSD; arrow/primitive counts for ~15 plants; **`test_catalog_class_smoke`** loops **40+** systems (`f` finite + `geometry_smoke`) | heavy import fan-out | Per-module headless smokes + one aggregator |
| `test_catalog_plant_contracts.py` | Deep `H/C/g/B` reference matrices + compile + graphics for CartPole, DoublePendulum, DynamicBicycle | keep (contract) | **`test_dynamics_catalog.py`** — 3 plants only |
| `test_manipulators.py` | Manipulator subclass ports, FK spot checks, camera scale | partial overlap | Fold FK/port checks into **`catalog/manipulators/smoke.py`** + 1 parametrized contract test |
| `test_kinematic_regression.py` | Manifest-driven Agg PNG render (33 frames, 11 plants) | medium | Same registry as catalog smokes; optional pixel tier for flagships only |

**Principle:** catalog *breadth* (every class instantiates, `f` is finite, frame resolves) belongs in **runnable headless scripts** the user can also run locally; pytest keeps **equation reference values** and **API contracts** that are expensive to recover if they regress silently.

### Recommended layout (not one `__main__` per class)

Putting a 10-line `__main__` in every catalog class file violates the repo’s “hello-world ≤10 lines in core modules” spirit and duplicates JAX-skip logic. Prefer:

```
minilink/dynamics/catalog/
  smoke_registry.json          # single source of truth (or Python registry module)
  pendulum/smoke.py            # one headless runner per catalog *module file*
  vehicles/smoke.py
  manipulators/smoke.py
  ...
examples/scripts/_smoke/
  run_catalog_smokes.py        # runs all catalog smokes, structured exit code
  run_flagship_demos.py          # whitelist of flagship demos in --smoke mode (see below)
```

Each **`smoke.py`** (colocated with the catalog module it covers):

1. Instantiates every **public plant class** exported from that file.
2. Runs **`f(x,u)` finiteness** (and `step` if `StepSystem`).
3. Optionally **3-step Euler** at default `x0` (headless `Simulator`, `tf=0.03`, `MPLBACKEND=Agg`).
4. Calls shared **`geometry_smoke`** (draw-list finite transforms — no PNG).
5. Prints one line per class; exits non-zero on failure.
6. Skips JAX classes when `jax` missing (same policy as today).

Example contract (sketch):

```python
# minilink/dynamics/catalog/pendulum/smoke.py
"""Headless catalog smoke for pendulum.py classes. Run: python -m ..."""
def main() -> int: ...
if __name__ == "__main__":
    raise SystemExit(main())
```

### Aggregator: one helper script

**`examples/scripts/_smoke/run_catalog_smokes.py`** (or `python -m minilink.dynamics.catalog.run_smokes`):

- Discovers smokes via registry (module path → smoke entrypoint).
- Runs each subprocess with timeout; summarizes pass/fail table.
- Flags: `--plant pendulum`, `--fast` (skip short sim, only `f` + geometry), `--json`.

**Pytest surface after consolidation:** one test in `test_smoke_scripts.py`:

```python
def test_catalog_smokes_exit_zero():
    subprocess.run([sys.executable, "examples/scripts/_smoke/run_catalog_smokes.py", "--fast"], check=True)
```

Optional nightly job runs full smokes (with 3-step sim). Default CI uses `--fast`.

### What to delete from pytest

| Remove / shrink | Retain |
| --- | --- |
| `test_catalog_class_smoke` loop (40+ plants) | Deep matrix tests in `test_catalog_plant_contracts` (3 plants) |
| Arrow-count table in migration (move spot checks to flagship graphics or 2 plants) | Low-risk **equation literals** for integrators, MSD, 1–2 vehicles (public math) |
| Duplicate manipulator FK in migration | `test_manipulators` → 1 parametrized port/FK table or merge into catalog contracts |
| Standalone `test_kinematic_regression` loop | Render step invoked from smoke registry **or** flagship manifest (below) |

### Demo scripts: whitelist smoke, not all 78

Running all 78 demos headless in CI is brittle (user-tuned plots, JAX, meshcat, long MPC). Plan:

1. **Do not** auto-discover every file under `examples/scripts/`.
2. Maintain a **whitelist manifest** (`examples/scripts/_smoke/flagship_manifest.json`) of ~8–12 **README-tier** demos.
3. Each whitelisted demo gains a **`run_smoke()`** function (new code at bottom — **does not change default `__main__` behavior** when user runs the demo normally):
   - Sets `MPLBACKEND=Agg`, `show=False`, short `tf`, minimal `maxiter` where applicable.
   - Runs the same call chain as the demo but skips `animate(show=True)` unless smoke tier requires one frame.
   - Preserves user’s commented plots, tuning constants, and disabled sections untouched.

**`run_flagship_demos.py`** iterates the manifest, imports `run_smoke` from each module (dynamic import), collects structured results.

| Tier | Runner | CI |
| --- | --- | --- |
| Fast | `run_catalog_smokes.py --fast` + flagship `run_smoke()` without PNG | every PR |
| Full | catalog smokes + 3-step sim + flagship smokes | nightly or pre-release |
| Manual | user runs full demos with animation | unchanged |

---

## Flagship graphics validation (Phase 6)

Goal: a **simple visual check** on core demos that exercises the main **graphics output pipelines** without maintaining PNGs for every catalog pose.

### Pipelines to cover

| Pipeline | Exercised by | Current test |
| --- | --- | --- |
| Kinematic frame → Matplotlib Agg PNG | catalog plants, `Animator._resolve_frame` | `test_kinematic_regression` |
| Diagram `plot_trajectory` / `plot_diagram` | most demos | none (implicit) |
| `animate()` Matplotlib native | `demo_readme`, computed torque, bicycle cascade | none |
| Overlays (`mpc_animation_overlays`, etc.) | MPC demos | `test_overlays` (unit) |
| Plotly renderer | trajopt live plot demos | `test_plotly_renderer` (optional) |
| Meshcat / pygame | engine, interactive demos | optional markers only |

### Three-tier graphics validation

#### Tier G0 — Draw-list contract (pytest, fast) — **keep / extend Layer A**

For each **flagship demo**, at a **canonical terminal state** (fixed seed, short sim):

- Assert `resolve_draw_frame` / diagram boundary frame: finite transforms, primitive count > 0 where expected.
- Assert stable **frame keys** (primitive types, overlay ids) — no pixel I/O.

One file: **`test_flagship_graphics_contract.py`** (~8 parametrized cases driven by manifest).

#### Tier G1 — Single-frame render smoke (pytest or Layer B-lite)

Extend kinematic fixture pattern to **flagship end-states**:

| Flagship | Canonical capture | Renderer |
| --- | --- | --- |
| `demo_readme.py` (impedance @ pendulum) | `t=tf`, diagram plant state | Matplotlib Agg |
| `demo_computed_torque_pendulum.py` | post-sim pendulum pose | Matplotlib Agg |
| `demo_dynamic_bicycle_cascade_path_tracking.py` | mid-trajectory bicycle | Matplotlib Agg |
| `demo_mpc_minimal.py` | terminal plant + MPC overlay frame | Matplotlib Agg + overlay keys |
| `demo_diagram_compiling.py` or `demo_closed_loop.py` | diagram topology / signals plot | plot_diagram smoke (no PNG) |
| `demo_cartpole_direct_collocation_jax_ipopt.py` | traj plot only | plot_trajectory finite axes |
| `demo_holonomic_obstacles.py` | planning scene draw | scene bounds + path polyline |
| `demo_animation/demo_native_comparison.py` | native vs non-native same frame | Matplotlib Agg (2 frames) |

**Manifest:** `tests/fixtures/flagship_graphics/manifest.json`

Each entry:

```json
{
  "id": "readme_impedance_pendulum",
  "demo": "examples/scripts/plots/demo_readme.py",
  "smoke_fn": "run_smoke",
  "capture": "matplotlib_frame",
  "state": "from_smoke",
  "hash": "sha256:…",
  "tolerance": "perceptual"
}
```

**Gating options** (pick one in implementation):

| Approach | Pros | Cons |
| --- | --- | --- |
| **A. No committed pixels** (today’s kinematic test) | zero flake, fast | only catches crashes, not visual drift |
| **B. Downsampled perceptual hash** (pHash / average hash) | catches major layout breaks | some CI flake; needs `--update-manifest` |
| **C. Committed tiny PNG** (64×64) in `tests/fixtures/flagship_graphics/` | precise | repo size; anti-aliasing flake |

**Recommendation:** start with **A for CI** (finite image + non-blank pixel variance threshold, same as kinematic test today); add **B for 3–5 hero frames** locally / nightly once stable.

Regenerator: `python tests/fixtures/flagship_graphics/regenerate_manifest.py` (mirrors kinematic workflow).

#### Tier G2 — Manual / release visual review

- Full `animate()` with user timing; meshcat optional.
- Not gated in CI.

### Consolidation with catalog smokes

Use **one plant registry** feeding:

1. `run_catalog_smokes.py` (f + geometry + optional sim)
2. Kinematic manifest entries (static poses — can shrink to 1 pose per plant)
3. Flagship demos that **use** catalog plants (don't duplicate plant list)

Remove duplicate plant lists in `regenerate_manifest.py`, `test_catalog_migration`, and kinematic JSON — generate from `smoke_registry`.

### Target file map (catalog + graphics)

| Current | Validates | New owner |
| --- | --- | --- |
| `test_catalog_migration.py` (bulk) | 40-class smoke | `catalog/*/smoke.py` + `run_catalog_smokes.py` |
| `test_catalog_migration.py` (refs) | equation literals | `test_dynamics_catalog.py` (small) |
| `test_catalog_plant_contracts.py` | deep 3-plant | `test_dynamics_catalog.py` |
| `test_manipulators.py` | manipulator ports/FK | `catalog/manipulators/smoke.py` + 1 contract test |
| `test_kinematic_regression.py` | PNG finite | `run_catalog_smokes --render` or flagship G1 |
| `test_overlays.py` | overlay API | keep (unit) |
| `test_plotly_renderer.py` | plotly | keep `@pytest.mark.plotting` |
| `test_camera_transform.py`, `test_world_frame_contract.py` | frame math | `test_graphics.py` |
| (none) | flagship demo pipelines | `test_flagship_graphics_contract.py` + G1 manifest |
| (none) | demo headless execution | `run_flagship_demos.py` + 1 pytest subprocess |

### Phase 5–6 migration steps

1. Add `smoke_registry` + `pendulum/smoke.py` pilot; wire `run_catalog_smokes.py`.
2. Port remaining catalog modules; delete `test_catalog_class_smoke` once aggregator passes.
3. Add `run_smoke()` to first 3 flagship demos; `run_flagship_demos.py` + G0 contract tests.
4. Add `flagship_graphics/manifest.json` + regenerate script; G1 nightly job optional.
5. Merge kinematic manifest generation into registry; drop duplicate plant tables.

### Open questions (catalog / graphics)

1. **Registry format:** JSON (easy for tooling) vs Python module (typed, importable) — preference?
2. **Pixel gate:** acceptable to commit 5 small PNGs for hero demos, or hash-only?
3. **Demo smoke ownership:** ok to require each new README-tier demo to add `run_smoke()` in the same PR?

---

## Open questions

_(catalog/graphics questions above; prior decisions in §Decisions applied)_

### Phase 1 — Shared scenario extraction (low risk)

- Extract MPC hybrid and trajopt scenario builders used by tests **and** benchmarks.
- No test deletion yet; imports switch to shared modules.

### Phase 2 — Domain merges (medium risk)

- Merge file groups in table order: hybrid → step → mpc → compile → planning.
- Run full `pytest` after each merge; keep git moves as `git mv` + import fixes for blame.

### Phase 3 — Benchmark CLI unification

- Implement `run_study.py`; deprecate individual `run_*_speed.py` with one-release shim forwarding + stderr warning.
- Extend `run_regression_check.py --suite all` to include `e4` and `f_mpc`.

### Phase 4 — Docs & CI alignment

- [x] Update `tests/README.md` with Layer B CI regression note.
- [x] Update `benchmarks/README.md` with unified suites + CI flags.
- [x] Update `AGENTS.md` verification table (CI regression job).
- [x] Optional: `run_study.py` CLI unification (Phase 3).
- [x] L6 flagship manifest expanded (11 demos).
- [x] L4 `test_flagship_graphics_contract.py` + flagship graphics manifest.
- [x] L5 `run_graphics_visual_check.py` checklist.
- [x] `run_all_demos.py` for optional nightly sweep.

---

## Success metrics

| Metric | Was | Now |
| --- | ---: | ---: |
| Pytest files | 91 | **22** |
| Catalog plant coverage | 4 pytest files | `test_dynamics_catalog.py` + L6 smokes |
| Flagship demo visual check | kinematic manifest only | G0 draw-list + L4 PNG smoke + L5 checklist |
| Benchmark `run_*.py` | 15 | **2** primary (`run_regression_check`, `run_study`) + shims |
| Documented test layers | implicit | **L1–L6** in `tests/README.md` |

---

## References

- [tests/README.md](../../tests/README.md) — marker policy, philosophy
- [benchmarks/README.md](../../benchmarks/README.md) — suite layout, baseline semantics
- [AGENTS.md](../../AGENTS.md) — verification gates, regression check command
