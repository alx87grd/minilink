# Minilink Manual Review Checklist

Ordered **contracts → core → libraries → tools → demos → verification**.
Each bullet is sized for roughly **one 10-minute pass**.

Use [DESIGN.md](DESIGN.md), [ROADMAP.md](ROADMAP.md), and [agent.md](agent.md) as
the rubric throughout. Mark TRL from the roadmap table as you go.

**Rough total:** ~55 sessions ≈ **9 hours** of focused review.

---

## How to use this

- **One bullet = one ~10 min session.** Stop when the timer rings; pick up on the
  next bullet.
- **Review lens** (repeat mentally): Does the math read like a textbook? Are
  contracts in DESIGN honored? Any stale/duplicate paths? Does import direction
  follow the dependency law?
- **Optional run** lines are smoke checks, not full test suites.
- **Skip notebook cell outputs** unless you are validating a specific demo (per
  `agent.md`).

### Per-session capture template

```text
[ ] #17 ExecutionPlan — OK / issue: … / TRL bump?: …
```

---

## Phase 0 — Contracts and orientation (4 × 10 min)

- [ ] **1. North star** — Read [README.md](README.md) Quick start + Features +
  Call chains. Check: does the advertised workflow (`controller @ plant` →
  simulate → plot → animate) match what you expect to see in code?
- [ ] **2. Architecture contracts** — Read [DESIGN.md](DESIGN.md) §1–3
  (principles, layers, package map, dependency law). Note the four bands:
  framework / libraries / tools / quarantine.
- [ ] **3. Object contracts** — Read [DESIGN.md](DESIGN.md) §4–6 (`System`,
  params nesting, `DiagramSystem`, compile/sim, NLP/planning). Flag anything
  that feels underspecified for your sign-off.
- [ ] **4. Maturity map** — Read [ROADMAP.md](ROADMAP.md) §1 + §4 (Review
  queue). Decide which areas need deep review vs. light pass based on TRL.

---

## Phase 1 — Core framework (12 × 10 min)

- [ ] **5. `System` contract** — `minilink/core/system.py` (first screen:
  `DynamicSystem`, `f`/`h`, ports, dims `n`/`m`/`p`). Lens: bare equation
  signatures, `params` unpacked to locals, no `self.` in final equation lines.
- [ ] **6. `System` visualization hooks** — Same file, kinematic/camera methods.
  Lens: are these the right place, or should anything move to `graphical/`?
  (ROADMAP review queue item.)
- [ ] **7. Signals and ports** — `minilink/core/signals.py`. Lens: port IDs,
  dims, metadata; student-facing file stays above validation plumbing.
- [ ] **8. `DiagramSystem` wiring** — `minilink/core/diagram.py`
  (`add_subsystem`, `connect`, flattening, nested params live-view). Optional:
  `pytest tests/unittest/test_diagrams.py -q`.
- [ ] **9. Composition shortcuts** — `minilink/core/composition.py` (`+`, `>>`,
  `@`, `autowire`). Lens: shortcuts produce ordinary `DiagramSystem`; no hidden
  nesting.
- [ ] **10. Backends policy** — `minilink/core/backends.py`. Lens: lazy JAX,
  `array_module`, no global mode switch.
- [ ] **11. `Trajectory`** — `minilink/core/trajectory.py`. Lens: `t`, `x`, `u`,
  `signals` shapes; NumPy reporting object only.
- [ ] **12. Sets** — `minilink/core/sets.py`. Lens: `margin ≥ 0` feasible;
  boundary conversion only at API edge.
- [ ] **13. Costs** — `minilink/core/costs.py`. Lens: `g`/`h` on planning side,
  not on plants.
- [ ] **14. Facades** — `minilink/core/facades.py`. Lens: thin user shortcuts;
  `self.traj` is cache only, not library input.
- [ ] **15. Core tests** — Skim `tests/unittest/test_core.py`,
  `test_composition.py`, `test_backends.py`. Lens: do tests encode contracts you
  care about?
- [ ] **16. Legacy duplicate: `core/blocks/`** — Compare
  `minilink/core/blocks/` vs top-level `minilink/blocks/`. Lens: is anything
  still imported from the old path? Should it be deleted?

---

## Phase 2 — Compile pipeline (6 × 10 min) — TRL 4, needs sign-off

- [ ] **17. `ExecutionPlan`** — `minilink/core/compile/execution_plan.py`. Lens:
  `output_slices` vs `external_output_slices`; boundary vs internal outputs.
- [ ] **18. Compiler orchestrator** — `minilink/core/compile/compiler.py`.
  Lens: `compile(system, backend)` → evaluator; no extra `compilers/` layer.
- [ ] **19. Evaluator base API** —
  `minilink/core/compile/evaluators/evaluator.py`. Lens: `f`/`f_p`,
  `outputs`/`outputs_p`, parametric tier — ROADMAP P1 review item.
- [ ] **20. NumPy evaluator** —
  `minilink/core/compile/evaluators/numpy_evaluator.py`. Lens: flat execution vs
  reference path parity.
- [ ] **21. JAX evaluator** —
  `minilink/core/compile/evaluators/jax_evaluator.py`. Lens: traceability,
  `jacobian_f_params`, nested params as pytree.
- [ ] **22. Compile tests + legacy `minilink/compile/`** —
  `tests/unittest/test_compile_pipeline.py` + skim duplicate tree under
  `minilink/compile/`. Lens: which path is canonical? Any stale imports still
  pointing at legacy?

---

## Phase 3 — Blocks (wiring library) (5 × 10 min) — TRL ~6

- [ ] **23. Sources** — `minilink/blocks/sources.py` (`Step`, noise,
  `TrajectorySource`). Lens: time-varying sources belong in diagram, not forcing
  hacks.
- [ ] **24. Basic + integrator** — `minilink/blocks/basic.py`. Lens: overlap
  with `dynamics/catalog/equations/integrators.py` is intentional — roles
  differ.
- [ ] **25. Routing** — `minilink/blocks/routing.py` (`Sum`, `Gain`, `Mux`,
  `Demux`). Lens: plant-agnostic wiring only.
- [ ] **26. Nonlinear + filters + TF** — `minilink/blocks/nonlinear.py`,
  `filters.py`, `transfer_function.py`.
- [ ] **27. Block tests** — `tests/unittest/test_blocks.py`,
  `test_signal_blocks.py`. Optional:
  `python examples/scripts/blocks/demo_signal_blocks.py`.

---

## Phase 4 — Dynamics abstraction (4 × 10 min)

- [ ] **28. State-space base** — `minilink/dynamics/abstraction/state_space.py`
  (`A(t,params)`…`D`, `LTISystem`). Lens: matrix methods mirror textbook
  `dx = A @ x + B @ u`.
- [ ] **29. Mechanical base** — `minilink/dynamics/abstraction/mechanical.py`
  (`H`, `C`, `g`). Lens: Lagrangian structure readable; JAX twin pattern if
  present.
- [ ] **30. Generalized mechanical** —
  `minilink/dynamics/abstraction/generalized_mechanical.py`.
- [ ] **31. Abstraction tests** — `test_state_space_system.py`,
  `test_mechanical.py`, `test_mechanical_jax.py`,
  `test_generalized_mechanical.py`, `test_jaxmechanical_inheritance.py`.

---

## Phase 5 — Dynamics catalog (10 × 10 min) — TRL 6

- [ ] **32. Pendulum family** — `dynamics/catalog/pendulum/pendulum.py`,
  `cartpole.py`, `double_pendulum.py` + `test_pendulum_plants.py`,
  `test_jax_pendulum_subclass.py`.
- [ ] **33. Mass-spring-damper** —
  `dynamics/catalog/mass_spring_damper/linear.py`.
- [ ] **34. Textbook ODEs** — `dynamics/catalog/equations/integrators.py`,
  `oscillators.py` (teaching demos, graphics/labels).
- [ ] **35. Vehicles — bicycle** —
  `dynamics/catalog/vehicles/dynamic_bicycle.py` +
  `test_dynamic_bicycle_graphics.py`. ROADMAP: module split review.
- [ ] **36. Vehicles — rest** — `mountain_car.py`, `propulsion.py`,
  `steering.py`, `suspension.py`.
- [ ] **37. Aerial** — `dynamics/catalog/aerial/drone.py`, `plane.py`,
  `rocket.py`.
- [ ] **38. Marine + manipulators** — `dynamics/catalog/marine/boat.py`,
  `manipulators/arms.py`.
- [ ] **39. Catalog migration notes** — `docs/plans/catalog-migration-notes.md`
  + `test_catalog_migration.py`. Lens: pyro parity claims still credible?
- [ ] **40. Catalog equations doc** — Skim `docs/catalog_eom.tex` if you want
  math cross-check (optional second session if dense).

---

## Phase 6 — Contact / physics engines (3 × 10 min) — TRL 1

- [ ] **41. World system** — `minilink/dynamics/engines/world.py` +
  `test_engine_world_system.py`.
- [ ] **42. Contact JAX** — `minilink/dynamics/engines/contact_jax.py` +
  `test_contact_engine_jax.py`. Lens: energy/analytic validation still missing
  per ROADMAP.
- [ ] **43. ANCF tire + legacy `physics/`** — `ancf_tire_jax.py`,
  `test_ancf_tire_jax.py`, compare `minilink/physics/` vs `dynamics/engines/`.
  Lens: quarantine rules — does anything import `physics/`?

---

## Phase 7 — Control (2 × 10 min) — TRL 5

- [ ] **44. Linear controllers** — `minilink/control/linear.py` (P/PD/PID/MIMO).
  Lens: control naming `r`/`y`/`u`; factories return blocks.
- [ ] **45. LQR** — `minilink/control/lqr.py` + `tests/unittest/test_control_linear.py`,
  `test_analysis_control.py`. Optional:
  `python examples/scripts/analysis/demo_linearize_lqr.py`.

---

## Phase 8 — Simulation tool (3 × 10 min) — TRL 7

- [ ] **46. Simulator** — `minilink/simulation/simulator.py`,
  `input_interpolation.py`.
- [ ] **47. Solvers** — `simulation/solvers/solver.py`, `scipy_ivp.py`,
  `euler.py`, `rk4_fixed.py`. Lens: preset names match DESIGN §5.
- [ ] **48. Simulation tests** — `test_simulator.py`,
  `test_simulation_speed_benchmarks.py`. Optional:
  `python benchmarks/run_simulator_standard.py`.

---

## Phase 9 — Analysis tool (2 × 10 min) — TRL 4

- [ ] **49. Linearize + structural** — `analysis/linearize.py`,
  `structural.py` (→ `LTISystem`, ctrb/obsv).
- [ ] **50. Equilibria** — `analysis/equilibria.py` + `test_analysis_control.py`.
  Lens: finite-diff Jacobians — acceptable for now?

---

## Phase 10 — Planning / trajopt (4 × 10 min) — TRL 2, prototype-heavy

- [ ] **51. Planning problems** — `planning/problems.py`, `planner.py`,
  `initial_guess.py`.
- [ ] **52. Transcription core** —
  `planning/trajectory_optimization/transcription.py`, `direct_collocation.py`,
  `multiple_shooting.py`, `shooting.py`.
- [ ] **53. Trajopt planner + live plot** — `trajectory_optimization/planner.py`,
  `live_plot.py` + `test_planning_architecture.py`,
  `test_jax_direct_collocation.py`.
- [ ] **54. Trajopt consolidation review** — Re-read ROADMAP §4 “Trajopt
  transcription internal consolidation”; note what you would freeze vs. rewrite.

---

## Phase 11 — Optimization / NLP (4 × 10 min) — TRL 5

- [ ] **55. `MathematicalProgram`** — `optimization/mathematical_program.py`.
  Lens: pure `J`/`h`/`g`; native backend field.
- [ ] **56. `Optimizer` + backends** — `optimizer.py`,
  `optimizers/scipy_minimize.py`, `ipopt.py`, `optimizer_backend.py`.
- [ ] **57. Program evaluators** — `optimization/evaluators/` (compiler, numpy,
  jax, `program_evaluator.py`).
- [ ] **58. Optimization tests** — `test_mathematical_program_evaluators.py`,
  `test_ipopt_optimizer.py`, `test_optimizer_solve_time.py`. Optional:
  `python examples/scripts/optimization/demo_simple_optimization.py`.

---

## Phase 12 — Graphical tool (6 × 10 min) — TRL 3

- [ ] **59. Time-series plots** — `graphical/signals/time_signals.py`,
  matplotlib/plotly backends.
- [ ] **60. Phase plane** — `graphical/phase_plane/phase_plane.py` +
  `test_phase_plane.py`. Optional:
  `python examples/scripts/plots/demo_phase_plane.py`.
- [ ] **61. Diagram rendering** — `graphical/diagrams/topology.py`, `dot.py`,
  `mermaid.py`, `export.py`.
- [ ] **62. Animation core** — `graphical/animation/animator.py`,
  `primitives.py`, `timing.py`.
- [ ] **63. Renderers** — `renderers/renderer.py`, matplotlib, plotly, meshcat,
  pygame + `test_plotly_renderer.py`, `test_camera_transform.py`. ROADMAP:
  camera contract consolidation.
- [ ] **64. Plot demos** — `examples/scripts/plots/demo_readme.py`,
  `demo_internal_signals.py`, `demo_plot_trajectory_options.py`.

---

## Phase 13 — Symbolic quarantine (2 × 10 min) — TRL 1

- [ ] **65. Symbolic mechanics** — `symbolic/mechanics/model.py`, `derivation.py`,
  `export.py`, `symbolic_system.py` + `test_symbolic.py`. Lens: nothing outside
  quarantine should import this.
- [ ] **66. Symbolic demo** —
  `python examples/scripts/symbolic/demo_symbolic_quadruple_pendulum.py` (needs
  `.[symbolic]`).

---

## Phase 14 — Example scripts: diagrams & compile (6 × 10 min)

- [ ] **67. Shortcuts vs explicit wiring** —
  `examples/scripts/diagrams/demo_diagram_shortcuts.py`.
- [ ] **68. Closed loop + nested loops** — `demo_closed_loop.py`,
  `demo_nested_loop_diagram.py`.
- [ ] **69. Sources and noise ports** — `demo_sources.py`, `demo_noise_ports.py`.
- [ ] **70. Compile + params gradient** — `demo_diagram_compiling.py`,
  `demo_params_gradient.py` (parametric tier sign-off).
- [ ] **71. Autowire + advanced wiring** — `demo_advanced_autowire.py`.
- [ ] **72. Vehicle diagram cascades** —
  `demo_dynamic_bicycle_basic_cascade_path_tracking.py`,
  `demo_dynamic_bicycle_cascade_path_tracking.py`.

---

## Phase 15 — Example scripts: animation, engine, trajopt (6 × 10 min)

- [ ] **73. Animation sweep** — `examples/scripts/animation/demo_animations.py`
  (baseline).
- [ ] **74. Interactive + camera** — `demo_interactive.py`,
  `demo_camera_options.py`.
- [ ] **75. Bicycle animations** — `demo_dynamic_bicycle.py`,
  `demo_dynamic_bicycle_realistic.py`.
- [ ] **76. Physics engine demos** —
  `examples/scripts/engine/demo_physics_in_diagram.py`,
  `demo_physics_many_spheres.py`, `demo_ancf_tire_fall.py`.
- [ ] **77. Cartpole trajopt** —
  `trajectory_optimization/demo_cartpole_direct_collocation_jax_ipopt.py`,
  `demo_cartpole_direct_collocation_live_plot.py`.
- [ ] **78. Bicycle trajopt** — `demo_dynamic_bicycle_trajopt_lanechange.py`,
  `demo_dynamic_bicycle_trajopt_uturn.py`,
  `demo_dynamic_bicycle_rate_trajopt.py`.

---

## Phase 16 — Notebooks (3 × 10 min) — skim structure, not outputs

- [ ] **79. `demo_showcase.ipynb`** — Entry point linked from README; check
  imports and narrative flow.
- [ ] **80. `demo_overview.ipynb` + `demo_animations.ipynb`** — Onboarding and
  graphics story.
- [ ] **81. `demo_optimization.ipynb` + `simulation_benchmark.ipynb` +
  `demo_colab.ipynb`** — Advanced workflows; note Colab-specific deps.

---

## Phase 17 — Tests, benchmarks, docs, packaging (5 × 10 min)

- [ ] **82. Unittest map** — Skim `tests/README.md` if present, else
  `tests/unittest/` file list grouped by area; spot-check any area you flagged
  weak during code review.
- [ ] **83. Manual / bug repros** — `tests/manual/`, `tests/bugs/`. Lens:
  promote or delete?
- [ ] **84. Benchmarks** — `benchmarks/common.py`, `systems/`, `run_*` scripts.
  Lens: imports minilink like external user; no package import of benchmarks.
- [ ] **85. Sphinx docs** — `docs/index.rst`, `docs/api/*.rst`, `docs/conf.py`.
  Lens: stale module paths (`compile/` vs `core/compile/`)?
- [ ] **86. Packaging** — `pyproject.toml`, `minilink/__init__.py` (empty
  namespace), `minilink/_version.py`. ROADMAP P1: public export policy for
  top-level `minilink`.

---

## Phase 18 — Sign-off pass (3 × 10 min)

- [ ] **87. Dependency law audit** — Grep mentally: do libraries import only
  `core` + abstraction? Do tools avoid defining user-facing `System` subclasses?
- [ ] **88. Compiled vs reference parity** — Revisit any compile/sim mismatches
  you noted; run
  `pytest tests/unittest/test_compile_pipeline.py tests/unittest/test_simulator.py -q`.
- [ ] **89. Review queue decisions** — Close ROADMAP §4 items with
  yes/no/rewrite: exports, diagram `validate()`, trajopt consolidation, bicycle
  split, graphics/camera contract.

---

## Suggested pacing

| Block | Sessions | ~Hours |
| --- | --- | --- |
| Phase 0–2 (contracts + core + compile) | 22 | 3.5 h |
| Phase 3–7 (libraries) | 24 | 4 h |
| Phase 8–13 (tools) | 21 | 3.5 h |
| Phase 14–18 (demos + sign-off) | 23 | 3.5 h |

Do **Phase 0 → 1 → 2** first; that covers the framework everything else hangs
on. Run demos in Phases 14–15 only after you have reviewed the module they
exercise.
