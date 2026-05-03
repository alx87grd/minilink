# Minilink Roadmap & Pyro Migration

This document tracks subsystem maturity, active priorities, and the longer-term migration path toward a Pyro-style ecosystem built on `minilink`.

## 1. Maturity Matrix

| Component | Status | Next milestone |
| --- | --- | --- |
| **Core abstractions** | **TRL 7** | Finalize stable public exports and remaining naming/details |
| **Compile pipeline** | **TRL 4** | Validate architecture and deepen integration across the library |
| **Leaf evaluators** | **TRL 4** | Add differentiation and richer rollout helpers |
| **Diagram evaluators** | **TRL 4** | Add diagram parametric tier and finish boundary/internal signal polish |
| **Simulation** | **TRL 4** | Harden the new simulator path, unify forcing behavior, and keep compile-backend / auto-solver heuristics well-defined |
| **Graphical** | **TRL 2** | Matplotlib theme and layout policy are converging; keep **interactive integrator** + **live I/O** backends (see §7) before treating graphics as “frozen” like `core/` |
| **Dynamics abstractions** (`dynamics/abstraction/`) | **TRL 1** | Keep the numeric ladder (e.g. `MechanicalSystem`) functional and ready for user review |
| **Symbolic mechanics** | **TRL 1** | Keep derivation/export workflows working for examples and review |
| **Physics** | **TRL 1** | Keep JAX contact demos working and extend the MVP carefully |
| **Dynamics library** (`minilink.dynamics.catalog`) | **TRL 0** | Curated plant models; grow by domain (`vehicles/`, `msd/`, `pendulum/`, …) |
| **Blocks library** (`minilink.core.blocks`) | **TRL 0** | Wiring and signal primitives only; not the home for full plants |
| **Benchmark helpers** (`minilink.compile.benchmark` / `minilink.simulation.benchmark` / `minilink.planning.trajectory_optimization.benchmark`) | **TRL 1** | Keep `tests/benchmark/` scripts runnable; `simulation/scenarios/` holds shared stress scenarios |
| **Planning** | **TRL 1** | Deterministic planning architecture; generic trajectory-optimization planner plus direct collocation, single shooting, multiple shooting, live-plot callback hooks, and JAX-traceable trajopt prototypes that now compile through `MathematicalProgram` evaluators (see [DESIGN.md](DESIGN.md) §3.5) |
| **Optimization** (`minilink.optimization`) | **TRL 1** | Pure `MathematicalProgram` NLP description plus NumPy/JAX program evaluators and bound optimizer method presets shared by direct optimization and trajectory optimization |
| **Control** (`minilink.control`) | **TRL 0** | Controller blocks (starting with tutorial PD); expand as patterns stabilize |

> [!NOTE]
> Progress is tracked through **Task Readiness Levels (TRL 1-9)**. See [agent.md](agent.md) for the definitions and the 3-level verification strategy.

## 2. Repository Snapshot

- **Diagram primitives layout**: reusable sources, integrators, and small signal blocks live under **`minilink.core.blocks`** (nested in `core/`), not a top-level `minilink.blocks` package.
- **Most mature today**: `core/` and the system/diagram composition model
- **Architecture under active validation**: compilation, evaluators, and simulation (including `compile_backend="auto"` and optional auto-`rk4` path for long JAX runs—see `DESIGN.md` §4.5)
- **Stabilizing UX layer (still not core-frozen)**: `graphical/` matplotlib look (`matplotlib_style`), env-aware stacked-figure height for notebooks vs console, and `plot_trajectory` / `System.plot_trajectory` **plot** modes (`"x"`, `"u"`, `"xu"`)
- **Early MVP work**: non-matplotlib render paths, `dynamics/`, `symbolic/mechanics/`, `physics/`
- **Exploratory / not stabilized**: `dynamics/`, `core/blocks/`, `control`; `planning` now has family-level deterministic architecture contracts awaiting review
- **Pyro-style plant ports**: `dynamics/catalog/pendulum/` includes `CartPole` and `DoublePendulum` on `MechanicalSystem` (see `DESIGN.md` §2.7)
- **Still needs a clearer top-level package surface**: public exports and import story
- **Layout and naming**: on-disk `minilink/` tree, pluggable-role file naming, and package boundaries are documented in [DESIGN.md](DESIGN.md) §2
- **Benchmarks**: subsystem-local `benchmark.py` modules + `tests/benchmark/` for optional benchmark workflows (documented in `DESIGN.md` §4.6)

## 3. Active Priorities

### P0

- Fix external-input behavior through SciPy integration
- Keep docs and public contracts aligned with the actual code

### P1

- Add diagram parametric evaluator tier
- Add diagram validation around subsystem ids and port wiring
- Introduce a `SimulationOptions` dataclass or equivalent solver-config surface
- Finish clarifying the high-level public API and top-level exports
- Review deterministic planning contracts (`PlanningProblem`, sets, costs,
  family-specific result types, generic trajectory optimization, and
  transcriptions) before widening RRT/DP and trajopt feature coverage; **JAX
  direct collocation** is explicitly a subset of the NumPy feature surface until
  set/cost coverage matches or the API is split deliberately

### P2

- Add core reusable blocks such as `StateSpaceSystem`, `TransferFunction`, and `PID`
- Add linearization and differentiation helpers on compiled evaluators
- Add port export / nesting ergonomics for `DiagramSystem`
- Extend simulator-level forcing helpers beyond the new `compute_forced(...)` shortcut when richer experiment workflows become clearer
- **Interactive integrator backends** (see `Animator.game` / `run_interactive` today): factor the fixed-step loop and Euler-only stepping behind a small **base class + pluggable backends**, mirroring the idea of `Simulator` + multiple integration schemes—so real-time loops can swap integrator without rewriting pygame/render glue
- **Live I/O backends for interactive mode**: today live `u` is read only via **pygame keyboard** inside `Animator.game`; introduce a **base class + backends** (keyboard, **TCP/UDP or similar for cosimulation**, file replay, etc.). **Live output push** (streaming state to a peer) is out of scope for now but should stay easy to add beside the same abstraction—see comments in `minilink/graphical/animation.py` and `ROADMAP.md` §7

## 4. Development Phases

### Phase 1: Infrastructure

- [x] `pyproject.toml`, CI, linting, typing foundation

### Phase 2: Core architecture

- [x] Decouple modeling from graphics and simulation with lazy imports
- [x] Consolidate compilation around a shared `ExecutionPlan`
- [x] Add `DynamicsEvaluator` as the main compiled API
- [x] Add NumPy and JAX leaf evaluators
- [x] Add NumPy and JAX diagram evaluators
- [x] Move simulation to `minilink.simulation`
- [x] Move the official trajectory object to `minilink.core.trajectory`
- [x] Move diagram primitives from top-level `blocks/` into `core/blocks/` (import `minilink.core.blocks`)
- [x] Move internal-signal reconstruction to `DiagramSystem`

### Phase 3: Integration and API cleanup

- [ ] Diagram parametric tier
- [ ] Top-level public exports in `minilink/__init__.py`
- [ ] Solver configuration cleanup (partial: explicit SciPy `rtol`/`atol` on default presets; `COMPILE_BACKEND_AUTO` and auto-RK4 heuristic in code—still room for a `SimulationOptions`-style object)
- [ ] Wiring validation and UX polish
- [x] Migrate `compile/`, `simulation/`, and `graphical/` package layout: `evaluators/`, `solvers/`, `renderers/renderer.py`, subsystem-local benchmark modules, no top-level benchmark package; layout rules consolidated in [DESIGN.md](DESIGN.md) §2

### Phase 4: User-facing library growth

- [ ] Core reusable block library
- [ ] Linearization tools
- [ ] Port exporting / nested diagrams
- [ ] Optional operator overloading / reference-based connection ergonomics
- [ ] Interactive real-time loop: **integrator backends** (not only Euler in `Animator.game`)
- [ ] Interactive real-time loop: **live input backends** (not only pygame keys; e.g. cosimulation socket); live output streaming later

## 5. External Design Lessons

Cross-reading Drake and pycollimator still points to the same high-level conclusions:

- keep equations separate from runtime state
- keep diagram-level I/O first-class
- prefer explicit backend selection over global mutable backend switches
- keep the compile-once flat IR unless profiling proves it is the wrong abstraction

Ad-hoc design notes are folded into the repository history; no separate analysis docs are required for day-to-day work.

## 6. Pyro Migration Direction

`minilink` is intended to become the cleaner, more composable base for future Pyro-like robotics and control tooling.

Guiding ideas:

- textbook-readable equations
- named-port MIMO composition
- true diagram composition rather than flat vector plumbing
- compiled NumPy / JAX execution paths
- headless-first architecture
- validation through tests, manual scripts, and demos

Current mapping:

| Pyro feature | Minilink equivalent | Status |
| --- | --- | --- |
| `ContinuousDynamicSystem` | `DynamicSystem` | **TRL 7** |
| Compiled leaf dynamics | `NumpyLeafEvaluator` / `JaxLeafEvaluator` | **TRL 4** |
| Compiled diagram dynamics | `NumpyDiagramEvaluator` / `JaxDiagramEvaluator` | **TRL 4** |
| `StateSpaceSystem` | planned | Planned |
| `MechanicalSystem` | `minilink.dynamics.abstraction.mechanical` | **TRL 1** |
| Symbolic model export | `minilink.symbolic.mechanics` | **TRL 1** |
| `Manipulator` | planned | Planned |

## 7. Future Directions

- mature JAX-based optimization and trajectory optimization
- fully differentiable simulation with scan- or custom-VJP-based rollouts
- hybrid systems and event handling
- differentiable LQR / control tooling
- Gymnasium / RL bridges
- richer real-time interactive control loops
- multibody plant workflows and model import

### Interactive graphics / cosimulation (planned shape)

Two separate **backend-style** extension points (base class + multiple implementations), analogous to choosing a solver on `Simulator`:

1. **Real-time integrator** — `Animator.game` (and related loops) currently embed a **fixed Euler** inner loop with `dynamics_substeps`. This should become swappable (e.g. RK substep, compiled `f`, or reusing simulator machinery) without entangling pygame or the renderer.
2. **Live external I/O** — **Input**: today only **pygame keyboard** supplies `u` each frame; other backends could read **TCP/UDP** (cosimulation), shared memory, etc. **Output**: optional **live push** of state or outputs to a socket or sink is a natural sibling API for later; not required for the first cut.

Keep user-facing demos thin; hide protocol and substeps in `graphical/` or a small dedicated submodule when implemented.
