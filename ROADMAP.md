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
| **Mechanics** | **TRL 1** | Keep the numeric path functional and ready for user review |
| **Symbolic mechanics** | **TRL 1** | Keep derivation/export workflows working for examples and review |
| **Physics** | **TRL 1** | Keep JAX contact demos working and extend the MVP carefully |
| **Blocks library** | **TRL 0** | Decide what belongs in a real reusable block library |
| **Benchmark helpers** (`minilink.benchmark`) | **TRL 1** | Keep `tests/benchmark/` scripts runnable; grow exports only when a stable need appears |
| **Planning** | **TRL 0** | Not started as a rated subsystem |
| **Control** | **TRL 0** | Not started as a rated subsystem |

> [!NOTE]
> Progress is tracked through **Task Readiness Levels (TRL 1-9)**. See [agent.md](agent.md) for the definitions and the 3-level verification strategy.

## 2. Repository Snapshot

- **Most mature today**: `core/` and the system/diagram composition model
- **Architecture under active validation**: compilation, evaluators, and simulation (including `compile_backend="auto"` and optional auto-`rk4` path for long JAX runs—see `DESIGN.md` §4.5)
- **Stabilizing UX layer (still not core-frozen)**: `graphical/` matplotlib look (`matplotlib_style`), env-aware stacked-figure height for notebooks vs console, and `plot_trajectory` / `System.plot_trajectory` **plot** modes (`"x"`, `"u"`, `"xu"`)
- **Early MVP work**: non-matplotlib render paths, mechanics, symbolic mechanics, physics
- **Exploratory / not stabilized**: blocks, planning, control
- **Still needs a clearer top-level package surface**: public exports and import story
- **Benchmarks**: `minilink.benchmark` + `tests/benchmark/` for optional timing workflows (documented in `DESIGN.md` §4.6)

## 3. Active Priorities

### P0

- Fix external-input behavior through SciPy integration
- Keep docs and public contracts aligned with the actual code

### P1

- Add diagram parametric evaluator tier
- Add diagram validation around subsystem ids and port wiring
- Introduce a `SimulationOptions` dataclass or equivalent solver-config surface
- Finish clarifying the high-level public API and top-level exports

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
- [x] Move internal-signal reconstruction to `DiagramSystem`

### Phase 3: Integration and API cleanup

- [ ] Diagram parametric tier
- [ ] Top-level public exports in `minilink/__init__.py`
- [ ] Solver configuration cleanup (partial: explicit SciPy `rtol`/`atol` on default presets; `COMPILE_BACKEND_AUTO` and auto-RK4 heuristic in code—still room for a `SimulationOptions`-style object)
- [ ] Wiring validation and UX polish

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

Detailed notes remain in [drake_analysis.md](drake_analysis.md) and [pycollimator_analysis.md](pycollimator_analysis.md).

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
| `MechanicalSystem` | `mechanics.MechanicalSystem` | **TRL 1** |
| Symbolic model export | `mechanics.symbolic` | **TRL 1** |
| `Manipulator` | planned | Planned |

## 7. Future Directions

- JAX-based optimization and trajectory optimization
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
