# Minilink Roadmap

Maturity and priorities for the **teaching release**. Contracts and product
identity: [DESIGN.md](DESIGN.md). Agent rules: [AGENTS.md](AGENTS.md).
Operational backlog: [docs/plans/TODO.md](docs/plans/TODO.md).

**Pyro parity audit:**
[docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md).

## 1. North star

Next milestone: **pyro parity (in-scope tools + demos) + minilink mature enough
to release as a teaching tool.**

Minilink is the pyro successor for teaching dynamics and control — readable
equations, diagrams, simulate / analyze / plan / MPC — without becoming a
Simulink GUI, multibody OS, or RL physics engine. Full vision and landscape
position: [DESIGN.md — Product identity & scope](DESIGN.md#product-identity--scope).

## 2. Maturity (TRL)

Readiness levels are an internal maturity scale for planning and review—not a
release process by themselves.

| Level | Name | Description |
| --- | --- | --- |
| **TRL 1** | Agent MVP | Initial code exists and works |
| **TRL 2** | User-check MVP | User performs a high-level functional review |
| **TRL 3** | Architecture Validated | High-level architecture is approved |
| **TRL 4** | Integration Proposed | Final integration/refactor is proposed |
| **TRL 5** | Integration Validated | User approves main-codebase integration |
| **TRL 6** | Automated Tests Pass | Final pytest coverage exists and passes |
| **TRL 7** | Details Validated | Naming and implementation details are approved |
| **TRL 8** | Demo Released | Demo script is created and validated |
| **TRL 9** | Mission Complete | Tests, demo, and user approval are all complete |

| Area | TRL | Rationale | Next |
| --- | --- | --- | --- |
| Core + diagrams | 7 | Public API and diagram API largely stable. | Export policy; remaining edge cases. |
| Compile (`core/compile/`) | 4 | Integrated; evaluator surface still needs review. | Teaching-API review; backend parity. |
| Simulation | 7 | Mature workflow; stable solvers/forcing. | Keep stable; optional `SimulationOptions` cleanup. |
| Optimization | 5 | `MathematicalProgram` + `Optimizer` useful; backends need hardening. | Harden SciPy/Ipopt before TRL 6. |
| Planning/trajopt | 5 | Collocation, shooting, MS; `PlanningProblem` + live plot. | Teaching demos; scene params later. |
| Planning/policy synthesis | 4 | DP + grid; `loop` / `numpy` / `jax`; lookup + `PolicyEvaluator`. | Raster cost maps later. |
| Planning/search | 4 | RRT/RRT*; spatial `Scene` via `X`. | RRT-Connect later. |
| Geometry / spatial | 4 | SDF + `Scene` / fields / bodies; JAX twins tested. | Architecture validation; scene bind later. |
| Graphics / animation | 4 | Frame-keyed `tf` / geometry / overlays; `Animator`. | Renderer polish. |
| Realtime simulation | 2 | `RealtimeSimulator` + pygame I/O; live draw. | Architectural review. |
| Dynamics (abstraction + catalog) | 6 | Plants QA'd; `MechanicalSystem` / `Manipulator`; arms rebased. UR5 `f` uses spatial **ABA**; `inverse_dynamics` uses **RNEA**; `H`/`C`/`g` via RNEA. | Optional `f_ext`; shared RNEA serial stack (Later). |
| Analysis | 5 | Linearize, structural, equilibria, modal, selected-channel Bode. | Frequency completion (priority 1). |
| Control | 6 | Linear, LQR, PID; model-based SMC; robotic impedance/kinematic. | Robotic PID wrappers; traj LQR; SMC demos. |
| Blocks | 5 | Routing, nonlinear, filters, sources, TF, 1-layer NN. | MLP later. |
| Estimation | 1 | Placeholder. | Luenberger, Kalman (priority 2). |
| Identification | 2 | Parametric-tier prototype only. | `fitting.py` (priority 2). |
| Interfaces | 3 | `Sys2Gym` + `SB3Controller` (`minilink[rl]`). | MJX leaf later. |
| Quarantine (symbolic, hand-rolled contact) | 1 | Experimental; not teaching path. | Keep isolated; prefer MJX leaf later. |
| External multibody leaf (MJX) | 0 | Not started. | Spike later (`interfaces/mjx.py`). |
| Pyro 2.0 overall | 3 | Catalog + core + search/DP/trajopt done; many demos unported. | Teaching release via §3–§4; detail in gap doc. |

## 3. Teaching-release criteria

Ready to teach with when:

1. Every **in-scope** pyro library module has a minilink home or documented drop
   ([pyro-port-remaining.md](docs/plans/pyro-port-remaining.md)).
2. Representative closed-loop demo per major plant family (not every pyro script).
3. Teaching bands at usable TRL: core/sim high; analysis frequency tools landed;
   control (incl. robotic teaching demos) ≥ 6; planning search/DP/trajopt
   demonstrable.
4. README includes a pyro → minilink migration guide.
5. Public export policy decided; intro/showcase notebooks match the core teaching
   surface ([AGENTS.md](AGENTS.md) intro-doc scope).
6. Pre-release gate green: ruff + pytest + notebook smoke
   ([tests/README.md](tests/README.md)).

## 4. Teaching-release priorities

Exactly these five (open work for the milestone):

1. [ ] **Frequency / classical MIMO analysis** — `pole_zero_map()`, `nyquist()`,
   `margin()`, `ss2tf()` so courses can cover MATLAB CST-style workflows.
2. [ ] **Remaining pyro library tools** — `trajectory_generation/`; estimation
   (Luenberger/Kalman); `identification/fitting.py`; robotic PID wrappers +
   trajectory LQR
   ([pyro-port-remaining.md](docs/plans/pyro-port-remaining.md)).
3. [ ] **Teaching demos** — representative closed-loop demo per major plant
   family; SMC traj-following; TRL-8 demos for landed teaching bands (not every
   pyro script).
4. [ ] **Docs for teachers / pyro migrators** — README pyro → minilink migration
   guide; public export policy decided; intro/showcase stay aligned with the
   core teaching surface.
5. [ ] **Release hardening** — compile vs reference parity; stable teaching API
   (evaluator surface only as needed); pre-release gate green (ruff + pytest +
   notebook smoke).

## 5. Review queue

Decisions that block or shape the teaching release (maintainer sign-off):

- ~~Public export policy for `minilink/__init__.py`~~ — done (teaching-first
  root prelude + `catalog` / `control` / `analysis` band facades;
  [DESIGN.md §2](DESIGN.md#public-imports-teaching-first)).
- ~~Control-block contract (implementation)~~ — landed: feedback-port
  declaration (`core/feedback.py`), `Controller` / `DynamicController`
  markers, declaration-first `@`, `plot_control_law` /
  `plot_input_output_map`. DESIGN §feedback-profiles still carries
  `TODO: User Architectural Review`
  ([docs/plans/control-block-contract.md](docs/plans/control-block-contract.md)).
- Diagram validation as separate `validate()` vs inline wiring.
- Pyro game demos — port remaining to `simulation/realtime/` or explicitly drop.
- Optional `KinematicModel` delegate — adopt or drop.
- Dynamic bicycle module split (if it blocks teaching demos).

## 6. Backlog homes

Operational and detailed backlogs live outside this file (keep ROADMAP
skimmable for maturity and milestone decisions):

| Doc | Job |
| --- | --- |
| [docs/plans/TODO.md](docs/plans/TODO.md) | Small fixes, pre-v0.2 hardening, demo pulls, new modules, Later ideas |
| [docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md) | Full pyro library / example parity audit |
| [docs/plans/](docs/plans/) | Multi-step design writeups (see [plans README](docs/plans/README.md)) |

## 7. Out of scope

By decision — see [DESIGN.md](DESIGN.md):

Full Simulink / arbitrary multi-clock hybrid parity; event-driven switching as a
framework feature; becoming a Multibody/contact OS or batched RL physics engine.
Narrow step/hybrid for discrete control on continuous plants stays in scope as a
subsidiary path. Live interaction is `simulation/realtime/`, not a pyro
`sys2game` port.
