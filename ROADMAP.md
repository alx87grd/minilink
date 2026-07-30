# Minilink Roadmap

Maturity and priorities for the **teaching release**. Contracts and product
identity: [DESIGN.md](DESIGN.md). Agent rules: [AGENTS.md](AGENTS.md).

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
| Interfaces | 1 | Placeholder. | Gymnasium later; MJX leaf later. |
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
- Diagram validation as separate `validate()` vs inline wiring.
- Pyro game demos — port remaining to `simulation/realtime/` or explicitly drop.
- Optional `KinematicModel` delegate — adopt or drop.
- Dynamic bicycle module split (if it blocks teaching demos).

## 6. Pre-v0.2 hardening backlog (provisional bands)

Findings from the v0.1 pre-freeze architecture review that touch
**provisional** bands ([README.md — API stability](README.md#api-stability-v01)).
Deferred by decision — the v0.1 freeze fixes only the stable tier. Each row is
problem → proposed solution.

| Problem | Proposed solution |
| --- | --- |
| **`MultipleShootingTranscription` inherits collocation `transcribe_parametric`** (`planning/trajectory_optimization/multiple_shooting.py`): `compile_parametric_program()` with multiple shooting silently builds collocation defects. **Highest priority — cheap guard.** | Override to raise `NotImplementedError` (or implement true MS parametric); replace the planner's `hasattr(..., "transcribe_parametric")` feature check with an explicit capability flag. |
| RRT `KinodynamicExtender` ignores `problem.params.system` (`planning/search/extenders.py`): propagates with frozen-params `rk4_step`; `U.sample` also drops `params.sets`. | Use `rk4_step_p` with `problem.params.system` (frozen compile when `None`); thread `params.sets` into input sampling. |
| MPC port computes drop `params` (`control/mpc/controller.py` `del params` in `_compute_*` / `step`): online params cannot reach the hybrid tick path. | Document that ports ignore `params` (or raise on non-`None`); align `step_disp` with the `verbose` convention. |
| Planning band exports empty: `planning/`, `trajectory_optimization/`, `search/`, `policy_synthesis/` `__init__` export nothing while `spatial/` exports richly. | Decide band facade (`_EXPORTS` + lazy `__getattr__` like `catalog/`) vs a documented deep-import policy; apply uniformly. |
| `ShootingTranscription` orphaned: implemented and tested, absent from `transcription=` string presets. | Add a `"shooting"` preset or delete the class (pre-1.0 no-alias rule: no half-exposed API). |
| `ParametricMathematicalProgram` / `JaxParametricProgramEvaluator` placement and duplication: live under `planning/`, duck-copy `optimization/evaluators/jax_evaluator.py` helpers; parametric path supports only `scipy_slsqp` vs offline `Optimizer` presets. | Relocate beside `MathematicalProgram` in `optimization/`; dedupe evaluator helpers; document or close the optimizer-method cliff. |
| Dual online-params façades: `ProblemParameters.scene` field is never read while online `params={"scene": …}` raises — two things named "params". | Unify naming/messaging when pipeline B (`J(z, p)` bind) lands; until then keep the loud `NotImplementedError`. |
| `HybridSimulator` conventions drift: ad-hoc one-line verbose vs `sim_reporting` panels; `last_result` vs `last_traj` cache naming; forced-input coercion copy; kwargs-only ctor differs from `Simulator`. | Adopt `sim_reporting` panels; pick one cache name (no aliases); reuse the shared forced-input helper from `simulation/input_coercion`; align ctor style. |
| `HybridDiagram` hand-copies facade bodies (`animate` / plot vs `SharedSystemFacades`). | Share the facade implementation once the hybrid API stabilizes. |
| DP metadata: `DynamicProgrammingOptions.final_time` ignores `problem.tf`; `SolveMetadata.success` is always `True` even when tolerance is not met. | Read horizon from `problem.tf` when set; report convergence honestly in metadata. |
| Verbose flag naming varies: planner `solve_disp` / `step_disp`, optimizer `disp`, DP `verbose`. | Standardize on `verbose` for setup/report printing; keep genuinely different knobs (`live_plot`) distinct. |
| `interfaces/c_export.py`: a real experimental JAX→C transpiler ships inside a "placeholder" band with no TRL row. | Move to quarantine or add an honest TRL row here; do not let it ride the freeze implicitly. |
| Realtime band: `TODO: User Architectural Review` markers stand; `compile_backend=None`→auto default diverges from offline simulators. | Architectural review (existing §2 row); align auto-backend policy with the shared helper. |
| Spatial exports: `CostDensityField` / `WorkspaceField` are public-ish but unexported from `planning/spatial/__init__.py`. | Decide export or keep internal; document either way. |

## 7. Later / out of scope

**Later** (post teaching release — do not displace §4). Lightweight **feature
backlog**: one-line ideas land here; if a change needs architecture tradeoffs or
a multi-step design, open a short doc under [`docs/plans/`](docs/plans/) and link
it from [docs/plans/README.md](docs/plans/README.md).

- Scene params / `J(z, p)` bind (moving obstacles, terrain SDFs without JIT rebuild)
- `SolverFactory` — unify SciPy / Ipopt / CasADi wiring for trajopt and MPC
- `MjxPlant` under `interfaces/mjx.py` (`minilink[mjx]`); prefer deprecate
  hand-rolled contact in `dynamics/engines/`
- Gymnasium / RL bridges; Pacejka; stochastic forcing; neural MLP
- ROS2 / FMI; sparse long-horizon trajopt; parametric `core/` Shape/Set/Cost
  call-time overrides; trajectory post-filter; RRT-Connect / informed sampling
- **Shared RNEA serial-chain stack** (not copy-paste per 6-DoF arm): extract
  DH + spatial RNEA helpers (or a thin `SerialRneaManipulator` base) so catalog
  plants only supply `a`/`d`/`alpha`, mass/COM/inertia; keep public `H`/`C`/`g`
  on the mechanical API.
- **ABA on other RNEA arms** (pattern from UR5): keep public `H` / `C` /
  `g` for teaching; use Articulated-Body Algorithm for `forward_dynamics` /
  `f` so integration does not form \(H\) each step. UR5 catalog plant done;
  generalize when adding the next spatial manipulator.

**Out of scope** (by decision — see [DESIGN.md](DESIGN.md)):

Full Simulink / arbitrary multi-clock hybrid parity; event-driven switching as a
framework feature; becoming a Multibody/contact OS or batched RL physics engine.
Narrow step/hybrid for discrete control on continuous plants stays in scope as a
subsidiary path. Live interaction is `simulation/realtime/`, not a pyro
`sys2game` port.
