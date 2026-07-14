# Minilink Roadmap

Maturity and priorities. Contracts: [DESIGN.md](DESIGN.md). Agent rules:
[AGENTS.md](AGENTS.md).

**Pyro 2.0 remaining backlog:**
[docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md).

## 1. Maturity

| Area | TRL | Rationale | Next |
| --- | --- | --- | --- |
| Core + diagrams | 7 | Public API and diagram API are probably stable after the final port-declaration update. | Keep stable; finish export policy and remaining edge cases. |
| Compile (`core/compile/`) | 4 | Integrated, but dynamic evaluator methods and exposed surface still need user review. | Review evaluator API, diagram parametric tier, and backend parity. |
| Simulation | 7 | Mature workflow with stable API and solver/forcing coverage. | Keep behavior stable; treat `SimulationOptions` as ergonomic cleanup, not a redesign. |
| Optimization | 5 | `MathematicalProgram` and `Optimizer` are integrated and useful, but backend details still need hardening. | Harden SciPy/Ipopt behavior and evaluator details before test-gated promotion. |
| Planning/trajopt | 5 | Direct collocation, shooting, MS transcriptions integrated; `PlanningProblem` + live-plot hooks. | Scene params tier; trajectory post-filter. |
| Planning/policy synthesis | 4 | `DynamicProgrammingPlanner` + `StateSpaceGrid`; `loop` / `numpy` / `jax` backends; lookup policy + `PolicyEvaluator`. | Raster cost maps; GPU tuning. |
| Planning/search | 4 | `RRTPlanner`, `RRTStarPlanner`, extenders, steering, KD-tree nearest; spatial `Scene` via `X`. | RRT-Connect; informed sampling. |
| Geometry / spatial | 4 | Integrated architecture proposed for obstacle and terrain planning: `core/geometry.py` SDF primitives + cost algebra (`SumCost`/`ScaledCost`), and `planning/spatial/`: `Scene` (obstacles + `workspace_fields`), `WorkspaceField`/`StateField`, `RobotBody`/`TranslationBody`, export via `as_constraint`/`as_cost`. Tested incl. JAX twins. | User architecture validation; scene params (`ProblemParameters.scene`, future); RRT consumers; oriented/multi-sphere bodies and raster cost maps. |
| Graphical | 4 | Frame-keyed ``tf`` / geometry / overlay contract integrated. | Renderer polish; optional ``KinematicModel`` delegate review. |
| Animation | 4 | ``Animator`` + overlays (``SceneHistory``, ``Replay``); collision reuses ``tf``. | Interactive integrator backends; live I/O backends. |
| Dynamics catalog | 6 | Pyro plants ported, QA'd term-by-term; catalog arms on `Manipulator`. | Optional `JaxManipulator`; `f_ext` port if approved. |
| Dynamics abstraction | 6 | `MechanicalSystem` + `Manipulator` (`p`, `pdot`, FK/J); catalog arms rebased. | `JaxManipulator` if needed; external wrench port. |
| Symbolic mechanics | 1 | One-shot AI-generated demos, not a validated subsystem. | Keep isolated until clear use cases justify review. |
| Contact engine (`dynamics/engines/`) | 1 | Experimental hand-rolled contact; math not QA-validated. | Prefer MJX (or similar) leaf path in §5.11 over growing a Drake-scale in-house contact stack. |
| External multibody leaf (MJX, …) | 0 | Vision only — see §5.11. Continuous `DynamicSystem` wrapper around an external JAX multibody/contact engine for complex plants. | Architecture sketch + spike (load model → `f`/`step` ports → closed-loop with minilink controllers). |
| Analysis | 5 | Linearize, structural, equilibria, modal, selected-channel Bode. | Pole-zero, Nyquist, margins, `ss2tf`; reachability costs. |
| Control | 6 | Linear, LQR, filtered PID; `modelbased.py` (CT, Pyro-parity SMC); `robotic.py` (impedance, kinematic, nullspace). | SMC traj-following demos; dynamic joint/effector PID wrappers; trajectory LQR. |
| Blocks | 5 | Routing, nonlinear, filters, sources, transfer function, 1-layer NN. | Multi-layer `MLP`, atomic layers (see neural-blocks plan). |
| Estimation | 1 | Placeholder only. | Luenberger, Kalman, EKF. |
| Identification | 2 | Parametric-tier prototype demo only. | `fitting.py` for physical + NN params. |
| Interfaces | 1 | Placeholder only. | Gymnasium, Flax/Torch adapters. |
| Pyro 2.0 overall | 3 | Catalog + core framework + planning search/DP/trajopt done; ~143/195 pyro demos not ported. | Phased port per [pyro-port-remaining.md](docs/plans/pyro-port-remaining.md). |

### TRL definitions

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

## 2. Done (architecture)

- Separated model / compile / simulate / optimize / plan / graphics.
- `ExecutionPlan` + NumPy/JAX evaluators; shared `Trajectory`.
- **JAX evaluator trace tier** — fast vs trace execution tiers on compiled
  evaluators ([evaluator-trace-tier-api.md](docs/plans/evaluator-trace-tier-api.md)).
- Composition shortcuts (`+`, `>>`, `@`, `autowire`) → ordinary `DiagramSystem`.
- Explicit ports; `DynamicSystem` textbook ports via constructor options.
- Pure `MathematicalProgram` + `Optimizer`; backend-native trajopt transcriptions.
- Phase-plane plotting (`plot_phase_plane`, matplotlib).
- User docs: [README.md](README.md) (workflows and minimal call chains).
- Package taxonomy: four bands (framework / system libraries / tools /
  quarantine) with a dependency law and placement algorithm
  ([DESIGN.md §3](DESIGN.md)); `compile/` folded into `core/compile/`;
  generic blocks in top-level `blocks/`; generic control laws in
  `control/linear.py` and `control/pid.py`; `System` facades split into `core/facades.py`
  (API unchanged).
- **Pyro catalog plants** — all EoM models ported and QA'd.
- **Kinematic graphics contract** — string-keyed ``tf``, frame-keyed geometry,
  ``skin`` attribute, overlays at ``animate(overlays=…)``, collision ``bind()``
  reusing ``tf`` (DESIGN §4, §6).
- **Pyro tool tranche 1** — blocks routing/nonlinear/filters/sources,
  linear control + LQR, analysis linearize/structural/equilibria/modal/Bode.
- **Planning search + DP** — RRT/RRT*, value iteration, spatial scene integration.
- **DP/RRT on continuous `PlanningProblem`** — discrete pyro framework remains out of scope.

## 3. Priorities

**P0** — Docs/contracts aligned with code; compiled vs reference path parity.

**P1** — **Parametric primitives** (`Shape`, `Set`, `CostFunction` reading `params` at call time); Dynamic evaluator API review; ~~diagram parametric evaluators (`f_p`,
`h_p`)~~ done; diagram validation; top-level `minilink` exports; NLP hardening.

**P2** — ~~Analysis seed~~ done (remaining frequency tools). ~~Blocks round-out~~
done (routing, nonlinear, filters, `TrajectorySource`, PID, MIMO proportional).
~~`control/lqr.py`~~ done. Remaining:

- ~~`Manipulator` base + catalog rebase~~ — done (`manipulator.py`, `arms.py`).
- ~~`control/modelbased.py`, `control/robotic.py`~~ — done.
- Nested-diagram ergonomics; forced-input helpers

**P3** — Remaining pyro 2.0 tools (see [pyro-port-remaining.md](docs/plans/pyro-port-remaining.md)):

- ~~`planning/policy_synthesis/`~~ done; ~~`planning/search/rrt.py`~~ done
- `planning/trajectory_generation/` (polynomial / min-snap)
- `estimation/luenberger.py`, `estimation/kalman.py`
- `identification/fitting.py`
- `interfaces/gymnasium.py`

**P4** — Demo parity + release polish (phases D–G in gap doc):

- Port representative `demos_by_system/` closed-loop scripts per plant
- Frequency completion; obstacle/Pacejka/stochastic layers (if approved)
- Pyro migration guide in README; TRL 8 demos per tool band

**P5 (vision, not scheduled)** — External multibody leaf (**MJX-first**
`DynamicSystem` wrapper for complex plants / contact); see
[§5.11](#511-vision--multibody-via-external-engines-mjx-first). Does not
displace P0–P4; gated on review-queue architecture sign-off.

## 4. Review queue (needs maintainer sign-off)

- Public export policy for `minilink/__init__.py`.
- Diagram validation as separate `validate()` vs inline wiring.
- Trajopt transcription internal consolidation.
- Dynamic bicycle module split.
- Graphics/camera contract consolidation (`KinematicModel` delegate) — optional follow-up.
- **Pyro game demos** — port via interactive animation or explicitly drop.
- **MJX (external multibody) leaf vision** — package home, continuous vs
  discrete adapter split, and deprecate-vs-keep for hand-rolled
  `dynamics/engines/` contact ([§5.11](#511-vision--multibody-via-external-engines-mjx-first)).

## 5. Future

Pre-decided homes ([DESIGN.md §3](DESIGN.md)), build order adjusted for pyro 2.0:

### 5.0 Compile (evaluators)

- [x] **Integration API rename** — `rk4_integrate_{zoh,linear,ivp}`, `euler_integrate_{zoh,ivp}`,
  full `_p` / JAX `_trace` grid, lazy rollout JIT, by-backend evaluator layout
  ([evaluator-integration-api.md](docs/plans/evaluator-integration-api.md))

### 5.1 Analysis

- [x] Linearize (→ matrices/`LTISystem`), ctrb/obsv, equilibria, modal, Bode (selected channel)
- [ ] Pole-zero, Nyquist, margins, `ss2tf`
- [ ] Reachability / domain-check costs (for DP demos)
- [ ] Phase-plane math migration from `graphical/` when touched

### 5.2 Control

- [x] `lqr.py`, `linear.py`, `pid.py` (`FilteredController`)
- [x] `modelbased.py` — computed torque, sliding mode (Pyro parity)
- [x] **Continuous SMC closed loop** — SMC `solver_info` flag, diagram aggregation, auto **Euler** + finer `dt`, forced-solver warnings (see also [DESIGN.md §5 — Discontinuous closed loops](DESIGN.md#discontinuous-closed-loops--known-issues))
- [ ] `robotic.py` — joint/effector PD/PID wrappers (kinematic + nullspace landed)
- [ ] `trajectory_lqr.py` — time-varying LQR along a reference
- [ ] `mpc.py` (uses `optimization/`) — minilink extra, no pyro equivalent
- [ ] `neural.py` — policy wrappers ([neural-blocks-collection.md](docs/plans/neural-blocks-collection.md))

### 5.3 Blocks

- [x] Routing, nonlinear, filters, `TrajectorySource`, `TransferFunction`, 1-layer `NeuralNetwork`
- [ ] Multi-layer `MLP`, `Dense`/activation layers, `mlp_diagram` factory
- [ ] `Switch` (deferred — selection semantics)
- [ ] `RateLimiter`, `Hysteresis` (stateful nonlinear)

### 5.4 Dynamics abstraction

- [x] `MechanicalSystem`, `JaxMechanicalSystem`, `StateSpaceSystem`, `GeneralizedMechanicalSystem`
- [x] `MechanicalSystem` ports `q`, `dq` (`mechanical.py`)
- [x] `Manipulator` base — `p`, `pdot`, `forward_kinematics`, `J` (`manipulator.py`)
- [x] Rebase `dynamics/catalog/manipulators/arms.py` on `Manipulator`
- [ ] Optional `f_ext` input port for external end-effector forces

### 5.5 Planning

- [x] Trajectory optimization (direct collocation, shooting, multiple shooting)
- [x] **MPC compile-once** — `planning/mpc/` (`MPCPlanner`, parametric
  `x_start`, JAX direct collocation; primary demos under `examples/scripts/mpc/`;
  legacy per-step trajopt reference: `demo_dynamic_bicycle_rate_mpc_straight_line_trajopt.py`)
- [ ] **Scene params** — `ProblemParameters.scene`, transcription merge helpers,
  indexed overrides in `Scene` / `StateField` (moving obstacles, scenario sweeps,
  MPC without scene rebuild).
- [ ] **Parametric `core/` primitives** (promoted to P1) — call-time `params`
  overrides on `Shape.sdf`, `Set.margin`, and `CostFunction.g`/`h` (e.g. `BallSet`
  center/radius, `QuadraticCost` weights). Signatures exist; frozen attributes are the
  only source of truth today.
- [ ] `trajectory_generation/` — polynomial / min-snap
- [x] `policy_synthesis/` — `DynamicProgrammingPlanner`, `StateSpaceGrid`,
  `loop` / `numpy` / `jax` backends, `LookupTableController`, `PolicyEvaluator`
- [x] `search/` — `RRTPlanner`, `RRTStarPlanner`, extenders, steering, tree
- [ ] Trajectory post-filter (Butterworth `filtfilt`)

### 5.5a Step / hybrid simulation (subsidiary)

Narrow add-on for discrete control in the loop (MPC, SMC, sampled regulators) on
continuous plants — not full Simulink parity. Minilink remains focused on continuous-time as the primary framework. If the hybrid module matures significantly, it may become the default for discontinuous dynamics.

- [x] **Phase 0** — `WiredDiagramMixin` in `core/wiring.py`; `DiagramSystem` delegates;
  continuous diagram API unchanged
- [x] **Phase 1** — `StepSystem`, `StepRollout`, `compile()` step branch, `compute_rollout` / `plot_rollout`
- [x] **Phase 2** — `StepDiagramSystem`, `compile_step_diagram`, `compute_rollout`
- [x] **Phase 3** — `discretize()` in `minilink/analysis/discretize.py`
- [x] **Phase 4** — `StepSchedule`, `Computer` (`.tick(u)`, double buffer)
- [x] **Phase 5** — `HybridDiagram`, `HybridSimulator`, `HybridSimResult`, multi-rate hybrid demo, Pyro SMC hybrid compare *(5a)*
- [x] **Phase 5** — fine plant recording (`integrate_zoh_rollout`, `plant_dt_inner`); façade `traj` / `last_result` / `rollout` / `animate()`
- [x] **Phase 5c** — `HybridDiagram.plot_diagram`, `build_hybrid_topology`, Mermaid/Graphviz export, `abstract_boundary` topology *(shortcuts: `hybrid_closed_loop`, `Computer @ plant`)*
- [x] **Phase 6a** — `MPCStatelessController` (algebraic block, `u_ff`/`x_ff`/`z`), hybrid straight-line demo, `mpc_plans_from_rollout`
- [x] **Phase 6b** — warm-start `MPCStatefulController` (`StepSystem`, state = optimizer `z`), `warm_start` helpers

### 5.6 Estimation and identification

- [ ] `estimation/luenberger.py`, `kalman.py`, `ekf.py`, `recursive.py`
- [ ] `identification/fitting.py` — one verb for physical params and NN weights

### 5.7 Interfaces

- [ ] `gymnasium.py` — diagram as RL env (train outside)
- [ ] `flax.py`, `torch.py` — external model → static `System`
- [ ] `ros2.py` — ROS 2 node exporter (wraps a minilink block as a ROS 2 node)
- [ ] Cosimulation / FMI
- [ ] Multibody engine wrappers — see **§5.11** (MJX-first; not a second identity)

### 5.8 Catalog capability gaps (not EoM)

- [ ] Obstacle collision layer (replace pyro `*withObstacles` plants)
- [ ] `Pacejka` tire model
- [ ] Stochastic forcing / `NoiseSignal` blocks

### 5.9 Quarantine graduation

- [ ] `symbolic/` as dynamics-authoring tool
- [ ] Hand-rolled contact in `dynamics/engines/` — graduate **or** deprecate in favor
  of §5.11 external engine leaves once an MJX (or peer) wrapper is TRL ≥ 3

### 5.10 Out of scope (by decision)

Full Simulink / arbitrary multi-clock hybrid parity, event-driven switching
(guards, impacts) as a *framework* feature, RNN policy blocks, becoming a
standalone batched RL physics engine (Brax/MJX competitor) — see
[DESIGN.md §3](DESIGN.md). The **narrow** step/hybrid subset in §5.5a is in
scope as a subsidiary program; it does not replace the continuous-time core.
Pygame game framework (`sys2game`) — no first-class port unless reversed.

**In scope as leaves (not as core rewrite):** wrapping an external multibody /
contact engine (MJX first) as a `DynamicSystem` — §5.11. That is *using*
MJX for complex plants, not *rebuilding* MJX/Drake inside minilink.

### 5.11 Vision — Multibody via external engines (MJX-first)

**Status:** directional vision (TRL 0). Needs maintainer architecture sign-off
before a spike lands in-tree. Does **not** change the continuous-core grammar.

#### Intent

Textbook catalog plants and `Manipulator` remain the default teaching /
controls surface. For **complex multibody + contact** (multi-DoF robots,
manipulation scenes), prefer a **thin `DynamicSystem` leaf** that wraps an
external JAX-native engine — starting with **MuJoCo MJX** — rather than growing
an in-house Drake-scale MultibodyPlant or competing as a physics engine.

```text
minilink controller / NN / HybridDiagram
        │  ports u, y (and optional q, dq, …)
        ▼
  MjxPlant(DynamicSystem)     ← thin adapter leaf
        │
        ▼
     MJX model / data         ← owns contact, constraints, assets
```

Continuous EoM stay first-class at the leaf boundary when the engine exposes
them (`f` / mass-matrix / bias style); timestepped contact advance may also
appear as an optional discrete path without polluting `DynamicSystem` itself
(sibling `StepSystem` or documented ZOH/`HybridDiagram` usage — same law as
§5.5a).

#### Why this fits the competitive edge

| Keep (minilink identity) | Borrow (engine leaf) | Do not become |
| --- | --- | --- |
| Causal Systems diagrams, `f(x,u,t,params)`, compile/JAX tiers | MJX articulated bodies, contacts, assets | Drake/MJX competitor as a physics OS |
| Control, trajopt, DP/RRT, analysis, animate | Fast/differentiable plant rollouts | Simulink DAE / Stateflow framework |
| NN / ID as peer Systems on the same diagram | Gradients through the plant when MJX+JAX allow | CasADi-only OCP language |

Position vs toolboxes (summary of strategy discussion):

- **vs Drake** — compose and differentiate control/learning *around* complex
  plants; do not re-implement MultibodyPlant / hydroelastic as the product.
- **vs MJX/MuJoCo** — they own physics; we own diagrammed control, hybrid digital
  loops, planning tools, and teaching UX wired to that plant.
- **vs CasADi / acados** — Systems-first; use engine leaf + minilink MathProg /
  JAX transcriptions rather than becoming an SX/MX modeling language.
- **vs Simulink** — code-first causal diagrams; no GUI/DAE/network ambition.
- **vs Brax** — not a batched RL engine; batch later only as a tool facade over
  the same leaf if needed.

#### Architectural constraints (non-negotiable)

1. **Leaf, not core** — adapter lives under `dynamics/engines/` or
   `interfaces/` (placement TBD at TRL 3); `System` / `DynamicSystem` /
   `DiagramSystem` contracts unchanged
   ([DESIGN.md §3](DESIGN.md#continuous-time-core-stephybrid-subsidiary)).
2. **Args/results purity** — leaf still implements `f` / `h` (and/or `step`)
   with `(x, u, t, params)`; no Context-shaped equation API.
3. **Optional dependency** — MJX/MuJoCo lazy or extra (`minilink[mjx]`);
   default install and catalog demos stay free of it.
4. **Params story** — engine knobs map into nested `params` (diagram-compatible)
   where practical; freeze vs call-time overrides follow the same tiers as other
   plants.
5. **Graphics** — prefer bridge to existing `tf` / frame-keyed geometry or
   Meshcat overlays; do not fork a second viz stack in core.
6. **Contact hardness stays in the engine** — minilink Simulator keeps ODE /
   HybridDiagram semantics; do not add diagram-level DAE or witness frameworks
   just to host MJX.

#### Suggested phases (when scheduled)

| Phase | Outcome | Exit |
| --- | --- | --- |
| **0 — Spike** | Load a simple MJX model; map state/actuation to ports; `f` or fixed-step rollout; compare energy / one closed loop (`ctl @ plant`) | Architecture note + go/no-go |
| **1 — Leaf MVP** | `MjxPlant` (name TBD) as `DynamicSystem`; Nominal + nontrivial params test; JAX twin or trace-tier path | TRL 2–3 |
| **2 — Control demos** | Impedance / PD / MPC or HybridDiagram on an arm or mobile base from MJX assets | TRL 8 demo |
| **3 — Differentiable hooks** | Document what is `grad`-able (MJX limitations honest); one ID or residual-NN demo through the leaf | Aligns with JAX edge |
| **4 — Optional batch façade** | `vmap`-style rollouts as a *tool*, not a new System kind | Only if demos demand it |

Pinocchio (or similar) may appear later as a second engine leaf for
contact-light RBD/derivatives; MJX is the contact-capable default candidate.

#### Explicit non-goals for this track

- Replacing the pyro catalog with MJX models for teaching plants
- Making MJX a required backend for `compile(backend="jax")`
- Diagram-level algebraic-loop / DAE solving for contact
- Feature parity with Drake `MultibodyPlant` + `SceneGraph`
- Shipping a second hand-rolled contact solver that races MJX

#### Review queue hooks

Before implementation: confirm package home (`dynamics/engines/` vs
`interfaces/`), naming (`MjxPlant` vs `MuJoCoSystem`), continuous-`f` vs
discrete-step adapter split, and whether hand-rolled `dynamics/engines/`
contact is quarantined-for-deprecate once Phase 1 exists
(add to [§4 Review queue](#4-review-queue-needs-maintainer-sign-off)).

## 6. Pyro 2.0 port status

Snapshot vs [SherbyRobotics/pyro](https://github.com/SherbyRobotics/pyro) (June 2026).

| Bucket | Pyro | Minilink | Gap |
| --- | ---: | ---: | ---: |
| Catalog plants | 40+ | 40+ | **0** (QA complete) |
| Library modules | 38 | ~25 equivalent | **~10 tools** (control nonlinear/robot, DP, RRT, traj gen, estimation, interfaces) |
| Example scripts | 195 | 60 | **~135** |
| Course notebooks | 3 | 7 (new topics) | different mix |

### Port phases (from gap doc)

| Phase | Focus | Unblocks |
| --- | --- | --- |
| **A** | ~~`Manipulator` + computed torque + sliding mode + `robotic.py`~~ (library landed) | representative closed-loop demos per plant band |
| **B** | ~~DP/RRT~~ + polynomial traj gen + trajectory LQR | remaining: traj gen, traj LQR |
| **C** | Estimation + identification + Gym interface | LQG + RL demos |
| **D** | Frequency completion + planning cost variants | analysis + DP reachability |
| **E** | Obstacles, Pacejka, stochastic | specialized catalog |
| **F** | Demo audit (`demos_by_system/`, courses, projects) | pyro parity visibility |
| **G** | Export policy, compile review, migration guide, TRL 8 | release criteria |

### Definition of done (pyro 2.0)

1. Every **in-scope** pyro library module has a minilink home or documented replacement.
2. Every **in-scope** pyro plant folder has ≥1 representative closed-loop demo.
3. DP/RRT approved and landed **or** explicitly dropped with alternatives documented.
4. Robot control suite (computed torque, sliding mode, robotic) at TRL ≥ 6.
5. `identification/fitting.py` covers the params-gradient workflow.
6. README includes a pyro → minilink API migration guide.

Full per-module backlog:
[docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md).
