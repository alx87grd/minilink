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
| Contact engine (`dynamics/engines/`) | 1 | Experimental; math not QA-validated. | Validation tests toward TRL 2. |
| Analysis | 5 | Linearize, structural, equilibria, modal, selected-channel Bode. | Pole-zero, Nyquist, margins, `ss2tf`; reachability costs. |
| Control | 6 | Linear, LQR, filtered PID; `modelbased.py` (CT, SMC); `robotic.py` (impedance, kinematic, nullspace). | Sliding-mode + traj-following demos; dynamic joint/effector PID wrappers; trajectory LQR. |
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

**P1** — Dynamic evaluator API review; ~~diagram parametric evaluators (`f_p`,
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

## 4. Review queue (needs maintainer sign-off)

- Public export policy for `minilink/__init__.py`.
- Diagram validation as separate `validate()` vs inline wiring.
- Trajopt transcription internal consolidation.
- Dynamic bicycle module split.
- Graphics/camera contract consolidation (`KinematicModel` delegate) — optional follow-up.
- **Pyro game demos** — port via interactive animation or explicitly drop.

## 5. Future

Pre-decided homes ([DESIGN.md §3](DESIGN.md)), build order adjusted for pyro 2.0:

### 5.1 Analysis

- [x] Linearize (→ matrices/`LTISystem`), ctrb/obsv, equilibria, modal, Bode (selected channel)
- [ ] Pole-zero, Nyquist, margins, `ss2tf`
- [ ] Reachability / domain-check costs (for DP demos)
- [ ] Phase-plane math migration from `graphical/` when touched

### 5.2 Control

- [x] `lqr.py`, `linear.py`, `pid.py` (`FilteredController`)
- [x] `modelbased.py` — computed torque, sliding mode
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
- [ ] **Parametric `core/` primitives** (deferred follow-up) — call-time `params`
  overrides on `Shape.sdf`, `Set.margin`, and `CostFunction.g`/`h` (e.g. `BallSet`
  center/radius, `QuadraticCost` weights). Signatures exist; frozen attributes are the
  only source of truth today.
- [ ] `trajectory_generation/` — polynomial / min-snap
- [x] `policy_synthesis/` — `DynamicProgrammingPlanner`, `StateSpaceGrid`,
  `loop` / `numpy` / `jax` backends, `LookupTableController`, `PolicyEvaluator`
- [x] `search/` — `RRTPlanner`, `RRTStarPlanner`, extenders, steering, tree
- [ ] Trajectory post-filter (Butterworth `filtfilt`)

### 5.6 Estimation and identification

- [ ] `estimation/luenberger.py`, `kalman.py`, `ekf.py`, `recursive.py`
- [ ] `identification/fitting.py` — one verb for physical params and NN weights

### 5.7 Interfaces

- [ ] `gymnasium.py` — diagram as RL env (train outside)
- [ ] `flax.py`, `torch.py` — external model → `StaticSystem`
- [ ] Cosimulation / FMI, multibody import

### 5.8 Catalog capability gaps (not EoM)

- [ ] Obstacle collision layer (replace pyro `*withObstacles` plants)
- [ ] `Pacejka` tire model
- [ ] Stochastic forcing / `NoiseSignal` blocks

### 5.9 Quarantine graduation

- [ ] `symbolic/` as dynamics-authoring tool
- [ ] Contact engine validation (`dynamics/engines/`)

### 5.10 Out of scope (by decision)

Discrete time (ZOH/delay, digital control), RNNs, mixed-rate simulation,
differentiable-rollout library, hybrid/events — see [DESIGN.md §3](DESIGN.md).
Pygame game framework (`sys2game`) — no first-class port unless reversed.

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
