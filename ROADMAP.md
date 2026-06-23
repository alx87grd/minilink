# Minilink Roadmap

Maturity and priorities. Contracts: [DESIGN.md](DESIGN.md). Agent rules:
[agent.md](agent.md).

**Pyro 2.0 master gap checklist:**
[docs/plans/pyro-port-gap-todo.md](docs/plans/pyro-port-gap-todo.md).

## 1. Maturity

| Area | TRL | Rationale | Next |
| --- | --- | --- | --- |
| Core + diagrams | 7 | Public API and diagram API are probably stable after the final port-declaration update. | Keep stable; finish export policy and remaining edge cases. |
| Compile (`core/compile/`) | 4 | Integrated, but dynamic evaluator methods and exposed surface still need user review. | Review evaluator API, diagram parametric tier, and backend parity. |
| Simulation | 7 | Mature workflow with stable API and solver/forcing coverage. | Keep behavior stable; treat `SimulationOptions` as ergonomic cleanup, not a redesign. |
| Optimization | 5 | `MathematicalProgram` and `Optimizer` are integrated and useful, but backend details still need hardening. | Harden SciPy/Ipopt behavior and evaluator details before test-gated promotion. |
| Planning/trajopt | 2 | Direct collocation / shooting exist; DP/RRT/polynomial generation not ported. | Architectural review for offline DP/RRT; traj generation. |
| Planning/policy synthesis | 1 | Pyro DP/value iteration not ported; stubs removed. | Redesign under `planning/policy_synthesis/` (see pyro gap doc §4). |
| Planning/search | 1 | Pyro RRT not ported; stubs removed. | Redesign under `planning/search/rrt.py`. |
| Geometry / environment | 1 | `core/geometry.py` SDF primitives (`Sphere`/`Box`/`Union`/`Inflated`) + cost algebra (`SumCost`/`ScaledCost`) landed as the dual constraint/cost foundation for obstacles. | Build `planning/environment/` (`Obstacle`, `GaussianField`, `RobotShape`, `Environment` with `as_free_set`/`as_cost`), then RRT consumers. |
| Graphical | 3 | Useful, but plotting/diagram APIs are still evolving. | Kinematic composition review before API freeze. |
| Animation | 3 | Substantial work exists, but renderer, camera, and live-loop contracts may still change. | Same gate as Graphical. |
| Dynamics catalog | 6 | Pyro plants ported, QA'd term-by-term; `DynamicBicycle` params thread fully. | `Manipulator` abstraction + catalog rebase (see review queue). |
| Dynamics abstraction | 5 | `MechanicalSystem` + joint ports `q`/`dq`; `Manipulator` in `manipulator.py` with `p`/`pdot`. | Rebase catalog arms; `JaxManipulator` if needed. |
| Symbolic mechanics | 1 | One-shot AI-generated demos, not a validated subsystem. | Keep isolated until clear use cases justify review. |
| Contact engine (`dynamics/engines/`) | 1 | Experimental; math not QA-validated. | Validation tests toward TRL 2. |
| Analysis | 5 | Linearize, structural, equilibria, modal, selected-channel Bode. | Pole-zero, Nyquist, margins, `ss2tf`; reachability costs. |
| Control | 5 | Linear, LQR, filtered PID done. | Computed torque, sliding mode, robotic controllers (blocked on `Manipulator`). |
| Blocks | 5 | Routing, nonlinear, filters, sources, transfer function, 1-layer NN. | Multi-layer `MLP`, atomic layers (see neural-blocks plan). |
| Estimation | 1 | Placeholder only. | Luenberger, Kalman, EKF. |
| Identification | 2 | Parametric-tier prototype demo only. | `fitting.py` for physical + NN params. |
| Interfaces | 1 | Placeholder only. | Gymnasium, Flax/Torch adapters. |
| Pyro 2.0 overall | 3 | Catalog + core framework done; ~151/195 pyro demos not yet ported. | Phased port per [pyro-port-gap-todo.md](docs/plans/pyro-port-gap-todo.md) §5. |

TRL definitions: [agent.md §8](agent.md#8-trl-lifecycle).

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
- **Pyro catalog plants** — all EoM models ported and QA'd
  ([catalog-migration-notes.md](docs/plans/catalog-migration-notes.md)).
- **Pyro tool tranche 1** — blocks routing/nonlinear/filters/sources,
  linear control + LQR, analysis linearize/structural/equilibria/modal/Bode
  ([tool-migration-notes.md](docs/plans/tool-migration-notes.md)).

## 3. Priorities

**P0** — Docs/contracts aligned with code; compiled vs reference path parity.

**P1** — Dynamic evaluator API review; ~~diagram parametric evaluators (`f_p`,
`h_p`)~~ done; diagram validation; top-level `minilink` exports; NLP hardening.

**P2** — ~~Analysis seed~~ done (remaining frequency tools). ~~Blocks round-out~~
done (routing, nonlinear, filters, `TrajectorySource`, PID, MIMO proportional).
~~`control/lqr.py`~~ done. Remaining:

- `Manipulator` abstraction (`q`/`dq`/`p`/`pdot` ports) — **unblocks robot control**
- `control/computed_torque.py`, `control/sliding_mode.py`, `control/robotic.py`
- Nested-diagram ergonomics; forced-input helpers; graphics kinematic composition

**P3** — Pyro 2.0 tool port (phases B–C in gap doc):

- `planning/policy_synthesis/` (DP, lookup policy, grid discretizer)
- `planning/search/rrt.py`
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
- Graphics/camera contract consolidation (`KinematicModel` delegate).
- **`Manipulator` base class** — `MechanicalSystem` + `q`/`dq` ports;
  `Manipulator` + `p`/`pdot` + `forward_kinematics` / `J(q)` (plant task outputs;
  controller reference stays `r` per DESIGN)
  ([manipulator-abstraction.md](docs/plans/manipulator-abstraction.md)).
- **DP/RRT return** — offline planning on continuous plants vs out-of-scope
  discrete framework (see gap doc §4).
- **Pyro game demos** — port via interactive animation or explicitly drop.

## 5. Future

Pre-decided homes ([DESIGN.md §3](DESIGN.md)), build order adjusted for pyro 2.0:

### 5.1 Analysis

- [x] Linearize (→ matrices/`LTISystem`), ctrb/obsv, equilibria, modal, Bode (selected channel)
- [ ] Pole-zero, Nyquist, margins, `ss2tf`
- [ ] Reachability / domain-check costs (for DP demos)
- [ ] Phase-plane math migration from `graphical/` when touched

### 5.2 Control

- [x] `lqr.py`, `linear.py`, `pid.py` (`FilteredPIDController`)
- [ ] `computed_torque.py`, `sliding_mode.py`
- [ ] `robotic.py` — joint/effector PD/PID, kinematic, nullspace
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
- [ ] Rebase `dynamics/catalog/manipulators/arms.py` on `Manipulator`
- [ ] Optional `f_ext` input port for external end-effector forces

### 5.5 Planning

- [x] Trajectory optimization (direct collocation, shooting, multiple shooting)
- [ ] `trajectory_generation/` — polynomial / min-snap
- [ ] `policy_synthesis/` — DP, lookup table controller, policy evaluator, grid discretizer
- [ ] `search/rrt.py`
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
| Example scripts | 195 | 44 | **~151** |
| Course notebooks | 3 | 7 (new topics) | different mix |

### Port phases (from gap doc)

| Phase | Focus | Unblocks |
| --- | --- | --- |
| **A** | `Manipulator` abstraction + computed torque + sliding mode + `robotic.py` | ~50 robot/pendulum demos |
| **B** | DP/RRT redesign + polynomial traj gen + trajectory LQR | ~50 planning demos |
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

Full per-module and per-demo checklists:
[docs/plans/pyro-port-gap-todo.md](docs/plans/pyro-port-gap-todo.md).
