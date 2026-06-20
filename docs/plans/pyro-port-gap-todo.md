# Pyro → Minilink Port Gap Todo (Pyro 2.0)

Working checklist for finalizing minilink as the successor to
[SherbyRobotics/pyro](https://github.com/SherbyRobotics/pyro). Status snapshot:
**June 2026**.

Companion docs:

- [catalog-migration-notes.md](catalog-migration-notes.md) — plants (done)
- [tool-migration-notes.md](tool-migration-notes.md) — first tool tranche (partial)
- [neural-blocks-collection.md](neural-blocks-collection.md) — NN blocks (planned)

## Headline counts

| Bucket | Pyro | Minilink today | Gap |
| --- | ---: | ---: | ---: |
| Library `.py` modules under `pyro/` | 38 | — | see §2 |
| Example scripts | 195 | 44 | ~151 demos not yet ported |
| Course notebooks | 3 | 7 (new topics) | different mix |
| Catalog plant classes | 40+ | 40+ | **0 math gaps** (QA'd) |
| Control law modules | 5 files | 3 files | 2+ files missing |
| Planning modules | 6 files | 1 trajopt stack | 4+ areas missing |
| Analysis modules | 4 files | 5 tools | mostly ported; cost variants gap |
| Tools / interfaces | 2 files | 0 | 2 missing |

**Verdict:** catalog migration is complete; **tools, planning, control nonlinear/robot
laws, interfaces, and ~75% of pyro demos** remain before calling pyro 2.0 done.

---

## 1. Already ported (do not re-port)

### 1.1 Catalog plants (`pyro/dynamic/` → `minilink/dynamics/catalog/`)

All pyro plant EoMs are ported and term-by-term QA'd. See
[catalog-migration-notes.md](catalog-migration-notes.md).

- [x] Integrators (`Simple` / `Double` / `Triple`)
- [x] `VanderPol`
- [x] Pendulums (`Pendulum`, `InvertedPendulum`, `DoublePendulum`, `Acrobot`,
  `TwoIndependentPendulums`)
- [x] Cart-pole family (`CartPole`, `RotatingCartPole`, `UnderactuatedRotatingCartPole`,
  JAX twin)
- [x] Mass–spring–damper (`Single`…`Three`, floating variants)
- [x] `MountainCar`
- [x] Propulsion cars (slip + torque inputs)
- [x] Steering (`KinematicBicycle`, `KinematicCar`, `ConstantSpeedKinematicCar`,
  holonomic 2D/3D, `UdeSRacecar`)
- [x] `DynamicBicycle` (+ `LinearTire`, SL/JAX variants)
- [x] `QuarterCarOnRoughTerrain` (suspension)
- [x] Aerial (`Rocket`, `Drone2D`, side thruster, speed-controlled, helicopter tunnel)
- [x] `Plane2D`
- [x] `Boat2D`, `Boat2DWithCurrent`
- [x] Manipulators (`OneLink`…`FiveLink`, speed-controlled)

### 1.2 Blocks and routing (`pyro/planning/filters` partial)

- [x] `Sum`, `Gain`, `Mux`, `Demux` (`blocks/routing.py`)
- [x] `Saturation`, `DeadZone`, `Relay` (`blocks/nonlinear.py`)
- [x] `LowPassFilter`, `NotchFilter`, `Washout` (`blocks/filters.py`)
- [x] `TransferFunction` (`blocks/transfer_function.py`)
- [x] `Integrator` (`blocks/basic.py`)
- [x] `TrajectorySource` (replaces pyro `OpenLoopController`)
- [x] `NeuralNetwork` 1-hidden-layer MLP (`blocks/neural.py`) — **no pyro equivalent**

### 1.3 Linear control (`pyro/control/linear.py`, `lqr.py`)

- [x] `ProportionalController` (SISO + MIMO)
- [x] `PDController`
- [x] `PIDController` (continuous, no anti-windup in `control/linear.py`)
- [x] `FilteredPIDController` with anti-windup (`control/pid.py`) — **minilink extra**
- [x] `LinearStateFeedbackController` / `LinearFeedbackController`
- [x] `lqr_gain`, `lqr`, `lqr_at_operating_point` (`control/lqr.py`)

### 1.4 Analysis tools (`pyro/analysis/` + `pyro/dynamic/statespace.py`)

- [x] `linearize_matrices`, `linearize` → `LTISystem` (FD + JAX)
- [x] Controllability / observability (`analysis/structural.py`)
- [x] `find_equilibrium` (`analysis/equilibria.py`)
- [x] Modal analysis + animation (`analysis/modal.py`)
- [x] Selected-channel Bode (`analysis/frequency.py`)
- [x] Phase-plane plotting (`graphical/phase_plane/`) — basic `PhasePlot` parity

### 1.5 Simulation, trajectory, costs

- [x] `Trajectory` (`core/trajectory.py`)
- [x] `Simulator` + solvers (`simulation/`)
- [x] Quadratic / time costs (`core/costs.py`)
- [x] Diagram compile + NumPy/JAX evaluators (`core/compile/`)
- [x] Parametric tier `f_p`, `jacobian_f_params` (diagram + identification prototype)

### 1.6 Planning (partial)

- [x] Direct collocation trajopt (`planning/trajectory_optimization/`)
- [x] Multiple shooting, shooting, live plot hooks
- [x] MPC demos on dynamic bicycle (minilink **extra** — no pyro MPC library)

### 1.7 Graphics / kinematics (`pyro/kinematic/`, `pyro/analysis/graphical.py`)

- [x] 2D transforms, arrows, primitives (`graphical/animation/primitives.py`)
- [x] Matplotlib / Plotly / Meshcat / Pygame renderers
- [x] `animate()`, `plot_trajectory()`, diagram export
- [x] Interactive animation demo (`examples/scripts/animation/demo_interactive.py`)

### 1.8 Framework (no direct pyro module — minilink additions)

- [x] `DiagramSystem`, composition (`+`, `>>`, `@`, autowire)
- [x] `MathematicalProgram` + `Optimizer` (SciPy + Ipopt)
- [x] JAX identification prototype (`demo_params_gradient.py`)
- [x] Symbolic mechanics quarantine (`symbolic/mechanics/`)
- [x] Contact / ANCF tire engines (experimental, no pyro equivalent)

---

## 2. Missing library tools (pyro module → minilink home)

### 2.1 Control — high priority

| Pyro symbol | Proposed minilink home | Pyro demo refs (approx.) | Status |
| --- | --- | ---: | --- |
| `ComputedTorqueController` | `control/computed_torque.py` | 23 | [ ] |
| `SlidingModeController` | `control/sliding_mode.py` | 6 | [ ] |
| `TrajectoryLQRController` | `control/trajectory_lqr.py` | 3 + courses | [ ] |
| `JointPD`, `EndEffectorPD` | `control/robotic.py` | 15+ | [ ] |
| `JointPID`, `EndEffectorPID` | `control/robotic.py` | 10+ | [ ] |
| `EndEffectorKinematicController` | `control/robotic.py` | 8+ | [ ] |
| `EndEffectorKinematicControllerWithNullSpaceTask` | `control/robotic.py` | 3 + 1 notebook | [ ] |
| `linearize_and_synthesize_lqr_controller` | tool in `analysis/` (not `control/`) | several | [ ] workaround: two-step demo exists |
| `stable_baseline3_controller` | `interfaces/` or `control/` thin wrapper | 7+ | [ ] |
| NN policy blocks | `control/neural.py` + `blocks/neural/` | 0 in pyro | [ ] minilink-only |
| MPC controller factory | `control/mpc.py` | 0 in pyro | [ ] minilink-only |

**Deferred from first tranche** (already logged in tool-migration-notes):

- [ ] `Switch` routing block (selection semantics undecided)
- [ ] `RateLimiter`, `Hysteresis` stateful nonlinear blocks
- [ ] Plain `PIDController` anti-windup in `control/linear.py` (use `FilteredPIDController` today)

### 2.2 Planning — high priority

| Pyro symbol | Proposed minilink home | Pyro demo refs (approx.) | Status |
| --- | --- | ---: | --- |
| `RRT` | `planning/search/rrt.py` | 20 | [ ] redesign (stubs removed) |
| `DynamicProgramming` | `planning/policy_synthesis/dp.py` | 29 | [ ] redesign (stubs removed) |
| `LookUpTableController` | `planning/policy_synthesis/lookup_policy.py` | bundled with DP | [ ] |
| `GridDynamicSystem` | `planning/policy_synthesis/discretizer.py` | bundled with DP | [ ] |
| `PolicyEvaluator` (+ lookup variants) | `planning/policy_synthesis/policy_eval.py` | 4 | [ ] |
| `SingleAxisPolynomialTrajectoryGenerator` | `planning/trajectory_generation/polynomial.py` | 2 | [ ] |
| `MultiPointSingleAxisPolynomialTrajectoryGenerator` | same | 2 | [ ] |
| `TrajectoryFilter` (Butterworth `filtfilt`) | `planning/trajectory_generation/filter.py` or `analysis/` | 0 standalone | [ ] |
| `DirectCollocationTrajectoryOptimisation` | `planning/trajectory_optimization/` | 18 | [x] ported (JAX/Ipopt path) |

**Design note:** pyro DP/RRT operate on **continuous plants + discrete grids/trees**.
Minilink declared continuous-time simulation only; DP/RRT need an explicit
**offline/discrete planning** contract before port (see §4).

### 2.3 Estimation — medium priority

| Pyro symbol | Proposed minilink home | Pyro demo refs | Status |
| --- | --- | ---: | --- |
| `StateObserver` (Luenberger) | `estimation/luenberger.py` | 1 (`cartpole_LQG`) | [ ] |
| `ObservedSystem` | diagram composition recipe | 1 | [ ] |
| EKF | `estimation/ekf.py` (uses `analysis/linearize`) | 0 | [ ] |
| Kalman filter design | `estimation/kalman.py` | 0 | [ ] |
| Online RLS / adaptive | `estimation/recursive.py` | 5 example-local | [ ] |

### 2.4 Identification — medium priority

| Need | Proposed home | Pyro equivalent | Status |
| --- | --- | ---: | --- |
| Batch param fitting | `identification/fitting.py` | manual in examples | [ ] |
| Equation-error loss | `identification/fitting.py` | — | [ ] prototype in `demo_params_gradient.py` |
| Rollout / prediction-error loss | `identification/fitting.py` | RL/DP examples | [ ] |
| NN weight fitting | same API as physical params | — | [ ] minilink-only |

### 2.5 Interfaces — medium priority

| Pyro symbol | Proposed minilink home | Pyro demo refs | Status |
| --- | --- | ---: | --- |
| `Sys2Gym` | `interfaces/gymnasium.py` | 12+ RL | [ ] |
| `stable_baseline3_controller` | `interfaces/` + `control/` | 7 | [ ] |
| `InteractiveContinuousDynamicSystem` | defer / `graphical/` only | 10 games | [ ] see §4 |
| Flax / PyTorch model import | `interfaces/flax.py`, `torch.py` | 0 | [ ] minilink-only |
| FMI / cosimulation | `interfaces/` | 0 | [ ] |

### 2.6 Analysis — lower priority (extensions)

| Pyro symbol | Proposed minilink home | Status |
| --- | --- | --- |
| `QuadraticCostFunctionWithDomainCheck` | `core/costs.py` or `planning/problems.py` | [ ] |
| `Reachability` cost | same | [ ] |
| `QuadraticCostFunctionVectorized` | trajopt evaluator | [ ] partial via JAX |
| `PhasePlot3` | `graphical/phase_plane/` | [ ] deferred |
| `ss2tf` | `analysis/frequency.py` | [ ] |
| Pole-zero, Nyquist, margins | `analysis/frequency.py` | [ ] ROADMAP §5 |
| `linearize_and_lqr` one-shot tool | `analysis/` design tool | [ ] |

### 2.7 Catalog / plant gaps (not EoM — capability gaps)

| Pyro class | Issue | Proposed approach | Status |
| --- | --- | --- | --- |
| `*withObstacles` (4 classes) | collision checks in plant | engine or `planning/` collision, not per-plant | [ ] |
| `Pacejka` tire | magic formula | `dynamics/catalog/vehicles/` or tire submodule | [ ] deferred |
| `TireModel` abstraction | interface only | optional if Pacejka lands | [ ] |
| `StochasticSystemWrapper` | process noise | `estimation/` or port noise sources | [ ] |
| `NoiseSignal` | noise time series | extend `blocks/sources` or noise ports | [ ] partial (`demo_noise_ports`) |
| `MechanicalSystemWithPositionInputs` | mixed inputs | named ports per DESIGN (no subclass) | [x] by design |

### 2.8 Pyro API sugar — document, don't necessarily port

| Pyro pattern | Minilink replacement | Status |
| --- | --- | --- |
| `ctl + plant` → `ClosedLoopSystem` | `controller >> plant` or `DiagramSystem` | [x] different API |
| `ClosedLoopSystem.compute_trajectory` | `diagram.compute_trajectory` | [x] |
| `convert_to_gymnasium()` on plant | `interfaces/gymnasium.py` | [ ] |
| `convert_to_pygame()` on plant | `graphical/animation` + pygame renderer | [~] partial |

---

## 3. Missing demos — port checklist by technique

Use this as the **example port backlog**. A demo is "done" when an equivalent
`examples/scripts/` (or notebook) workflow exists in minilink using the new
framework APIs (diagrams, compile, tools) — not a line-by-line translation.

### 3.1 By control / planning technique

| Technique | Pyro scripts | Minilink coverage today | Todo |
| --- | ---: | --- | --- |
| LQR stabilization | ~16 | `cartpole_lqr_stabilization.py`, `demo_linearize.py` | [ ] port remaining plants (pendulum, MSD, drone, rocket, plane, double pendulum, acrobot) |
| PID / filtered PID | ~32 | `demo_filtered_pid_anti_windup.py`, MSD in catalog `__main__` | [ ] robot joint/effector PID demos |
| Computed torque | 23 | none | [ ] blocked on `control/computed_torque.py` |
| Sliding mode | 6 | none | [ ] blocked on `control/sliding_mode.py` |
| Trajectory LQR / stabilization | 3 | none | [ ] blocked on `control/trajectory_lqr.py` |
| Impedance control | 9 | none | [ ] example-local in pyro; needs `control/robotic.py` |
| Adaptive control | 5 | none | [ ] example-local; needs `estimation/recursive.py` or dedicated control |
| Value iteration / DP | 29 | none | [ ] blocked on policy synthesis redesign |
| RRT | 20 | none | [ ] blocked on `planning/search/rrt.py` |
| Trajectory optimization | ~18 | 5+ trajopt + MPC demos | [ ] port pendulum/cartpole/double-pendulum/acrobot/mountain-car/drone/plane car demos not yet covered |
| Differential flatness / min-snap | 4 | none | [ ] blocked on `planning/trajectory_generation/` |
| RL / PPO / Gym | 12 | `demo_neural_controller_jax.py` only | [ ] blocked on `interfaces/gymnasium.py` |
| Robot kinematic / nullspace | ~12 | none | [ ] blocked on `control/robotic.py` |
| Open-loop / traj replay | 3 | `TrajectorySource` | [ ] port explicit open-loop pendulum/double-pendulum demos |
| Transfer function / Bode | 2 | `demo_bode.py` | [ ] port `pendulum_frequency_response`, `mass_with_pid` |
| Policy evaluation / reachability | 4 | none | [ ] blocked on DP |
| Modal / eigenmodes | ~8 | `demo_modal.py` | [ ] port `*_modes.py`, `*_eigen_modes.py` per plant |
| LQG / observer | 1 | none | [ ] blocked on `estimation/luenberger.py` |
| Feedback linearization | 1 | none | [ ] low priority; specialized |
| Games / pygame / joystick | ~10 | `demo_interactive.py` | [ ] port or explicitly drop |
| Plant showcase only | ~25 | many catalog `__main__` + animation demos | [ ] audit which are still needed as scripts |
| Course / project bundles | ~30 | partial | [ ] lower priority than `demos_by_system` |

### 3.2 Pyro `demos_by_system/` — per-plant demo gaps

| Plant folder | Pyro scripts | Ported? |
| --- | ---: | --- |
| `pendulum_simple` | 14 | [ ] mostly missing (only catalog `__main__`) |
| `pendulum_double` | 14 | [ ] |
| `cartpole` | 5 | [~] `cartpole_lqr_stabilization.py` + trajopt |
| `cartpole_rotating` | 8 | [ ] |
| `mass_spring_damper` | 8 | [~] modal demo exists; PID/LQR per variant missing |
| `robot_arm_1dof` | 1 | [ ] |
| `robot_arm_2dof` | 9 | [ ] |
| `robot_arm_3dof` | 5 | [ ] |
| `robot_arm_5dof` | 4 | [ ] |
| `car_steering` | 9 | [~] bicycle path-tracking demos exist |
| `car_dynamic` | 3 | [~] `demo_dynamic_bicycle.py` |
| `car_propulsion` | 2 | [ ] |
| `mountain_car` | 1 | [ ] |
| `drone` | 3 | [ ] |
| `plane` | 2 | [ ] |
| `rocket` | 2 | [ ] |
| `boat` | 2 | [ ] |
| `holonomic_mobile_robot` | 3 | [ ] |
| `acrobot` | 2 | [ ] |
| `equations` | 6 | [~] partial via integrator/oscillator catalog |
| `suspension` | 1 | [ ] |

### 3.3 Minilink demos with no pyro counterpart (keep)

These are **not** gaps — they validate new framework capabilities:

- Diagram compiling, autowire, nested loops, noise ports
- MPC + rate-MPC on dynamic bicycle (6 scripts)
- JAX optimization / identification / neural controller
- Physics engine / ANCF tire / contact
- Symbolic quadruple pendulum
- Showcase / overview notebooks

---

## 4. Explicitly out of scope (do not port as-is)

Recorded minilink decisions that conflict with pyro features:

| Topic | Decision | Pyro surface |
| --- | --- | --- |
| Discrete-time framework | Continuous-time only | `GridDynamicSystem.dt`, DP time stepping |
| Digital control / ZOH / delays | Out of scope | — |
| RNNs / LSTM | Out of scope | — |
| Mixed-rate simulation | Out of scope | — |
| Differentiable rollouts (library) | Out of scope | demo-level JAX OK |
| Hybrid events | Out of scope | — |
| Pygame game framework | No first-class port | `sys2game`, `*game.py` |
| DP / RRT stubs | Removed; redesign first | `planning/dynamicprogramming.py`, `randomtree.py` |

**Action items for pyro 2.0 closure:**

- [ ] User sign-off: DP/RRT return as **offline planning tools** on continuous plants
- [ ] User sign-off: drop or replace **~10 game demos** with `demo_interactive` pattern
- [ ] User sign-off: **RL/Gym** trains outside; minilink only supplies `interfaces/gymnasium.py`

---

## 5. Recommended port phases (pyro 2.0 finish line)

### Phase A — Unblock the largest demo surfaces

1. [ ] `control/computed_torque.py`
2. [ ] `control/sliding_mode.py`
3. [ ] `control/robotic.py` (joint/effector PD, PID, kinematic, nullspace)
4. [ ] Port top robot demos: 2-link computed torque, joint PD, effector PD, kinematic
5. [ ] Port top pendulum demos: PID, computed torque, sliding mode, LQR

### Phase B — Planning beyond trajopt

6. [ ] Architectural review: DP + grid discretizer contract
7. [ ] `planning/policy_synthesis/` (DP, lookup policy, policy evaluator)
8. [ ] Architectural review: RRT state space + collision hook
9. [ ] `planning/search/rrt.py`
10. [ ] `planning/trajectory_generation/` (polynomial / min-snap)
11. [ ] `control/trajectory_lqr.py`
12. [ ] Port DP/RRT demo subset (pendulum, car, holonomic robot)

### Phase C — Estimation, identification, interfaces

13. [ ] `estimation/luenberger.py` + LQG diagram recipe
14. [ ] `estimation/kalman.py`
15. [ ] `identification/fitting.py` (unify physical + NN params)
16. [ ] `interfaces/gymnasium.py`
17. [ ] Port RL demo subset or document SB3-outside workflow

### Phase D — Analysis and frequency completion

18. [ ] `analysis/frequency.py`: pole-zero, Nyquist, margins, `ss2tf`
19. [ ] Reachability / domain-check costs for planning
20. [ ] Phase-plane 3D / Plotly follow-ups (if still wanted)

### Phase E — Catalog capability gaps

21. [ ] Obstacle collision layer (replace `*withObstacles` plants)
22. [ ] `Pacejka` tire (optional)
23. [ ] Stochastic forcing / noise source blocks

### Phase F — Demo audit and courses

24. [ ] Port remaining `demos_by_system/` showcase scripts (plant-only)
25. [ ] Port `demos_by_tool/transfer_functions/`
26. [ ] Decide fate of `examples/courses/` and `examples/projects/` (ugv, asimov, wcrt, adaptive)

### Phase G — Framework polish (pyro 2.0 release criteria)

27. [ ] Public export policy (`minilink/__init__.py`)
28. [ ] Compile evaluator API review (TRL 4 → 7)
29. [ ] Graphics kinematic composition refactor (ROADMAP review queue)
30. [ ] Document pyro → minilink API mapping (README migration guide)
31. [ ] TRL 8 demos for each major tool band
32. [ ] Full pytest coverage for ported tools

---

## 6. Quick reference — pyro file → minilink status

| Pyro module | Minilink | Status |
| --- | --- | --- |
| `dynamic/*` plants | `dynamics/catalog/` | **Done** |
| `dynamic/statespace.py` | `abstraction/state_space.py` + `analysis/linearize.py` | **Done** |
| `dynamic/stochastic.py` | — | **Missing** |
| `control/linear.py` | `control/linear.py`, `control/pid.py` | **Done** |
| `control/lqr.py` | `control/lqr.py` | **Done** (no traj LQR) |
| `control/nonlinear.py` | — | **Missing** |
| `control/robotcontrollers.py` | — | **Missing** |
| `control/reinforcementlearning.py` | — | **Missing** |
| `control/controller.py` | `core/diagram.py`, `core/composition.py` | **Replaced** (different API) |
| `planning/trajectoryoptimisation.py` | `planning/trajectory_optimization/` | **Done** |
| `planning/trajectorygeneration.py` | — | **Missing** |
| `planning/dynamicprogramming.py` | — | **Missing** (redesign) |
| `planning/randomtree.py` | — | **Missing** (redesign) |
| `planning/discretizer.py` | — | **Missing** |
| `planning/filters.py` | — | **Missing** |
| `planning/plan.py` | `blocks/sources.TrajectorySource` | **Done** |
| `analysis/simulation.py` | `simulation/`, `core/trajectory.py` | **Done** |
| `analysis/costfunction.py` | `core/costs.py` | **Partial** |
| `analysis/phaseanalysis.py` | `graphical/phase_plane/` | **Partial** |
| `analysis/graphical.py` | `graphical/` | **Done** (different structure) |
| `tools/sys2gym.py` | — | **Missing** |
| `tools/sys2game.py` | — | **Out of scope** |
| `kinematic/*` | `graphical/animation/primitives.py` | **Done** |

---

## 7. Definition of done — pyro 2.0

Treat pyro 2.0 (minilink) as **feature-complete** when:

1. Every **in-scope** pyro library module has a minilink home or a documented replacement.
2. Every **in-scope** pyro `demos_by_system/` plant has at least one representative
   closed-loop demo in `examples/scripts/`.
3. DP/RRT either land with approved architecture **or** are explicitly dropped with
   migrated alternatives documented.
4. Robot control suite (`computed_torque`, `sliding_mode`, `robotic`) reaches TRL ≥ 6.
5. Estimation seed (Luenberger) + identification `fitting.py` exist for the
   params-gradient workflow shown in pyro's parameter-ID examples.
6. README includes a **pyro migration guide** (API mapping table).

Until then, use this file as the master unchecked todo.
