# Pyro 2.0 ↔ Minilink parity audit

Snapshot vs [SherbyRobotics/pyro](https://github.com/SherbyRobotics/pyro) local checkout (2026-06-29).

Status legend: **Done** = equivalent landed · **Partial** = framework exists, gap noted · **TODO** = planned/in-scope · **Drop** = explicit non-goal

---

## Summary

| Bucket | Pyro | Done | Partial | TODO | Drop |
| --- | ---: | ---: | ---: | ---: | ---: |
| Library symbols (table below) | 104 | 79 | 10 | 11 | 4 |
| Example scripts | 195 | 25 | 45 | 90 | 35 |
| Minilink scripts | 60 | — | — | — | — |
| Minilink notebooks | 7 | — | — | — | — |

Release criteria (unchanged): every **in-scope** pyro library module has a minilink home or documented replacement; representative closed-loop demo per major plant family; README pyro → minilink migration guide.

---

## 1. Library modules — pyro → minilink

| Band | Pyro module | Pyro symbol | Minilink home | Minilink symbol | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Framework | `pyro/dynamic/system.py` | ContinuousDynamicSystem | `minilink/core/system.py` | DynamicSystem, System, DiagramSystem | **Done** | Richer diagram/compile layer |
| Framework | `pyro/dynamic/mechanical.py` | MechanicalSystem | `minilink/dynamics/abstraction/mechanical.py` | MechanicalSystem | **Done** |  |
| Framework | `pyro/dynamic/mechanical.py` | MechanicalSystemWithPositionInputs | `minilink/dynamics/abstraction/mechanical.py` | MechanicalSystemWithPositionInputs | **Done** |  |
| Framework | `pyro/dynamic/rigidbody.py` | GeneralizedMechanicalSystem | `minilink/dynamics/abstraction/generalized_mechanical.py` | GeneralizedMechanicalSystem | **Done** |  |
| Framework | `pyro/dynamic/rigidbody.py` | GeneralizedMechanicalSystemWithPositionInputs | `minilink/dynamics/abstraction/generalized_mechanical.py` | GeneralizedMechanicalSystemWithPositionInputs | **Done** |  |
| Framework | `pyro/dynamic/rigidbody.py` | RigidBody2D | `—` | — | **Drop** | Use GeneralizedMechanicalSystem or catalog plant |
| Framework | `pyro/dynamic/statespace.py` | StateSpaceSystem | `minilink/dynamics/abstraction/state_space.py` | LTISystem, StateSpaceSystem | **Done** |  |
| Framework | `pyro/dynamic/statespace.py` | linearize() | `minilink/analysis/analysis_linearize.py` | linearize() | **Done** |  |
| Framework | `pyro/dynamic/statespace.py` | StateObserver | `minilink/estimation/` | — | **TODO** | estimation/luenberger.py |
| Framework | `pyro/dynamic/statespace.py` | ObservedSystem | `minilink/estimation/` | — | **TODO** | LQG blocked on observers |
| Framework | `pyro/dynamic/stochastic.py` | NoiseSignal | `minilink/blocks/sources.py` | noise ports in diagrams | **Partial** | No StochasticSystemWrapper |
| Framework | `pyro/dynamic/stochastic.py` | StochasticSystemWrapper | `—` | — | **Drop** | Explicit non-goal unless reversed ([TODO.md](TODO.md) Later) |
| Framework | `pyro/dynamic/tranferfunction.py` | TransferFunction | `minilink/blocks/transfer_function.py` | TransferFunction | **Done** |  |
| Framework | `pyro/dynamic/tranferfunction.py` | ss2tf() | `minilink/analysis/` | — | **TODO** | Frequency backlog |
| Catalog | `pyro/dynamic/equation.py` | VanderPol | `minilink/dynamics/catalog/equations/oscillators.py` | VanderPol | **Done** |  |
| Catalog | `pyro/dynamic/integrator.py` | SimpleIntegrator | `minilink/dynamics/catalog/equations/integrators.py` | SimpleIntegrator | **Done** |  |
| Catalog | `pyro/dynamic/integrator.py` | DoubleIntegrator | `minilink/dynamics/catalog/equations/integrators.py` | DoubleIntegrator | **Done** |  |
| Catalog | `pyro/dynamic/integrator.py` | TripleIntegrator | `minilink/dynamics/catalog/equations/integrators.py` | TripleIntegrator | **Done** |  |
| Catalog | `pyro/dynamic/pendulum.py` | SinglePendulum | `minilink/dynamics/catalog/pendulum/pendulum.py` | Pendulum | **Done** | Renamed |
| Catalog | `pyro/dynamic/pendulum.py` | InvertedPendulum | `minilink/dynamics/catalog/pendulum/pendulum.py` | InvertedPendulum | **Done** |  |
| Catalog | `pyro/dynamic/pendulum.py` | DoublePendulum | `minilink/dynamics/catalog/pendulum/double_pendulum.py` | DoublePendulum | **Done** |  |
| Catalog | `pyro/dynamic/pendulum.py` | Acrobot | `minilink/dynamics/catalog/pendulum/double_pendulum.py` | Acrobot | **Done** |  |
| Catalog | `pyro/dynamic/pendulum.py` | TwoIndependentSinglePendulum | `minilink/dynamics/catalog/pendulum/pendulum.py` | TwoIndependentPendulums | **Done** |  |
| Catalog | `pyro/dynamic/cartpole.py` | CartPole | `minilink/dynamics/catalog/pendulum/cartpole.py` | CartPole (+ JaxCartPole) | **Done** |  |
| Catalog | `pyro/dynamic/cartpole.py` | RotatingCartPole | `minilink/dynamics/catalog/pendulum/cartpole.py` | RotatingCartPole | **Done** |  |
| Catalog | `pyro/dynamic/cartpole.py` | UnderActuatedRotatingCartPole | `minilink/dynamics/catalog/pendulum/cartpole.py` | UnderActuatedRotatingCartPole | **Done** |  |
| Catalog | `pyro/dynamic/massspringdamper.py` | SingleMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` | SingleMass | **Done** |  |
| Catalog | `pyro/dynamic/massspringdamper.py` | TwoMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` | TwoMass | **Done** |  |
| Catalog | `pyro/dynamic/massspringdamper.py` | ThreeMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` | ThreeMass | **Done** |  |
| Catalog | `pyro/dynamic/massspringdamper.py` | FloatingSingleMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` | FloatingSingleMass | **Done** |  |
| Catalog | `pyro/dynamic/massspringdamper.py` | FloatingTwoMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` | FloatingTwoMass | **Done** |  |
| Catalog | `pyro/dynamic/massspringdamper.py` | FloatingThreeMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` | FloatingThreeMass | **Done** |  |
| Catalog | `pyro/dynamic/mountaincar.py` | MountainCar | `minilink/dynamics/catalog/vehicles/mountain_car.py` | MountainCar | **Done** |  |
| Catalog | `pyro/dynamic/boat.py` | Boat2D | `minilink/dynamics/catalog/marine/boat.py` | Boat2D | **Done** |  |
| Catalog | `pyro/dynamic/boat.py` | Boat2DwithCurrent | `minilink/dynamics/catalog/marine/boat.py` | Boat2DWithCurrent | **Done** |  |
| Catalog | `pyro/dynamic/drone.py` | Drone2D | `minilink/dynamics/catalog/aerial/drone.py` | Drone2D | **Done** |  |
| Catalog | `pyro/dynamic/drone.py` | SpeedControlledDrone2D | `minilink/dynamics/catalog/aerial/drone.py` | SpeedControlledDrone2D | **Done** |  |
| Catalog | `pyro/dynamic/drone.py` | ConstantSpeedHelicopterTunnel | `minilink/dynamics/catalog/aerial/drone.py` | ConstantSpeedHelicopterTunnel | **Done** |  |
| Catalog | `pyro/dynamic/drone.py` | Drone2DwithSideTruster | `minilink/dynamics/catalog/aerial/drone.py` | Drone2DWithSideThruster | **Done** |  |
| Catalog | `pyro/dynamic/plane.py` | Plane2D | `minilink/dynamics/catalog/aerial/plane.py` | Plane2D | **Done** |  |
| Catalog | `pyro/dynamic/rocket.py` | Rocket | `minilink/dynamics/catalog/aerial/rocket.py` | Rocket | **Done** |  |
| Catalog | `pyro/dynamic/suspension.py` | QuarterCarOnRoughTerrain | `minilink/dynamics/catalog/vehicles/suspension.py` | QuarterCarOnRoughTerrain | **Done** |  |
| Catalog | `pyro/dynamic/vehicle_dynamic.py` | TireModel (ABC) | `minilink/dynamics/catalog/vehicles/dynamic_bicycle.py` | — | **Partial** | Only the linear tire implemented (pure functions) |
| Catalog | `pyro/dynamic/vehicle_dynamic.py` | LinearTire | `minilink/dynamics/catalog/vehicles/dynamic_bicycle.py` | `tire_slip` / `linear_tire_forces` (coefficients in plant `params`) | **Done** |  |
| Catalog | `pyro/dynamic/vehicle_dynamic.py` | Pacejka | `—` | — | **TODO** | [TODO.md](TODO.md) Later |
| Catalog | `pyro/dynamic/vehicle_dynamic.py` | DynamicBicycle | `minilink/dynamics/catalog/vehicles/dynamic_bicycle.py` | DynamicBicycle (+ JAX twins, rate inputs) | **Done** | Minilink adds rate-input variant |
| Catalog | `pyro/dynamic/vehicle_propulsion.py` | LongitudinalFrontWheelDriveCarWithWheelSlipInput | `minilink/dynamics/catalog/vehicles/propulsion.py` | same | **Done** |  |
| Catalog | `pyro/dynamic/vehicle_propulsion.py` | LongitudinalFrontWheelDriveCarWithTorqueInput | `minilink/dynamics/catalog/vehicles/propulsion.py` | same | **Done** |  |
| Catalog | `pyro/dynamic/vehicle_steering.py` | KinematicBicyleModel | `minilink/dynamics/catalog/vehicles/steering.py` | KinematicBicycle | **Done** |  |
| Catalog | `pyro/dynamic/vehicle_steering.py` | HolonomicMobileRobot | `minilink/dynamics/catalog/vehicles/steering.py` | HolonomicMobileRobot | **Done** |  |
| Catalog | `pyro/dynamic/vehicle_steering.py` | HolonomicMobileRobotwithObstacles | `minilink/planning/spatial/` | Scene + bind() | **Partial** | Scene replaces plant wrapper |
| Catalog | `pyro/dynamic/vehicle_steering.py` | Holonomic3DMobileRobot | `minilink/dynamics/catalog/vehicles/steering.py` | HolonomicMobileRobot3D | **Done** |  |
| Catalog | `pyro/dynamic/vehicle_steering.py` | Holonomic3DMobileRobotwithObstacles | `minilink/planning/spatial/` | Scene | **Partial** | Scene replaces plant wrapper |
| Catalog | `pyro/dynamic/vehicle_steering.py` | KinematicCarModel | `minilink/dynamics/catalog/vehicles/steering.py` | KinematicCar | **Done** |  |
| Catalog | `pyro/dynamic/vehicle_steering.py` | KinematicCarModelwithObstacles | `minilink/planning/spatial/` | Scene | **Partial** |  |
| Catalog | `pyro/dynamic/vehicle_steering.py` | ConstantSpeedKinematicCarModel | `minilink/dynamics/catalog/vehicles/steering.py` | ConstantSpeedKinematicCar | **Done** |  |
| Catalog | `pyro/dynamic/vehicle_steering.py` | UdeSRacecar | `minilink/dynamics/catalog/vehicles/steering.py` | UdeSRacecar | **Done** |  |
| Catalog | `pyro/dynamic/manipulator.py` | Manipulator | `minilink/dynamics/abstraction/manipulator.py` | Manipulator | **Done** | Catalog rebase on arms.py pending |
| Catalog | `pyro/dynamic/manipulator.py` | SpeedControlledManipulator | `minilink/dynamics/catalog/manipulators/arms.py` | SpeedControlledManipulator | **Done** |  |
| Catalog | `pyro/dynamic/manipulator.py` | OneLinkManipulator | `minilink/dynamics/catalog/manipulators/arms.py` | OneLinkManipulator | **Done** |  |
| Catalog | `pyro/dynamic/manipulator.py` | TwoLinkManipulator | `minilink/dynamics/catalog/manipulators/arms.py` | TwoLinkManipulator | **Done** |  |
| Catalog | `pyro/dynamic/manipulator.py` | ThreeLinkManipulator3D | `minilink/dynamics/catalog/manipulators/arms.py` | ThreeLinkManipulator3D | **Done** |  |
| Catalog | `pyro/dynamic/manipulator.py` | FiveLinkPlanarManipulator | `minilink/dynamics/catalog/manipulators/arms.py` | FiveLinkPlanarManipulator | **Done** |  |
| Catalog | `pyro/dynamic/manipulator.py` | TwoLinkManipulatorwithObstacles | `minilink/planning/spatial/` | Scene | **Partial** |  |
| Catalog | `pyro/dynamic/manipulator.py` | FiveLinkPlanarManipulatorwithObstacles | `minilink/planning/spatial/` | Scene | **Partial** |  |
| Control | `pyro/control/controller.py` | StaticController | `minilink/core/diagram.py` | DiagramSystem wiring | **Done** |  |
| Control | `pyro/control/controller.py` | ClosedLoopSystem | `minilink/core/diagram.py` | `@`, `>>`, `+` composition | **Done** |  |
| Control | `pyro/control/controller.py` | DynamicController | `minilink/core/diagram.py` | DiagramSystem | **Partial** | No separate wrapper class |
| Control | `pyro/control/controller.py` | DynamicClosedLoopSystem | `minilink/core/diagram.py` | DiagramSystem | **Partial** |  |
| Control | `pyro/control/linear.py` | ProportionalController | `minilink/control/linear.py` | PDController, ProportionalGain | **Done** |  |
| Control | `pyro/control/linear.py` | PIDController | `minilink/control/pid.py` | PIDController | **Done** |  |
| Control | `pyro/control/lqr.py` | synthesize_lqr_controller | `minilink/control/lqr.py` | synthesize_lqr | **Done** |  |
| Control | `pyro/control/lqr.py` | linearize_and_synthesize_lqr_controller | `minilink/control/lqr.py` | linearize + synthesize_lqr | **Done** |  |
| Control | `pyro/control/lqr.py` | TrajectoryLQRController | `minilink/control/lqr.py` | — | **TODO** | Trajectory stabilization demos |
| Control | `pyro/control/nonlinear.py` | ComputedTorqueController | `minilink/control/modelbased.py` | ComputedTorqueController | **Done** |  |
| Control | `pyro/control/nonlinear.py` | SlidingModeController | `minilink/control/modelbased.py` | SlidingModeController | **Done** | Pyro `K(q) sign(s)` law; traj-following demos TODO |
| Control | `pyro/control/robotcontrollers.py` | JointPD, EndEffectorPD, … | `minilink/control/robotic.py` | JointImpedance, TaskImpedance, TaskKinematic, TaskKinematicNullspace | **Partial** | Dynamic joint/effector PID wrappers TODO |
| Control | `pyro/control/reinforcementlearning.py` | stable_baseline3_controller | `minilink/interfaces/gymnasium.py` | SB3Controller | **Done** | Duck-typed on `model.predict`; train outside |
| Analysis | `pyro/analysis/simulation.py` | Trajectory | `minilink/core/trajectory.py` | Trajectory | **Done** |  |
| Analysis | `pyro/analysis/simulation.py` | Simulator | `minilink/simulation/simulator.py` | Simulator | **Done** |  |
| Analysis | `pyro/analysis/simulation.py` | CLosedLoopSimulator | `minilink/core/facades.py` | compute_trajectory() | **Done** |  |
| Analysis | `pyro/analysis/simulation.py` | DynamicCLosedLoopSimulator | `minilink/simulation/simulator.py` | Simulator | **Done** |  |
| Analysis | `pyro/analysis/costfunction.py` | CostFunction, QuadraticCostFunction, … | `minilink/core/costs.py` | CostFunction, QuadraticCost, … | **Done** |  |
| Analysis | `pyro/analysis/costfunction.py` | Reachability | `minilink/core/costs.py` | ReachabilityCost | **Done** |  |
| Analysis | `pyro/analysis/graphical.py` | TrajectoryPlotter | `minilink/graphical/` | plot_trajectory() | **Done** |  |
| Analysis | `pyro/analysis/graphical.py` | Animator | `minilink/graphical/animation/` | animate() | **Done** | Multi-renderer |
| Analysis | `pyro/analysis/phaseanalysis.py` | PhasePlot, PhasePlot3 | `minilink/graphical/phase_plane/` | plot_phase_plane() | **Done** |  |
| Planning | `pyro/planning/discretizer.py` | GridDynamicSystem | `minilink/planning/policy_synthesis/discretizer.py` | StateSpaceGrid | **Done** | Continuous PlanningProblem |
| Planning | `pyro/planning/dynamicprogramming.py` | DynamicProgramming | `minilink/planning/policy_synthesis/dp.py` | DynamicProgrammingPlanner | **Done** | loop / numpy / jax |
| Planning | `pyro/planning/dynamicprogramming.py` | DynamicProgrammingWithLookUpTable | `minilink/planning/policy_synthesis/dp.py` | same planner + lookup | **Done** |  |
| Planning | `pyro/planning/dynamicprogramming.py` | DynamicProgramming2DRectBivariateSpline | `minilink/planning/policy_synthesis/dp.py` | — | **Drop** | Not needed; grid backends cover use cases |
| Planning | `pyro/planning/dynamicprogramming.py` | LookUpTableController | `minilink/planning/policy_synthesis/lookup_policy.py` | LookupTableController | **Done** |  |
| Planning | `pyro/planning/dynamicprogramming.py` | PolicyEvaluator | `minilink/planning/policy_synthesis/policy_eval.py` | PolicyEvaluator | **Done** |  |
| Planning | `pyro/planning/filters.py` | TrajectoryFilter | `minilink/planning/` | — | **TODO** | Butterworth filtfilt post-filter |
| Planning | `pyro/planning/plan.py` | OpenLoopController | `minilink/blocks/sources.py` | TrajectorySource | **Partial** |  |
| Planning | `pyro/planning/plan.py` | Planner | `minilink/planning/planner.py` | Planner | **Done** |  |
| Planning | `pyro/planning/randomtree.py` | RRT | `minilink/planning/search/` | RRTPlanner, RRTStarPlanner | **Done** | Extended API + Dubins |
| Planning | `pyro/planning/trajectorygeneration.py` | SingleAxisPolynomialTrajectoryGenerator | `minilink/planning/` | — | **TODO** | trajectory_generation/ |
| Planning | `pyro/planning/trajectorygeneration.py` | MultiPointSingleAxisPolynomialTrajectoryGenerator | `minilink/planning/` | — | **TODO** | min-snap / differential flatness |
| Planning | `pyro/planning/trajectoryoptimisation.py` | DirectCollocationTrajectoryOptimisation | `minilink/planning/trajectory_optimization/` | DirectCollocation, Shooting, MS | **Done** | Ipopt/JAX backends |
| Graphics | `pyro/kinematic/geometry.py` | transformation_matrix_2D | `minilink/core/kinematics.py` | frame tf helpers | **Done** |  |
| Graphics | `pyro/kinematic/drawing.py` | transform_points_2D, arrows | `minilink/graphical/animation/` | primitives, drawables | **Done** |  |
| Tools | `pyro/tools/sys2game.py` | InteractiveContinuousDynamicSystem | `minilink/simulation/realtime/` | superseded by `RealtimeSimulator` + `PygameInput` | **Drop** | Own realtime tool, not a port |
| Tools | `pyro/tools/sys2gym.py` | Sys2Gym | `minilink/interfaces/gymnasium.py` | Sys2Gym | **Done** | Cost passed explicitly; `rl` extra |

---

## 2. Minilink-only (no pyro equivalent)

| Minilink | What | Notes |
| --- | --- | --- |
| `minilink/core/compile/` | ExecutionPlan, NumPy/JAX evaluators | Compile band; pyro has no separate compile layer |
| `minilink/core/geometry.py` | SDF shapes, cost algebra | Spatial planning primitives |
| `minilink/optimization/` | MathematicalProgram, Optimizer | General NLP; pyro trajopt is narrower |
| `minilink/planning/spatial/` | Scene, WorkspaceField, RobotBody | Obstacle/clearance layer replaces *withObstacles plants |
| `minilink/planning/search/dubins.py` | Dubins steering | Extra beyond pyro RRT |
| `minilink/blocks/neural.py` | MLP block (JAX) | Prototype; pyro RL is SB3-only |
| `minilink/symbolic/` | Symbolic derivation | Quarantine; no pyro equivalent |
| `minilink/dynamics/engines/` | Contact, ANCF tire (JAX) | Experimental physics engines |
| `examples/demos/mpc/` | Rate MPC closed-loop demos | Minilink extra; no pyro MPC module |

---

## 3. Example scripts — full inventory

All 195 pyro scripts under `examples/`, grouped by top-level folder.


### 3.1 `courses/`

| Pyro script | Status | Minilink equivalent / notes |
| --- | --- | --- |
| `courses/corom_impedance_control/corom_robots.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/corom_impedance_control/custom_drilling_controller.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/corom_impedance_control/custom_drilling_controller_test.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/corom_impedance_control/twolinkrobot_effector_impedance_controller.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/corom_impedance_control/twolinkrobot_joint_impedance_controller.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gmc714/demo_simple_pendulum_multiple_controller_options.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gmc714/manipulator_dynamic_terms.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro501/float_mass_pids/one_mass_with_pid.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro501/float_mass_pids/three_mass_with_pid.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro501/float_mass_pids/two_mass_with_pid.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro501/mass-spring-dampers/one_mass_with_pid.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro501/mass-spring-dampers/three_mass_with_pid.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro501/mass-spring-dampers/two_mass_with_pid.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro501/phaseplane/demo.py` | **Done** | examples/demos/plots/plot_phase_plane.py |
| `courses/udes_gro640/prob/abcd1234.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/demo_crash_commande_en_position.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/gro640_robots.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/test_commande_en_force.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/test_commande_en_position.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/test_f.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/test_trajectoire_3D.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro860/dp_demo_swingup.py` | **Done** | examples/demos/planning/value_iteration/vi_pendulum.py |
| `courses/udes_gro860/dp_mass_min_time_optimal.py` | **Done** | examples/demos/planning/value_iteration/vi_basics.py |
| `courses/udes_gro860/dp_mass_min_time_policy_evaluation.py` | **Partial** | Core tool demos exist; course variant not ported |
| `courses/udes_gro860/lqr_cartpole_stab.py` | **Done** | examples/demos/statespace/cartpole_lqr.py |
| `courses/udes_gro860/lqr_cartpole_traj.py` | **Partial** | Core tool demos exist; course variant not ported |
| `courses/udes_gro860/rl_drone_demo_learning2fly.py` | **Done** | examples/learn/teaching/courses/gro860/labs/drone_ppo_learn_to_fly.ipynb |
| `courses/udes_gro860/rl_drone_demo_training.py` | **Done** | same GRO860 drone PPO notebook |
| `courses/udes_gro860/rl_pendulum_swingup_basic_bangbang.py` | **Done** | examples/learn/teaching/courses/gro860/labs/pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb |
| `courses/udes_gro860/rl_pendulum_swingup_test.py` | **Done** | same GRO860 pendulum swing-up VI vs LQR vs PPO notebook |

### 3.2 `demos_by_system/`

| Pyro script | Status | Minilink equivalent / notes |
| --- | --- | --- |
| `demos_by_system/acrobot/acrobot_with_lqr.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/acrobot/acrobot_with_trajectory_optimization.py` | **Partial** | Trajopt done; plant-specific demo TODO |
| `demos_by_system/boat/boat_position_controller.py` | **TODO** | Representative closed-loop for boat |
| `demos_by_system/boat/boat_with_ppo_demo.py` | **TODO** | Representative closed-loop for boat |
| `demos_by_system/car_dynamic/demo_oversteer.py` | **TODO** | Representative closed-loop for car_dynamic |
| `demos_by_system/car_dynamic/demo_understeer.py` | **TODO** | Representative closed-loop for car_dynamic |
| `demos_by_system/car_dynamic/dynamic_bicycle.py` | **Done** | examples/demos/animation/game_bicycle.py |
| `demos_by_system/car_propulsion/longitudinal_car_braking_value_iteration.py` | **Partial** | DP done; plant-specific demo TODO |
| `demos_by_system/car_propulsion/longitudinal_car_with_torque_input.py` | **TODO** | Representative closed-loop for car_propulsion |
| `demos_by_system/car_steering/bicycle.py` | **TODO** | Representative closed-loop for car_steering |
| `demos_by_system/car_steering/bicycle_exploration_with_rrt.py` | **Partial** | RRT done; plant-specific demo TODO |
| `demos_by_system/car_steering/bicycle_parallel_parking_with_rrt.py` | **Partial** | examples/demos/planning/rrt/rrt_car_parking.py |
| `demos_by_system/car_steering/car.py` | **TODO** | Representative closed-loop for car_steering |
| `demos_by_system/car_steering/car_trajectory_optimisation.py` | **Done** | examples/projects/car_trajopt/car_trajopt_*.py |
| `demos_by_system/car_steering/car_trajectory_with_rrt.py` | **Partial** | RRT done; plant-specific demo TODO |
| `demos_by_system/car_steering/car_with_custom_lateral_controller.py` | **TODO** | Representative closed-loop for car_steering |
| `demos_by_system/car_steering/car_with_valueiteration_minimum_time.py` | **Partial** | DP done; plant-specific demo TODO |
| `demos_by_system/car_steering/car_with_valueiteration_quadratic_cost.py` | **Partial** | DP done; plant-specific demo TODO |
| `demos_by_system/cartpole/cartpole_LQG.py` | **TODO** | estimation/kalman.py |
| `demos_by_system/cartpole/cartpole_demo.py` | **TODO** | Open-loop cartpole showcase |
| `demos_by_system/cartpole/cartpole_stabilization.py` | **Partial** | examples/demos/statespace/cartpole_lqr.py |
| `demos_by_system/cartpole/cartpole_with_lqr.py` | **Done** | examples/demos/statespace/cartpole_lqr.py |
| `demos_by_system/cartpole/cartpole_with_trajectory_optimization.py` | **Partial** | trajopt framework; cartpole-specific demo TODO |
| `demos_by_system/cartpole_rotating/cartpole_modes.py` | **Partial** | Use examples/demos/analysis/analysis_modal.py + open-loop sim |
| `demos_by_system/cartpole_rotating/cartpole_natural_behavior.py` | **TODO** | Representative closed-loop for cartpole_rotating |
| `demos_by_system/cartpole_rotating/cartpole_swingup_trajectory_optimisation.py` | **Partial** | Trajopt done; plant-specific demo TODO |
| `demos_by_system/cartpole_rotating/cartpole_with_computed_torque.py` | **TODO** | control/modelbased.py |
| `demos_by_system/cartpole_rotating/cartpole_with_rrt_and_computed_torque.py` | **TODO** | control/modelbased.py |
| `demos_by_system/cartpole_rotating/underactuated_cartpole_swingup.py` | **TODO** | Representative closed-loop for cartpole_rotating |
| `demos_by_system/cartpole_rotating/underactuated_cartpole_with_partialfeedbacklinearization.py` | **TODO** | control/modelbased.py |
| `demos_by_system/cartpole_rotating/underactuated_cartpole_with_rrt.py` | **Partial** | RRT done; plant-specific demo TODO |
| `demos_by_system/cartpole_rotating/undercartpole_with_rrt_and_direct_colocation.py` | **Partial** | RRT done; plant-specific demo TODO |
| `demos_by_system/drone/planar_drone_trajectory_optimisation.py` | **Partial** | Trajopt done; plant-specific demo TODO |
| `demos_by_system/drone/planar_drone_trajectory_optimisation_linearized.py` | **Partial** | Trajopt done; plant-specific demo TODO |
| `demos_by_system/drone/planar_drone_with_lqr.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/equations/double_integrator.py` | **TODO** | Representative closed-loop for equations |
| `demos_by_system/equations/double_integrator_optimal_controller.py` | **TODO** | Representative closed-loop for equations |
| `demos_by_system/equations/integrators_with_closed_loops.py` | **TODO** | Representative closed-loop for equations |
| `demos_by_system/equations/simple_integrator.py` | **TODO** | Representative closed-loop for equations |
| `demos_by_system/equations/triple_integrator.py` | **TODO** | Representative closed-loop for equations |
| `demos_by_system/equations/vanderpol.py` | **TODO** | Representative closed-loop for equations |
| `demos_by_system/holonomic_mobile_robot/holonomic_mobile_robot_exploration_with_obstacles_with_rrt.py` | **Done** | examples/demos/planning/rrt/rrt_holonomic_obstacles.py |
| `demos_by_system/holonomic_mobile_robot/holonomic_mobile_robot_exploration_with_rrt.py` | **Partial** | RRT done; plant-specific demo TODO |
| `demos_by_system/holonomic_mobile_robot/holonomic_mobile_robot_with_valueiteration.py` | **Partial** | DP done; plant-specific demo TODO |
| `demos_by_system/mass_spring_damper/single_mass_with_pid.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/mass_spring_damper/three_mass_dynamic.py` | **TODO** | Representative closed-loop for mass_spring_damper |
| `demos_by_system/mass_spring_damper/three_mass_eigen_modes.py` | **Partial** | Use examples/demos/analysis/analysis_modal.py + open-loop sim |
| `demos_by_system/mass_spring_damper/three_mass_with_lqr.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/mass_spring_damper/three_mass_with_pid.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/mass_spring_damper/two_mass_dynamic.py` | **TODO** | Representative closed-loop for mass_spring_damper |
| `demos_by_system/mass_spring_damper/two_mass_eigen_modes.py` | **Partial** | Use examples/demos/analysis/analysis_modal.py + open-loop sim |
| `demos_by_system/mass_spring_damper/two_mass_with_lqr.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/mass_spring_damper/two_mass_with_pid.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/mountain_car/mountain_car_with_valueiteration_quadratic.py` | **Partial** | DP done; plant-specific demo TODO |
| `demos_by_system/pendulum_double/double_pendulum.py` | **TODO** | Representative closed-loop for pendulum_double |
| `demos_by_system/pendulum_double/double_pendulum_game.py` | **Drop** | sys2game interactive game |
| `demos_by_system/pendulum_double/double_pendulum_modes.py` | **Partial** | Use examples/demos/analysis/analysis_modal.py + open-loop sim |
| `demos_by_system/pendulum_double/double_pendulum_with_computed_torque.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_double/double_pendulum_with_computed_torque_and_sinus_ref.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_double/double_pendulum_with_lqr.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/pendulum_double/double_pendulum_with_rrt.py` | **Partial** | RRT done; plant-specific demo TODO |
| `demos_by_system/pendulum_double/double_pendulum_with_rrt_and_computed_torque.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_double/double_pendulum_with_sliding_mode.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_double/double_pendulum_with_trajectory_following_computed_torque.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_double/double_pendulum_with_trajectory_following_open_loop_controller.py` | **TODO** | Representative closed-loop for pendulum_double |
| `demos_by_system/pendulum_double/double_pendulum_with_trajectory_following_sliding_mode_controller.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_double/double_pendulum_with_trajectory_optimization.py` | **Partial** | Trajopt done; plant-specific demo TODO |
| `demos_by_system/pendulum_double/double_pendulum_with_trajectory_optimization_and_computed_torque.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_simple/pendulum_game.py` | **Drop** | sys2game interactive game |
| `demos_by_system/pendulum_simple/simple_pendulum.py` | **TODO** | Representative closed-loop for pendulum_simple |
| `demos_by_system/pendulum_simple/simple_pendulum_custom_parameters.py` | **TODO** | Representative closed-loop for pendulum_simple |
| `demos_by_system/pendulum_simple/simple_pendulum_modes.py` | **Partial** | Use examples/demos/analysis/analysis_modal.py + open-loop sim |
| `demos_by_system/pendulum_simple/simple_pendulum_trajectory_optimization.py` | **Partial** | Trajopt done; plant-specific demo TODO |
| `demos_by_system/pendulum_simple/simple_pendulum_with_computed_torque.py` | **Done** | examples/demos/control/computed_torque_pendulum.py |
| `demos_by_system/pendulum_simple/simple_pendulum_with_lqr.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/pendulum_simple/simple_pendulum_with_open_loop_controller.py` | **TODO** | Representative closed-loop for pendulum_simple |
| `demos_by_system/pendulum_simple/simple_pendulum_with_pid.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/pendulum_simple/simple_pendulum_with_rrt.py` | **Done** | examples/demos/planning/rrt/rrt_pendulum_swingup.py |
| `demos_by_system/pendulum_simple/simple_pendulum_with_sliding_mode_controller.py` | **Done** | examples/demos/control/sliding_mode_pendulum.py · hybrid: examples/demos/hybrid/smc_pendulum_rate.py |
| `demos_by_system/pendulum_simple/simple_pendulum_with_trajectory_following_computed_torque.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_simple/simple_pendulum_with_trajectory_following_sliding_mode_controller.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_simple/simple_pendulum_with_valueiteration_minimum_time.py` | **Partial** | examples/demos/planning/value_iteration/vi_basics.py (mass) + examples/demos/planning/value_iteration/vi_pendulum.py |
| `demos_by_system/pendulum_simple/simple_pendulum_with_valueiteration_quadratic.py` | **Done** | examples/demos/planning/value_iteration/vi_pendulum.py |
| `demos_by_system/plane/plane_cobra.py` | **TODO** | Representative closed-loop for plane |
| `demos_by_system/plane/plane_simple_controller.py` | **TODO** | Representative closed-loop for plane |
| `demos_by_system/robot_arm_1dof/onelinkrobot_joint_impedance_controller.py` | **Done** | examples/demos/robotic/joint_impedance_two_link.py (covers 1-dof pattern) |
| `demos_by_system/robot_arm_2dof/twolinkrobot_computed_torque_controller.py` | **Done** | examples/demos/robotic/computed_torque_two_link.py |
| `demos_by_system/robot_arm_2dof/twolinkrobot_effector_impedance_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_2dof/twolinkrobot_effector_pid_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_2dof/twolinkrobot_joint_impedance_controller.py` | **Done** | examples/demos/robotic/joint_impedance_two_link.py |
| `demos_by_system/robot_arm_2dof/twolinkrobot_joint_pid_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_2dof/twolinkrobot_kinematic_controller.py` | **Done** | examples/demos/robotic/kinematic_two_link.py |
| `demos_by_system/robot_arm_2dof/twolinkrobot_kinematic_vs_dynamic_openloop.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_2dof/twolinkrobot_quasi_static_controllers.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_2dof/twolinkrobot_sliding_mode_controller.py` | **TODO** | control/modelbased.py |
| `demos_by_system/robot_arm_2dof/twolinkrobot_with_obstacles_path_planning.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_3dof/threelinkrobot_computed_torque_controller.py` | **TODO** | control/modelbased.py |
| `demos_by_system/robot_arm_3dof/threelinkrobot_effector_impedance_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_3dof/threelinkrobot_effector_pid_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_3dof/threelinkrobot_joint_impedance_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_3dof/threelinkrobot_kinematic_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_5dof/fivelinkrobot_kinematic_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_5dof/fivelinkrobot_kinematic_nullspace_controller.py` | **Done** | examples/demos/robotic/kinematic_nullspace_five_link.py |
| `demos_by_system/robot_arm_5dof/fivelinkrobot_with_obstacles_load_plan.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_5dof/fivelinkrobot_with_obstacles_path_planning.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/rocket/rocket_game.py` | **Drop** | sys2game interactive game |
| `demos_by_system/rocket/rocket_landing_with_lqr.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/suspension/suspension.py` | **TODO** | Representative closed-loop for suspension |

### 3.3 `demos_by_tool/`

| Pyro script | Status | Minilink equivalent / notes |
| --- | --- | --- |
| `demos_by_tool/differentialflatness/droneminsnap.py` | **TODO** | trajectory_generation/ |
| `demos_by_tool/differentialflatness/droneminsnap_waypoints.py` | **TODO** | trajectory_generation/ |
| `demos_by_tool/differentialflatness/rigidbody.py` | **Drop** | RigidBody2D not ported |
| `demos_by_tool/differentialflatness/rigidbody_waypoints.py` | **Drop** | RigidBody2D not ported |
| `demos_by_tool/dynamicprogramming/2D_navigation.py` | **TODO** | DP specialty demo not ported |
| `demos_by_tool/dynamicprogramming/active_suspension.py` | **TODO** | DP specialty demo not ported |
| `demos_by_tool/dynamicprogramming/braking_reachability.py` | **TODO** | DP specialty demo not ported |
| `demos_by_tool/dynamicprogramming/car_braking.py` | **TODO** | DP specialty demo not ported |
| `demos_by_tool/dynamicprogramming/rrt_car_parking.py` | **Done** | examples/demos/planning/rrt/rrt_car_parking.py |
| `demos_by_tool/dynamicprogramming/double_pendulum_optimal_swingup.py` | **Done** | examples/demos/planning/value_iteration/vi_double_pendulum_jax.py |
| `demos_by_tool/dynamicprogramming/double_pendulum_optimal_swingup_load.py` | **Partial** | examples/demos/planning/value_iteration/vi_double_pendulum_jax.py |
| `demos_by_tool/dynamicprogramming/float_mass_dp_optimal_controller.py` | **Done** | examples/demos/planning/value_iteration/vi_basics.py |
| `demos_by_tool/dynamicprogramming/helicopter_tunnel.py` | **TODO** | DP specialty demo not ported |
| `demos_by_tool/dynamicprogramming/pendulum_optimal_swingup.py` | **Done** | examples/demos/planning/value_iteration/vi_pendulum.py |
| `demos_by_tool/dynamicprogramming/pendulum_optimal_swingup_demo.py` | **Done** | examples/demos/planning/value_iteration/vi_pendulum.py |
| `demos_by_tool/dynamicprogramming/pendulum_optimal_swingup_low_def_fast_computation.py` | **Partial** | examples/demos/planning/value_iteration/vi_pendulum.py (grid tuning differs) |
| `demos_by_tool/dynamicprogramming/pendulum_reachability.py` | **Partial** | ReachabilityCost exists; demo TODO |
| `demos_by_tool/dynamicprogramming/policy_evaluator_with_computed_torque.py` | **TODO** | control/modelbased.py |
| `demos_by_tool/lqr_vs_valueiteration_for_a_simple_pendulum.py` | **Done** | examples/demos/planning/value_iteration/vi_pendulum.py Part 2 |
| `demos_by_tool/optimal_control_demo.py` | **TODO** | Not yet audited |
| `demos_by_tool/rl_with_stable_baseline3/double_pendulum_with_ppo.py` | **Drop** | External SB3 training |
| `demos_by_tool/rl_with_stable_baseline3/drone_with_ppo.py` | **Drop** | External SB3 training |
| `demos_by_tool/rl_with_stable_baseline3/pendulum_dp_vs_ppo_bangbang.py` | **Drop** | External SB3 training |
| `demos_by_tool/rl_with_stable_baseline3/pendulum_dp_vs_ppo_pump.py` | **Drop** | External SB3 training |
| `demos_by_tool/rl_with_stable_baseline3/pendulum_dp_vs_ppo_tmotor_bangbang.py` | **Drop** | External SB3 training |
| `demos_by_tool/rl_with_stable_baseline3/pendulum_with_PPO_baseline_gym_example.py` | **Drop** | External SB3 training |
| `demos_by_tool/rl_with_stable_baseline3/pendulum_with_PPO_baseline_pyro_reproduction.py` | **Drop** | External SB3 training |
| `demos_by_tool/trajectory_planning/double_pendulum_with_trajectory_optimization.py` | **Partial** | Trajopt done; plant-specific demo TODO |
| `demos_by_tool/trajectory_planning/mountain_car_trajectory_optimization.py` | **Partial** | Trajopt done; plant-specific demo TODO |
| `demos_by_tool/trajectory_stabilization/cartpole_swing_up_with_lqr_stabilization.py` | **TODO** | TrajectoryLQRController |
| `demos_by_tool/trajectory_stabilization/double_pendulum_with_trajectory_following_lqr_controller.py` | **TODO** | TrajectoryLQRController |
| `demos_by_tool/trajectory_stabilization/pendulum_swing_up_with_lqr_stabilization.py` | **TODO** | TrajectoryLQRController |
| `demos_by_tool/transfer_functions/mass_with_pid.py` | **Partial** | examples/demos/control/filtered_pid_anti_windup.py |
| `demos_by_tool/transfer_functions/pendulum_frequency_response.py` | **Done** | examples/demos/analysis/analysis_bode.py |

### 3.4 `projects/`

| Pyro script | Status | Minilink equivalent / notes |
| --- | --- | --- |
| `projects/adaptive_controllers/adaptive_computed_torque.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/adaptive_controllers/double_pendulum_with_adaptative.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/adaptive_controllers/simple_pendulum_with_adaptative.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/adaptive_controllers/simple_pendulum_with_adaptative_traj_following.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/asimov/asimov.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/asimov/asimov_computed_torque_controller.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/asimov/asimov_endeffector_pid_controller.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/asimov/asimov_joint_pid_controller.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/asimov/asimov_kinematic_controller.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/pygame/boat_game.py` | **Drop** | sys2game; use `simulation/realtime/` or skip |
| `projects/pygame/double_pendulum_game.py` | **Drop** | sys2game; use `simulation/realtime/` or skip |
| `projects/pygame/test_double_pendulum_joy.py` | **Drop** | sys2game; use `simulation/realtime/` or skip |
| `projects/pygame/test_pendulum_joy.py` | **Drop** | sys2game; use `simulation/realtime/` or skip |
| `projects/tmotor_robot/tmotor_robot_controller_simulation_tests.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/ugv/ugv_backup.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/ugv/ugv_dubins.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/ugv/ugv_gemini.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/ugv/ugv_map.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/ugv/ugv_model.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/ugv/ugv_planner.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/wcrt/WCRT_with_adaptative.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/wcrt/wcrt.py` | **Drop** | Research/course project; out of library demo scope |
| `projects/wcrt/wcrt_with_computed_torque.py` | **Drop** | Research/course project; out of library demo scope |

---

## 4. Minilink examples (reference)

These are minilink-native; they may cover pyro workflows without 1:1 filename parity.

| Script | Topic |
| --- | --- |
| `examples/demos/analysis/analysis_bode.py` | analysis |
| `examples/demos/analysis/analysis_equilibrium.py` | analysis |
| `examples/demos/analysis/analysis_linearize.py` | analysis |
| `examples/demos/analysis/analysis_linearize_fd_vs_jax.py` | analysis |
| `examples/demos/analysis/analysis_modal.py` | analysis |
| `examples/demos/analysis/analysis_structural.py` | analysis |
| `examples/demos/analysis/analysis_discretize.py` | analysis |
| `examples/demos/animation/animation_backends.py` | animation |
| `examples/learn/intro/10_graphical.ipynb` | animation |
| `examples/demos/animation/game_bicycle.py` | animation |
| `examples/demos/animation/animation_native_comparison.py` | animation |
| `examples/demos/blocks/signal_blocks.py` | blocks |
| `examples/demos/blocks/blocks_sources.py` | blocks |
| `examples/demos/control/computed_torque_pendulum.py` | control |
| `examples/demos/control/filtered_pid_anti_windup.py` | control |
| `examples/demos/control/neural_controller_jax.py` | control |
| `examples/demos/control/pid_autotuning_jax.py` | control |
| `examples/demos/control/sliding_mode_pendulum.py` | control |
| `examples/demos/diagrams/diagram_advanced_autowire.py` | diagrams |
| `examples/demos/diagrams/diagram_closed_loop.py` | diagrams |
| `examples/demos/diagrams/diagram_compiling.py` | diagrams |
| `examples/demos/diagrams/diagram_shortcuts.py` | diagrams |
| `examples/demos/diagrams/diagram_nested_loop.py` | diagrams |
| `examples/demos/diagrams/diagram_noise_ports.py` | diagrams |
| `examples/sandbox/engine/engine_ancf_tire_fall.py` | engine |
| `examples/sandbox/engine/engine_physics_in_diagram.py` | engine |
| `examples/sandbox/engine/engine_physics_many_spheres.py` | engine |
| `examples/demos/hybrid/hybrid_multi_rate.py` | hybrid |
| `examples/demos/hybrid/smc_pendulum_rate.py` | hybrid |
| `examples/demos/identification/params_gradient.py` | identification |
| `examples/demos/interfaces/c_export.py` | interfaces |
| `examples/demos/interfaces/c_export_proportional.py` | interfaces |
| `examples/demos/mpc/mpc_car_minimal.py` | mpc |
| `examples/demos/mpc/mpc_integrator_numpy.py` | mpc |
| `examples/demos/mpc/mpc_car_circuit.py` | mpc |
| `examples/projects/mpc/mpc_dual_rate.py` | mpc |
| `examples/projects/mpc/mpc_path.py` | mpc |
| `examples/projects/mpc/mpc_slalom.py` | mpc |
| `examples/projects/mpc/mpc_spatial.py` | mpc |
| `examples/demos/optimization/optim_basic.py` | optimization |
| `examples/demos/optimization/optim_plot.py` | optimization |
| `examples/projects/pathtracking/cascade_path_tracking.py` | pathtracking |
| `examples/projects/pathtracking/bicycle_los/` | pathtracking |
| `examples/projects/pathtracking/bicycle_los_v2/` | pathtracking |
| `examples/projects/pathtracking/mpc_v1/` | pathtracking |
| `examples/demos/planning/rrt/rrt_car_parking.py` | rrt |
| `examples/demos/planning/rrt/rrt_holonomic_obstacles.py` | rrt |
| `examples/demos/planning/rrt/rrt_pendulum_swingup.py` | rrt |
| `examples/demos/planning/rrt/rrt_star_live.py` | rrt |
| `examples/demos/planning/value_iteration/vi_basics.py` | value_iteration |
| `examples/demos/planning/value_iteration/vi_double_pendulum_jax.py` | value_iteration |
| `examples/demos/planning/value_iteration/vi_pendulum.py` | value_iteration |
| `examples/demos/plots/plot_internal_signals.py` | plots |
| `examples/demos/plots/plot_phase_plane.py` | plots |
| `examples/demos/plots/plot_trajectory_options.py` | plots |
| `examples/demos/plots/plot_readme.py` | plots |
| `examples/demos/realtime/game_cartpole.py` | realtime |
| `examples/demos/realtime/game_ur5_setpoint.py` | realtime |
| `examples/demos/robotic/computed_torque_two_link.py` | robotic |
| `examples/demos/robotic/joint_impedance_two_link.py` | robotic |
| `examples/demos/robotic/kinematic_nullspace_five_link.py` | robotic |
| `examples/demos/robotic/kinematic_two_link.py` | robotic |
| `examples/demos/robotic/task_impedance_two_link.py` | robotic |
| `examples/sandbox/robotic/joint_impedance_ur5.py` | robotic |
| `examples/sandbox/robotic/task_impedance_ur5.py` | robotic |
| `examples/demos/statespace/cartpole_lqr.py` | statespace |
| `examples/demos/step/diagram_cascade_zoh.py` | step |
| `examples/demos/step/diagram_unity_feedback.py` | step |
| `examples/demos/step/step_accumulator.py` | step |
| `examples/demos/step/step_fibonacci.py` | step |
| `examples/sandbox/symbolic/symbolic_quadruple_pendulum.py` | symbolic |
| `examples/demos/trajopt/trajopt_cartpole_collocation_jax.py` | trajopt |
| `examples/demos/trajopt/trajopt_ur5_collocation_jax.py` | trajopt |
| `examples/demos/trajopt/trajopt_cartpole_rollout_gradients.py` | trajopt |
| `examples/demos/trajopt/trajopt_holonomic_corridor.py` | trajopt |
| `examples/projects/car_trajopt/car_trajopt_compare.py` | trajopt |
| `examples/projects/car_trajopt/car_trajopt_lanechange.py` | trajopt |
| `examples/projects/car_trajopt/car_trajopt_uturn.py` | trajopt |

| Notebook |
| --- |
| `examples/learn/teaching/topics/mpc.ipynb` |
| `examples/projects/car_trajopt/car_trajopt.ipynb` |
| `examples/sandbox/scratch/cartpole_rollout_gradients.ipynb` |
| `examples/learn/intro/08_optimization.ipynb` |
| `examples/learn/intro/10_graphical.ipynb` |
| `examples/learn/intro/07_compile.ipynb` |
| `examples/learn/intro/showcase_minilink.ipynb` |
| `examples/learn/intro/showcase_jax.ipynb` |
| `examples/tooling/notebooks/benchmark.ipynb` |

---

## 5. Explicit non-goals (library + demos)

| Item | Decision | Replacement |
| --- | --- | --- |
| Discrete-time / ZOH library | **Drop** | Continuous-time only (DESIGN §3) |
| RNN / recurrent blocks | **Drop** | — |
| Hybrid events / mixed-rate sim | **Drop** | — |
| Pyro `sys2game` framework | **Drop** | superseded by `simulation/realtime/` (`RealtimeSimulator`) |
| Stable-Baselines3 in-library RL | **Drop** | interfaces/gymnasium.py; train externally |
| `*withObstacles` plant subclasses | **Drop** | planning/spatial/Scene + bind() |
| DynamicProgramming2DRectBivariateSpline | **Drop** | Grid DP backends |
| RigidBody2D standalone plant | **Drop** | GeneralizedMechanicalSystem |
| StochasticSystemWrapper | **Drop** | Noise ports only unless reversed |
| University course folders | **Drop** | Port ideas via library demos, not 1:1 |
| Research projects (asimov, ugv, wcrt, …) | **Drop** | Out of library scope |
| Pacejka tire | **TODO/review** | [TODO.md](TODO.md) Later |

---

## 6. Priority backlog (from gaps above)

Ordered by unblock count:

| Priority | Work | Unblocks |
| --- | --- | --- |
| P2 | ~~`Manipulator` catalog rebase + `control/modelbased.py`, `control/robotic.py`~~ (library landed) | representative closed-loop demos per plant band |
| P2 | `TrajectoryLQRController` | trajectory_stabilization/ demos |
| P3 | `planning/trajectory_generation/` (min-snap) | differentialflatness/ demos |
| P3 | `estimation/luenberger.py`, `kalman.py` | LQG demos |
| P3 | `identification/fitting.py` | params-gradient workflow |
| P3 | ~~`interfaces/gymnasium.py`~~ (landed) | GRO860 course notebooks |
| P3 | Frequency tools (pole-zero, Nyquist, margins, `ss2tf`) | transfer_functions/ completion |
| P3 | `planning/filters.py` TrajectoryFilter | traj post-processing |
| P4 | Representative closed-loop demo per `demos_by_system/*` plant band | ~93 TODO demos shrink to ~20 targets |
| P4 | README pyro → minilink API mapping table | migration guide |
