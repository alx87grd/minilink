# Pyro 2.0 ↔ Minilink parity audit

Snapshot vs [SherbyRobotics/pyro](https://github.com/SherbyRobotics/pyro) local checkout (2026-06-29).

Status legend: **Done** = equivalent landed · **Partial** = framework exists, gap noted · **TODO** = planned/in-scope · **Drop** = explicit non-goal

---

## Summary

Shrunk 2026-09-05 (Phase D6) to the **open** rows; landed symbols are listed
once in the name map at the end. Status legend: **Partial** = framework exists,
gap noted · **TODO** = planned/in-scope · **Drop** = explicit non-goal.

| Bucket | Pyro | Done | Partial | TODO | Drop |
| --- | ---: | ---: | ---: | ---: | ---: |
| Library symbols | 105 | 80 | 11 | 8 | 6 |
| Example scripts | 195 | 28 | 45 | 87 | 35 |

Parity is the **v0.2** milestone ([ROADMAP.md §1](../../ROADMAP.md#1-north-star)); v0.1 is GRO860 end to end.
Release criterion carried over: every **in-scope** pyro library module has a minilink home or a documented drop; representative closed-loop demo per major plant family; README pyro → minilink migration guide (built from the name map below).

## 1. Library modules — pyro → minilink

| Band | Pyro module | Pyro symbol | Minilink home | Minilink symbol | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Framework | `pyro/dynamic/mechanical.py` | MechanicalSystemWithPositionInputs | `—` | — | **Drop** | No such class in minilink; DESIGN rejects `WithPositionInputs` inheritance branches — mixed inputs use named ports + `generalized_force` |
| Framework | `pyro/dynamic/rigidbody.py` | GeneralizedMechanicalSystemWithPositionInputs | `—` | — | **Drop** | No such class in minilink; DESIGN rejects `WithPositionInputs` inheritance branches — mixed inputs use named ports + `generalized_force` |
| Framework | `pyro/dynamic/rigidbody.py` | RigidBody2D | `—` | — | **Drop** | Use GeneralizedMechanicalSystem or catalog plant |
| Framework | `pyro/dynamic/statespace.py` | StateObserver | `minilink/estimation/` | — | **TODO** | estimation/luenberger.py |
| Framework | `pyro/dynamic/statespace.py` | ObservedSystem | `minilink/estimation/` | — | **TODO** | LQG blocked on observers |
| Framework | `pyro/dynamic/stochastic.py` | NoiseSignal | `minilink/blocks/sources.py` | noise ports in diagrams | **Partial** | No StochasticSystemWrapper |
| Framework | `pyro/dynamic/stochastic.py` | StochasticSystemWrapper | `—` | — | **Drop** | Explicit non-goal unless reversed ([TODO.md](TODO.md) Later) |
| Framework | `pyro/dynamic/tranferfunction.py` | ss2tf() | `minilink/analysis/` | — | **TODO** | Frequency backlog |
| Catalog | `pyro/dynamic/vehicle_dynamic.py` | TireModel (ABC) | `minilink/dynamics/catalog/vehicles/dynamic_bicycle.py` | — | **Partial** | Only the linear tire implemented (pure functions) |
| Catalog | `pyro/dynamic/vehicle_dynamic.py` | Pacejka | `—` | — | **TODO** | [TODO.md](TODO.md) Later |
| Catalog | `pyro/dynamic/vehicle_steering.py` | HolonomicMobileRobotwithObstacles | `minilink/planning/spatial/` | Scene + bind() | **Partial** | Scene replaces plant wrapper |
| Catalog | `pyro/dynamic/vehicle_steering.py` | Holonomic3DMobileRobotwithObstacles | `minilink/planning/spatial/` | Scene | **Partial** | Scene replaces plant wrapper |
| Catalog | `pyro/dynamic/vehicle_steering.py` | KinematicCarModelwithObstacles | `minilink/planning/spatial/` | Scene | **Partial** |  |
| Catalog | `pyro/dynamic/manipulator.py` | TwoLinkManipulatorwithObstacles | `minilink/planning/spatial/` | Scene | **Partial** |  |
| Catalog | `pyro/dynamic/manipulator.py` | FiveLinkPlanarManipulatorwithObstacles | `minilink/planning/spatial/` | Scene | **Partial** |  |
| Control | `pyro/control/controller.py` | DynamicClosedLoopSystem | `minilink/core/diagram.py` | DiagramSystem | **Partial** |  |
| Control | `pyro/control/linear.py` | PIDController | `minilink/control/siso.py` | FilteredController (filtered PID) | **Partial** | Dedicated PID wrapper pending (v0.2 robotic PID wrappers) |
| Control | `pyro/control/lqr.py` | TrajectoryLQRController | `minilink/control/lqr.py` | — | **TODO** | Trajectory stabilization demos |
| Control | `pyro/control/robotcontrollers.py` | JointPD, EndEffectorPD, … | `minilink/control/robotic.py` | JointImpedance, TaskImpedance, TaskKinematic, TaskKinematicNullspace | **Partial** | Dynamic joint/effector PID wrappers TODO |
| Planning | `pyro/planning/dynamicprogramming.py` | DynamicProgramming2DRectBivariateSpline | `minilink/planning/policy_synthesis/dp.py` | — | **Drop** | Not needed; grid backends cover use cases |
| Planning | `pyro/planning/filters.py` | TrajectoryFilter | `minilink/planning/` | — | **TODO** | Butterworth filtfilt post-filter |
| Planning | `pyro/planning/plan.py` | OpenLoopController | `minilink/blocks/sources.py` | TrajectorySource | **Partial** |  |
| Planning | `pyro/planning/trajectorygeneration.py` | SingleAxisPolynomialTrajectoryGenerator | `minilink/planning/` | — | **TODO** | trajectory_generation/ |
| Planning | `pyro/planning/trajectorygeneration.py` | MultiPointSingleAxisPolynomialTrajectoryGenerator | `minilink/planning/` | — | **TODO** | min-snap / differential flatness |
| Tools | `pyro/tools/sys2game.py` | InteractiveContinuousDynamicSystem | `minilink/simulation/realtime/` | superseded by `RealtimeSimulator` + `PygameInput` | **Drop** | Own realtime tool, not a port |

## 2. Minilink-only (no pyro equivalent)

| Minilink | What | Notes |
| --- | --- | --- |
| `minilink/core/compile/` | ExecutionPlan, NumPy/JAX evaluators | Compile band; pyro has no separate compile layer |
| `minilink/core/geometry.py` | SDF shapes, cost algebra | Spatial planning primitives |
| `minilink/optimization/` | MathematicalProgram, Optimizer | General NLP; pyro trajopt is narrower |
| `minilink/planning/spatial/` | Scene, WorkspaceField, RobotBody | Obstacle/clearance layer replaces *withObstacles plants |
| `minilink/planning/search/dubins.py` | Dubins steering | Extra beyond pyro RRT |
| `minilink/blocks/neural.py` | MLP block (JAX) | Prototype; pyro RL is SB3-only |
| `minilink/experimental/symbolic/` | Symbolic derivation | Quarantine; no pyro equivalent |
| `minilink/experimental/engines/` | Contact, ANCF tire (JAX) | Experimental physics engines |
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
| `courses/udes_gro640/prob/abcd1234.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/demo_crash_commande_en_position.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/gro640_robots.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/test_commande_en_force.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/test_commande_en_position.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/test_f.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro640/prob/test_trajectoire_3D.py` | **TODO** | Robot control; blocked on control/robotic.py |
| `courses/udes_gro860/dp_mass_min_time_policy_evaluation.py` | **Partial** | Core tool demos exist; course variant not ported |
| `courses/udes_gro860/lqr_cartpole_traj.py` | **Partial** | Core tool demos exist; course variant not ported |

### 3.2 `demos_by_system/`

| Pyro script | Status | Minilink equivalent / notes |
| --- | --- | --- |
| `demos_by_system/acrobot/acrobot_with_lqr.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/acrobot/acrobot_with_trajectory_optimization.py` | **Partial** | Trajopt done; plant-specific demo TODO |
| `demos_by_system/boat/boat_position_controller.py` | **TODO** | Representative closed-loop for boat |
| `demos_by_system/boat/boat_with_ppo_demo.py` | **TODO** | Representative closed-loop for boat |
| `demos_by_system/car_dynamic/demo_oversteer.py` | **TODO** | Representative closed-loop for car_dynamic |
| `demos_by_system/car_dynamic/demo_understeer.py` | **TODO** | Representative closed-loop for car_dynamic |
| `demos_by_system/car_propulsion/longitudinal_car_braking_value_iteration.py` | **Partial** | DP done; plant-specific demo TODO |
| `demos_by_system/car_propulsion/longitudinal_car_with_torque_input.py` | **TODO** | Representative closed-loop for car_propulsion |
| `demos_by_system/car_steering/bicycle.py` | **TODO** | Representative closed-loop for car_steering |
| `demos_by_system/car_steering/bicycle_exploration_with_rrt.py` | **Partial** | RRT done; plant-specific demo TODO |
| `demos_by_system/car_steering/bicycle_parallel_parking_with_rrt.py` | **Partial** | examples/demos/planning/rrt/rrt_car_parking.py |
| `demos_by_system/car_steering/car.py` | **TODO** | Representative closed-loop for car_steering |
| `demos_by_system/car_steering/car_trajectory_with_rrt.py` | **Partial** | RRT done; plant-specific demo TODO |
| `demos_by_system/car_steering/car_with_custom_lateral_controller.py` | **TODO** | Representative closed-loop for car_steering |
| `demos_by_system/car_steering/car_with_valueiteration_minimum_time.py` | **Partial** | DP done; plant-specific demo TODO |
| `demos_by_system/car_steering/car_with_valueiteration_quadratic_cost.py` | **Partial** | DP done; plant-specific demo TODO |
| `demos_by_system/cartpole/cartpole_LQG.py` | **TODO** | estimation/kalman.py |
| `demos_by_system/cartpole/cartpole_demo.py` | **TODO** | Open-loop cartpole showcase |
| `demos_by_system/cartpole/cartpole_stabilization.py` | **Partial** | examples/demos/control/cartpole_lqr.py |
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
| `demos_by_system/pendulum_simple/simple_pendulum_with_lqr.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/pendulum_simple/simple_pendulum_with_open_loop_controller.py` | **TODO** | Representative closed-loop for pendulum_simple |
| `demos_by_system/pendulum_simple/simple_pendulum_with_pid.py` | **Partial** | Controller exists; plant demo TODO |
| `demos_by_system/pendulum_simple/simple_pendulum_with_trajectory_following_computed_torque.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_simple/simple_pendulum_with_trajectory_following_sliding_mode_controller.py` | **TODO** | control/modelbased.py |
| `demos_by_system/pendulum_simple/simple_pendulum_with_valueiteration_minimum_time.py` | **Partial** | examples/demos/planning/value_iteration/vi_minimum_time.py (mass) + examples/demos/planning/value_iteration/vi_pendulum_swingup.py |
| `demos_by_system/plane/plane_cobra.py` | **TODO** | Representative closed-loop for plane |
| `demos_by_system/plane/plane_simple_controller.py` | **TODO** | Representative closed-loop for plane |
| `demos_by_system/robot_arm_2dof/twolinkrobot_effector_impedance_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_2dof/twolinkrobot_effector_pid_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
| `demos_by_system/robot_arm_2dof/twolinkrobot_joint_pid_controller.py` | **TODO** | control/robotic.py + manipulator rebase |
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
| `demos_by_tool/dynamicprogramming/double_pendulum_optimal_swingup_load.py` | **Partial** | examples/demos/planning/value_iteration/vi_double_pendulum_jax.py |
| `demos_by_tool/dynamicprogramming/helicopter_tunnel.py` | **TODO** | DP specialty demo not ported |
| `demos_by_tool/dynamicprogramming/pendulum_optimal_swingup_low_def_fast_computation.py` | **Partial** | examples/demos/planning/value_iteration/vi_pendulum_swingup.py (grid tuning differs) |
| `demos_by_tool/dynamicprogramming/pendulum_reachability.py` | **Partial** | ReachabilityCost exists; demo TODO |
| `demos_by_tool/dynamicprogramming/policy_evaluator_with_computed_torque.py` | **TODO** | control/modelbased.py |
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

## Appendix — pyro → minilink name map (landed)

Two-column record of every symbol marked Done before the shrink; the source for the v0.2 migration guide.

| Pyro symbol | Minilink symbol | Minilink module |
| --- | --- | --- |
| ContinuousDynamicSystem | DynamicSystem, System, DiagramSystem | `minilink/core/system.py` |
| MechanicalSystem | MechanicalSystem | `minilink/dynamics/abstraction/mechanical.py` |
| GeneralizedMechanicalSystem | GeneralizedMechanicalSystem | `minilink/dynamics/abstraction/generalized_mechanical.py` |
| StateSpaceSystem | LTISystem, StateSpaceSystem | `minilink/dynamics/abstraction/state_space.py` |
| linearize() | linearize() | `minilink/analysis/linearize.py` |
| TransferFunction | TransferFunction | `minilink/blocks/transfer_function.py` |
| VanderPol | VanderPol | `minilink/dynamics/catalog/equations/oscillators.py` |
| SimpleIntegrator | SimpleIntegrator | `minilink/dynamics/catalog/equations/integrators.py` |
| DoubleIntegrator | DoubleIntegrator | `minilink/dynamics/catalog/equations/integrators.py` |
| TripleIntegrator | TripleIntegrator | `minilink/dynamics/catalog/equations/integrators.py` |
| SinglePendulum | Pendulum | `minilink/dynamics/catalog/pendulum/pendulum.py` |
| InvertedPendulum | InvertedPendulum | `minilink/dynamics/catalog/pendulum/pendulum.py` |
| DoublePendulum | DoublePendulum | `minilink/dynamics/catalog/pendulum/double_pendulum.py` |
| Acrobot | Acrobot | `minilink/dynamics/catalog/pendulum/double_pendulum.py` |
| TwoIndependentSinglePendulum | TwoIndependentPendulums | `minilink/dynamics/catalog/pendulum/pendulum.py` |
| CartPole | CartPole (+ JaxCartPole) | `minilink/dynamics/catalog/pendulum/cartpole.py` |
| RotatingCartPole | RotatingCartPole | `minilink/dynamics/catalog/pendulum/cartpole.py` |
| UnderActuatedRotatingCartPole | UnderActuatedRotatingCartPole | `minilink/dynamics/catalog/pendulum/cartpole.py` |
| SingleMass | SingleMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` |
| TwoMass | TwoMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` |
| ThreeMass | ThreeMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` |
| FloatingSingleMass | FloatingSingleMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` |
| FloatingTwoMass | FloatingTwoMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` |
| FloatingThreeMass | FloatingThreeMass | `minilink/dynamics/catalog/mass_spring_damper/linear.py` |
| MountainCar | MountainCar | `minilink/dynamics/catalog/vehicles/mountain_car.py` |
| Boat2D | Boat2D | `minilink/dynamics/catalog/marine/boat.py` |
| Boat2DwithCurrent | Boat2DWithCurrent | `minilink/dynamics/catalog/marine/boat.py` |
| Drone2D | Drone2D | `minilink/dynamics/catalog/aerial/drone.py` |
| SpeedControlledDrone2D | SpeedControlledDrone2D | `minilink/dynamics/catalog/aerial/drone.py` |
| ConstantSpeedHelicopterTunnel | ConstantSpeedHelicopterTunnel | `minilink/dynamics/catalog/aerial/drone.py` |
| Drone2DwithSideTruster | Drone2DWithSideThruster | `minilink/dynamics/catalog/aerial/drone.py` |
| Plane2D | Plane2D | `minilink/dynamics/catalog/aerial/plane.py` |
| Rocket | Rocket | `minilink/dynamics/catalog/aerial/rocket.py` |
| QuarterCarOnRoughTerrain | QuarterCarOnRoughTerrain | `minilink/dynamics/catalog/vehicles/suspension.py` |
| LinearTire | `tire_slip` / `linear_tire_forces` (coefficients in plant `params`) | `minilink/dynamics/catalog/vehicles/dynamic_bicycle.py` |
| DynamicBicycle | DynamicBicycle (+ JAX twins, rate inputs) | `minilink/dynamics/catalog/vehicles/dynamic_bicycle.py` |
| LongitudinalFrontWheelDriveCarWithWheelSlipInput | same | `minilink/dynamics/catalog/vehicles/propulsion.py` |
| LongitudinalFrontWheelDriveCarWithTorqueInput | same | `minilink/dynamics/catalog/vehicles/propulsion.py` |
| KinematicBicyleModel | KinematicBicycle | `minilink/dynamics/catalog/vehicles/steering.py` |
| HolonomicMobileRobot | HolonomicMobileRobot | `minilink/dynamics/catalog/vehicles/steering.py` |
| Holonomic3DMobileRobot | HolonomicMobileRobot3D | `minilink/dynamics/catalog/vehicles/steering.py` |
| KinematicCarModel | KinematicCar | `minilink/dynamics/catalog/vehicles/steering.py` |
| ConstantSpeedKinematicCarModel | ConstantSpeedKinematicCar | `minilink/dynamics/catalog/vehicles/steering.py` |
| UdeSRacecar | UdeSRacecar | `minilink/dynamics/catalog/vehicles/steering.py` |
| Manipulator | Manipulator | `minilink/dynamics/abstraction/manipulator.py` |
| SpeedControlledManipulator | SpeedControlledManipulator | `minilink/dynamics/catalog/manipulators/arms.py` |
| OneLinkManipulator | OneLinkManipulator | `minilink/dynamics/catalog/manipulators/arms.py` |
| TwoLinkManipulator | TwoLinkManipulator | `minilink/dynamics/catalog/manipulators/arms.py` |
| ThreeLinkManipulator3D | ThreeLinkManipulator3D | `minilink/dynamics/catalog/manipulators/arms.py` |
| FiveLinkPlanarManipulator | FiveLinkPlanarManipulator | `minilink/dynamics/catalog/manipulators/arms.py` |
| StaticController | DiagramSystem wiring | `minilink/core/diagram.py` |
| StaticController.plot_control_law | plot_control_law(), plot_input_output_map() | `minilink/graphical/port_map.py` |
| ClosedLoopSystem | `@`, `>>`, `+` composition | `minilink/core/diagram.py` |
| DynamicController | DynamicController marker (FilteredController, ImpedanceIntegralController) | `minilink/core/feedback.py` |
| ProportionalController | ProportionalController | `minilink/control/output.py` |
| synthesize_lqr_controller | synthesize_lqr | `minilink/control/lqr.py` |
| linearize_and_synthesize_lqr_controller | linearize + synthesize_lqr | `minilink/control/lqr.py` |
| ComputedTorqueController | ComputedTorqueController | `minilink/control/modelbased.py` |
| SlidingModeController | SlidingModeController | `minilink/control/modelbased.py` |
| stable_baseline3_controller | SB3Controller | `minilink/interfaces/gymnasium.py` |
| Trajectory | Trajectory | `minilink/core/trajectory.py` |
| Simulator | Simulator | `minilink/simulation/simulator.py` |
| CLosedLoopSimulator | compute_trajectory() | `minilink/core/facades.py` |
| DynamicCLosedLoopSimulator | Simulator | `minilink/simulation/simulator.py` |
| CostFunction, QuadraticCostFunction, … | CostFunction, QuadraticCost, … | `minilink/core/costs.py` |
| Reachability | ReachabilityCost | `minilink/core/costs.py` |
| TrajectoryPlotter | plot_trajectory() | `minilink/graphical/` |
| Animator | animate() | `minilink/graphical/animation/` |
| PhasePlot, PhasePlot3 | plot_phase_plane() | `minilink/graphical/phase_plane/` |
| GridDynamicSystem | StateSpaceGrid | `minilink/planning/policy_synthesis/discretizer.py` |
| DynamicProgramming | DynamicProgrammingPlanner | `minilink/planning/policy_synthesis/dp.py` |
| DynamicProgrammingWithLookUpTable | same planner + lookup | `minilink/planning/policy_synthesis/dp.py` |
| LookUpTableController | LookupTableController | `minilink/planning/policy_synthesis/lookup_policy.py` |
| PolicyEvaluator | PolicyEvaluator | `minilink/planning/policy_synthesis/policy_eval.py` |
| Planner | Planner | `minilink/planning/planner.py` |
| RRT | RRTPlanner, RRTStarPlanner | `minilink/planning/search/` |
| DirectCollocationTrajectoryOptimisation | DirectCollocation, Shooting, MS | `minilink/planning/trajectory_optimization/` |
| transformation_matrix_2D | frame tf helpers | `minilink/core/kinematics.py` |
| transform_points_2D, arrows | primitives, drawables | `minilink/graphical/animation/` |
| Sys2Gym | Sys2Gym | `minilink/interfaces/gymnasium.py` |
