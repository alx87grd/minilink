# Proposed minimal demos

A draft for the maintainer's review: the ten scripts proposed as the whole demo set of the
consolidated library, written beside the existing `examples/demos/`, which they do not
replace until he rules. The reasoning is in
[docs/plans/minimal-surface.md](../../docs/plans/minimal-surface.md), §4: the tutorials
carry what fits in a notebook, and a demo is kept only for what a notebook shows badly, a
long run, a 3-D scene, a real-time game, a learning run of minutes.

| Script | The story | Built from | Run time, animations skipped |
| --- | --- | --- | --- |
| `cartpole_swingup.py` | a trajectory planned once, then LQR feedback along it | `control/trajectory_lqr_cartpole.py` | 17 s |
| `cartpole_lqg.py` | a Kalman filter and an LQR gain under noise, scored over 200 runs | `control/lqg_cartpole.py` and `compile/lqg_monte_carlo.py`, merged | 7 s |
| `racecar_mpc.py` | model predictive control round a circuit, past two cones | `udes_racecar/mpc_racecar_dynamic.py` | 30 s |
| `racecar_path_tracking.py` | pure pursuit, Stanley and line of sight on one car and circuit | `udes_racecar/path_tracking_racecar_compare.py`, as it is | 9 s |
| `two_link_task_impedance.py` | a spring and a damper at an arm's end effector | `robotic/task_impedance_two_link.py` | 2 s |
| `rocket_landing_rl.py` | a landing learned by reinforcement, from a random start | `rl/rocket_landing_rl.py` | minutes |
| `ur5_rl_to_bode.py` | a six-axis arm: an impedance loop, a learned law, its Bode plot | `rl/ur5_rl_to_bode.py`, as it is | 2 min |
| `offroad_car_hills.py` | the 3-D car on its suspension over rolling hills | `dynamics/offroad_car_3d.py`, as it is | 4 s |
| `rrt_star_obstacles.py` | a tree rewiring among obstacles, drawn while it solves | `rrt/rrt_star_live.py` | 32 s |
| `game_cartpole.py` | a cart-pole balanced by hand with the arrow keys | `realtime/game_cartpole.py` | interactive |

The run times are on one cloud machine, with the animations and the game skipped; three
scripts animate in the MeshCat viewer and wait for a browser when run with no display
(`racecar_mpc.py`, `ur5_rl_to_bode.py`, `offroad_car_hills.py`).
