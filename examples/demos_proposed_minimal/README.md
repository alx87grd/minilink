# Proposed minimal demos

A draft for the maintainer's review: the scripts proposed as the whole demo set of the
consolidated library, written beside the existing `examples/demos/`, which they do not
replace until he rules. Seven are the shortlist; `racecar_mppi.py` and
`racecar_fast_lap.py` are two prototypes added after it, and
`pendulum_value_iteration.py` and `cartpole_value_iteration.py` the two value-iteration
swing-ups. The reasoning is in
[docs/plans/consolidation.md](../../docs/plans/consolidation.md), §4: the tutorials
carry what fits in a notebook, and a demo is kept only for a story a notebook tells badly.
Each script tells one story, shows the diagram where the wiring is the point, and ends on
one plot and one animation.

| Script | The story | Run time, animation skipped |
| --- | --- | --- |
| `cartpole_swingup.py` | a swing-up planned once falls when replayed from another start; LQR feedback along the same plan holds it | 9 s |
| `pendulum_value_iteration.py` | value iteration swings a pendulum up with a weak motor; the cost-to-go and the law are animated sweep by sweep | 20 s (a laptop) |
| `cartpole_value_iteration.py` | the same Bellman sweeps on a 4-D grid swing a cart-pole up | 27 s (a laptop) |
| `pendulum_every_planner.py` | one swing-up problem for LQR, value iteration, trajectory optimization, RRT and the optimized plan tracked: each plan scored as written beside its run, then the runs judged over 100 random starts | 41 s |
| `pendulum_pid_gradient_tuning.py` | a PID on its computer's clock, the whole sampled loop differentiated, its three gains tuned by an optimizer | 6 s |
| `cartpole_lqg.py` | a Kalman filter and an LQR gain balance a cart-pole under noise: one run, then 200 scored | 8 s |
| `racecar_mpc.py` | a plan made on the kinematic bicycle drives the dynamic racecar through an inner speed loop, round a circuit and its cone | 22 s |
| `racecar_mppi.py` | the same cascade with a sampling planner: 2048 futures drawn on the kinematic model at every tick, no gradient | 27 s |
| `racecar_fast_lap.py` | an SQP MPC on the dynamic model brakes into the corners and uses the tires' grip: a lap in 4.8 s against 6.4 s | 10 s |
| `ur5_impedance.py` | a spring and a damper at the hand of a six-axis arm, pulled through four points in 3-D | 13 s |
| `drone_learns_to_fly.py` | a neural law trained by reinforcement brings a drone back to hover from a random start | 43 s |
| `rrt_star_obstacles.py` | a tree rewires among obstacles and its path shortens, drawn while it solves | 31 s |
| `offroad_car_game.py` | the 3-D car on its suspension, driven by hand over rolling hills | interactive |

The run times are on one cloud machine unless marked. `racecar_mpc.py` and `ur5_impedance.py` animate in
the MeshCat viewer, and `offroad_car_game.py` needs a keyboard or a gamepad.
