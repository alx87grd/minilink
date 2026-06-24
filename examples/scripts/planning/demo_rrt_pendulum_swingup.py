"""Pendulum swing-up with a kinodynamic RRT — minilink's take on pyro's randomtree demo.

Run from repo root::

    python examples/scripts/planning/demo_rrt_pendulum_swingup.py

The pendulum is underactuated: the torque limit (±5 Nm) is below the gravity
torque needed to hold it horizontal (m·g·l ≈ 9.8 Nm), so the planner must pump
energy by swinging back and forth. The kinodynamic extender forward-integrates
the pendulum ODE under a few discrete torques, and the tree explores the
``(theta, dtheta)`` phase space until the inverted goal region is reached. No
obstacles — this is pure state-space planning with the dynamics.
"""

import numpy as np

from minilink.core.sets import BallSet
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.extenders import KinodynamicExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner

sys = Pendulum()  # state [theta, dtheta]; theta=0 hangs down, theta=pi inverted
sys.state.lower_bound = np.array([-2.0 * np.pi, -12.0])
sys.state.upper_bound = np.array([2.0 * np.pi, 12.0])
sys.inputs["u"].lower_bound = np.array([-5.0])
sys.inputs["u"].upper_bound = np.array([5.0])

x_start = np.array([0.0, 0.0])  # hanging down, at rest
x_goal = np.array([np.pi, 0.0])  # inverted, at rest
problem = PlanningProblem(
    sys=sys, x_start=x_start, x_goal=x_goal, Xf=BallSet(x_goal, 0.2)
)

torques = [np.array([tau]) for tau in (-5.0, -2.0, 0.0, 2.0, 5.0)]
planner = RRTPlanner(
    problem,
    extender=KinodynamicExtender(controls=torques, horizon=0.3, n_substeps=6),
    options=RRTOptions(seed=0, goal_bias=0.05, max_nodes=20000),
)
traj = planner.compute_solution()
print(
    f"pendulum swing-up: {len(planner.tree.nodes)} nodes, "
    f"final theta={traj.x[0, -1]:.2f} rad (goal pi), dtheta={traj.x[1, -1]:.2f} rad/s, "
    f"swing time {traj.t[-1]:.1f} s"
)

planner.plot_tree(x_axis=0, y_axis=1)
planner.animate_search(x_axis=0, y_axis=1)
planner.plot_solution(signals=("x", "u"))
planner.animate_solution()
