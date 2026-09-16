"""Pendulum swing-up with a kinodynamic RRT — minilink's take on pyro's randomtree demo."""

import numpy as np

from minilink import BallSet, Pendulum, PlanningProblem, RRTPlanner
from minilink.planning.search.extenders import KinodynamicExtender
from minilink.planning.search.rrt import RRTOptions

sys = Pendulum()  # state [theta, dtheta]; theta=0 hangs down, theta=pi inverted
sys.state.lower_bound = np.array([-2.0 * np.pi, -12.0])
sys.state.upper_bound = np.array([2.0 * np.pi, 12.0])
sys.inputs["u"].lower_bound = np.array([-5.0])
sys.inputs["u"].upper_bound = np.array([5.0])

x_start = np.array([0.0, 0.0])  # hanging down, at rest
x_goal = np.array([np.pi, 0.0])  # inverted, at rest
problem = PlanningProblem(
    sys=sys,
    x_start=x_start,
    x_goal=x_goal,
    Xf=BallSet(x_goal, 0.2),
    X=sys.state.box,
)

torques = [np.array([tau]) for tau in (-5.0, -2.0, 0.0, 2.0, 5.0)]
planner = RRTPlanner(
    problem,
    extender=KinodynamicExtender(controls=torques, horizon=0.3, n_substeps=6),
    options=RRTOptions(seed=0, goal_bias=0.05, max_nodes=20000),
)
solution = planner.solve()
print(solution.solver)

planner.plot_tree(x_axis=0, y_axis=1)
planner.animate_search(x_axis=0, y_axis=1)
planner.plot_solution(signals=("x", "u"))
planner.animate_solution()
