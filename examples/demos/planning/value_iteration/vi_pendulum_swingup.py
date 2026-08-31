import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.problems import PlanningProblem

INF = 500.0
UPRIGHT = np.array([-np.pi, 0.0])
TORQUE = 5.0
X0 = np.array([-0.1, 0.0])

plant = Pendulum()
plant.state.lower_bound = np.array([-10.0, -10.0])
plant.state.upper_bound = np.array([10.0, 10.0])
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])

cost = QuadraticCost.from_system(
    plant,
    xbar=UPRIGHT,
    Q=np.eye(2),
    R=np.array([[1.0]]),
    S=np.diag([10.0, 10.0]),
)
problem = PlanningProblem(plant, x_goal=UPRIGHT, cost=cost)

grid = StateSpaceGrid(problem, x_grid_shape=(201, 201), u_grid_shape=(21,), dt=0.05)
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    options=DynamicProgrammingOptions(
        alpha=1.0, tol=0.1, max_iterations=2000, out_of_bound_cost=INF, verbose=True
    ),
)
planner.solve()
planner.clean_infeasible_set()

planner.plot_cost2go(jmax=INF, show_3d=True)
planner.plot_policy()

controller = planner.get_controller()
diagram = controller @ plant
diagram.name = "Pendulum swing-up (value iteration)"

plant.x0 = X0.copy()
diagram.plot_diagram()
trajectory = diagram.compute_trajectory(tf=10.0)
diagram.plot_trajectory(trajectory)

planner.plot_policy(trajectory=trajectory)
diagram.camera_scale = 2.0
diagram.animate()
