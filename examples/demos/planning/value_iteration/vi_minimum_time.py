import numpy as np

from minilink.core.costs import TimeCost
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import DynamicProgrammingPlanner
from minilink.planning.problems import PlanningProblem

INF = 10.0
X0 = np.array([1.2, 0.0])

plant = DoubleIntegrator()
plant.state.lower_bound[:] = [-2.0, -2.0]
plant.state.upper_bound[:] = [2.0, 2.0]
plant.inputs["u"].lower_bound = np.array([-1.0])
plant.inputs["u"].upper_bound = np.array([1.0])

cost = TimeCost.from_system(plant, eps=1e-3)
problem = PlanningProblem(plant, x_goal=np.zeros(2), cost=cost)

grid = StateSpaceGrid(problem, x_grid_shape=(201, 201), u_grid_shape=(3,), dt=0.05)
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    alpha=1.0,
    tol=1e-3,
    max_iterations=800,
    out_of_bound_cost=INF,
    verbose=True,
)
planner.solve()
planner.clean_infeasible_set()

planner.plot_cost2go(jmax=INF, show_3d=True)
planner.plot_policy()

controller = planner.get_controller()
controller.plot_control_law()  # interpolated law next to the discrete policy table
diagram = controller @ plant
diagram.name = "Minimum-time double integrator (value iteration)"

plant.x0 = X0.copy()
diagram.plot_diagram()
trajectory = diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory(trajectory)

planner.plot_policy(trajectory=trajectory)
# diagram.animate()
