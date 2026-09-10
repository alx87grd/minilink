"""Double integrator with a quadratic cost by value iteration."""

import numpy as np

from minilink import (
    DoubleIntegrator,
    DynamicProgrammingPlanner,
    PlanningProblem,
    QuadraticCost,
)

INF = 300.0

plant = DoubleIntegrator()
plant.state.lower_bound[:] = [-10.0, -5.0]
plant.state.upper_bound[:] = [10.0, 5.0]
plant.inputs["u"].lower_bound = np.array([-5.0])
plant.inputs["u"].upper_bound = np.array([5.0])

cost = QuadraticCost.from_system(
    plant, xbar=np.zeros(2), R=np.array([[10.0]]), S=np.diag([10.0, 10.0])
)
problem = PlanningProblem(plant, x_goal=np.zeros(2), cost=cost)

planner = DynamicProgrammingPlanner(
    problem,
    x_grid=(101, 101),
    u_grid=(41,),
    dt=0.05,
    alpha=1.0,
    tol=0.5,
    max_iterations=1000,
    out_of_bound_cost=INF,
    verbose=True,
)
planner.solve()
planner.plot_cost2go(jmax=INF, show_3d=True)

controller = planner.get_controller()
controller.plot_control_law()  # interpolated law the closed loop actually feels
diagram = controller @ plant

plant.x0 = np.array([5.0, 3.0])
diagram.plot_diagram()
trajectory = diagram.compute_trajectory(tf=20.0)
diagram.plot_trajectory(trajectory)

planner.plot_policy(trajectory=trajectory)
