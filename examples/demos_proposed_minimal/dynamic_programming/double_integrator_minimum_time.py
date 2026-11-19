"""Minimum-time double integrator by value iteration."""

import numpy as np

from minilink import (
    DoubleIntegrator,
    DynamicProgrammingPlanner,
    PlanningProblem,
    TimeCost,
    # QuadraticCost,
)

INF = 10.0
X0 = np.array([1.2, 0.0])

plant = DoubleIntegrator()
plant.state.lower[:] = [-2.0, -2.0]
plant.state.upper[:] = [2.0, 2.0]
plant.inputs["u"].lower = np.array([-1.0])
plant.inputs["u"].upper = np.array([1.0])

cost = TimeCost.from_system(plant, radius=1e-3)

# cost = QuadraticCost.from_system(
#     plant, x_bar=np.zeros(2), R=np.array([[10.0]]), S=np.diag([10.0, 10.0])
# )

problem = PlanningProblem(plant, cost=cost, U=plant.input_box)

planner = DynamicProgrammingPlanner(
    problem,
    x_grid=(201, 201),
    u_grid=(101,),
    dt=0.05,
    gamma=1.0,
    tol=1e-3,
    max_iterations=800,
    out_of_bound_cost=INF,
    backend="jax",
)
planner.solve(verbose=False)

planner.plot_cost_to_go(jmax=INF)
planner.plot_cost_to_go(jmax=INF, is_3d=True)
planner.plot_control_law()

controller = planner.controller
controller.plot_control_law()  # interpolated law next to the discrete policy table
diagram = controller @ plant

plant.x0 = X0
diagram.plot_diagram()
trajectory = diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory(trajectory)

planner.plot_control_law(trajectory=trajectory)
# diagram.animate()
