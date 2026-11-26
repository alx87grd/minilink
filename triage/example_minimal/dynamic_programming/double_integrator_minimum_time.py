"""Minimum-time double integrator by value iteration."""

import numpy as np

from minilink import (
    DoubleIntegrator,
    DynamicProgrammingPlanner,
    PlanningProblem,
    TimeCost,
    # QuadraticCost,
)

########################################################
### Tunables
########################################################

INF = 10.0
X0 = np.array([1.2, 0.0])

########################################################
### The plant
########################################################

plant = DoubleIntegrator()
plant.state.lower[:] = [-2.0, -2.0]
plant.state.upper[:] = [2.0, 2.0]
plant.inputs["u"].lower = np.array([-1.0])
plant.inputs["u"].upper = np.array([1.0])

########################################################
### Minimum-time task
########################################################

cost = TimeCost.from_system(plant, radius=1e-3)

# cost = QuadraticCost.from_system(
#     plant, x_bar=np.zeros(2), R=np.array([[10.0]]), S=np.diag([10.0, 10.0])
# )

problem = PlanningProblem(plant, cost=cost, U=plant.input_box)

########################################################
### Value iteration
########################################################

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
planner.solve(verbose=False, record_history=True)

# planner.animate_cost_to_go()
# planner.animate_control_law()

planner.plot_cost_to_go(jmax=INF)
planner.plot_cost_to_go(jmax=INF, is_3d=True)
planner.plot_control_law()

########################################################
### Closed-loop rollout
########################################################

controller = planner.controller
controller.plot_control_law()
diagram = controller @ plant

plant.x0 = X0
diagram.plot_diagram()
trajectory = diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory(trajectory)

planner.plot_control_law(trajectory=trajectory)
# diagram.animate()
