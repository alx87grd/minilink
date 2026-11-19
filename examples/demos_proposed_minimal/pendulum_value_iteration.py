"""Pendulum swing-up by value iteration: the cost-to-go and the law, sweep by sweep."""

import numpy as np

from minilink import DynamicProgrammingPlanner, Pendulum, PlanningProblem, QuadraticCost

TORQUE = 5.0  # [Nm] the motor's limit

# The plant: theta = 0 hanging; leaving the state box is a failure

plant = Pendulum()
plant.inputs["u"].lower[0] = -TORQUE
plant.inputs["u"].upper[0] = TORQUE
plant.state.lower = np.array([-10.0, -10.0])
plant.state.upper = np.array([10.0, 10.0])

# The task: J = ∫ (x − x̄)ᵀ Q (x − x̄) + uᵀ R u dt

x_bar = np.array([-np.pi, 0.0])  # upright, at rest

cost = QuadraticCost.from_system(
    plant, x_bar=x_bar, Q=np.eye(2), R=np.eye(1), S=np.diag([10.0, 10.0])
)
problem = PlanningProblem(plant, cost=cost, U=plant.input_box)

# Value iteration: J(x) = min_u [ g(x, u) dt + J(x + f(x, u) dt) ]

planner = DynamicProgrammingPlanner(
    problem,
    x_grid=(201, 201),
    u_grid=(21,),
    dt=0.05,
    tol=0.1,
    max_iterations=2000,
    out_of_bound_cost=500.0,
)
planner.solve(verbose=True, record_history=True)

planner.animate_cost_to_go()
planner.animate_control_law()

planner.plot_cost_to_go(is_3d=True)

plant.x0 = np.array([-0.1, 0.0])

closed_loop = planner.controller @ plant
closed_loop.plot_diagram()

traj = closed_loop.compute_trajectory(tf=10.0)

closed_loop.plot_trajectory()
planner.plot_control_law(trajectory=traj)
closed_loop.animate()
