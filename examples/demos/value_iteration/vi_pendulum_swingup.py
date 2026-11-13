"""Pendulum swing-up by value iteration."""

import numpy as np

from minilink import DynamicProgrammingPlanner, Pendulum, PlanningProblem, QuadraticCost

INF = 500.0
UPRIGHT = np.array([-np.pi, 0.0])
TORQUE = 5.0
X0 = np.array([-0.1, 0.0])

plant = Pendulum()
plant.state.lower = np.array([-10.0, -10.0])
plant.state.upper = np.array([10.0, 10.0])
plant.inputs["u"].lower = np.array([-TORQUE])
plant.inputs["u"].upper = np.array([TORQUE])

cost = QuadraticCost.from_system(
    plant,
    x_bar=UPRIGHT,
    Q=np.eye(2),
    R=np.array([[1.0]]),
    S=np.diag([10.0, 10.0]),
)
problem = PlanningProblem(plant, cost=cost, U=plant.input_box)

planner = DynamicProgrammingPlanner(
    problem,
    x_grid=(201, 201),
    u_grid=(21,),
    dt=0.05,
    tol=0.1,
    max_iterations=2000,
    out_of_bound_cost=INF,
    # backend="jax",
)
planner.solve(verbose=True)

planner.plot_cost_to_go(jmax=INF, is_3d=True)
planner.plot_control_law()

controller = planner.controller
diagram = controller @ plant

plant.x0 = X0
diagram.plot_diagram()
trajectory = diagram.compute_trajectory(tf=10.0)
diagram.plot_trajectory(trajectory)

planner.plot_control_law(trajectory=trajectory)
diagram.animate()
