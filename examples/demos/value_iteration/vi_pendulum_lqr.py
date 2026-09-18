"""Pendulum swing-up: value iteration vs LQR."""

import numpy as np

from minilink import (
    DynamicProgrammingPlanner,
    LQRPlanner,
    Pendulum,
    PlanningProblem,
    PolicyEvaluator,
    QuadraticCost,
    compare,
)

INF = 500.0
UPRIGHT = np.array([-np.pi, 0.0])
TORQUE = 5.0
Q = np.eye(2)
R = np.eye(1)
X0 = np.array([-0.1, 0.0])
LO, HI = -2.0 * np.pi, 2.0 * np.pi

plant = Pendulum()
plant.state.lower_bound = np.array([LO, LO])
plant.state.upper_bound = np.array([HI, HI])
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])

cost = QuadraticCost.from_system(plant, xbar=UPRIGHT, Q=Q, R=R)
problem = PlanningProblem(plant, x_start=X0, x_goal=UPRIGHT, cost=cost)

# Two planners on one problem
planner = DynamicProgrammingPlanner(
    problem, x_grid=(101, 101), u_grid=(11,), dt=0.05, tol=0.1, max_iterations=2000
)
grid = planner.grid  # shared with the LQR policy evaluation below
sol_vi = planner.solve()
sol_lqr = LQRPlanner(problem).solve()

# What each method claims: its law and its own cost-to-go, on one scale
race = compare(VI=sol_vi, LQR=sol_lqr)
print(race)
race.plot_control_law()
race.plot_cost_to_go(jmax=INF)
planner.plot_cost2go(jmax=INF, show_3d=True)

# The LQR law measured on the nonlinear plant: the Bellman expectation equation on VI's grid
J_lqr = PolicyEvaluator(
    problem, grid=grid, policy=sol_lqr, options=planner.options
).solve()
grid.plot_value(J_lqr, vmax=INF, title="LQR cost-to-go, measured")

# Close the loops, explicitly, and simulate each from the same start
plant.x0 = X0
vi_diagram = sol_vi.policy @ plant
lqr_diagram = sol_lqr.policy @ plant
vi_diagram.name = "Pendulum swing-up (value iteration)"
lqr_diagram.name = "Pendulum swing-up (LQR)"
vi_diagram.plot_diagram()
lqr_diagram.plot_diagram()
vi_traj = vi_diagram.compute_trajectory(tf=10.0)
lqr_traj = lqr_diagram.compute_trajectory(tf=10.0)
vi_diagram.plot_trajectory(vi_traj)
lqr_diagram.plot_trajectory(lqr_traj)

# print("VI  cost-to-go at x0:", round(planner.value_at(X0), 2))
# print("LQR cost-to-go at x0:", round(float(grid.interpolate(J_lqr, X0.reshape(1, -1))[0]), 2))
# print("VI  | final angle error:", round(abs(vi_traj.x[0, -1] - UPRIGHT[0]), 3), "rad")
# print("LQR | final angle error:", round(abs(lqr_traj.x[0, -1] - UPRIGHT[0]), 3), "rad")
