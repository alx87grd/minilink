import numpy as np

from minilink.control.lqr import lqr_at_operating_point
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.planning.policy_synthesis import plotting
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingPlanner,
)
from minilink.planning.policy_synthesis.policy_eval import PolicyEvaluator
from minilink.planning.problems import PlanningProblem

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
problem = PlanningProblem(plant, x_goal=UPRIGHT, cost=cost)

grid = StateSpaceGrid(problem, x_grid_shape=(101, 101), u_grid_shape=(11,), dt=0.05)
planner = DynamicProgrammingPlanner(
    problem,
    grid=grid,
    options=DynamicProgrammingOptions(alpha=1.0, tol=0.1, max_iterations=2000),
)
planner.solve()
planner.clean_infeasible_set()

lqr_plant = Pendulum()
lqr_plant.state.lower_bound = np.array([LO, LO])
lqr_plant.state.upper_bound = np.array([HI, HI])
lqr_plant.inputs["u"].lower_bound = np.array([-TORQUE])
lqr_plant.inputs["u"].upper_bound = np.array([TORQUE])
lqr = lqr_at_operating_point(lqr_plant, UPRIGHT, Q, R)

# The LQR block is the policy and the law: no params unpacking needed.
J_lqr = PolicyEvaluator(problem, grid=grid, policy=lqr, options=planner.options).solve()

planner.plot_cost2go(jmax=INF, show_3d=True)
plotting.plot_value(grid, J_lqr, vmax=INF, title="LQR cost-to-go")

planner.plot_policy()
lqr.plot_control_law(
    bounds=((LO, HI), (LO, HI)),
    vmin=-TORQUE,
    vmax=TORQUE,
    title="LQR control law",
)
planner.get_controller().plot_control_law(title="VI control law (interpolated)")

sim_vi = Pendulum()
sim_vi.state.lower_bound = np.array([LO, LO])
sim_vi.state.upper_bound = np.array([HI, HI])
sim_vi.inputs["u"].lower_bound = np.array([-TORQUE])
sim_vi.inputs["u"].upper_bound = np.array([TORQUE])
sim_lqr = Pendulum()
sim_lqr.state.lower_bound = np.array([LO, LO])
sim_lqr.state.upper_bound = np.array([HI, HI])
sim_lqr.inputs["u"].lower_bound = np.array([-TORQUE])
sim_lqr.inputs["u"].upper_bound = np.array([TORQUE])

vi_diagram = planner.get_controller() @ sim_vi
lqr_diagram = lqr @ sim_lqr
vi_diagram.name = "Pendulum swing-up (value iteration)"
lqr_diagram.name = "Pendulum swing-up (LQR)"

sim_vi.x0 = X0.copy()
sim_lqr.x0 = X0.copy()
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
