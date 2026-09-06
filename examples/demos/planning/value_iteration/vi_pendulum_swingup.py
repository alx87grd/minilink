import numpy as np

from minilink import DynamicProgrammingPlanner, Pendulum, PlanningProblem, QuadraticCost

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

planner = DynamicProgrammingPlanner(
    problem,
    x_grid=(201, 201),
    u_grid=(21,),
    dt=0.05,
    tol=0.1,
    max_iterations=2000,
    out_of_bound_cost=INF,
    verbose=True,
)
planner.solve()

planner.plot_cost2go(jmax=INF, show_3d=True)
planner.plot_policy()

controller = planner.get_controller()
diagram = controller @ plant

plant.x0 = X0
diagram.plot_diagram()
trajectory = diagram.compute_trajectory(tf=10.0)
diagram.plot_trajectory(trajectory)

planner.plot_policy(trajectory=trajectory)
diagram.animate()
