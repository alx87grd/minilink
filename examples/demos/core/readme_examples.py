"""The README examples, runnable."""

import numpy as np

from minilink import ImpedanceController, Pendulum

controller = ImpedanceController()  # u = Kp * (r - theta) - Kd * theta_dot
plant = Pendulum()  # theta_ddot = -(g / l) * sin(theta) + tau / (m * l**2)

plant.x0[0] = 2.0
plant.params["l"] = 5.0
plant.params["m"] = 1.0

diagram = controller @ plant
diagram.compute_trajectory(tf=10.0)
diagram.plot_diagram()
diagram.plot_trajectory()
diagram.animate()


########################################################
# What is a System: f for the dynamics, tf and a skin for the animation
from minilink import DynamicSystem, Step  # noqa: E402
from minilink.core.kinematics import translation  # noqa: E402
from minilink.graphical.animation.primitives import Box, ground_line  # noqa: E402


class MassSpringDamper(DynamicSystem):
    # m p'' + c p' + k p = u

    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2)
        self.params = {"m": 1.0, "k": 4.0, "c": 0.3}
        self.skin = lambda sys: {
            "world": [ground_line(length=8.0)],
            "body": [Box(length_x=0.6, length_y=0.6, length_z=0.1)],
        }
        self.camera_scale = 4.0

    def f(self, x, u, t=0, params=None):
        p = self.params if params is None else params
        pos, vel = x
        acc = (u[0] - p["c"] * vel - p["k"] * pos) / p["m"]
        return np.array([vel, acc])

    def tf(self, x, u, t=0, params=None):
        return {"body": translation(x[0], 0.0, 0.0)}


msd = MassSpringDamper()
msd.x0[0] = 1.0
loop = Step(final_value=np.array([10.0]), step_time=2.0) >> msd
loop.compute_trajectory(tf=20.0)
loop.animate()  # renderer="plotly" | "meshcat" | "pygame"
# msd.game()    # keyboard drives u, live (opens a window)


########################################################
# One problem, three planners
from minilink import (  # noqa: E402
    BallSet,
    DynamicProgrammingPlanner,
    PlanningProblem,
    QuadraticCost,
    RRTPlanner,
    TrajectoryOptimizationPlanner,
)

plant = Pendulum()
plant.inputs["u"].lower_bound[:] = -5.0
plant.inputs["u"].upper_bound[:] = 5.0
plant.state.lower_bound = np.array([-2.0 * np.pi, -12.0])
plant.state.upper_bound = np.array([2.0 * np.pi, 12.0])

x_down, x_up = np.array([0.0, 0.0]), np.array([np.pi, 0.0])
problem = PlanningProblem(
    sys=plant,
    x_start=x_down,
    x_goal=x_up,
    Xf=BallSet(x_up, 0.2),
    tf=4.0,
    cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), xbar=x_up),
)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=0.05)
vi.solve()  # value iteration on a grid
loop = vi.get_controller() @ plant  # the policy is a controller
loop.compute_trajectory(tf=10.0)
loop.plot_trajectory()

rrt = RRTPlanner(problem, seed=0)
tree_traj = rrt.solve().trajectory  # kinodynamic tree search, bang-bang inputs
rrt.plot_tree(x_axis=0, y_axis=1)

opt = TrajectoryOptimizationPlanner(
    problem, n_steps=40, transcription="direct_collocation"
)
opt_traj = opt.solve().trajectory  # direct collocation
opt.plot_solution(signals=("x", "u"))
