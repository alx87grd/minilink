"""The README examples, runnable."""

from minilink import ImpedanceController, Pendulum

controller = ImpedanceController()
plant = Pendulum()

plant.x0[0] = 2.0
plant.params["l"] = 5.0
plant.params["m"] = 1.0

diagram = controller @ plant
diagram.compute_trajectory(tf=10.0)
diagram.plot_diagram()
diagram.plot_trajectory()
diagram.animate()


########################################################
# What is a System
import numpy as np  # noqa: E402

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
        params = self.params if params is None else params
        m, k, c = params["m"], params["k"], params["c"]
        p, v = x

        # m p̈ + c ṗ + k p = u
        a = (u[0] - c * v - k * p) / m

        return np.array([v, a])

    def tf(self, x, u, t=0, params=None):
        return {"body": translation(x[0], 0.0, 0.0)}


msd = MassSpringDamper()
msd.x0[0] = 1.0
loop = Step(final_value=np.array([10.0]), step_time=2.0) >> msd
loop.compute_trajectory(tf=20.0)
loop.animate()  # renderer="plotly" | "meshcat" | "pygame"
# msd.game()    # keyboard drives u, live


########################################################
# One model, every tool
import numpy as np  # noqa: E402

from minilink import (  # noqa: E402
    BallSet,
    DynamicProgrammingPlanner,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    RRTPlanner,
    TrajectoryOptimizationPlanner,
)

plant = Pendulum()
x_down, x_up = np.array([0.0, 0.0]), np.array([np.pi, 0.0])
problem = PlanningProblem(
    sys=plant,
    x_start=x_down,
    x_goal=x_up,
    Xf=BallSet(x_up, 0.2),
    tf=4.0,
    cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), xbar=x_up),
    X=plant.state.box,
)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=0.05)
vi.solve()  # value iteration on a grid
loop = vi.get_controller() @ plant  # the policy is a controller

rrt = RRTPlanner(problem, seed=0)
tree_traj = rrt.solve().trajectory  # kinodynamic tree search, bang-bang inputs

opt = TrajectoryOptimizationPlanner(
    problem, n_steps=40, transcription="direct_collocation"
)
opt_traj = opt.solve().trajectory  # direct collocation


########################################################
# Differentiable and compiled
import numpy as np  # noqa: E402

from minilink import Pendulum  # noqa: E402

plant = Pendulum()
ev = plant.compile(backend="jax")
x_bar = np.array([np.pi, 0.0])  # upright
x0s = np.tile([1.0, 0.0], (1000, 1))  # 1000 rollouts released at 1 rad
lengths = np.linspace(0.5, 2.0, 1000)  # one rod length per rollout

A = plant.jacobian("f", "x", x_bar)  # exact linearization
S = plant.jacobian("f", "params", x_bar)  # sensitivity to each physical parameter
xs = ev.rollout_batch(
    x0s, n_steps=1000, dt=0.005, params=dict(plant.params, l=lengths)
)  # a family of rod lengths, one call
