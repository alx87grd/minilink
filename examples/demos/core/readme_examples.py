"""The README examples, runnable."""

import numpy as np

from minilink import ImpedanceController, Pendulum

controller = ImpedanceController()
plant = Pendulum()

plant.x0[0] = 2.0
plant.params["l"] = 5.0

diagram = controller @ plant
diagram.compute_trajectory(tf=10.0)
diagram.plot_diagram()
diagram.plot_trajectory()
diagram.animate()


########################################################
# What is a System: write f, and h to choose what the sensor measures
from minilink import DynamicSystem  # noqa: E402


class MassSpringDamper(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=1)

    def f(self, x, u, t=0, params=None):
        m, k, b = 1.0, 4.0, 0.3
        p, v = x

        # m p̈ + b ṗ + k p = u
        a = (u[0] - b * v - k * p) / m

        return np.array([v, a])

    def h(self, x, u, t=0, params=None):
        p, v = x

        return np.array([p])  # the sensor reads the position only


plant = MassSpringDamper()
plant.x0[0] = 1.0
plant.compute_trajectory(tf=10.0)
plant.plot_trajectory()


########################################################
# One model, every tool: one problem, three planners
from minilink import (  # noqa: E402
    BallSet,
    DynamicProgrammingPlanner,
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


########################################################
# Differentiable and compiled
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
