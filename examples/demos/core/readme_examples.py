"""The README examples, runnable, in the README's order."""

import numpy as np

from minilink import DynamicSystem

########################################################
# Write a model: f, and h to choose what the sensor measures


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
# Compose into a diagram: a catalog controller on the model above
from minilink import PID, Pendulum  # noqa: E402

loop = PID(2.0, 0.5, 1.0) @ plant
loop.compute_trajectory(tf=10.0)
loop.plot_diagram()
loop.plot_trajectory()

# A catalog plant: its parameters and start are data
pendulum = Pendulum()
pendulum.params["l"] = 2.0
pendulum.x0[0] = 2.0
pendulum.compute_trajectory(tf=10.0)
pendulum.animate()


########################################################
# One problem, every planner
from minilink import (  # noqa: E402
    BallSet,
    DynamicProgrammingPlanner,
    PlanningProblem,
    QuadraticCost,
    RRTPlanner,
    TrajectoryOptimizationPlanner,
    compare,
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
    U=plant.input_box,
)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=0.05)
rrt = RRTPlanner(problem, seed=0)
opt = TrajectoryOptimizationPlanner(
    problem, n_steps=40, transcription="direct_collocation"
)

vi_sol = vi.solve()  # value iteration: the policy is a controller
rrt_sol = rrt.solve()  # kinodynamic tree search, bang-bang inputs
opt_sol = opt.solve()  # direct collocation

loop = vi_sol.policy @ plant  # the script closes the loop
print(compare(problem, VI=vi_sol, RRT=rrt_sol, TrajOpt=opt_sol, dt=0.05))
loop.compute_trajectory(tf=10.0)
loop.plot_trajectory()
rrt.plot_tree(x_axis=0, y_axis=1)
opt.plot_solution(signals=("x", "u"))


########################################################
# Fast and differentiable with JAX, optional
from importlib.util import find_spec  # noqa: E402

if find_spec("jax") is None:
    print("JAX is not installed: the differentiable block is skipped")
else:
    import jax

    from minilink import Optimizer

    plant = Pendulum()
    ev = plant.compile(backend="jax")
    x0s = np.tile([1.0, 0.0], (1000, 1))  # 1000 rollouts released at 1 rad
    lengths = np.linspace(0.5, 2.0, 1000)  # one rod length per rollout

    A = plant.jacobian("f", "x", x_up)  # exact linearization
    S = plant.jacobian("f", "params", x_up)  # sensitivity to each physical parameter
    xs = ev.rollout_batch(
        x0s, n_steps=1000, dt=0.005, params=dict(plant.params, l=lengths)
    )  # a family of rod lengths, one call

    tuning = PlanningProblem(
        plant, x_start=[1.0, 0.0], cost=QuadraticCost.from_system(plant), tf=4.0
    )
    pe = tuning.compile(PID(2.0, 0.5, 1.0), dt=0.02)
    dJ = jax.grad(lambda p: pe.evaluate(p).objective)(
        pe.params
    )  # gradient w.r.t. every gain and plant param
    program = pe.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"])
    result = Optimizer(program, method="auto").solve()  # L-BFGS on the closed-loop cost
