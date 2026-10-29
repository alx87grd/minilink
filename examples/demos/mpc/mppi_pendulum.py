"""MPPI swings the pendulum up: the path-integral planner closed in the loop by the MPC block."""

import jax.numpy as jnp
import numpy as np

from minilink import CostFunction, Pendulum, PlanningProblem
from minilink.analysis import discretize
from minilink.control import ModelPredictiveController
from minilink.planning import MPPIPlanner

DT = 0.05  # control period, s
HORIZON = 2.0  # MPPI horizon, s (40 periods)
TF_SIM = 6.0  # closed-loop run, s
K = 1024  # sampled input sequences per tick
LAMBDA = 0.1  # temperature of the softmin
SIGMA = 3.0  # std of the input noise, Nm
TORQUE = 4.0  # Nm, below m g l: the pendulum must pump energy

plant = Pendulum()
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])


# The wrapped angle to upright: 1 + cos(theta) gives the sampler no signal at the bottom
class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        theta, dtheta = x
        e = jnp.arctan2(jnp.sin(theta - jnp.pi), jnp.cos(theta - jnp.pi))
        return e**2 + 0.1 * dtheta**2 + 0.001 * u[0] ** 2

    def h(self, x, t=0.0, params=None):
        return 0.0


problem = PlanningProblem(
    plant, cost=SwingUpCost(), tf=HORIZON, x_start=[0.0, 0.0], U=plant.input_box
)
planner = MPPIPlanner(problem, dt=DT, n_samples=K, temperature=LAMBDA, noise=SIGMA**2)
print(planner.solve().solver)

mpc = ModelPredictiveController(planner, delay=False)
loop = (mpc % DT) @ discretize(plant, DT)
loop.compute_trajectory(tf=TF_SIM)
loop.plot_trajectory()
loop.animate()
