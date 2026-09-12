"""Cart-pole swing-up: a stochastic planning problem solved by the RL planner."""

import jax.numpy as jnp
import numpy as np

from minilink import CartPole, CostFunction
from minilink.control import angle_features
from minilink.planning import (
    MonteCarloEvaluator,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
    Uniform,
)

TRAINING_TIMESTEPS = 1_000_000  # about 30 s on a laptop CPU
DT = 0.05

# Plant: catalog cart-pole, theta = 0 hanging, pi upright, force |F| <= 10 N.
# The state bounds are the training box; leaving it truncates the episode.
# The angle is unbounded: the policy sees it through periodic features.
plant = CartPole()
plant.inputs["u"].lower_bound = np.array([-10.0])
plant.inputs["u"].upper_bound = np.array([10.0])
plant.state.lower_bound = np.array([-5.0, -1e3, -20.0, -30.0])
plant.state.upper_bound = np.array([5.0, 1e3, 20.0, 30.0])


# Cost: periodic angle term, small cart, velocity and effort penalties (the
# velocity weight stays small so the fast swing is not punished)
class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        x_cart, theta, dx, dtheta = x
        return (
            (1.0 + jnp.cos(theta))  # traces under JAX
            + 0.1 * x_cart**2
            + 0.001 * (dx**2 + dtheta**2)
            + 0.001 * u[0] ** 2
        )

    def h(self, x, t=0.0, params=None):
        return 0.0


problem = StochasticPlanningProblem(
    plant,
    cost=SwingUpCost(),
    tf=np.inf,
    x0_distribution=Uniform([-1.0, -np.pi, -1.0, -1.0], [1.0, np.pi, 1.0, 1.0]),
)

planner = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=angle_features(angles=[1], scales={2: 0.2, 3: 0.1}),
    hidden=(64, 64),
    algorithm="ppo",
    n_envs=16,
    n_steps=256,
    batch_size=512,
    gamma=0.99,
)
plan = planner.solve(timesteps=TRAINING_TIMESTEPS)
print(f"\n{plan.metadata.message} in {plan.metadata.solve_time_s:.1f} s")
planner.plot_learning_curve()

ppo_ctl = planner.get_controller()
ppo_ctl.plot_control_law(x_axis=1, y_axis=3, u_axis=0)  # force vs (theta, dtheta)

report = MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1).evaluate(ppo_ctl)
print("Monte Carlo over the task's starts:", report)

plant.x0 = np.array([0.0, 0.05, 0.0, 0.0])  # hanging, a tiny tip
cl_sys = ppo_ctl @ plant
cl_sys.name = "Cart-pole with the learned law"
traj = cl_sys.compute_trajectory(tf=10.0, dt=0.01)
cl_sys.plot_trajectory(traj)
angle_error = np.abs(np.mod(traj.x[1], 2 * np.pi) - np.pi)
print(
    "Angle error to upright, last 2 s:",
    round(float(angle_error[-200:].max()), 3),
    "rad",
)
print("Cart excursion max |x| =", round(float(np.abs(traj.x[0]).max()), 2), "m")
cl_sys.animate(traj)
