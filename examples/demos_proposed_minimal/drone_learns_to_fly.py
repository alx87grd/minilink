"""A drone learns to fly: a neural law trained by reinforcement to bring a planar drone back to hover."""

import numpy as np

from minilink import (
    CostFunction,
    Drone2D,
    Gaussian,
    PlanningProblem,
    ReinforcementLearningPlanner,
    evaluate,
)

TRAINING_TIMESTEPS = 600_000  # about 40 s on a laptop CPU

# The plant: the planar drone, its two thrusts normalized so that u = 0 hovers and
# u = ±1 is the least and the most each thruster gives, 1.2 times its share of the weight


class NormalizedDrone(Drone2D):
    def __init__(self):
        super().__init__()
        self.params["mass"] = 1.0
        self.params["inertia"] = 0.1
        self.inputs["u"].lower = np.array([-1.0, -1.0])
        self.inputs["u"].upper = np.array([1.0, 1.0])
        self.state.upper = np.array([10.0, 10.0, 2.0 * np.pi, 10.0, 10.0, 10.0])
        self.state.lower = -self.state.upper

    def thrusts(self, u, params):
        mass = params["mass"]
        gravity = params["gravity"]

        # T = m g (½ + 0.2 u): half the weight each at u = 0
        T = mass * gravity * (0.5 + 0.2 * u)

        return T

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        dx = super().f(x, self.thrusts(u, params), t, params)

        return dx

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        return super().get_dynamic_geometry(x, self.thrusts(u, params), t, params)


plant = NormalizedDrone()

# The cost: quadratic about the hover at the origin, the tilt weighed most


class HoverCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        Q = np.diag([1.0, 1.0, 6.0, 0.1, 0.1, 0.1])
        R = 0.001 * np.eye(2)

        g = x @ Q @ x + u @ R @ u

        return g

    def h(self, x, t=0.0, params=None):
        return 0.0


# The task: a start drawn up to several metres away, tilted and moving

problem = PlanningProblem(
    plant,
    cost=HoverCost(),
    x0=Gaussian(np.zeros(6), [5.0, 5.0, 1.0, 1.0, 1.0, 0.2]),
    U=plant.input_box,
    horizon=np.inf,
)

# The learner: PPO on a small network that reads the state and commands the thrusts

planner = ReinforcementLearningPlanner(
    problem,
    dt=0.05,
    normalize=False,
    n_rollouts=1,
    n_steps_per_update=2048,
    batch_size=64,
    gamma=0.99,
)
planner.solve(n_timesteps=TRAINING_TIMESTEPS)
planner.plot_learning_curve()

# The learned law is a controller block: scored on 100 starts drawn from the problem,
# then flown from one of them

controller = planner.controller
print(evaluate(problem, controller, dt=0.05, n_trials=100, episode_length=10.0, key=1))

plant.x0 = np.array([-1.0, -2.0, 1.0, 0.0, 0.0, 0.0])
closed_loop = controller @ plant
traj = closed_loop.compute_trajectory(tf=10.0, dt=0.01)
closed_loop.plot_trajectory(traj)
closed_loop.animate(traj)
