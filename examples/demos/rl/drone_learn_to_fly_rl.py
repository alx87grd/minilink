"""Learn to fly: the planar drone hover task as a stochastic planning problem solved by PPO."""

import numpy as np

from minilink import CostFunction, Drone2D
from minilink.planning import (
    Gaussian,
    MonteCarloEvaluator,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
)

TRAINING_TIMESTEPS = 100_000  # a few seconds on a laptop CPU
DT = 0.05


# Plant: the learn-to-fly notebook's drone with thrusts normalized to [-1, 1]
class NormalizedDrone2D(Drone2D):
    """Planar drone with thrust inputs normalized between -1 and 1."""

    def __init__(self):
        super().__init__()
        self.params["mass"] = 1.0
        self.params["inertia"] = 0.1
        self.inputs["u"].lower_bound = np.array([-1.0, -1.0])
        self.inputs["u"].upper_bound = np.array([+1.0, +1.0])
        self.inputs["u"].units = ["%", "%"]
        self.weight = self.params["gravity"] * self.params["mass"]
        self.thrust2weight = 1.2
        self.state.upper_bound = np.array([10, 10, 2 * np.pi, 10, 10, 10])
        self.state.lower_bound = -self.state.upper_bound

    def thrust(self, u):
        """Normalized input to thruster forces: u = 0 hovers, +-1 is max / min."""
        return self.weight * ((self.thrust2weight - 1.0) * u + np.array([0.5, 0.5]))

    def f(self, x, u, t=0.0, params=None):
        return super().f(x, self.thrust(u), t, params)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return super().get_dynamic_geometry(x, self.thrust(u), t, params)


plant = NormalizedDrone2D()


# Cost: quadratic about hover at the origin, as in the notebook
class HoverCost(CostFunction):
    Q = np.diag([1.0, 1.0, 6.0, 0.1, 0.1, 0.1])
    R = np.diag([0.001, 0.001])

    def g(self, x, u, t=0.0, params=None):
        return x @ self.Q @ x + u @ self.R @ u

    def h(self, x, t=0.0, params=None):
        return 0.0


# The problem: Gaussian starts around hover (the notebook's exploration spread)
problem = StochasticPlanningProblem(
    plant,
    cost=HoverCost(),
    tf=np.inf,
    x0_distribution=Gaussian(np.zeros(6), [5.0, 5.0, 1.0, 1.0, 1.0, 0.2]),
)

# The notebook's PPO: one plant, 2048-step rollouts, raw state features
planner = ReinforcementLearningPlanner(
    problem, dt=DT, normalize=False, n_envs=1, n_steps=2048, batch_size=64
)
plan = planner.solve(timesteps=TRAINING_TIMESTEPS)
print(f"\n{plan.metadata.message} in {plan.metadata.solve_time_s:.1f} s")
planner.plot_learning_curve()

ppo_ctl = planner.get_controller()
ppo_ctl.plot_control_law(x_axis=2, y_axis=5, u_axis=0)  # T1 vs (theta, omega)
ppo_ctl.plot_control_law(x_axis=2, y_axis=5, u_axis=1)  # T2 vs (theta, omega)

report = MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1).evaluate(ppo_ctl)
print("Monte Carlo over the task's starts:", report)

plant.x0 = np.array([-1.0, -2.0, 1.0, 0.0, 0.0, 0.0])
cl_sys = ppo_ctl @ plant
cl_sys.name = "Drone with the learned law"
traj = cl_sys.compute_trajectory(tf=10.0, dt=0.01)
cl_sys.plot_trajectory(traj)
print("Final state:", np.round(traj.x[:, -1], 3))
cl_sys.animate(traj)
