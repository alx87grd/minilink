"""Rocket landing: gimballed thrust, ground termination with a crash cost, learned by the RL planner."""

import numpy as np

from minilink import CostFunction, Rocket
from minilink.planning import (
    Gaussian,
    MonteCarloEvaluator,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
)

TRAINING_TIMESTEPS = 4_000_000  # about 80 s on a laptop CPU
DT = 0.05
TF = 15.0  # a finite horizon: the episode ends at tf with the terminal cost h
GIMBAL = 0.05  # rad
THRUST_TO_WEIGHT = 2.0

# Plant: catalog planar rocket (y up, ground at y = 0, c.g. 1 m above the
# nozzle) with a rocket-like inertia; one-sided thrust and a gimbal angle.
plant = Rocket()
plant.params["inertia"] = 1000.0  # catalog: 100, which tumbles at a 0.05 rad gimbal
weight = plant.params["mass"] * plant.params["gravity"]
plant.inputs["u"].lower_bound = np.array([0.0, -GIMBAL])
plant.inputs["u"].upper_bound = np.array([THRUST_TO_WEIGHT * weight, GIMBAL])
plant.state.lower_bound = np.array([-100.0, 0.0, -1.0, -50.0, -50.0, -5.0])
plant.state.upper_bound = np.array([100.0, 200.0, 1.0, 50.0, 50.0, 5.0])

X_LANDED = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])  # upright on the pad, at rest


# Cost: quadratic about the landed state, scaled so that returns are of order
# 100; h = 0 at the horizon. The crash is the problem's exit rule below.
class LandingCost(CostFunction):
    Q = 0.1 * np.diag([1.0, 1.0, 10.0, 0.1, 0.1, 1.0])
    R = 0.1 * np.diag([1e-8, 1.0])

    def g(self, x, u, t=0.0, params=None):
        dx = x - X_LANDED
        return dx @ self.Q @ dx + u @ self.R @ u

    def h(self, x, t=0.0, params=None):
        return 0.0


# The problem: starts around 20 m altitude; touching the ground (y < 0)
# terminates the episode and costs 100, more than a gentle landing does.
problem = StochasticPlanningProblem(
    plant,
    cost=LandingCost(),
    tf=TF,
    x0_distribution=Gaussian(
        [0.0, 20.0, 0.0, 0.0, 0.0, 0.0], [10.0, 8.0, 0.3, 2.0, 2.0, 0.3]
    ),
    on_exit="terminate",
    exit_cost=100.0,
)

# Policy features: the error to the landed state, metres scaled by 0.25
FEATURE_SCALE = np.array([0.25, 0.25, 1.0, 0.25, 0.25, 1.0])

planner = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=lambda x: (x - X_LANDED) * FEATURE_SCALE,
    hidden=(64, 64),
    algorithm="ppo",
    n_envs=64,
    n_steps=32,
    batch_size=256,
    learning_rate=1e-3,
    gamma=0.995,
    log_std_init=-1.0,  # gentle exploration: a twitchy attitude loop
)
plan = planner.solve(timesteps=TRAINING_TIMESTEPS)
print(f"\n{plan.metadata.message} in {plan.metadata.solve_time_s:.1f} s")
planner.plot_learning_curve()

ppo_ctl = planner.get_controller()
ppo_ctl.plot_control_law(x_axis=1, y_axis=4, u_axis=0)  # thrust vs (altitude, vy)

report = MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1).evaluate(ppo_ctl)
print("Monte Carlo over the task's starts (failure = touched the ground):", report)

plant.x0 = np.array([10.0, 30.0, 0.0, 0.0, 0.0, 0.0])
cl_sys = ppo_ctl @ plant
cl_sys.name = "Rocket with the learned law"
traj = cl_sys.compute_trajectory(tf=TF, dt=0.01)
cl_sys.plot_trajectory(traj)
final = traj.x[:, -1] - X_LANDED
print("Final position error:", round(float(np.hypot(final[0], final[1])), 2), "m")
print("Final speed:", round(float(np.hypot(final[3], final[4])), 2), "m/s")
print(
    "Lowest altitude of the c.g.:",
    round(float(traj.x[1].min()), 2),
    "m (1.0 = on the pad)",
)
cl_sys.animate(traj)
