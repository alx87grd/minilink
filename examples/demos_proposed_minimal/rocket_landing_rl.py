"""Rocket landing learned by reinforcement: a gimballed thrust, a random start, a crash that costs."""

import numpy as np

from minilink import (
    CostFunction,
    Gaussian,
    PlanningProblem,
    ReinforcementLearningPlanner,
    Rocket,
    evaluate,
)

TRAINING_TIMESTEPS = 4_000_000  # about 80 s on a laptop CPU
TF = 15.0  # [s] an episode: the landing has to be done by then

# The plant: the planar rocket (y up, the ground at y = 0, the centre of gravity 1 m
# above the nozzle), its thrust one-sided up to twice its weight, its gimbal ±0.05 rad

gimbal = 0.05  # [rad]

plant = Rocket()
plant.params["inertia"] = 1000.0  # [kg m^2] a slender body: it does not tumble
weight = plant.params["mass"] * plant.params["gravity"]
plant.inputs["u"].lower = np.array([0.0, -gimbal])
plant.inputs["u"].upper = np.array([2.0 * weight, gimbal])
plant.state.lower = np.array([-100.0, 0.0, -1.0, -50.0, -50.0, -5.0])
plant.state.upper = np.array([100.0, 200.0, 1.0, 50.0, 50.0, 5.0])

# The cost: quadratic about the landed state, upright on the pad and at rest

x_landed = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])


class LandingCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        Q = 0.1 * np.diag([1.0, 1.0, 10.0, 0.1, 0.1, 1.0])
        R = 0.1 * np.diag([1e-8, 1.0])
        dx = x - x_landed

        g = dx @ Q @ dx + u @ R @ u

        return g

    def h(self, x, t=0.0, params=None):
        return 0.0


# The task: a start drawn around 20 m of altitude; leaving the state box, the ground
# included (y < 0), ends the episode and costs 100, more than a gentle landing does

problem = PlanningProblem(
    plant,
    cost=LandingCost(),
    x0=Gaussian([0.0, 20.0, 0.0, 0.0, 0.0, 0.0], [10.0, 8.0, 0.3, 2.0, 2.0, 0.3]),
    X=plant.state.box,
    U=plant.input_box,
    horizon=TF,
    penalize_failure=True,
    failure_cost=100.0,
)

# The learner: PPO on a small network that reads the error to the landed state, its
# lengths scaled by 0.25

feature_scale = np.array([0.25, 0.25, 1.0, 0.25, 0.25, 1.0])

planner = ReinforcementLearningPlanner(
    problem,
    dt=0.05,
    features=lambda x: (x - x_landed) * feature_scale,
    hidden=(64, 64),
    n_rollouts=64,
    n_steps_per_update=32,
    batch_size=256,
    learning_rate=1e-3,
    gamma=0.995,
    log_std_init=-1.0,  # gentle exploration: the attitude loop is twitchy
)
solution = planner.solve(n_timesteps=TRAINING_TIMESTEPS)
print(solution.record)
planner.plot_learning_curve()

# The learned law: scored on 100 starts drawn from the problem, then flown from one

controller = planner.controller
print(evaluate(problem, controller, dt=0.05, n_trials=100, key=1))
controller.plot_control_law(x_axis=1, y_axis=4, u_axis=0)  # thrust over altitude and vy

plant.x0 = np.array([10.0, 30.0, 0.0, 0.0, 0.0, 0.0])
closed_loop = controller @ plant
traj = closed_loop.compute_trajectory(tf=TF, dt=0.01)
closed_loop.plot_trajectory(traj)
closed_loop.animate(traj)
