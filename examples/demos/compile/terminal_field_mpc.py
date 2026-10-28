"""A short-horizon MPC ended by the LQR value: ``QuadraticField(x̄, S).as_terminal_cost()`` from the Riccati ``S``."""

import numpy as np
from scipy.linalg import solve_continuous_are

from minilink import InvertedPendulum, PlanningProblem, QuadraticCost
from minilink.analysis import linearize
from minilink.control import ModelPredictiveController, lqr
from minilink.core import QuadraticField
from minilink.planning import TrajectoryOptimizationPlanner

TF = 4.0  # the scored run, s
DT = 0.1  # the MPC's period, s
HORIZON = 0.3  # the plan's horizon: three periods, too short to see the pendulum settle
ITERATIONS = 40  # Adam steps per tick: a fixed count, so the tick traces

# The task: bring the inverted pendulum back up from 0.4 rad, within the torque limits
plant = InvertedPendulum()
Q = np.diag([10.0, 1.0])
R = 0.1 * np.eye(1)
x_start = np.array([0.4, 0.0])
score = QuadraticCost.from_system(plant, Q=Q, R=R)
problem = PlanningProblem(plant, cost=score, x_start=x_start, tf=TF, U=plant.input_box)

# The LQR value about the top, V(x) = xᵀ S x, S the Riccati solution of the linearized plant
model = linearize(plant)
S = solve_continuous_are(model.A(), model.B(), Q, R)
V = QuadraticField(model.xbar, S)


def mpc_ended_by(cost):
    """The MPC planning ``HORIZON`` seconds ahead on ``cost``."""
    planner = TrajectoryOptimizationPlanner(
        PlanningProblem(
            plant, cost=cost, x_start=x_start, tf=HORIZON, U=plant.input_box
        ),
        n_steps=3,
        transcription="single_shooting",
        backend="jax",
        method="adam",
        iterations=ITERATIONS,
        optimizer_options={"learning_rate": 0.2},
    )
    return ModelPredictiveController(planner)


# The same short MPC, without and with the LQR value as its terminal cost
short = problem.compile(mpc_ended_by(score) % DT, backend="jax")
ended = problem.compile(mpc_ended_by(score + V.as_terminal_cost()) % DT, backend="jax")
print("short horizon, no terminal cost:", short.evaluate(short.params))
print("short horizon, ended by V      :", ended.evaluate(ended.params))

# The infinite-horizon law the terminal cost stands for, sampled at the same period: it
# asks for more torque than the plant has, so its run is reported failed
law = problem.compile(lqr(plant, Q, R) % DT, backend="jax")
print("LQR at the same period         :", law.evaluate(law.params))

ended.loop.plot_trajectory(ended.trajectory(ended.params))
