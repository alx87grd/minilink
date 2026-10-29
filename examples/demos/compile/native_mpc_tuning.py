"""Tune an MPC by gradient: a native fixed-iteration planner, its closed loop compiled once, its cost weights optimized."""

import numpy as np

from minilink import Optimizer, Pendulum, PlanningProblem, QuadraticCost
from minilink.control import ModelPredictiveController
from minilink.planning import TrajectoryOptimizationPlanner

TF = 3.0  # the scored run, s
DT = 0.1  # the MPC's period, s
HORIZON = 1.0  # the plan's horizon, s
ITERATIONS = 30  # Adam steps per tick: a fixed count, so the tick traces

# The task the loop is scored on: bring the pendulum down from 1 rad with little effort
plant = Pendulum()
score = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1))
problem = PlanningProblem(
    plant, cost=score, x_start=np.array([1.0, 0.0]), tf=TF, U=plant.input_box
)

# The MPC's own plan: a shorter horizon and its own weights, the ones to tune
plan_cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 1.0]), R=np.eye(1))
planner = TrajectoryOptimizationPlanner(
    PlanningProblem(
        plant,
        cost=plan_cost,
        x_start=np.array([1.0, 0.0]),
        tf=HORIZON,
        U=plant.input_box,
    ),
    n_steps=10,
    transcription="single_shooting",
    backend="jax",
    method="adam",
    iterations=ITERATIONS,
    optimizer_options={"learning_rate": 0.05},
)
mpc = ModelPredictiveController(planner)

# The closed loop, one scan on JAX: J of the plan's weights
pe = problem.compile(mpc % DT, substeps=1, backend="jax")
start = pe.params
print(pe.evaluate(start))

# The tuning program over the plan's weights, solved by the class default (L-BFGS)
wrt = ["controller.cost.Q", "controller.cost.R"]
best = Optimizer(pe.program(wrt, params=start), method="auto").solve()
print(best)

# The tuned loop, judged and drawn
tuned = pe.unpack(best.z, wrt, params=start)
print(pe.evaluate(tuned))
pe.loop.plot_trajectory(pe.trajectory(tuned))
