"""Cart-pole swing-up with direct-collocation trajectory optimization."""

from __future__ import annotations

import argparse

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationPlanner,
)


sys = CartPole()

sys.inputs["u"].lower_bound[0] = -10.0
sys.inputs["u"].upper_bound[0] = 10.0

x_start = np.array([-2.0, 1.0, 0.0, 0.0])
x_goal = np.array([0.0, np.pi, 0.0, 0.0])

cost = QuadraticCost.from_system(
    sys,
    Q=np.diag([1.0, 1.0, 0.0, 0.0]),
    R=np.diag([1.0]),
    S=np.zeros((sys.n, sys.n)),
    xbar=x_goal,
    ubar=np.zeros(sys.m),
)
problem = PlanningProblem(
    sys=sys,
    x_start=x_start,
    x_goal=x_goal,
    cost=cost,
)

optimizer = ScipyMinimizeOptimizer(
    options={
        "disp": True,
        "maxiter": int(500),
        "ftol": 1e-6,
    }
)


planner = DirectCollocationPlanner(
    problem,
    tf=5.0,
    n_steps=50,
    optimizer=optimizer,
    compile_backend="numpy",
)


traj = planner.compute_solution()
result = planner.last_optimization_result

print(f"success: {result.success}")
print(f"message: {result.message}")
print(f"cost: {result.cost:.6g}")

planner.plot_solution(plot="xu")

planner.problem.sys.animate(traj)
