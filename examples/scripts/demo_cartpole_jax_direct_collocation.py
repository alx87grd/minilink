"""Cart-pole swing-up with JAX-backed direct collocation."""

from __future__ import annotations

import argparse

import numpy as np

from minilink.core.costs import JaxQuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.jax_direct_collocation import (
    JaxDirectCollocationPlanner,
)


sys = JaxCartPole()

sys.inputs["u"].lower_bound[0] = -10.0
sys.inputs["u"].upper_bound[0] = 10.0

x_start = np.array([-2.0, 1.0, 0.0, 0.0])
x_goal = np.array([0.0, np.pi, 0.0, 0.0])

cost = JaxQuadraticCost.from_system(
    sys,
    Q=np.diag([1.0, 1.0, 0.0, 0.0]),
    R=np.diag([0.01]),
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
        "maxiter": 500,
        "ftol": 1e-1,
    }
)
planner = JaxDirectCollocationPlanner(
    problem,
    tf=4.0,
    n_steps=50,
    optimizer=optimizer,
    compile_backend="jax",
    use_gradient=True,
    use_hessian=False,
    enable_x64=True,
)

traj = planner.compute_solution()
result = planner.last_optimization_result

print(f"success: {result.success}")
print(f"message: {result.message}")
if result.cost is not None:
    print(f"cost: {result.cost:.6g}")


planner.plot_solution(plot="xu")

planner.problem.sys.animate(traj)
