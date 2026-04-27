"""Cart-pole swing-up with JAX-backed direct collocation."""

from __future__ import annotations

import numpy as np

from minilink.compile.jax_utils import configure_jax
from minilink.core.costs import JaxQuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.jax_direct_collocation import (
    JaxDirectCollocationOptions,
    JaxDirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=False)
# configure_jax(enable_x64=True)

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
        "ftol": 1e-2,
    }
)
planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=JaxDirectCollocationTranscription(
        JaxDirectCollocationOptions(
            tf=4.0,
            n_steps=50,
            use_gradient=True,
            use_hessian=False,
        )
    ),
    optimizer=optimizer,
    options=TrajectoryOptimizationOptions(compile_backend="jax"),
)

traj = planner.compute_solution()
result = planner.last_optimization_result

print(f"success: {result.success}")
print(f"message: {result.message}")
if result.cost is not None:
    print(f"cost: {result.cost:.6g}")


planner.plot_solution(plot="xu")

planner.problem.sys.animate(traj)
