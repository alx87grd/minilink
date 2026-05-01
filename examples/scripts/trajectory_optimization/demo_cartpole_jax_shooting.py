"""Cart-pole stabilization with JAX-backed single shooting."""

import numpy as np

from minilink.compile.jax_utils import configure_jax
from minilink.core.costs import JaxQuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
from minilink.optimization.optimizer import Optimizer
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.jax_shooting import (
    JaxShootingOptions,
    JaxShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

sys = JaxCartPole()
sys.inputs["u"].lower_bound[0] = -20.0
sys.inputs["u"].upper_bound[0] = 20.0

x_start = np.array([0.0, 0.2, 0.0, 0.0])
x_goal = np.zeros(sys.n)

cost = JaxQuadraticCost.from_system(
    sys,
    Q=np.diag([1.0, 20.0, 0.1, 0.1]),
    R=np.diag([0.01]),
    S=np.diag([20.0, 80.0, 1.0, 1.0]),
    xbar=x_goal,
    ubar=np.zeros(sys.m),
)
problem = PlanningProblem(
    sys=sys,
    x_start=x_start,
    x_goal=x_goal,
    cost=cost,
)

planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=JaxShootingTranscription(
        JaxShootingOptions(
            tf=2.0,
            n_steps=31,
            use_gradient=True,
        )
    ),
    optimizer=Optimizer(
        backend="scipy",
        options={
            "disp": True,
            "maxiter": 200,
            "ftol": 1e-7,
        },
    ),
    options=TrajectoryOptimizationOptions(compile_backend="jax"),
)

traj = planner.compute_solution()
result = planner.last_optimization_result

print(f"success: {result.success}")
print(f"message: {result.message}")
print(f"cost: {result.cost:.6g}")

planner.plot_solution(plot="xu")
planner.problem.sys.animate(traj)
