"""Cart-pole swing-up with multiple shooting."""

import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.optimization.optimizer import Optimizer
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.multiple_shooting import (
    MultipleShootingOptions,
    MultipleShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
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

planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=MultipleShootingTranscription(
        MultipleShootingOptions(tf=5.0, n_steps=50)
    ),
    optimizer=Optimizer(
        backend="scipy",
        options={
            "disp": True,
            "maxiter": 1000,
            "ftol": 1e-2,
        },
    ),
    options=TrajectoryOptimizationOptions(compile_backend="numpy"),
)

traj = planner.compute_solution()
result = planner.last_optimization_result

print(f"success: {result.success}")
print(f"message: {result.message}")
print(f"cost: {result.cost:.6g}")

planner.plot_solution(plot="xu")
planner.problem.sys.animate(traj)
