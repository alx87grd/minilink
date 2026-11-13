"""Cart-pole swing-up: a planned trajectory, then LQR feedback along it."""

import importlib.util

import numpy as np

from minilink import (
    CartPole,
    PlanningProblem,
    QuadraticCost,
    SingletonSet,
    TrajectoryOptimizationPlanner,
    TrajectorySource,
    lqr_trajectory,
)

TF = 4.0
X_START = np.array([0.0, 0.0, 0.0, 0.0])  # hanging, at rest
X_GOAL = np.array([0.0, np.pi, 0.0, 0.0])  # upright
X0 = X_START + np.array([0.5, 0.3, 0.0, 0.0])  # where the real cart-pole starts
OPTIMIZER = "ipopt" if importlib.util.find_spec("cyipopt") else "slsqp"

plant = CartPole()
plant.inputs["u"].lower[0] = -10.0
plant.inputs["u"].upper[0] = 10.0

# 1. The reference: a swing-up planned by direct collocation
cost = QuadraticCost.from_system(
    plant, Q=np.diag([1.0, 1.0, 0.0, 0.0]), R=np.diag([0.01]), x_bar=X_GOAL
)
problem = PlanningProblem(
    plant,
    horizon=TF,
    x0=X_START,
    Xf=SingletonSet(X_GOAL),
    cost=cost,
    X=plant.state.box,
    U=plant.input_box,
)
planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=40,
    transcription="direct_collocation",
    backend="jax",
    method=OPTIMIZER,
)
reference = planner.solve().trajectory
planner.plot_trajectory()

# Open-loop replay of the reference
replay = TrajectorySource(reference.t, reference.u) >> plant
replay.compute_trajectory(tf=2 * TF)
# replay.plot_trajectory()
replay.animate()

# 2. Feedback along the reference: u = u_d(t) - K(t) (x - x_d(t))
controller = lqr_trajectory(
    plant, reference, Q=np.diag([1.0, 10.0, 1.0, 1.0]), R=np.diag([1.0])
)
controller.plot_gain_schedule()

# 3. The closed loop from a perturbed start, against the open-loop replay of u_d(t)
plant.x0 = X0
loop = controller @ plant
loop.compute_trajectory(tf=2 * TF)
loop.plot_trajectory()
loop.animate()
