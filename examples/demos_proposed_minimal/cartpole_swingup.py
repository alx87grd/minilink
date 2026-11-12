"""Cart-pole swing-up: a trajectory planned once, then LQR feedback along it."""

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

TF = 4.0  # [s] time allowed for the swing-up

# The plant: a cart-pole whose motor pushes the cart with at most 10 N

plant = CartPole()
plant.inputs["u"].lower[0] = -10.0
plant.inputs["u"].upper[0] = 10.0

# The task: from hanging at rest to upright at rest, in TF seconds

x_start = np.array([0.0, 0.0, 0.0, 0.0])
x_goal = np.array([0.0, np.pi, 0.0, 0.0])

cost = QuadraticCost.from_system(
    plant, Q=np.diag([1.0, 1.0, 0.0, 0.0]), R=np.diag([0.01]), x_bar=x_goal
)
problem = PlanningProblem(
    plant,
    x0=x_start,
    Xf=SingletonSet(x_goal),
    cost=cost,
    X=plant.state.box,
    U=plant.input_box,
    horizon=TF,
)

# The plan: direct collocation over 40 intervals

planner = TrajectoryOptimizationPlanner(
    problem, n_steps=40, transcription="direct_collocation", backend="jax"
)
plan = planner.solve().trajectory
planner.plot_trajectory()

# The plan replayed open loop from a start the plan did not expect: 0.5 m and 0.3 rad
# off. It falls

plant.x0 = x_start + np.array([0.5, 0.3, 0.0, 0.0])

replay = TrajectorySource(plan.t, plan.u) >> plant
replay.compute_trajectory(tf=2 * TF)
replay.animate()

# Feedback along the plan, u = u_d(t) − K(t) (x − x_d(t)), from the same start

controller = lqr_trajectory(
    plant, plan, Q=np.diag([1.0, 10.0, 1.0, 1.0]), R=np.diag([1.0])
)
controller.plot_gain_schedule()

closed_loop = controller @ plant
closed_loop.compute_trajectory(tf=2 * TF)
closed_loop.plot_trajectory()
closed_loop.animate()
