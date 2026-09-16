"""Q-learning on the value-iteration grid: sampled Bellman backups reach the same cost-to-go, without the model."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import DynamicProgrammingPlanner, InvertedPendulum, QuadraticCost
from minilink.planning import (
    MonteCarloEvaluator,
    StochasticPlanningProblem,
    TabularLearningPlanner,
    Uniform,
)

EPISODES = 8000  # about 25 s: one Python step per control period, no model
DT = 0.05
ALPHA = 0.98  # discount per control period, the same for both tools
X_GRID, U_GRID = (41, 41), (3,)
INF = 500.0  # price of leaving the grid, the same for both tools
UPRIGHT = np.array([0.0, 0.0])
HANGING = np.array([-np.pi, 0.0])

# Plant: a light pendulum with less torque than gravity, theta = 0 upright
plant = InvertedPendulum()
plant.params["m"] = 0.1
plant.params["l"] = 0.5
plant.params["I"] = 1.0 / 12.0 * 0.1
plant.params["d"] = 0.0
plant.state.lower_bound = np.array([-2.0 * np.pi, -12.0])
plant.state.upper_bound = np.array([2.0 * np.pi, 12.0])
plant.inputs["u"].lower_bound = np.array([-1.0])
plant.inputs["u"].upper_bound = np.array([1.0])

# Task: quadratic cost about the upright, starts around hanging, the exit priced
cost = QuadraticCost.from_system(
    plant, xbar=UPRIGHT, Q=np.diag([1.0 / DT, 0.1 / DT]), R=np.diag([0.1 / DT])
)
problem = StochasticPlanningProblem(
    plant,
    cost=cost,
    tf=np.inf,
    x0_distribution=Uniform(HANGING - 0.5, HANGING + 0.5),
    infeasible_cost=INF,
    X=plant.state.box,
)

# With the model: value iteration sweeps every node of the grid until the Bellman
# equation holds, J(x) = min_u [g(x, u) dt + alpha J(x + f(x, u) dt)]
vi = DynamicProgrammingPlanner(
    problem, x_grid=X_GRID, u_grid=U_GRID, dt=DT, alpha=ALPHA, tol=0.1
)
vi.solve()

# Without it: Q-learning steps the plant from node to node on the same grid and
# moves one cell of Q toward each observed sample c + alpha min_u' Q(x', u')
ql = TabularLearningPlanner(
    problem,
    x_grid=X_GRID,
    u_grid=U_GRID,
    dt=DT,
    alpha=ALPHA,
    eta=0.3,
    integrator="euler",  # the grid's own step, so the two tools sample one world
)
solution = ql.solve(episodes=EPISODES)
print(solution.solver)
ql.plot_learning_curve(window=200)

# Side by side: the cost-to-go and the greedy policy of both
fig, axes = plt.subplots(2, 2, figsize=(11, 8))
vi.plot_cost2go(jmax=INF, ax=axes[0, 0], title="value iteration: J*", show=False)
ql.plot_cost2go(jmax=INF, ax=axes[0, 1], title="Q-learning: min_u Q", show=False)
vi.plot_policy(ax=axes[1, 0], show=False)
ql.plot_policy(ax=axes[1, 1], show=False)
axes[1, 0].set_title("value iteration: greedy action")
axes[1, 1].set_title("Q-learning: greedy action")

feasible = vi.result.J < INF
error = np.abs(ql.result.J - vi.result.J)[feasible]
print(f"median |J_learned - J*| over the feasible nodes: {np.median(error):.1f}")
print(f"median J*: {np.median(vi.result.J[feasible]):.1f}")

# The two laws on one yardstick, then the learned table closes the loop from hanging
evaluator = MonteCarloEvaluator(problem, dt=DT, n_trials=50, seed=1, backend="numpy")
print("value iteration law:", evaluator.evaluate(vi.get_controller()))
print("Q-learning law:     ", evaluator.evaluate(ql.get_controller()))

plant.x0 = HANGING
cl_sys = ql.get_controller() @ plant
cl_sys.name = "Pendulum with the Q-learning table"
traj = cl_sys.compute_trajectory(tf=10.0, dt=0.01)
cl_sys.plot_trajectory(traj)
plt.show()
cl_sys.animate(traj)
