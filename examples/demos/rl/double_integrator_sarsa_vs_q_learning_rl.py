"""SARSA against Q-learning beside a cliff: on-policy learning keeps the margin the off-policy optimum drops."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import DoubleIntegrator, QuadraticCost
from minilink.planning import (
    EpsilonGreedy,
    MonteCarloEvaluator,
    StochasticPlanningProblem,
    TabularLearningPlanner,
    Uniform,
)

EPISODES = 3000
DT = 0.1
ALPHA = 0.95
EPSILON = 0.2  # kept constant: the learner keeps slipping while it learns
TARGET = 0.8  # the set point, close to the cliff at p = 1
CLIFF_COST = 10.0

# Plant: a point mass on a line, the box ends at the cliff
plant = DoubleIntegrator()
plant.state.lower_bound = np.array([-1.0, -1.0])
plant.state.upper_bound = np.array([1.0, 1.0])
plant.inputs["u"].lower_bound = np.array([-1.0])
plant.inputs["u"].upper_bound = np.array([1.0])

# Cost: reach the set point; falling off the cliff terminates the episode and is charged
cost = QuadraticCost.from_system(
    plant, Q=np.diag([1.0, 0.1]), R=np.diag([0.01]), xbar=[TARGET, 0.0]
)
problem = StochasticPlanningProblem(
    plant,
    cost=cost,
    tf=np.inf,
    x0_distribution=Uniform([-1.0, -0.2], [0.0, 0.2]),
    infeasible_cost=CLIFF_COST,
    X=plant.state.box,
)

# Two learners on one grid, one exploration rule; only the target of the update differs:
# Q-learning backs up the best next action, SARSA the one the explorer really takes
learners = {}
for name in ("q_learning", "sarsa"):
    learners[name] = TabularLearningPlanner(
        problem,
        x_grid=(21, 21),
        u_grid=(3,),
        dt=DT,
        alpha=ALPHA,
        eta=0.1,
        algorithm=name,
        exploration=EpsilonGreedy(epsilon=EPSILON, final=EPSILON),
        exploring_starts=False,
        integrator="euler",
        seed=0,
    )
    learners[name].learn(EPISODES)

# 1. While learning, exploration included: SARSA pays for its slips less often
fig, ax = plt.subplots(figsize=(8, 3))
for name, learner in learners.items():
    learner.plot_learning_curve(ax=ax, window=100)
ax.legend(["Q-learning", "_", "SARSA", "_"])
ax.set_title(f"episode cost under epsilon-greedy exploration, epsilon = {EPSILON}")

# 2. The greedy laws, exploration off: Q-learning's is the cheaper one
evaluator = MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1, backend="numpy")
for name, learner in learners.items():
    print(
        f"{name:11s} greedy law, Monte Carlo:",
        evaluator.evaluate(learner.get_controller()),
    )
    slips = np.mean([record["cost"] > CLIFF_COST for record in learner.history[-500:]])
    print(
        f"{name:11s} fell off the cliff in {100 * slips:.0f}% of the last 500 training episodes"
    )

# 3. Where each law brakes: the SARSA policy turns around earlier before the cliff
fig, axes = plt.subplots(1, 2, figsize=(10, 4))
for ax, (name, learner) in zip(axes, learners.items()):
    learner.plot_policy(ax=ax, show=False)
    ax.set_title(f"{name}: greedy action")
    ax.axvline(TARGET, color="k", linestyle="--")
plt.show()
