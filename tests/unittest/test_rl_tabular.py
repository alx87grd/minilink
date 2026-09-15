"""Tabular reinforcement learning on a grid: the learners, exploration, and the value-iteration reference."""

import numpy as np
import pytest

from minilink import DynamicProgrammingPlanner, PlanningProblem, QuadraticCost
from minilink.dynamics.catalog.equations import DoubleIntegrator
from minilink.planning import StochasticPlanningProblem, Uniform
from minilink.planning.policy_synthesis.dp import DynamicProgrammingResult
from minilink.planning.policy_synthesis.lookup_policy import LookupTableController
from minilink.planning.reinforcement_learning import (
    SARSA,
    UCB,
    EpsilonGreedy,
    MonteCarloControl,
    QLearning,
    TabularLearningPlanner,
)
from minilink.planning.reinforcement_learning.environment import RolloutEnvironment
from minilink.planning.results import PlanningSolution

GRID = dict(x_grid=(11, 11), u_grid=(3,), dt=0.2)


def double_integrator_problem(stochastic=False):
    plant = DoubleIntegrator()
    plant.state.lower_bound = np.array([-2.0, -2.0])
    plant.state.upper_bound = np.array([2.0, 2.0])
    plant.inputs["u"].lower_bound = np.array([-1.0])
    plant.inputs["u"].upper_bound = np.array([1.0])
    cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=np.diag([0.01]))
    exit_rule = dict(on_exit="terminate", exit_cost=20.0)  # one price for both tools
    if stochastic:
        return StochasticPlanningProblem(
            plant,
            cost=cost,
            tf=np.inf,
            x0_distribution=Uniform([-1, -1], [1, 1]),
            **exit_rule,
        )
    return PlanningProblem(plant, x_start=[1.0, 0.0], cost=cost, tf=np.inf, **exit_rule)


def test_q_learning_approaches_value_iteration_on_the_same_grid():
    problem = double_integrator_problem()
    vi = DynamicProgrammingPlanner(problem, alpha=0.9, tol=1e-6, **GRID)
    vi.solve()

    # The same grid, Euler step and exit price; the learner samples what value iteration sweeps
    learner = TabularLearningPlanner(
        problem, alpha=0.9, eta=0.1, integrator="euler", seed=0, **GRID
    )
    solution = learner.solve(episodes=2000)
    assert isinstance(solution, PlanningSolution) and solution.success
    assert isinstance(solution.policy, LookupTableController)
    assert isinstance(learner.result, DynamicProgrammingResult)
    assert solution.solver.episodes == 2000 and len(learner.history) == 2000
    assert solution.cost_to_go(np.zeros(2)) == learner.value_at(np.zeros(2))
    assert np.all(learner.visits.sum(axis=1) > 0)

    J_vi = vi.result.J
    error = np.abs(learner.result.J - J_vi)
    assert np.median(error) < 0.35 * np.median(J_vi)

    # The greedy law is a lookup-table block that closes the loop and regulates
    ctl = learner.get_controller()
    assert isinstance(ctl, LookupTableController)
    plant = problem.sys
    plant.x0 = np.array([1.0, 0.0])
    traj = (ctl @ plant).compute_trajectory(tf=6.0, dt=0.01, verbose=False)
    assert abs(traj.x[0, -1]) < 0.3


def test_nearest_rounding_aliases_a_coarse_grid():
    # From a node with a small speed, one Euler step rounds back to the same node
    problem = double_integrator_problem()
    nearest = TabularLearningPlanner(
        problem, rounding="nearest", integrator="euler", seed=0, **GRID
    )
    x = nearest.grid.states[nearest.node(np.array([0.9, 0.5]))]
    x_next = x + problem.sys.f(x, np.array([0.0])) * GRID["dt"]
    assert nearest.node(x_next) == nearest.node(x)
    # stochastic rounding moves on with the probability of the fractional position
    stochastic = TabularLearningPlanner(problem, integrator="euler", seed=0, **GRID)
    nodes = {stochastic.node(x_next) for _ in range(200)}
    assert len(nodes) == 2
    with pytest.raises(ValueError):
        TabularLearningPlanner(problem, rounding="floor", **GRID)


@pytest.mark.parametrize(
    "algorithm", ["sarsa", "monte_carlo", QLearning(), SARSA(), MonteCarloControl()]
)
def test_every_tabular_algorithm_learns_from_the_same_loop(algorithm):
    learner = TabularLearningPlanner(
        double_integrator_problem(stochastic=True),
        algorithm=algorithm,
        alpha=0.9,
        exploring_starts=False,
        **GRID,
    )
    learner.learn(50)
    assert len(learner.history) == 50
    assert learner.visits.sum() == sum(record["steps"] for record in learner.history)
    assert np.all(np.isfinite(learner.Q))
    assert not np.array_equal(learner.Q, np.zeros_like(learner.Q))


def test_exploration_policies_choose_valid_actions():
    rng = np.random.default_rng(0)
    Q_s = np.array([3.0, 1.0, 1.0, 2.0])
    N_s = np.array([5, 5, 0, 5])

    greedy = EpsilonGreedy(epsilon=0.0)
    picks = {greedy.choose(Q_s, N_s, rng, 0.0) for _ in range(50)}
    assert picks <= {1, 2}  # the two greedy actions, ties broken at random

    annealed = EpsilonGreedy(epsilon=1.0, final=0.1)
    assert annealed.epsilon(0.0) == 1.0 and abs(annealed.epsilon(1.0) - 0.1) < 1e-12
    assert 0 <= annealed.choose(Q_s, N_s, rng, 0.5) < 4

    ucb = UCB(c=1.0)
    assert ucb.choose(Q_s, N_s, rng, 0.0) == 2  # an untried action comes first
    N_s = np.array([50, 50, 50, 1])
    assert ucb.choose(Q_s, N_s, rng, 0.0) in (1, 2, 3)


def test_sample_average_step_and_the_planner_arguments():
    problem = double_integrator_problem()
    learner = TabularLearningPlanner(problem, eta=None, **GRID)
    solution = learner.solve(episodes=5, evaluate=True)
    assert learner.alpha == 1.0  # an undiscounted cost, as for value iteration
    assert "epsilon" in learner.history[-1]
    assert solution.evaluation.n_trials == 1 and solution.trajectory is not None
    assert "episodes" in str(solution.solver)
    with pytest.raises(ValueError):
        TabularLearningPlanner(problem, x_grid=(5, 5))
    with pytest.raises(ValueError):
        TabularLearningPlanner(problem, algorithm="dqn", **GRID)


def test_numpy_environment_steps_like_the_problem():
    problem = double_integrator_problem(stochastic=True)
    env = RolloutEnvironment(problem, dt=0.2, episode_length=0.6, backend="numpy")
    rng = np.random.default_rng(0)
    x, t = env.reset(rng), 0.0
    assert x.shape == (2,)
    for k in range(3):
        x, t, r, terminated, truncated = env.step(x, t, np.array([0.5]), rng)
        assert np.isfinite(r) and not terminated
        assert bool(truncated) == (k == 2)
    # a state pushed out of the box pays the problem's exit price and terminates
    _, _, r, terminated, truncated = env.step(
        np.array([1.99, 1.99]), 0.0, np.array([1.0]), rng
    )
    assert bool(terminated) and not bool(truncated) and float(r) < -19.0
