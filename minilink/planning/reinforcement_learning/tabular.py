"""Tabular reinforcement learning on a state-space grid: Q-learning, SARSA and Monte Carlo control."""

from dataclasses import dataclass, replace

import numpy as np

from minilink.planning.evaluation import nominal_trajectory
from minilink.planning.planner import Planner
from minilink.planning.policy_synthesis.discretizer import StateSpaceGrid
from minilink.planning.policy_synthesis.dp import (
    DynamicProgrammingOptions,
    DynamicProgrammingResult,
)
from minilink.planning.reinforcement_learning.environment import RolloutEnvironment
from minilink.planning.results import PlanningSolution

# Public API


class TabularLearningPlanner(Planner):
    """
    Model-free learning of the cost-to-go table ``Q(x, u)`` on a state-space grid.

    The learner never reads ``f``. Its world is the grid: an episode moves
    from node to node, each control period stepping the plant from the
    current node under one of the grid's input levels and rounding the state
    it reaches to a node, so the learner sees only transitions
    ``(x_k, u_k, c_k, x_k+1)`` between nodes. Every algorithm updates one cell of the table by a
    moving average toward its own sample ``q`` of the cost-to-go,

        Q(x, u) <- Q(x, u) + eta (q - Q(x, u)),

    Q-learning with the Bellman target ``q = c + alpha min_u' Q(x', u')``,
    SARSA with the action actually taken next, Monte Carlo control with the
    cost observed to the end of the episode. The result is the cost-to-go
    field and greedy policy that value iteration returns, so ``plot_cost2go``,
    ``plot_policy`` and ``get_controller`` (a lookup-table block) read the
    same way, and the same grid solved by
    :class:`~minilink.planning.policy_synthesis.dp.DynamicProgrammingPlanner`
    is the exact answer the learner approaches.

    Parameters
    ----------
    problem : PlanningProblem or StochasticPlanningProblem
        The task: plant, cost, box and exit rule; a stochastic problem adds
        its start, parameter and disturbance draws.
    x_grid, u_grid, dt
        Grid levels per state and input axis and the control period, as for
        value iteration; or pass ``grid=`` directly.
    algorithm : {"q_learning", "sarsa", "monte_carlo"} or object
        The target of the moving average, or an instance of :class:`QLearning`,
        :class:`SARSA` or :class:`MonteCarloControl`.
    exploration : EpsilonGreedy or UCB, optional
        How actions are chosen while learning (default ``EpsilonGreedy()``,
        annealed from 1 to 0.05 over each call to :meth:`learn`).
    eta : float, optional
        Learning rate of the moving average; ``None`` averages the samples of
        each cell (the step ``1/N(x, u)``).
    alpha : float, optional
        Discount per control period; default the cost's
        ``discount_factor(dt)``, which is 1 for an undiscounted cost, as for
        value iteration.
    episode_length : float, optional
        Duration of an episode for an infinite-horizon problem (default 10 s).
    exploring_starts : bool
        Draw episode starts uniformly over the grid (default), or from the
        problem's start distribution.
    rounding : {"stochastic", "nearest"}
        How a state becomes a node. ``"nearest"`` rounds each coordinate.
        ``"stochastic"`` (default) picks, on each axis, the lower or the upper
        level with the probability of the state's fractional position, so a
        value read at the node is, in expectation, the multilinear
        interpolation value iteration uses: the learner then samples the same
        Markov decision process value iteration solves on the grid.
    integrator : {"rk4", "euler"}
        Integration scheme of one control period; value iteration's grid
        steps with Euler.
    seed : int
        Seed of the starts, the exploration and the rounding.

    Attributes
    ----------
    Q : ndarray, shape (nodes_n, actions_n)
        The cost-to-go table, updated in place.
    visits : ndarray of int, shape (nodes_n, actions_n)
        How often each cell was updated.
    history : list of dict
        One record per episode: its cost, length and exploration rate.
    """

    accepts_stochastic = True

    def __init__(
        self,
        problem,
        *,
        grid=None,
        x_grid=None,
        u_grid=None,
        dt=None,
        algorithm="q_learning",
        exploration=None,
        eta=0.1,
        alpha=None,
        episode_length=None,
        exploring_starts=True,
        rounding="stochastic",
        integrator="rk4",
        seed=0,
    ):
        super().__init__(problem)
        cost = self.require_cost()
        if grid is None:
            if x_grid is None or u_grid is None or dt is None:
                raise ValueError(
                    "pass grid=StateSpaceGrid(...) or all of x_grid, u_grid, and dt"
                )
            grid = StateSpaceGrid(
                problem, x_grid_shape=x_grid, u_grid_shape=u_grid, dt=dt
            )
        self.grid = grid
        # The world is the grid: leaving it is infeasible, at the problem's price or
        # value iteration's default, so the learner approaches the same table
        price = problem.infeasible_cost
        if price is None:
            price = DynamicProgrammingOptions().out_of_bound_cost
        self.env = RolloutEnvironment(
            replace(problem, X=problem.X & grid.X, infeasible_cost=price),
            dt=grid.dt,
            episode_length=episode_length,
            integrator=integrator,
            backend="numpy",
            training_zone=grid.X,
        )
        if rounding not in ("stochastic", "nearest"):
            raise ValueError(
                f"rounding must be 'stochastic' or 'nearest', got {rounding!r}"
            )
        self.rounding = rounding
        self.algorithm = self.build_algorithm(algorithm)
        self.exploration = EpsilonGreedy() if exploration is None else exploration
        self.eta = None if eta is None else float(eta)
        self.alpha = cost.discount_factor(grid.dt) if alpha is None else float(alpha)
        self.exploring_starts = bool(exploring_starts)
        self.rng = np.random.default_rng(seed)

        self.Q = np.zeros((grid.nodes_n, grid.actions_n))
        self.visits = np.zeros((grid.nodes_n, grid.actions_n), dtype=int)
        self.history = []

    def learn(self, episodes):
        """Run ``episodes`` episodes of interaction, learning as they go; returns ``self``."""
        grid, env, Q, N = self.grid, self.env, self.Q, self.visits
        algorithm, explore, alpha, rng = (
            self.algorithm,
            self.exploration,
            self.alpha,
            self.rng,
        )

        for episode in range(episodes):
            progress = episode / max(episodes - 1, 1)
            params = env.sample_params(rng)
            s, t = self.node(self.start()), 0.0
            x = grid.states[s]
            a = explore.choose(Q[s], N[s], rng, progress)
            visited, episode_cost = [], 0.0

            for _ in range(env.n_steps_per_episode):
                # One control period of the plant under the grid input of action a
                u = grid.inputs[a]
                x_next, t_next, reward, terminated, truncated = env.step(
                    x, t, u, rng, params
                )

                # What the learner sees: the stage cost c_k = g dt (plus a priced exit or a
                # terminal cost), the next node, and the action it will take there
                c = -float(reward)
                s_next = self.node(x_next)
                a_next = explore.choose(Q[s_next], N[s_next], rng, progress)
                x_next = grid.states[
                    s_next
                ]  # the world is the grid: the plant continues from the node

                # Step size of the moving average: eta, or the sample average 1/N(x, u)
                N[s, a] += 1
                eta = 1.0 / N[s, a] if self.eta is None else self.eta

                # Temporal-difference methods update the cell now, from this transition
                algorithm.update(
                    Q, s, a, c, s_next, a_next, bool(terminated), eta, alpha
                )

                visited.append((s, a, c, eta))
                episode_cost += c
                if terminated or truncated:
                    break
                x, t, s, a = x_next, t_next, s_next, a_next

            # Monte Carlo methods update the visited cells once the episode's cost is known
            algorithm.update_episode(Q, visited, alpha)
            self.history.append(
                {
                    "episode": len(self.history) + 1,
                    "cost": episode_cost,
                    "steps": len(visited),
                    "epsilon": explore.epsilon(progress),
                }
            )
        return self

    def solve(self, episodes=1000, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """
        Learn for ``episodes`` episodes; return the greedy law and its cost-to-go as a :class:`PlanningSolution`.

        ``evaluate=True`` also rolls the law out from the problem's start and
        scores it over the problem's draws.
        """
        self.learn(episodes)
        result = self.result
        policy = self.get_controller()
        trajectory = evaluation = None
        if evaluate:
            trajectory = nominal_trajectory(
                self.problem, policy, dt=self.grid.dt, tf=self.env.tf
            )
            evaluation = self.evaluate(
                policy, dt=self.grid.dt, n_trials=n_trials, tf=self.env.tf
            )
        return self.store_solution(
            PlanningSolution(
                self.problem,
                policy,
                self.learning_record(),
                trajectory,
                evaluation,
                result.value_at,
            )
        )

    def solve_policy(
        self, episodes=1000, *, evaluate=False, n_trials=50
    ) -> PlanningSolution:
        """Policy-family name of :meth:`solve`."""
        return self.solve(episodes, evaluate=evaluate, n_trials=n_trials)

    def nominal_trajectory(self, tf=None):
        """The greedy law from the problem's start on the grid's control period."""
        return nominal_trajectory(
            self.problem,
            self.get_controller(),
            dt=self.grid.dt,
            tf=self.env.tf if tf is None else tf,
        )

    def learning_record(self) -> "TabularLearningRecord":
        """The solver record: episodes, coverage of the grid, and the cost of the last episodes."""
        last = [
            record["cost"]
            for record in self.history[-max(len(self.history) // 10, 1) :]
        ]
        return TabularLearningRecord(
            algorithm=type(self.algorithm).__name__,
            episodes=len(self.history),
            visited_fraction=float(np.mean(self.visits.sum(axis=1) > 0)),
            cost=float(np.mean(last)) if last else float("nan"),
            table_finite=bool(np.all(np.isfinite(self.Q))),
            history=list(self.history),
        )

    @property
    def result(self) -> DynamicProgrammingResult:
        """
        The table as a cost-to-go field ``J = min_u Q`` and greedy policy ``pi = argmin_u Q``.

        A cell never visited keeps its initial value, zero: the optimistic
        start that makes untried actions worth trying (``visits`` tells which).
        """
        return DynamicProgrammingResult(
            grid=self.grid,
            J=self.Q.min(axis=1),
            pi=self.Q.argmin(axis=1),
            iterations=len(self.history),
            delta=float("nan"),
        )

    def get_controller(self, **kwargs):
        """The greedy policy as a :class:`~minilink.planning.policy_synthesis.lookup_policy.LookupTableController`."""
        from minilink.planning.policy_synthesis import plotting

        return plotting.get_controller(self.result, **kwargs)

    def value_at(self, x) -> float:
        """Interpolate the learned cost-to-go at a state ``x``."""
        return self.result.value_at(x)

    def plot_cost2go(self, **kwargs):
        from minilink.planning.policy_synthesis import plotting

        return plotting.plot_cost2go(self.result, **kwargs)

    def plot_policy(self, **kwargs):
        from minilink.planning.policy_synthesis import plotting

        return plotting.plot_policy(self.result, **kwargs)

    def plot_learning_curve(self, ax=None, window=20):
        """Episode cost against episodes, smoothed by a moving average of ``window`` episodes."""
        import matplotlib.pyplot as plt

        if ax is None:
            _, ax = plt.subplots(figsize=(8, 3))
        cost = np.array([record["cost"] for record in self.history])
        kernel = np.ones(min(window, cost.size)) / min(window, cost.size)
        ax.plot(np.arange(1, cost.size + 1), cost, alpha=0.3)
        ax.plot(
            np.arange(kernel.size, cost.size + 1), np.convolve(cost, kernel, "valid")
        )
        ax.set_xlabel("episode")
        ax.set_ylabel("episode cost")
        ax.grid(True, alpha=0.3)
        return ax

    # Internal machinery

    def build_algorithm(self, algorithm):
        if isinstance(algorithm, TabularAlgorithm):
            return algorithm
        if algorithm in TABULAR_ALGORITHMS:
            return TABULAR_ALGORITHMS[algorithm]()
        raise ValueError(
            f"algorithm must be one of {sorted(TABULAR_ALGORITHMS)} or a TabularAlgorithm"
        )

    def start(self):
        """An episode's first state: anywhere on the grid, or the problem's own draw."""
        if self.exploring_starts:
            return self.grid.X.sample(self.rng)  # anywhere on the grid
        return self.env.reset(self.rng)

    def node(self, x):
        """The grid node that stands for a state (see ``rounding``)."""
        grid = self.grid
        if self.rounding == "nearest":
            return grid.nearest_node(x)

        # Fractional position of x between its two neighbouring levels on each axis
        shape = np.array(grid.x_grid_shape)
        f = np.clip(
            (np.asarray(x, dtype=float) - grid.x_lb) / grid.x_step, 0.0, shape - 1
        )
        lower = np.floor(f).astype(int)
        upper = np.minimum(lower + 1, shape - 1)

        # The upper level with the probability of the fractional part, else the lower
        index = np.where(self.rng.random(grid.n) < f - lower, upper, lower)
        return int(np.ravel_multi_index(index, grid.x_grid_shape))


@dataclass(frozen=True)
class TabularLearningRecord:
    """Episodes run, how much of the grid was visited, and the mean episode cost at the end of learning."""

    algorithm: str
    episodes: int
    visited_fraction: float
    cost: float
    table_finite: bool
    history: list

    @property
    def success(self) -> bool:
        """The table stayed finite."""
        return bool(self.table_finite)

    def __str__(self) -> str:
        return (
            f"{self.algorithm}: {self.episodes} episodes, {100 * self.visited_fraction:.0f}% of "
            f"the grid visited, mean episode cost {self.cost:.2f} at the end"
        )


class TabularAlgorithm:
    """The target of the moving average: one of two hooks, per step or per episode."""

    def update(self, Q, s, a, c, s_next, a_next, terminated, eta, alpha):
        """Update after one transition (temporal-difference methods)."""

    def update_episode(self, Q, visited, alpha):
        """Update once the episode ended, from its ``(s, a, c, eta)`` steps (Monte Carlo methods)."""


class QLearning(TabularAlgorithm):
    """Off-policy temporal-difference control: the target takes the best next action, whatever the agent does next."""

    def update(self, Q, s, a, c, s_next, a_next, terminated, eta, alpha):
        # Target: q = c + alpha min_u' Q(x', u'), nothing beyond a terminal state
        q = c + alpha * (1.0 - terminated) * Q[s_next].min()

        # Moving average toward the target: Q(x, u) <- Q(x, u) + eta (q - Q(x, u))
        Q[s, a] += eta * (q - Q[s, a])


class SARSA(TabularAlgorithm):
    """On-policy temporal-difference control: the target takes the action the agent really takes next."""

    def update(self, Q, s, a, c, s_next, a_next, terminated, eta, alpha):
        # Target: q = c + alpha Q(x', u'), with u' the next action, exploration included
        q = c + alpha * (1.0 - terminated) * Q[s_next, a_next]

        # Moving average toward the target: Q(x, u) <- Q(x, u) + eta (q - Q(x, u))
        Q[s, a] += eta * (q - Q[s, a])


class MonteCarloControl(TabularAlgorithm):
    """On-policy Monte Carlo control: the target is the cost actually observed to the end of the episode."""

    def update_episode(self, Q, visited, alpha):
        # Observed cost-to-go of each visited cell, backwards: q_k = c_k + alpha q_k+1
        q = 0.0
        for s, a, c, eta in reversed(visited):
            q = c + alpha * q

            # Moving average toward the target: Q(x, u) <- Q(x, u) + eta (q - Q(x, u))
            Q[s, a] += eta * (q - Q[s, a])


class EpsilonGreedy:
    """
    With probability ``epsilon`` a random action, otherwise the greedy one.

    ``epsilon`` decreases linearly to ``final`` over a run: exploration
    first, exploitation at the end. Ties among greedy actions break at random.
    """

    def __init__(self, epsilon=1.0, final=0.05):
        self.epsilon_start = float(epsilon)
        self.final = float(final)

    def epsilon(self, progress):
        """The exploration rate at ``progress`` in ``[0, 1]`` of the run."""
        return self.epsilon_start + (self.final - self.epsilon_start) * progress

    def choose(self, Q_s, N_s, rng, progress):
        """Action id at a state with cost-to-go row ``Q_s`` and visit counts ``N_s``."""
        if rng.random() < self.epsilon(progress):
            return int(rng.integers(Q_s.size))
        return int(rng.choice(np.flatnonzero(Q_s == Q_s.min())))


class UCB:
    """
    Optimism in the face of uncertainty: the action with the lowest confidence bound on its cost.

    ``Q(x, u) - c sqrt(ln N(x) / N(x, u))``: an action tried few times carries
    a large bonus and gets tried; a bad one tried often sees its bonus fade.
    Untried actions come first.
    """

    def __init__(self, c=1.0):
        self.c = float(c)

    def epsilon(self, progress):
        """No random action: the bonus is the exploration."""
        return 0.0

    def choose(self, Q_s, N_s, rng, progress):
        """Action id at a state with cost-to-go row ``Q_s`` and visit counts ``N_s``."""
        untried = np.flatnonzero(N_s == 0)
        if untried.size:
            return int(rng.choice(untried))

        # Lower confidence bound on the cost of each action
        bound = Q_s - self.c * np.sqrt(np.log(N_s.sum()) / N_s)
        return int(rng.choice(np.flatnonzero(bound == bound.min())))


TABULAR_ALGORITHMS = {
    "q_learning": QLearning,
    "sarsa": SARSA,
    "monte_carlo": MonteCarloControl,
}
