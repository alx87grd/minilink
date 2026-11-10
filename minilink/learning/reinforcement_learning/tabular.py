"""Tabular reinforcement learning on a state-space grid: Q-learning, SARSA and Monte Carlo control."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from minilink.planning.dynamic_programming import plotting
from minilink.planning.dynamic_programming.dp import (
    DynamicProgrammingResult,
    grid_failure_cost,
    grid_of,
)
from minilink.planning.dynamic_programming.lookup_policy import LookupPolicy
from minilink.planning.dynamic_programming.progress import progress as report_progress
from minilink.planning.environment import RolloutEnvironment
from minilink.planning.planner import (
    Planner,
    given_numbers,
    planner_at,
    planner_evidence,
    require_solution,
)
from minilink.planning.problems import require_cost
from minilink.planning.results import PlanningSolution
from minilink.planning.step_model import is_continuous

###############################################################################
# Public API
###############################################################################


class TabularLearningPlanner(Planner):
    """
    Model-free learning of the cost-to-go table ``Q(x, u)`` on a state-space grid.

    The learner never reads ``f``. Its world is the grid: an episode moves
    from node to node, each control period stepping the plant from the
    current node under one of the grid's input levels and rounding the state
    it reaches to a node, so the learner sees only transitions
    ``(x_k, u_k, c_k, x_{k+1})`` between nodes. Every algorithm updates one cell of the table by a
    moving average toward its own sample ``q`` of the cost-to-go,

        Q(x, u) <- Q(x, u) + eta (q - Q(x, u)),

    Q-learning with the Bellman target ``q = c + gamma min_u' Q(x', u')``,
    SARSA with the action actually taken next, Monte Carlo control with the
    cost observed to the end of the episode. The result is the cost-to-go
    field and greedy policy that value iteration returns, so ``plot_cost_to_go``,
    ``plot_control_law`` and ``controller`` (a lookup-table block) read the
    same way, and the same grid solved by
    :class:`~minilink.planning.dynamic_programming.dp.DynamicProgrammingPlanner`
    is the exact answer the learner approaches.

    Parameters
    ----------
    problem : PlanningProblem
        The task: plant, cost, box and exit rule; a stochastic one adds its
        start law, its parameter laws and its disturbance sources.
    x_grid, u_grid, dt
        Grid levels per state and input axis and the control period, as for
        value iteration; or pass ``grid=`` instead, never both.
    method : {"q_learning", "sarsa", "monte_carlo"} or object
        The target of the moving average, or an instance of :class:`QLearning`,
        :class:`SARSA` or :class:`MonteCarloControl`.
    exploration : EpsilonGreedy or UCB, optional
        How actions are chosen while learning (default ``EpsilonGreedy()``,
        annealed from 1 to 0.05 over each call to :meth:`learn`).
    eta : float, optional
        Learning rate of the moving average; ``None`` averages the samples of
        each cell (the step ``1/N(x, u)``).
    gamma : float, optional
        Discount per control period; default the problem's
        ``discount_factor(dt)``, which is 1 for an undiscounted problem, as for
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
    solver : {"rk4", "euler"}, optional
        Integration scheme of one control period; default the grid's (Euler
        for a grid built from ``x_grid`` and ``u_grid``), so the learner steps
        the plant as value iteration does on the same grid.
    seed : int
        Seed of the starts, the exploration and the rounding.

    Attributes
    ----------
    Q : ndarray, shape (n_nodes, n_actions)
        The cost-to-go table, updated in place.
    visits : ndarray of int, shape (n_nodes, n_actions)
        How often each cell was updated.
    history : list of dict
        One record per episode: its cost, length and exploration rate.
    """

    accepts_stochastic = True

    number_names = ("eta", "gamma")

    def __init__(
        self,
        problem,
        *,
        grid=None,
        x_grid=None,
        u_grid=None,
        dt=None,
        method="q_learning",
        exploration=None,
        eta=0.1,
        gamma=None,
        episode_length=None,
        exploring_starts=True,
        rounding="stochastic",
        solver=None,
        substeps=1,
        seed=0,
    ):
        super().__init__(problem)
        require_cost(self.problem)
        require_continuous_problem(problem)

        # Discretized state and input spaces
        grid = grid_of(problem, grid, x_grid, u_grid, dt)
        self.grid = grid

        # The world is the grid: leaving it is infeasible, at value iteration's
        # failure cost, and the reward its stage cost, so the learner approaches the
        # same table
        gamma_dp = problem.discount_factor(grid.dt) if gamma is None else float(gamma)
        failure_cost = grid_failure_cost(grid, gamma_dp)
        self.env = RolloutEnvironment(
            problem.replace(X=problem.X & grid.X, failure_cost=failure_cost),
            dt=grid.dt,
            episode_length=episode_length,
            solver=grid.solver if solver is None else solver,
            substeps=substeps,
            backend="numpy",
            training_zone=grid.X,
            reward="stage",
        )

        # Learning: the rounding to a node, the target, the exploration, the step and
        # the discount
        require_rounding(rounding)
        self.rounding = rounding
        self.algorithm = self.build_algorithm(method)
        self.exploration = EpsilonGreedy() if exploration is None else exploration
        self.eta = None if eta is None else float(eta)
        self.gamma = problem.discount_factor(grid.dt) if gamma is None else float(gamma)
        self.given = given_numbers(gamma=gamma)
        self.exploring_starts = bool(exploring_starts)
        self.seed = int(seed)
        self.rng = np.random.default_rng(seed)

        # The episodes' length and the plant's integration, as given
        self.episode_length = episode_length
        self.solver = solver
        self.substeps = int(substeps)

        # Memory: the table, its visit counts, one record per episode
        self.Q = np.zeros((grid.n_nodes, grid.n_actions))
        self.visits = np.zeros((grid.n_nodes, grid.n_actions), dtype=int)
        self.history = []

    @property
    def dt(self) -> float:
        """The grid's step: the period of an episode's interactions, and the integration step of the evidence and the views' run."""
        return self.grid.dt

    @property
    def result(self) -> DynamicProgrammingResult:
        """
        The table as it stands, as a cost-to-go field ``J = min_u Q`` and greedy policy ``pi = argmin_u Q``.

        The learner's own model, updated by every episode; the shortcuts
        (:attr:`controller`, :meth:`cost_to_go`, the plots) read the latest
        solution's tables instead, ``solution.record.tables``.

        A cell never visited keeps its initial value, zero: the optimistic
        start that makes untried actions worth trying (``visits`` tells which).
        Its ``n_iterations`` counts the episodes learned so far (value iteration's
        counts sweeps), ``delta`` is ``nan``.
        """
        grid = self.grid
        Q = self.Q
        n_episodes = len(self.history)

        # J(x) = min_u Q(x, u) and π(x) = argmin_u Q(x, u)
        J = Q.min(axis=1)
        pi = Q.argmin(axis=1)

        return DynamicProgrammingResult(
            grid=grid, J=J, pi=pi, n_iterations=n_episodes, delta=float("nan")
        )

    def solve(
        self,
        n_episodes=1000,
        *,
        params=None,
        evaluate=False,
        n_trials=50,
        verbose: bool = False,
    ) -> PlanningSolution:
        """
        Learn for ``n_episodes`` episodes; return the greedy law and its cost-to-go as a :class:`PlanningSolution`.

        ``evaluate=True`` also scores the law, as it is, over the problem's
        draws (integrated on the grid's ``dt``, over an episode on an
        infinite horizon).
        ``params`` (the complete tree of :attr:`params`, a copy edited) solves
        the planner rebuilt there (:func:`~minilink.planning.planner.planner_at`), this one unchanged.
        ``verbose`` prints the episodes' progress and, at the end, the record.
        """

        # At another tree: the planner there, its solution kept here
        if params is not None:
            solution = planner_at(self, params).solve(
                n_episodes, evaluate=evaluate, n_trials=n_trials, verbose=verbose
            )
            return self.store_solution(solution)

        # Episodes of interaction, learning as they go
        self.learn(n_episodes, verbose=verbose)

        # The greedy law of the learned table
        result = self.result
        controller = LookupPolicy(result.grid, result.pi)

        # Optionally, the law, as it is, scored over the problem's draws
        evaluation = None
        if evaluate:
            evaluation = planner_evidence(
                self,
                controller,
                dt=self.dt,
                n_trials=n_trials,
                episode_length=self.env.episode_length,
                backend="auto",
            )

        # The solution: the law, the record, the score, and the cost-to-go as a field
        # on the grid, the failure cost past it (the price of leaving the grid)
        record = self.learning_record(result)
        cost_to_go = result.grid.field(
            result.J, method="linear", outside=self.env.failure_cost
        )
        solution = PlanningSolution(
            problem=self.problem,
            controller=controller,
            record=record,
            cost_to_go=cost_to_go,
            evaluation=evaluation,
        )
        if verbose:
            print(record)

        return self.store_solution(solution)

    # Learning

    def build_kwargs(self) -> dict:
        """The keywords that rebuild this planner on another problem: the grid's shape, the learning's structure, the step, the seed, the discount it was given."""
        grid = self.grid
        kwargs = dict(
            x_grid=grid.x_grid_shape,
            u_grid=grid.u_grid_shape,
            dt=grid.dt,
            method=self.algorithm,
            exploration=self.exploration,
            eta=self.eta,
            episode_length=self.episode_length,
            exploring_starts=self.exploring_starts,
            rounding=self.rounding,
            solver=self.solver,
            substeps=self.substeps,
            seed=self.seed,
        )

        return {**kwargs, **self.given}

    def learn(self, n_episodes, *, verbose=False):
        """Run ``n_episodes`` episodes of interaction, learning as they go; returns ``self`` (``verbose``: the episodes' progress)."""
        grid = self.grid
        env = self.env
        Q = self.Q
        N = self.visits
        algorithm = self.algorithm
        explore = self.exploration
        eta = self.eta
        gamma = self.gamma
        rng = self.rng
        episodes = report_progress(
            range(n_episodes), "Learning Q(x,u)", verbose, unit="episodes"
        )

        for episode in episodes:
            # The episode's exploration schedule, parameter draw, start node and
            # first action
            progress = episode / max(n_episodes - 1, 1)
            params = env.sample_params(rng)
            s = self.node(self.start())
            t = env.t0
            x = grid.states[s]
            a = explore.choose(Q[s], N[s], rng, progress)
            visited = []
            episode_cost = 0.0

            for _ in range(env.n_steps_per_episode):
                # One control period of the plant under the grid input of action a
                u = grid.inputs[a]
                x_next, t_next, reward, terminated, truncated = env.step(
                    x, u, t, rng, params
                )

                # What the learner sees: the stage cost c_k (plus a charged exit or a
                # terminal cost), the next node, and the action it will take there
                c = -float(reward)
                s_next = self.node(x_next)
                a_next = explore.choose(Q[s_next], N[s_next], rng, progress)

                # The world is the grid: the plant continues from the node
                x_next = grid.states[s_next]

                # Step size of the moving average: η, or the sample average 1/N(x, u)
                N[s, a] += 1
                eta_k = 1.0 / N[s, a] if eta is None else eta

                # Temporal-difference methods update the cell now, from this transition
                algorithm.update(
                    Q, s, a, c, s_next, a_next, bool(terminated), eta_k, gamma
                )

                # The episode so far, until it terminates or runs out of time
                visited.append((s, a, c, eta_k))
                episode_cost += c
                if terminated or truncated:
                    break

                # The next step starts from the node reached, with the action chosen there
                x, t, s, a = x_next, t_next, s_next, a_next

            # Monte Carlo methods update the visited cells once the episode's cost is known
            algorithm.update_episode(Q, visited, gamma)

            self.record_episode(episode_cost, len(visited), explore.epsilon(progress))

        return self

    def learning_record(self, tables=None) -> TabularLearningRecord:
        """The solver record: episodes, coverage of the grid, and the cost of the last episodes."""
        history = self.history
        visits = self.visits
        Q = self.Q
        method = type(self.algorithm).__name__

        # Coverage: the fraction of the nodes with at least one visited cell
        visited_fraction = float(np.mean(visits.sum(axis=1) > 0))

        # The mean cost of the last tenth of the episodes, the last one at least
        last = [record["cost"] for record in history[-max(len(history) // 10, 1) :]]
        cost = float(np.mean(last)) if last else float("nan")

        table_finite = bool(np.all(np.isfinite(Q)))

        return TabularLearningRecord(
            method=method,
            n_episodes=len(history),
            visited_fraction=visited_fraction,
            cost=cost,
            table_finite=table_finite,
            history=list(history),
            tables=tables,
        )

    # Plots

    def plot_cost_to_go(self, **kwargs):
        """The latest solution's cost-to-go table on the grid itself, as value iteration draws it."""
        kwargs.setdefault("jmax", plotting.cost_scale_limit(self.env.failure_cost))
        return plotting.plot_cost_to_go(require_solution(self).record.tables, **kwargs)

    def plot_control_law(self, **kwargs):
        """The latest solution's greedy policy table on the grid itself, as value iteration draws it."""
        return plotting.plot_control_law(require_solution(self).record.tables, **kwargs)

    def plot_learning_curve(self, ax=None, window=20):
        """Episode cost against episodes, smoothed by a moving average of ``window`` episodes."""
        import matplotlib.pyplot as plt

        if ax is None:
            _, ax = plt.subplots(figsize=(8, 3))

        # The episode costs, and the averaging kernel over the window
        cost = np.array([record["cost"] for record in self.history])
        kernel = np.ones(min(window, cost.size)) / min(window, cost.size)

        # The raw costs, then their moving average
        ax.plot(np.arange(1, cost.size + 1), cost, alpha=0.3)
        ax.plot(
            np.arange(kernel.size, cost.size + 1), np.convolve(cost, kernel, "valid")
        )

        ax.set_xlabel("episode")
        ax.set_ylabel("episode cost")
        ax.grid(True, alpha=0.3)

        return ax

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def build_algorithm(self, method):
        """The target of the moving average: an instance as given, or one built by name."""
        if isinstance(method, TabularAlgorithm):
            return method
        if method in TABULAR_ALGORITHMS:
            return TABULAR_ALGORITHMS[method]()
        raise ValueError(
            f"method must be one of {sorted(TABULAR_ALGORITHMS)} or a TabularAlgorithm"
        )

    def start(self):
        """An episode's first state: anywhere on the grid, or the problem's own draw."""
        rng = self.rng

        # Exploring starts: anywhere on the grid
        if self.exploring_starts:
            return self.grid.X.sample(rng)

        return self.env.reset(rng)

    def node(self, x):
        """The grid node that stands for a state (see ``rounding``)."""
        grid = self.grid
        rng = self.rng
        if self.rounding == "nearest":
            return grid.nearest_node(x)

        shape = np.array(grid.x_grid_shape)
        x_lb = grid.x_lb
        x_step = grid.x_step

        # Position of x in levels on each axis, between its two neighbouring levels
        level = np.clip((np.asarray(x, dtype=float) - x_lb) / x_step, 0.0, shape - 1)
        lower = np.floor(level).astype(int)
        upper = np.minimum(lower + 1, shape - 1)

        # The upper level with the probability of the fractional part, else the lower
        index = np.where(rng.random(grid.n) < level - lower, upper, lower)
        s = int(np.ravel_multi_index(index, grid.x_grid_shape))

        return s

    def record_episode(self, cost, steps, epsilon):
        """Append one episode's record to ``history``: its number, cost, length and exploration rate."""
        record = {
            "episode": len(self.history) + 1,
            "cost": cost,
            "steps": steps,
            "epsilon": epsilon,
        }
        self.history.append(record)


@dataclass(frozen=True)
class TabularLearningRecord:
    """Episodes run, how much of the grid was visited, the mean episode cost at the end of learning, and the tables ``J`` and ``pi`` the solution was read from (their ``n_iterations`` the episodes, ``n_episodes``)."""

    method: str
    n_episodes: int
    visited_fraction: float
    cost: float
    table_finite: bool
    history: list
    tables: DynamicProgrammingResult | None = field(
        default=None, repr=False, compare=False
    )

    @property
    def success(self) -> bool:
        """The table stayed finite."""
        return bool(self.table_finite)

    # Display

    def __str__(self) -> str:
        return (
            f"{self.method}: {self.n_episodes} episodes, {100 * self.visited_fraction:.0f}% of "
            f"the grid visited, mean episode cost {self.cost:.2f} at the end"
        )


class TabularAlgorithm:
    """The target of the moving average: one of two hooks, per step or per episode."""

    def update(self, Q, s, a, c, s_next, a_next, terminated, eta, gamma):
        """Update after one transition (temporal-difference methods)."""

    def update_episode(self, Q, visited, gamma):
        """Update once the episode ended, from its ``(s, a, c, eta)`` steps (Monte Carlo methods)."""


class QLearning(TabularAlgorithm):
    """Off-policy temporal-difference control: the target takes the best next action, whatever the agent does next."""

    def update(self, Q, s, a, c, s_next, a_next, terminated, eta, gamma):
        # Target: q = c + γ min_u' Q(x', u'), nothing beyond a terminal state
        q = c + gamma * (1.0 - terminated) * Q[s_next].min()

        # Moving average toward the target: Q(x, u) ← Q(x, u) + η (q − Q(x, u))
        Q[s, a] += eta * (q - Q[s, a])


class SARSA(TabularAlgorithm):
    """On-policy temporal-difference control: the target takes the action the agent really takes next."""

    def update(self, Q, s, a, c, s_next, a_next, terminated, eta, gamma):
        # Target: q = c + γ Q(x', u'), with u' the next action, exploration included
        q = c + gamma * (1.0 - terminated) * Q[s_next, a_next]

        # Moving average toward the target: Q(x, u) ← Q(x, u) + η (q − Q(x, u))
        Q[s, a] += eta * (q - Q[s, a])


class MonteCarloControl(TabularAlgorithm):
    """On-policy Monte Carlo control: the target is the cost actually observed to the end of the episode."""

    def update_episode(self, Q, visited, gamma):
        # Observed cost-to-go of each visited cell, backwards: q_k = c_k + γ q_{k+1}
        q = 0.0
        for s, a, c, eta in reversed(visited):
            q = c + gamma * q

            # Moving average toward the target: Q(x, u) ← Q(x, u) + η (q − Q(x, u))
            Q[s, a] += eta * (q - Q[s, a])


# The algorithms ``method=`` names
TABULAR_ALGORITHMS = {
    "q_learning": QLearning,
    "sarsa": SARSA,
    "monte_carlo": MonteCarloControl,
}


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
        epsilon_start = self.epsilon_start
        epsilon_final = self.final

        # ε = ε_start + (ε_final − ε_start) progress: linear over the run
        epsilon = epsilon_start + (epsilon_final - epsilon_start) * progress

        return epsilon

    def choose(self, Q_s, N_s, rng, progress):
        """Action id at a state with cost-to-go row ``Q_s`` and visit counts ``N_s``."""
        # Explore: a random action with probability ε
        if rng.random() < self.epsilon(progress):
            return int(rng.integers(Q_s.size))

        # Exploit: a greedy action, ties broken at random
        greedy = np.flatnonzero(Q_s == Q_s.min())

        return int(rng.choice(greedy))


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
        c = self.c

        # Untried actions first
        untried = np.flatnonzero(N_s == 0)
        if untried.size:
            return int(rng.choice(untried))

        # Lower confidence bound of each action: Q(x, u) − c √(ln N(x) / N(x, u))
        bound = Q_s - c * np.sqrt(np.log(N_s.sum()) / N_s)

        # The lowest bound, ties broken at random
        lowest = np.flatnonzero(bound == bound.min())

        return int(rng.choice(lowest))


###############################################################################
# Internal machinery
###############################################################################


def require_continuous_problem(problem):
    """Refuse a step problem: the learner rounds a continuous plant's states to the grid; a step problem's table is value iteration's."""
    if not is_continuous(problem):
        raise NotImplementedError(
            "TabularLearningPlanner learns on a continuous problem's grid; solve a "
            "step problem's table with DynamicProgrammingPlanner(problem, x_grid=, u_grid=)"
        )


def require_rounding(rounding):
    """Refuse a rounding rule other than ``"stochastic"`` or ``"nearest"``."""
    if rounding not in ("stochastic", "nearest"):
        raise ValueError(
            f"rounding must be 'stochastic' or 'nearest', got {rounding!r}"
        )
