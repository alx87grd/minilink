"""Reinforcement learning as a planner: a neural law trained on a stochastic planning problem."""

import math
import time
import warnings
from dataclasses import dataclass
from functools import partial

import numpy as np

from minilink.control.neural import NeuralPolicyController
from minilink.core.backends import require_jax, require_jax_numpy
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import Evaluation
from minilink.planning.planner import Planner
from minilink.planning.reinforcement_learning.algorithms import (
    PPO,
    REINFORCE,
    SAC,
    ActorCritic,
    Algorithm,
)
from minilink.planning.reinforcement_learning.collect import (
    ReplayBuffer,
    flatten,
    reset_carry,
    rollout,
)
from minilink.planning.reinforcement_learning.critics import QFunction, ValueFunction
from minilink.planning.reinforcement_learning.environment import RolloutEnvironment
from minilink.planning.reinforcement_learning.policy import (
    GaussianHead,
    SquashedGaussianHead,
    StochasticPolicy,
)
from minilink.planning.results import PlanningSolution

#: The update rule of each name ``algorithm=`` accepts.
ALGORITHMS = {
    "reinforce": REINFORCE,
    "actor_critic": ActorCritic,
    "ppo": PPO,
    "sac": SAC,
}

#: Discount per control period when neither the algorithm, the planner nor the cost sets one.
DEFAULT_GAMMA = 0.99


###############################################################################
# Public API
###############################################################################


class ReinforcementLearningPlanner(Planner):
    """
    Policy-family planner that learns ``u = pi(x)`` by reinforcement learning.

    The training objective is the expected discounted return on the control
    grid, ``E[-sum_k gamma^k g(x_k, u_k) dt]``: a failure (leaving ``X``) pays the
    problem's price of infeasibility, leaving the training zone truncates the episode.
    ``solve`` returns the learned law as a
    :class:`~minilink.planning.results.PlanningSolution`; asked to evaluate, it
    also rolls the law out and scores it by Monte Carlo on the problem's own
    cost, the yardstick shared with every planner.
    The loop is generic, collect experience then update, and the update rule
    is an :class:`~minilink.planning.reinforcement_learning.algorithms.Algorithm`,
    so a new method is one file, not a new planner. The law being trained is
    a :class:`~minilink.planning.reinforcement_learning.policy.StochasticPolicy`:
    the controller block's mean action with the algorithm family's
    exploration head around it.

    Parameters
    ----------
    problem : StochasticPlanningProblem
        Task with a start distribution, cost and constraint set. A plain
        :class:`~minilink.planning.problems.PlanningProblem` is accepted and
        trained from its single ``x_start``.
    dt : float
        Control period of the learned law (one RK4 step per period).
    policy : NeuralPolicyController, optional
        The law to train; default built from ``features``, ``hidden``,
        ``activation`` and ``normalize`` on the problem's plant.
    algorithm : str or Algorithm
        ``"reinforce"``, ``"actor_critic"``, ``"ppo"`` (on-policy) or
        ``"sac"`` (off-policy), or an :class:`Algorithm` instance; keyword
        arguments not listed here go to the algorithm (``learning_rate``,
        ``n_epochs``, ``tau``, ...).
    n_envs, n_steps : int
        Plants simulated in parallel and control periods per collection (an
        episodic method collects one episode length instead of ``n_steps``).
    training_zone : Set, optional
        Where episodes run (the plant's state box by default); leaving it truncates
        the episode. A training choice, not a constraint: the constraint is ``problem.X``.
    episode_length : float, optional
        Episode duration for an infinite-horizon problem (default 10 s).
    gamma : float, optional
        Discount per control period, used when the algorithm does not set its
        own. Default: the cost's ``discount_factor(dt)`` when the cost declares
        a rate, else 0.99 with a warning that states the effective horizon.
    log_std_init : float
        Initial exploration log-std on the normalized action (Gaussian head).
    seed : int
        Seed of the weights, exploration and starts.
    verbose : bool
        Print the environment's semantics and one line per learning iteration
        (default ``True``). Pass ``False`` to silence training.

    Attributes
    ----------
    controller : NeuralPolicyController
        The law, updated in place after every learning iteration.
    policy : StochasticPolicy
        The controller with its exploration head, what the algorithm trains.
    critic : ValueFunction or QFunction or None
        The critic of the algorithm's family.
    history : list of dict
        One record per iteration (timesteps, mean episode return, losses, fps).
    env : RolloutEnvironment
        The compiled environment (its ``describe()`` states the semantics).
    """

    accepts_stochastic = True

    def __init__(
        self,
        problem,
        *,
        dt=0.05,
        policy=None,
        features=None,
        hidden=(64, 64),
        activation="tanh",
        normalize=True,
        algorithm="ppo",
        n_envs=16,
        n_steps=128,
        episode_length=None,
        gamma=None,
        log_std_init=0.0,
        integrator="rk4",
        training_zone=None,
        seed=0,
        verbose=True,
        **algorithm_kwargs,
    ):
        super().__init__(problem)
        jax = require_jax()

        # Training settings
        self.dt = float(dt)
        self.n_envs = int(n_envs)
        self.verbose = bool(verbose)

        # Training progress: plant steps, wall-clock time, the latest mean episode
        # return, and one record per learning iteration
        self.num_timesteps = 0
        self.train_time = 0.0
        self.last_ep_return_mean = np.nan
        self.history = []

        # The task, compiled for rollouts; training optimizes the expectation only
        self.env = RolloutEnvironment(
            problem,
            dt=dt,
            episode_length=episode_length,
            integrator=integrator,
            training_zone=training_zone,
        )
        self.require_expectation_criterion()
        if self.verbose:
            print(f"ReinforcementLearningPlanner: {self.env.describe()}")

        # The update rule, and its collection length: one whole episode for an
        # episodic method
        self.algorithm = self.build_algorithm(algorithm, algorithm_kwargs)
        self.n_steps = (
            self.env.n_steps_per_episode if self.algorithm.episodic else int(n_steps)
        )

        # The law, the exploration head and the critic of the algorithm's family,
        # bound to the algorithm with the one discount
        self.controller = policy or NeuralPolicyController(
            problem.sys,
            features=features,
            hidden=hidden,
            activation=activation,
            normalize=normalize,
            seed=seed,
        )
        head = self.build_head(hidden, activation, log_std_init, seed)
        self.policy = StochasticPolicy(self.controller, head)
        self.critic = self.build_critic(hidden, activation, seed)
        self.algorithm.bind(self.policy, self.critic, self.resolve_discount(gamma))

        # Train state and the parallel plants, from the seed's keys
        self.key = jax.random.PRNGKey(seed)
        self.key, k_init, k_reset = jax.random.split(self.key, 3)
        self.train_state = self.algorithm.init(k_init)
        self.carry = reset_carry(self.env, k_reset, self.n_envs)

        # One jitted call per collection, update and action; an off-policy method
        # updates from a replay of its transitions
        self.rollout_jit = jax.jit(
            partial(
                rollout, self.env, self.policy, n_steps=self.n_steps, n_envs=self.n_envs
            )
        )
        self.replay = None
        if self.algorithm.on_policy:
            self.update_jit = jax.jit(self.algorithm.update)
        else:
            self.replay = ReplayBuffer(
                self.algorithm.buffer_size, self.env.n, self.env.m
            )
            self.update_jit = jax.jit(self.replay_updates)
        self.action_jit = jax.jit(
            lambda theta, x: self.controller.action(x, {"mlp": theta["mlp"]})
        )

    @property
    def gamma(self):
        """The one discount per control period the algorithm trains with."""
        return self.algorithm.gamma

    def solve(
        self, timesteps=100_000, *, evaluate=False, n_trials=50, verbose=None
    ) -> PlanningSolution:
        """
        Train for ``timesteps`` plant steps; return the learned law as a :class:`PlanningSolution`.

        The solver record carries the training facts. ``evaluate=True`` also
        rolls the deterministic law out from the problem's start and scores it
        by Monte Carlo over ``n_trials`` draws of the problem. ``verbose``
        overrides :attr:`verbose` for this call only.
        """
        # Train, then store the learned law as the planner's solution
        self.learn(timesteps, verbose=verbose)
        solution = self.solution(evaluate, n_trials)

        return self.store_solution(solution)

    def solve_policy(
        self, timesteps=100_000, *, evaluate=False, n_trials=50, verbose=None
    ) -> PlanningSolution:
        """The planner-family name of :meth:`solve`, answered by every policy planner."""
        return self.solve(
            timesteps, evaluate=evaluate, n_trials=n_trials, verbose=verbose
        )

    # Training

    def learn(self, timesteps, *, verbose=None):
        """
        Train for ``timesteps`` plant steps (rounded up to whole collections); returns ``self``.

        ``verbose`` overrides :attr:`verbose` for this call only.
        """
        jax = require_jax()

        log = self.verbose if verbose is None else bool(verbose)
        steps_per_iteration = self.n_steps * self.n_envs
        n_iterations = math.ceil(timesteps / steps_per_iteration)

        for _ in range(n_iterations):
            # Fresh keys for this iteration's collection and update
            self.key, k_collect, k_update = jax.random.split(self.key, 3)
            t0 = time.time()

            # Collect experience and update the weights, timed to completion
            batch, stats = self.collect_and_update(k_collect, k_update)
            jax.block_until_ready(self.train_state)
            self.train_time += time.time() - t0
            self.num_timesteps += steps_per_iteration

            # The new weights, copied into the controller block
            self.sync_controller()

            # The iteration's record, and its progress line
            record = self.record_iteration(batch, stats, t0)
            if log:
                self.print_iteration(record, stats)

        return self

    def sync_controller(self):
        """Copy the current policy network weights into the controller block."""
        theta = self.algorithm.params(self.train_state)["policy"]
        self.controller.params["mlp"] = {
            k: np.asarray(v) for k, v in theta["mlp"].items()
        }

    # The learned law

    def get_controller(self):
        """The learned law as a controller block (``controller @ plant``)."""
        return self.controller

    def predict(self, x, deterministic=True):
        """
        The learned law ``u = pi(x)`` for one state or a batch of states (rows).

        Returns ``(u, None)``, the shape of Stable-Baselines3's ``predict``, so
        a script written for one trains and evaluates with the other.
        """
        jax = require_jax()
        jnp = require_jax_numpy()

        theta = self.algorithm.params(self.train_state)["policy"]
        x = jnp.asarray(x, dtype=float)

        # u = π(x), row by row for a batch of states
        if x.ndim == 2:
            u = jax.vmap(self.action_jit, in_axes=(None, 0))(theta, x)
        else:
            u = self.action_jit(theta, x)
        u = np.asarray(u)

        return u, None

    def solve_trajectory_from(self, x0, tf=None, *, evaluate=False) -> PlanningSolution:
        """
        Roll the learned law out from ``x0`` on the training environment, disturbances drawn.

        The solution's trajectory is that rollout; ``evaluate=True`` scores it
        under the problem's contract. The planner's stored solution is left as
        it is.
        """
        jax = require_jax()

        # The rollout, its disturbances drawn from a fresh key
        self.key, key = jax.random.split(self.key)
        trajectory = self.rollout_law(x0, tf, key)

        # Its score under the problem's contract, when asked
        evaluation = (
            Evaluation.of_trajectory(self.problem, trajectory) if evaluate else None
        )

        return self.learned_solution(trajectory, evaluation)

    def nominal_trajectory(self, tf=None) -> Trajectory:
        """The learned law from the problem's start, with nominal parameters and disturbances."""
        return self.rollout_law(self.problem.x_start, tf)

    # Plots

    def plot_learning_curve(self, ax=None):
        """Mean return of the exploration episodes against training steps."""
        import matplotlib.pyplot as plt

        if ax is None:
            _, ax = plt.subplots(figsize=(8, 3))

        # Mean episode return against plant steps, one point per iteration
        steps = [h["timesteps"] for h in self.history]
        returns = [h["ep_return_mean"] for h in self.history]
        ax.plot(steps, returns)
        ax.set_xlabel("timesteps")
        ax.set_ylabel("mean episode return")
        ax.grid(True, alpha=0.3)

        return ax

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_expectation_criterion(self):
        """Refuse a criterion other than the expectation: reinforcement learning minimizes the expected cost; other criteria are only evaluated."""
        criterion = self.env.problem.criterion
        if criterion != "expectation":
            raise NotImplementedError(
                "reinforcement learning optimizes the expectation; "
                f"criterion={criterion!r} is only reported by MonteCarloEvaluator"
            )

    def build_algorithm(self, algorithm, algorithm_kwargs):
        """An algorithm instance as given, or the named one built from the keyword arguments."""
        if isinstance(algorithm, Algorithm):
            return algorithm

        if algorithm in ALGORITHMS:
            return ALGORITHMS[algorithm](**algorithm_kwargs)

        raise ValueError(
            f"algorithm must be one of {sorted(ALGORITHMS)} or an Algorithm"
        )

    def build_head(self, hidden, activation, log_std_init, seed):
        """The exploration head of the algorithm's family."""
        ctl = self.controller
        m = self.env.m

        # A Gaussian around the block's mean action
        if self.algorithm.head_kind == "gaussian":
            return GaussianHead(m, log_std_init)

        # A tanh-squashed law: the block squashes its mean the same way once
        # training is over
        ctl.squash = "tanh"
        n_features = int(ctl.mlp.inputs["u"].dim)

        return SquashedGaussianHead(n_features, m, hidden, activation, seed=seed + 3)

    def build_critic(self, hidden, activation, seed):
        """The critic of the algorithm's family: a state value, an action value, or none."""
        ctl = self.controller
        m = self.env.m
        n_features = int(ctl.mlp.inputs["u"].dim)
        kind = self.algorithm.critic_kind

        # No critic, a state value V_w(x), or an action value Q_w(x, a), on the
        # law's features
        if kind is None:
            return None

        if kind == "V":
            return ValueFunction(
                ctl.observe, n_features, hidden, activation, seed=seed + 1
            )

        return QFunction(ctl.observe, n_features, m, hidden, activation, seed=seed + 1)

    def resolve_discount(self, gamma):
        """The one discount: the algorithm's own, else this planner's, else the cost's rate, else the default."""
        own = self.algorithm.gamma

        # The algorithm's own discount, which the planner's may only repeat
        if own is not None:
            if gamma is not None and float(gamma) != own:
                raise ValueError(
                    f"two discounts given: the algorithm's gamma={own} and the "
                    f"planner's gamma={gamma}; set it in one place"
                )
            return own

        # The planner's discount
        if gamma is not None:
            return float(gamma)

        cost = self.env.cost
        dt = self.dt

        # The cost's discount rate over one control period
        if cost.discount_rate > 0:
            return cost.discount_factor(dt)

        # The default, announced with its effective horizon dt / (−ln γ)
        horizon = dt / -math.log(DEFAULT_GAMMA)
        warnings.warn(
            f"the cost is undiscounted, so training discounts at gamma={DEFAULT_GAMMA} "
            f"per control period, an effective horizon of {horizon:.1f} s at "
            f"dt={dt:g}; declare discount_rate on the cost or pass gamma to choose it",
            stacklevel=3,
        )

        return DEFAULT_GAMMA

    def replay_updates(self, train_state, data, size, key):
        """Several gradient steps per collection, each on a fresh minibatch drawn from the replay."""
        jax = require_jax()
        jnp = require_jax_numpy()

        algorithm = self.algorithm
        n_steps = self.n_steps
        n_envs = self.n_envs

        # The algorithm's count of gradient steps, else one per collected transition
        gradient_steps = algorithm.gradient_steps or n_steps * n_envs

        def gradient_step(state, key):
            # A minibatch drawn uniformly from the filled part of the replay
            k_sample, k_update = jax.random.split(key)
            idx = jax.random.randint(k_sample, (algorithm.batch_size,), 0, size)
            minibatch = {k: v[idx] for k, v in data.items()}

            # One update of the algorithm on it
            return algorithm.update(state, minibatch, k_update)

        # The gradient steps in sequence, then each statistic averaged over them
        keys = jax.random.split(key, gradient_steps)
        state, stats = jax.lax.scan(gradient_step, train_state, keys)
        stats = {k: jnp.mean(v) for k, v in stats.items()}

        return state, stats

    def collect_and_update(self, k_collect, k_update):
        """One collection of experience and the algorithm's update on it; returns ``(batch, stats)``."""
        jax = require_jax()
        jnp = require_jax_numpy()

        theta = self.algorithm.params(self.train_state)["policy"]

        # An episodic method learns from complete episodes: every plant restarts, then
        # runs one
        if self.algorithm.episodic:
            k_collect, k_reset = jax.random.split(k_collect)
            self.carry = reset_carry(self.env, k_reset, self.n_envs)

        # Collect: n_steps control periods of the n_envs plants
        batch, self.carry = self.rollout_jit(theta, self.carry, k_collect)

        # On-policy: one update on the fresh batch
        if self.algorithm.on_policy:
            self.train_state, stats = self.update_jit(self.train_state, batch, k_update)
            return batch, stats

        # Off-policy: store the transitions, then update from replayed minibatches
        # once the replay holds learning_starts of them
        self.replay.add(flatten(batch))
        stats = {}
        if self.replay.size >= self.algorithm.learning_starts:
            self.train_state, stats = self.update_jit(
                self.train_state,
                self.replay.data,
                jnp.asarray(self.replay.size),
                k_update,
            )

        return batch, stats

    def record_iteration(self, batch, stats, t0):
        """Append this iteration's training record to :attr:`history` and return it."""
        # Mean return of the episodes that ended here; when none ended, the last mean
        # stands
        ep_returns = np.asarray(batch["ep_return"])
        ep_returns = ep_returns[np.isfinite(ep_returns)]
        if ep_returns.size:
            self.last_ep_return_mean = float(ep_returns.mean())

        # The record: steps, mean return, episodes, speed, time, the algorithm's losses
        record = {
            "timesteps": self.num_timesteps,
            "ep_return_mean": self.last_ep_return_mean,
            "n_episodes": int(ep_returns.size),
            "fps": self.n_steps * self.n_envs / max(time.time() - t0, 1e-9),
            "elapsed": self.train_time,
            **{k: float(v) for k, v in stats.items()},
        }
        self.history.append(record)

        return record

    def print_iteration(self, record, stats):
        """One progress line: timesteps, mean episode return, the algorithm's losses, speed."""
        shown = " | ".join(f"{k} {float(v):8.4g}" for k, v in stats.items())
        print(
            f"steps {record['timesteps']:>8d} | "
            f"ep_return {record['ep_return_mean']:9.1f} | "
            f"{shown} | fps {record['fps']:7.0f} | {record['elapsed']:.0f}s"
        )

    def solution(self, evaluate, n_trials) -> PlanningSolution:
        """The learned law with its training record; rolled out and scored when asked."""
        trajectory = None
        evaluation = None

        # The nominal rollout, and the Monte Carlo score over the problem's draws
        if evaluate:
            trajectory = self.nominal_trajectory()
            evaluation = self.evaluate(
                self.controller,
                dt=self.dt,
                n_trials=n_trials,
                tf=self.env.tf,
                backend="jax",
            )

        return self.learned_solution(trajectory, evaluation)

    def learned_solution(self, trajectory, evaluation) -> PlanningSolution:
        """The learned law as a solution: its training record, the rollout and score given, the critic's cost-to-go."""
        record = self.training_record()
        cost_to_go = self.critic_cost_to_go()

        return PlanningSolution(
            problem=self.problem,
            policy=self.controller,
            solver=record,
            trajectory=trajectory,
            evaluation=evaluation,
            cost_to_go=cost_to_go,
        )

    def training_record(self) -> "ReinforcementLearningRecord":
        """The solver record: the training facts, episode returns turned into costs."""
        jax = require_jax()

        # The weights are finite when every leaf of the pytree is
        weights = self.algorithm.params(self.train_state)
        weights_finite = all(
            bool(np.all(np.isfinite(np.asarray(w))))
            for w in jax.tree_util.tree_leaves(weights)
        )

        # Each iteration's mean episode return R turned into its cost J = −R
        history = [
            {
                **{k: v for k, v in record.items() if k != "ep_return_mean"},
                "cost": -record["ep_return_mean"],
            }
            for record in self.history
        ]

        return ReinforcementLearningRecord(
            algorithm=type(self.algorithm).__name__,
            timesteps=self.num_timesteps,
            train_time_s=self.train_time,
            cost=-self.last_ep_return_mean,
            weights_finite=weights_finite,
            history=history,
        )

    def critic_cost_to_go(self):
        """``J(x) = -V_w(x)`` from the critic, only when it estimates the problem's own discount; else ``None``."""
        # Only a state-value critic trained on the cost's own discount estimates J
        if self.algorithm.critic_kind != "V":
            return None

        if self.gamma != self.env.cost.discount_factor(self.dt):
            return None

        jnp = require_jax_numpy()

        critic = self.critic
        w = self.algorithm.params(self.train_state)["critic"]

        # J(x) = −V_w(x)
        def cost_to_go(x):
            J = -float(critic.value(w, jnp.asarray(x, dtype=float)))
            return J

        return cost_to_go

    def rollout_law(self, x0, tf=None, key=None) -> Trajectory:
        """The deterministic law on the training grid, input held per period; disturbances drawn only with a ``key``."""
        jax = require_jax()
        jnp = require_jax_numpy()

        env = self.env
        pi = self.action_jit
        theta = self.algorithm.params(self.train_state)["policy"]

        # N = t_f / dt control periods, over the episode unless tf is given
        n_steps = int(round((env.tf if tf is None else float(tf)) / env.dt))

        # Closed loop on the grid: u_k = π(x_k), then one control period of the plant
        def body(carry, key):
            x, t = carry
            u = pi(theta, x)
            x_next, t_next, _, _, _ = env.step(x, t, u, key)
            return (x_next, t_next), (x, u)

        # The N periods from x0 at t = 0, the disturbances drawn only with a key
        keys = None if key is None else jax.random.split(key, n_steps)
        x0 = jnp.asarray(x0, dtype=float)
        (x_end, _), (xs, us) = jax.lax.scan(body, (x0, 0.0), keys, length=n_steps)

        # N + 1 samples: the final state appended, the last input held at it
        xs = np.concatenate([np.asarray(xs), np.asarray(x_end)[None]])
        us = np.concatenate([np.asarray(us), np.asarray(us)[-1:]])
        t = env.dt * np.arange(n_steps + 1)

        return Trajectory(t=t, x=xs.T, u=us.T)


@dataclass(frozen=True)
class ReinforcementLearningRecord:
    """
    The training facts of a learned law: plant steps, training time, and the last mean episode cost.

    ``cost`` is the mean episode cost ``J`` of the last collection, positive
    and falling. ``history`` holds one dict per learning iteration
    (``timesteps``, ``cost``, ``n_episodes``, the algorithm's losses); returns
    stay internal to the algorithms.
    """

    algorithm: str
    timesteps: int
    train_time_s: float
    cost: float
    weights_finite: bool
    history: list

    @property
    def success(self) -> bool:
        """The training finished with finite weights."""
        return bool(self.weights_finite)

    # Display

    def __str__(self) -> str:
        return (
            f"{self.algorithm}: {self.timesteps} plant steps in "
            f"{self.train_time_s:.1f} s, mean episode cost {self.cost:.2f}"
        )
