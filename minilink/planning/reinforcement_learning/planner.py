"""Reinforcement learning as a planner: train a neural law on a stochastic planning problem."""

import math
import time
import warnings
from functools import partial

import numpy as np

from minilink.control.neural import NeuralPolicyController
from minilink.core.backends import require_jax, require_jax_numpy
from minilink.core.trajectory import Trajectory
from minilink.planning.evaluation import MonteCarloEvaluator, score_trajectory
from minilink.planning.planner import Planner
from minilink.planning.reinforcement_learning.algorithms import PPO, SAC, Algorithm
from minilink.planning.reinforcement_learning.algorithms.base import PolicyFunctions
from minilink.planning.reinforcement_learning.collect import (
    ReplayBuffer,
    collect_transitions,
    reset_carry,
    rollout,
)
from minilink.planning.reinforcement_learning.critics import QFunction, ValueFunction
from minilink.planning.reinforcement_learning.environment import RolloutEnvironment
from minilink.planning.reinforcement_learning.heads import (
    GaussianHead,
    SquashedGaussianHead,
)
from minilink.planning.results import PolicyPlan, SolveMetadata, TrajectoryPlan

ALGORITHMS = {"ppo": PPO, "sac": SAC}

#: Discount per control period when neither the algorithm, the planner nor the cost sets one.
DEFAULT_GAMMA = 0.99

# Public API


class ReinforcementLearningPlanner(Planner):
    """
    Policy-family planner that learns ``u = pi(x)`` by reinforcement learning.

    The training objective is the expected discounted return on the control
    grid, ``E[-sum_k gamma^k g(x_k, u_k) dt]`` under the problem's exit rule;
    the reported ``cost`` of a plan is the problem's Monte Carlo score of the
    deterministic law (:class:`~minilink.planning.evaluation.MonteCarloEvaluator`).
    The training loop is generic, collect experience then update, and the
    update rule is an :class:`~minilink.planning.reinforcement_learning.algorithms.Algorithm`,
    so a new method is one file, not a new planner.

    Parameters
    ----------
    problem : StochasticPlanningProblem
        Task with a start distribution, cost, box and exit rule. A plain
        :class:`~minilink.planning.problems.PlanningProblem` is accepted and
        trained from its single ``x_start``.
    dt : float
        Control period of the learned law (one RK4 step per period).
    policy : NeuralPolicyController, optional
        The law to train; default built from ``features``, ``hidden``,
        ``activation`` and ``normalize`` on the problem's plant.
    algorithm : str or Algorithm
        ``"ppo"`` (on-policy) or ``"sac"`` (off-policy), or an
        :class:`Algorithm` instance; keyword arguments not listed here go to
        the algorithm (``learning_rate``, ``n_epochs``, ``tau``, ...).
    n_envs, n_steps : int
        Plants simulated in parallel and control periods per collection.
    episode_length : float, optional
        Episode duration for an infinite-horizon problem (default 10 s).
    gamma : float, optional
        Discount per control period, used when the algorithm does not set its
        own. Default: the cost's ``discount_factor(dt)`` when the cost declares
        a rate, else 0.99 with a warning that states the effective horizon.
    log_std_init : float
        Initial exploration log-std on the normalized action (on-policy).
    seed : int
        Seed of the weights, exploration and starts.

    Attributes
    ----------
    controller : NeuralPolicyController
        The law, updated in place after every learning iteration.
    gamma : float
        The one discount the algorithm trains with.
    history : list of dict
        One record per iteration (timesteps, mean episode return, losses, fps).
    last_evaluation : MonteCarloReport or None
        The full Monte Carlo report of the last :meth:`solve_policy`.
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
        seed=0,
        verbose=1,
        **algorithm_kwargs,
    ):
        super().__init__(problem)
        jax = require_jax()

        self.dt = float(dt)
        self.n_envs = int(n_envs)
        self.n_steps = int(n_steps)
        self.verbose = verbose
        self.num_timesteps = 0
        self.train_time = 0.0
        self.last_ep_return_mean = np.nan
        self.history = []
        self.last_evaluation = None

        # The task, compiled for rollouts
        self.env = RolloutEnvironment(
            problem, dt=dt, episode_length=episode_length, integrator=integrator
        )
        self.require_expectation_criterion()
        if verbose:
            print(f"ReinforcementLearningPlanner: {self.env.describe()}")

        # The law to learn, the update rule, and the head and critic of its family
        self.controller = policy or NeuralPolicyController(
            problem.sys,
            features=features,
            hidden=hidden,
            activation=activation,
            normalize=normalize,
            seed=seed,
        )
        self.algorithm = self.build_algorithm(algorithm, algorithm_kwargs)
        self.head, self.critic, critic_weights = self.build_head_and_critic(
            hidden, activation, log_std_init, seed
        )
        functions = self.policy_functions()
        self.algorithm.bind(functions, self.resolve_discount(gamma))

        # Initial weights, train state, and the parallel plants
        self.key = jax.random.PRNGKey(seed)
        self.key, k_init, k_reset = jax.random.split(self.key, 3)
        self.train_state = self.algorithm.init(
            k_init, self.initial_weights(critic_weights)
        )
        self.carry = reset_carry(self.env, k_reset, self.n_envs)

        # One jitted call per collection, per update and per action; each family fills its own
        self.rollout_jit = None
        self.update_jit = None
        self.replay = None
        self.collect_jit = None
        self.update_many_jit = None
        self.compile_training_steps(functions)
        self.action_jit = jax.jit(
            lambda params, x: self.controller.action(x, {"mlp": params["actor"]})
        )

    @property
    def gamma(self):
        """The one discount per control period the algorithm trains with."""
        return self.algorithm.gamma

    def learn(self, timesteps):
        """Train for ``timesteps`` plant steps (rounded up to whole collections); returns ``self``."""
        jax = require_jax()
        steps_per_iteration = self.n_steps * self.n_envs
        for _ in range(math.ceil(timesteps / steps_per_iteration)):
            self.key, k_collect, k_update = jax.random.split(self.key, 3)
            t0 = time.time()

            # Collect experience, update the weights, and copy them into the controller
            batch, stats = self.collect_and_update(k_collect, k_update)
            jax.block_until_ready(self.train_state)
            self.train_time += time.time() - t0
            self.num_timesteps += steps_per_iteration
            self.sync_controller()

            record = self.record_iteration(batch, stats, t0)
            if self.verbose:
                self.print_iteration(record, stats)
        return self

    def sync_controller(self):
        """Copy the current actor weights into the controller block."""
        actor = self.algorithm.params(self.train_state)["actor"]
        self.controller.params["mlp"] = {k: np.asarray(v) for k, v in actor.items()}

    def solve(self, timesteps=100_000, **kwargs) -> PolicyPlan:
        return self.solve_policy(timesteps=timesteps, **kwargs)

    def solve_policy(self, timesteps=100_000, n_trials=50, **kwargs) -> PolicyPlan:
        """
        Train, score the deterministic law by Monte Carlo, wrap it in a :class:`PolicyPlan`.

        ``metadata.cost`` is the mean problem cost over ``n_trials`` draws,
        ``stats["failure_rate"]`` the fraction of trials that left the box, and
        ``success`` means the training finished with finite weights and a
        finite score. The full report stays on :attr:`last_evaluation`.
        """
        t0 = time.time()
        self.learn(timesteps)
        self.last_evaluation = MonteCarloEvaluator(
            self.problem, dt=self.dt, n_trials=n_trials, episode_length=self.env.tf
        ).evaluate(self.controller)
        return self._store_policy_plan(self.policy_plan(time.time() - t0))

    def get_controller(self):
        """The learned law as a controller block (``controller @ plant``)."""
        return self.controller

    def predict(self, x, deterministic=True):
        """
        The learned law ``u = pi(x)`` for one state or a batch of states (rows).

        Returns ``(u, None)``, the shape of Stable-Baselines3's ``predict``, so
        a script written for one trains and evaluates with the other.
        """
        jax, jnp = require_jax(), require_jax_numpy()
        params = self.algorithm.params(self.train_state)
        x = jnp.asarray(x, dtype=float)
        if x.ndim == 2:
            u = jax.vmap(self.action_jit, in_axes=(None, 0))(params, x)
        else:
            u = self.action_jit(params, x)
        return np.asarray(u), None

    def solve_trajectory_from(self, x0, tf=None, **kwargs) -> TrajectoryPlan:
        """Roll the learned law out from ``x0`` on the training environment, disturbances drawn."""
        jax = require_jax()
        self.key, key = jax.random.split(self.key)
        trajectory = self.rollout_law(x0, tf, key)
        J, failed = score_trajectory(self.problem, trajectory)
        metadata = SolveMetadata(success=True, cost=J, stats={"failed": bool(failed)})
        return self._store_trajectory_plan(TrajectoryPlan(trajectory, metadata))

    def nominal_trajectory(self, tf=None) -> Trajectory:
        """The learned law from the problem's start, with nominal parameters and disturbances."""
        return self.rollout_law(self.problem.x_start, tf)

    def plot_learning_curve(self, ax=None):
        """Mean return of the exploration episodes against training steps."""
        import matplotlib.pyplot as plt

        if ax is None:
            _, ax = plt.subplots(figsize=(8, 3))
        steps = [h["timesteps"] for h in self.history]
        ax.plot(steps, [h["ep_return_mean"] for h in self.history])
        ax.set_xlabel("timesteps")
        ax.set_ylabel("mean episode return")
        ax.grid(True, alpha=0.3)
        return ax

    # Internal machinery

    def require_expectation_criterion(self):
        """Reinforcement learning minimizes the expected cost; other criteria are only evaluated."""
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

    def build_head_and_critic(self, hidden, activation, log_std_init, seed):
        """The exploration head and critic of the algorithm's family, with the critic's initial weights."""
        ctl, m = self.controller, self.env.m
        n_features = int(ctl.mlp.inputs["u"].dim)

        # On-policy: a Gaussian around the mean action, a state-value baseline
        if self.algorithm.on_policy:
            head = GaussianHead(m, log_std_init)
            critic = ValueFunction(
                ctl.observe, n_features, hidden, activation, seed=seed + 1
            )
            return head, critic, critic.init()

        # Off-policy: a tanh-squashed law, twin action-value critics
        ctl.squash = getattr(self.algorithm, "squash", "tanh")
        head = SquashedGaussianHead(
            ctl.observe, n_features, m, hidden, activation, seed=seed + 3
        )
        critic = QFunction(
            ctl.observe, n_features, m, hidden, activation, seed=seed + 1
        )
        twin = QFunction(ctl.observe, n_features, m, hidden, activation, seed=seed + 2)
        return head, critic, {"q1": critic.init(), "q2": twin.init()}

    def policy_functions(self):
        """The callables the algorithm consumes: mean action, head, features, and its critic."""
        ctl, m = self.controller, self.env.m

        def mean(actor, x):
            return ctl.mean_action(x, {"mlp": actor})

        if self.algorithm.on_policy:
            return PolicyFunctions(
                mean=mean,
                head=self.head,
                observe=ctl.observe,
                m=m,
                value=self.critic.value,
            )
        return PolicyFunctions(
            mean=mean, head=self.head, observe=ctl.observe, m=m, q=self.critic.value
        )

    def resolve_discount(self, gamma):
        """The one discount: the algorithm's own, else this planner's, else the cost's rate, else the default."""
        own = self.algorithm.gamma
        if own is not None:
            if gamma is not None and float(gamma) != own:
                raise ValueError(
                    f"two discounts given: the algorithm's gamma={own} and the "
                    f"planner's gamma={gamma}; set it in one place"
                )
            return own
        if gamma is not None:
            return float(gamma)
        cost = self.env.cost
        if cost.discount_rate > 0:
            return cost.discount_factor(self.dt)
        horizon = self.dt / -math.log(DEFAULT_GAMMA)
        warnings.warn(
            f"the cost is undiscounted, so training discounts at gamma={DEFAULT_GAMMA} "
            f"per control period, an effective horizon of {horizon:.1f} s at "
            f"dt={self.dt:g}; declare discount_rate on the cost or pass gamma to choose it",
            stacklevel=3,
        )
        return DEFAULT_GAMMA

    def initial_weights(self, critic_weights):
        """The weights pytree ``{"actor", "head", "critic"}`` the train state starts from."""
        jax, jnp = require_jax(), require_jax_numpy()
        return {
            "actor": jax.tree_util.tree_map(jnp.asarray, self.controller.params["mlp"]),
            "head": self.head.init(),
            "critic": jax.tree_util.tree_map(jnp.asarray, critic_weights),
        }

    def compile_training_steps(self, functions):
        """Jit the collection and the update of the algorithm's family."""
        jax, jnp = require_jax(), require_jax_numpy()
        env, algorithm = self.env, self.algorithm

        if algorithm.on_policy:
            self.rollout_jit = jax.jit(
                partial(
                    rollout,
                    env,
                    functions,
                    n_steps=self.n_steps,
                    n_envs=self.n_envs,
                    gamma=self.gamma,
                )
            )
            self.update_jit = jax.jit(algorithm.update)
            return

        self.replay = ReplayBuffer(algorithm.buffer_size, env.n, env.m)
        self.collect_jit = jax.jit(
            partial(
                collect_transitions,
                env,
                functions,
                n_steps=self.n_steps,
                n_envs=self.n_envs,
            )
        )
        gradient_steps = algorithm.gradient_steps or self.n_steps * self.n_envs

        # Several gradient steps per collection, each on a fresh replay minibatch
        def update_many(train_state, data, size, key):
            def gradient_step(state, key):
                k_sample, k_update = jax.random.split(key)
                idx = jax.random.randint(k_sample, (algorithm.batch_size,), 0, size)
                minibatch = {k: v[idx] for k, v in data.items()}
                return algorithm.update(state, minibatch, k_update)

            keys = jax.random.split(key, gradient_steps)
            state, stats = jax.lax.scan(gradient_step, train_state, keys)
            return state, {k: jnp.mean(v) for k, v in stats.items()}

        self.update_many_jit = jax.jit(update_many)

    def collect_and_update(self, k_collect, k_update):
        """One collection of experience and the algorithm's update on it; returns ``(batch, stats)``."""
        jnp = require_jax_numpy()
        params = self.algorithm.params(self.train_state)

        if self.algorithm.on_policy:
            batch, self.carry, last_value = self.rollout_jit(
                params, self.carry, k_collect
            )
            batch["last_value"] = last_value
            self.train_state, stats = self.update_jit(self.train_state, batch, k_update)
            return batch, stats

        batch, self.carry = self.collect_jit(params, self.carry, k_collect)
        self.replay.add({k: v for k, v in batch.items() if k != "ep_return"})
        stats = {}
        if self.replay.size >= self.algorithm.learning_starts:
            self.train_state, stats = self.update_many_jit(
                self.train_state,
                self.replay.data,
                jnp.asarray(self.replay.size),
                k_update,
            )
        return batch, stats

    def record_iteration(self, batch, stats, t0):
        """Append this iteration's training record to :attr:`history` and return it."""
        # Mean return of the episodes that ended here; when none ended, the last mean stands
        ep_returns = np.asarray(batch["ep_return"])
        ep_returns = ep_returns[np.isfinite(ep_returns)]
        if ep_returns.size:
            self.last_ep_return_mean = float(ep_returns.mean())
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

    def policy_plan(self, solve_time_s) -> PolicyPlan:
        """The returned plan: the controller, its weights and history, scored by the last evaluation."""
        jax = require_jax()
        report = self.last_evaluation
        weights = self.algorithm.params(self.train_state)
        weights_finite = all(
            bool(np.all(np.isfinite(np.asarray(w))))
            for w in jax.tree_util.tree_leaves(weights)
        )
        metadata = SolveMetadata(
            success=bool(weights_finite and np.isfinite(report.mean)),
            message=f"{self.num_timesteps} steps, {type(self.algorithm).__name__}; {report}",
            cost=report.mean,
            solve_time_s=solve_time_s,
            stats={
                "timesteps": self.num_timesteps,
                "failure_rate": report.failure_rate,
                "worst": report.worst,
                "history": list(self.history),
            },
        )
        payload = {
            "controller": self.controller,
            "params": weights,
            "history": list(self.history),
        }
        return PolicyPlan(policy=payload, metadata=metadata)

    def rollout_law(self, x0, tf=None, key=None) -> Trajectory:
        """The deterministic law on the training grid, input held per period; disturbances drawn only with a ``key``."""
        jax, jnp = require_jax(), require_jax_numpy()
        env = self.env
        params = self.algorithm.params(self.train_state)
        n_steps = int(round((env.tf if tf is None else float(tf)) / env.dt))

        # Closed loop on the grid: u_k = pi(x_k), then one control period of the plant
        def body(carry, key):
            x, t = carry
            u = self.action_jit(params, x)
            x_next, t_next, _, _, _ = env.step(x, t, u, key)
            return (x_next, t_next), (x, u)

        keys = None if key is None else jax.random.split(key, n_steps)
        (x_end, _), (xs, us) = jax.lax.scan(
            body, (jnp.asarray(x0, dtype=float), 0.0), keys, length=n_steps
        )
        # N + 1 samples: the final state appended, the last input held at it
        xs = np.concatenate([np.asarray(xs), np.asarray(x_end)[None]])
        us = np.concatenate([np.asarray(us), np.asarray(us)[-1:]])
        t = env.dt * np.arange(n_steps + 1)
        return Trajectory(t=t, x=xs.T, u=us.T)
