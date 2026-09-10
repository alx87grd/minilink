"""
Reinforcement learning as a planner: train a neural law on a stochastic planning problem.

:class:`ReinforcementLearningPlanner` mirrors the dynamic-programming planner:
the problem states the task, the planner owns the workflow, and the result is
a :class:`~minilink.planning.results.PolicyPlan` whose law is a
:class:`~minilink.control.neural.NeuralPolicyController` block. The training
loop is generic — collect experience, update — and the update rule is an
:class:`~minilink.planning.reinforcement_learning.algorithms.Algorithm`
(``"ppo"`` on-policy, ``"sac"`` off-policy), so a new method is one file, not
a new planner.

The common setup is one object::

    planner = ReinforcementLearningPlanner(problem, dt=0.05, hidden=(64, 64))
    planner.solve(timesteps=200_000)
    cl_sys = planner.get_controller() @ plant
"""

import math
import time

import numpy as np

from minilink.control.neural import NeuralPolicyController
from minilink.core.backends import require_jax_numpy
from minilink.core.trajectory import Trajectory
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

# Public API


def jax_leaves(tree):
    import jax

    return jax.tree_util.tree_leaves(tree)


class ReinforcementLearningPlanner(Planner):
    """
    Policy-family planner that learns ``u = pi(x)`` by reinforcement learning.

    The training objective is the expected discounted return on the control
    grid, ``E[-sum_k gamma^k g(x_k, u_k) dt]`` under the problem's exit rule;
    the reported ``cost`` of a plan is the problem's Monte Carlo score of the
    deterministic law (:class:`~minilink.planning.evaluation.MonteCarloEvaluator`).

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
        Discount factor; default the cost's ``discount_factor(dt)`` when the
        cost declares a rate, else 0.99.
    log_std_init : float
        Initial exploration log-std on the normalized action (on-policy).
    seed : int
        Seed of the weights, exploration and starts.

    Attributes
    ----------
    controller : NeuralPolicyController
        The law, updated in place after every learning iteration.
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
        seed=0,
        verbose=1,
        **algorithm_kwargs,
    ):
        super().__init__(problem)
        jnp = require_jax_numpy()
        import jax

        if getattr(problem, "criterion", "expectation") != "expectation":
            raise NotImplementedError(
                "reinforcement learning optimizes the expectation; "
                f"criterion={problem.criterion!r} is only reported by MonteCarloEvaluator"
            )

        self.dt = float(dt)
        self.n_envs = int(n_envs)
        self.n_steps = int(n_steps)
        self.verbose = verbose
        self.num_timesteps = 0
        self.train_time = 0.0
        self.history = []

        self.env = RolloutEnvironment(
            problem, dt=dt, episode_length=episode_length, integrator=integrator
        )
        if verbose:
            print(f"ReinforcementLearningPlanner: {self.env.describe()}")
        sys = problem.sys
        self.controller = policy or NeuralPolicyController(
            sys,
            features=features,
            hidden=hidden,
            activation=activation,
            normalize=normalize,
            seed=seed,
        )
        ctl = self.controller
        n_features = int(ctl.mlp.inputs["u"].dim)
        m = self.env.m

        # Discount: the cost's declared rate, else the usual default
        cost = problem.require_cost()
        if gamma is None:
            gamma = cost.discount_factor(dt) if cost.discount_rate > 0 else 0.99
        self.gamma = float(gamma)

        if isinstance(algorithm, Algorithm):
            self.algorithm = algorithm
        elif algorithm in ALGORITHMS:
            self.algorithm = ALGORITHMS[algorithm](gamma=self.gamma, **algorithm_kwargs)
        else:
            raise ValueError(
                f"algorithm must be one of {sorted(ALGORITHMS)} or an Algorithm"
            )
        on_policy = self.algorithm.on_policy

        # The family decides the head and the critic; the block gets the matching squash
        if on_policy:
            self.head = GaussianHead(m, log_std_init)
            self.critic = ValueFunction(
                ctl.observe, n_features, hidden, activation, seed=seed + 1
            )
            critic_params = self.critic.init()
            functions = PolicyFunctions(
                mean=lambda actor, x: ctl.mean_action(x, {"mlp": actor}),
                head=self.head,
                observe=ctl.observe,
                m=m,
                value=self.critic.value,
            )
        else:
            ctl.squash = getattr(self.algorithm, "squash", "tanh")
            self.head = SquashedGaussianHead(
                ctl.observe, n_features, m, hidden, activation, seed=seed + 3
            )
            self.critic = QFunction(
                ctl.observe, n_features, m, hidden, activation, seed=seed + 1
            )
            twin = QFunction(
                ctl.observe, n_features, m, hidden, activation, seed=seed + 2
            )
            critic_params = {"q1": self.critic.init(), "q2": twin.init()}
            functions = PolicyFunctions(
                mean=lambda actor, x: ctl.mean_action(x, {"mlp": actor}),
                head=self.head,
                observe=ctl.observe,
                m=m,
                q=self.critic.value,
            )
        self.algorithm.bind(functions)

        self.key = jax.random.PRNGKey(seed)
        self.key, k_init, k_reset = jax.random.split(self.key, 3)
        params = {
            "actor": jax.tree_util.tree_map(jnp.asarray, ctl.params["mlp"]),
            "head": self.head.init(),
            "critic": jax.tree_util.tree_map(jnp.asarray, critic_params),
        }
        self.train_state = self.algorithm.init(k_init, params)
        self.carry = reset_carry(self.env, k_reset, self.n_envs)

        # One jitted call per collection and per update
        if on_policy:
            self.rollout_jit = jax.jit(
                lambda params, carry, key: rollout(
                    self.env,
                    functions,
                    params,
                    carry,
                    key,
                    n_steps=self.n_steps,
                    n_envs=self.n_envs,
                    gamma=self.gamma,
                )
            )
            self.update_jit = jax.jit(self.algorithm.update)
        else:
            alg = self.algorithm
            self.replay = ReplayBuffer(alg.buffer_size, self.env.n, m)
            self.collect_jit = jax.jit(
                lambda params, carry, key: collect_transitions(
                    self.env,
                    functions,
                    params,
                    carry,
                    key,
                    n_steps=self.n_steps,
                    n_envs=self.n_envs,
                )
            )
            gradient_steps = alg.gradient_steps or self.n_steps * self.n_envs

            def update_many(train_state, data, size, key):
                def one(state, key):
                    k_sample, k_update = jax.random.split(key)
                    idx = jax.random.randint(k_sample, (alg.batch_size,), 0, size)
                    minibatch = {k: v[idx] for k, v in data.items()}
                    return alg.update(state, minibatch, k_update)

                keys = jax.random.split(key, gradient_steps)
                state, stats = jax.lax.scan(one, train_state, keys)
                return state, {k: jnp.mean(v) for k, v in stats.items()}

            self.update_many_jit = jax.jit(update_many)
        self.action_jit = jax.jit(
            lambda params, x: ctl.action(x, {"mlp": params["actor"]})
        )

    # --- training ---

    def learn(self, timesteps):
        """Train for ``timesteps`` plant steps (rounded up to whole collections); returns ``self``."""
        jnp = require_jax_numpy()
        import jax

        per_iteration = self.n_steps * self.n_envs
        for _ in range(math.ceil(timesteps / per_iteration)):
            self.key, k_collect, k_update = jax.random.split(self.key, 3)
            t0 = time.time()
            params = self.algorithm.params(self.train_state)
            if self.algorithm.on_policy:
                batch, self.carry, last_value = self.rollout_jit(
                    params, self.carry, k_collect
                )
                batch["last_value"] = last_value
                self.train_state, stats = self.update_jit(
                    self.train_state, batch, k_update
                )
            else:
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
            jax.block_until_ready(self.train_state)
            self.train_time += time.time() - t0
            self.num_timesteps += per_iteration
            self.sync_controller()

            ep_returns = np.asarray(batch["ep_return"])
            ep_returns = ep_returns[np.isfinite(ep_returns)]
            record = {
                "timesteps": self.num_timesteps,
                "ep_return_mean": float(ep_returns.mean())
                if ep_returns.size
                else np.nan,
                "n_episodes": int(ep_returns.size),
                "fps": per_iteration / max(time.time() - t0, 1e-9),
                "elapsed": self.train_time,
                **{k: float(v) for k, v in stats.items()},
            }
            self.history.append(record)
            if self.verbose:
                shown = " | ".join(f"{k} {float(v):8.4g}" for k, v in stats.items())
                print(
                    f"steps {record['timesteps']:>8d} | "
                    f"ep_return {record['ep_return_mean']:9.1f} | "
                    f"{shown} | fps {record['fps']:7.0f} | {record['elapsed']:.0f}s"
                )
        return self

    def sync_controller(self):
        """Copy the current actor weights into the controller block."""
        actor = self.algorithm.params(self.train_state)["actor"]
        self.controller.params["mlp"] = {k: np.asarray(v) for k, v in actor.items()}

    # --- Planner interface ---

    def solve(self, timesteps=100_000, **kwargs) -> PolicyPlan:
        return self.solve_policy(timesteps=timesteps, **kwargs)

    def solve_policy(self, timesteps=100_000, n_trials=50, **kwargs) -> PolicyPlan:
        """
        Train, score the deterministic law by Monte Carlo, wrap it in a :class:`PolicyPlan`.

        ``metadata.cost`` is the mean problem cost over ``n_trials`` draws,
        ``stats["failure_rate"]`` the fraction of trials that left the box, and
        ``success`` means the training finished with finite weights and a
        finite score.
        """
        from minilink.planning.evaluation import MonteCarloEvaluator

        t0 = time.time()
        self.learn(timesteps)
        report = MonteCarloEvaluator(
            self.problem, dt=self.dt, n_trials=n_trials, episode_length=self.env.tf
        ).evaluate(self.controller)
        weights_finite = all(
            bool(np.all(np.isfinite(np.asarray(w))))
            for w in jax_leaves(self.algorithm.params(self.train_state))
        )
        metadata = SolveMetadata(
            success=bool(weights_finite and np.isfinite(report.mean)),
            message=f"{self.num_timesteps} steps, {type(self.algorithm).__name__}; {report}",
            cost=report.mean,
            solve_time_s=time.time() - t0,
            stats={
                "timesteps": self.num_timesteps,
                "failure_rate": report.failure_rate,
                "worst": report.worst,
                "history": list(self.history),
            },
        )
        payload = {
            "controller": self.controller,
            "params": self.algorithm.params(self.train_state),
            "history": list(self.history),
        }
        return self._store_policy_plan(PolicyPlan(policy=payload, metadata=metadata))

    def get_controller(self):
        """The learned law as a controller block (``controller @ plant``)."""
        return self.controller

    def predict(self, x, deterministic=True):
        """Return ``(u, None)`` for one state or a batch of states (rows)."""
        jnp = require_jax_numpy()
        import jax

        params = self.algorithm.params(self.train_state)
        x = jnp.asarray(x, dtype=float)
        if x.ndim == 2:
            u = jax.vmap(self.action_jit, in_axes=(None, 0))(params, x)
        else:
            u = self.action_jit(params, x)
        return np.asarray(u), None

    def solve_trajectory_from(self, x0, tf=None, **kwargs) -> TrajectoryPlan:
        """Roll the learned law out from ``x0`` on the training environment."""
        jnp = require_jax_numpy()
        import jax

        env = self.env
        params = self.algorithm.params(self.train_state)
        n_steps = int(round((env.tf if tf is None else float(tf)) / env.dt))
        self.key, key = jax.random.split(self.key)

        def body(carry, key):
            x, t = carry
            u = self.action_jit(params, x)
            x_next, t_next, reward, _, _ = env.step(x, t, u, key)  # nominal params
            return (x_next, t_next), (x, u, reward)

        keys = jax.random.split(key, n_steps)
        (x_end, t_end), (xs, us, rewards) = jax.lax.scan(
            body, (jnp.asarray(x0, dtype=float), 0.0), keys
        )
        xs = np.concatenate([np.asarray(xs), np.asarray(x_end)[None]])
        us = np.concatenate([np.asarray(us), np.asarray(us)[-1:]])
        t = env.dt * np.arange(n_steps + 1)
        trajectory = Trajectory(t=t, x=xs.T, u=us.T)
        metadata = SolveMetadata(success=True, cost=-float(np.sum(np.asarray(rewards))))
        return self._store_trajectory_plan(TrajectoryPlan(trajectory, metadata))

    # --- plots ---

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
