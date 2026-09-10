"""
Monte Carlo evaluation of a controller on a stochastic planning problem (the second verb).

``solve(problem)`` finds a law; ``evaluate(problem, law)`` scores one. The
evaluator draws initial states (and disturbances) from the problem, closes the
loop with any state-feedback block, and reports the distribution of the cost
``J`` — mean, spread, worst case, failure rate — so LQR, dynamic programming,
MPC and reinforcement learning are compared on the same task with the same
numbers.

Two paths: ``backend="jax"`` runs every trial in one ``vmap`` over the
compiled plant (static laws ``u = pi(x)`` that trace); ``backend="numpy"``
runs the closed-loop :class:`~minilink.simulation.simulator.Simulator` per
trial and works with any controller, dynamic ones included.
"""

from dataclasses import dataclass

import numpy as np

from minilink.core.trajectory import Trajectory

# Public API


@dataclass
class MonteCarloReport:
    """Per-trial costs and their summary; ``failed`` marks trials that left the box."""

    J: np.ndarray
    failed: np.ndarray
    x0: np.ndarray
    trajectories: list | None = None

    @property
    def mean(self) -> float:
        return float(np.mean(self.J))

    @property
    def std(self) -> float:
        return float(np.std(self.J))

    @property
    def worst(self) -> float:
        return float(np.max(self.J))

    @property
    def failure_rate(self) -> float:
        return float(np.mean(self.failed))

    def __str__(self) -> str:
        return (
            f"J over {self.J.size} trials: mean {self.mean:.2f}, std {self.std:.2f}, "
            f"worst {self.worst:.2f}, failure rate {100 * self.failure_rate:.0f}%"
        )


class MonteCarloEvaluator:
    """
    Score a controller over draws of a stochastic planning problem.

    Parameters
    ----------
    problem : StochasticPlanningProblem
        Task (plant, cost, box, exit rule, horizon, start distribution).
    dt : float
        Control period (JAX path) or sampling step (NumPy path).
    n_trials : int
        Number of draws.
    episode_length : float, optional
        Duration of each trial for an infinite-horizon problem (default 10 s).
    backend : {"jax", "numpy"}
        Vectorized compiled rollouts, or the closed-loop simulator per trial.
    seed : int
        Seed of the draws.
    record : bool
        Keep the trial trajectories (NumPy path only).
    """

    def __init__(
        self,
        problem,
        *,
        dt=0.05,
        n_trials=100,
        episode_length=None,
        backend="jax",
        seed=0,
        record=False,
    ):
        self.problem = problem
        self.dt = float(dt)
        self.n_trials = int(n_trials)
        self.episode_length = episode_length
        self.backend = backend
        self.seed = int(seed)
        self.record = bool(record)
        if backend not in ("jax", "numpy"):
            raise ValueError(f"backend must be 'jax' or 'numpy', got {backend!r}")

    def evaluate(self, controller) -> MonteCarloReport:
        """Return the :class:`MonteCarloReport` of ``controller`` on the problem."""
        if self.backend == "jax":
            return self.evaluate_jax(controller)
        return self.evaluate_numpy(controller)

    # Internal machinery

    def evaluate_jax(self, controller) -> MonteCarloReport:
        import jax
        import jax.numpy as jnp

        from minilink.planning.reinforcement_learning.environment import (
            RolloutEnvironment,
        )

        env = RolloutEnvironment(
            self.problem, dt=self.dt, episode_length=self.episode_length
        )
        n_steps = env.n_steps_per_episode
        law = static_law(controller)

        def trial(key):
            k0, k_steps = jax.random.split(key)
            x0 = env.reset(k0)

            def body(carry, key):
                x, t, J, alive, failed = carry
                u = law(x)
                x_next, t_next, reward, terminated, truncated = env.step(x, t, u, key)
                J = J - alive * reward  # cost accrues only while the episode is alive
                failed = failed | (alive & truncated & (t_next < env.tf - 0.5 * env.dt))
                failed = failed | (
                    alive & terminated & (t_next < env.tf - 0.5 * env.dt)
                )
                alive = alive & ~(terminated | truncated)
                return (x_next, t_next, J, alive, failed), None

            init = (x0, 0.0, 0.0, jnp.bool_(True), jnp.bool_(False))
            (x, t, J, alive, failed), _ = jax.lax.scan(
                body, init, jax.random.split(k_steps, n_steps)
            )
            return J, failed, x0

        keys = jax.random.split(jax.random.PRNGKey(self.seed), self.n_trials)
        J, failed, x0 = jax.jit(jax.vmap(trial))(keys)
        return MonteCarloReport(np.asarray(J), np.asarray(failed), np.asarray(x0))

    def evaluate_numpy(self, controller) -> MonteCarloReport:
        problem = self.problem
        sys = problem.sys
        cost = problem.require_cost()
        rng = np.random.default_rng(self.seed)
        finite = problem.horizon_kind() == "finite"
        tf = float(problem.tf) if finite else float(self.episode_length or 10.0)
        x_lb, x_ub = (
            np.asarray(sys.state.lower_bound),
            np.asarray(sys.state.upper_bound),
        )

        J = np.zeros(self.n_trials)
        failed = np.zeros(self.n_trials, dtype=bool)
        x0s = problem.sample_x0(rng, n=self.n_trials)
        trajectories = [] if self.record else None
        x0_saved = np.asarray(sys.x0, dtype=float).copy()
        for i, x0 in enumerate(x0s):
            sys.x0 = np.asarray(x0, dtype=float)
            cl_sys = controller @ sys
            traj = cl_sys.compute_trajectory(tf=tf, dt=self.dt, verbose=False)
            traj = cl_sys.reconstruct_internal_signals(traj)
            u = traj.signals[next(k for k in traj.signals if k.endswith(":u"))]
            inside = np.all(
                (traj.x >= x_lb[:, None]) & (traj.x <= x_ub[:, None]), axis=0
            )
            exit_index = int(np.argmin(inside)) if not inside.all() else traj.n_samples
            kept = Trajectory(
                t=traj.t[: exit_index + 1],
                x=traj.x[:, : exit_index + 1],
                u=u[:, : exit_index + 1],
            )
            J[i] = cost.evaluate_trajectory(kept).signals["cost"][0, -1]
            if exit_index < traj.n_samples:
                failed[i] = True
                penalty = problem.exit_penalty(
                    traj.x[:, exit_index], traj.t[exit_index]
                )
                J[i] += 0.0 if penalty is None else float(penalty)
            elif finite:
                J[i] += float(cost.h(traj.x[:, -1], traj.t[-1]))
            if trajectories is not None:
                trajectories.append(traj)
        sys.x0 = x0_saved
        return MonteCarloReport(J, failed, np.asarray(x0s), trajectories)


def static_law(controller):
    """
    ``u = pi(x)`` of any static feedback block, traceable under JAX.

    Compiles the block and evaluates its control port with the measurement
    port fed the plant state and the other inputs at their nominal values
    (a reference port left unconnected holds its set point, as in a diagram).
    """
    import jax.numpy as jnp

    from minilink.core.feedback import feedback_ports

    roles = feedback_ports(controller)
    if roles is None:
        raise ValueError("the controller must declare measurement and control ports")
    if int(controller.n) > 0:
        raise ValueError(
            "the JAX evaluator handles static laws u = pi(x); use backend='numpy' "
            "for a controller with internal state"
        )
    evaluator = controller.compile(backend="jax", verbose=False)
    u_nominal = jnp.asarray(controller.get_u_from_input_ports(), dtype=float)
    start = 0
    for port_id, port in controller.inputs.items():
        if port_id == roles.measurement:
            measurement = slice(start, start + port.dim)
        start += port.dim
    x_ctl = jnp.zeros(0)

    def law(x):
        u_in = u_nominal.at[measurement].set(x)
        return evaluator.outputs_trace(x_ctl, u_in, 0.0)[roles.control]

    return law
