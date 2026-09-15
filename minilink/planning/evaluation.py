"""
Scoring and Monte Carlo evaluation of policies on a planning problem (the second verb).

``solve(problem)`` finds a law; ``evaluate(problem, law)`` scores one. Both
verbs, and the reporting of every planner, use one contract for the cost of a
sampled trajectory, :func:`score_trajectory`:

- the discounted running cost ``exp(-rho t) g(x, u, t)`` integrated by the
  trapezoidal rule on the trajectory's own samples
  (:meth:`~minilink.core.costs.CostFunction.evaluate_trajectory`);
- cut at the first sample outside the allowed box ``X`` — that sample is the
  last one counted, the trial is a *failure*, and the exit is charged when the
  problem prices it (``exit_cost`` or ``on_exit="terminate"``);
- plus the terminal cost ``h(x_f, tf)`` when a finite horizon is reached.

:class:`MonteCarloEvaluator` draws initial states (and plant parameters and
disturbances when the problem randomizes them), connects the policy by what
its ports declare — a feedback block ``u = pi(x)`` through ``@``, an
open-loop source ``u = pi(t)`` (a planned input, replayed) through ``>>`` —
samples the loop on the control grid, and reports the distribution of that
score as an :class:`Evaluation`: mean, spread, worst case, failure rate. A
deterministic problem gives one trial. Three backends produce the samples:

- ``"jax"``: every trial in one ``vmap`` over the compiled plant, the law held
  over each control period (static laws that trace, and sources);
- ``"numpy"``: the same held-input RK4 samples, one trial at a time on the
  NumPy evaluator (identical numbers, no JAX needed);
- ``"simulator"``: the continuous-time loop integrated by the
  :class:`~minilink.simulation.simulator.Simulator` (any controller, dynamic
  ones included; no parameter or disturbance draws).

Every backend applies the law as the block computes it. Input-port bounds are
information, not saturation: a law that must respect them saturates inside
its own equations or through a :class:`~minilink.blocks.nonlinear.Saturation`
block.

Reinforcement learning trains on the left-Riemann discretization of the same
running cost (``r_k = -g dt``); the trapezoidal score is the reporting rule
shared with trajectory optimization and dynamic programming.
"""

import warnings
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.core.trajectory import Trajectory
from minilink.planning.problems import as_stochastic

# Public API


def score_trajectory(problem, traj: Trajectory, params=None):
    """
    Return ``(J, failed)`` of a sampled closed-loop trajectory under the problem's contract.

    ``traj`` carries the states and the applied inputs on a time grid;
    ``params`` are the cost parameters. ``failed`` is ``True`` when the
    trajectory left the allowed box before the end of the grid.
    """
    cost = problem.require_cost()
    inside = np.array([problem.X.contains(traj.x[:, k]) for k in range(traj.n_samples)])
    failed = not bool(inside.all())
    last = int(np.argmin(inside)) if failed else traj.n_samples - 1
    kept = Trajectory(
        t=traj.t[: last + 1], x=traj.x[:, : last + 1], u=traj.u[:, : last + 1]
    )
    J = float(cost.evaluate_trajectory(kept, params=params).signals["cost"][0, -1])
    if failed:
        penalty = problem.exit_penalty(traj.x[:, last], float(traj.t[last]))
        if penalty is not None:
            J += float(penalty)
    elif problem.horizon_kind() == "finite" and traj.t[-1] >= problem.tf - 1e-6:
        J += float(cost.h(traj.x[:, -1], float(traj.t[-1]), params=params))
    return J, failed


@dataclass
class Evaluation:
    """
    The cost of a policy on the problem's draws: per-trial ``J`` and its summary.

    ``failed`` marks the trials that left the allowed box; ``x0`` holds the
    starts; ``trajectories`` the recorded trials when asked for. One trial is
    a deterministic problem's whole evaluation.
    """

    J: np.ndarray
    failed: np.ndarray
    x0: np.ndarray
    trajectories: list | None = None

    @classmethod
    def of_trajectory(cls, problem, traj: Trajectory) -> "Evaluation":
        """One trial: a sampled trajectory scored under the problem's contract."""
        J, failed = score_trajectory(problem, traj)
        return cls(np.array([J]), np.array([failed]), traj.x[:, :1].T.copy(), [traj])

    @property
    def n_trials(self) -> int:
        return int(np.size(self.J))

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

    def value(self, criterion="expectation") -> float:
        """The problem's criterion: ``"expectation"`` (mean) or ``"worst_case"`` (max)."""
        if criterion == "expectation":
            return self.mean
        if criterion == "worst_case":
            return self.worst
        raise ValueError(f"unknown criterion {criterion!r}")

    def __str__(self) -> str:
        if self.n_trials == 1:
            note = ", left the box" if bool(self.failed[0]) else ""
            return f"J = {self.mean:.2f}{note}"
        return (
            f"J over {self.n_trials} trials: mean {self.mean:.2f}, std {self.std:.2f}, "
            f"worst {self.worst:.2f}, failure rate {100 * self.failure_rate:.0f}%"
        )


class MonteCarloEvaluator:
    """
    Score a policy over the draws of a planning problem.

    Parameters
    ----------
    problem : StochasticPlanningProblem
        Task (plant, cost, box, exit rule, horizon, distributions). A
        deterministic :class:`~minilink.planning.problems.PlanningProblem` is
        scored from its single start.
    dt : float
        Control period (the law is held over it) and sampling step.
    n_trials : int
        Number of draws.
    episode_length : float, optional
        Duration of each trial for an infinite-horizon problem (default 10 s).
    backend : {"jax", "numpy", "simulator"}
        See the module docstring.
    seed : int
        Seed of the draws (the same seed gives the same draws on the JAX and
        NumPy backends only through a :class:`~minilink.planning.distributions.Particles`
        start distribution; the two random streams differ otherwise).
    record : bool
        Keep the trial trajectories (NumPy and simulator backends).
    """

    BACKENDS = ("jax", "numpy", "simulator")

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
        self.problem = as_stochastic(problem)
        self.dt = float(dt)
        self.n_trials = int(n_trials)
        self.episode_length = episode_length
        self.backend = backend
        self.seed = int(seed)
        self.record = bool(record)
        if backend not in self.BACKENDS:
            raise ValueError(f"backend must be one of {self.BACKENDS}, got {backend!r}")

    @property
    def tf(self) -> float:
        problem = self.problem
        if problem.horizon_kind() == "finite":
            return float(problem.tf)
        return 10.0 if self.episode_length is None else float(self.episode_length)

    def evaluate(self, controller) -> Evaluation:
        """Return the :class:`Evaluation` of a policy block on the problem."""
        if self.backend == "jax":
            return self.evaluate_jax(controller)
        if self.backend == "numpy":
            return self.evaluate_numpy(controller)
        return self.evaluate_simulator(controller)

    # Internal machinery

    def evaluate_jax(self, controller) -> Evaluation:
        jax, jnp = require_jax(), require_jax_numpy()
        from minilink.planning.reinforcement_learning.environment import (
            RolloutEnvironment,
        )

        env = RolloutEnvironment(self.problem, dt=self.dt, episode_length=self.tf)
        dt, rho, g = env.dt, env.discount_rate, env.cost.g
        law = control_law(
            controller, dt * np.arange(env.n_steps_per_episode + 1), "jax"
        )

        def trial(key):
            k0, k_params, k_steps = jax.random.split(key, 3)
            x0 = env.reset(k0)
            params = env.sample_params(k_params)

            def body(carry, key):
                x, t, J, alive, failed = carry
                u = law(x, t)
                x_next = env.plant_step(x, env.input_vector(u, key), t, params)
                t_next = t + dt

                # Discounted running cost at both ends of the period, trapezoid while alive
                g_k = jnp.exp(-rho * t) * g(x, u, t)
                g_next = jnp.exp(-rho * t_next) * g(x_next, law(x_next, t_next), t_next)
                J = J + alive * 0.5 * (g_k + g_next) * dt

                # An exit sample is the last one counted, and is charged when priced
                out = (
                    jnp.any(x_next < env.x_lb)
                    | jnp.any(x_next > env.x_ub)
                    | ~jnp.all(jnp.isfinite(x_next))
                )
                exits = alive & out
                if env.charge_exit:
                    J = J + exits * env.exit_penalty(x_next, t_next)
                failed = failed | exits
                alive = alive & ~out
                return (x_next, t_next, J, alive, failed), None

            init = (x0, 0.0, 0.0, jnp.bool_(True), jnp.bool_(False))
            (x, t, J, alive, failed), _ = jax.lax.scan(
                body, init, jax.random.split(k_steps, env.n_steps_per_episode)
            )
            if env.finite_horizon:
                J = J + alive * env.cost.h(x, t)
            return J, failed, x0

        keys = jax.random.split(jax.random.PRNGKey(self.seed), self.n_trials)
        J, failed, x0 = jax.jit(jax.vmap(trial))(keys)
        return Evaluation(np.asarray(J), np.asarray(failed), np.asarray(x0))

    def evaluate_numpy(self, controller) -> Evaluation:
        from minilink.planning.reinforcement_learning.environment import (
            RolloutEnvironment,
        )

        problem = self.problem
        env = RolloutEnvironment(
            problem, dt=self.dt, episode_length=self.tf, backend="numpy"
        )
        rng = np.random.default_rng(self.seed)
        n_steps, dt = env.n_steps_per_episode, env.dt
        law = control_law(controller, dt * np.arange(n_steps + 1), "numpy")

        J = np.zeros(self.n_trials)
        failed = np.zeros(self.n_trials, dtype=bool)
        x0s = np.asarray(problem.sample_x0(rng, n=self.n_trials), dtype=float)
        trajectories = [] if self.record else None
        for i, x0 in enumerate(x0s):
            params = env.sample_params(rng)
            t = dt * np.arange(n_steps + 1)
            xs = np.zeros((env.n, n_steps + 1))
            us = np.zeros((env.m, n_steps + 1))
            xs[:, 0] = x0

            # The law held over each control period, one plant step at a time
            for k in range(n_steps + 1):
                us[:, k] = law(xs[:, k], t[k])
                if k == n_steps:
                    break
                u_full = env.input_vector(us[:, k], rng)
                xs[:, k + 1] = env.plant_step(xs[:, k], u_full, t[k], params)
            traj = Trajectory(t=t, x=xs, u=us)
            J[i], failed[i] = score_trajectory(problem, traj)
            if trajectories is not None:
                trajectories.append(traj)
        return Evaluation(J, failed, x0s, trajectories)

    def evaluate_simulator(self, controller) -> Evaluation:
        problem = self.problem
        sys = problem.sys
        if problem.params_distribution or problem.disturbances:
            warnings.warn(
                "the simulator backend ignores parameter and disturbance draws; "
                "use backend='jax' or 'numpy' for a randomized plant",
                stacklevel=2,
            )
        rng = np.random.default_rng(self.seed)
        J = np.zeros(self.n_trials)
        failed = np.zeros(self.n_trials, dtype=bool)
        x0s = np.asarray(problem.sample_x0(rng, n=self.n_trials), dtype=float)
        trajectories = [] if self.record else None
        x0_saved = np.asarray(sys.x0, dtype=float).copy()
        action_port = env_action_port(sys)
        try:
            for i, x0 in enumerate(x0s):
                sys.x0 = np.asarray(x0, dtype=float)

                # An open-loop source drives the plant in series; a feedback block closes the loop
                cl_sys = (
                    controller >> sys if int(controller.m) == 0 else controller @ sys
                )
                cl_traj = cl_sys.compute_trajectory(
                    tf=self.tf, dt=self.dt, verbose=False
                )

                # The plant as it ran in the loop; the cost reads its action port
                plant_traj = cl_sys.trajectory_of(sys, cl_traj)
                u = sys.get_port_values_from_u(plant_traj.u, action_port)
                traj = Trajectory(t=plant_traj.t, x=plant_traj.x, u=u)
                J[i], failed[i] = score_trajectory(problem, traj)
                if trajectories is not None:
                    trajectories.append(traj)
        finally:
            sys.x0 = x0_saved
        return Evaluation(J, failed, x0s, trajectories)


def nominal_trajectory(problem, policy, *, dt, tf=None) -> Trajectory:
    """
    The policy from the problem's start on a control grid, nominal parameters and disturbances.

    One recorded trial of the NumPy evaluator on the certainty-equivalent
    problem: the trajectory a feedback planner reports beside its policy.
    """
    nominal = problem.nominal() if problem.is_stochastic else problem
    evaluator = MonteCarloEvaluator(
        nominal, dt=dt, n_trials=1, episode_length=tf, backend="numpy", record=True
    )
    return evaluator.evaluate(policy).trajectories[0]


def env_action_port(sys) -> str:
    from minilink.control.neural import action_port_of

    return action_port_of(sys)


def control_law(controller, t, backend="jax"):
    """``u = pi(x, t)`` of a policy block: a source by its time table, a feedback block by its state law."""
    if int(controller.m) == 0:
        return time_law(controller, t, backend)
    pi = static_law(controller, backend)
    return lambda x, t: pi(x)


def time_law(source, t, backend="jax"):
    """``u = pi(t)`` of a source block, sampled on the trial grid ``t`` and held over each period."""
    empty = np.zeros(0)
    table = np.stack(
        [np.asarray(source.h(empty, empty, float(t_k)), dtype=float) for t_k in t]
    )
    dt = float(t[1] - t[0])
    if backend == "jax":
        jnp = require_jax_numpy()
        table = jnp.asarray(table)
        return lambda x, t: table[jnp.rint(t / dt).astype(int)]
    return lambda x, t: table[int(round(t / dt))]


def static_law(controller, backend="jax"):
    """
    ``u = pi(x)`` of any static feedback block, on the NumPy or JAX backend.

    Compiles the block and evaluates its control port with the measurement
    port fed the plant state and the other inputs at their nominal values
    (a reference port left unconnected holds its set point, as in a diagram).
    """
    from minilink.core.feedback import feedback_ports

    roles = feedback_ports(controller)
    if roles is None:
        raise ValueError("the controller must declare measurement and control ports")
    if int(controller.n) > 0:
        raise ValueError(
            "static laws u = pi(x) only; use backend='simulator' for a controller "
            "with internal state"
        )
    evaluator = controller.compile(backend=backend, verbose=False)
    start = 0
    for port_id, port in controller.inputs.items():
        if port_id == roles.measurement:
            measurement = slice(start, start + port.dim)
        start += port.dim

    if backend == "jax":
        jnp = require_jax_numpy()

        u_nominal = jnp.asarray(controller.get_u_from_input_ports(), dtype=float)
        x_ctl = jnp.zeros(0)

        def law(x):
            u_in = u_nominal.at[measurement].set(x)
            return evaluator.outputs_trace(x_ctl, u_in, 0.0)[roles.control]

    else:
        u_nominal = np.asarray(controller.get_u_from_input_ports(), dtype=float)
        x_ctl = np.zeros(0)

        def law(x):
            u_in = u_nominal.copy()
            u_in[measurement] = x
            return np.asarray(evaluator.outputs(x_ctl, u_in, 0.0)[roles.control])

    return law
