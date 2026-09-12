"""
Scoring and Monte Carlo evaluation of controllers on a planning problem (the second verb).

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
disturbances when the problem randomizes them), closes the loop with a
state-feedback block, samples the closed loop on the control grid, and
reports the distribution of that score: mean, spread, worst case, failure
rate. Three backends produce the samples:

- ``"jax"``: every trial in one ``vmap`` over the compiled plant, the law held
  over each control period (static laws ``u = pi(x)`` that trace);
- ``"numpy"``: the same held-input RK4 samples, one trial at a time on the
  NumPy evaluator (static laws; identical numbers, no JAX needed);
- ``"simulator"``: the continuous-time closed loop integrated by the
  :class:`~minilink.simulation.simulator.Simulator` (any controller, dynamic
  ones included; no parameter or disturbance draws, and the law is applied
  as the block computes it — a law that exceeds the input-port bounds is
  not clipped, and the evaluator warns).

Reinforcement learning trains on the left-Riemann discretization of the same
running cost (``r_k = -g dt``); the trapezoidal score is the reporting rule
shared with trajectory optimization and dynamic programming.
"""

import warnings
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.core.trajectory import Trajectory
from minilink.planning.problems import merge_params

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

    def value(self, criterion="expectation") -> float:
        """The problem's criterion: ``"expectation"`` (mean) or ``"worst_case"`` (max)."""
        if criterion == "expectation":
            return self.mean
        if criterion == "worst_case":
            return self.worst
        raise ValueError(f"unknown criterion {criterion!r}")

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
        Task (plant, cost, box, exit rule, horizon, distributions).
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
        self.problem = problem
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

    def evaluate(self, controller) -> MonteCarloReport:
        """Return the :class:`MonteCarloReport` of ``controller`` on the problem."""
        if self.backend == "jax":
            return self.evaluate_jax(controller)
        if self.backend == "numpy":
            return self.evaluate_numpy(controller)
        return self.evaluate_simulator(controller)

    # Internal machinery

    def evaluate_jax(self, controller) -> MonteCarloReport:
        jax, jnp = require_jax(), require_jax_numpy()
        from minilink.planning.reinforcement_learning.environment import (
            RolloutEnvironment,
        )

        env = RolloutEnvironment(self.problem, dt=self.dt, episode_length=self.tf)
        n_steps = env.n_steps_per_episode
        law = static_law(controller, backend="jax")
        charged = env.charge_exit
        finite = env.finite_horizon
        dt = env.dt
        rho = env.discount_rate
        x_lb, x_ub = env.x_lb, env.x_ub

        def trial(key):
            k0, k_theta, k_steps = jax.random.split(key, 3)
            x0 = env.reset(k0)
            theta = env.sample_params(k_theta)

            def body(carry, key):
                x, t, J, alive, failed = carry
                u = law(x)
                u_full = env.input_vector(u, key)
                g = jnp.exp(-rho * t) * env.running_cost(
                    x, u_full[env.port_slices[env.action_port]], t
                )
                x_next = env.plant_step(x, u_full, t, theta)
                t_next = t + dt
                u_next = jnp.clip(law(x_next), env.u_lb, env.u_ub)
                g_next = jnp.exp(-rho * t_next) * env.running_cost(
                    x_next, u_next, t_next
                )
                # trapezoid on this interval while alive; an exit sample is the last one counted
                J = J + alive * 0.5 * (g + g_next) * dt
                out = (
                    jnp.any(x_next < x_lb)
                    | jnp.any(x_next > x_ub)
                    | ~jnp.all(jnp.isfinite(x_next))
                )
                exits = alive & out
                if charged:
                    penalty = env.problem.exit_penalty(x_next, t_next)
                    J = J + exits * (0.0 if penalty is None else penalty)
                failed = failed | exits
                alive = alive & ~out
                return (x_next, t_next, J, alive, failed), None

            init = (x0, 0.0, 0.0, jnp.bool_(True), jnp.bool_(False))
            (x, t, J, alive, failed), _ = jax.lax.scan(
                body, init, jax.random.split(k_steps, n_steps)
            )
            if finite:
                J = J + alive * env.cost.h(x, t)
            return J, failed, x0

        keys = jax.random.split(jax.random.PRNGKey(self.seed), self.n_trials)
        J, failed, x0 = jax.jit(jax.vmap(trial))(keys)
        return MonteCarloReport(np.asarray(J), np.asarray(failed), np.asarray(x0))

    def evaluate_numpy(self, controller) -> MonteCarloReport:
        problem = self.problem
        sys = problem.sys
        evaluator = sys.compile(backend="numpy", verbose=False)
        law = static_law(controller, backend="numpy")
        rng = np.random.default_rng(self.seed)
        n_steps = int(round(self.tf / self.dt))
        dt = self.dt
        u_nominal = np.asarray(sys.get_u_from_input_ports(), dtype=float)
        slices, i = {}, 0
        for port_id, port in sys.inputs.items():
            slices[port_id] = slice(i, i + port.dim)
            i += port.dim
        action_port = env_action_port(sys)
        u_lb = np.asarray(sys.inputs[action_port].lower_bound, dtype=float)
        u_ub = np.asarray(sys.inputs[action_port].upper_bound, dtype=float)
        randomizes = bool(getattr(problem, "params_distribution", None))

        J = np.zeros(self.n_trials)
        failed = np.zeros(self.n_trials, dtype=bool)
        x0s = np.asarray(problem.sample_x0(rng, n=self.n_trials), dtype=float)
        trajectories = [] if self.record else None
        for i, x0 in enumerate(x0s):
            params = (
                merge_params(sys.params, problem.sample_params(rng))
                if randomizes
                else None
            )
            t = dt * np.arange(n_steps + 1)
            xs = np.zeros((sys.n, n_steps + 1))
            us = np.zeros((sys.inputs[action_port].dim, n_steps + 1))
            xs[:, 0] = x0
            for k in range(n_steps + 1):
                us[:, k] = np.clip(law(xs[:, k]), u_lb, u_ub)
                if k == n_steps:
                    break
                u_full = u_nominal.copy()
                u_full[slices[action_port]] = us[:, k]
                for port_id, value in problem.sample_disturbances(rng).items():
                    u_full[slices[port_id]] = value
                if params is None:
                    xs[:, k + 1] = evaluator.rk4_step(xs[:, k], u_full, t[k], dt)
                else:
                    xs[:, k + 1] = evaluator.rk4_step_p(
                        xs[:, k], u_full, t[k], dt, params
                    )
            traj = Trajectory(t=t, x=xs, u=us)
            J[i], failed[i] = score_trajectory(problem, traj)
            if trajectories is not None:
                trajectories.append(traj)
        return MonteCarloReport(J, failed, x0s, trajectories)

    def evaluate_simulator(self, controller) -> MonteCarloReport:
        problem = self.problem
        sys = problem.sys
        if getattr(problem, "params_distribution", None) or getattr(
            problem, "disturbances", None
        ):
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
        u_lb = np.asarray(sys.inputs[action_port].lower_bound, dtype=float)
        u_ub = np.asarray(sys.inputs[action_port].upper_bound, dtype=float)
        saturation_warned = False
        try:
            for i, x0 in enumerate(x0s):
                sys.x0 = np.asarray(x0, dtype=float)
                cl_sys = controller @ sys
                traj = cl_sys.compute_trajectory(tf=self.tf, dt=self.dt, verbose=False)
                traj = cl_sys.reconstruct_internal_signals(traj)
                u = traj.signals[next(k for k in traj.signals if k.endswith(":u"))]
                if not saturation_warned and (
                    np.any(u < u_lb[:, None]) or np.any(u > u_ub[:, None])
                ):
                    warnings.warn(
                        "the simulator backend applies the law unsaturated and it exceeds "
                        "the input-port bounds; the jax and numpy backends clip to them",
                        stacklevel=2,
                    )
                    saturation_warned = True
                traj = Trajectory(t=traj.t, x=traj.x, u=u)
                J[i], failed[i] = score_trajectory(problem, traj)
                if trajectories is not None:
                    trajectories.append(traj)
        finally:
            sys.x0 = x0_saved
        return MonteCarloReport(J, failed, x0s, trajectories)


def env_action_port(sys) -> str:
    from minilink.control.neural import action_port_of

    return action_port_of(sys)


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
