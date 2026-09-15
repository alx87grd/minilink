"""The rollout environment: a stochastic planning problem as pure step functions, on NumPy or JAX."""

import numpy as np

from minilink.control.neural import action_port_of
from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY, array_module
from minilink.core.sets import BoxSet
from minilink.planning.problems import as_stochastic, merge_params

# Public API


class RolloutEnvironment:
    """
    A planning problem seen by a learner: ``reset`` draws a start, ``step`` runs one control period.

    The plant is compiled once. ``step`` integrates it over ``dt`` with the input
    held, scores the period with the problem's own cost and tells how the episode
    ends, under the problem's exit rule and horizon. On the JAX backend every
    method traces, so collectors ``vmap`` over plants and ``scan`` over time with
    no Python per step; on NumPy the same methods drive plain loops (tabular
    learning, Monte Carlo trials).

    Parameters
    ----------
    problem : StochasticPlanningProblem
        The task: plant, cost, box, exit rule, horizon, start distribution,
        optional parameter and disturbance distributions. A deterministic
        :class:`~minilink.planning.problems.PlanningProblem` is adapted by
        :func:`~minilink.planning.problems.as_stochastic` and restarts at its
        ``x_start``.
    dt : float
        Control period (one integration step, input held).
    episode_length : float, optional
        Episode duration for an infinite-horizon problem (default 10 s);
        ignored for a finite horizon, whose episodes end at ``problem.tf``.
    integrator : {"rk4", "euler"}
        Integration scheme of one step.
    backend : {"jax", "numpy"}
        Arrays the step functions run on.
    """

    def __init__(
        self, problem, *, dt=0.05, episode_length=None, integrator="rk4", backend="jax"
    ):
        problem = as_stochastic(problem)
        self.problem = problem
        self.sys = problem.sys
        self.cost = problem.require_cost()
        self.dt = float(dt)
        self.backend = backend
        self.n = int(self.sys.n)

        # Horizon: a finite one ends at tf, an infinite one runs episodes of fixed length
        self.finite_horizon = problem.horizon_kind() == "finite"
        if self.finite_horizon:
            self.tf = float(problem.tf)
        else:
            self.tf = 10.0 if episode_length is None else float(episode_length)
        self.n_steps_per_episode = int(round(self.tf / self.dt))
        self.discount_rate = float(self.cost.discount_rate)

        # Plant: one integration step, on nominal or on drawn parameters
        self.evaluator = self.sys.compile(backend=backend, verbose=False)
        self.step_plant, self.step_plant_p = self.integration_step(integrator, backend)

        # Randomized plant parameters: the episode carries its own draw
        self.params_distribution = dict(problem.params_distribution)
        self.randomizes_params = bool(self.params_distribution)

        # Allowed box and exit rule: charge and terminate, or truncate and bootstrap
        self.x_lb, self.x_ub = self.allowed_box()
        self.charge_exit = (
            problem.exit_cost is not None or problem.on_exit == "terminate"
        )

        # Input vector: the action fills its port, disturbances fill theirs
        self.action_port = action_port_of(self.sys)
        self.m = int(self.sys.inputs[self.action_port].dim)
        self.nominal_inputs = self.nominal_port_values()

    def reset(self, key):
        """An initial state drawn from the problem's start distribution."""
        xp = array_module(key)
        return xp.asarray(self.problem.sample_x0(key), dtype=float)

    def sample_params(self, key):
        """The episode's parameter draws ``{name: array}`` (empty when nothing is randomized)."""
        if not self.randomizes_params:
            return {}
        return self.problem.sample_params(key)

    def step(self, x, t, u, key, params=None):
        """
        One control period: ``(x_next, t_next, reward, terminated, truncated)``.

        ``params`` is the episode's parameter draws (``None`` or empty: nominal);
        ``key`` draws the disturbances (``None``: nominal). ``terminated`` ends
        the episode with its cost fully accounted: the finite horizon is
        reached, or the box is left under a priced exit rule. ``truncated``
        ends it with the value of ``x_next`` still to come: the episode length
        is reached, or the box is left under an unpriced rule, which a policy
        can exploit by leaving on purpose.
        """
        xp = array_module(x)
        dt = self.dt

        # Dynamics: x_k+1 = f(x_k, u_k, w_k) over one control period, input held
        x_next = self.plant_step(x, self.input_vector(u, key), t, params)
        t_next = t + dt

        # Reward: r_k = -g(x_k, u_k) dt, the stage cost with its sign flipped; the
        # discount enters the return once, through the algorithm, never the reward
        reward = -self.cost.g(x, u, t) * dt

        # A step that blew up ends the episode; the last finite state stands in
        # for the next one so rewards, values and the reset stay finite
        finite = xp.all(xp.isfinite(x_next))
        x_next = xp.where(finite, x_next, x)

        # Leaving the allowed box, and reaching the end of the horizon
        outside = xp.any(x_next < self.x_lb) | xp.any(x_next > self.x_ub) | ~finite
        horizon_reached = xp.asarray(t_next >= self.tf - 0.5 * dt)

        # Exit rule: a priced exit pays its penalty; a finite horizon pays h(x_N)
        if self.charge_exit:
            reward = reward - outside * self.exit_penalty(x_next, t_next)
        if self.finite_horizon:
            reward = reward - horizon_reached * self.cost.h(x_next, t_next)

        # What is paid is terminal; what is left unpaid is truncated
        terminated = (outside & self.charge_exit) | (
            horizon_reached & self.finite_horizon
        )
        truncated = (outside & (not self.charge_exit)) | (
            horizon_reached & (not self.finite_horizon)
        )
        return x_next, t_next, reward, terminated, truncated

    def plant_step(self, x, u_full, t, params=None):
        """One integration step of the plant, on the nominal or the episode's parameters."""
        if not params:
            return self.step_plant(x, u_full, t, self.dt)
        return self.step_plant_p(x, u_full, t, self.dt, self.full_params(params))

    def input_vector(self, u, key):
        """
        Full plant input: the action on its port, fresh disturbance draws on theirs.

        The action is applied as given; port bounds are information, not
        saturation. Without a ``key`` the disturbance ports keep their nominal values.
        """
        xp = array_module(u)
        ports = dict(self.nominal_inputs)
        ports[self.action_port] = u
        if key is not None:
            ports.update(self.problem.sample_disturbances(key))
        return xp.concatenate(
            [xp.asarray(ports[port_id], dtype=float) for port_id in ports]
        )

    def exit_penalty(self, x, t):
        """The price of leaving the box at ``(x, t)``; zero under an unpriced termination."""
        penalty = self.problem.exit_penalty(x, t)
        return 0.0 if penalty is None else penalty

    def full_params(self, params):
        """The plant's params with the episode's draws merged in (other entries untouched)."""
        return merge_params(self.sys.params, params)

    def describe(self) -> str:
        """One line naming the horizon, the exit rule and the randomization in force."""
        kind = "finite horizon" if self.finite_horizon else "infinite horizon"
        if self.charge_exit:
            exit_rule = "leaving the box terminates and is charged"
        else:
            exit_rule = "leaving the box truncates (value bootstrapped, no charge)"
        randomized = (
            f", randomized params {sorted(self.params_distribution)}"
            if self.randomizes_params
            else ""
        )
        return (
            f"{kind}, episodes of {self.tf:g} s at dt={self.dt:g}; "
            f"{exit_rule}{randomized}"
        )

    # Internal machinery

    def integration_step(self, integrator, backend):
        """The compiled one-step integrator, without and with a params argument."""
        ev = self.evaluator
        if backend == BACKEND_JAX:
            steps = {
                "rk4": (ev.rk4_step_trace, ev.rk4_step_trace_p),
                "euler": (ev.euler_step_trace, ev.euler_step_trace_p),
            }
        elif backend == BACKEND_NUMPY:
            steps = {
                "rk4": (ev.rk4_step, ev.rk4_step_p),
                "euler": (ev.euler_step, ev.euler_step_p),
            }
        else:
            raise ValueError(f"backend must be 'jax' or 'numpy', got {backend!r}")
        if integrator not in steps:
            raise ValueError(f"integrator must be 'rk4' or 'euler', got {integrator!r}")
        return steps[integrator]

    def allowed_box(self):
        """The box episodes must stay in: the problem's ``X`` when it is a box, else the state bounds."""
        X = self.problem.X
        if isinstance(X, BoxSet):
            x_lb, x_ub = X.lower, X.upper
        else:
            x_lb, x_ub = self.sys.state.lower_bound, self.sys.state.upper_bound
        return np.asarray(x_lb, dtype=float), np.asarray(x_ub, dtype=float)

    def nominal_port_values(self):
        """The nominal value of each input port, in port order."""
        u_nominal = np.asarray(self.sys.get_u_from_input_ports(), dtype=float)
        values, i = {}, 0
        for port_id, port in self.sys.inputs.items():
            values[port_id] = u_nominal[i : i + port.dim]
            i += port.dim
        return values
