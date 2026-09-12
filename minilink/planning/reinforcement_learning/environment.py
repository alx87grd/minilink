"""The rollout environment: a stochastic planning problem as pure JAX step functions."""

import numpy as np

from minilink.control.neural import action_port_of
from minilink.core.backends import require_jax, require_jax_numpy
from minilink.core.sets import BoxSet
from minilink.planning.problems import as_stochastic, merge_params

# Public API


class RolloutEnvironment:
    """
    Compiled-plant environment for a stochastic planning problem.

    ``reset`` draws a start, ``step`` moves the plant one control period and
    scores it with the problem's own semantics. Every method traces under JAX,
    so collectors ``vmap`` over plants and ``scan`` over time with no Python
    per step.

    Parameters
    ----------
    problem : StochasticPlanningProblem
        The task: plant, cost, box, exit rule, horizon, start distribution,
        optional parameter and disturbance distributions. A deterministic
        :class:`~minilink.planning.problems.PlanningProblem` is adapted by
        :func:`~minilink.planning.problems.as_stochastic` and restarts at its
        ``x_start``.
    dt : float
        Control period (one RK4 step, input held).
    episode_length : float, optional
        Episode duration for an infinite-horizon problem (default 10 s);
        ignored for a finite horizon, whose episodes end at ``problem.tf``.
    integrator : {"rk4", "euler"}
        Integration scheme of one step.
    """

    def __init__(self, problem, *, dt=0.05, episode_length=None, integrator="rk4"):
        jnp = require_jax_numpy()
        problem = as_stochastic(problem)
        self.problem = problem
        self.sys = problem.sys
        self.cost = problem.require_cost()
        self.dt = float(dt)

        # Horizon: a finite one ends at tf, an infinite one runs episodes of fixed length
        self.finite_horizon = problem.horizon_kind() == "finite"
        if self.finite_horizon:
            self.tf = float(problem.tf)
        else:
            self.tf = 10.0 if episode_length is None else float(episode_length)
        self.n_steps_per_episode = int(round(self.tf / self.dt))
        self.discount_rate = float(self.cost.discount_rate)

        # Plant: one integration step, on nominal or on drawn parameters
        self.evaluator = self.sys.compile(backend="jax", verbose=False)
        self.step_plant, self.step_plant_p = self.integration_step(integrator)

        # Randomized plant parameters: the episode carries its own draw
        self.params_distribution = dict(problem.params_distribution)
        self.randomizes_params = bool(self.params_distribution)

        # Allowed box and exit rule: charge and terminate, or truncate and bootstrap
        self.x_lb, self.x_ub = self.allowed_box()
        self.n = int(self.sys.n)
        self.charge_exit = (
            problem.exit_cost is not None or problem.on_exit == "terminate"
        )

        # Input vector: the action fills its port, disturbances fill theirs
        self.action_port = action_port_of(self.sys)
        self.u_nominal = jnp.asarray(self.sys.get_u_from_input_ports(), dtype=float)
        self.port_slices = self.input_port_slices()
        action = self.sys.inputs[self.action_port]
        self.m = int(action.dim)
        self.u_lb = jnp.asarray(action.lower_bound, dtype=float)
        self.u_ub = jnp.asarray(action.upper_bound, dtype=float)

        # u = u_mid + u_half a maps the normalized action a in [-1, 1] onto the port bounds
        self.u_mid = 0.5 * (self.u_ub + self.u_lb)
        self.u_half = 0.5 * (self.u_ub - self.u_lb)

    def reset(self, key):
        """Draw an initial state from the problem's start distribution."""
        jnp = require_jax_numpy()
        return jnp.asarray(self.problem.sample_x0(key), dtype=float)

    def sample_params(self, key):
        """The episode's parameter draws ``{name: array}`` (empty when nothing is randomized)."""
        jax, jnp = require_jax(), require_jax_numpy()
        if not self.randomizes_params:
            return {}
        return jax.tree_util.tree_map(
            lambda v: jnp.asarray(v, dtype=float), self.problem.sample_params(key)
        )

    def full_params(self, theta):
        """The plant's params with the episode's draws merged in (other entries untouched)."""
        return merge_params(self.sys.params, theta)

    def plant_step(self, x, u_full, t, theta=None):
        """One integration step of the plant, on the nominal or the episode's parameters."""
        if not theta:
            return self.step_plant(x, u_full, t, self.dt)
        return self.step_plant_p(x, u_full, t, self.dt, self.full_params(theta))

    def input_vector(self, u, key):
        """
        Full plant input: the action on its port, fresh disturbance draws elsewhere.

        Without a ``key`` the disturbance ports keep their nominal values.
        """
        jnp = require_jax_numpy()

        # The nominal input vector, with the action written into its port
        u_full = self.u_nominal.at[self.port_slices[self.action_port]].set(
            jnp.clip(u, self.u_lb, self.u_ub)
        )
        if key is None:
            return u_full

        # Each disturbance port receives its own fresh draw w
        for port_id, w in self.problem.sample_disturbances(key).items():
            u_full = u_full.at[self.port_slices[port_id]].set(
                jnp.asarray(w, dtype=float)
            )
        return u_full

    def running_cost(self, x, u, t):
        """
        Running cost ``g(x, u, t)`` of the applied input.

        Undiscounted on purpose: the discount enters the return once, through
        the algorithm's ``gamma``, not through the per-step reward.
        """
        return self.cost.g(x, u, t)

    def step(self, x, t, u, key, params=None):
        """
        One control period: ``(x_next, t_next, reward, terminated, truncated)``.

        The reward is the running cost of the period with its sign flipped,
        integrated by the left-endpoint rule on the control grid. ``params`` is
        the episode's parameter draws (``None`` or empty: nominal); ``key``
        draws the disturbances (``None``: nominal). ``terminated`` ends the
        episode with its cost fully accounted: the finite horizon is reached,
        or the box is left under a charged exit rule. ``truncated`` ends it
        with the value of ``x_next`` still to come: the episode length is
        reached, or the box is left under an uncharged rule, which a policy can
        exploit by leaving on purpose.
        """
        jnp = require_jax_numpy()
        dt = self.dt

        # Dynamics: x' = f(x, u, w) over one control period, input held
        u_full = self.input_vector(u, key)
        x_next = self.plant_step(x, u_full, t, params)
        t_next = t + dt

        # Reward: r = -g(x, u, t) dt
        u_applied = u_full[self.port_slices[self.action_port]]
        reward = -self.running_cost(x, u_applied, t) * dt

        # A step that blew up ends the episode; the last finite state stands in
        # for the next one so rewards, values and the reset stay finite
        finite = jnp.all(jnp.isfinite(x_next))
        x_next = jnp.where(finite, x_next, x)

        # Leaving the allowed box, and reaching the end of the horizon
        outside = jnp.any(x_next < self.x_lb) | jnp.any(x_next > self.x_ub) | ~finite
        horizon_reached = t_next >= self.tf - 0.5 * dt

        # Exit rule: a charged exit pays its penalty and terminates
        if self.charge_exit:
            penalty = self.problem.exit_penalty(x_next, t_next)
            reward = reward - outside * (0.0 if penalty is None else penalty)
            exit_terminates = outside
        else:
            exit_terminates = jnp.zeros((), dtype=bool)

        # Horizon: a finite one pays the terminal cost h(x_N) and terminates
        if self.finite_horizon:
            reward = reward - horizon_reached * self.cost.h(x_next, t_next)
            terminated = exit_terminates | horizon_reached
            truncated = outside & ~exit_terminates
        else:
            terminated = exit_terminates
            truncated = horizon_reached | (outside & ~exit_terminates)
        return x_next, t_next, reward, terminated, truncated

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

    def bounds_numpy(self):
        """The allowed box as NumPy arrays ``(x_lb, x_ub)``."""
        return np.asarray(self.x_lb), np.asarray(self.x_ub)

    # Internal machinery

    def integration_step(self, integrator):
        """The compiled one-step integrator, without and with a params argument."""
        if integrator == "rk4":
            return self.evaluator.rk4_step_trace, self.evaluator.rk4_step_trace_p
        if integrator == "euler":
            return self.evaluator.euler_step_trace, self.evaluator.euler_step_trace_p
        raise ValueError(f"integrator must be 'rk4' or 'euler', got {integrator!r}")

    def allowed_box(self):
        """The box episodes must stay in: the problem's ``X`` when it is a box, else the state bounds."""
        jnp = require_jax_numpy()
        X = self.problem.X
        if isinstance(X, BoxSet):
            x_lb, x_ub = X.lower, X.upper
        else:
            x_lb, x_ub = self.sys.state.lower_bound, self.sys.state.upper_bound
        return jnp.asarray(x_lb, dtype=float), jnp.asarray(x_ub, dtype=float)

    def input_port_slices(self):
        """Where each input port sits in the stacked plant input vector."""
        slices, i = {}, 0
        for port_id, port in self.sys.inputs.items():
            slices[port_id] = slice(i, i + port.dim)
            i += port.dim
        return slices
