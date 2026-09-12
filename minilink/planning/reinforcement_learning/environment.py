"""
The rollout environment: a stochastic planning problem as pure JAX step functions.

``RolloutEnvironment`` turns a :class:`~minilink.planning.problems.StochasticPlanningProblem`
into what a learning loop needs — ``reset(key)`` draws a start (and, when the
problem randomizes them, the plant parameters of the episode), ``step`` moves
the compiled plant one control period and scores it — with the problem's own
semantics:

- reward ``r = -g(x, u, t) dt`` (the running cost over the step, the
  left-Riemann discretization of ``int g dt`` on the control grid; the
  discount is the planner's ``gamma``, applied once in the return);
- a finite horizon ends the episode at ``tf`` with the terminal cost ``h``;
  an infinite horizon uses ``episode_length`` and *truncates* (the value of
  the next state bootstraps the return);
- leaving the allowed box ends the episode: with an ``exit_cost`` (or
  ``on_exit="terminate"``) it is *terminated* and charged, otherwise it is
  *truncated* and the critic's value at the exit state bootstraps the
  return — the Gymnasium bridge's historical behaviour, and an approximation
  a policy can exploit (see ``examples/experimental/rl/RL_README.md`` §1).
  :meth:`describe` states which rule is in force.

Every function is traceable, so collectors ``vmap`` over plants and ``scan``
over time. There is no Python per step.
"""

import numpy as np

from minilink.control.neural import action_port_of
from minilink.core.backends import require_jax_numpy
from minilink.core.sets import BoxSet

# Public API


class RolloutEnvironment:
    """
    Compiled-plant environment for a stochastic planning problem.

    Parameters
    ----------
    problem : StochasticPlanningProblem
        The task: plant, cost, box, exit rule, horizon, start distribution,
        optional parameter and disturbance distributions. A deterministic
        :class:`~minilink.planning.problems.PlanningProblem` restarts at its
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
        self.problem = problem
        self.sys = problem.sys
        self.cost = problem.require_cost()
        self.dt = float(dt)
        self.finite_horizon = problem.horizon_kind() == "finite"
        if self.finite_horizon:
            self.tf = float(problem.tf)
        else:
            self.tf = 10.0 if episode_length is None else float(episode_length)
        self.n_steps_per_episode = int(round(self.tf / self.dt))
        self.discount_rate = float(self.cost.discount_rate)

        self.evaluator = self.sys.compile(backend="jax", verbose=False)
        if integrator == "rk4":
            self.step_plant = self.evaluator.rk4_step_trace
            self.step_plant_p = self.evaluator.rk4_step_trace_p
        elif integrator == "euler":
            self.step_plant = self.evaluator.euler_step_trace
            self.step_plant_p = self.evaluator.euler_step_trace_p
        else:
            raise ValueError(f"integrator must be 'rk4' or 'euler', got {integrator!r}")

        # Randomized plant parameters: the episode carries its own draw
        self.params_distribution = dict(
            getattr(problem, "params_distribution", {}) or {}
        )
        self.randomizes_params = bool(self.params_distribution)

        # The allowed box: problem.X when it is a box, else the state bounds
        X = problem.X
        if isinstance(X, BoxSet):
            x_lb, x_ub = X.lower, X.upper
        else:
            x_lb, x_ub = self.sys.state.lower_bound, self.sys.state.upper_bound
        self.x_lb = jnp.asarray(x_lb, dtype=float)
        self.x_ub = jnp.asarray(x_ub, dtype=float)
        self.n = int(self.sys.n)

        # The exit rule: charge and terminate, or truncate and bootstrap
        self.charge_exit = (
            problem.exit_cost is not None or problem.on_exit == "terminate"
        )

        # Stacked input vector: the action fills its port, disturbances theirs
        self.action_port = action_port_of(self.sys)
        self.u_nominal = jnp.asarray(self.sys.get_u_from_input_ports(), dtype=float)
        self.port_slices = {}
        i = 0
        for port_id, port in self.sys.inputs.items():
            self.port_slices[port_id] = slice(i, i + port.dim)
            i += port.dim
        action = self.sys.inputs[self.action_port]
        self.m = int(action.dim)
        self.u_lb = jnp.asarray(action.lower_bound, dtype=float)
        self.u_ub = jnp.asarray(action.upper_bound, dtype=float)
        self.u_mid = 0.5 * (self.u_ub + self.u_lb)  # normalized action a in [-1, 1]
        self.u_half = 0.5 * (self.u_ub - self.u_lb)

    # --- pure functions (traceable) ---

    def reset(self, key):
        """Draw an initial state (a deterministic problem restarts at ``x_start``)."""
        jnp = require_jax_numpy()
        if hasattr(self.problem, "sample_x0"):
            return jnp.asarray(self.problem.sample_x0(key), dtype=float)
        return jnp.asarray(self.problem.x_start, dtype=float)

    def sample_params(self, key):
        """The episode's parameter draws ``{name: array}`` (empty when nothing is randomized)."""
        jnp = require_jax_numpy()
        if not self.randomizes_params:
            return {}
        import jax

        return jax.tree_util.tree_map(
            lambda v: jnp.asarray(v, dtype=float), self.problem.sample_params(key)
        )

    def full_params(self, theta):
        """The plant's params with the episode's draws merged in (other entries untouched)."""
        from minilink.planning.problems import merge_params

        return merge_params(self.sys.params, theta)

    def plant_step(self, x, u_full, t, theta=None):
        """One integration step of the plant, on the nominal or the episode's parameters."""
        if not theta:
            return self.step_plant(x, u_full, t, self.dt)
        return self.step_plant_p(x, u_full, t, self.dt, self.full_params(theta))

    def input_vector(self, u, key):
        """Full plant input: action on port ``u``, fresh disturbance draws elsewhere."""
        jnp = require_jax_numpy()
        full = self.u_nominal.at[self.port_slices[self.action_port]].set(
            jnp.clip(u, self.u_lb, self.u_ub)
        )
        draws = (
            self.problem.sample_disturbances(key)
            if hasattr(self.problem, "sample_disturbances")
            else {}
        )
        for port_id, value in draws.items():
            full = full.at[self.port_slices[port_id]].set(
                jnp.asarray(value, dtype=float)
            )
        return full

    def running_cost(self, x, u, t):
        """Running cost ``g(x, u, t)`` of the applied input.

        Undiscounted on purpose: the discount enters the return once, through
        the planner's ``gamma`` (``cost.discount_factor(dt)`` by default), not
        through the per-step reward.
        """
        return self.cost.g(x, u, t)

    def step(self, x, t, u, key, params=None):
        """
        One control period: ``(x_next, t_next, reward, terminated, truncated)``.

        ``params`` is the episode's parameter draws (``None`` or empty: nominal).
        ``terminated`` ends the episode with its cost fully accounted (finite
        horizon reached, or a charged exit); ``truncated`` ends it with the
        value of ``x_next`` still to come (episode length, or an uncharged
        exit).
        """
        jnp = require_jax_numpy()
        dt = self.dt
        u_full = self.input_vector(u, key)
        x_next = self.plant_step(x, u_full, t, params)
        t_next = t + dt
        reward = (
            -self.running_cost(x, u_full[self.port_slices[self.action_port]], t) * dt
        )

        # A step that blew up ends the episode; the last finite state stands in
        # for the next one so rewards, values and the reset stay finite
        finite = jnp.all(jnp.isfinite(x_next))
        x_next = jnp.where(finite, x_next, x)
        out = jnp.any(x_next < self.x_lb) | jnp.any(x_next > self.x_ub) | ~finite
        horizon_reached = t_next >= self.tf - 0.5 * dt

        if self.charge_exit:
            penalty = self.problem.exit_penalty(x_next, t_next)
            reward = reward - out * (0.0 if penalty is None else penalty)
            exit_terminates = out
        else:
            exit_terminates = jnp.zeros((), dtype=bool)

        if self.finite_horizon:
            reward = reward - horizon_reached * self.cost.h(x_next, t_next)
            terminated = exit_terminates | horizon_reached
            truncated = out & ~exit_terminates
        else:
            terminated = exit_terminates
            truncated = horizon_reached | (out & ~exit_terminates)
        return x_next, t_next, reward, terminated, truncated

    # --- reporting helpers ---

    def describe(self) -> str:
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
        return np.asarray(self.x_lb), np.asarray(self.x_ub)
