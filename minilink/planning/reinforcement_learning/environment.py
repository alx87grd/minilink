"""
The rollout environment: a stochastic planning problem as pure JAX step functions.

``RolloutEnvironment`` turns a :class:`~minilink.planning.problems.StochasticPlanningProblem`
into what a learning loop needs — ``reset(key)`` draws a start, ``step`` moves
the compiled plant one control period and scores it — with the problem's own
semantics:

- reward ``r = -g(x, u, t) dt`` (the running cost over the step);
- a finite horizon ends the episode at ``tf`` with the terminal cost ``h``;
  an infinite horizon uses ``episode_length`` and *truncates* (the value of
  the next state bootstraps the return);
- leaving the allowed box ends the episode: with an ``exit_cost`` it is
  *terminated* and charged; without one and ``on_exit="infeasible"`` it is
  truncated (bootstrap), the Gymnasium bridge's historical behaviour.

Every function is traceable, so collectors ``vmap`` over plants and ``scan``
over time. There is no Python per step.
"""

import numpy as np

from minilink.core.backends import require_jax_numpy
from minilink.core.sets import BoxSet

# Public API


class RolloutEnvironment:
    """
    Compiled-plant environment for a stochastic planning problem.

    Parameters
    ----------
    problem : StochasticPlanningProblem
        The task: plant, cost, box, exit rule, horizon, start distribution.
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

        self.evaluator = self.sys.compile(backend="jax", verbose=False)
        if integrator == "rk4":
            self.step_plant = self.evaluator.rk4_step_trace
        elif integrator == "euler":
            self.step_plant = self.evaluator.euler_step_trace
        else:
            raise ValueError(f"integrator must be 'rk4' or 'euler', got {integrator!r}")

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

        # Stacked input vector: the action fills port "u", disturbances their ports
        self.u_nominal = jnp.asarray(self.sys.get_u_from_input_ports(), dtype=float)
        self.port_slices = {}
        i = 0
        for port_id, port in self.sys.inputs.items():
            self.port_slices[port_id] = slice(i, i + port.dim)
            i += port.dim
        self.m = int(self.sys.inputs["u"].dim)
        self.u_lb = jnp.asarray(self.sys.inputs["u"].lower_bound, dtype=float)
        self.u_ub = jnp.asarray(self.sys.inputs["u"].upper_bound, dtype=float)
        self.u_mid = 0.5 * (self.u_ub + self.u_lb)  # normalized action a in [-1, 1]
        self.u_half = 0.5 * (self.u_ub - self.u_lb)

    # --- pure functions (traceable) ---

    def reset(self, key):
        """Draw an initial state from the problem (a deterministic problem restarts at ``x_start``)."""
        jnp = require_jax_numpy()
        if hasattr(self.problem, "sample_x0"):
            return jnp.asarray(self.problem.sample_x0(key), dtype=float)
        return jnp.asarray(self.problem.x_start, dtype=float)

    def input_vector(self, u, key):
        """Full plant input: action on port ``u``, fresh disturbance draws elsewhere."""
        jnp = require_jax_numpy()
        full = self.u_nominal.at[self.port_slices["u"]].set(
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

    def step(self, x, t, u, key):
        """
        One control period: ``(x_next, t_next, reward, terminated, truncated)``.

        ``terminated`` ends the episode with its cost fully accounted (finite
        horizon reached, or a charged exit); ``truncated`` ends it with the
        value of ``x_next`` still to come (episode length, or an uncharged
        exit).
        """
        jnp = require_jax_numpy()
        dt = self.dt
        u_full = self.input_vector(u, key)
        x_next = self.step_plant(x, u_full, t, dt)
        t_next = t + dt
        reward = -self.cost.g(x, u_full[self.port_slices["u"]], t) * dt

        out = jnp.any(x_next < self.x_lb) | jnp.any(x_next > self.x_ub)
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
        exit_rule = "charged exit" if self.charge_exit else "truncated exit (bootstrap)"
        return f"{kind}, episodes of {self.tf:g} s at dt={self.dt:g}, {exit_rule}"

    def bounds_numpy(self):
        return np.asarray(self.x_lb), np.asarray(self.x_ub)
