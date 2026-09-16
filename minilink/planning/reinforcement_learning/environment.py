"""The rollout environment: a stochastic planning problem as pure step functions, on NumPy or JAX."""

import warnings

import numpy as np
from scipy.stats import qmc

from minilink.control.neural import action_port_of
from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY, array_module
from minilink.core.sets import is_finite_box
from minilink.planning.problems import as_stochastic, merge_params

# Public API


class RolloutEnvironment:
    """
    A planning problem seen by a learner: ``reset`` draws a start, ``step`` runs one control period.

    The plant is compiled once. ``step`` integrates it over ``dt`` with the input
    held, scores the period with the problem's own cost and tells how the episode
    ends: at the horizon, at a failure (the state left the constraint set ``X``, a
    priced terminal event), or at the edge of the training zone (a truncation: the
    model is not studied there, the value of the state stands in for the rest). On the JAX backend every
    method traces, so collectors ``vmap`` over plants and ``scan`` over time with
    no Python per step; on NumPy the same methods drive plain loops (tabular
    learning, Monte Carlo trials).

    Parameters
    ----------
    problem : StochasticPlanningProblem
        The task: plant, cost, constraint set ``X`` and its price of infeasibility,
        horizon, start distribution, optional parameter and disturbance distributions. A deterministic
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
    training_zone : Set, optional
        Where episodes run; leaving it truncates the episode. A training
        management choice, not a constraint: the plant's state box by default.
    """

    def __init__(
        self,
        problem,
        *,
        dt=0.05,
        episode_length=None,
        integrator="rk4",
        backend="jax",
        training_zone=None,
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

        # The constraint X: leaving it is a failure, charged the price of infeasibility
        # (declared on the problem, else a bound of any feasible cost, announced once)
        self.X = problem.X
        self.set_params = problem.params.sets
        self.training_zone = (
            self.sys.state.box if training_zone is None else training_zone
        )

        # Input vector: the action fills its port, disturbances fill theirs
        self.action_port = action_port_of(self.sys)
        self.m = int(self.sys.inputs[self.action_port].dim)
        self.nominal_inputs = self.nominal_port_values()
        self.infeasible_cost = self.price_of_infeasibility()

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
        reached, or the state left ``X`` (a failure, charged). ``truncated``
        ends it with the value of ``x_next`` still to come: the episode length
        is reached, or the state left the training zone.
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

        # Leaving X is a failure; leaving the training zone is not; the horizon ends
        X, zone, set_params = self.X, self.training_zone, self.set_params
        failed = ~xp.all(X.margin(x_next, t_next, set_params) >= 0.0) | ~finite
        left_zone = ~xp.all(zone.margin(x_next, t_next, set_params) >= 0.0)
        horizon_reached = xp.asarray(t_next >= self.tf - 0.5 * dt)

        # A failure pays the price of infeasibility; a finite horizon pays h(x_N)
        reward = reward - xp.where(failed, self.infeasible_penalty(x_next, t_next), 0.0)
        if self.finite_horizon:
            reward = reward - horizon_reached * self.cost.h(x_next, t_next)

        # What is paid is terminal; what is left unpaid is truncated
        terminated = failed | (horizon_reached & self.finite_horizon)
        truncated = (left_zone & ~failed) | (
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

    def infeasible_penalty(self, x, t):
        """The price of infeasibility at ``(x, t)``: the problem's, scalar or callable."""
        price = self.infeasible_cost
        return price(x, t) if callable(price) else price

    def full_params(self, params):
        """The plant's params with the episode's draws merged in (other entries untouched)."""
        return merge_params(self.sys.params, params)

    def describe(self) -> str:
        """One line naming the horizon, the constraint's price, the training zone and the randomization."""
        kind = "finite horizon" if self.finite_horizon else "infinite horizon"
        price = self.infeasible_cost
        charged = "infeasible_cost(x, t)" if callable(price) else f"{price:.3g}"
        randomized = (
            f", randomized params {sorted(self.params_distribution)}"
            if self.randomizes_params
            else ""
        )
        return (
            f"{kind}, episodes of {self.tf:g} s at dt={self.dt:g}; a failure (leaving X, "
            f"or a non-finite state) terminates and is charged {charged}; leaving the "
            f"training zone truncates (value bootstrapped){randomized}"
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

    def price_of_infeasibility(self):
        """The problem's ``infeasible_cost``; else a bound of any feasible cost, announced once."""
        if self.problem.infeasible_cost is not None:
            return self.problem.infeasible_cost
        bound = feasible_cost_bound(
            self.problem,
            self.training_zone,
            self.action_port,
            self.dt,
            self.n_steps_per_episode,
            self.discount_rate,
        )
        if is_finite_box(self.X.bounding_box()) and np.isfinite(bound):
            warnings.warn(
                f"leaving X is charged {bound:.3g}, a bound of any feasible cost over "
                f"{self.tf:g} s; set infeasible_cost on the problem to choose",
                stacklevel=3,
            )
        return bound

    def nominal_port_values(self):
        """The nominal value of each input port, in port order."""
        u_nominal = np.asarray(self.sys.get_u_from_input_ports(), dtype=float)
        values, i = {}, 0
        for port_id, port in self.sys.inputs.items():
            values[port_id] = u_nominal[i : i + port.dim]
            i += port.dim
        return values


def feasible_cost_bound(
    problem, zone, action_port, dt, n_steps, discount_rate, samples=4096
):
    """
    A bound of the discounted cost of any trajectory that stays in the zone.

    Samples the running cost on the box of the zone (else of ``X``) and of the
    action port, the other ports at their nominal values (a Halton set), for its
    maximum ``g_max``, then sums it over the episode with the discount:
    ``g_max dt (1 - gamma^N) / (1 - gamma)``. A failure charged this much is
    never preferable to any feasible continuation (the penalty method). ``+inf``
    when no finite box bounds the cost.
    """
    sys = problem.sys
    X_box = zone.bounding_box()
    if not is_finite_box(X_box):
        X_box = problem.X.bounding_box()
    U_box = sys.inputs[action_port].box
    if not (is_finite_box(X_box) and is_finite_box(U_box)):
        return np.inf
    lower = np.concatenate([X_box.lower, U_box.lower])
    upper = np.concatenate([X_box.upper, U_box.upper])
    unit = qmc.Halton(d=lower.size, scramble=False).random(int(samples) + 1)[1:]
    points = lower + unit * (upper - lower)
    n = X_box.dim
    g = problem.cost.g
    u_full = np.asarray(sys.get_u_from_input_ports(), dtype=float)
    action = sys.get_input_port_slice(action_port)

    def running_cost(point):
        u = u_full.copy()
        u[action] = point[n:]
        return float(g(point[:n], u, 0.0))

    g_max = max(running_cost(p) for p in points)

    # sum_k gamma^k g_max dt over N steps, gamma = exp(-rho dt)
    gamma = float(np.exp(-discount_rate * dt))
    if gamma >= 1.0:
        return g_max * dt * n_steps
    return g_max * dt * (1.0 - gamma**n_steps) / (1.0 - gamma)
