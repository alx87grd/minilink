"""The rollout environment: a stochastic planning problem as pure step functions, on NumPy or JAX."""

import warnings
import zlib

import numpy as np

from minilink.core.backends import (
    BACKEND_JAX,
    BACKEND_NUMPY,
    array_module,
    is_jax_key,
    numpy_generator,
    require_jax,
)
from minilink.core.compile.evaluators.stepping import require_step_knobs
from minilink.core.params import flatten, merge_tree
from minilink.core.roles import command_port

###############################################################################
# Public API
###############################################################################


# An episode's seed of a source: a non-negative 31-bit integer
MAX_SEED = 2**31 - 1


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
    problem : PlanningProblem
        The task: plant, cost, constraint set ``X`` and its price of infeasibility,
        horizon, start (a point, a set or a law), the plant's parameter laws and
        the disturbance sources. A point start restarts every episode at
        ``x_start``.
    dt : float
        Control period (one integration step, input held).
    episode_length : float, optional
        Episode duration for an infinite-horizon problem (default 10 s);
        ignored for a finite horizon, whose episodes end at ``problem.tf``.
    solver : {"rk4", "euler"}
        Integration scheme of the step model.
    substeps : int
        Integration steps per control period, the action held over them.
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
        solver="rk4",
        substeps=1,
        backend="jax",
        training_zone=None,
    ):
        # The task: the problem, its plant and its cost, on one control period
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
        require_episode_steps(self.n_steps_per_episode, self.tf, self.dt)
        self.discount_rate = problem.discount_rate

        # Plant: the step model of one control period, on nominal or drawn parameters
        require_rollout_backend(backend)
        require_step_knobs(solver, substeps)
        self.evaluator = self.sys.compile(backend=backend, verbose=False)
        self.solver = solver
        self.substeps = int(substeps)

        # The episode's realization: the plant's laws drawn, a seed per random source
        self.laws = problem.laws.get("plant", {})
        self.sources = dict(problem.disturbances)
        random_sources = any(block.is_random for block in self.sources.values())
        self.randomizes_params = bool(self.laws) or random_sources

        # The constraint X, whose exit is a failure, and the training zone, whose
        # exit is a truncation
        self.X = problem.X
        self.set_params = None
        self.training_zone = (
            self.sys.state.box if training_zone is None else training_zone
        )

        # Input vector: the action fills its port, disturbances fill theirs
        self.action_port = command_port(self.sys)
        self.m = int(self.sys.inputs[self.action_port].dim)
        self.nominal_inputs = self.nominal_port_values()

        # The price of infeasibility a failure is charged: declared on the problem,
        # else the bound of its cost over an episode
        self.infeasible_cost = self.price_of_infeasibility()

        # Noise the held-input rollout never sees is announced
        warn_inert_noise(self.sys, self.sources)

    def reset(self, key):
        """An initial state drawn from the problem's start (its law, or its one point)."""
        xp = array_module(key)

        x0 = xp.asarray(self.problem.sample_x0(key), dtype=float)

        return x0

    def sample_params(self, key):
        """
        The episode's realization: ``{"plant": draws, port: {"seed": s}}`` (empty when nothing is drawn).

        Each law of the plant is drawn once, at its path; each random source gets
        the episode's seed (RD-D21), so two episodes from one training key see
        distinct noise.
        """
        if not self.randomizes_params:
            return {}

        draws = self.problem.sample_params(key) if self.laws else {}
        realization = {"plant": draws.get("plant", {})} if self.laws else {}
        realization.update(self.source_seeds(key))

        return realization

    def source_seeds(self, key):
        """The episode's seed of each random source, ``{port: {"seed": s}}``."""
        return {
            port: {"seed": episode_seed(key, port)}
            for port, block in self.sources.items()
            if block.is_random
        }

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
        tf = self.tf
        finite_horizon = self.finite_horizon
        g = self.cost.g
        h = self.cost.h
        X = self.X
        zone = self.training_zone
        set_params = self.set_params

        # Plant input: the action on its port, each source's signal at t on theirs
        u_full = self.input_vector(u, t, params)

        # Dynamics: x_{k+1} = f(x_k, u_k, w_k) over one control period, input held
        x_next = self.plant_step(x, u_full, t, params)
        t_next = t + dt

        # Reward: r_k = −g(x_k, u_k) dt, the stage cost on the plant's whole input
        # with its sign flipped; the discount enters the return once, through the
        # algorithm, never the reward
        reward = -g(x, u_full, t) * dt

        # A step that blew up ends the episode; the last finite state stands in
        # for the next one so rewards, values and the reset stay finite
        finite = xp.all(xp.isfinite(x_next))
        x_next = xp.where(finite, x_next, x)

        # Leaving X is a failure; leaving the training zone is not; the horizon ends
        failed = ~xp.all(X.margin(x_next, t_next, set_params) >= 0.0) | ~finite
        left_zone = ~xp.all(zone.margin(x_next, t_next, set_params) >= 0.0)
        horizon_reached = xp.asarray(t_next >= tf - 0.5 * dt)

        # A failure pays the price of infeasibility; a finite horizon pays h(x_N)
        penalty = self.infeasible_penalty(x_next, t_next)
        reward = reward - xp.where(failed, penalty, 0.0)
        if finite_horizon:
            reward = reward - horizon_reached * h(x_next, t_next)

        # What is paid is terminal; what is left unpaid is truncated
        terminated = failed | (horizon_reached & finite_horizon)
        truncated = (left_zone & ~failed) | (horizon_reached & (not finite_horizon))

        return x_next, t_next, reward, terminated, truncated

    def plant_step(self, x, u_full, t, params=None):
        """One control period of the plant, on the nominal or the episode's parameters."""
        plant = (params or {}).get("plant")
        p = self.full_params(plant) if plant else None

        return self.evaluator.step(
            x, u_full, t, p, dt=self.dt, solver=self.solver, substeps=self.substeps
        )

    def input_vector(self, u, t, params=None):
        """
        Full plant input: the action on its port, each source's signal at ``t`` on theirs.

        The action is applied as given; port bounds are information, not
        saturation. A random source reads the episode's seed from ``params``;
        without one it is at its mean (RD-D17).
        """
        xp = array_module(u)

        # Every port at its nominal value, the action on its own
        ports = dict(self.nominal_inputs)
        ports[self.action_port] = u

        # Each source's output at t on its port, with the episode's seed
        for port, block in self.sources.items():
            ports[port] = source_signal(block, t, (params or {}).get(port), xp)

        # The ports stacked in port order
        u_full = xp.concatenate(
            [xp.asarray(ports[port_id], dtype=float) for port_id in ports]
        )

        return u_full

    def infeasible_penalty(self, x, t):
        """The price of infeasibility at ``(x, t)``: one number for every exit."""
        return self.infeasible_cost

    def full_params(self, params):
        """The plant's params with the episode's draws merged in (other entries untouched)."""
        return merge_tree(self.sys.params, params)

    def describe(self) -> str:
        """One line naming the horizon, the constraint's price, the training zone and the randomization."""
        kind = "finite horizon" if self.finite_horizon else "infinite horizon"
        price = self.infeasible_cost
        charged = f"{price:.3g}"
        names = [".".join(path) for path, _ in flatten(self.laws)]
        randomized = f", randomized params {sorted(names)}" if self.laws else ""

        return (
            f"{kind}, episodes of {self.tf:g} s at dt={self.dt:g}; a failure (leaving X, "
            f"or a non-finite state) terminates and is charged {charged}; leaving the "
            f"training zone truncates (value bootstrapped){randomized}"
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def nominal_port_values(self):
        """The nominal value of each input port, in port order."""
        u_nominal = np.asarray(self.sys.get_u_from_input_ports(), dtype=float)

        # The nominal input vector cut into its ports
        values = {}
        i = 0
        for port_id, port in self.sys.inputs.items():
            values[port_id] = u_nominal[i : i + port.dim]
            i += port.dim

        return values

    def price_of_infeasibility(self):
        """The problem's ``infeasible_cost`` when a number, else the bound of its cost over the episode."""
        return self.problem.infeasibility_price(self.tf, dt=self.dt)


###############################################################################
# Internal machinery
###############################################################################


def require_episode_steps(n_steps, tf, dt):
    """Refuse an episode shorter than half a control period: it holds no step."""
    if n_steps == 0:
        raise ValueError(
            f"an episode of {tf:g} s is shorter than half the control period dt={dt:g} s "
            "and holds no step; lengthen the episode or shorten dt"
        )


def episode_seed(key, port):
    """The episode's integer seed of the source on ``port``, from the episode's key."""
    if is_jax_key(key):
        jax = require_jax()
        k = jax.random.fold_in(key, zlib.crc32(port.encode()))
        return jax.random.randint(k, (), 0, MAX_SEED)

    return int(numpy_generator(key).integers(0, MAX_SEED))


def source_signal(block, t, realization, xp):
    """A source block's output at ``t``: the episode's seed when given, its mean otherwise."""
    params = block.realize(None) if block.is_random else block.params
    if realization is not None:
        params = dict(params, **realization)
    nothing = xp.zeros(0)

    w = block.h(nothing, nothing, t, params)

    return xp.reshape(xp.asarray(w, dtype=float), (-1,))


def warn_inert_noise(sys, disturbances):
    """Warn about noise the held-input rollout never sees: a port only ``h`` reads, a random block inside the plant."""
    # Disturbance ports whose Jacobian on f is zero at the nominal point
    inert = [
        port
        for port in disturbances
        if not np.any(sys.jacobian("f", port, method="fd"))
    ]
    if inert:
        warnings.warn(
            f"disturbances on {inert} do not reach f at the nominal point: the stepped "
            "rollout feeds the true state to the law and never calls h, so these draws "
            "change nothing here",
            stacklevel=3,
        )

    # A random block inside the plant draws the same signal every episode
    if sys.is_random:
        warnings.warn(
            "the plant holds a random block with a fixed seed: every trial and episode "
            "sees the same noise signal",
            stacklevel=3,
        )


def require_rollout_backend(backend):
    """Refuse an array backend other than ``"jax"`` and ``"numpy"``."""
    if backend not in (BACKEND_JAX, BACKEND_NUMPY):
        raise ValueError(f"backend must be 'jax' or 'numpy', got {backend!r}")
