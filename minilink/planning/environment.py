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
from minilink.core.distributions import child_seed
from minilink.core.params import flatten, merge_tree
from minilink.core.roles import command_port
from minilink.core.wiring import WiredDiagramMixin

###############################################################################
# Public API
###############################################################################


# An episode's seed of a source: a non-negative 31-bit integer
MAX_SEED = 2**31 - 1


class RolloutEnvironment:
    """
    A planning problem seen by a learner: ``reset`` draws a start, ``step`` runs one control period.

    The problem is compiled once to its step face. ``step`` is the face's
    ``transition``: the plant over ``dt`` with the input held and the period's
    stage cost ``c_k = ∫₀^dt e^{−ρτ} g dτ`` integrated beside it by the same
    scheme, the reward ``−c_k``; it tells how the episode ends: at the
    horizon, at a failure (the state left ``X`` or the input left ``U``, checked
    at the sample as the judge does: a priced terminal event, a training device
    that ends the episode early), or at the edge of the training zone (a
    truncation: the model is not studied there, the value of the state stands in
    for the rest). On the JAX backend every method traces, so collectors ``vmap`` over plants and ``scan`` over time with
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
        self.t_start = float(problem.t_start)

        # Plant and stage cost: the problem's step face at dt, one transition per
        # control period, on nominal or drawn parameters
        require_rollout_backend(backend)
        require_step_knobs(solver, substeps)
        self.solver = solver
        self.substeps = int(substeps)
        self.face = problem.compile(dt=self.dt, solver=solver, substeps=self.substeps)
        self.face_params = self.face.params

        # The episode's realization: the plant's laws drawn, a seed per random source
        self.laws = problem.laws.get("plant", {})
        self.sources = dict(problem.disturbances)
        require_stepped_sources(self.sys, self.sources)
        random_sources = any(block.is_random for block in self.sources.values())
        self.randomizes_params = bool(self.laws) or random_sources

        # The constraints X and U, whose violation is a failure (the judge's rule,
        # checked at the sample), and the training zone, whose exit is a truncation
        self.X = problem.X
        self.U = problem.U
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
        the episode's seed, so two episodes from one training key see
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

    def step(self, x, t, u, key, params=None, *, t_origin=None):
        """
        One control period: ``(x_next, t_next, reward, terminated, truncated)``.

        ``params`` is the episode's parameter draws (``None`` or empty: nominal);
        ``key`` draws the disturbances (``None``: nominal). ``terminated`` ends
        the episode with its cost fully accounted: the finite horizon is
        reached, or the state left ``X`` (a failure, charged). ``truncated``
        ends it with the value of ``x_next`` still to come: the episode length
        is reached, or the state left the training zone. ``t_origin`` is the
        episode's start when it is not the problem's ``t_start`` (a receding
        horizon solved from a later tick): the horizon is ``tf`` after it.
        """
        xp = array_module(x)

        dt = self.dt
        tf = self.tf
        t_start = self.t_start if t_origin is None else t_origin
        finite_horizon = self.finite_horizon
        face = self.face
        gamma = face.gamma
        X = self.X
        U = self.U
        zone = self.training_zone
        set_params = self.set_params
        tree = self.episode_tree(params)

        # Plant input: the action on its port, each source's signal at t on theirs
        u_full = self.input_vector(u, t, params)

        # Transition: x_{k+1}, c_k over one control period, input held
        x_next, c = face.transition_at(x, u_full, t, tree)
        t_next = t + dt

        # Reward: r_k = −c_k, the stage cost on the plant's whole input with its
        # sign flipped; the discount enters the return once, through the
        # algorithm, never the reward
        reward = -c

        # A step that blew up ends the episode; the last finite state stands in
        # for the next one so rewards, values and the reset stay finite
        finite = xp.all(xp.isfinite(x_next))
        x_next = xp.where(finite, x_next, x)

        # Leaving X or U is a failure; leaving the training zone is not; the horizon ends
        out_of_U = ~xp.all(U.margin(u_full, x, t, set_params) >= 0.0)
        out_of_X = ~xp.all(X.margin(x_next, t_next, set_params) >= 0.0)
        failed = out_of_X | out_of_U | ~finite
        left_zone = ~xp.all(zone.margin(x_next, t_next, set_params) >= 0.0)
        horizon_reached = xp.asarray(t_next >= t_start + tf - 0.5 * dt)

        # A failure pays the price of infeasibility; a finite horizon pays h(x_N),
        # one tick later than the last stage cost: r_{N−1} = −c_{N−1} − γ h(x_N),
        # so the return Σ γᵏ r_k carries γᴺ h(x_N), as the face's J does
        penalty = self.infeasible_penalty(x_next, t_next)
        reward = reward - xp.where(failed, penalty, 0.0)
        if finite_horizon:
            h_N = face.terminal(x_next, t_next, tree)
            reward = reward - horizon_reached * gamma * h_N

        # What is paid is terminal; what is left unpaid is truncated
        terminated = failed | (horizon_reached & finite_horizon)
        truncated = (left_zone & ~failed) | (horizon_reached & (not finite_horizon))

        return x_next, t_next, reward, terminated, truncated

    def plant_step(self, x, u_full, t, params=None):
        """``x_next`` alone of one control period, on the nominal or the episode's parameters (no cost evaluated)."""
        tree = self.episode_tree(params)

        return self.face.step_at(x, u_full, t, tree["plant"])

    def episode_tree(self, params=None):
        """The face's tree with the episode's plant draws merged in (none: the face's own)."""
        plant = (params or {}).get("plant")
        if not plant:
            return self.face_params

        return dict(
            self.face_params, plant=merge_tree(self.face_params["plant"], plant)
        )

    def input_vector(self, u, t, params=None):
        """
        Full plant input: the action on its port, each source's signal at ``t`` on theirs.

        The action is applied as given; port bounds are information, not
        saturation. A random source reads the episode's seed from ``params``;
        without one it is at its mean.
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
            f"{kind}, episodes of {self.tf:g} s at dt={self.dt:g}; a failure (leaving X or U, "
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
    """The episode's integer seed of the source on ``port``, from the episode's key: one stream per port."""
    if is_jax_key(key):
        jax = require_jax()
        k = jax.random.fold_in(key, zlib.crc32(port.encode()))
        return jax.random.randint(k, (), 0, MAX_SEED)
    if isinstance(key, (int, np.integer)):
        return child_seed(int(key), port)

    return int(numpy_generator(key).integers(0, MAX_SEED))


def require_stepped_sources(sys, sources):
    """Refuse a source the stepped episode cannot feed: a port the plant lacks, a state, a diagram."""
    for port, block in sources.items():
        if port not in sys.inputs:
            raise ValueError(
                f"the disturbance on {port!r} has no port on {sys.name} "
                f"(inputs {list(sys.inputs)}): the RL episode feeds plant ports only"
            )
        if isinstance(block, WiredDiagramMixin) or block.n > 0:
            raise TypeError(
                f"the source on {port!r} ({block.name}) has a state or blocks of its own: "
                "the RL episode evaluates a source's h at t; give a memoryless source"
            )


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
