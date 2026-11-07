"""Planning problems: the task, deterministic or stochastic by its parts (a law where a number goes)."""

from __future__ import annotations

import warnings
import zlib
from collections.abc import Mapping
from copy import deepcopy

import numpy as np

from minilink.core.backends import is_jax_key, require_jax
from minilink.core.costs import CostFunction
from minilink.core.distributions import Distribution, child_seed, split_keys, tiled_mean
from minilink.core.inspect import (
    body_line,
    format_inspect,
    inspect_inline,
    repr_pretty,
)
from minilink.core.params import (
    flatten,
    lookup,
    merge_tree,
    require_complete,
    set_leaf,
    with_leaf,
)
from minilink.core.sets import (
    BallSet,
    BoxInputSet,
    BoxSet,
    InputSet,
    Set,
    SingletonSet,
    is_finite_box,
)
from minilink.core.system import System
from minilink.core.wiring import WiredDiagramMixin

# The problem's own numbers, held in its tree beside the parts' trees
OWN_LEAVES = ("x0", "t0", "discount_rate")

###############################################################################
# Public API
###############################################################################


class PlanningProblem:
    """
    Planning problem for a Minilink system: deterministic, or stochastic by its parts.

    The generic mathematical form is ``x(t) in X(t)``,
    ``u(t) in U(x(t), t)``, with boundary sets ``X0`` and optional ``Xf``.
    A cost can be attached for optimization and policy-planning solvers,
    but feasibility/search planners may leave it unset.

    The problem is the continuous task: the system, the admissible and boundary
    sets, the continuous ``horizon``, the optional cost. The knot count and
    the other discretization choices belong to the solvers, not to the problem.
    The parameters live on the parts; :attr:`params` holds their trees by role,
    as a diagram's are by block, and the problem's own numbers (``x0``, ``t0``,
    ``discount_rate``) beside them. It holds numbers only: the laws over them
    are :attr:`distributions`.

    A problem is stochastic by its parts: where a number goes, a law may
    go (``x0=`` a law or a set; ``distributions=`` laws in the tree's own shape),
    and a disturbance is a source block on a plant port. :meth:`realize`
    draws them into one tree, :meth:`nominal` is the problem without its
    laws (the start at its law's mean, the parts at their own numbers), and
    :attr:`is_stochastic` is read off the parts.

    The dials, in one table:

    =========== =========================================== =====================================
    dial        meaning                                     default
    =========== =========================================== =====================================
    ``X0``      the start constraint ``x(0) ∈ X0``          from ``x0``, else ``sys.x0``
    ``Xf``      the terminal constraint ``x(tf) ∈ Xf``      ``None``, a free end
    ``x0``      the evaluation start (a point or a law)     from ``X0``
    ``x_bar``   the objective's target, the cost's number   the cost's
    =========== =========================================== =====================================

    :meth:`goal_point` is the point a heuristic aims at (an initial guess's
    end, RRT's goal bias, a plot's anchor): ``Xf``'s point, else the cost's
    ``x_bar`` inside ``X``, else the start.

    Parameters
    ----------
    sys : System
        Minilink system or diagram to plan for.
    x0 : array_like, Distribution or Set, optional
        The evaluation start, where runs begin and what :meth:`realize` draws: a
        point, or a law over ``x(0)`` (a set given here is the uniform law on
        it; no randomness unless a law is given). It must belong to ``X0``. By
        default: the point of a singleton ``X0``; for a region, the centre of its
        bounding box when finite and inside, else ``sys.x0`` when inside (a
        region holding neither is refused: give ``x0``).
    cost : CostFunction, optional
        Planning cost. Required by solvers that optimize an objective.
    X : Set, optional
        State constraint ``x(t) in X``: a hard constraint, leaving it is a
        failure. Unconstrained by default (the whole state space); pass
        ``X=sys.state.box`` to constrain the plant to its declared state range,
        or any set (``sys.state.box & free``).
    U : InputSet or BoxSet, optional
        Input constraint ``u(t) in U(x, t)``: a hard constraint, like ``X``.
        Unconstrained by default; pass ``U=sys.input_box`` to constrain the input
        to its ports' declared range. A planner that needs a finite box (value
        iteration's grid, an RL action space, RRT's bang-bang controls) refuses
        an unbounded one.
    X0 : Set or array_like, optional
        The start constraint ``x(0) ∈ X0``, what planners enforce; a vector is
        the singleton at it. By default from ``x0``: a point's singleton, a
        law's support; with neither, the singleton at ``sys.x0``.
    Xf : Set or array_like, optional
        The terminal constraint ``x(tf) ∈ Xf``; a vector is the singleton at it
        (a hard terminal equality for collocation and shooting); ``None``
        (default) is a free end.

    horizon : float, optional
        Continuous planning horizon length in seconds, measured from
        ``t0``. ``None`` means unset (RRT / some DP tasks). ``+inf`` means
        infinite-horizon. Finite trajopt / MPC grids require a finite
        ``horizon`` — demos should always set it explicitly. Knot count ``N``
        lives on transcription options.
    t0 : float, optional
        The start time of the horizon, ``[t0, t0 + horizon]``: a run
        starts at ``t0``, its costs, sets and references read absolute
        time, and the discount counts from it. ``0`` (default) is today's
        horizon; it is a leaf of :attr:`params`, so an MPC binds the time of its
        tick and ``jax.grad`` differentiates it.
    discount_rate : float, optional
        The continuous discount rate ``ρ ≥ 0`` of the objective
        ``∫ e^{−ρ(t − t0)} g dt + e^{−ρ T} h`` (``T`` the horizon), applied by every consumer (value
        iteration's ``α`` and reinforcement learning's ``γ`` are
        :meth:`discount_factor` of their step). ``0`` (default) is undiscounted.
    penalize_failure : bool, optional
        Whether an evaluation charges a run that fails (leaves ``X`` or ``U``, or
        stops being finite) or misses ``Xf``: ``False`` (default) reports the
        plain cost and the facts; ``True`` adds the failure cost to its ``J``.
    failure_cost : float, optional
        The finite stand-in for the infinite cost of a failure; ``None``
        (default) is :meth:`cost_bound`. Value iteration and the learners always
        charge a failure this cost. A declared cost should exceed the cost-to-go
        of finishing the task from a typical start.
    distributions : mapping, optional
        Laws over the problem's parameters, in the tree's own shape
        (``{"plant": {"m": Uniform([0.9], [1.1])}}``; a block inside a diagram
        plant by its id, ``{"plant": {"plant": {"m": ...}}}``): each leaf a
        :class:`~minilink.core.distributions.Distribution` over that number,
        drawn per trial by :meth:`realize` (domain randomization, robustness).
    disturbances : mapping, optional
        Source blocks on the plant's input ports, ``{"d": WhiteNoise(...)}``: the loop the evaluator closes is exactly the one ``closed_loop``
        builds with the same sources, the source of ``d`` its block
        ``disturbance`` and of ``n`` its ``noise`` (any other port's under the
        port's name), so its seed is the leaf ``"disturbance.seed"``. A bare law
        is refused: a disturbance states its time behaviour as a block.
    criterion : {"expectation", "worst_case"} or ("cvar", alpha)
        What "optimal" means over the draws of a stochastic problem: the mean,
        the worst, or the mean of the worst ``alpha`` fraction of the runs' ``J``
        (the conditional value at risk). An evaluation reports every statistic
        and its ``objective`` is this one.
    """

    FIELDS = (
        "sys",
        "x0",
        "cost",
        "X",
        "U",
        "X0",
        "Xf",
        "horizon",
        "t0",
        "penalize_failure",
        "failure_cost",
        "discount_rate",
        "distributions",
        "disturbances",
        "criterion",
    )
    ROLES = (
        "plant",
        "cost",
        "X",
        "U",
        "X0",
        "Xf",
        "x0",
        "t0",
        "discount_rate",
    )
    CRITERIA = ("expectation", "worst_case", "cvar")

    def __init__(
        self,
        sys: System,
        x0=None,
        cost: CostFunction | None = None,
        X: Set | None = None,
        U: InputSet | None = None,
        X0: Set | None = None,
        Xf: Set | None = None,
        horizon: float | None = None,
        t0: float = 0.0,
        penalize_failure: bool = False,
        failure_cost: float | None = None,
        discount_rate: float = 0.0,
        distributions: Mapping | None = None,
        disturbances: Mapping[str, System] | None = None,
        criterion="expectation",
    ):
        # The problem's own numbers: x0, t0 and the discount rate
        self.held_params = {}

        # The start: a point, or a law over x(0) (a set is the uniform law on it)
        self.start_law = start_law_of(x0)
        if self.start_law is not None:
            x0 = None
            if X0 is None:
                X0 = self.start_law.support

        # The fields as given, coerced below; a vector boundary is its singleton
        self.sys = sys
        self.held_params["x0"] = x0
        self.cost = cost
        self.X = X
        self.U = U
        self.X0 = boundary_set_of(X0)
        self.Xf = boundary_set_of(Xf)
        self.horizon = horizon
        self.t0 = coerce_start_time(t0)
        self.penalize_failure = penalize_failure
        self.failure_cost = failure_cost
        self.discount_rate = discount_rate
        n = int(self.sys.n)

        # The horizon and the discount rate, and the failure's switch and cost
        horizon = self.coerce_horizon(self.horizon)
        self.discount_rate = coerce_rate(self.discount_rate)
        self.penalize_failure = coerce_switch(self.penalize_failure)
        self.failure_cost = coerce_failure_cost(self.failure_cost)

        # The evaluation start: the point given, else the law's mean, else X0's
        x0 = self.coerce_state(
            default_x0_of(self),
            label="x0",
            required=True,
        )

        # The sets: X and U unconstrained unless given, X0 the start's singleton,
        # Xf as given (none by default)
        X = unconstrained(n) if self.X is None else self.X
        U = BoxInputSet(unconstrained(int(self.sys.m))) if self.U is None else self.U
        if isinstance(U, Set):
            U = BoxInputSet(require_input_box(U))
        X0 = SingletonSet(x0) if self.X0 is None else self.X0
        Xf = self.Xf

        # The sets' types
        self.require_type("X", X, Set)
        self.require_type("U", U, InputSet)
        self.require_type("X0", X0, Set)
        if Xf is not None:
            self.require_type("Xf", Xf, Set)

        # The start: a state of size n inside X0
        self.check_state_shape("x0", x0, n)
        self.require_member(
            X0,
            x0,
            set_label="X0",
            point_label="x0",
        )

        # The coerced fields
        self.horizon = horizon
        self.held_params["x0"] = x0
        self.X = X
        self.U = U
        self.X0 = X0
        self.Xf = Xf

        # The laws over the parameters, the source blocks on the plant's ports, and
        # the criterion over the draws
        self.disturbances = coerce_disturbances(disturbances, self.sys)
        self.distributions = coerce_laws(distributions, self.parts_view())
        require_criterion(criterion, self.CRITERIA)
        self.criterion = criterion

    @property
    def params(self) -> dict:
        """
        The problem's numbers, by role, as a diagram's are by block: one tree of numbers.

        ``{"plant": sys.params, "cost": cost.params, "X": …, "U": …, "X0": …,
        "Xf": …, "x0": x0, "t0": t₀, "discount_rate": ρ}``. Each part's entry is
        the part's own tree, its one home (a set's entry is its own ``params``,
        ``{}`` for a set without parameters; each disturbance source's under its
        port), read again from the parts at each access; ``x0``, ``t0`` and
        ``discount_rate`` live here, so ``problem.params["discount_rate"] = 0.5``
        sets the rate. The laws over the numbers are :attr:`distributions`.
        Assigning a dictionary routes each entry to its part, so
        ``problem.params = {"plant": {...}}`` sets the plant's parameters.
        """
        tree = self.held_params
        parts = part_trees_of(self)
        own = {key: tree.get(key) for key in OWN_LEAVES}

        # The parts' trees, refreshed when a part was replaced, in the roles' order
        stale = list(tree) != [*parts, *OWN_LEAVES] or any(
            tree[role] is not entry for role, entry in parts.items()
        )
        if stale:
            tree.clear()
            tree.update(parts)
            tree.update(own)

        return tree

    @params.setter
    def params(self, value):
        view = self.params
        require_roles(value, view)
        require_numbers(value or {})
        require_complete(value, view)

        for role, entry in value.items():
            if role == "plant":
                self.sys.params = entry
            elif role == "cost":
                self.cost.params = entry
            elif role == "x0":
                self.x0 = entry
            elif role == "t0":
                self.t0 = coerce_start_time(entry)
            elif role == "discount_rate":
                self.discount_rate = coerce_rate(entry)
            elif role in sources_of(self):
                sources_of(self)[role].params = entry
            else:
                # A set's numbers in place, its one home
                getattr(self, role).params = entry

    @property
    def t0(self) -> float:
        """The start time of the horizon, the leaf ``params["t0"]``."""
        return self.held_params["t0"]

    @t0.setter
    def t0(self, value):
        self.held_params["t0"] = coerce_start_time(value)

    @property
    def discount_rate(self) -> float:
        """The continuous discount rate ``ρ``, the leaf ``params["discount_rate"]``."""
        return self.held_params["discount_rate"]

    @discount_rate.setter
    def discount_rate(self, value):
        self.held_params["discount_rate"] = coerce_rate(value)

    @property
    def x0(self) -> np.ndarray:
        """The evaluation start's point (a law's mean): the one leaf ``params["x0"]``; assigning it moves a singleton ``X0`` with it, any other ``X0`` must hold it."""
        return self.held_params.get("x0")

    @x0.setter
    def x0(self, value):
        start = self.coerce_state(value, label="x0", required=True)
        self.check_state_shape("x0", start, int(self.sys.n))
        if isinstance(self.X0, SingletonSet):
            self.X0 = SingletonSet(start)
        self.require_member(self.X0, start, set_label="X0", point_label="x0")

        self.held_params["x0"] = start

    def goal_point(self) -> np.ndarray:
        """
        The point a heuristic aims at: ``Xf``'s point, else the cost's target ``x_bar`` inside ``X``, else the start.

        ``Xf``'s point is a singleton's point, a ball's centre, else the centre of
        its bounding box when finite. A target outside the state constraint
        ``X`` is no place to aim a guess at: the start is. An initial guess's
        end, RRT's goal bias and a plot's anchor read it; it never constrains a
        plan.
        """
        point = set_point(self.Xf)
        if point is not None:
            return point

        x_bar = None if self.cost is None else self.cost.params.get("x_bar")
        if x_bar is not None:
            x_bar = np.asarray(x_bar, dtype=float).reshape(-1).copy()
            if self.X is None or bool(self.X.contains(x_bar, self.t0)):
                return x_bar

        return self.x0.copy()

    def realize(self, key=None, n=1):
        """
        ``n`` realizations of the problem's draws from one key: one tree.

        The problem's numbers (:meth:`parts_view`), with ``"x0"`` of shape
        ``(n, nx)``, each law's leaf drawn ``n`` times in its number's shape
        (``(n, *shape)``), and each random disturbance source's ``seed`` an
        integer leaf of shape ``(n,)``; the other leaves are shared. Each
        consumer draws from its own stream, named by its path
        (``child_seed(key, "x0")``, ``child_seed(key, "plant.m")``; a
        source's seed by its port and the trial): adding a law or a source
        leaves every other draw unchanged. An integer key draws on the host,
        the same values for a NumPy and a JAX run; a JAX key draws
        traceable JAX arrays.
        :meth:`drawn_paths` names the leaves with a trial axis. A deterministic
        problem has one realization, its own start. ``key`` follows the draw
        convention of :class:`~minilink.core.distributions.Distribution`; the
        sources' seeds need an integer key. ``key=None`` draws nothing: every
        law at its mean (tiled ``n`` times) and every source's seed ``None``,
        its mean signal: the tree :meth:`nominal`
        writes.
        """
        n = int(n)
        tree = self.parts_view()

        # A deterministic problem: its one start, the shared numbers
        if not self.is_stochastic:
            require_one_run(n)
            tree["x0"] = self.sample_x0(None, n)
            return tree

        # x(0) ~ the start's law, n times, from the stream named "x0"
        tree["x0"] = self.sample_x0(stream_of(key, "x0"), n)

        # Each law's leaf n times, in its number's shape, from the stream named by
        # its path (the tree copied along the path: the parts' dicts stay untouched)
        tree = with_law_draws(tree, self.distributions, key, n)

        # Each random source's seeds, n times, named by its block id and the trial;
        # without a key, every seed None (the mean signal)
        for sys_id, block in sources_of(self).items():
            if block.is_random:
                if key is None:
                    seeds = seeds_of(block, block.realize(None))
                else:
                    seeds = source_seeds(block, key, sys_id, n)
                tree = with_leaf(tree, (sys_id,), merge_tree(tree[sys_id], seeds))

        return tree

    def drawn_paths(self):
        """The paths of the leaves :meth:`realize` draws with a trial axis."""
        paths = [("x0",)]
        paths.extend(path for path, _ in flatten(self.distributions))
        for sys_id, block in sources_of(self).items():
            if block.is_random:
                seeds = seeds_of(block, block.realize(0))
                paths.extend((sys_id, *path) for path, _ in flatten(seeds))

        return paths

    def sample_x0(self, key, n=None):
        """Draw starts from the start's law: ``(nx,)`` for one, ``(n, nx)`` with ``n``."""
        if self.start_law is None:
            x0 = self.x0
            return x0.copy() if n is None else np.tile(x0, (int(n), 1))

        return self.start_law.sample(key, n)

    def sample_params(self, key):
        """One draw of each law, at its path in the problem's tree (empty when nothing is drawn)."""
        return law_draws(self.distributions, self.parts_view(), key)

    def nominal(self) -> PlanningProblem:
        """
        The certainty-equivalent problem: the problem at ``realize(None)``, every law at its mean, every seed ``None``.

        A copy, its parts copied (the caller's stay as they are): the start at
        its law's mean (a singleton ``X0``), each parameter with a law at the
        law's mean in the copied part, each random source kept with its seed
        ``None``, its mean signal; no law remains.
        """
        tree = self.realize(None)

        # The copy without its laws, the start at its mean
        nominal = deepcopy(self).replace(x0=self.x0, X0=None, distributions=None)

        # Each law's mean in the copied part
        means = {}
        for path, _ in flatten(self.distributions):
            set_leaf(means, path, lookup(tree, path)[0])
        if means:
            nominal.params = merge_tree(nominal.parts_view(), means)

        # Each random source at its mean signal: its seed None
        for block in sources_of(nominal).values():
            if block.is_random:
                block.params = block.realize(None)

        return nominal

    def replace(self, **changes):
        """A new problem of the same class with the named fields replaced (the others shared)."""
        fields = {name: getattr(self, name) for name in self.FIELDS}
        fields["distributions"] = self.distributions or None
        if self.start_law is not None:
            fields["x0"] = self.start_law
        unknown = sorted(set(changes) - set(fields))
        if unknown:
            raise TypeError(f"{type(self).__name__} has no field(s) {unknown}")
        fields.update(changes)

        # A new start takes its own singleton X0, unless one is given
        if "x0" in changes and "X0" not in changes:
            if isinstance(self.X0, SingletonSet):
                fields["X0"] = None

        # A new start law takes its own support, unless one is given
        if isinstance(changes.get("x0"), (Distribution, Set)):
            if "X0" not in changes:
                fields["X0"] = None

        return type(self)(**fields)

    @property
    def has_goal(self) -> bool:
        """Return ``True`` when a terminal goal or boundary set is available."""
        return self.Xf is not None

    @property
    def has_cost(self) -> bool:
        """Return ``True`` when the problem has a cost function."""
        return self.cost is not None

    @property
    def is_stochastic(self) -> bool:
        """``True`` when a part is random: a law on the start or a parameter, a random source."""
        random_source = any(block.is_random for block in self.disturbances.values())

        return self.start_law is not None or bool(self.distributions) or random_source

    def parts_view(self) -> dict:
        """:attr:`params` copied at its top level: the parts' own trees by role and the problem's own numbers, for a caller that sets entries."""
        return dict(self.params)

    def discount_factor(self, dt) -> float:
        """The per-step factor ``γ = e^{−ρ dt}`` of a planner stepping ``dt``."""
        rho = self.discount_rate

        # γ = exp(−ρ dt)
        gamma = np.exp(-rho * float(dt))

        return float(gamma)

    def cost_bound(self, horizon=None, *, discount_rate=None, dt=None, params=None):
        """
        An upper bound of the cost of any run that stays in the boxes: always finite.

        ``g_max ∫₀ᵀ e^{−ρt} dt + e^{−ρT} h_max``, with ``g_max`` and ``h_max`` the
        cost's bounds over the boxes of ``X`` and ``U`` (the declared sets', else
        the plant's declared state and input ranges), ``T`` the run's length
        (``horizon``, default the problem's) and ``ρ`` the problem's rate (or
        ``discount_rate``, a planner's own). A stepped tool passes its period
        ``dt``: the integral is then its sum ``Σ_{k<N} γᵏ dt``, ``γ = e^{−ρ dt}``,
        ``N = T / dt``, which is larger. An infinite horizon charges no ``h``; one
        with ``ρ = 0``, or boxes with no finite bound, falls back to ``1e6`` with
        a warning naming ``failure_cost``. ``params`` are the cost's numbers to
        bound (an evaluation's overridden weights); ``None`` reads its own.
        """
        cost = require_cost(self)
        rho = self.discount_rate if discount_rate is None else float(discount_rate)
        T = (
            (self.horizon if self.horizon_kind() == "finite" else np.inf)
            if horizon is None
            else float(horizon)
        )

        # The boxes: the declared sets', else the plant's declared ranges
        X_box = finite_box_or(self.X.bounding_box(), self.sys.state.box)
        U_box = finite_box_or(self.U.bounding_box(), self.sys.input_box.box)
        if X_box is None or U_box is None:
            return fallback_bound("the state or input box has no finite bound")

        g_max = cost.g_max(X_box, U_box, params)
        h_max = cost.h_max(X_box, params) if np.isfinite(T) else 0.0

        if not (rho > 0.0 or np.isfinite(T)):
            return fallback_bound("an infinite undiscounted horizon has no cost bound")

        # ∫₀ᵀ e^{−ρt} dt = (1 − e^{−ρT}) / ρ, or T when ρ = 0
        if dt is None:
            discounted_time = (1.0 - np.exp(-rho * T)) / rho if rho > 0.0 else T

        # Σ_{k<N} γᵏ dt = dt (1 − γᴺ) / (1 − γ), or N dt when γ = 1
        else:
            dt = float(dt)
            gamma = np.exp(-rho * dt)
            N = max(1, round(T / dt)) if np.isfinite(T) else np.inf
            discounted_time = (
                dt * (1.0 - gamma**N) / (1.0 - gamma) if gamma < 1.0 else N * dt
            )

        # J_max = g_max ∫₀ᵀ e^{−ρt} dt + e^{−ρT} h_max
        bound = g_max * discounted_time + (
            np.exp(-rho * T) * h_max if np.isfinite(T) else 0.0
        )
        if not np.isfinite(bound):
            return fallback_bound("the cost has no finite bound on the boxes")

        return float(bound)

    def failure_cost_at(
        self, horizon=None, *, discount_rate=None, dt=None, params=None
    ):
        """The cost a failure is charged: :attr:`failure_cost` when a number, else :meth:`cost_bound` (its arguments)."""
        failure_cost = self.failure_cost
        if failure_cost is None:
            return self.cost_bound(
                horizon, discount_rate=discount_rate, dt=dt, params=params
            )

        return failure_cost

    def compile(
        self,
        controller=None,
        dt=None,
        *,
        solver="rk4",
        substeps=None,
        backend="auto",
        episode_length=None,
    ):
        """
        The :class:`~minilink.planning.evaluator.ProblemEvaluator` of a controller on this problem.

        The controller closed on the plant (or a built loop with a ``plant`` block),
        compiled once: ``pe.evaluate()`` scores it, ``pe.params`` holds every
        parameter of the run, ``pe.program(wrt)`` exports the tuning program.
        A sampled controller (``pid % 0.01``) runs the sampled loop on its own
        clock: ``dt`` omitted reads its base period. On a sampled loop
        ``substeps`` is the plant's steps per base period, ``None`` the
        automatic step (``problem.compile(mpc % 0.2, substeps=20)``). One verb,
        one return type: the problem's step face is its discretization's,
        ``problem.discretize(dt).compile()``.
        """
        require_controller_to_compile(controller)

        # Lazy import: the engine loads with its first evaluator
        from minilink.planning.evaluator import ProblemEvaluator

        return ProblemEvaluator(
            self,
            controller,
            dt,
            solver=solver,
            substeps=substeps,
            backend=backend,
            episode_length=episode_length,
        )

    def discretize(self, dt, *, solver="rk4", substeps=1):
        """
        The problem on its plant sampled at ``dt``: a :class:`~minilink.planning.step_problems.StepPlanningProblem`.

        Its cost is the :class:`~minilink.core.costs.SampledCost` of this one,
        ``N = horizon / dt``, ``γ = e^{−ρ dt}``; ``compile()`` gives its step
        face, ``transition(x, u, k, params) -> (x_next, c_k)``.
        """
        # Lazy import: the step problem imports this module
        from minilink.planning.step_problems import discretize_problem

        return discretize_problem(self, dt, solver=solver, substeps=substeps)

    def horizon_kind(self) -> str:
        """``"finite"`` when ``horizon`` is a finite length (``h`` is charged there), ``"infinite"`` for ``+inf`` or unset."""
        return (
            "infinite"
            if self.horizon is None or not np.isfinite(self.horizon)
            else "finite"
        )

    # Display

    def __str__(self):
        if self.is_stochastic:
            return stochastic_problem_text(self)

        return problem_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    @staticmethod
    def coerce_horizon(horizon: object) -> float | None:
        """The horizon as a float: positive or ``+inf``, ``None`` when unset."""
        if horizon is None:
            return None

        value = float(horizon)
        if np.isnan(value) or value <= 0.0:
            raise ValueError(
                "horizon must be positive, None (unset), or +inf (infinite-horizon)"
            )
        if not (np.isfinite(value) or np.isposinf(value)):
            raise ValueError(
                "horizon must be positive, None (unset), or +inf (infinite-horizon)"
            )

        return value

    @staticmethod
    def coerce_state(
        x: object,
        *,
        label: str,
        required: bool,
    ) -> np.ndarray | None:
        """A flat float copy of the point, ``None`` when it is optional and missing."""
        if x is None:
            if required:
                raise ValueError(f"{label} is required")
            return None

        return np.asarray(x, dtype=float).reshape(-1).copy()

    @staticmethod
    def require_type(label: str, value: object, expected_type: type) -> None:
        """Refuse a value that is not an instance of ``expected_type``."""
        if not isinstance(value, expected_type):
            raise TypeError(f"{label} must be a {expected_type.__name__} instance")

    @staticmethod
    def check_state_shape(label: str, x: np.ndarray, n: int) -> None:
        """Validate a representative point as a flat state of size ``n``."""
        if x.shape != (n,):
            raise ValueError(f"{label} must have shape ({n},)")

    @staticmethod
    def require_member(
        set_: Set,
        point: np.ndarray,
        *,
        set_label: str,
        point_label: str,
    ) -> None:
        """Refuse a representative point outside its boundary set."""
        if not set_.contains(point):
            raise ValueError(f"{point_label} must belong to {set_label}")


###############################################################################
# Internal machinery
###############################################################################


def default_x0_of(problem):
    """
    The start given, else its law's mean, else ``X0``'s: a singleton's point, a region's bounding-box centre when finite and inside, else ``sys.x0``.

    With no ``X0`` the start is ``sys.x0``; a region holding neither its
    centre nor ``sys.x0`` is refused.
    """
    if problem.held_params.get("x0") is not None:
        return problem.held_params["x0"]
    if problem.start_law is not None:
        return problem.start_law.mean
    X0 = problem.X0
    if X0 is None:
        return problem.sys.x0
    if isinstance(X0, SingletonSet):
        return X0.point

    # A region: the centre of its bounding box, else the plant's own start
    centre = region_centre(X0)
    if centre is not None and bool(X0.contains(centre)):
        return centre
    x0 = np.asarray(problem.sys.x0, dtype=float).reshape(-1)
    if x0.size == int(problem.sys.n) and bool(X0.contains(x0)):
        return x0

    raise ValueError(
        f"X0 ({type(X0).__name__}) holds neither its bounding box's centre nor "
        "sys.x0: give x0, a point of X0 or a law over it"
    )


def part_trees_of(problem) -> dict:
    """Each part's own tree by role, in the roles' order: the start's singleton left out (its point is ``x0``, one home)."""
    trees = {"plant": problem.sys.params}
    if problem.cost is not None:
        trees["cost"] = problem.cost.params
    for role in ("X", "U", "X0", "Xf"):
        set_ = getattr(problem, role)
        if set_ is None or (role == "X0" and isinstance(set_, SingletonSet)):
            continue
        trees[role] = getattr(set_, "params", None)
    for port, block in getattr(problem, "disturbances", {}).items():
        trees[source_id(port)] = block.params

    return trees


def sources_of(problem) -> dict:
    """The disturbance sources by the id of their block in the evaluated loop (``d`` → ``disturbance``, ``n`` → ``noise``)."""
    return {source_id(port): block for port, block in problem.disturbances.items()}


def require_cost(problem) -> CostFunction:
    """Refuse a problem, continuous or step, without a cost; return the cost."""
    if problem.cost is None:
        raise ValueError("This planner requires problem.cost")

    return problem.cost


def require_finite_horizon(problem) -> float:
    """Refuse an unset or infinite horizon; return ``horizon`` as a float."""
    if problem.horizon is None:
        raise ValueError("This planner requires a finite problem.horizon")
    if not np.isfinite(problem.horizon):
        raise ValueError(
            "This planner requires a finite problem.horizon "
            "(got +inf for infinite-horizon)"
        )

    return float(problem.horizon)


def require_controller_to_compile(controller):
    """Refuse a compile with no controller: a problem compiles a controller's evaluator, its step face is its discretization's."""
    if controller is None:
        raise ValueError(
            "problem.compile(controller) builds the controller's evaluator; the "
            "problem's step face is problem.discretize(dt).compile()"
        )


class SetStart(Distribution):
    """The uniform law on a start set: a :class:`~minilink.core.sets.Set` given as ``x0``."""

    def __init__(self, set_):
        box = set_.bounding_box()
        require_samplable_start(set_, box)
        self.support = set_
        self.mean = 0.5 * (box.lower + box.upper)

    @property
    def dim(self) -> int:
        return int(self.mean.size)

    def sample(self, key, n=None, params=None):
        if key is None:
            return tiled_mean(self.mean, n)
        return self.support.sample(key, n, params)


def start_law_of(x0):
    """The law of a start given as a law or a set, ``None`` for a point."""
    if isinstance(x0, Distribution):
        return x0
    if isinstance(x0, Set):
        return SetStart(x0)

    return None


def require_samplable_start(set_, box):
    """Refuse a start set that cannot draw: no finite bounding box, or no ``sample`` of its own."""
    if type(set_).sample is Set.sample:
        raise ValueError(
            f"x0 {type(set_).__name__} cannot draw a start (only a box samples): "
            "give a law (a Distribution), or a box"
        )
    if not is_finite_box(box):
        raise ValueError(
            f"x0 {type(set_).__name__} has no finite bounding box: give a law "
            "(a Distribution) or a bounded set"
        )


# The blocks closed_loop names its loop inputs' sources by (any other port: its own name)
SOURCE_IDS = {"d": "disturbance", "n": "noise"}


def source_id(port):
    """The id of the source of ``port`` in the evaluated loop: ``closed_loop``'s role name, else the port's."""
    return SOURCE_IDS.get(port, port)


def coerce_disturbances(disturbances, sys) -> dict:
    """The disturbance sources as a dict ``{port: block}``; refuse a bare law, a missing port, the command port."""
    sources = dict(disturbances or {})

    # The command port: u, else the one input there is
    inputs = list(sys.inputs)
    command = "u" if "u" in inputs else (inputs[0] if len(inputs) == 1 else None)

    for port, block in sources.items():
        if isinstance(block, Distribution):
            raise TypeError(
                f"disturbances[{port!r}] is a bare law: a disturbance is a source block "
                "that states its time behaviour; pass WhiteNoise(...) for a "
                "white noise, NoiseSource(law, sample_period=...) for a held draw"
            )
        if not isinstance(block, System) or int(block.m) != 0:
            raise TypeError(
                f"disturbances[{port!r}] must be a source block (a System with no input)"
            )
        if port not in sys.inputs and port not in SOURCE_IDS:
            raise ValueError(
                f"disturbances key {port!r} is not an input port of sys (nor d or n, "
                "which closed_loop adds to the command and the measurement)"
            )
        if port == command:
            raise ValueError(
                f"disturbances key {port!r} is the command port of sys: a source "
                "there would replace the policy's command. Give the disturbance "
                "its own input port (for example 'd')."
            )
        if source_id(port) in PlanningProblem.ROLES:
            raise ValueError(
                f"disturbances key {port!r} is a role of the problem's parameters"
            )

    return sources


def coerce_laws(laws, view) -> dict:
    """The laws over the parameters, in the tree's own shape; refuse a path the tree lacks."""
    laws = {} if laws is None else dict(laws)
    for path, law in flatten(laws):
        name = ".".join(path)
        if not isinstance(law, Distribution):
            raise TypeError(
                f"distributions[{name!r}] must be a Distribution, got {law!r}"
            )
        if path[0] == "x0":
            raise ValueError("a law over the start is x0=law, not distributions=")
        try:
            leaf = lookup(view, path)
        except KeyError:
            raise ValueError(
                f"distributions law {name!r} names no leaf of the problem's parameters"
            ) from None
        if isinstance(leaf, dict):
            raise ValueError(
                f"distributions law over {name!r}: a law applies to a number, not to a whole dict"
            )

    return laws


def require_criterion(criterion, criteria) -> None:
    """Refuse a criterion that is not one of ``criteria`` (``("cvar", alpha)`` with ``0 < alpha ≤ 1``)."""
    if isinstance(criterion, tuple):
        kind = criterion[0] if criterion else None
        ok = kind == "cvar" and len(criterion) == 2 and 0.0 < float(criterion[1]) <= 1
    else:
        ok = criterion in criteria and criterion != "cvar"
    if not ok:
        raise ValueError(
            "criterion must be 'expectation', 'worst_case' or ('cvar', alpha) with "
            f"0 < alpha <= 1, got {criterion!r}"
        )


def stream_of(key, name):
    """The stream of the consumer ``name`` under ``key``: an integer seed, a JAX key folded by the name, or ``None`` for no key (no draw)."""
    if key is None:
        return None
    if is_jax_key(key):
        jax = require_jax()
        return jax.random.fold_in(key, zlib.crc32(name.encode()))

    require_integer_key(key)

    return child_seed(int(key), name)


def with_law_draws(tree, laws, key, n):
    """The tree with each law's leaf drawn ``n`` times in its number's shape, from the stream named by its path (copied along the path)."""
    for path, law in flatten(laws):
        nominal = np.shape(lookup(tree, path))
        values = law.sample(stream_of(key, ".".join(path)), n)
        tree = with_leaf(tree, path, values.reshape((n, *nominal)))

    return tree


def law_draws(laws, view, key):
    """One draw of each law, at its path, in the shape of its number in the tree ``view`` (empty when nothing is drawn)."""
    paths = [path for path, _ in flatten(laws)]
    keys = split_keys(key, len(paths))

    draws = {}
    for path, k in zip(paths, keys):
        # One draw, in the nominal entry's shape, at its path
        value = lookup(laws, path).sample(k)
        set_leaf(draws, path, value.reshape(np.shape(lookup(view, path))))

    return draws


def source_seeds(block, key, sys_id, n):
    """The seeds of a random source over ``n`` trials, nested like the block: trial ``i`` its realization under ``child_seed(key, "trial i")``."""
    require_integer_key(key)
    trial_keys = [child_seed(int(key), f"trial {i}") for i in range(n)]
    seeds = [seeds_of(block, block.realize(child_seed(k, sys_id))) for k in trial_keys]

    return stack_leaves(seeds)


def require_numbers(value):
    """Refuse a law in a tree of numbers: the laws over a problem's numbers are its distributions."""
    for path, leaf in flatten(value):
        if isinstance(leaf, Distribution):
            name = ".".join(path)
            raise TypeError(
                f"params[{name!r}] is a law: params holds numbers; a law over the start "
                "is problem.replace(x0=law), over a number problem.distributions"
            )


def seeds_of(system, realized):
    """The ``"seed"`` entries of the random blocks in a realization of ``system``, nested like it."""
    if not isinstance(system, WiredDiagramMixin):
        return {"seed": realized["seed"]}

    return {
        sys_id: seeds_of(subsystem, realized[sys_id])
        for sys_id, subsystem in system.subsystems.items()
        if subsystem.is_random
    }


def stack_leaves(trees):
    """One tree of the shape of each of ``trees``, each leaf their values stacked on a leading axis."""
    if isinstance(trees[0], dict):
        return {key: stack_leaves([tree[key] for tree in trees]) for key in trees[0]}

    return np.stack([np.asarray(leaf) for leaf in trees])


def require_integer_key(key):
    """Refuse a key that is not an integer (or a JAX key, for the laws alone)."""
    if isinstance(key, bool) or not isinstance(key, (int, np.integer)):
        raise TypeError(
            "the problem's draws are realized from an integer key (a JAX key draws "
            f"the laws, but not a source's seed): got {type(key).__name__}"
        )


def require_one_run(n):
    """Refuse more than one realization of a deterministic problem."""
    if int(n) != 1:
        raise ValueError(
            f"a deterministic problem has one realization, got n={n!r}: give the "
            "problem laws (a law on x0, laws in params, a random source) to "
            "draw several"
        )


def nominal_plant_params(problem):
    """
    The plant's parameters to plan with, the design point: every law of the plant at its mean, a random plant's seeds ``None``.

    ``None`` (the plant's own numbers) when the plant draws nothing and no law
    is declared on it; the same numbers ``problem.nominal()`` plans with.
    """
    plant = problem.sys.realize(None) if problem.sys.is_random else None
    means = problem.sample_params(None).get("plant")
    if not means:
        return plant

    # Each law's mean over the plant's own numbers
    base = problem.sys.params if plant is None else plant
    design = merge_tree(base, means)

    return design


def require_roles(value, view):
    """Refuse a params assignment with a role the problem does not have."""
    unknown = sorted(set(value or {}) - set(view))
    if unknown:
        raise ValueError(
            f"unknown problem role(s) {unknown}; the problem's are {list(view)}"
        )


def boundary_set_of(boundary):
    """A boundary set as given (``None``, a :class:`~minilink.core.sets.Set`), or a vector's singleton."""
    if boundary is None or isinstance(boundary, Set):
        return boundary

    return SingletonSet(np.asarray(boundary, dtype=float).reshape(-1))


def set_point(set_):
    """A set's point: a singleton's, a ball's centre, else its bounding box's centre when finite (``None``: no set or no point)."""
    if set_ is None:
        return None
    if isinstance(set_, SingletonSet):
        return set_.point.copy()
    if isinstance(set_, BallSet):
        return np.asarray(set_.center, dtype=float).copy()

    return region_centre(set_)


def region_centre(set_):
    """The centre of a set's bounding box, ``None`` when it has none or it is not finite."""
    box = set_.bounding_box()
    if box is None or not is_finite_box(box):
        return None

    # c = (lower + upper) / 2
    centre = 0.5 * (
        np.asarray(box.lower, dtype=float) + np.asarray(box.upper, dtype=float)
    )

    return centre


def coerce_switch(penalize_failure) -> bool:
    """The failure switch as a bool; refuse anything else (a cost goes in ``failure_cost``)."""
    if not isinstance(penalize_failure, bool):
        raise TypeError(
            "penalize_failure is True or False; the cost a failure is charged is "
            f"failure_cost (a number, None for the cost bound), got {penalize_failure!r}"
        )

    return penalize_failure


def coerce_failure_cost(failure_cost):
    """The failure cost as ``None`` (the cost bound) or a float; refuse anything else."""
    if failure_cost is None:
        return None
    if (
        isinstance(failure_cost, bool)
        or callable(failure_cost)
        or isinstance(failure_cost, str)
    ):
        raise TypeError(
            "failure_cost must be None (the cost bound) or a number, got "
            f"{failure_cost!r}"
        )

    return float(failure_cost)


def finite_box_or(box, fallback):
    """``box`` when every face is finite, else ``fallback`` when it is, else ``None``."""
    for candidate in (box, fallback):
        if candidate is not None and is_finite_box(candidate):
            return candidate

    return None


def fallback_bound(reason):
    """``1e6``, announced with the reason and the line that sets the failure's cost."""
    warnings.warn(
        f"{reason}: a failure costs 1e6; set failure_cost on the problem to choose it",
        stacklevel=4,
    )

    return 1.0e6


def coerce_start_time(t0) -> float:
    """The start time as a float; refuse one that is not finite."""
    t0 = float(t0)
    if not np.isfinite(t0):
        raise ValueError(f"t0 must be finite, got {t0!r}")

    return t0


def coerce_rate(rate) -> float:
    """The discount rate as a float; refuse a negative one."""
    rate = float(rate)
    if not rate >= 0.0:
        raise ValueError(f"discount_rate must be >= 0, got {rate!r}")

    return rate


def unconstrained(n):
    """The whole state space as a set: no constraint on ``x``."""
    return BoxSet(np.full(int(n), -np.inf), np.full(int(n), np.inf))


def require_input_box(U):
    """Refuse a state set as ``U`` unless it is a box: the input box is state-independent."""
    if U.bounding_box() is not U:
        raise TypeError(
            "U given as a Set must be a box over the inputs, BoxSet(lower, upper); "
            "a state-dependent input set is an InputSet"
        )

    return U


def problem_text(problem):
    """The print text of a deterministic problem: its headline, sets and cost."""
    headline, body = problem_parts(problem)
    return format_inspect(headline, body)


def stochastic_problem_text(problem):
    """The print text of a stochastic problem: the deterministic one with its criterion and laws."""
    headline, body = problem_parts(problem)
    criterion = problem.criterion
    if isinstance(criterion, tuple):
        criterion_s = f"cvar, alpha={criterion[1]:g}"
    else:
        criterion_s = str(criterion)
    headline = f"{headline}, criterion={criterion_s}"

    if problem.start_law is not None:
        body.append(body_line("x0", inspect_inline(problem.start_law)))
    if problem.distributions:
        paths = [".".join(path) for path, _ in flatten(problem.distributions)]
        body.append(
            body_line("params", f"{len(paths)} laws ({', '.join(sorted(paths))})")
        )
    if problem.disturbances:
        body.append(body_line("disturbances", ", ".join(sorted(problem.disturbances))))

    return format_inspect(headline, body)


def problem_parts(problem):
    """``(headline, body)`` shared by both problem texts."""
    sys = problem.sys
    name = getattr(sys, "name", type(sys).__name__)
    headline = (
        f"{type(problem).__name__}, {name}, "
        f"horizon={planning_horizon_text(problem.horizon)}, rho={problem.discount_rate:g}"
    )
    body = []
    for label, item in (
        ("X", problem.X),
        ("X0", problem.X0),
        ("Xf", problem.Xf),
        ("U", problem.U),
        ("cost", problem.cost),
    ):
        if item is not None:
            body.append(body_line(label, inspect_inline(item)))

    return headline, body


def planning_horizon_text(tf):
    """The horizon as printed: ``None``, ``inf`` or its number."""
    if tf is None:
        return "None"
    if not np.isfinite(tf):
        return "inf"

    return f"{tf:g}"
