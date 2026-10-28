"""Planning problems: the task, deterministic or stochastic by its parts (a law where a number goes)."""

from __future__ import annotations

import warnings
import zlib
from collections.abc import Mapping

import numpy as np

from minilink.core.backends import is_jax_key, require_jax
from minilink.core.costs import CostFunction
from minilink.core.distributions import Distribution, child_seed, split_keys
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.params import flatten, lookup, merge_tree, set_leaf, with_leaf
from minilink.core.sets import (
    BoxInputSet,
    BoxSet,
    InputSet,
    Set,
    SingletonSet,
    is_finite_box,
)
from minilink.core.system import System
from minilink.core.wiring import WiredDiagramMixin

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
    sets, the continuous horizon ``tf``, the optional cost. The knot count and
    the other discretization choices belong to the solvers, not to the problem.
    The parameters live on the parts; :attr:`params` is their live view by role,
    as a diagram's are by block.

    A problem is stochastic by its parts (R143): where a number goes, a law may
    go (``x_start=`` a law or a set; ``params=`` laws in the tree's own shape),
    and a disturbance is a source block on a plant port (R128). :meth:`realize`
    draws them into one tree (R144), :meth:`nominal` is the problem without its
    laws (the start at its law's mean, the parts at their own numbers), and
    :attr:`is_stochastic` is read off the parts.

    Parameters
    ----------
    sys : System
        Minilink system or diagram to plan for.
    x_start : array_like, Distribution or Set, optional
        The plant's start (R124): a point, a law over ``x(0)``, or a set (the
        uniform law on it). A point is the representative start: when ``X0`` is
        omitted, it creates a singleton ``X0``; when ``X0`` is provided, it must
        belong to ``X0``. A law's mean is the representative start and its
        support the default ``X0``; a set is the default ``X0``. Defaults to the
        singleton ``X0`` when available, otherwise ``sys.x0``.
    x_goal : array_like, optional
        Representative terminal target, for the tools that need one point (an
        initial guess, RRT's goal bias). It never creates ``Xf``: an exact
        terminal state is ``Xf=SingletonSet(x_goal)``, written. When ``Xf`` is
        provided, ``x_goal`` must belong to it; unset, it is the point of a
        singleton ``Xf``.
    cost : CostFunction, optional
        Planning cost. Required by solvers that optimize an objective.
    X : Set, optional
        State constraint ``x(t) in X``: a hard constraint, leaving it is a
        failure. Unconstrained by default (the whole state space); pass
        ``X=sys.state.box`` to constrain the plant to its declared state range,
        or any set (``sys.state.box & free``).
    U : InputSet, optional
        Input constraint ``u(t) in U(x, t)``: a hard constraint, like ``X``.
        Unconstrained by default; pass ``U=sys.input_box`` to constrain the input
        to its ports' declared range. A planner that needs a finite box (value
        iteration's grid, an RL action space, RRT's bang-bang controls) refuses
        an unbounded one.
    X0, Xf : Set, optional
        Initial and terminal boundary sets. These are the authoritative
        feasibility constraints; ``x_start`` and ``x_goal`` are shortcuts or
        representative points.
    tf : float, optional
        Continuous planning horizon length in seconds. ``None`` means unset
        (RRT / some DP tasks). ``+inf`` means infinite-horizon. Finite trajopt /
        MPC grids require a finite ``tf`` — demos should always set it
        explicitly. Knot count ``N`` lives on transcription options.
    discount_rate : float, optional
        The continuous discount rate ``ρ ≥ 0`` of the objective
        ``∫ e^{−ρt} g dt + e^{−ρ tf} h``, applied by every consumer (value
        iteration's ``α`` and reinforcement learning's ``γ`` are
        :meth:`discount_factor` of their step). ``0`` (default) is undiscounted.
    infeasible_cost : None, True or float, optional
        The price switch: the finite stand-in for the infinite cost of a run that
        fails (leaves ``X`` or ``U``, or stops being finite) or misses ``Xf``.
        ``None`` (default): an evaluation reports the plain cost and the facts;
        ``True``: the price is :meth:`cost_bound`; a number: that price. Value
        iteration and the learners always price a failure: this number when it
        is one, else :meth:`cost_bound`. A declared price should exceed the
        cost-to-go of finishing the task from a typical start.
    params : mapping, optional
        Laws over the problem's parameters, in the tree's own shape
        (``{"plant": {"m": Uniform([0.9], [1.1])}}``; a block inside a diagram
        plant by its id, ``{"plant": {"plant": {"m": ...}}}``): each leaf a
        :class:`~minilink.core.distributions.Distribution` over that number,
        drawn per trial by :meth:`realize` (domain randomization, robustness).
    disturbances : mapping, optional
        Source blocks on the plant's input ports, ``{"d": WhiteNoise(...)}``
        (R128): the loop the evaluator closes is exactly the one ``closed_loop``
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
        "x_start",
        "x_goal",
        "cost",
        "X",
        "U",
        "X0",
        "Xf",
        "tf",
        "infeasible_cost",
        "discount_rate",
        "params",
        "disturbances",
        "criterion",
    )
    ROLES = ("plant", "cost", "X", "U", "X0", "Xf", "x_start", "discount_rate")
    CRITERIA = ("expectation", "worst_case", "cvar")

    def __init__(
        self,
        sys: System,
        x_start=None,
        x_goal=None,
        cost: CostFunction | None = None,
        X: Set | None = None,
        U: InputSet | None = None,
        X0: Set | None = None,
        Xf: Set | None = None,
        tf: float | None = None,
        infeasible_cost: bool | float | None = None,
        discount_rate: float = 0.0,
        params: Mapping | None = None,
        disturbances: Mapping[str, System] | None = None,
        criterion="expectation",
    ):
        # The start: a point, or a law over x(0) (a set is the uniform law on it)
        self.start_law = start_law_of(x_start)
        if self.start_law is not None:
            x_start = None
            if X0 is None:
                X0 = self.start_law.support

        # The fields as given, coerced below
        self.sys = sys
        self.x_start = x_start
        self.x_goal = x_goal
        self.cost = cost
        self.X = X
        self.U = U
        self.X0 = X0
        self.Xf = Xf
        self.tf = tf
        self.infeasible_cost = infeasible_cost
        self.discount_rate = discount_rate
        n = int(self.sys.n)

        # The horizon and the discount rate, and a scalar price of infeasibility as a float
        tf = self.coerce_tf(self.tf)
        self.discount_rate = coerce_rate(self.discount_rate)
        self.infeasible_cost = coerce_price(self.infeasible_cost)

        # The representative start and goal
        x_start = self.coerce_state(
            self.default_x_start(),
            label="x_start",
            required=True,
        )
        x_goal = self.coerce_state(
            self.default_x_goal(),
            label="x_goal",
            required=False,
        )

        # The sets: X and U unconstrained unless given, X0 the start's singleton,
        # Xf as given (none by default)
        X = unconstrained(n) if self.X is None else self.X
        U = unconstrained_inputs(int(self.sys.m)) if self.U is None else self.U
        X0 = SingletonSet(x_start) if self.X0 is None else self.X0
        Xf = self.Xf

        # The sets' types
        self.require_type("X", X, Set)
        self.require_type("U", U, InputSet)
        self.require_type("X0", X0, Set)
        if Xf is not None:
            self.require_type("Xf", Xf, Set)

        # The representative points: states of size n, inside their boundary sets
        self.check_state_shape("x_start", x_start, n)
        if x_goal is not None:
            self.check_state_shape("x_goal", x_goal, n)
        self.require_member(
            X0,
            x_start,
            set_label="X0",
            point_label="x_start",
        )
        if Xf is not None and x_goal is not None:
            self.require_member(
                Xf,
                x_goal,
                set_label="Xf",
                point_label="x_goal",
            )

        # The coerced fields
        self.tf = tf
        self.x_start = x_start
        self.x_goal = x_goal
        self.X = X
        self.U = U
        self.X0 = X0
        self.Xf = Xf

        # The laws over the parameters, the source blocks on the plant's ports, and
        # the criterion over the draws
        self.disturbances = coerce_disturbances(disturbances, self.sys)
        self.laws = coerce_laws(params, self.parts_view())
        require_criterion(criterion, self.CRITERIA)
        self.criterion = criterion

    @property
    def params(self) -> dict:
        """
        The live view of the problem's parameters, by role, as a diagram's are by block.

        ``{"plant": sys.params, "cost": cost.params, "X": …, "U": …, "X0": …,
        "Xf": …, "x_start": x_start, "discount_rate": ρ}``, assembled on each
        access from the parts, which stay the one home of each number (a set's
        entry is its own ``params``, ``{}`` for a set without parameters); each
        disturbance source's ``params`` under its port; a law in place of each
        number the problem draws (the start's law under ``"x_start"``).
        Assigning a dictionary routes each entry to its part, so
        ``problem.params = {"plant": {...}}`` sets the plant's parameters.
        """
        view = self.parts_view()
        if self.start_law is not None:
            view["x_start"] = self.start_law
        if self.laws:
            view = merge_tree(view, self.laws)

        return view

    @params.setter
    def params(self, value):
        require_roles(value, self.params)

        # A law in the tree moves the problem's laws; the numbers move the parts
        value, laws = split_laws(value, self.parts_view())
        if "x_start" in laws:
            self.start_law = laws.pop("x_start")
            value.pop("x_start", None)
        self.laws = merge_tree(self.laws, laws) if laws else self.laws

        for role, entry in value.items():
            if role == "plant":
                self.sys.params = entry
            elif role == "cost":
                self.cost.params = entry
            elif role == "x_start":
                self.move_start(entry)
            elif role == "discount_rate":
                self.discount_rate = coerce_rate(entry)
            elif role in self.sources_by_id():
                self.sources_by_id()[role].params = entry
            else:
                setattr(self, role, moved_set(role, getattr(self, role), entry))

    def move_start(self, x_start):
        """Set ``x_start``: a singleton ``X0`` moves with it, any other ``X0`` must hold it."""
        start = self.coerce_state(x_start, label="x_start", required=True)
        self.check_state_shape("x_start", start, int(self.sys.n))
        if isinstance(self.X0, SingletonSet):
            self.X0 = SingletonSet(start)
        self.require_member(self.X0, start, set_label="X0", point_label="x_start")

        self.x_start = start

    def realize(self, key=None, n=1):
        """
        ``n`` realizations of the problem's draws from one key: one tree (R144).

        The problem's numbers (:meth:`parts_view`), with ``"x_start"`` of shape
        ``(n, nx)``, each law's leaf drawn ``n`` times in its number's shape
        (``(n, *shape)``), and each random disturbance source's ``seed`` an
        integer leaf of shape ``(n,)``; the other leaves are shared. Each
        consumer draws from its own stream, named by its path
        (``child_seed(key, "x_start")``, ``child_seed(key, "plant.m")``; a
        source's seed by its port and the trial): adding a law or a source
        leaves every other draw unchanged. An integer key draws on the host,
        the same values for a NumPy and a JAX run (RD-D13); a JAX key draws
        traceable JAX arrays.
        :meth:`drawn_paths` names the leaves with a trial axis. A deterministic
        problem has one realization, its own start. ``key`` follows the draw
        convention of :class:`~minilink.core.distributions.Distribution`; the
        sources' seeds need an integer key.
        """
        n = int(n)
        tree = self.parts_view()

        # A deterministic problem: its one start, the shared numbers
        if not self.is_stochastic:
            require_one_run(n)
            tree["x_start"] = self.sample_x0(None, n)
            return tree

        # x(0) ~ the start's law, n times, from the stream named "x_start"
        tree["x_start"] = self.sample_x0(stream_of(key, "x_start"), n)

        # Each law's leaf n times, in its number's shape, from the stream named by
        # its path (the tree copied along the path: the parts' dicts stay untouched)
        for path, law in flatten(self.laws):
            nominal = np.shape(lookup(tree, path))
            values = law.sample(stream_of(key, ".".join(path)), n)
            tree = with_leaf(tree, path, values.reshape((n, *nominal)))

        # Each random source's seeds, n times, named by its block id and the trial
        for sys_id, block in self.sources_by_id().items():
            if block.is_random:
                seeds = source_seeds(block, key, sys_id, n)
                tree = with_leaf(tree, (sys_id,), merge_tree(tree[sys_id], seeds))

        return tree

    def drawn_paths(self):
        """The paths of the leaves :meth:`realize` draws with a trial axis."""
        paths = [("x_start",)]
        paths.extend(path for path, _ in flatten(self.laws))
        for sys_id, block in self.sources_by_id().items():
            if block.is_random:
                seeds = seeds_of(block, block.realize(0))
                paths.extend((sys_id, *path) for path, _ in flatten(seeds))

        return paths

    def sample_x0(self, key, n=None):
        """Draw starts from the start's law: ``(nx,)`` for one, ``(n, nx)`` with ``n``."""
        if self.start_law is None:
            x_start = self.x_start
            return x_start.copy() if n is None else np.tile(x_start, (int(n), 1))

        return self.start_law.sample(key, n)

    def sample_params(self, key):
        """One draw of each law, at its path in the problem's tree (empty when nothing is drawn)."""
        paths = [path for path, _ in flatten(self.laws)]
        keys = split_keys(key, len(paths))
        view = self.parts_view()

        draws = {}
        for path, k in zip(paths, keys):
            # One draw, in the nominal entry's shape, at its path
            value = lookup(self.laws, path).sample(k)
            set_leaf(draws, path, value.reshape(np.shape(lookup(view, path))))

        return draws

    def nominal(self) -> PlanningProblem:
        """
        The certainty-equivalent problem: the start at its law's mean, no law, no random source.

        The start's mean is the one start (a singleton ``X0``); a parameter
        with a law keeps its part's own number (the parts are shared, never
        written); a deterministic source (a load ``Step``) stays.
        """
        sources = {
            port: block
            for port, block in self.disturbances.items()
            if not block.is_random
        }

        return self.replace(
            x_start=self.x_start, X0=None, params=None, disturbances=sources
        )

    def replace(self, **changes):
        """A new problem of the same class with the named fields replaced (the others shared)."""
        fields = {name: getattr(self, name) for name in self.FIELDS}
        fields["params"] = self.laws or None
        if self.start_law is not None:
            fields["x_start"] = self.start_law
        unknown = sorted(set(changes) - set(fields))
        if unknown:
            raise TypeError(f"{type(self).__name__} has no field(s) {unknown}")
        fields.update(changes)

        # A new start takes its own singleton X0, unless one is given
        if "x_start" in changes and "X0" not in changes:
            if isinstance(self.X0, SingletonSet):
                fields["X0"] = None

        # A new start law takes its own support, unless one is given
        if isinstance(changes.get("x_start"), (Distribution, Set)):
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

        return self.start_law is not None or bool(self.laws) or random_source

    def sources_by_id(self) -> dict:
        """The disturbance sources by the id of their block in the evaluated loop (``d`` → ``disturbance``, ``n`` → ``noise``)."""
        return {source_id(port): block for port, block in self.disturbances.items()}

    def parts_view(self) -> dict:
        """The parts' numbers by role, live: :attr:`params` without the laws (the start its point)."""
        view = {"plant": self.sys.params}
        if self.cost is not None:
            view["cost"] = self.cost.params
        for role in ("X", "U", "X0", "Xf"):
            set_ = getattr(self, role)
            if set_ is not None:
                view[role] = getattr(set_, "params", None)
        for port, block in getattr(self, "disturbances", {}).items():
            view[source_id(port)] = block.params
        view["x_start"] = self.x_start
        view["discount_rate"] = self.discount_rate

        return view

    def discount_factor(self, dt) -> float:
        """The per-step factor ``γ = e^{−ρ dt}`` of a planner stepping ``dt``."""
        rho = self.discount_rate

        # γ = exp(−ρ dt)
        gamma = np.exp(-rho * float(dt))

        return float(gamma)

    def cost_bound(self, tf=None, *, discount_rate=None, dt=None):
        """
        An upper bound of the cost of any run that stays in the boxes: always finite.

        ``g_max ∫₀ᵀ e^{−ρt} dt + e^{−ρT} h_max``, with ``g_max`` and ``h_max`` the
        cost's bounds over the boxes of ``X`` and ``U`` (the declared sets', else
        the plant's declared state and input ranges), ``T`` the run's length
        (``tf``, default the problem's horizon) and ``ρ`` the problem's rate (or
        ``discount_rate``, a planner's own). A stepped tool passes its period
        ``dt``: the integral is then its sum ``Σ_{k<N} γᵏ dt``, ``γ = e^{−ρ dt}``,
        ``N = T / dt``, which is larger. An infinite horizon charges no ``h``; one
        with ``ρ = 0``, or boxes with no finite bound, falls back to ``1e6`` with
        a warning naming ``infeasible_cost``.
        """
        cost = self.require_cost()
        rho = self.discount_rate if discount_rate is None else float(discount_rate)
        T = (
            (self.tf if self.horizon_kind() == "finite" else np.inf)
            if tf is None
            else float(tf)
        )

        # The boxes: the declared sets', else the plant's declared ranges
        X_box = finite_box_or(self.X.bounding_box(), self.sys.state.box)
        U_box = finite_box_or(self.U.bounding_box(), self.sys.input_box.box)
        if X_box is None or U_box is None:
            return fallback_bound("the state or input box has no finite bound")

        g_max = cost.g_max(X_box, U_box)
        h_max = cost.h_max(X_box) if np.isfinite(T) else 0.0

        if not (rho > 0.0 or np.isfinite(T)):
            return fallback_bound("an infinite undiscounted horizon has no cost bound")

        # ∫₀ᵀ e^{−ρt} dt = (1 − e^{−ρT}) / ρ, or T when ρ = 0
        if dt is None:
            horizon = (1.0 - np.exp(-rho * T)) / rho if rho > 0.0 else T

        # Σ_{k<N} γᵏ dt = dt (1 − γᴺ) / (1 − γ), or N dt when γ = 1
        else:
            dt = float(dt)
            gamma = np.exp(-rho * dt)
            N = max(1, round(T / dt)) if np.isfinite(T) else np.inf
            horizon = dt * (1.0 - gamma**N) / (1.0 - gamma) if gamma < 1.0 else N * dt

        # J_max = g_max ∫₀ᵀ e^{−ρt} dt + e^{−ρT} h_max
        bound = g_max * horizon + (np.exp(-rho * T) * h_max if np.isfinite(T) else 0.0)
        if not np.isfinite(bound):
            return fallback_bound("the cost has no finite bound on the boxes")

        return float(bound)

    def infeasibility_price(self, tf=None, *, discount_rate=None, dt=None):
        """The price a planner charges a failure: ``infeasible_cost`` when a number, else :meth:`cost_bound`."""
        price = self.infeasible_cost
        if price is None or price is True:
            return self.cost_bound(tf, discount_rate=discount_rate, dt=dt)

        return price

    def compile(
        self,
        controller,
        dt=None,
        *,
        solver="rk4",
        substeps=1,
        backend="auto",
        episode_length=None,
    ):
        """
        The :class:`~minilink.planning.evaluator.ProblemEvaluator` of a controller on this problem.

        The controller closed on the plant (or a built loop with a ``plant`` block),
        compiled once: ``pe.evaluate()`` scores it, ``pe.params`` holds every
        parameter of the run, ``pe.program(wrt)`` exports the tuning program.
        A sampled controller (``pid % 0.01``) runs the sampled loop on its own
        clock: ``dt`` omitted reads its base period.
        """
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

    def horizon_kind(self) -> str:
        """``"finite"`` when ``tf`` is a finite length (``h`` is charged there), ``"infinite"`` for ``+inf`` or unset."""
        return "infinite" if self.tf is None or not np.isfinite(self.tf) else "finite"

    def require_cost(self) -> CostFunction:
        """Refuse a problem without a cost; return the cost."""
        if self.cost is None:
            raise ValueError("This planner requires problem.cost")

        return self.cost

    def require_goal(self) -> Set:
        """Refuse a problem without a terminal set; return it."""
        if self.Xf is None:
            raise ValueError("This planner requires a terminal set or x_goal")

        return self.Xf

    def require_finite_tf(self) -> float:
        """Refuse an unset or infinite horizon; return ``tf`` as a float."""
        if self.tf is None:
            raise ValueError("This planner requires a finite problem.tf")
        if not np.isfinite(self.tf):
            raise ValueError(
                "This planner requires a finite problem.tf "
                "(got +inf for infinite-horizon)"
            )

        return float(self.tf)

    # Display

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    @staticmethod
    def coerce_tf(tf: object) -> float | None:
        """The horizon as a float: positive or ``+inf``, ``None`` when unset."""
        if tf is None:
            return None

        value = float(tf)
        if np.isnan(value) or value <= 0.0:
            raise ValueError(
                "tf must be positive, None (unset), or +inf (infinite-horizon)"
            )
        if not (np.isfinite(value) or np.isposinf(value)):
            raise ValueError(
                "tf must be positive, None (unset), or +inf (infinite-horizon)"
            )

        return value

    def default_x_start(self):
        """The start given, else its law's mean, else the point of a singleton ``X0``, else ``sys.x0``."""
        if self.x_start is not None:
            return self.x_start
        if self.start_law is not None:
            return self.start_law.mean
        if isinstance(self.X0, SingletonSet):
            return self.X0.point

        return self.sys.x0

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

    def default_x_goal(self):
        """The goal given, else the point of a singleton ``Xf``, else ``None``."""
        if self.x_goal is not None:
            return self.x_goal
        if isinstance(self.Xf, SingletonSet):
            return self.Xf.point

        return None

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


class SetStart(Distribution):
    """The uniform law on a start set: a :class:`~minilink.core.sets.Set` given as ``x_start``."""

    def __init__(self, set_):
        box = set_.bounding_box()
        require_samplable_start(set_, box)
        self.support = set_
        self.mean = 0.5 * (box.lower + box.upper)

    @property
    def dim(self) -> int:
        return int(self.mean.size)

    def sample(self, key, n=None, params=None):
        return self.support.sample(key, n, params)


def start_law_of(x_start):
    """The law of a start given as a law or a set, ``None`` for a point."""
    if isinstance(x_start, Distribution):
        return x_start
    if isinstance(x_start, Set):
        return SetStart(x_start)

    return None


def require_samplable_start(set_, box):
    """Refuse a start set that cannot draw: no finite bounding box, or no ``sample`` of its own."""
    if type(set_).sample is Set.sample:
        raise ValueError(
            f"x_start {type(set_).__name__} cannot draw a start (only a box samples): "
            "give a law (a Distribution), or a box"
        )
    if not is_finite_box(box):
        raise ValueError(
            f"x_start {type(set_).__name__} has no finite bounding box: give a law "
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
                "that states its time behaviour (R128); pass WhiteNoise(...) for a "
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
            raise TypeError(f"params[{name!r}] must be a Distribution, got {law!r}")
        if path[0] == "x_start":
            raise ValueError("a law over the start is x_start=law, not params=")
        try:
            leaf = lookup(view, path)
        except KeyError:
            raise ValueError(
                f"params law {name!r} names no leaf of the problem's parameters"
            ) from None
        if isinstance(leaf, dict):
            raise ValueError(
                f"params law over {name!r}: a law applies to a number, not to a whole dict"
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
    """The stream of the consumer ``name`` under ``key``: an integer seed, or a JAX key folded by the name."""
    if is_jax_key(key):
        jax = require_jax()
        return jax.random.fold_in(key, zlib.crc32(name.encode()))

    require_integer_key(key)

    return child_seed(int(key), name)


def source_seeds(block, key, sys_id, n):
    """The seeds of a random source over ``n`` trials, nested like the block: trial ``i`` its realization under ``child_seed(key, "trial i")``."""
    require_integer_key(key)
    trial_keys = [child_seed(int(key), f"trial {i}") for i in range(n)]
    seeds = [seeds_of(block, block.realize(child_seed(k, sys_id))) for k in trial_keys]

    return stack_leaves(seeds)


def split_laws(value, view):
    """``(numbers, laws)``: the tree with each law replaced by the part's current number, and the laws alone."""
    numbers = dict(value)
    laws = {}
    for path, leaf in flatten(value):
        if isinstance(leaf, Distribution):
            set_leaf(laws, path, leaf)
            if path != ("x_start",):
                numbers = with_leaf(numbers, path, lookup(view, path))

    return numbers, laws


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
            "problem laws (a law on x_start, laws in params, a random source) to "
            "draw several"
        )


def nominal_plant_params(problem):
    """The plant's parameters to plan with: a random plant's draw at its mean, else ``None`` (its own)."""
    if problem.sys.is_random:
        return problem.sys.realize(None)

    return None


def require_roles(value, view):
    """Refuse a params assignment with a role the problem does not have."""
    unknown = sorted(set(value or {}) - set(view))
    if unknown:
        raise ValueError(
            f"unknown problem role(s) {unknown}; the problem's are {list(view)}"
        )


def moved_set(role, set_, entry):
    """The set of ``role`` with its numbers ``entry``; refuse numbers a set does not hold."""
    # The view's own entry for a set without parameters writes back as nothing
    if not entry and not set_.params:
        return set_
    if not set_.params:
        raise TypeError(f"{role} ({type(set_).__name__}) holds no parameters")

    return set_.with_params(entry)


def coerce_price(price):
    """The price switch as ``None``, ``True`` or a float; refuse anything else."""
    if price is None or price is True:
        return price
    if price is False or callable(price) or isinstance(price, str):
        raise TypeError(
            "infeasible_cost must be None (no price), True (the cost bound) or a "
            f"number, got {price!r}"
        )

    return float(price)


def finite_box_or(box, fallback):
    """``box`` when every face is finite, else ``fallback`` when it is, else ``None``."""
    for candidate in (box, fallback):
        if candidate is not None and is_finite_box(candidate):
            return candidate

    return None


def fallback_bound(reason):
    """``1e6``, announced with the reason and the line that sets a price."""
    warnings.warn(
        f"{reason}: a failure is priced 1e6; set infeasible_cost on the problem to "
        "choose the price",
        stacklevel=4,
    )

    return 1.0e6


def coerce_rate(rate) -> float:
    """The discount rate as a float; refuse a negative one."""
    rate = float(rate)
    if not rate >= 0.0:
        raise ValueError(f"discount_rate must be >= 0, got {rate!r}")

    return rate


def unconstrained(n):
    """The whole state space as a set: no constraint on ``x``."""
    return BoxSet(np.full(int(n), -np.inf), np.full(int(n), np.inf))


def unconstrained_inputs(m):
    """The whole input space as an input set: no constraint on ``u``."""
    return BoxInputSet(unconstrained(m))
