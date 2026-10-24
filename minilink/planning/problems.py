"""Planning problems: the deterministic task and its stochastic class."""

from __future__ import annotations

import warnings
from collections.abc import Mapping
from types import MappingProxyType

import numpy as np

from minilink.core.backends import is_jax_key, numpy_generator, require_jax
from minilink.core.costs import CostFunction
from minilink.core.distributions import Distribution, Particles, split_keys
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.sets import (
    BoxInputSet,
    BoxSet,
    InputSet,
    Set,
    SingletonSet,
    is_finite_box,
)
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class PlanningProblem:
    """
    Deterministic planning problem for a Minilink system.

    The generic mathematical form is ``x(t) in X(t)``,
    ``u(t) in U(x(t), t)``, with boundary sets ``X0`` and optional ``Xf``.
    A cost can be attached for optimization and policy-planning solvers,
    but feasibility/search planners may leave it unset.

    The problem is the continuous task: the system, the admissible and boundary
    sets, the continuous horizon ``tf``, the optional cost. The knot count and
    the other discretization choices belong to the solvers, not to the problem.
    The parameters live on the parts; :attr:`params` is their live view by role,
    as a diagram's are by block.

    Parameters
    ----------
    sys : System
        Minilink system or diagram to plan for.
    x_start : array_like, optional
        Representative initial state. When ``X0`` is omitted, this creates a
        singleton ``X0``. When ``X0`` is provided, it must belong to ``X0`` and
        is used by solvers or guesses that need one concrete start point.
        Defaults to singleton ``X0`` when available, otherwise ``sys.x0``.
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
        "metadata",
        "infeasible_cost",
        "discount_rate",
    )
    ROLES = ("plant", "cost", "X", "U", "X0", "Xf", "x_start", "discount_rate")

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
        metadata: Mapping[str, object] | None = None,
        infeasible_cost: bool | float | None = None,
        discount_rate: float = 0.0,
    ):
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
        self.metadata = metadata
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

        # The metadata, read-only
        metadata = self.coerce_metadata(self.metadata)

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
        self.metadata = metadata

    @property
    def params(self) -> dict:
        """
        The live view of the problem's parameters, by role, as a diagram's by block.

        ``{"plant": sys.params, "cost": cost.params, "X": …, "U": …, "X0": …,
        "Xf": …, "x_start": x_start, "discount_rate": ρ}``, assembled on each
        access from the parts, which stay the one home of each number (a set's
        entry is its own ``params``, ``None`` for a set without parameters).
        Assigning a dictionary routes each entry to its part, so
        ``problem.params = {"plant": {...}}`` sets the plant's parameters.
        """
        view = {"plant": self.sys.params}
        if self.cost is not None:
            view["cost"] = self.cost.params
        for role in ("X", "U", "X0", "Xf"):
            set_ = getattr(self, role)
            if set_ is not None:
                view[role] = getattr(set_, "params", None)
        view["x_start"] = self.x_start
        view["discount_rate"] = self.discount_rate

        return view

    @params.setter
    def params(self, value):
        require_roles(value, self.params)
        for role, entry in value.items():
            if role == "plant":
                self.sys.params = entry
            elif role == "cost":
                self.cost.params = entry
            elif role == "x_start":
                self.move_start(entry)
            elif role == "discount_rate":
                self.discount_rate = coerce_rate(entry)
            else:
                write_set_params(role, getattr(self, role), entry)

    def move_start(self, x_start):
        """Set ``x_start``: a singleton ``X0`` moves with it, any other ``X0`` must hold it."""
        start = self.coerce_state(x_start, label="x_start", required=True)
        self.check_state_shape("x_start", start, int(self.sys.n))
        if isinstance(self.X0, SingletonSet):
            self.X0 = SingletonSet(start)
        self.require_member(self.X0, start, set_label="X0", point_label="x_start")

        self.x_start = start

    def realize(self, key=None, n=1):
        """The one realization of a deterministic problem: its start, no draws (``n`` is 1)."""
        require_one_run(n)
        x0s = np.reshape(self.x_start, (1, -1))

        return x0s, {"params": {}, "disturbances": {}}

    def replace(self, **changes):
        """A new problem of the same class with the named fields replaced (the others shared)."""
        fields = {name: getattr(self, name) for name in self.FIELDS}
        unknown = sorted(set(changes) - set(fields))
        if unknown:
            raise TypeError(f"{type(self).__name__} has no field(s) {unknown}")
        fields.update(changes)

        # A new start takes its own singleton X0, unless one is given
        if "x_start" in changes and "X0" not in changes:
            if isinstance(self.X0, SingletonSet):
                fields["X0"] = None

        # A new start law takes its own mean and support, unless they are given
        if "x0_distribution" in changes:
            for name in ("x_start", "X0"):
                if name not in changes:
                    fields[name] = None

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
        """``True`` when the problem draws starts, parameters or disturbances."""
        return False

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
        dt,
        *,
        solver="rk4",
        substeps=1,
        backend="auto",
        episode_length=None,
    ):
        """
        The :class:`~minilink.planning.engine.ProblemEvaluator` of a controller on this problem.

        The controller closed on the plant (or a built loop with a ``plant`` block),
        compiled once: ``pe.evaluate()`` scores it, ``pe.params`` holds every
        parameter of the run, ``pe.program(wrt)`` exports the tuning program.
        """
        # Lazy import: the engine loads with its first evaluator
        from minilink.planning.engine import ProblemEvaluator

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
        """The start given, else the point of a singleton ``X0``, else ``sys.x0``."""
        if self.x_start is not None:
            return self.x_start
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
    def coerce_metadata(metadata) -> Mapping[str, object]:
        """A read-only copy of the metadata, empty when unset."""
        if metadata is None:
            return MappingProxyType({})
        if not isinstance(metadata, Mapping):
            raise TypeError("metadata must be a mapping or None")

        return MappingProxyType(dict(metadata))

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


class StochasticPlanningProblem(PlanningProblem):
    """
    Planning problem with probabilistic uncertainty (the stochastic class).

    Same spine as :class:`PlanningProblem` — the deterministic problem is its
    base class, so every planner that accepts one accepts this — plus the
    uncertainty channels and the criterion:

    Parameters
    ----------
    x0_distribution : Distribution
        Law of the initial state ``x(0) ~ p(x0)``. Its mean is the default
        ``x_start`` and its support (when it has one) the default ``X0``.
    params_distribution : mapping, optional
        Mapping of parameter names to :class:`~minilink.core.distributions.Distribution`
        over entries of ``sys.params`` (domain randomization, robustness
        sweeps); each sample overrides those entries. A dotted name reaches
        into a diagram's subsystem params (``sys.mass`` for the plant inside
        a closed loop).
    disturbances : mapping, optional
        Mapping of input port ids to :class:`~minilink.core.distributions.Distribution`
        on ``sys``: a fresh draw per step held on that port (a seeded
        disturbance signal).
    criterion : {"expectation", "worst_case"} or ("cvar", alpha)
        What "optimal" means over the draws: the mean, the worst, or the mean
        of the worst ``alpha`` fraction of the runs' ``J`` (the conditional value
        at risk). Reinforcement learning optimizes the expectation; an
        evaluation reports every statistic and its ``objective`` is this one.

    Notes
    -----
    Solve verbs (RL, gain search) and the evaluate verb (Monte Carlo) both
    read this description; :meth:`nominal` returns the certainty-equivalent
    :class:`PlanningProblem` for trajectory optimization and LQR.
    """

    FIELDS = (
        *PlanningProblem.FIELDS,
        "x0_distribution",
        "params_distribution",
        "disturbances",
        "criterion",
    )
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
        metadata: Mapping[str, object] | None = None,
        infeasible_cost: bool | float | None = None,
        discount_rate: float = 0.0,
        x0_distribution: Distribution | None = None,
        params_distribution: Mapping[str, Distribution] | None = None,
        disturbances: Mapping[str, Distribution] | None = None,
        criterion="expectation",
    ):
        # The start law: its mean the default start, its support the default X0
        self.require_start_law(x0_distribution)
        if x_start is None:
            x_start = x0_distribution.mean
        if X0 is None and x0_distribution.support is not None:
            X0 = x0_distribution.support

        # The deterministic spine
        super().__init__(
            sys,
            x_start=x_start,
            x_goal=x_goal,
            cost=cost,
            X=X,
            U=U,
            X0=X0,
            Xf=Xf,
            tf=tf,
            metadata=metadata,
            infeasible_cost=infeasible_cost,
            discount_rate=discount_rate,
        )

        # The criterion, and the draws against the system they perturb
        self.require_criterion(criterion, self.CRITERIA)
        self.require_start_dim(x0_distribution, self.sys)
        self.require_param_names(params_distribution, self.sys)
        self.require_disturbance_ports(disturbances, self.sys)

        # The laws, the draws as plain dicts
        self.x0_distribution = x0_distribution
        self.params_distribution = dict(params_distribution or {})
        self.disturbances = dict(disturbances or {})
        self.criterion = criterion

    @property
    def is_stochastic(self) -> bool:
        return True

    def sample_x0(self, key, n=None):
        """Draw initial states: ``(n,)`` for one, ``(n_samples, n)`` with ``n``."""
        return self.x0_distribution.sample(key, n)

    def sample_params(self, key):
        """Draw one ``{name: value}`` override of ``sys.params`` (empty if none), nominal shapes."""
        names = list(self.params_distribution)
        keys = split_keys(key, len(names))

        draws = {}
        for name, k in zip(names, keys):
            # One draw, in the nominal entry's shape
            value = self.params_distribution[name].sample(k)
            value = value.reshape(np.shape(lookup_param(self.sys.params, name)))

            # Placed at its dotted path in the nested override
            node = draws
            *path, last = name.split(".")
            for part in path:
                node = node.setdefault(part, {})
            node[last] = value

        return draws

    def sample_disturbances(self, key):
        """Draw one ``{port_id: value}`` of held disturbance inputs (empty if none)."""
        ports = list(self.disturbances)
        keys = split_keys(key, len(ports))

        return {port: self.disturbances[port].sample(k) for port, k in zip(ports, keys)}

    def realize(self, key, n):
        """
        ``n`` realizations of the problem's draws from one key: ``(x0s, draws)``.

        ``x0s`` is ``(n, nx)``, drawn from the start law; ``draws`` is
        ``{"params": {...}, "disturbances": {...}}``, each drawn parameter of
        ``sys.params`` at its (dotted) path and each disturbance port's held value,
        every leaf with a leading axis of ``n``. ``key`` follows the draw
        convention of :class:`~minilink.core.distributions.Distribution`: a JAX key
        gives JAX arrays (traceable), a NumPy generator or a seed NumPy arrays.
        """
        n = int(n)

        # One key for the starts, one for the laws
        if is_jax_key(key):
            jax = require_jax()
            x_key, law_key = jax.random.split(key)
        else:
            x_key = law_key = numpy_generator(key)

        # x0 ~ p(x0), n times
        x0s = self.sample_x0(x_key, n)

        # Each drawn parameter n times, in its nominal entry's shape, at its path
        names = list(self.params_distribution)
        ports = list(self.disturbances)
        keys = split_keys(law_key, len(names) + len(ports))
        params = {}
        for name, k in zip(names, keys[: len(names)]):
            nominal = np.shape(lookup_param(self.sys.params, name))
            values = self.params_distribution[name].sample(k, n)
            node = params
            *path, last = name.split(".")
            for part in path:
                node = node.setdefault(part, {})
            node[last] = values.reshape((n, *nominal))

        # Each disturbance port's held value, n times
        disturbances = {
            port: self.disturbances[port].sample(k, n)
            for port, k in zip(ports, keys[len(names) :])
        }

        return x0s, {"params": params, "disturbances": disturbances}

    def nominal(self) -> PlanningProblem:
        """
        Certainty-equivalent deterministic problem: the mean start, no draws.

        The start law's mean is the one start (a singleton ``X0``): the support
        ``X0`` of the start law and an explicit ``x_start`` are dropped, and the
        parameter and disturbance draws with them.
        """
        return PlanningProblem(
            sys=self.sys,
            x_start=self.x0_distribution.mean,
            x_goal=self.x_goal,
            cost=self.cost,
            X=self.X,
            U=self.U,
            Xf=self.Xf,
            tf=self.tf,
            metadata=self.metadata,
            infeasible_cost=self.infeasible_cost,
            discount_rate=self.discount_rate,
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    @staticmethod
    def require_start_law(x0_distribution) -> None:
        """Refuse a stochastic problem without the law of its initial state."""
        if x0_distribution is None:
            raise ValueError("StochasticPlanningProblem requires x0_distribution")

    @staticmethod
    def require_criterion(criterion, criteria) -> None:
        """Refuse a criterion that is not one of ``criteria`` (``("cvar", alpha)`` with ``0 < alpha ≤ 1``)."""
        if isinstance(criterion, tuple):
            kind = criterion[0] if criterion else None
            ok = (
                kind == "cvar"
                and len(criterion) == 2
                and 0.0 < float(criterion[1]) <= 1
            )
        else:
            ok = criterion in criteria and criterion != "cvar"
        if not ok:
            raise ValueError(
                "criterion must be 'expectation', 'worst_case' or ('cvar', alpha) with "
                f"0 < alpha <= 1, got {criterion!r}"
            )

    @staticmethod
    def require_start_dim(x0_distribution, sys) -> None:
        """Refuse a start law whose dimension is not the state's."""
        if int(x0_distribution.dim) != int(sys.n):
            raise ValueError("x0_distribution.dim must equal sys.n")

    @staticmethod
    def require_param_names(params_distribution, sys) -> None:
        """Refuse a drawn name missing from ``sys.params``, or one naming a whole subsystem dict."""
        for name in params_distribution or {}:
            try:
                leaf = lookup_param(sys.params, name)
            except KeyError:
                raise ValueError(
                    f"params_distribution key {name!r} is not in sys.params"
                ) from None
            if isinstance(leaf, dict):
                raise ValueError(
                    f"params_distribution over {name!r}: distributions apply to "
                    "array-valued parameters, not to a whole subsystem dict"
                )

    @staticmethod
    def require_disturbance_ports(disturbances, sys) -> None:
        """Refuse a disturbance on a port ``sys`` lacks, or on its command port."""
        # The command port: u, else the one input there is
        inputs = list(sys.inputs)
        command = "u" if "u" in inputs else (inputs[0] if len(inputs) == 1 else None)

        for port in disturbances or {}:
            if port not in sys.inputs:
                raise ValueError(
                    f"disturbances key {port!r} is not an input port of sys"
                )
            if port == command:
                raise ValueError(
                    f"disturbances key {port!r} is the command port of sys: a draw "
                    "there would replace the policy's command. Give the disturbance "
                    "its own input port (for example 'd')."
                )


def as_stochastic(problem: PlanningProblem) -> StochasticPlanningProblem:
    """
    The same task as a stochastic problem, the inverse of :meth:`StochasticPlanningProblem.nominal`.

    A stochastic problem is returned unchanged. A deterministic one keeps every
    field and starts from its single ``x_start``, with no parameter or
    disturbance draws: it trains and evaluates from one start, so coverage of
    other starts comes from giving a stochastic problem a start distribution.
    """
    if isinstance(problem, StochasticPlanningProblem):
        return problem

    return StochasticPlanningProblem(
        sys=problem.sys,
        x_start=problem.x_start,
        x_goal=problem.x_goal,
        cost=problem.cost,
        X=problem.X,
        U=problem.U,
        X0=problem.X0,
        Xf=problem.Xf,
        tf=problem.tf,
        metadata=problem.metadata,
        infeasible_cost=problem.infeasible_cost,
        discount_rate=problem.discount_rate,
        x0_distribution=Particles(np.atleast_2d(problem.x_start)),
    )


###############################################################################
# Internal machinery
###############################################################################


def require_one_run(n):
    """Refuse more than one realization of a deterministic problem."""
    if int(n) != 1:
        raise ValueError(
            f"a deterministic problem has one realization, got n={n!r}: give the "
            "problem laws (StochasticPlanningProblem) to draw several"
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


def write_set_params(role, set_, entry):
    """Set a set's parameters; refuse a set that holds none (a frozen library set)."""
    # The view's own entry for a set without parameters writes back as nothing
    if entry is None and set_.params is None:
        return
    try:
        set_.params = entry
    except AttributeError:
        raise TypeError(f"{role} ({type(set_).__name__}) holds no parameters") from None


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


def lookup_param(params, name):
    """Entry of a (nested) params dict by dotted name; raises ``KeyError``."""
    node = params
    for part in name.split("."):
        node = node[part]

    return node


def merge_params(params, draws):
    """``params`` with the (nested) ``draws`` overriding matching entries, untouched elsewhere."""
    merged = dict(params)
    for name, value in draws.items():
        if isinstance(value, dict) and isinstance(merged.get(name), dict):
            merged[name] = merge_params(merged[name], value)
        else:
            merged[name] = value

    return merged
