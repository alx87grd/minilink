"""Planning problems: the deterministic task, its stochastic class, and their parameters."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, replace
from types import MappingProxyType

import numpy as np

from minilink.core.costs import CostFunction
from minilink.core.distributions import Distribution, Particles, split_keys
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.sets import BoxInputSet, BoxSet, InputSet, Set, SingletonSet
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class PlanningProblem:
    """
    Deterministic planning problem for a Minilink system.

    The generic mathematical form is ``x(t) in X(t)``,
    ``u(t) in U(x(t), t)``, with boundary sets ``X0`` and optional ``Xf``.
    A cost can be attached for optimization and policy-planning solvers,
    but feasibility/search planners may leave it unset.

    The problem is the continuous task: the system, the admissible and boundary
    sets, the continuous horizon ``tf``, the optional cost and parameter bundle.
    The knot count and the other discretization choices belong to the solvers,
    not to the problem.

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
    params : ProblemParameters, optional
        Explicit parameter bundle for system, cost, and set evaluation.
    infeasible_cost : float or callable, optional
        The finite stand-in for the infinite cost of an infeasible trajectory (one
        that leaves ``X``): a scalar, or ``infeasible_cost(x, t)`` evaluated at the
        exit state. Value iteration reads it as its default ``out_of_bound_cost``;
        the learners and the Monte Carlo score charge it when a trajectory leaves
        ``X``. Unset, the sampling tools derive a bound of any feasible cost and
        announce it; a declared price should exceed the cost-to-go of finishing the
        task from a typical start.
    """

    sys: System
    x_start: np.ndarray | None = None
    x_goal: np.ndarray | None = None
    cost: CostFunction | None = None
    X: Set | None = None
    U: InputSet | None = None
    X0: Set | None = None
    Xf: Set | None = None
    tf: float | None = None
    params: ProblemParameters | None = None
    metadata: Mapping[str, object] | None = None
    infeasible_cost: object = None

    def __post_init__(self) -> None:
        n = int(self.sys.n)

        # The horizon, and a scalar price of infeasibility as a float
        tf = self.coerce_tf(self.tf)
        if self.infeasible_cost is not None and not callable(self.infeasible_cost):
            object.__setattr__(self, "infeasible_cost", float(self.infeasible_cost))

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

        # The parameter bundle (a random plant's draw at its mean) and the metadata
        params = self.coerce_params(self.params)
        if params.system is None and self.sys.is_random:
            params = replace(params, system=self.sys.realize(None))  # Noise at its mean
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
            set_params=params.sets,
        )
        if Xf is not None and x_goal is not None:
            self.require_member(
                Xf,
                x_goal,
                set_label="Xf",
                point_label="x_goal",
                set_params=params.sets,
            )

        # The coerced fields, frozen
        object.__setattr__(self, "tf", tf)
        object.__setattr__(self, "x_start", x_start)
        object.__setattr__(self, "x_goal", x_goal)
        object.__setattr__(self, "X", X)
        object.__setattr__(self, "U", U)
        object.__setattr__(self, "X0", X0)
        object.__setattr__(self, "Xf", Xf)
        object.__setattr__(self, "params", params)
        object.__setattr__(self, "metadata", metadata)

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

    def infeasible_penalty(self, x, t=0.0):
        """The declared price of infeasibility at ``(x, t)`` (scalar or callable); ``None`` when unset."""
        price = self.infeasible_cost
        if price is None:
            return None

        return price(x, t) if callable(price) else price

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
    def coerce_params(params) -> ProblemParameters:
        """The parameter bundle, empty when unset."""
        if params is None:
            return ProblemParameters()
        if isinstance(params, ProblemParameters):
            return params

        raise TypeError("params must be a ProblemParameters instance or None")

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
        set_params,
    ) -> None:
        """Refuse a representative point outside its boundary set."""
        if not set_.contains(point, params=set_params):
            raise ValueError(f"{point_label} must belong to {set_label}")


@dataclass(frozen=True)
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
    criterion : {"expectation", "worst_case"}
        What "optimal" means over the draws. Reinforcement learning optimizes
        the expectation; Monte Carlo evaluation reports both.

    Notes
    -----
    Solve verbs (RL, gain search) and the evaluate verb (Monte Carlo) both
    read this description; :meth:`nominal` returns the certainty-equivalent
    :class:`PlanningProblem` for trajectory optimization and LQR.
    """

    x0_distribution: Distribution | None = None
    params_distribution: Mapping[str, Distribution] | None = None
    disturbances: Mapping[str, Distribution] | None = None
    criterion: str = "expectation"

    CRITERIA = ("expectation", "worst_case")

    def __post_init__(self) -> None:
        # The start law: its mean the default start, its support the default X0
        self.require_start_law(self.x0_distribution)
        if self.x_start is None:
            object.__setattr__(self, "x_start", self.x0_distribution.mean)
        if self.X0 is None and self.x0_distribution.support is not None:
            object.__setattr__(self, "X0", self.x0_distribution.support)

        # The deterministic spine
        super().__post_init__()

        # The criterion, and the draws against the system they perturb
        self.require_criterion(self.criterion, self.CRITERIA)
        self.require_start_dim(self.x0_distribution, self.sys)
        self.require_param_names(self.params_distribution, self.sys)
        self.require_disturbance_ports(self.disturbances, self.sys)

        # The draws as plain dicts, frozen
        object.__setattr__(
            self, "params_distribution", dict(self.params_distribution or {})
        )
        object.__setattr__(self, "disturbances", dict(self.disturbances or {}))

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
            params=self.params,
            metadata=self.metadata,
            infeasible_cost=self.infeasible_cost,
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
        """Refuse a criterion that is not one of ``criteria``."""
        if criterion not in criteria:
            raise ValueError(f"criterion must be one of {criteria}, got {criterion!r}")

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


@dataclass(frozen=True)
class ProblemParameters:
    """
    Optional scenario-level parameter bundle for planning.

    Parameters
    ----------
    system : object, optional
        Parameters for the system dynamics; a solver may freeze them at compile
        time.
    cost : object, optional
        Parameters passed to :class:`~minilink.core.costs.CostFunction`.
    sets : object, optional
        Parameters passed to allowable set objects.
    scene : object, optional
        Spatial overrides of the scene bind (``ObstacleBank`` /
        ``SceneParameters``), not implemented: always ``None``, and an online
        ``params={"scene": …}`` raises ``NotImplementedError``.
    """

    system: object | None = None
    cost: object | None = None
    sets: object | None = None
    scene: object | None = None


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
        params=problem.params,
        metadata=problem.metadata,
        infeasible_cost=problem.infeasible_cost,
        x0_distribution=Particles(np.atleast_2d(problem.x_start)),
    )


###############################################################################
# Internal machinery
###############################################################################


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
