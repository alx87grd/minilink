"""
Deterministic planning problem definitions.

A :class:`PlanningProblem` describes the continuous mathematical task:
the system, admissible sets, boundary sets, continuous horizon ``tf``,
optional cost, and optional parameter bundle. Knot count and other
discretization choices belong to solver packages, not the problem object.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from minilink.core.costs import CostFunction
from minilink.core.sets import BoxInputSet, BoxSet, InputSet, Set, SingletonSet
from minilink.core.system import System
from minilink.planning.distributions import Distribution


@dataclass(frozen=True)
class ProblemParameters:
    """
    Optional scenario-level parameter bundle for planning.

    Parameters
    ----------
    system : object, optional
        Parameters for system dynamics. First-pass solvers may freeze these
        at compile time until parametric evaluator tiers are mature.
    cost : object, optional
        Parameters passed to :class:`~minilink.core.costs.CostFunction`.
    sets : object, optional
        Parameters passed to allowable set objects.
    scene : object, optional
        Reserved for pipeline B spatial overrides (``ObstacleBank`` /
        ``SceneParameters``). Always ``None`` until bind ``J(z, p)`` lands;
        online ``params={"scene": …}`` raises ``NotImplementedError``.
    """

    system: object | None = None
    cost: object | None = None
    sets: object | None = None
    scene: object | None = None


@dataclass(frozen=True)
class PlanningProblem:
    """
    Deterministic planning problem for a Minilink system.

    The generic mathematical form is ``x(t) in X(t)``,
    ``u(t) in U(x(t), t)``, with boundary sets ``X0`` and optional ``Xf``.
    A cost can be attached for optimization and policy-planning solvers,
    but feasibility/search planners may leave it unset.

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
        Representative terminal target. When ``Xf`` is omitted, this creates a
        singleton ``Xf``. When ``Xf`` is provided, it must belong to ``Xf`` and
        is used by initial guesses or reports that need one concrete target.
    cost : CostFunction, optional
        Planning cost. Required by solvers that optimize an objective.
    X : Set, optional
        State allowable set. Defaults to the system state bounds.
    U : InputSet, optional
        Input allowable set. Defaults to the system input-port bounds.
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
    on_exit : {"infeasible", "terminate"}
        What a trajectory leaving ``X`` means. ``"infeasible"`` (default): a
        hard constraint — trajectory optimization rejects it, value iteration
        charges its out-of-bound cost. ``"terminate"``: the trajectory ends
        there and ``exit_cost`` is charged — the rule reinforcement learning
        trains with, so that trajopt, DP and RL score the same trajectory the
        same way.
    exit_cost : float or callable, optional
        Price of leaving ``X``: a scalar, or ``exit_cost(x, t)`` evaluated at
        the exit state. Read by value iteration as its default
        ``out_of_bound_cost`` and by RL as the terminal penalty. Should exceed
        the cost-to-go of finishing the task from a typical start.
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
    on_exit: str = "infeasible"
    exit_cost: object = None

    EXIT_RULES = ("infeasible", "terminate")

    def __post_init__(self) -> None:
        n = int(self.sys.n)

        tf = self._coerce_tf(self.tf)
        if self.on_exit not in self.EXIT_RULES:
            raise ValueError(
                f"on_exit must be one of {self.EXIT_RULES}, got {self.on_exit!r}"
            )
        if self.exit_cost is not None and not callable(self.exit_cost):
            object.__setattr__(self, "exit_cost", float(self.exit_cost))

        x_start = self._coerce_state(
            self._default_x_start(),
            label="x_start",
            required=True,
        )
        x_goal = self._coerce_state(
            self._default_x_goal(),
            label="x_goal",
            required=False,
        )

        X = BoxSet.from_system_state(self.sys) if self.X is None else self.X
        U = self._default_input_set(self.sys) if self.U is None else self.U
        X0 = SingletonSet(x_start) if self.X0 is None else self.X0
        Xf = SingletonSet(x_goal) if self.Xf is None and x_goal is not None else self.Xf
        params = self._coerce_params(self.params)
        metadata = self._coerce_metadata(self.metadata)

        self._require_type("X", X, Set)
        self._require_type("U", U, InputSet)
        self._require_type("X0", X0, Set)
        if Xf is not None:
            self._require_type("Xf", Xf, Set)

        if x_start.shape != (n,):
            raise ValueError(f"x_start must have shape ({n},)")
        if x_goal is not None and x_goal.shape != (n,):
            raise ValueError(f"x_goal must have shape ({n},)")
        self._require_member(
            X0,
            x_start,
            set_label="X0",
            point_label="x_start",
            set_params=params.sets,
        )
        if Xf is not None and x_goal is not None:
            self._require_member(
                Xf,
                x_goal,
                set_label="Xf",
                point_label="x_goal",
                set_params=params.sets,
            )

        object.__setattr__(self, "tf", tf)
        object.__setattr__(self, "x_start", x_start)
        object.__setattr__(self, "x_goal", x_goal)
        object.__setattr__(self, "X", X)
        object.__setattr__(self, "U", U)
        object.__setattr__(self, "X0", X0)
        object.__setattr__(self, "Xf", Xf)
        object.__setattr__(self, "params", params)
        object.__setattr__(self, "metadata", metadata)

    def _default_x_start(self):
        if self.x_start is not None:
            return self.x_start
        if isinstance(self.X0, SingletonSet):
            return self.X0.point
        return self.sys.x0

    def _default_x_goal(self):
        if self.x_goal is not None:
            return self.x_goal
        if isinstance(self.Xf, SingletonSet):
            return self.Xf.point
        return None

    @staticmethod
    def _coerce_state(
        x: object,
        *,
        label: str,
        required: bool,
    ) -> np.ndarray | None:
        if x is None:
            if required:
                raise ValueError(f"{label} is required")
            return None
        return np.asarray(x, dtype=float).reshape(-1).copy()

    @staticmethod
    def _coerce_params(params) -> ProblemParameters:
        if params is None:
            return ProblemParameters()
        if isinstance(params, ProblemParameters):
            return params
        raise TypeError("params must be a ProblemParameters instance or None")

    @staticmethod
    def _coerce_metadata(metadata) -> Mapping[str, object]:
        if metadata is None:
            return MappingProxyType({})
        if not isinstance(metadata, Mapping):
            raise TypeError("metadata must be a mapping or None")
        return MappingProxyType(dict(metadata))

    @staticmethod
    def _require_type(label: str, value: object, expected_type: type) -> None:
        if not isinstance(value, expected_type):
            raise TypeError(f"{label} must be a {expected_type.__name__} instance")

    @staticmethod
    def _require_member(
        set_: Set,
        point: np.ndarray,
        *,
        set_label: str,
        point_label: str,
        set_params,
    ) -> None:
        if not set_.contains(point, params=set_params):
            raise ValueError(f"{point_label} must belong to {set_label}")

    @staticmethod
    def _default_input_set(sys: object) -> BoxInputSet:
        lower = np.zeros(sys.m)
        upper = np.zeros(sys.m)
        i = 0
        for port in sys.inputs.values():
            lower[i : i + port.dim] = port.lower_bound
            upper[i : i + port.dim] = port.upper_bound
            i += port.dim
        return BoxInputSet.from_bounds(lower, upper)

    @property
    def has_goal(self) -> bool:
        """Return ``True`` when a terminal goal or boundary set is available."""
        return self.Xf is not None

    def exit_penalty(self, x, t=0.0):
        """Cost charged at an exit state (scalar or ``exit_cost(x, t)``); ``None`` if unset."""
        if self.exit_cost is None:
            return None
        if callable(self.exit_cost):
            return self.exit_cost(x, t)
        return self.exit_cost

    def horizon_kind(self) -> str:
        """``"finite"`` or ``"infinite"``: the cost's declaration, else from ``tf``."""
        if self.cost is None:
            return (
                "infinite" if self.tf is None or not np.isfinite(self.tf) else "finite"
            )
        return self.cost.horizon_kind(self.tf)

    @property
    def has_cost(self) -> bool:
        """Return ``True`` when the problem has a cost function."""
        return self.cost is not None

    def require_cost(self) -> CostFunction:
        """Return the cost or raise a clear solver-facing error."""
        if self.cost is None:
            raise ValueError("This planner requires problem.cost")
        return self.cost

    def require_goal(self) -> Set:
        """Return the terminal set or raise a clear solver-facing error."""
        if self.Xf is None:
            raise ValueError("This planner requires a terminal set or x_goal")
        return self.Xf

    def require_finite_tf(self) -> float:
        """Return a finite horizon or raise for solvers that need a time grid."""
        if self.tf is None:
            raise ValueError("This planner requires a finite problem.tf")
        if not np.isfinite(self.tf):
            raise ValueError(
                "This planner requires a finite problem.tf "
                "(got +inf for infinite-horizon)"
            )
        return float(self.tf)

    @staticmethod
    def _coerce_tf(tf: object) -> float | None:
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
        ``{name: Distribution}`` over entries of ``sys.params`` (domain
        randomization, robustness sweeps); each sample overrides those entries.
        A dotted name reaches into a diagram's subsystem params
        (``"sys.mass"`` for the plant inside a closed loop).
    disturbances : mapping, optional
        ``{port_id: Distribution}`` over input ports of ``sys``: a fresh draw
        per step held on that port (a seeded disturbance signal).
    criterion : {"expectation", "worst_case"}
        What "optimal" means over the draws. Reinforcement learning optimizes
        the expectation; Monte Carlo evaluation reports both.

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
        if self.x0_distribution is None:
            raise ValueError("StochasticPlanningProblem requires x0_distribution")
        if self.x_start is None:
            object.__setattr__(self, "x_start", self.x0_distribution.mean())
        if self.X0 is None and self.x0_distribution.support is not None:
            object.__setattr__(self, "X0", self.x0_distribution.support)
        super().__post_init__()
        if self.criterion not in self.CRITERIA:
            raise ValueError(
                f"criterion must be one of {self.CRITERIA}, got {self.criterion!r}"
            )
        if int(self.x0_distribution.dim) != int(self.sys.n):
            raise ValueError("x0_distribution.dim must equal sys.n")
        for name in self.params_distribution or {}:
            try:
                leaf = lookup_param(self.sys.params, name)
            except KeyError:
                raise ValueError(
                    f"params_distribution key {name!r} is not in sys.params"
                ) from None
            if isinstance(leaf, dict):
                raise ValueError(
                    f"params_distribution over {name!r}: distributions apply to "
                    "array-valued parameters, not to a whole subsystem dict"
                )
        for port in self.disturbances or {}:
            if port not in self.sys.inputs:
                raise ValueError(
                    f"disturbances key {port!r} is not an input port of sys"
                )
        object.__setattr__(
            self, "params_distribution", dict(self.params_distribution or {})
        )
        object.__setattr__(self, "disturbances", dict(self.disturbances or {}))

    def sample_x0(self, key, n=None):
        """Draw initial states: ``(n,)`` for one, ``(n_samples, n)`` with ``n``."""
        return self.x0_distribution.sample(key, n)

    def sample_params(self, key):
        """Draw one ``{name: value}`` override of ``sys.params`` (empty if none), nominal shapes."""
        names = list(self.params_distribution)
        keys = split_keys(key, len(names))
        draws = {}
        for name, k in zip(names, keys):
            value = self.params_distribution[name].sample(k)
            value = value.reshape(np.shape(lookup_param(self.sys.params, name)))
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

    @property
    def is_stochastic(self) -> bool:
        return True

    def nominal(self) -> PlanningProblem:
        """Certainty-equivalent deterministic problem: mean start, no draws."""
        return PlanningProblem(
            sys=self.sys,
            x_start=self.x0_distribution.mean(),
            x_goal=self.x_goal,
            cost=self.cost,
            X=self.X,
            U=self.U,
            Xf=self.Xf,
            tf=self.tf,
            params=self.params,
            metadata=self.metadata,
            on_exit=self.on_exit,
            exit_cost=self.exit_cost,
        )


def split_keys(key, n):
    """``n`` independent keys from a JAX key, or the same NumPy generator ``n`` times."""
    if n == 0:
        return []
    if type(key).__module__.startswith("jax"):
        import jax

        return list(jax.random.split(key, n))
    rng = key if isinstance(key, np.random.Generator) else np.random.default_rng(key)
    return [rng] * n


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
