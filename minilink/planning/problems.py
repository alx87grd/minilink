"""
Deterministic planning problem definitions.

A :class:`PlanningProblem` describes the continuous mathematical task:
the system, admissible sets, boundary sets, optional cost, and optional
parameter bundle. Numerical grids, transcriptions, optimizers, and planner
internals belong to solver packages rather than to the problem object.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from minilink.core.costs import CostFunction
from minilink.core.sets import BoxInputSet, BoxSet, InputSet, Set, SingletonSet
from minilink.core.system import System


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
    """

    system: object | None = None
    cost: object | None = None
    sets: object | None = None


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
    params : ProblemParameters, optional
        Explicit parameter bundle for system, cost, and set evaluation.
    """

    sys: System
    x_start: np.ndarray | None = None
    x_goal: np.ndarray | None = None
    cost: CostFunction | None = None
    X: Set | None = None
    U: InputSet | None = None
    X0: Set | None = None
    Xf: Set | None = None
    params: ProblemParameters | None = None
    metadata: Mapping[str, object] | None = None

    def __post_init__(self) -> None:
        n = int(self.sys.n)

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
