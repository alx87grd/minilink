"""
Deterministic planning problem definitions.

A :class:`PlanningProblem` describes the continuous mathematical task:
the system, admissible sets, boundary sets, optional cost, and optional
parameter bundle. Numerical grids, transcriptions, optimizers, and planner
internals belong to solver packages rather than to the problem object.
"""

from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from minilink.core.costs import CostFunction
from minilink.core.sets import BoxInputSet, BoxSet, InputSet, Set, SingletonSet


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
        Initial state shortcut. When provided, creates a singleton ``X0``.
        Defaults to ``sys.x0``.
    x_goal : array_like, optional
        Terminal goal shortcut. When provided, creates a singleton ``Xf``.
    cost : CostFunction, optional
        Planning cost. Required by solvers that optimize an objective.
    X : Set, optional
        State allowable set. Defaults to the system state bounds.
    U : InputSet, optional
        Input allowable set. Defaults to the system input-port bounds.
    X0, Xf : Set, optional
        Initial and terminal boundary sets.
    params : ProblemParameters, optional
        Explicit parameter bundle for system, cost, and set evaluation.
    """

    sys: object
    x_start: np.ndarray | None = None
    x_goal: np.ndarray | None = None
    cost: CostFunction | None = None
    X: Set | None = None
    U: InputSet | None = None
    X0: Set | None = None
    Xf: Set | None = None
    params: ProblemParameters | None = None
    metadata: dict[str, object] | None = None

    def __post_init__(self) -> None:
        n = int(self.sys.n)

        x_start = self._coerce_state(
            self.sys.x0 if self.x_start is None else self.x_start,
            label="x_start",
            required=True,
        )
        x_goal = self._coerce_state(self.x_goal, label="x_goal", required=False)

        X = BoxSet.from_system_state(self.sys) if self.X is None else self.X
        U = self._default_input_set(self.sys) if self.U is None else self.U
        X0 = SingletonSet(x_start) if self.X0 is None else self.X0
        Xf = SingletonSet(x_goal) if self.Xf is None and x_goal is not None else self.Xf
        params = ProblemParameters() if self.params is None else self.params
        metadata = MappingProxyType(dict(self.metadata or {}))

        if x_start.shape != (n,):
            raise ValueError(f"x_start must have shape ({n},)")
        if x_goal is not None and x_goal.shape != (n,):
            raise ValueError(f"x_goal must have shape ({n},)")

        object.__setattr__(self, "x_start", x_start)
        object.__setattr__(self, "x_goal", x_goal)
        object.__setattr__(self, "X", X)
        object.__setattr__(self, "U", U)
        object.__setattr__(self, "X0", X0)
        object.__setattr__(self, "Xf", Xf)
        object.__setattr__(self, "params", params)
        object.__setattr__(self, "metadata", metadata)

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
