"""
Generic finite-dimensional mathematical programs.

The optimization layer represents problems in the textbook NLP form

``minimize J(z) subject to h(z) = 0, g(z) >= 0, lower <= z <= upper``.

The :class:`MathematicalProgram` itself stays close to that math: it stores
pure callables of the decision vector ``z`` and no solver state such as an
initial guess. Backend-specific casting, JAX compilation, and SciPy-friendly
wrappers live outside this object on program evaluators owned by optimizers.
"""

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

ArrayFunction = Callable[[np.ndarray], np.ndarray]
ScalarFunction = Callable[[np.ndarray], float]


@dataclass(frozen=True)
class MathematicalProgram:
    """
    Pure finite-dimensional nonlinear program description.

    The generic form is
    ``minimize J(z)`` subject to ``h(z) = 0``, ``g(z) >= 0``, and optional
    box bounds on ``z``.

    Parameters
    ----------
    n_z : int
        Dimension of the decision vector ``z``.
    J : callable
        Objective function ``J(z) -> scalar``.
    h : callable, optional
        Equality residual ``h(z) -> array``. Missing means no equalities.
    g : callable, optional
        Inequality margin ``g(z) -> array`` with convention ``g(z) >= 0``.
        Missing means no inequalities.
    lower, upper : array_like, optional
        Lower and upper box bounds on ``z``. ``None`` means unbounded on that
        side.
    grad_J : callable, optional
        Objective gradient ``grad_J(z) -> np.ndarray``.
    hess_J : callable, optional
        Objective Hessian ``hess_J(z) -> np.ndarray``.
    jac_h, jac_g : callable, optional
        Constraint Jacobians ``dh/dz`` and ``dg/dz``.
    metadata : dict
        Optional transcription or diagnostic metadata.
    """

    n_z: int
    J: ScalarFunction
    h: ArrayFunction | None = None
    g: ArrayFunction | None = None
    lower: np.ndarray | None = None
    upper: np.ndarray | None = None
    grad_J: ArrayFunction | None = None
    hess_J: ArrayFunction | None = None
    jac_h: ArrayFunction | None = None
    jac_g: ArrayFunction | None = None
    metadata: dict[str, object] = field(default_factory=dict)

    problem_class: str = "nlp"

    def __post_init__(self) -> None:
        n_z = int(self.n_z)
        if n_z <= 0:
            raise ValueError("n_z must be a positive integer")
        lower = _as_vector_or_none("lower", self.lower, n_z)
        upper = _as_vector_or_none("upper", self.upper, n_z)
        if lower is not None and upper is not None and np.any(lower > upper):
            raise ValueError("lower bounds must be less than or equal to upper bounds")
        object.__setattr__(self, "n_z", n_z)
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)
        object.__setattr__(self, "metadata", dict(self.metadata))


def _as_vector_or_none(name: str, value, n_z: int) -> np.ndarray | None:
    if value is None:
        return None
    arr = np.asarray(value, dtype=float).reshape(-1).copy()
    if arr.shape != (n_z,):
        raise ValueError(f"{name} bounds must have shape ({n_z},)")
    return arr


@dataclass(frozen=True)
class OptimizationResult:
    """
    Result returned by an :class:`~minilink.optimization.optimizer.Optimizer`.

    Parameters
    ----------
    solve_time_s : float, optional
        Wall-clock time in seconds for the backend solver only, when the caller
        requested timing on :meth:`~minilink.optimization.optimizer.Optimizer.solve`
        (``record_solve_time=True``) or a summary report (``disp=True``).
        ``None`` means not measured.
    """

    z: np.ndarray
    success: bool
    cost: float | None = None
    message: str = ""
    stats: dict[str, object] = field(default_factory=dict)
    raw_result: object | None = None
    solve_time_s: float | None = None

    def __post_init__(self) -> None:
        z = np.asarray(self.z, dtype=float).reshape(-1).copy()
        object.__setattr__(self, "z", z)
        object.__setattr__(self, "stats", dict(self.stats))
        if self.solve_time_s is not None:
            object.__setattr__(self, "solve_time_s", float(self.solve_time_s))
