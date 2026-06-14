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
from typing import Any, TypeAlias

import numpy as np

from minilink.core.backends import BACKEND_NUMPY

# These stay loose on purpose: the runtime contract is native-backend behavior
# plus shape checks in evaluators, without making JAX a core dependency.
NativeArray: TypeAlias = Any
NativeScalarExpression: TypeAlias = Any
ObjectiveFunction: TypeAlias = Callable[[NativeArray], NativeScalarExpression]
ArrayFunction: TypeAlias = Callable[[NativeArray], NativeArray]


@dataclass(frozen=True)
class MathematicalProgram:
    """
    Pure finite-dimensional nonlinear program description.

    The generic form is
    ``minimize J(z)`` subject to ``h(z) = 0``, ``g(z) >= 0``, and optional
    box bounds on ``z``.

    Native function contract
    ------------------------
    The mathematical callables are backend-native functions. With NumPy input,
    they should return NumPy scalar/array expressions; with JAX input, they
    should return JAX scalar/array expressions. Do not cast with ``float(...)``
    or force NumPy arrays inside these functions if JAX tracing may be used.

    Shapes are:

    ``J : (n_z,) -> scalar`` (0-D scalar expression),
    ``h : (n_z,) -> (n_h,)``,
    ``g : (n_z,) -> (n_g,)``.

    Parameters
    ----------
    n_z : int
        Dimension of the decision vector ``z``.
    J : callable
        Objective function ``J(z) -> scalar``. The input and output should stay
        native to the active array backend: with NumPy input, return a NumPy
        scalar expression; with JAX input, return a JAX scalar expression.
        Prefer code like ``return z @ z`` over ``return float(z @ z)``.
    h : callable, optional
        Equality residual ``h(z) -> array`` with shape ``(n_h,)``. The returned
        array should be native to the active backend. Missing means no
        equalities.
    g : callable, optional
        Inequality margin ``g(z) -> array`` with shape ``(n_g,)`` and
        convention ``g(z) >= 0``. The returned array should be native to the
        active backend. Missing means no inequalities.
    lower, upper : array_like, optional
        Lower and upper box bounds on ``z``. ``None`` means unbounded on that
        side.
    grad_J : callable, optional
        Objective gradient ``grad_J(z) -> array`` with shape ``(n_z,)``.
    hess_J : callable, optional
        Objective Hessian ``hess_J(z) -> array`` with shape ``(n_z, n_z)``.
    jac_h, jac_g : callable, optional
        Constraint Jacobians ``dh/dz`` and ``dg/dz`` with shapes
        ``(n_h, n_z)`` and ``(n_g, n_z)``.
    backend : str
        Native array backend of the callables (``"numpy"`` or ``"jax"``) —
        the program-evaluator backend they should be compiled with.
    metadata : dict
        Optional transcription or diagnostic metadata.
    """

    n_z: int
    J: ObjectiveFunction
    h: ArrayFunction | None = None
    g: ArrayFunction | None = None
    lower: np.ndarray | None = None
    upper: np.ndarray | None = None
    grad_J: ArrayFunction | None = None
    hess_J: ArrayFunction | None = None
    jac_h: ArrayFunction | None = None
    jac_g: ArrayFunction | None = None
    backend: str = BACKEND_NUMPY
    metadata: dict[str, object] = field(default_factory=dict)

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
