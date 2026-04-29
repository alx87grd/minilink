"""
Generic finite-dimensional mathematical programs.

The optimization layer represents problems in the textbook form

``minimize J(z) subject to h(z) = 0, g(z) >= 0, lower <= z <= upper``.

Domain packages such as planning own transcription details. Optimizers
only see finite-dimensional functions of the decision vector ``z``.
"""

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

ArrayFunction = Callable[[np.ndarray], np.ndarray]
ScalarFunction = Callable[[np.ndarray], float]


@dataclass(frozen=True)
class VariableBounds:
    """
    Box bounds on the decision vector ``z``.

    Parameters
    ----------
    lower, upper : array_like, optional
        Lower and upper bounds. ``None`` means unbounded on that side.
    """

    lower: np.ndarray | None = None
    upper: np.ndarray | None = None

    def __post_init__(self) -> None:
        lower = (
            None
            if self.lower is None
            else np.asarray(self.lower, dtype=float).reshape(-1).copy()
        )
        upper = (
            None
            if self.upper is None
            else np.asarray(self.upper, dtype=float).reshape(-1).copy()
        )
        if lower is not None and upper is not None:
            if lower.shape != upper.shape:
                raise ValueError("lower and upper bounds must have the same shape")
            if np.any(lower > upper):
                raise ValueError(
                    "lower bounds must be less than or equal to upper bounds"
                )
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)

    def validate_dim(self, n_z: int) -> None:
        """Raise if the bounds do not match decision dimension ``n_z``."""
        if self.lower is not None and self.lower.shape != (n_z,):
            raise ValueError(f"lower bounds must have shape ({n_z},)")
        if self.upper is not None and self.upper.shape != (n_z,):
            raise ValueError(f"upper bounds must have shape ({n_z},)")


@dataclass(frozen=True)
class EqualityConstraint:
    """
    Vector equality constraint ``h(z) = 0``.

    Parameters
    ----------
    h : callable
        Function returning equality residuals.
    jac : callable, optional
        Function returning the Jacobian ``dh/dz``.
    name : str
        Human-readable label for diagnostics.
    metadata : dict
        Optional transcription metadata for debugging.
    """

    h: ArrayFunction
    jac: ArrayFunction | None = None
    name: str = ""
    metadata: dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "metadata", dict(self.metadata))

    def residual(self, z: np.ndarray) -> np.ndarray:
        """Return ``h(z)`` as a flat residual vector."""
        return np.asarray(self.h(np.asarray(z, dtype=float)), dtype=float).reshape(-1)


@dataclass(frozen=True)
class InequalityConstraint:
    """
    Vector inequality constraint ``g(z) >= 0``.

    Parameters
    ----------
    g : callable
        Function returning nonnegative feasibility margins.
    jac : callable, optional
        Function returning the Jacobian ``dg/dz``.
    name : str
        Human-readable label for diagnostics.
    metadata : dict
        Optional transcription metadata for debugging.
    """

    g: ArrayFunction
    jac: ArrayFunction | None = None
    name: str = ""
    metadata: dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "metadata", dict(self.metadata))

    def margin(self, z: np.ndarray) -> np.ndarray:
        """Return ``g(z)`` as a flat nonnegative margin vector."""
        return np.asarray(self.g(np.asarray(z, dtype=float)), dtype=float).reshape(-1)


@dataclass(frozen=True)
class MathematicalProgram:
    """
    Finite-dimensional mathematical program passed to a backend.

    The generic form is
    ``minimize J(z)`` subject to ``h(z) = 0``, ``g(z) >= 0``, and optional
    box bounds on ``z``.

    Parameters
    ----------
    J : callable
        Objective function ``J(z) -> float``.
    z0 : np.ndarray
        Initial decision vector.
    bounds : VariableBounds, optional
        Box bounds on the decision vector.
    equalities : tuple
        Equality constraints ``h(z) = 0``.
    inequalities : tuple
        Inequality constraints ``g(z) >= 0``.
    metadata : dict
        Optional transcription metadata for debugging.
    grad : callable, optional
        Objective gradient ``grad(z) -> np.ndarray``.
    hess : callable, optional
        Objective Hessian ``hess(z) -> np.ndarray``.
    """

    J: ScalarFunction
    z0: np.ndarray
    bounds: VariableBounds | None = None
    equalities: tuple[EqualityConstraint, ...] = ()
    inequalities: tuple[InequalityConstraint, ...] = ()
    metadata: dict[str, object] = field(default_factory=dict)
    grad: ArrayFunction | None = None
    hess: ArrayFunction | None = None

    def __post_init__(self) -> None:
        z0 = np.asarray(self.z0, dtype=float).reshape(-1).copy()
        bounds = self.bounds
        if bounds is not None:
            bounds.validate_dim(int(z0.size))
        object.__setattr__(self, "z0", z0)
        object.__setattr__(self, "equalities", tuple(self.equalities))
        object.__setattr__(self, "inequalities", tuple(self.inequalities))
        object.__setattr__(self, "metadata", dict(self.metadata))

    @property
    def n_z(self) -> int:
        """Dimension of the decision vector ``z``."""
        return int(self.z0.size)

    def objective(self, z: np.ndarray) -> float:
        """Return ``J(z)`` as a scalar float."""
        return float(self.J(np.asarray(z, dtype=float)))

    def gradient(self, z: np.ndarray) -> np.ndarray:
        """Return the objective gradient or raise if none is available."""
        if self.grad is None:
            raise ValueError("This mathematical program has no objective gradient")
        return np.asarray(self.grad(np.asarray(z, dtype=float)), dtype=float).reshape(
            self.n_z
        )

    def hessian(self, z: np.ndarray) -> np.ndarray:
        """Return the objective Hessian or raise if none is available."""
        if self.hess is None:
            raise ValueError("This mathematical program has no objective Hessian")
        arr = np.asarray(self.hess(np.asarray(z, dtype=float)), dtype=float)
        if arr.shape != (self.n_z, self.n_z):
            raise ValueError(
                f"objective Hessian must have shape ({self.n_z}, {self.n_z})"
            )
        return arr


@dataclass(frozen=True)
class OptimizationResult:
    """
    Result returned by an :class:`~minilink.optimization.optimizers.optimizer.Optimizer`.
    """

    z: np.ndarray
    success: bool
    cost: float | None = None
    message: str = ""
    stats: dict[str, object] = field(default_factory=dict)
    raw_result: object | None = None

    def __post_init__(self) -> None:
        z = np.asarray(self.z, dtype=float).reshape(-1).copy()
        object.__setattr__(self, "z", z)
        object.__setattr__(self, "stats", dict(self.stats))
