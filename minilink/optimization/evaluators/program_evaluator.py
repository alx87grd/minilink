"""
Base program-evaluator API for compiled mathematical programs.

A :class:`MathematicalProgramEvaluator` is the solver-facing counterpart to a
pure :class:`~minilink.optimization.mathematical_program.MathematicalProgram`.
It owns backend-specific validation, scalar conversion, residual flattening,
and derivative availability.
"""

from abc import ABC, abstractmethod

import numpy as np

from minilink.optimization.mathematical_program import MathematicalProgram


class MathematicalProgramEvaluator(ABC):
    """
    Base class for compiled mathematical-program evaluators.

    Subclasses provide backend-native ``J`` / ``h`` / ``g`` methods. The base
    class defines small SciPy/Ipopt bridge helpers and shared shape utilities.
    """

    def __init__(self, program: MathematicalProgram, sample_z=None):
        self.program = program
        self.n_z = int(program.n_z)
        self.sample_z = self._decision_vector(
            np.zeros(self.n_z) if sample_z is None else sample_z
        )
        self.backend = ""
        self.n_h = 0
        self.n_g = 0

    @abstractmethod
    def J(self, z):
        """Return backend-native objective value ``J(z)``."""
        ...

    @abstractmethod
    def h(self, z):
        """Return backend-native equality residual ``h(z)``."""
        ...

    @abstractmethod
    def g(self, z):
        """Return backend-native inequality margin ``g(z)``."""
        ...

    @property
    def has_gradient(self) -> bool:
        """True when :meth:`gradient` can be called."""
        return False

    @property
    def has_hessian(self) -> bool:
        """True when :meth:`hessian` can be called."""
        return False

    @property
    def has_jacobian_h(self) -> bool:
        """True when :meth:`jacobian_h` can be called."""
        return self.n_h == 0

    @property
    def has_jacobian_g(self) -> bool:
        """True when :meth:`jacobian_g` can be called."""
        return self.n_g == 0

    def objective(self, z) -> float:
        """Return ``J(z)`` as a Python ``float`` for optimizer backends."""
        return self._scalar(self.J(z), "objective")

    def equality_residual(self, z) -> np.ndarray:
        """Return ``h(z)`` as a flat NumPy residual vector."""
        return self._vector(self.h(z), self.n_h, "equality residual")

    def inequality_margin(self, z) -> np.ndarray:
        """Return ``g(z)`` as a flat NumPy nonnegative margin vector."""
        return self._vector(self.g(z), self.n_g, "inequality margin")

    def gradient(self, z) -> np.ndarray:
        """Return objective gradient ``dJ/dz`` or raise if unavailable."""
        raise ValueError("This mathematical-program evaluator has no objective gradient")

    def hessian(self, z) -> np.ndarray:
        """Return objective Hessian ``d2J/dz2`` or raise if unavailable."""
        raise ValueError("This mathematical-program evaluator has no objective Hessian")

    def jacobian_h(self, z) -> np.ndarray:
        """Return equality Jacobian ``dh/dz``."""
        if self.n_h == 0:
            return np.zeros((0, self.n_z), dtype=float)
        raise ValueError("This mathematical-program evaluator has no equality Jacobian")

    def jacobian_g(self, z) -> np.ndarray:
        """Return inequality Jacobian ``dg/dz``."""
        if self.n_g == 0:
            return np.zeros((0, self.n_z), dtype=float)
        raise ValueError("This mathematical-program evaluator has no inequality Jacobian")

    def scipy_bounds(self):
        """Return SciPy ``minimize`` bounds or ``None``."""
        if self.program.lower is None and self.program.upper is None:
            return None
        lower = (
            np.full(self.n_z, -np.inf)
            if self.program.lower is None
            else self.program.lower
        )
        upper = (
            np.full(self.n_z, np.inf)
            if self.program.upper is None
            else self.program.upper
        )
        return list(zip(lower, upper))

    def _decision_vector(self, z) -> np.ndarray:
        arr = np.asarray(z, dtype=float).reshape(-1)
        if arr.shape != (self.n_z,):
            raise ValueError(f"decision vector must have shape ({self.n_z},)")
        return arr

    def _scalar(self, value, label: str) -> float:
        arr = np.asarray(value, dtype=float)
        if arr.size != 1:
            raise ValueError(f"{label} must return a scalar")
        return float(arr.reshape(-1)[0])

    def _vector(self, value, n_expected: int, label: str) -> np.ndarray:
        arr = np.asarray(value, dtype=float).reshape(-1)
        if arr.shape != (n_expected,):
            raise ValueError(f"{label} must have shape ({n_expected},)")
        return arr

    def _matrix(
        self,
        value,
        shape: tuple[int, int],
        label: str,
    ) -> np.ndarray:
        arr = np.asarray(value, dtype=float)
        if arr.shape != shape:
            raise ValueError(f"{label} must have shape {shape}")
        return arr
