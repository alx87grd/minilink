"""Program evaluators: the solver-facing side of a mathematical program."""

from abc import ABC, abstractmethod

import numpy as np

from minilink.optimization.mathematical_program import MathematicalProgram

###############################################################################
# Public API
###############################################################################


class MathematicalProgramEvaluator(ABC):
    """
    Base class for compiled mathematical-program evaluators.

    The solver-facing counterpart of a pure
    :class:`~minilink.optimization.mathematical_program.MathematicalProgram`:
    it owns backend-specific validation, scalar conversion, residual
    flattening, and derivative availability. Subclasses provide
    backend-native ``J`` / ``h`` / ``g`` methods. The base class defines
    SciPy/Ipopt bridge helpers that convert native scalar and array
    expressions to Python ``float`` / flat NumPy arrays.
    """

    def __init__(self, program: MathematicalProgram, sample_z=None):
        # The program and its size
        self.program = program
        self.n_z = int(program.n_z)

        # The sample point, the origin unless given
        self.sample_z = self.decision_vector(
            np.zeros(self.n_z) if sample_z is None else sample_z
        )

        # The backend and the constraint sizes, set by each subclass
        self.backend = ""
        self.n_h = 0
        self.n_g = 0

    @abstractmethod
    def J(self, z):
        """Return backend-native objective scalar expression ``J(z)``."""
        ...

    @abstractmethod
    def h(self, z):
        """Return backend-native equality residual array ``h(z)``."""
        ...

    @abstractmethod
    def g(self, z):
        """Return backend-native inequality margin array ``g(z)``."""
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
        """True when :meth:`jacobian_h` can be evaluated.

        Note this means "callable", not "analytic derivatives exist": with no
        equality constraints the empty ``(0, n_z)`` Jacobian is always
        available. Backends with analytic/AD derivatives override this.
        """
        return self.n_h == 0

    @property
    def has_jacobian_g(self) -> bool:
        """True when :meth:`jacobian_g` can be evaluated (see :attr:`has_jacobian_h`)."""
        return self.n_g == 0

    # The SciPy/Ipopt bridge: Python floats and flat NumPy arrays

    def objective(self, z) -> float:
        """Return ``J(z)`` as a Python ``float`` for optimizer backends."""
        return self.as_scalar(self.J(z), "objective")

    def equality_residual(self, z) -> np.ndarray:
        """Return ``h(z)`` as a flat NumPy residual vector."""
        return self.as_vector(self.h(z), self.n_h, "equality residual")

    def inequality_margin(self, z) -> np.ndarray:
        """Return ``g(z)`` as a flat NumPy nonnegative margin vector."""
        return self.as_vector(self.g(z), self.n_g, "inequality margin")

    def constraint_violations(self, z) -> tuple[float, float | None, float]:
        """Constraint-violation summary of a solution ``z``.

        Returns
        -------
        eq_inf : float
            Max absolute equality residual (0.0 when there are no equalities).
        min_ineq : float or None
            Smallest inequality margin (negative = violated); ``None`` when
            there are no inequality constraints.
        bound_inf : float
            Max box-bound violation (0.0 when within bounds or unbounded).
        """
        n_h = self.n_h
        n_g = self.n_g
        lower = self.program.lower
        upper = self.program.upper

        # ‖h(z)‖∞, the largest equality residual
        eq_inf = 0.0
        if n_h:
            eq_inf = float(np.max(np.abs(self.equality_residual(z))))

        # minᵢ gᵢ(z), the smallest inequality margin
        min_ineq: float | None = None
        if n_g:
            min_ineq = float(np.min(self.inequality_margin(z)))

        # The largest bound violation, max(lower − z, z − upper, 0) over the entries
        bound_inf = 0.0
        if lower is not None:
            bound_inf = max(bound_inf, float(np.max(np.maximum(lower - z, 0.0))))
        if upper is not None:
            bound_inf = max(bound_inf, float(np.max(np.maximum(z - upper, 0.0))))

        return eq_inf, min_ineq, bound_inf

    # The derivatives a backend has not supplied

    def gradient(self, z) -> np.ndarray:
        """Return objective gradient ``dJ/dz`` or raise if unavailable."""
        raise ValueError(
            "This mathematical-program evaluator has no objective gradient"
        )

    def hessian(self, z) -> np.ndarray:
        """Return objective Hessian ``d2J/dz2`` or raise if unavailable."""
        raise ValueError("This mathematical-program evaluator has no objective Hessian")

    def jacobian_h(self, z) -> np.ndarray:
        """Return equality Jacobian ``dh/dz``."""
        # With no equalities the Jacobian is the empty (0, n_z) matrix
        if self.n_h == 0:
            return np.zeros((0, self.n_z), dtype=float)
        raise ValueError("This mathematical-program evaluator has no equality Jacobian")

    def jacobian_g(self, z) -> np.ndarray:
        """Return inequality Jacobian ``dg/dz``."""
        # With no inequalities the Jacobian is the empty (0, n_z) matrix
        if self.n_g == 0:
            return np.zeros((0, self.n_z), dtype=float)
        raise ValueError(
            "This mathematical-program evaluator has no inequality Jacobian"
        )

    def scipy_bounds(self):
        """Return SciPy ``minimize`` bounds or ``None``."""
        n_z = self.n_z
        lower = self.program.lower
        upper = self.program.upper

        # An unbounded program passes no bounds
        if lower is None and upper is None:
            return None

        # One (lower, upper) pair per entry of z, an open side at ∓∞
        lower = np.full(n_z, -np.inf) if lower is None else lower
        upper = np.full(n_z, np.inf) if upper is None else upper
        bounds = list(zip(lower, upper))

        return bounds

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def check_derivative_shapes(self, z) -> None:
        """Evaluate every available quantity once at ``z``: each checks its own shape (a JAX evaluator compiles it)."""
        self.objective(z)
        self.equality_residual(z)
        self.inequality_margin(z)
        if self.has_gradient:
            self.gradient(z)
        if self.has_hessian:
            self.hessian(z)
        if self.has_jacobian_h:
            self.jacobian_h(z)
        if self.has_jacobian_g:
            self.jacobian_g(z)

    def decision_vector(self, z) -> np.ndarray:
        """``z`` as a flat float array of shape ``(n_z,)``."""
        arr = np.asarray(z, dtype=float).reshape(-1)
        if arr.shape != (self.n_z,):
            raise ValueError(f"decision vector must have shape ({self.n_z},)")
        return arr

    def jax_decision_vector(self, z):
        """``z`` as a flat JAX array of shape ``(n_z,)``, on a JAX evaluator's ``jnp``."""
        arr = self.jnp.asarray(z).reshape(-1)
        if arr.shape != (self.n_z,):
            raise ValueError(f"decision vector must have shape ({self.n_z},)")
        return arr

    def as_scalar(self, value, label: str) -> float:
        """A native one-element expression as a Python ``float``."""
        arr = np.asarray(value, dtype=float)
        if arr.size != 1:
            raise ValueError(f"{label} must return a scalar")
        return float(arr.reshape(-1)[0])

    def as_vector(self, value, n_expected: int, label: str) -> np.ndarray:
        """A native array as a flat float NumPy vector of ``n_expected`` entries."""
        arr = np.asarray(value, dtype=float).reshape(-1)
        if arr.shape != (n_expected,):
            raise ValueError(f"{label} must have shape ({n_expected},)")
        return arr

    def as_matrix(self, value, shape: tuple[int, int], label: str) -> np.ndarray:
        """A native array as a float NumPy matrix of the given ``shape``."""
        arr = np.asarray(value, dtype=float)
        if arr.shape != shape:
            raise ValueError(f"{label} must have shape {shape}")
        return arr
