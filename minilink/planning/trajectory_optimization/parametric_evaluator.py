"""The parametric evaluator: a JAX-compiled program whose equalities read a bound initial state."""

import numpy as np

from minilink.core.backends import ensure_jax_x64, require_jax, require_jax_numpy
from minilink.planning.trajectory_optimization.parametric_program import (
    ParametricMathematicalProgram,
)

###############################################################################
# Public API
###############################################################################


class JaxParametricProgramEvaluator:
    """
    JIT-compiled evaluator for :class:`ParametricMathematicalProgram`.

    Equality constraints depend on both ``z`` and a bound runtime parameter
    ``x0``. Call :meth:`bind` before each solve to set the current initial
    state. The SciPy-facing surface matches
    :class:`~minilink.optimization.evaluators.program_evaluator.MathematicalProgramEvaluator`.
    """

    def __init__(
        self,
        program: ParametricMathematicalProgram,
        sample_z=None,
        *,
        sample_x0=None,
        verbose: bool = False,
    ):
        # JAX, an optional dependency; the float64 policy is applied at every
        # construction, not only at the first import
        try:
            jax = require_jax()
            jnp = require_jax_numpy()
            ensure_jax_x64()
        except ImportError as exc:  # pragma: no cover - optional dependency
            raise ImportError(
                "JAX is required for parametric trajectory optimization. "
                "Install with `pip install jax jaxlib`."
            ) from exc

        # The program and its sizes
        self.jax = jax
        self.jnp = jnp
        self.program = program
        self.backend = "jax"
        self.n_z = int(program.n_z)
        self.n_x0 = int(program.n_x0)

        # The sample point every function is traced and compiled at
        self.sample_z = self.decision_vector(
            np.zeros(self.n_z) if sample_z is None else sample_z
        )
        self._x0 = jnp.asarray(
            np.zeros(self.n_x0) if sample_x0 is None else sample_x0,
            dtype=jnp.float64,
        )

        z_sample = jnp.asarray(self.sample_z)
        x0_sample = self._x0

        # The raw functions: J(z), h(z, x0) and g(z), the constraints flattened
        def J_raw(z):
            return program.J(z)

        def h_raw(z, x0):
            return jnp.asarray(program.h(z, x0)).reshape(-1)

        def g_raw(z):
            if program.g is None:
                return jnp.zeros(0, dtype=z.dtype)
            return jnp.asarray(program.g(z)).reshape(-1)

        # Each one must trace under JAX
        self.check_traceable(J_raw, z_sample, "J")
        self.check_traceable(
            lambda z: h_raw(z, x0_sample),
            z_sample,
            "h",
        )
        if program.g is not None:
            self.check_traceable(g_raw, z_sample, "g")

        # The compiled functions and their derivatives ∇J, ∂h/∂z and ∂g/∂z
        self._jit_J = jax.jit(J_raw)
        self._jit_h = jax.jit(h_raw)
        self._jit_g = jax.jit(g_raw)
        self._jit_grad_J = jax.jit(jax.grad(J_raw))
        # TODO: Structural scale opportunity. Standard jacfwd dense allocation here is an O(N^2)
        # bottleneck for long horizons. Since collocation defects are block-diagonal, consider
        # using jax.vmap(jacfwd) on a single step and populating a scipy.sparse.csc_matrix or
        # JAX BCOO to avoid massive dense zero-allocations.
        self._jit_jac_h = jax.jit(jax.jacfwd(h_raw, argnums=0))
        if program.g is not None:
            self._jit_jac_g = jax.jit(jax.jacfwd(g_raw))
        else:
            self._jit_jac_g = None

        if verbose:
            print("[compile] Warm-starting JAX parametric program evaluator.")

        # The constraint sizes, then every quantity once at the samples
        self.n_h = int(np.asarray(self._jit_h(z_sample, x0_sample)).reshape(-1).size)
        self.n_g = int(np.asarray(self._jit_g(z_sample)).reshape(-1).size)
        self.check_derivative_shapes(z_sample, x0_sample)

    @property
    def has_gradient(self) -> bool:
        return True

    @property
    def has_hessian(self) -> bool:
        return False

    @property
    def has_jacobian_h(self) -> bool:
        return self.n_h == 0 or self._jit_jac_h is not None

    @property
    def has_jacobian_g(self) -> bool:
        return self.n_g == 0 or self._jit_jac_g is not None

    def bind(self, x0) -> None:
        """Set the runtime initial-state parameter for the next solve."""
        arr = np.asarray(x0, dtype=float).reshape(-1)
        if arr.shape != (self.n_x0,):
            raise ValueError(f"x0 must have shape ({self.n_x0},)")
        self._x0 = self.jnp.asarray(arr)

    # The program's functions, on the bound initial state

    def J(self, z):
        return self._jit_J(self.jax_decision_vector(z))

    def h(self, z):
        return self._jit_h(self.jax_decision_vector(z), self._x0)

    def g(self, z):
        return self._jit_g(self.jax_decision_vector(z))

    # The SciPy-facing surface: floats and flat NumPy arrays

    def objective(self, z) -> float:
        return self.as_scalar(self.J(z), "objective")

    def equality_residual(self, z) -> np.ndarray:
        return self.as_vector(self.h(z), self.n_h, "equality residual")

    def inequality_margin(self, z) -> np.ndarray:
        return self.as_vector(self.g(z), self.n_g, "inequality margin")

    def gradient(self, z) -> np.ndarray:
        value = self._jit_grad_J(self.jax_decision_vector(z))
        return self.as_vector(value, self.n_z, "objective gradient")

    def jacobian_h(self, z) -> np.ndarray:
        if self.n_h == 0:
            return np.zeros((0, self.n_z), dtype=float)
        value = self._jit_jac_h(self.jax_decision_vector(z), self._x0)
        return self.as_matrix(value, (self.n_h, self.n_z), "equality Jacobian")

    def jacobian_g(self, z) -> np.ndarray:
        if self.n_g == 0:
            return np.zeros((0, self.n_z), dtype=float)
        if self._jit_jac_g is None:
            raise ValueError("This parametric evaluator has no inequality Jacobian")
        value = self._jit_jac_g(self.jax_decision_vector(z))
        return self.as_matrix(value, (self.n_g, self.n_z), "inequality Jacobian")

    def constraint_violations(self, z) -> tuple[float, float | None, float]:
        """The largest equality residual, the smallest inequality margin and the largest bound violation."""
        eq_inf = 0.0
        if self.n_h:
            eq_inf = float(np.max(np.abs(self.equality_residual(z))))

        min_ineq: float | None = None
        if self.n_g:
            min_ineq = float(np.min(self.inequality_margin(z)))

        # The bounds are read on z flattened
        bound_inf = 0.0
        lower = self.program.lower
        upper = self.program.upper
        z_arr = np.asarray(z, dtype=float).reshape(-1)
        if lower is not None:
            bound_inf = max(bound_inf, float(np.max(np.maximum(lower - z_arr, 0.0))))
        if upper is not None:
            bound_inf = max(bound_inf, float(np.max(np.maximum(z_arr - upper, 0.0))))
        return eq_inf, min_ineq, bound_inf

    def scipy_bounds(self):
        """SciPy ``minimize`` bounds, or ``None`` when the program sets none."""
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

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def decision_vector(self, z) -> np.ndarray:
        """``z`` as a flat float NumPy array of shape ``(n_z,)``."""
        arr = np.asarray(z, dtype=float).reshape(-1)
        if arr.shape != (self.n_z,):
            raise ValueError(f"decision vector must have shape ({self.n_z},)")
        return arr

    def check_traceable(self, fn, z_sample, label: str) -> None:
        """Trace ``fn`` once at ``z_sample``; a NumPy-only callable raises here."""
        try:
            self.jax.make_jaxpr(fn)(z_sample)
        except Exception as exc:
            raise RuntimeError(
                f"ParametricMathematicalProgram function {label!r} is not "
                f"JAX-traceable: {exc}"
            ) from exc

    def check_derivative_shapes(self, z_sample, x0_sample) -> None:
        """Evaluate every quantity once at the samples: compiles each and checks its shape."""
        self.objective(z_sample)
        self.equality_residual(z_sample)
        self.inequality_margin(z_sample)
        self.gradient(z_sample)
        if self.has_jacobian_h:
            self.jacobian_h(z_sample)
        if self.has_jacobian_g:
            self.jacobian_g(z_sample)

    def jax_decision_vector(self, z):
        """``z`` as a flat JAX array of shape ``(n_z,)``."""
        arr = self.jnp.asarray(z).reshape(-1)
        if arr.shape != (self.n_z,):
            raise ValueError(f"decision vector must have shape ({self.n_z},)")
        return arr

    def as_scalar(self, value, label: str) -> float:
        """``value`` as a Python float; refuses anything but one element."""
        arr = np.asarray(value, dtype=float)
        if arr.size != 1:
            raise ValueError(f"{label} must return a scalar")
        return float(arr.reshape(-1)[0])

    def as_vector(self, value, n_expected: int, label: str) -> np.ndarray:
        """``value`` as a flat float NumPy array of shape ``(n_expected,)``."""
        arr = np.asarray(value, dtype=float).reshape(-1)
        if arr.shape != (n_expected,):
            raise ValueError(f"{label} must have shape ({n_expected},)")
        return arr

    def as_matrix(self, value, shape: tuple[int, ...], label: str) -> np.ndarray:
        """``value`` as a float NumPy array of exactly ``shape``."""
        arr = np.asarray(value, dtype=float)
        if arr.shape != shape:
            raise ValueError(f"{label} must have shape {shape}")
        return arr
