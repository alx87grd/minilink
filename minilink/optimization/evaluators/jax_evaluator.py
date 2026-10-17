"""The JAX program evaluator: a mathematical program JIT-compiled, its missing derivatives by autodiff."""

import numpy as np

from minilink.core.backends import ensure_jax_x64, require_jax, require_jax_numpy
from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
from minilink.optimization.mathematical_program import MathematicalProgram

###############################################################################
# Public API
###############################################################################


class JaxMathematicalProgramEvaluator(MathematicalProgramEvaluator):
    """
    JIT-compiled program evaluator for :class:`MathematicalProgram`.

    JAX is imported lazily. The program evaluator JIT-compiles ``J`` / ``h`` /
    ``g``, auto-generates ``grad_J`` and constraint Jacobians when not supplied,
    and only auto-generates a dense objective Hessian when ``use_hessian=True``.
    """

    def __init__(
        self,
        program: MathematicalProgram,
        sample_z=None,
        *,
        use_hessian: bool = False,
        verbose: bool = False,
    ):
        super().__init__(program, sample_z=sample_z)

        # JAX, an optional dependency; the float64 policy is applied at every
        # construction, not only at the first import
        try:
            jax = require_jax()
            jnp = require_jax_numpy()
            ensure_jax_x64()
        except ImportError as exc:  # pragma: no cover - optional dependency
            raise ImportError(
                "JAX is required for mathematical-program backend='jax'. "
                "Install with `pip install jax jaxlib`."
            ) from exc

        self.jax = jax
        self.jnp = jnp
        self.backend = "jax"

        # The sample point every function is traced and compiled at
        z_sample = jnp.asarray(self.sample_z)

        # The raw functions: J(z), and h(z) and g(z) flattened
        def J_raw(z):
            return program.J(z)

        def h_raw(z):
            if program.h is None:
                return jnp.zeros(0, dtype=z.dtype)
            return jnp.asarray(program.h(z)).reshape(-1)

        def g_raw(z):
            if program.g is None:
                return jnp.zeros(0, dtype=z.dtype)
            return jnp.asarray(program.g(z)).reshape(-1)

        # Each one must trace under JAX
        self.require_traceable(J_raw, z_sample, "J")
        if program.h is not None:
            self.require_traceable(h_raw, z_sample, "h")
        if program.g is not None:
            self.require_traceable(g_raw, z_sample, "g")

        # The compiled functions
        self._jit_J = jax.jit(J_raw)
        self._jit_h = jax.jit(h_raw)
        self._jit_g = jax.jit(g_raw)

        # ∇J: the program's own, else by reverse-mode autodiff
        if program.grad_J is not None:
            self._jit_grad_J = jax.jit(program.grad_J)
        else:
            self._jit_grad_J = jax.jit(jax.grad(J_raw))

        # ∇²J: the program's own, else by autodiff only when use_hessian asks for it
        self._jit_hess_J = None
        if program.hess_J is not None:
            self._jit_hess_J = jax.jit(program.hess_J)
        elif use_hessian:
            self._jit_hess_J = jax.jit(jax.hessian(J_raw))

        # ∂h/∂z and ∂g/∂z: the program's own, else by forward-mode autodiff
        self._jit_jac_h = None
        if program.jac_h is not None:
            self._jit_jac_h = jax.jit(program.jac_h)
        elif program.h is not None:
            self._jit_jac_h = jax.jit(jax.jacfwd(h_raw))

        self._jit_jac_g = None
        if program.jac_g is not None:
            self._jit_jac_g = jax.jit(program.jac_g)
        elif program.g is not None:
            self._jit_jac_g = jax.jit(jax.jacfwd(g_raw))

        if verbose:
            print("[compile] Warm-starting JAX mathematical-program evaluator.")

        # The constraint sizes, then every available quantity once at the sample
        self.n_h = int(np.asarray(self._jit_h(z_sample)).reshape(-1).size)
        self.n_g = int(np.asarray(self._jit_g(z_sample)).reshape(-1).size)
        self.check_derivative_shapes(z_sample)

    @property
    def has_gradient(self) -> bool:
        return True

    @property
    def has_hessian(self) -> bool:
        return self._jit_hess_J is not None

    @property
    def has_jacobian_h(self) -> bool:
        return self.n_h == 0 or self._jit_jac_h is not None

    @property
    def has_jacobian_g(self) -> bool:
        return self.n_g == 0 or self._jit_jac_g is not None

    # The compiled functions, on a checked decision vector

    def J(self, z):
        return self._jit_J(self.jax_decision_vector(z))

    def h(self, z):
        return self._jit_h(self.jax_decision_vector(z))

    def g(self, z):
        return self._jit_g(self.jax_decision_vector(z))

    # The compiled derivatives, as flat NumPy arrays

    def gradient(self, z) -> np.ndarray:
        value = self._jit_grad_J(self.jax_decision_vector(z))
        return self.as_vector(value, self.n_z, "objective gradient")

    def hessian(self, z) -> np.ndarray:
        if self._jit_hess_J is None:
            return super().hessian(z)
        value = self._jit_hess_J(self.jax_decision_vector(z))
        return self.as_matrix(value, (self.n_z, self.n_z), "objective Hessian")

    def jacobian_h(self, z) -> np.ndarray:
        if self.n_h == 0 or self._jit_jac_h is None:
            return super().jacobian_h(z)
        value = self._jit_jac_h(self.jax_decision_vector(z))
        return self.as_matrix(value, (self.n_h, self.n_z), "equality Jacobian")

    def jacobian_g(self, z) -> np.ndarray:
        if self.n_g == 0 or self._jit_jac_g is None:
            return super().jacobian_g(z)
        value = self._jit_jac_g(self.jax_decision_vector(z))
        return self.as_matrix(value, (self.n_g, self.n_z), "inequality Jacobian")

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_traceable(self, fn, z_sample, label: str) -> None:
        """Refuse a function that does not trace under JAX at ``z_sample`` (a NumPy-only callable)."""
        try:
            self.jax.make_jaxpr(fn)(z_sample)
        except Exception as exc:
            raise RuntimeError(
                f"MathematicalProgram function {label!r} is not JAX-traceable. "
                "Use backend='numpy' or rewrite the callable with jax.numpy."
            ) from exc
