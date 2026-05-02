"""JAX program evaluator for finite-dimensional mathematical programs."""

import numpy as np

from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
from minilink.optimization.mathematical_program import MathematicalProgram


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

        try:
            import jax
            import jax.numpy as jnp
        except ImportError as exc:  # pragma: no cover - optional dependency
            raise ImportError(
                "JAX is required for mathematical-program backend='jax'. "
                "Install with `pip install minilink[jax]` or `pip install jax jaxlib`."
            ) from exc

        self.jax = jax
        self.jnp = jnp
        self.backend = "jax"

        z_sample = jnp.asarray(self.sample_z)

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

        self._check_traceable(J_raw, z_sample, "J")
        if program.h is not None:
            self._check_traceable(h_raw, z_sample, "h")
        if program.g is not None:
            self._check_traceable(g_raw, z_sample, "g")

        self._jit_J = jax.jit(J_raw)
        self._jit_h = jax.jit(h_raw)
        self._jit_g = jax.jit(g_raw)

        self._jit_grad_J = self._compile_vector_derivative(
            program.grad_J,
            fallback=jax.grad(J_raw),
        )
        self._jit_hess_J = None
        if program.hess_J is not None:
            self._jit_hess_J = jax.jit(program.hess_J)
        elif use_hessian:
            self._jit_hess_J = jax.jit(jax.hessian(J_raw))

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

        self.n_h = int(np.asarray(self._jit_h(z_sample)).reshape(-1).size)
        self.n_g = int(np.asarray(self._jit_g(z_sample)).reshape(-1).size)
        self._check_derivative_shapes(z_sample)

    def J(self, z):
        return self._jit_J(self._jax_decision_vector(z))

    def h(self, z):
        return self._jit_h(self._jax_decision_vector(z))

    def g(self, z):
        return self._jit_g(self._jax_decision_vector(z))

    @property
    def has_gradient(self) -> bool:
        return self._jit_grad_J is not None

    @property
    def has_hessian(self) -> bool:
        return self._jit_hess_J is not None

    @property
    def has_jacobian_h(self) -> bool:
        return self.n_h == 0 or self._jit_jac_h is not None

    @property
    def has_jacobian_g(self) -> bool:
        return self.n_g == 0 or self._jit_jac_g is not None

    def gradient(self, z) -> np.ndarray:
        if self._jit_grad_J is None:
            return super().gradient(z)
        value = self._jit_grad_J(self._jax_decision_vector(z))
        return self._vector(value, self.n_z, "objective gradient")

    def hessian(self, z) -> np.ndarray:
        if self._jit_hess_J is None:
            return super().hessian(z)
        value = self._jit_hess_J(self._jax_decision_vector(z))
        return self._matrix(value, (self.n_z, self.n_z), "objective Hessian")

    def jacobian_h(self, z) -> np.ndarray:
        if self.n_h == 0:
            return super().jacobian_h(z)
        if self._jit_jac_h is None:
            return super().jacobian_h(z)
        value = self._jit_jac_h(self._jax_decision_vector(z))
        return self._matrix(value, (self.n_h, self.n_z), "equality Jacobian")

    def jacobian_g(self, z) -> np.ndarray:
        if self.n_g == 0:
            return super().jacobian_g(z)
        if self._jit_jac_g is None:
            return super().jacobian_g(z)
        value = self._jit_jac_g(self._jax_decision_vector(z))
        return self._matrix(value, (self.n_g, self.n_z), "inequality Jacobian")

    def _jax_decision_vector(self, z):
        arr = self.jnp.asarray(z).reshape(-1)
        if arr.shape != (self.n_z,):
            raise ValueError(f"decision vector must have shape ({self.n_z},)")
        return arr

    def _compile_vector_derivative(self, provided, *, fallback):
        if provided is not None:
            return self.jax.jit(provided)
        return self.jax.jit(fallback)

    def _check_traceable(self, fn, z_sample, label: str) -> None:
        try:
            self.jax.make_jaxpr(fn)(z_sample)
        except Exception as exc:
            raise RuntimeError(
                f"MathematicalProgram function {label!r} is not JAX-traceable. "
                "Use backend='numpy' or rewrite the callable with jax.numpy."
            ) from exc

    def _check_derivative_shapes(self, z_sample) -> None:
        self.objective(z_sample)
        self.equality_residual(z_sample)
        self.inequality_margin(z_sample)
        if self.has_gradient:
            self.gradient(z_sample)
        if self.has_hessian:
            self.hessian(z_sample)
        if self.has_jacobian_h:
            self.jacobian_h(z_sample)
        if self.has_jacobian_g:
            self.jacobian_g(z_sample)
