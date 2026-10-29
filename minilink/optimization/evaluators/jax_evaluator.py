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
    Each function has two compiled forms: the frozen one, the defaults
    constants of the trace (the faster run of one fixed program, an offline
    solve), used while the defaults are bound; the parametric one, ``(z,
    params)`` with the bound tree a traced argument, used after a
    :meth:`bind` of values, so a new value never recompiles. The derivatives
    are with respect to ``z``.
    """

    def __init__(
        self,
        program: MathematicalProgram,
        sample_z=None,
        *,
        use_hessian: bool = False,
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
                "JAX is required for mathematical-program backend='jax'. "
                "Install with `pip install jax jaxlib`."
            ) from exc

        self.jax = jax
        self.jnp = jnp
        super().__init__(program, sample_z=sample_z)
        self.backend = "jax"

        # The sample point every function is traced and compiled at
        z_sample = jnp.asarray(self.sample_z)
        call = self.raw_call

        # The raw functions of (z, params): J, and h and g flattened
        def J_raw(z, p):
            return call(program.J, z, p)

        def h_raw(z, p):
            if program.h is None:
                return jnp.zeros(0, dtype=z.dtype)
            return jnp.asarray(call(program.h, z, p)).reshape(-1)

        def g_raw(z, p):
            if program.g is None:
                return jnp.zeros(0, dtype=z.dtype)
            return jnp.asarray(call(program.g, z, p)).reshape(-1)

        def own(function):
            """A derivative the program supplies, as a function of ``(z, params)``."""
            return lambda z, p: call(function, z, p)

        # Each one must trace under JAX
        self.require_traceable(J_raw, z_sample, "J")
        if program.h is not None:
            self.require_traceable(h_raw, z_sample, "h")
        if program.g is not None:
            self.require_traceable(g_raw, z_sample, "g")

        # The raw functions of (z, params), each one's frozen form compiled on first use
        self.raw_forms = {"J": J_raw, "h": h_raw, "g": g_raw}
        self.frozen_forms = {}
        self.default_tree = self.params

        # The compiled functions
        self._jit_J = jax.jit(J_raw)
        self._jit_h = jax.jit(h_raw)
        self._jit_g = jax.jit(g_raw)

        # ∇J: the program's own, else by reverse-mode autodiff in z
        if program.grad_J is not None:
            self.raw_forms["grad_J"] = own(program.grad_J)
        else:
            self.raw_forms["grad_J"] = jax.grad(J_raw)
        self._jit_grad_J = jax.jit(self.raw_forms["grad_J"])

        # ∇²J: the program's own, else by autodiff only when use_hessian asks for it
        self._jit_hess_J = None
        if program.hess_J is not None:
            self.raw_forms["hess_J"] = own(program.hess_J)
        elif use_hessian:
            self.raw_forms["hess_J"] = jax.hessian(J_raw)
        if "hess_J" in self.raw_forms:
            self._jit_hess_J = jax.jit(self.raw_forms["hess_J"])

        # ∂h/∂z and ∂g/∂z: the program's own, else by forward-mode autodiff in z
        self._jit_jac_h = None
        if program.jac_h is not None:
            self.raw_forms["jac_h"] = own(program.jac_h)
        elif program.h is not None:
            self.raw_forms["jac_h"] = jax.jacfwd(h_raw)
        if "jac_h" in self.raw_forms:
            self._jit_jac_h = jax.jit(self.raw_forms["jac_h"])

        self._jit_jac_g = None
        if program.jac_g is not None:
            self.raw_forms["jac_g"] = own(program.jac_g)
        elif program.g is not None:
            self.raw_forms["jac_g"] = jax.jacfwd(g_raw)
        if "jac_g" in self.raw_forms:
            self._jit_jac_g = jax.jit(self.raw_forms["jac_g"])

        if verbose:
            print("[compile] Warm-starting JAX mathematical-program evaluator.")

        # The constraint sizes, then every available quantity once at the sample
        self.n_h = int(np.asarray(self.form("h")(z_sample)).reshape(-1).size)
        self.n_g = int(np.asarray(self.form("g")(z_sample)).reshape(-1).size)
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
        return self.form("J")(self.jax_decision_vector(z))

    def h(self, z):
        return self.form("h")(self.jax_decision_vector(z))

    def g(self, z):
        return self.form("g")(self.jax_decision_vector(z))

    # The oracle of a traced solve: the compiled functions on JAX arrays, no conversion

    def native_objective(self, z, params):
        return self._jit_J(z, params)

    def native_gradient(self, z, params):
        return self._jit_grad_J(z, params)

    # The compiled derivatives, as flat NumPy arrays

    def gradient(self, z) -> np.ndarray:
        value = self.form("grad_J")(self.jax_decision_vector(z))
        return self.as_vector(value, self.n_z, "objective gradient")

    def hessian(self, z) -> np.ndarray:
        if self._jit_hess_J is None:
            return super().hessian(z)
        value = self.form("hess_J")(self.jax_decision_vector(z))
        return self.as_matrix(value, (self.n_z, self.n_z), "objective Hessian")

    def jacobian_h(self, z) -> np.ndarray:
        if self.n_h == 0 or self._jit_jac_h is None:
            return super().jacobian_h(z)
        value = self.form("jac_h")(self.jax_decision_vector(z))
        return self.as_matrix(value, (self.n_h, self.n_z), "equality Jacobian")

    def jacobian_g(self, z) -> np.ndarray:
        if self.n_g == 0 or self._jit_jac_g is None:
            return super().jacobian_g(z)
        value = self.form("jac_g")(self.jax_decision_vector(z))
        return self.as_matrix(value, (self.n_g, self.n_z), "inequality Jacobian")

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def form(self, name):
        """The compiled ``name`` for the bound tree, a function of ``z``: frozen while the defaults are bound, else parametric."""
        if not self.frozen:
            parametric = getattr(self, f"_jit_{name}")
            return lambda z: parametric(z, self.params)

        # The defaults constants of the trace, compiled on first use
        if name not in self.frozen_forms:
            raw = self.raw_forms[name]
            tree = self.default_tree
            self.frozen_forms[name] = self.jax.jit(lambda z: raw(z, tree))

        return self.frozen_forms[name]

    def raw_call(self, function, z, params):
        """A program callable on traced ``(z, params)``: ``function(z)`` for a program without parameters."""
        if self.program.params is None:
            return function(z)

        return function(z, params)

    def native_tree(self, tree):
        """The bound tree with every numeric leaf a JAX array (``None`` kept): a traced argument of the compiled functions."""
        jnp = self.jnp
        return self.jax.tree.map(jnp.asarray, tree)

    def require_traceable(self, fn, z_sample, label: str) -> None:
        """Refuse a function that does not trace under JAX at ``z_sample`` (a NumPy-only callable)."""
        try:
            self.jax.make_jaxpr(fn)(z_sample, self.params)
        except Exception as exc:
            raise RuntimeError(
                f"MathematicalProgram function {label!r} is not JAX-traceable. "
                "Use backend='numpy' or rewrite the callable with jax.numpy."
            ) from exc
