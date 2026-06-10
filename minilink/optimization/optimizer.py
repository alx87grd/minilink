"""
Orchestrator for finite-dimensional mathematical-program solvers.

:class:`Optimizer` is a bound solver object: it owns one
:class:`~minilink.optimization.mathematical_program.MathematicalProgram`, a
compiled program evaluator, a default initial guess ``z0``, and a
solver method preset. Calling :meth:`solve` runs the already-compiled program
from either the default ``z0`` or a one-off override.

Typical use::

    program = MathematicalProgram(n_z=2, J=J, grad_J=grad_J)
    opt = Optimizer(program, z0=[0.0, 0.0], method="scipy_slsqp")
    result = opt.solve(disp=True)

Optional progress hook: ``callback(z, J, t)`` on :meth:`Optimizer.solve` with
``t`` = elapsed wall seconds; see :data:`OptimizationProgressCallback`.
"""

import time
from collections.abc import Callable
from dataclasses import replace

import numpy as np

from minilink.optimization.evaluators.compiler import compile_program_evaluator
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizers.optimizer_backend import (
    BackendIterateCallback,
    OptimizerBackend,
)
from minilink.optimization.reporting import (
    DISP_RULE_DIV,
    DISP_RULE_MAIN,
    preview_vector,
)

# User progress hook: current ``z``, objective ``J``, elapsed wall time ``t`` (seconds since solve start).
OptimizationProgressCallback = Callable[[np.ndarray, float, float], None]

# User-facing optimizer methods mapped to (backend_key, default_constructor_kwargs).
_USER_OPTIMIZER_METHODS: dict[str, tuple[str, dict]] = {
    "scipy_slsqp": ("scipy_minimize", {"scipy_method": "SLSQP", "options": {}}),
    "scipy_trust_constr": (
        "scipy_minimize",
        {"scipy_method": "trust-constr", "options": {}},
    ),
    "ipopt": ("ipopt", {"options": {}, "tol": 1e-8}),
}


class Optimizer:
    """
    Bound optimizer for one mathematical program.

    Parameters
    ----------
    program : MathematicalProgram
        Pure NLP description to compile and solve.
    z0 : array_like
        Default initial decision vector for :meth:`solve`.
    method : str, optional
        Optimizer method preset. One of ``"scipy_slsqp"``,
        ``"scipy_trust_constr"``, or ``"ipopt"``.
    compile_backend : str, optional
        Mathematical-program evaluator backend, ``"numpy"`` or ``"jax"``.
        Defaults to ``program.backend`` (the native backend of the program's
        callables).
    use_hessian : bool, optional
        For JAX program evaluators, auto-generate a dense objective Hessian
        when no ``hess_J`` is supplied.
    options : dict, optional
        Solver options merged into the preset's ``options`` dict (user keys win).
    **method_kwargs
        Extra keyword arguments override the preset's top-level kwargs and are
        forwarded to the backend constructor.
    """

    def __init__(
        self,
        program: MathematicalProgram,
        z0,
        *,
        method: str = "scipy_slsqp",
        compile_backend: str | None = None,
        use_hessian: bool = False,
        options: dict | None = None,
        **method_kwargs,
    ):
        if method not in _USER_OPTIMIZER_METHODS:
            valid = ", ".join(sorted(_USER_OPTIMIZER_METHODS))
            raise ValueError(
                f"Unknown optimizer method {method!r}. Expected one of: {valid}."
            )

        self.program = program
        self.z0 = self._decision_vector(z0, program.n_z)
        self.method = method
        self.compile_backend = (
            program.backend if compile_backend is None else compile_backend
        )
        self.program_evaluator = compile_program_evaluator(
            program,
            backend=self.compile_backend,
            sample_z=self.z0,
            use_hessian=use_hessian,
        )

        backend_key, preset = _USER_OPTIMIZER_METHODS[method]
        kwargs = {}
        for key, value in preset.items():
            kwargs[key] = dict(value) if isinstance(value, dict) else value
        kwargs.update(method_kwargs)

        if options is not None:
            existing = kwargs.get("options", {})
            kwargs["options"] = {**existing, **options}

        self.backend = self._select_backend(backend_key, kwargs)

    @staticmethod
    def _select_backend(backend_key: str, kwargs: dict) -> OptimizerBackend:

        if backend_key == "scipy_minimize":
            from minilink.optimization.optimizers.scipy_minimize import (
                ScipyMinimizeOptimizer,
            )

            return ScipyMinimizeOptimizer(**kwargs)

        if backend_key == "ipopt":
            try:
                import cyipopt  # noqa: F401
            except ImportError:
                raise ImportError(
                    "IpoptOptimizer requires the optional 'cyipopt' package."
                )

            from minilink.optimization.optimizers.ipopt import IpoptOptimizer

            return IpoptOptimizer(**kwargs)

        raise ValueError(f"Unknown optimizer backend key {backend_key!r}.")

    def solve(
        self,
        *,
        z0=None,
        callback: OptimizationProgressCallback | None = None,
        record_solve_time: bool = False,
        disp: bool = False,
    ) -> OptimizationResult:
        """
        Solve the bound finite-dimensional mathematical program.

        Parameters
        ----------
        z0 : array_like, optional
            One-off initial decision vector. If omitted, use the default ``z0``
            supplied to :class:`Optimizer`.
        record_solve_time : bool, optional
            If True, set
            :attr:`~minilink.optimization.mathematical_program.OptimizationResult.solve_time_s`
            to the wall-clock duration (seconds) of the backend solve only,
            measured with :func:`time.perf_counter`.
        callback : callable, optional
            If not ``None``, called during the solve as ``callback(z, J, t)``:
            current decision ``z`` (1-D :class:`~numpy.ndarray`), objective
            :math:`J(z)`, and ``t`` elapsed wall time (seconds) since the
            backend solve started. Each invocation evaluates ``J(z)`` again
            (one extra objective call per iterate). Not supported by the Ipopt
            backend (native path); use SciPy or omit.
        disp : bool, optional
            If True, print one framed ``disp`` panel.
        """
        z_start = self.z0 if z0 is None else self._decision_vector(z0, self.program.n_z)
        time_solve = record_solve_time or disp

        if disp:
            self._print_solve_preamble(z_start)
        if time_solve:
            t0 = time.perf_counter()

        backend_cb: BackendIterateCallback | None = None
        if callback is not None:
            t_cb0 = time.perf_counter()

            def backend_cb(z: np.ndarray) -> None:
                z_arr = self._decision_vector(z, self.program.n_z)
                J = self.program_evaluator.objective(z_arr)
                t_elapsed = time.perf_counter() - t_cb0
                callback(z_arr.copy(), J, t_elapsed)

        ##############################################################
        # --- Backend solve ---
        result = self.backend.solve(
            self.program_evaluator,
            z_start,
            callback=backend_cb,
        )
        ##############################################################

        if time_solve:
            elapsed = time.perf_counter() - t0
            result = replace(result, solve_time_s=elapsed)

        if disp:
            self._print_solve_report(result)

        return result

    @staticmethod
    def _decision_vector(z, n_z: int) -> np.ndarray:
        arr = np.asarray(z, dtype=float).reshape(-1)
        if arr.shape != (int(n_z),):
            raise ValueError(f"decision vector must have shape ({int(n_z)},)")
        return arr

    def _print_solve_preamble(self, z0: np.ndarray) -> None:
        print()
        print(DISP_RULE_MAIN)
        print("===               Optimization Program                   ===")
        print(DISP_RULE_MAIN)
        print("n_z:", int(self.program.n_z))
        print("z0:", preview_vector(z0))
        print(f"method={self.method!r}")
        print(f"compile_backend={self.program_evaluator.backend!r}")
        print("options:", getattr(self.backend, "options", {}))
        print(DISP_RULE_DIV)
        print("Running solver...")

    def _print_solve_report(
        self,
        result: OptimizationResult,
    ) -> None:
        """Print the **After solve** section and close the ``disp`` panel."""
        print("Completed in", result.solve_time_s, "seconds")
        print(DISP_RULE_DIV)
        print("success:", result.success)
        print("z*:", preview_vector(result.z))
        print("J*:", result.cost)
        print("stats:", result.stats)
        print(DISP_RULE_MAIN)


if __name__ == "__main__":
    from minilink.optimization.mathematical_program import MathematicalProgram

    z_lo = 0.6
    z_hi = 2.05

    def J(z: np.ndarray):
        y = z**3 + z * z + np.sin(5.0 * np.pi * z) + 0.12 * np.sin(15.0 * np.pi * z)
        return y

    def g(z: np.ndarray) -> np.ndarray:
        return np.array([z[0] - z_lo, z_hi - z[0]], dtype=float)

    prog = MathematicalProgram(
        n_z=1,
        J=J,
        g=g,
    )

    opt = Optimizer(
        prog,
        z0=np.array([0.0]),
        method="scipy_slsqp",
        options={
            "disp": False,
            "maxiter": 200,
            "ftol": 1e-12,
        },
    )

    def callback(z: np.ndarray, J: float, t: float) -> None:
        print(f"z: {z}, J: {J}, t: {t}")

    out = opt.solve(
        callback=callback,
        disp=True,
    )
    print(out)
