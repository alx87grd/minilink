"""The SciPy backend: a compiled program solved by :func:`scipy.optimize.minimize`."""

import numpy as np
from scipy.optimize import minimize

from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
from minilink.optimization.mathematical_program import OptimizationResult
from minilink.optimization.optimizers.optimizer_backend import (
    BackendIterateCallback,
    OptimizerBackend,
    scipy_constraints_of,
)

###############################################################################
# Public API
###############################################################################


class ScipyMinimizeOptimizer(OptimizerBackend):
    """
    Optimizer adapter for :func:`scipy.optimize.minimize`.

    Equality constraints are passed as ``type='eq'`` residuals and inequality
    constraints as ``type='ineq'`` nonnegative margins, matching Minilink's
    mathematical-program convention.
    """

    def __init__(self, scipy_method="SLSQP", options=None):
        # The SciPy method, and its solver options, copied
        self.scipy_method = scipy_method
        self.options = {} if options is None else dict(options)

    def solve(
        self,
        program_evaluator: MathematicalProgramEvaluator,
        z0: np.ndarray,
        *,
        callback: BackendIterateCallback | None = None,
    ) -> OptimizationResult:
        """Solve ``program_evaluator`` with SciPy and return a backend-neutral result."""
        # Equalities h(z) = 0 and inequalities g(z) ≥ 0, each with its Jacobian when
        # the evaluator has one
        constraints = scipy_constraints_of(program_evaluator)

        # The user's hook, fed the iterate z in whichever form the method passes it
        scipy_callback = None
        if callback is not None:

            def scipy_callback(xk=None, state=None, *, intermediate_result=None):
                z_step = z_from_scipy_minimize_callback(
                    xk, state, intermediate_result=intermediate_result
                )
                callback(z_step)

        # The solve from z0, inside the box bounds; the Hessian only for a method
        # that uses it
        raw_result = minimize(
            program_evaluator.objective,
            np.asarray(z0, dtype=float).reshape(program_evaluator.n_z),
            method=self.scipy_method,
            jac=program_evaluator.gradient if program_evaluator.has_gradient else None,
            hess=(
                program_evaluator.hessian
                if self.uses_hessian() and program_evaluator.has_hessian
                else None
            ),
            bounds=program_evaluator.scipy_bounds(),
            constraints=constraints,
            callback=scipy_callback,
            options=dict(self.options),
        )

        # The counters and the status the solver reports
        stats = {}
        for name in ("nit", "nfev", "njev", "status"):
            if hasattr(raw_result, name):
                stats[name] = getattr(raw_result, name)

        return OptimizationResult(
            z=np.asarray(raw_result.x, dtype=float),
            success=bool(raw_result.success),
            cost=float(raw_result.fun) if raw_result.fun is not None else None,
            message=str(raw_result.message),
            stats=stats,
            raw_result=raw_result,
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def uses_hessian(self) -> bool:
        """Check if the SciPy method uses the Hessian."""
        method = self.scipy_method.lower()
        return method in {
            "dogleg",
            "trust-ncg",
            "trust-exact",
            "trust-krylov",
            "trust-constr",
        }


###############################################################################
# Internal machinery
###############################################################################


def z_from_scipy_minimize_callback(
    xk=None,
    state=None,
    *,
    intermediate_result=None,
) -> np.ndarray:
    """Recover decision ``z`` from :func:`scipy.optimize.minimize` callback variants."""
    # The first form given: the method's intermediate result, its state, or xk
    for obj in (intermediate_result, state, xk):
        if obj is None:
            continue
        if isinstance(obj, np.ndarray):
            z = np.asarray(obj, dtype=float).reshape(-1).copy()
            return z
        x = getattr(obj, "x", None)
        if x is not None:
            z = np.asarray(x, dtype=float).reshape(-1).copy()
            return z

    msg = (
        "SciPy minimize callback did not provide a usable decision vector "
        "(expected an ndarray or an object with attribute ``x``)."
    )
    raise RuntimeError(msg)
