"""SciPy :func:`scipy.optimize.minimize` optimizer."""

import numpy as np
from scipy.optimize import minimize

from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizers.optimizer_backend import (
    BackendIterateCallback,
    OptimizerBackend,
)


def _z_from_scipy_minimize_callback(
    xk=None,
    state=None,
    *,
    intermediate_result=None,
) -> np.ndarray:
    """Recover decision ``z`` from :func:`scipy.optimize.minimize` callback variants."""
    for obj in (intermediate_result, state, xk):
        if obj is None:
            continue
        if isinstance(obj, np.ndarray):
            return np.asarray(obj, dtype=float).reshape(-1).copy()
        x = getattr(obj, "x", None)
        if x is not None:
            return np.asarray(x, dtype=float).reshape(-1).copy()
    msg = (
        "SciPy minimize callback did not provide a usable decision vector "
        "(expected an ndarray or an object with attribute ``x``)."
    )
    raise RuntimeError(msg)


class ScipyMinimizeOptimizer(OptimizerBackend):
    """
    Optimizer adapter for :func:`scipy.optimize.minimize`.

    Equality constraints are passed as ``type='eq'`` residuals and
    inequality constraints are passed as ``type='ineq'`` nonnegative margins, matching
    the generic :class:`~minilink.optimization.mathematical_program.MathematicalProgram`
    convention.
    """

    def __init__(self, method="SLSQP", options=None):
        self.method = method
        self.options = {} if options is None else dict(options)

    def solve(
        self,
        program: MathematicalProgram,
        *,
        callback: BackendIterateCallback | None = None,
    ) -> OptimizationResult:
        """Solve ``program`` with SciPy and return a backend-neutral result."""

        constraints = []

        # Equality constraints
        for equality in program.equalities:
            entry = {"type": "eq", "fun": equality.residual}
            if equality.jac is not None:
                entry["jac"] = equality.jac
            constraints.append(entry)

        # Inequality constraints
        for inequality in program.inequalities:
            entry = {"type": "ineq", "fun": inequality.margin}
            if inequality.jac is not None:
                entry["jac"] = inequality.jac
            constraints.append(entry)

        # Box bounds
        bounds = None
        if program.bounds is not None:
            lower = (
                np.full(program.n_z, -np.inf)
                if program.bounds.lower is None
                else program.bounds.lower
            )
            upper = (
                np.full(program.n_z, np.inf)
                if program.bounds.upper is None
                else program.bounds.upper
            )
            bounds = list(zip(lower, upper))

        scipy_callback = None
        if callback is not None:

            def scipy_callback(xk=None, state=None, *, intermediate_result=None):
                z_step = _z_from_scipy_minimize_callback(
                    xk, state, intermediate_result=intermediate_result
                )
                callback(z_step)

        raw_result = minimize(
            program.objective,
            program.z0,
            method=self.method,
            jac=program.gradient if program.grad is not None else None,
            hess=program.hessian if self._uses_hessian() and program.hess else None,
            bounds=bounds,
            constraints=constraints,
            callback=scipy_callback,
            options=dict(self.options),
        )

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

    def _uses_hessian(self) -> bool:
        """Check if the method uses the Hessian."""
        method = self.method.lower()
        return method in {
            "dogleg",
            "trust-ncg",
            "trust-exact",
            "trust-krylov",
            "trust-constr",
        }
