"""Cyipopt :func:`cyipopt.minimize_ipopt` optimizer adapter."""

import numpy as np

from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
from minilink.optimization.mathematical_program import OptimizationResult
from minilink.optimization.optimizers.optimizer_backend import (
    BackendIterateCallback,
    OptimizerBackend,
)


class IpoptOptimizer(OptimizerBackend):
    """
    Optimizer adapter for :func:`cyipopt.minimize_ipopt`.

    Ipopt expects ``g(x) >= 0`` for inequality constraints, the same convention
    used by :class:`~minilink.optimization.mathematical_program.MathematicalProgram`.
    Per-iterate callbacks are not supported by cyipopt's native path.
    """

    def __init__(self, options: dict | None = None, tol: float | None = None):
        self.options = {} if options is None else dict(options)
        self.tol = tol

    def solve(
        self,
        program_evaluator: MathematicalProgramEvaluator,
        z0: np.ndarray,
        *,
        callback: BackendIterateCallback | None = None,
    ) -> OptimizationResult:
        """Solve ``program_evaluator`` with Ipopt and return a backend-neutral result."""
        if callback is not None:
            raise NotImplementedError(
                "IpoptOptimizer does not support solve(callback=...): cyipopt's "
                "native Ipopt path does not run a Python callback each iteration. "
                "Use a SciPy optimizer method (e.g. 'scipy_slsqp') or omit callback."
            )
        try:
            from cyipopt import minimize_ipopt
        except ImportError as exc:  # pragma: no cover - optional dep
            raise ImportError(
                "IpoptOptimizer requires the optional 'cyipopt' package; "
                "install with `pip install cyipopt`."
            ) from exc

        constraints = []

        if program_evaluator.n_h > 0:
            entry = {"type": "eq", "fun": program_evaluator.equality_residual}
            if program_evaluator.has_jacobian_h:
                entry["jac"] = program_evaluator.jacobian_h
            constraints.append(entry)

        if program_evaluator.n_g > 0:
            entry = {"type": "ineq", "fun": program_evaluator.inequality_margin}
            if program_evaluator.has_jacobian_g:
                entry["jac"] = program_evaluator.jacobian_g
            constraints.append(entry)

        raw_result = minimize_ipopt(
            program_evaluator.objective,
            np.asarray(z0, dtype=float).reshape(program_evaluator.n_z),
            jac=program_evaluator.gradient if program_evaluator.has_gradient else None,
            hess=program_evaluator.hessian if program_evaluator.has_hessian else None,
            bounds=program_evaluator.scipy_bounds(),
            constraints=constraints,
            tol=self.tol,
            options=dict(self.options),
        )

        stats = {}
        for name in ("nit", "nfev", "njev", "status"):
            if hasattr(raw_result, name):
                stats[name] = getattr(raw_result, name)

        cost = None
        if getattr(raw_result, "fun", None) is not None:
            cost = float(raw_result.fun)

        return OptimizationResult(
            z=np.asarray(raw_result.x, dtype=float),
            success=bool(raw_result.success),
            cost=cost,
            message=str(raw_result.message),
            stats=stats,
            raw_result=raw_result,
        )
