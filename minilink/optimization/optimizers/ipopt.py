"""The Ipopt backend: a compiled program solved by cyipopt's interior-point method."""

import numpy as np

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


class IpoptOptimizer(OptimizerBackend):
    """
    Optimizer adapter for :func:`cyipopt.minimize_ipopt`.

    Ipopt expects ``g(x) >= 0`` for inequality constraints, the same convention
    used by :class:`~minilink.optimization.mathematical_program.MathematicalProgram`.
    Per-iterate callbacks are not supported by cyipopt's native path.
    """

    def __init__(self, options: dict | None = None, tol: float | None = None):
        # Solver options, copied, and the convergence tolerance
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
        require_no_callback(callback)
        minimize_ipopt = require_cyipopt()

        # Equalities h(z) = 0 and inequalities g(z) ≥ 0, each with its Jacobian when
        # the evaluator has one
        constraints = scipy_constraints_of(program_evaluator)

        # The interior-point solve from z0, inside the box bounds
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

        # The counters and the status the solver reports, and the final cost
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


###############################################################################
# Internal machinery
###############################################################################


def require_no_callback(callback):
    """Refuse a per-iterate callback: cyipopt's native path runs none."""
    if callback is not None:
        raise NotImplementedError(
            "IpoptOptimizer does not support solve(callback=...): cyipopt's "
            "native Ipopt path does not run a Python callback each iteration. "
            "Use a SciPy optimizer method (e.g. 'scipy_slsqp') or omit callback."
        )


def require_cyipopt():
    """Refuse a solve without the optional cyipopt package; return its ``minimize_ipopt``."""
    try:
        from cyipopt import minimize_ipopt
    except ImportError as exc:  # pragma: no cover - optional dep
        raise ImportError(
            "IpoptOptimizer requires the optional 'cyipopt' package, which needs a "
            "system Ipopt: `conda install -c conda-forge cyipopt` (the conda environment "
            "has it), or `pip install cyipopt` after installing Ipopt."
        ) from exc

    return minimize_ipopt
