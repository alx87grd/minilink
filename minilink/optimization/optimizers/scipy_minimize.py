"""SciPy :func:`scipy.optimize.minimize` optimizer."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from scipy.optimize import minimize

from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizers.optimizer import Optimizer


@dataclass(frozen=True)
class ScipyMinimizeOptimizer(Optimizer):
    """
    Optimizer adapter for :func:`scipy.optimize.minimize`.

    Equality constraints are passed as ``type='eq'`` residuals and
    inequality constraints as ``type='ineq'`` nonnegative margins, matching
    the generic :class:`~minilink.optimization.mathematical_program.MathematicalProgram`
    convention.
    """

    method: str = "SLSQP"
    options: dict[str, Any] = field(default_factory=dict)

    def solve(self, program: MathematicalProgram) -> OptimizationResult:
        """Solve ``program`` with SciPy and return a backend-neutral result."""
        constraints = [
            {
                "type": "eq",
                "fun": equality.residual,
            }
            for equality in program.equalities
        ]
        constraints.extend(
            {
                "type": "ineq",
                "fun": inequality.margin,
            }
            for inequality in program.inequalities
        )

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

        raw_result = minimize(
            program.objective,
            program.z0,
            method=self.method,
            bounds=bounds,
            constraints=constraints,
            options=dict(self.options),
        )

        stats = {
            name: getattr(raw_result, name)
            for name in ("nit", "nfev", "njev", "status")
            if hasattr(raw_result, name)
        }
        return OptimizationResult(
            z=np.asarray(raw_result.x, dtype=float),
            success=bool(raw_result.success),
            cost=float(raw_result.fun) if raw_result.fun is not None else None,
            message=str(raw_result.message),
            stats=stats,
            raw_result=raw_result,
        )
