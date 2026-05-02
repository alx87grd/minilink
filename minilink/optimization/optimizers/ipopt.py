"""Cyipopt :func:`cyipopt.minimize_ipopt` optimizer.

Wraps the COIN-OR Ipopt interior-point solver behind the same generic
:class:`~minilink.optimization.optimizers.optimizer_backend.OptimizerBackend` contract
used by :class:`~minilink.optimization.optimizers.scipy_minimize.ScipyMinimizeOptimizer`,
so a :class:`~minilink.optimization.mathematical_program.MathematicalProgram`
can switch between the SciPy and Ipopt backends with no other code changes.

Equality constraints ``h(z) = 0`` and inequality constraints ``g(z) >= 0`` are
forwarded as ``type='eq'`` and ``type='ineq'`` dictionaries, matching the
SciPy-style interface that ``minimize_ipopt`` accepts.

The optional dependency ``cyipopt`` (PyPI name ``cyipopt``) provides the Ipopt
binding; install with ``pip install cyipopt`` or via the ``ipopt`` extra.
"""

import numpy as np

from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizers.optimizer_backend import (
    BackendIterateCallback,
    OptimizerBackend,
)


class IpoptOptimizer(OptimizerBackend):
    """
    Optimizer adapter for :func:`cyipopt.minimize_ipopt`.

    Parameters
    ----------
    options : dict, optional
        Ipopt options forwarded to :func:`cyipopt.minimize_ipopt`. The ``disp``
        and ``maxiter`` keys are remapped to ``print_level`` / ``max_iter`` by
        cyipopt; any other key is passed straight through to Ipopt (see
        https://coin-or.github.io/Ipopt/OPTIONS.html).
    tol : float, optional
        Relative convergence tolerance forwarded to Ipopt as the ``tol`` option.

    Notes
    -----
    Ipopt expects ``g(x) >= 0`` for inequality constraints, the same convention
    used by :class:`~minilink.optimization.mathematical_program.MathematicalProgram`,
    so margins are passed through unchanged.

    Per-iterate callbacks are not supported on cyipopt's native Ipopt path;
    :meth:`solve` raises ``NotImplementedError`` if ``callback`` is not
    ``None``.
    """

    def __init__(self, options: dict | None = None, tol: float | None = None):
        self.options = {} if options is None else dict(options)
        self.tol = tol

    def solve(
        self,
        program: MathematicalProgram,
        *,
        callback: BackendIterateCallback | None = None,
    ) -> OptimizationResult:
        """Solve ``program`` with Ipopt and return a backend-neutral result."""
        if callback is not None:
            raise NotImplementedError(
                "IpoptOptimizer does not support solve(callback=...): cyipopt's "
                "native Ipopt path does not run a Python callback each iteration. "
                "Use a SciPy optimizer label (e.g. 'scipy') or omit callback."
            )
        try:
            from cyipopt import minimize_ipopt
        except ImportError as exc:  # pragma: no cover - optional dep
            raise ImportError(
                "IpoptOptimizer requires the optional 'cyipopt' package; "
                "install with `pip install cyipopt`."
            ) from exc

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

        raw_result = minimize_ipopt(
            program.objective,
            program.z0,
            jac=program.gradient if program.grad is not None else None,
            hess=program.hessian if program.hess is not None else None,
            bounds=bounds,
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
