"""
Orchestrator for finite-dimensional mathematical-program solvers.

:class:`Optimizer` selects a concrete
:class:`~minilink.optimization.optimizers.optimizer.OptimizerBackend` by string
label and owns all wrapping concerns (wall-clock timing, ``disp`` reports,
default options). The backend itself exposes only
:meth:`~minilink.optimization.optimizers.optimizer.OptimizerBackend.solve`.

Modelled on :class:`~minilink.simulation.simulator.Simulator`'s
pluggable-solver pattern: each user-facing label in
:data:`_USER_OPTIMIZER_MODES` resolves to a backend key plus preset
constructor kwargs. Extra keyword arguments override the preset.

Typical use::

    opt = Optimizer(backend="scipy", options={"maxiter": 200, "ftol": 1e-12})
    result = opt.solve(program, disp=True)
"""

import time
from collections.abc import Callable
from dataclasses import replace

import numpy as np

from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizers.optimizer import OptimizerBackend

# User-facing optimizer labels mapped to (backend_key, default_constructor_kwargs).
# Mirrors :data:`minilink.simulation.simulator._USER_SOLVER_MODES` in spirit.
_USER_OPTIMIZER_MODES: dict[str, tuple[str, dict]] = {
    "scipy": ("scipy_minimize", {"method": "SLSQP", "options": {}}),
    "scipy_trust": ("scipy_minimize", {"method": "trust-constr", "options": {}}),
    "ipopt": ("ipopt", {"options": {}, "tol": 1e-8}),
}


class Optimizer:
    """
    Pluggable orchestrator over :class:`OptimizerBackend`.

    Parameters
    ----------
    backend : str, optional
        Optimizer label. One of ``"scipy"``, ``"scipy_trust"``, ``"ipopt"``;
        see :data:`_USER_OPTIMIZER_MODES`.
    options : dict, optional
        Solver options merged into the preset's ``options`` dict (user keys win).
    **backend_kwargs
        Extra keyword arguments override the preset's top-level kwargs and are
        forwarded to the backend constructor (e.g. ``method="L-BFGS-B"`` for the
        scipy backend, ``tol=1e-9`` for ipopt).
    """

    def __init__(
        self,
        backend: str = "scipy",
        *,
        options: dict | None = None,
        **backend_kwargs,
    ):
        if backend not in _USER_OPTIMIZER_MODES:
            valid = ", ".join(sorted(_USER_OPTIMIZER_MODES))
            raise ValueError(
                f"Unknown optimizer backend {backend!r}. Expected one of: {valid}."
            )
        backend_key, preset = _USER_OPTIMIZER_MODES[backend]
        kwargs = {k: dict(v) if isinstance(v, dict) else v for k, v in preset.items()}
        kwargs.update(backend_kwargs)
        if options is not None:
            kwargs["options"] = {**kwargs.get("options", {}), **options}
        self.backend_label = backend
        self.backend = self._select_backend(backend_key, kwargs)

    @staticmethod
    def _select_backend(backend_key: str, kwargs: dict) -> OptimizerBackend:
        if backend_key == "scipy_minimize":
            from minilink.optimization.optimizers.scipy_minimize import (
                ScipyMinimizeOptimizer,
            )

            return ScipyMinimizeOptimizer(**kwargs)
        if backend_key == "ipopt":
            from minilink.optimization.optimizers.ipopt import IpoptOptimizer

            return IpoptOptimizer(**kwargs)
        raise ValueError(f"Unknown optimizer backend key {backend_key!r}.")

    def solve(
        self,
        program: MathematicalProgram,
        *,
        callback: Callable[[object], None] | None = None,
        record_solve_time: bool = False,
        disp: bool = False,
    ) -> OptimizationResult:
        """
        Solve a finite-dimensional mathematical program.

        Parameters
        ----------
        record_solve_time : bool, optional
            If True, set
            :attr:`~minilink.optimization.mathematical_program.OptimizationResult.solve_time_s`
            to the wall-clock duration (seconds) of the backend solve only,
            measured with :func:`time.perf_counter`.
        disp : bool, optional
            If True, print a short text report (success, cost, ``z`` preview,
            ``stats``, wall-clock time when available). Independent of
            backend-specific flags such as SciPy ``options["disp"]``. Implies
            timing so the report always includes ``solve_time_s``.
        """
        time_solve = record_solve_time or disp
        if not time_solve:
            result = self.backend.solve(program, callback=callback)
        else:
            t0 = time.perf_counter()
            result = self.backend.solve(program, callback=callback)
            elapsed = time.perf_counter() - t0
            result = replace(result, solve_time_s=elapsed)
        if disp:
            self._print_solve_report(program, result)
        return result

    def _print_solve_report(
        self,
        program: MathematicalProgram,
        result: OptimizationResult,
    ) -> None:
        """Print a human-readable summary to stdout (used when ``disp=True``)."""
        nz = int(program.n_z)
        z = result.z
        if nz <= 8:
            z_str = np.array2string(z, precision=6, max_line_width=96)
        else:
            head = np.array2string(z[:4], precision=6, max_line_width=96)
            z_str = f"{head} ... ({nz} values)"

        backend_name = type(self.backend).__name__
        lines = [
            "",
            "=" * 60,
            f"Optimization report — Optimizer(backend={self.backend_label!r}) [{backend_name}]",
            "=" * 60,
            f"  n_z             : {nz}",
            f"  z               : {z_str}",
            f"  success         : {result.success}",
            f"  message         : {result.message}",
        ]
        if result.cost is None:
            lines.append("  cost            : (none)")
        else:
            lines.append(f"  cost            : {result.cost:.12g}")
        if result.solve_time_s is not None:
            lines.append(f"  solve_time_s    : {result.solve_time_s:.6g}")
        else:
            lines.append("  solve_time_s    : (not recorded)")
        lines.append("  stats           :")
        if result.stats:
            keyw = max(len(str(k)) for k in result.stats)
            for k, v in sorted(result.stats.items(), key=lambda kv: str(kv[0])):
                lines.append(f"    {str(k).ljust(keyw)} : {v}")
        else:
            lines.append("    (empty)")
        lines.append("=" * 60)
        print("\n".join(lines))
