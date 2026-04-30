"""
Abstract optimizer for finite-dimensional mathematical programs.

Optimizers consume :class:`~minilink.optimization.mathematical_program.MathematicalProgram`
instances. They do not know about planning systems, trajectories, or
transcription internals.
"""

import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import replace

import numpy as np

from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)


class Optimizer(ABC):
    """
    Base class for finite-dimensional optimizers (mathematical-program solvers).
    """

    @abstractmethod
    def _solve_impl(
        self,
        program: MathematicalProgram,
        *,
        callback: Callable[[object], None] | None = None,
    ) -> OptimizationResult:
        """Backend implementation; use :meth:`solve` from callers."""
        ...

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
            If True, set :attr:`~minilink.optimization.mathematical_program.OptimizationResult.solve_time_s`
            to the wall-clock duration (seconds) of the backend solve only,
            measured with :func:`time.perf_counter`.
        disp : bool, optional
            If True, print a short text report (success, cost, ``z`` preview, ``stats``,
            wall-clock time when available). Independent of backend-specific flags such as
            SciPy ``options["disp"]``). Implies timing so the report always includes
            ``solve_time_s``.
        """
        time_solve = record_solve_time or disp
        if not time_solve:
            result = self._solve_impl(program, callback=callback)
        else:
            t0 = time.perf_counter()
            result = self._solve_impl(program, callback=callback)
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

        lines = [
            "",
            "=" * 60,
            f"Optimization report — {type(self).__name__}",
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
