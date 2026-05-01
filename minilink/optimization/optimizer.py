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

# Human-readable ``disp=True`` panel (preamble + report share one frame).
_DISP_LINE_WIDTH = 60
_DISP_RULE_MAIN = "=" * _DISP_LINE_WIDTH
_DISP_RULE_DIV = "-" * _DISP_LINE_WIDTH

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
        kwargs = {}
        for key, value in preset.items():
            kwargs[key] = dict(value) if isinstance(value, dict) else value
        kwargs.update(backend_kwargs)
        if options is not None:
            existing = kwargs.get("options", {})
            kwargs["options"] = {**existing, **options}
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
            If True, print one framed ``disp`` panel: a **Before solve** section
            (``n_z``, ``z0``, backend settings), a divider, then **After solve**
            (``z``, outcome, ``stats``, wall-clock time when available).
            Independent of backend-specific flags such as SciPy
            ``options["disp"]``. Implies timing so the panel always includes
            ``solve_time_s``.
        """
        time_solve = record_solve_time or disp

        if disp:
            self._print_solve_preamble(program)
        if time_solve:
            t0 = time.perf_counter()

        ##############################################################
        # --- Backend solve ---
        result = self.backend.solve(program, callback=callback)
        ##############################################################

        if time_solve:
            elapsed = time.perf_counter() - t0
            result = replace(result, solve_time_s=elapsed)

        if disp:
            self._print_solve_report(program, result)

        return result

    @staticmethod
    def _preview_z(z: np.ndarray, nz: int) -> str:
        """Compact string for a length-``nz`` slice of ``z`` (used when ``disp=True``)."""
        if nz <= 8:
            return np.array2string(z, precision=6, max_line_width=96)
        head = np.array2string(z[:4], precision=6, max_line_width=96)
        return f"{head} ... ({nz} values)"

    @staticmethod
    def _disp_sorted_dict_lines(mapping: object) -> list[str]:
        """Sorted ``key : value`` lines with a four-space indent (used under ``disp`` headings)."""
        if isinstance(mapping, dict) and mapping:
            keyw = max(len(str(k)) for k in mapping)
            return [
                f"    {str(k).ljust(keyw)} : {v}"
                for k, v in sorted(mapping.items(), key=lambda kv: str(kv[0]))
            ]
        return ["    (empty)"]

    def _disp_banner_title(self) -> str:
        backend_name = type(self.backend).__name__
        return f"Optimization (backend={self.backend_label!r}) "

    def _print_solve_preamble(self, program: MathematicalProgram) -> None:
        """Open the ``disp`` panel and print the **Before solve** section."""
        nz = int(program.n_z)
        z_str = self._preview_z(program.z0, nz)
        backend = self.backend
        fields: list[tuple[str, str]] = [
            ("n_z", str(nz)),
            ("z0", z_str),
        ]
        if hasattr(backend, "method"):
            fields.append(("method", str(getattr(backend, "method"))))
        tol = getattr(backend, "tol", None)
        if tol is not None:
            fields.append(("tol", str(tol)))
        keyw = max(len(k) for k, _ in fields)

        lines = [
            "",
            _DISP_RULE_MAIN,
            self._disp_banner_title(),
            _DISP_RULE_MAIN,
            " Running solver...",
        ]
        for key, value in fields:
            lines.append(f"  {key.ljust(keyw)} : {value}")
        lines.append("  options:")
        lines.extend(self._disp_sorted_dict_lines(getattr(backend, "options", None)))
        lines.append("")
        lines.append(_DISP_RULE_DIV)
        print("\n".join(lines))

    def _print_solve_report(
        self,
        program: MathematicalProgram,
        result: OptimizationResult,
    ) -> None:
        """Print the **After solve** section and close the ``disp`` panel."""
        nz = int(program.n_z)
        z_str = self._preview_z(result.z, nz)

        if result.cost is None:
            cost_str = "(none)"
        else:
            cost_str = f"{result.cost:.12g}"
        if result.solve_time_s is not None:
            time_str = f"{result.solve_time_s:.6g}"
        else:
            time_str = "(not recorded)"

        outcome: list[tuple[str, str]] = [
            ("z* (optimal)", z_str),
            ("J* (optimal)", cost_str),
            ("success", str(result.success)),
            # ("message", str(result.message)),
            ("solve_time_s", time_str),
        ]
        keyw = max(len(k) for k, _ in outcome)

        lines = [
            "Result:",
        ]
        for key, value in outcome:
            lines.append(f"  {key.ljust(keyw)} : {value}")
        lines.append("  stats:")
        lines.extend(self._disp_sorted_dict_lines(result.stats))
        lines.append("")
        lines.append(_DISP_RULE_MAIN)
        print("\n".join(lines))
