"""
Abstract backend for finite-dimensional mathematical-program solvers.

Concrete subclasses (e.g. :class:`~minilink.optimization.optimizers.scipy_minimize.ScipyMinimizeOptimizer`,
:class:`~minilink.optimization.optimizers.ipopt.IpoptOptimizer`) implement
:meth:`OptimizerBackend.solve`. Backends consume
:class:`~minilink.optimization.mathematical_program.MathematicalProgram`
instances and know nothing about planning systems, trajectories, or
transcription internals.

User-facing orchestration (timing, ``disp`` reports, default selection) lives
on :class:`minilink.optimization.optimizer.Optimizer`; backends are pure
adapters whose only contract is :meth:`solve`.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable

import numpy as np

from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)

# Called by backends with the current iterate ``z`` only; the orchestrator adds ``J`` and elapsed time.
BackendIterateCallback = Callable[[np.ndarray], None]


class OptimizerBackend(ABC):
    """
    Pure adapter contract for a finite-dimensional optimizer backend.

    Subclasses implement :meth:`solve`; the
    :class:`~minilink.optimization.optimizer.Optimizer` orchestrator wraps
    that with timing and reporting.
    """

    @abstractmethod
    def solve(
        self,
        program: MathematicalProgram,
        *,
        callback: BackendIterateCallback | None = None,
    ) -> OptimizationResult:
        """Run the backend solver; called by :meth:`Optimizer.solve`.

        ``callback`` receives only the current decision vector ``z`` (1-D float).
        """
        ...
