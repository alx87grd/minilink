"""
Abstract backend for finite-dimensional optimizer adapters.

Concrete subclasses adapt a compiled
:class:`~minilink.optimization.evaluators.program_evaluator.MathematicalProgramEvaluator`
to a third-party solver such as SciPy or Ipopt. The user-facing
:class:`minilink.optimization.optimizer.Optimizer` owns program compilation,
timing, diagnostics, and default initial guesses.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable

import numpy as np

from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
from minilink.optimization.mathematical_program import OptimizationResult

# Called by backends with the current iterate ``z`` only; the orchestrator adds ``J`` and elapsed time.
BackendIterateCallback = Callable[[np.ndarray], None]


class OptimizerBackend(ABC):
    """
    Pure adapter contract for a finite-dimensional optimizer backend.
    """

    @abstractmethod
    def solve(
        self,
        program_evaluator: MathematicalProgramEvaluator,
        z0: np.ndarray,
        *,
        callback: BackendIterateCallback | None = None,
    ) -> OptimizationResult:
        """Run the backend solver from ``z0``."""
        ...
