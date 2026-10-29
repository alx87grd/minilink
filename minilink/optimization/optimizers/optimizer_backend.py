"""Optimizer backends: the contract of an adapter to a third-party solver."""

from abc import ABC, abstractmethod
from collections.abc import Callable

import numpy as np

from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
from minilink.optimization.mathematical_program import OptimizationResult

# Called by backends with the current iterate ``z`` only; the orchestrator adds ``J`` and elapsed time.
BackendIterateCallback = Callable[[np.ndarray], None]


###############################################################################
# Public API
###############################################################################


class OptimizerBackend(ABC):
    """
    Pure adapter contract for a finite-dimensional optimizer backend.

    Concrete subclasses adapt a compiled
    :class:`~minilink.optimization.evaluators.program_evaluator.MathematicalProgramEvaluator`
    to a third-party solver such as SciPy or Ipopt. The user-facing
    :class:`minilink.optimization.optimizer.Optimizer` owns program compilation,
    timing, diagnostics, and default initial guesses. :attr:`method_params`
    is the method's numbers a solve can rebind (``{}`` for a method whose
    options are its stopping rule only).
    """

    @property
    def method_params(self) -> dict:
        """The method's numbers, by name: ``{}`` (a host solver's options are its stopping rule)."""
        return {}

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


def scipy_constraints_of(program_evaluator: MathematicalProgramEvaluator) -> list:
    """
    The program's constraints in SciPy's dict form, as SciPy and Ipopt take them.

    Equalities ``h(z) = 0`` as one ``"eq"`` entry and inequalities ``g(z) ≥ 0``
    as one ``"ineq"`` entry, each with its Jacobian when the evaluator has one;
    an empty kind is left out.
    """
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

    return constraints
