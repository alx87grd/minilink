"""
Optimization backend contracts for planning transcriptions.

This module intentionally defines a thin interface first. Concrete solver
behavior, including SciPy SLSQP wiring, is deferred until the planning
architecture has been reviewed.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np


@dataclass(frozen=True)
class OptimizationRequest:
    """
    Finite-dimensional optimization problem passed to a backend.

    Parameters
    ----------
    objective : callable
        Function ``objective(decision) -> float``.
    x0 : np.ndarray
        Initial decision vector.
    bounds : object, optional
        Backend-specific variable bounds.
    constraints : tuple
        Backend-specific constraint descriptors.
    metadata : dict
        Optional transcription metadata for debugging.
    """

    objective: Callable[[np.ndarray], float]
    x0: np.ndarray
    bounds: Any | None = None
    constraints: tuple[Any, ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        x0 = np.asarray(self.x0, dtype=float).reshape(-1).copy()
        object.__setattr__(self, "x0", x0)
        object.__setattr__(self, "constraints", tuple(self.constraints))
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class OptimizationResult:
    """
    Result returned by an optimization backend.
    """

    x: np.ndarray
    success: bool
    cost: float | None = None
    message: str = ""
    stats: dict[str, Any] = field(default_factory=dict)
    raw_result: Any | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", np.asarray(self.x, dtype=float).reshape(-1).copy())
        object.__setattr__(self, "stats", dict(self.stats))


class OptimizationBackend(ABC):
    """
    Base class for finite-dimensional optimization backends.
    """

    @abstractmethod
    def solve(self, request: OptimizationRequest) -> OptimizationResult:
        """Solve a finite-dimensional optimization request."""
        ...


@dataclass(frozen=True)
class ScipyMinimizeBackend(OptimizationBackend):
    """
    Skeleton for a future :func:`scipy.optimize.minimize` backend.

    The first planning pass keeps this as a reviewable contract. Full
    SciPy wiring is intentionally deferred.
    """

    method: str = "SLSQP"
    options: dict[str, Any] = field(default_factory=dict)

    def solve(self, request: OptimizationRequest) -> OptimizationResult:
        """
        Solve the optimization request.

        TODO: User Architectural Review - implement SciPy minimize wiring
        after the deterministic planning architecture is reviewed.
        """
        raise NotImplementedError(
            "ScipyMinimizeBackend.solve is deferred until architecture review"
        )
