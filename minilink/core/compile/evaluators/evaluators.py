"""
Compiled evaluator bases — the output, dynamics, step and static contracts.

Every method is one body taking ``params=None`` (``None`` reads the parameters
compiled in); on JAX each is compiled once (:func:`~minilink.core.compile.evaluators.compiled.compiled`).
Concrete backends live in :mod:`numpy_evaluators` and :mod:`jax_evaluators`.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

###############################################################################
# Public API
###############################################################################


class OutputEvaluator(ABC):
    """
    Base class for compiled evaluators: the boundary outputs.

    Boundary signals are returned as a dict keyed by output port id.
    """

    n: int
    m: int
    p: int
    backend: str
    _u_nominal: np.ndarray

    @abstractmethod
    def outputs(self, x, u, t=0.0, params=None) -> dict:
        """Every boundary output port at ``(x, u, t)``; ``params=None`` reads the compiled ones."""
        ...

    @property
    def nominal_input(self):
        """The input held when none is given: every port at its nominal value."""
        return self._u_nominal


class DynamicsEvaluator(OutputEvaluator):
    """
    Base class for compiled continuous-time evaluators: ``f``, ``outputs`` and the step model.

    The step model (``step``, ``rollout``) is mixed in by the NumPy / JAX backend modules.
    """

    @abstractmethod
    def f(self, x, u, t=0.0, params=None):
        """``dx = f(x, u, t; p)``; ``params=None`` reads the compiled parameters."""
        ...


class StepEvaluator(OutputEvaluator):
    """Base class for compiled discrete-time evaluators: the system's own ``step`` and ``outputs``."""

    @abstractmethod
    def step(self, x, u, k=0, params=None):
        """``x_{k+1} = step(x, u, k; p)``; ``params=None`` reads the compiled parameters."""
        ...


class StaticEvaluator(OutputEvaluator):
    """Compiled evaluator for stateless IO blocks: outputs only."""


def outputs_from_ports(system, x, u, t, params) -> dict:
    """Evaluate every output port on ``system``."""
    return {
        port_id: port.compute(x, u, t, params)
        for port_id, port in system.outputs.items()
    }
