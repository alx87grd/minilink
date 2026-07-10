"""Shared output-tier contract for all compiled evaluators."""

from abc import ABC, abstractmethod

import numpy as np


def outputs_from_ports(system, x, u, t, params) -> dict:
    """Evaluate every output port on ``system``."""
    return {
        port_id: port.compute(x, u, t, params)
        for port_id, port in system.outputs.items()
    }


class OutputEvaluator(ABC):
    """
    Base class for compiled evaluators — output tier only.

    Boundary signals are returned as a dict keyed by output port id.
    """

    n: int
    m: int
    p: int
    backend: str
    _frozen_params: dict
    _u_nominal: np.ndarray

    @abstractmethod
    def outputs(self, x, u, t=0.0) -> dict:
        """All boundary output ports, frozen params."""
        ...

    @abstractmethod
    def outputs_p(self, x, u, t, params) -> dict:
        """All boundary output ports; caller-supplied params."""
        ...
