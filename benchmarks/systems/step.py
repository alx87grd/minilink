"""Discrete-time benchmark fixtures."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem


class LogisticMap(StepSystem):
    """Scalar logistic map ``x_{k+1} = r x_k (1 - x_k)``."""

    def __init__(self, *, r=3.9):
        super().__init__(n=1, input_dim=0, output_dim=1, y_dependencies=())
        self.name = "LogisticMap"
        self.r = r
        self.x0 = np.array([0.4])

    def step(self, x, u, k=0, params=None):
        xp = array_module(x)
        x_val = xp.asarray(x, dtype=float).reshape(1)
        return xp.array([self.r * x_val[0] * (1.0 - x_val[0])])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


class _DiscreteAccumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "DiscreteAccumulator"
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        x_val = xp.asarray(x, dtype=float).reshape(1)
        u_val = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x_val[0] + u_val[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


def make_step_chain(*, depth: int = 50) -> StepDiagramSystem:
    """Build a serial chain of discrete integrators for step-diagram timing."""
    if depth < 1:
        raise ValueError("depth must be >= 1")

    diagram = StepDiagramSystem()
    ids = []
    for i in range(depth):
        sys_id = f"acc{i}"
        diagram.add_subsystem(_DiscreteAccumulator(), sys_id)
        ids.append(sys_id)

    diagram.add_input_port("u")
    diagram.connect("input", "u", ids[0], "u")
    for i in range(depth - 1):
        diagram.connect(ids[i], "y", ids[i + 1], "u")
    diagram.connect_new_output_port(ids[-1], "y", "y")
    diagram.name = f"StepChain(depth={depth})"
    return diagram
