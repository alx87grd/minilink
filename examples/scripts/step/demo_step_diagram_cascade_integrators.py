"""Cascade of two discrete integrators in series.

Constant input drives a double-integrator chain (triangular state growth).

Run from the repo root::

    python examples/scripts/step/demo_step_diagram_cascade_integrators.py
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem


class DiscreteAccumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "DiscreteAccumulator"
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


diagram = StepDiagramSystem()
diagram.add_subsystem(DiscreteAccumulator(), "acc1")
diagram.add_subsystem(DiscreteAccumulator(), "acc2")
diagram.add_input_port("u")
diagram.connect("input", "u", "acc1", "u")
diagram.connect("acc1", "y", "acc2", "u")
diagram.connect_new_output_port("acc2", "y", "y")

diagram.plot_diagram()

diagram.compute_rollout(n_steps=20, u=np.array([1.0]))
diagram.plot_rollout()
