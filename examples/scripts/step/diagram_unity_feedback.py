"""Discrete unity-feedback step diagram: P controller + accumulator plant.

Digital counterpart to ``examples/scripts/diagrams/diagram_closed_loop.py``.

Run from the repo root::

    python examples/scripts/step/diagram_unity_feedback.py
"""

import numpy as np

from minilink.control.output import ProportionalController
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
diagram.add_subsystem(ProportionalController(0.3), "ctl")
diagram.add_subsystem(DiscreteAccumulator(), "plant")
diagram.add_input_port("r")
diagram.connect("input", "r", "ctl", "r")
diagram.connect("plant", "y", "ctl", "y")
diagram.connect("ctl", "u", "plant", "u")
diagram.connect_new_output_port("plant", "y", "y")

diagram.plot_diagram()

diagram.compute_rollout(n_steps=40, u=np.array([1.0]))
diagram.plot_rollout()
