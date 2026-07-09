"""ZOHHold actuator feeding a discrete integrator — composed step loop.

Command is latched, then accumulated: classical sampled actuation pattern.

Run from the repo root::

    python examples/scripts/step/demo_step_diagram_zoh_actuator.py
"""

import numpy as np

from minilink.blocks.step import ZOHHold
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


def pulse_input(k):
    return np.array([1.0 if k < 10 else 0.0])


diagram = StepDiagramSystem()
diagram.add_subsystem(ZOHHold(), "hold")
diagram.add_subsystem(DiscreteAccumulator(), "plant")
diagram.add_input_port("u")
diagram.connect("input", "u", "hold", "u")
diagram.connect("hold", "y", "plant", "u")
diagram.connect_new_output_port("plant", "y", "y")

diagram.plot_diagram()

diagram.compute_rollout(n_steps=25, u=pulse_input)
diagram.plot_rollout()
