"""Four ZOHHold blocks in series — cumulative one-step phase lag.

A finite pulse on the boundary input propagates down the chain; each latch
delays the edge by one step (``y_k = u_{k-1}`` per hold).

Run from the repo root::

    python examples/demos/step/diagram_cascade_zoh.py
"""

import numpy as np

from minilink.blocks.step import ZOHHold
from minilink.core.diagram import StepDiagramSystem


def pulse_input(k):
    return np.array([1.0 if k < 8 else 0.0])


diagram = StepDiagramSystem()
diagram.add_subsystem(ZOHHold(), "hold1")
diagram.add_subsystem(ZOHHold(), "hold2")
diagram.add_subsystem(ZOHHold(), "hold3")
diagram.add_subsystem(ZOHHold(), "hold4")
diagram.add_input_port("u")
diagram.connect("input", "u", "hold1", "u")
diagram.connect("hold1", "y", "hold2", "u")
diagram.connect("hold2", "y", "hold3", "u")
diagram.connect("hold3", "y", "hold4", "u")
diagram.connect_new_output_port("hold4", "y", "y")

diagram.plot_diagram()

diagram.compute_rollout(n_steps=24, u=pulse_input)
diagram.plot_rollout(signals=("u", "x"))
