"""Multi-rate filter + controller with a continuous integrator plant.

Same topology as ``demo_computer_multi_rate.py``, but the plant runs in
continuous time via :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`.

Run from the repo root::

    python examples/scripts/hybrid/demo_hybrid_multi_rate.py
"""

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.backends import array_module
from minilink.core.diagram import StepDiagramSystem
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.core.system import StepSystem
from minilink.simulation.computer import StepSchedule

DT_BASE = 0.01
TF = 2.0
PLANT_DT_INNER = 0.002


class DiscreteLowPass(StepSystem):
    """First-order low-pass: ``x_{k+1} = α x_k + (1-α) u_k``, ``y = x``."""

    def __init__(self, alpha=0.25):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.params = {"alpha": float(alpha)}
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        alpha = params["alpha"]
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([alpha * x[0] + (1.0 - alpha) * u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


def reference(t):
    """Step reference on the computer boundary input."""
    return np.array([1.0 if t < 1.0 else 0.0])


step_diagram = StepDiagramSystem()
step_diagram.add_subsystem(DiscreteLowPass(alpha=0.6), "filter")
step_diagram.add_subsystem(ProportionalController(10.0), "ctl")
step_diagram.add_input_port("r")
step_diagram.add_input_port("y_meas")
step_diagram.connect("input", "r", "ctl", "r")
step_diagram.connect("input", "y_meas", "filter", "u")
step_diagram.connect("filter", "y", "ctl", "y")
step_diagram.connect_new_output_port("filter", "y", "y_f")
step_diagram.connect_new_output_port("ctl", "u", "u_cmd")

schedule = StepSchedule.from_rates(
    dt_base=DT_BASE,
    rates_hz={"filter": 100.0, "ctl": 10.0},
)

hybrid = hybrid_closed_loop(
    step_diagram,
    Integrator(),
    schedule=schedule,
    computer_out="u_cmd",
    computer_in="y_meas",
)

hybrid.plot_diagram()

result = hybrid.compute_forced(
    reference,
    t0=0,
    tf=TF,
    input_port_id="r",
    plant_dt_inner=PLANT_DT_INNER,
)
hybrid.plot_trajectory(signals=("r", "y", "y_f", "u_cmd", "x_plant"), show=True)
