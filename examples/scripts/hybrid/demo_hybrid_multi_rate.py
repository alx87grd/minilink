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
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.hybrid_diagram import HybridDiagram
from minilink.core.system import StepSystem
from minilink.simulation.computer import Computer, StepSchedule

DT_BASE = 0.01
TF = 1.6
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
    return np.array([1.0 if t < 0.6 else 0.0])


def build_step_diagram():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(DiscreteLowPass(alpha=0.3), "filter")
    diagram.add_subsystem(ProportionalController(0.35), "ctl")
    diagram.add_input_port("r")
    diagram.add_input_port("y_meas")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("input", "y_meas", "filter", "u")
    diagram.connect("filter", "y", "ctl", "y")
    diagram.connect_new_output_port("filter", "y", "y_f")
    diagram.connect_new_output_port("ctl", "u", "u_cmd")
    return diagram


def build_plant():
    plant = DiagramSystem()
    plant.add_subsystem(Integrator(), "plant")
    plant.add_input_port("u")
    plant.connect("input", "u", "plant", "u")
    plant.connect_new_output_port("plant", "y", "y")
    return plant


def build_hybrid():
    schedule = StepSchedule.from_rates(
        dt_base=DT_BASE,
        rates_hz={"filter": 100.0, "ctl": 10.0},
    )
    computer = Computer(build_step_diagram(), schedule)
    hybrid = HybridDiagram(computer=computer, plant=build_plant())
    hybrid.connect_boundary(
        direction="computer_to_plant",
        computer_port="u_cmd",
        plant_port="u",
    )
    hybrid.connect_boundary(
        direction="plant_to_computer",
        computer_port="y_meas",
        plant_port="y",
    )
    return hybrid


hybrid = build_hybrid()
hybrid.computer.diagram.plot_diagram()

result = hybrid.compute_forced(
    reference,
    t0=0,
    tf=TF,
    input_port_id="r",
    plant_dt_inner=PLANT_DT_INNER,
)
result.plot(signals=("r", "y", "y_f", "u_cmd", "x_plant"))
