"""Multi-rate hybrid simulation: scheduled computer + continuous plant."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.backends import array_module
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.hybrid_diagram import HybridDiagram
from minilink.core.system import StepSystem
from minilink.simulation.computer import Computer, StepSchedule
from minilink.simulation.hybrid_simulator import HybridSimulator


class DiscreteLowPass(StepSystem):
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


def _build_step_diagram():
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


def _build_plant():
    plant = DiagramSystem()
    plant.add_subsystem(Integrator(), "plant")
    plant.add_input_port("u")
    plant.connect("input", "u", "plant", "u")
    plant.connect_new_output_port("plant", "y", "y")
    return plant


def _build_hybrid():
    schedule = StepSchedule.from_rates(
        dt_base=0.01,
        rates_hz={"filter": 100.0, "ctl": 10.0},
    )
    computer = Computer(_build_step_diagram(), schedule)
    plant = _build_plant()
    hybrid = HybridDiagram(computer=computer, plant=plant)
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


def _reference(k):
    return np.array([1.0 if k < 60 else 0.0])


class TestHybridMultiRate(unittest.TestCase):
    def test_scheduled_hybrid_runs(self):
        hybrid = _build_hybrid()
        sim = HybridSimulator(hybrid, t0=0, tf=1.6)
        result = sim.solve_forced(_reference, input_port_id="r")

        self.assertEqual(result.computer.n_samples, 160)
        self.assertIn("y_f", result.computer.signals)
        self.assertIn("u_cmd", result.computer.signals)
        self.assertTrue(np.all(np.isfinite(result.plant.x)))

    def test_hand_loop_coarse_parity(self):
        hybrid = _build_hybrid()
        dt_base = hybrid.computer.schedule.dt_base
        plant_eval = hybrid.plant.compile()
        computer = hybrid.computer
        computer.compile()
        computer.reset()

        x_plant = np.zeros(1)
        u_nom = hybrid.plant.get_u_from_input_ports()
        sample = plant_eval.outputs(x_plant, u_nom, 0.0)
        y_sample = float(sample["y"][0])

        n_steps = 20
        y_hand = []
        for _ in range(n_steps):
            k = computer.k
            u_computer = hybrid.computer.diagram.get_u_from_input_ports().copy()
            u_computer[hybrid.computer.diagram.get_input_port_slice("r")] = _reference(
                k
            )
            u_computer[hybrid.computer.diagram.get_input_port_slice("y_meas")] = (
                y_sample
            )
            outs = computer.tick(u_computer)
            u_plant = np.array([float(outs["u_cmd"][0])])
            x_plant = plant_eval.integrate_zoh(x_plant, u_plant, k * dt_base, dt_base)
            y_sample = float(
                plant_eval.outputs(x_plant, u_plant, (k + 1) * dt_base)["y"][0]
            )
            y_hand.append(y_sample)

        sim = HybridSimulator(hybrid, t0=0, n_steps=n_steps)
        result = sim.solve_forced(_reference, input_port_id="r")
        np.testing.assert_allclose(
            result.computer.signals["y_meas"][0, :n_steps],
            y_hand,
            rtol=1e-6,
        )


if __name__ == "__main__":
    unittest.main()
