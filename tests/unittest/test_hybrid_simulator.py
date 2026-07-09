"""Tests for :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.simulation.hybrid_simulator import HybridSimulator


def _build_hybrid():
    step_diagram = StepDiagramSystem()
    step_diagram.add_subsystem(ProportionalController(0.5), "ctl")
    step_diagram.add_input_port("r")
    step_diagram.add_input_port("y")
    step_diagram.connect("input", "r", "ctl", "r")
    step_diagram.connect("input", "y", "ctl", "y")
    step_diagram.connect_new_output_port("ctl", "u", "u_cmd")

    plant = DiagramSystem()
    plant.add_subsystem(Integrator(), "plant")
    plant.add_input_port("u")
    plant.connect("input", "u", "plant", "u")
    plant.connect_new_output_port("plant", "y", "y")

    return hybrid_closed_loop(
        step_diagram,
        plant,
        schedule=0.01,
        computer_out="u_cmd",
    )


class TestHybridSimulator(unittest.TestCase):
    def test_records_both_states(self):
        hybrid = _build_hybrid()
        sim = HybridSimulator(hybrid, t0=0, tf=0.2)
        result = sim.solve_forced(np.array([1.0]), input_port_id="r")

        self.assertEqual(result.x_computer.shape[0], hybrid.computer.diagram.n)
        self.assertEqual(result.x_plant.shape[0], hybrid.plant.n)
        self.assertEqual(result.n_samples, sim.n_ticks)
        self.assertIn("u_cmd", result.signals)

    def test_one_tick_measurement_delay(self):
        hybrid = _build_hybrid()
        sim = HybridSimulator(hybrid, t0=0, tf=0.05, n_steps=5)
        result = sim.solve_forced(np.array([1.0]), input_port_id="r")

        # Controller at tick 0 sees zero measurement from the initial sample buffer.
        self.assertAlmostEqual(float(result.signals["u_cmd"][0, 0]), 0.5)

    def test_plot_smoke(self):
        import matplotlib

        matplotlib.use("Agg")

        hybrid = _build_hybrid()
        hybrid.compute_forced(
            np.array([1.0]),
            t0=0,
            tf=0.1,
            input_port_id="r",
        )
        hybrid.plot_trajectory(
            signals=("r", "y", "u_cmd", "x_plant"),
            show=False,
        )

    def test_compute_forced_caches_traj(self):
        hybrid = _build_hybrid()
        self.assertIsNone(hybrid.traj)
        result = hybrid.compute_forced(
            np.array([1.0]),
            t0=0,
            tf=0.05,
            input_port_id="r",
        )
        self.assertIs(hybrid.traj, result)


if __name__ == "__main__":
    unittest.main()
