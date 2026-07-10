"""Tests for :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.core.step_rollout import StepRollout
from minilink.core.trajectory import Trajectory
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

        self.assertEqual(result.computer.x.shape[0], hybrid.computer.diagram.n)
        self.assertEqual(result.plant.x.shape[0], hybrid.plant.n)
        self.assertEqual(result.computer.n_samples, sim.n_ticks)
        self.assertGreaterEqual(result.plant.n_samples, sim.n_ticks)
        self.assertIsInstance(result.computer, StepRollout)
        self.assertIsInstance(result.plant, Trajectory)
        self.assertIn("u_cmd", result.computer.signals)

    def test_one_tick_measurement_delay(self):
        hybrid = _build_hybrid()
        sim = HybridSimulator(hybrid, t0=0, tf=0.05, n_steps=5)
        result = sim.solve_forced(np.array([1.0]), input_port_id="r")

        # Controller at tick 0 sees zero measurement from the initial sample buffer.
        self.assertAlmostEqual(float(result.computer.signals["u_cmd"][0, 0]), 0.5)

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
            signals=("r", "y", "u_cmd", "x"),
            show=False,
        )

    def test_plot_smoke_default_computer_out_u(self):
        """``hybrid_closed_loop`` default ``computer_out='u'`` must not clash with Trajectory.u."""
        import matplotlib

        matplotlib.use("Agg")

        from minilink.control.modelbased import SlidingModeController
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

        plant = Pendulum(length=1.0, mass=1.0)
        smc = SlidingModeController(plant)
        hybrid = hybrid_closed_loop(
            smc, plant, schedule=0.02, computer_in="y", plant_out="y"
        )
        hybrid.compute_forced(
            np.array([0.0, 0.0]),
            t0=0.0,
            tf=0.1,
            input_port_id="r",
            verbose=False,
        )
        hybrid.plot_trajectory(show=False)

    def test_plot_defaults_states_and_inputs_only(self):
        from minilink.control.modelbased import SlidingModeController
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
        from minilink.graphical.signals.time_signals import (
            build_signal_plot_spec,
            resolve_plot_signals,
        )

        plant = Pendulum(length=1.0, mass=1.0)
        smc = SlidingModeController(plant)
        hybrid = hybrid_closed_loop(
            smc, plant, schedule=0.02, computer_in="y", plant_out="y"
        )
        result = hybrid.compute_forced(
            np.array([0.0, 0.0]),
            t0=0.0,
            tf=0.1,
            input_port_id="r",
            verbose=False,
        )
        self.assertIn("y", result.plant.signals)
        signals = resolve_plot_signals(hybrid.plant)
        if result.plant.u.shape[0]:
            signals = tuple(dict.fromkeys((*signals, "u")))
        spec = build_signal_plot_spec(
            hybrid.plant,
            result.plant,
            signals=signals,
        )
        plotted = {trace.signal for trace in spec.traces}
        self.assertIn("x", plotted)
        self.assertIn("u", plotted)
        self.assertNotIn("y", plotted)
        self.assertNotIn("r", plotted)

    def test_compute_forced_caches_traj(self):
        hybrid = _build_hybrid()
        self.assertIsNone(hybrid.traj)
        self.assertIsNone(hybrid.last_result)
        result = hybrid.compute_forced(
            np.array([1.0]),
            t0=0,
            tf=0.05,
            input_port_id="r",
            verbose=False,
        )
        self.assertIs(hybrid.last_result, result)
        self.assertIs(hybrid.traj, result.plant)
        self.assertIs(hybrid.rollout, result.computer)

    def test_animate_uses_plant_traj(self):
        hybrid = _build_hybrid()
        hybrid.compute_forced(
            np.array([1.0]),
            t0=0,
            tf=0.05,
            input_port_id="r",
            verbose=False,
        )
        hybrid.animate(show=False)

    def test_plant_diagram_inherits_camera_scale(self):
        from minilink.core.hybrid_composition import _as_plant_diagram
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

        plant = Pendulum(length=1.0, mass=1.0)
        plant_diagram = _as_plant_diagram(plant)
        self.assertEqual(plant_diagram.camera_scale, plant.camera_scale)

    def test_hybrid_animate_refreshes_plant_camera_scale(self):
        from minilink.blocks.basic import Integrator
        from minilink.control.output import ProportionalController
        from minilink.core.hybrid_composition import hybrid_closed_loop

        plant = Integrator()
        plant.camera_scale = 3.0
        hybrid = hybrid_closed_loop(ProportionalController(0.5), plant, schedule=0.01)
        self.assertEqual(hybrid.plant.camera_scale, 3.0)
        plant.camera_scale = 9.0
        hybrid.animate(show=False)
        self.assertEqual(hybrid.plant.camera_scale, 9.0)


if __name__ == "__main__":
    unittest.main()
