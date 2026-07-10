"""Fine plant trajectory recording in hybrid simulation."""

import unittest

import numpy as np

from minilink.control.modelbased import SlidingModeController
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.core.step_rollout import StepRollout
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.simulation.hybrid_simulator import HybridSimulator


class TestHybridFineRecording(unittest.TestCase):
    def test_plant_records_finer_than_computer(self):
        plant = Pendulum(length=1.0, mass=1.0)
        plant.x0 = np.array([0.5, 0.0])
        ctl = SlidingModeController(plant, lam=2.0, gain=4.0, nab=0.1)
        hybrid = hybrid_closed_loop(ctl, plant, schedule=0.1, computer_in="y", plant_out="y")

        sim = HybridSimulator(
            hybrid,
            t0=0.0,
            tf=0.5,
            plant_dt_inner=0.01,
        )
        result = sim.solve_forced(np.array([0.0, 0.0]), input_port_id="r")

        self.assertIsInstance(result.computer, StepRollout)
        self.assertIsInstance(result.plant, Trajectory)
        self.assertGreater(result.plant.n_samples, result.computer.n_samples)
        self.assertEqual(result.computer.n_samples, sim.n_ticks)

        dt_median = np.median(np.diff(result.plant.t))
        self.assertAlmostEqual(dt_median, 0.01, places=5)

        tick_end = hybrid.computer.schedule.dt_base * sim.n_ticks
        np.testing.assert_allclose(result.plant.t[0], 0.0)
        np.testing.assert_allclose(result.plant.t[-1], tick_end, rtol=1e-9)


if __name__ == "__main__":
    unittest.main()
