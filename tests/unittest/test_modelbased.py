"""Tests for model-based mechanical controllers."""

import unittest

import numpy as np

from minilink.control.modelbased import ComputedTorqueController, SlidingModeController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


class TestModelBasedControllers(unittest.TestCase):
    def test_computed_torque_pd_plus_inverse_dynamics(self):
        plant = Pendulum()
        ctl = ComputedTorqueController(plant, Kp=[30.0], Kd=[6.0])
        q = np.array([0.2])
        dq = np.array([0.1])
        r = np.array([0.5, 0.0])
        u = ctl.ctl(None, np.concatenate([r, q, dq]))
        qdd = 30.0 * (0.5 - 0.2) + 6.0 * (0.0 - 0.1)
        np.testing.assert_allclose(
            u,
            plant.inverse_dynamics(q, dq, np.array([qdd])),
        )

    def test_sliding_mode_output_dim(self):
        plant = Pendulum()
        ctl = SlidingModeController(plant)
        u = ctl.ctl(
            None,
            np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
        )
        self.assertEqual(u.shape, (1,))


if __name__ == "__main__":
    unittest.main()
