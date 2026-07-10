"""Tests for model-based mechanical controllers."""

import unittest

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.modelbased import ComputedTorqueController, SlidingModeController
from minilink.core.composition import closed_loop_qdq
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

    def test_sliding_mode_matches_pyro_law(self):
        plant = Pendulum(length=1.0, mass=1.0)
        lam = np.array([2.0])
        gain = np.array([3.0])
        nab = np.array([0.2])
        ctl = SlidingModeController(plant, lam=lam, gain=gain, nab=nab)

        q = np.array([0.3])
        dq = np.array([-0.15])
        q_d = np.array([0.0])
        dq_d = np.array([0.0])
        r = np.concatenate([q_d, dq_d])
        boundary = np.concatenate([r, q, dq])

        q_e = q - q_d
        dq_e = dq - dq_d
        s = dq_e + lam * q_e
        ddq_r = -lam * dq_e
        H = plant.H(q)
        K = np.diag(gain) + H @ np.diag(nab)
        expected = plant.inverse_dynamics(q, dq, ddq_r) - K @ np.sign(s)

        np.testing.assert_allclose(ctl.ctl(None, boundary), expected)

    def test_sliding_mode_ports_match_computed_torque(self):
        plant = Pendulum()
        ctl = SlidingModeController(plant)
        self.assertIn("r", ctl.inputs)
        self.assertIn("y", ctl.inputs)
        self.assertIn("u", ctl.outputs)
        self.assertEqual(ctl.inputs["y"].dim, 2)

    def test_sliding_mode_sets_discontinuous_behavior(self):
        ctl = SlidingModeController(Pendulum())
        self.assertTrue(ctl.solver_info["discontinuous_behavior"])

    def test_closed_loop_qdq_aggregates_discontinuous_behavior(self):
        plant = Pendulum()
        smc = SlidingModeController(plant)
        ref = Step(
            initial_value=np.zeros(2),
            final_value=np.zeros(2),
            step_time=1.0,
        )
        diagram = ref >> closed_loop_qdq(smc, plant)
        self.assertTrue(diagram.solver_info["discontinuous_behavior"])


if __name__ == "__main__":
    unittest.main()
