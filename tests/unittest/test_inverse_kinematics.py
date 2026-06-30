"""Tests for numerical inverse kinematics on Manipulator."""

import unittest

import numpy as np

from minilink.dynamics.catalog.manipulators.arms import (
    OneLinkManipulator,
    TwoLinkManipulator,
)


class TestInverseKinematics(unittest.TestCase):
    def test_one_link_roundtrip(self):
        arm = OneLinkManipulator()
        q_true = np.array([0.4])
        p = arm.forward_kinematics(q_true)
        q = arm.inverse_kinematics(p, np.array([0.0]))
        np.testing.assert_allclose(arm.forward_kinematics(q), p, atol=1e-8)
        np.testing.assert_allclose(q, q_true, atol=1e-8)

    def test_default_q_guess_uses_q_nominal(self):
        arm = OneLinkManipulator()
        q_true = np.array([0.4])
        p = arm.forward_kinematics(q_true)
        q = arm.inverse_kinematics(p)
        np.testing.assert_allclose(arm.forward_kinematics(q), p, atol=1e-8)
        np.testing.assert_allclose(q, q_true, atol=1e-8)

    def test_two_link_roundtrip(self):
        arm = TwoLinkManipulator()
        q_true = np.array([0.5, -0.3])
        p = arm.forward_kinematics(q_true)
        q = arm.inverse_kinematics(p, q_true + 0.05)
        np.testing.assert_allclose(arm.forward_kinematics(q), p, atol=1e-8)

    def test_two_link_different_branch_from_guess(self):
        arm = TwoLinkManipulator()
        p = arm.forward_kinematics(np.array([0.6, -0.2]))
        q_a = arm.inverse_kinematics(p, np.array([0.2, 0.2]))
        q_b = arm.inverse_kinematics(p, np.array([1.0, -1.0]))
        np.testing.assert_allclose(arm.forward_kinematics(q_a), p, atol=1e-8)
        np.testing.assert_allclose(arm.forward_kinematics(q_b), p, atol=1e-8)
        self.assertGreater(np.linalg.norm(q_a - q_b), 0.1)


if __name__ == "__main__":
    unittest.main()
