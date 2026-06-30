"""Tests for manipulator task ports and robotic wrappers."""

import unittest

import numpy as np

from minilink.control.robotic import JointImpedance, ModelJointImpedance, TaskImpedance
from minilink.core.composition import closed_loop_qdq
from minilink.dynamics.catalog.manipulators.arms import OneLinkManipulator, TwoLinkManipulator


class TestManipulatorPorts(unittest.TestCase):
    def test_h_p_matches_forward_kinematics(self):
        arm = OneLinkManipulator()
        q = np.array([0.3])
        x = arm.q2x(q, np.zeros(1))
        np.testing.assert_allclose(arm.h_p(x, np.zeros(1)), arm.forward_kinematics(q))

    def test_h_pdot_matches_jacobian(self):
        arm = TwoLinkManipulator()
        q = np.array([0.2, -0.1])
        dq = np.array([0.5, -0.3])
        x = arm.q2x(q, dq)
        np.testing.assert_allclose(arm.h_pdot(x, np.zeros(2)), arm.J(q) @ dq)


class TestRoboticWrappers(unittest.TestCase):
    def test_joint_impedance_closed_loop_qdq(self):
        diagram = closed_loop_qdq(JointImpedance(dof=2), TwoLinkManipulator())
        self.assertIn("mux", diagram.subsystems)
        self.assertEqual(
            diagram.connections["ctl"]["y"],
            ("mux", "y"),
        )

    def test_model_joint_impedance_gravity_at_setpoint(self):
        arm = TwoLinkManipulator()
        ctl = JointImpedance(arm, gravity_comp=True)
        q = np.array([0.8, -0.5])
        r = q.copy()
        u = ctl.ctl(None, np.concatenate([r, q, np.zeros(2)]))
        np.testing.assert_allclose(u, arm.g(q))

    def test_model_joint_impedance_no_gravity_by_default(self):
        arm = TwoLinkManipulator()
        ctl = JointImpedance(arm)
        q = np.array([0.8, -0.5])
        u = ctl.ctl(None, np.concatenate([q, q, np.zeros(2)]))
        np.testing.assert_allclose(u, np.zeros(2))

    def test_model_joint_impedance_custom_gravity_hook(self):
        arm = TwoLinkManipulator()
        ctl = ModelJointImpedance(arm, gravity_comp=True, gravity=lambda q: np.array([1.0, 2.0]))
        q = np.array([0.2, 0.3])
        u = ctl.ctl(None, np.concatenate([q, q, np.zeros(2)]))
        np.testing.assert_allclose(u, [1.0, 2.0])

    def test_joint_impedance_gravity_requires_plant(self):
        with self.assertRaises(ValueError):
            JointImpedance(dof=2, gravity_comp=True)

    def test_task_impedance_uses_internal_kinematics(self):
        arm = TwoLinkManipulator()
        ctl = TaskImpedance(arm)
        q = np.array([0.2, -0.1])
        dq = np.array([0.1, 0.05])
        p_d = arm.forward_kinematics(np.array([0.0, 0.0]))
        u = ctl.ctl(None, np.concatenate([p_d, q, dq]))
        self.assertEqual(u.shape, (2,))

        p = arm.forward_kinematics(q)
        J = arm.J(q)
        pdot = J @ dq
        e_p = p_d - p
        expected = J.T @ (ctl.params["Kp"] * e_p - ctl.params["Kd"] * pdot)
        np.testing.assert_allclose(u, expected)

    def test_task_impedance_gravity_feedforward(self):
        arm = TwoLinkManipulator()
        ctl = TaskImpedance(arm, gravity_comp=True)
        q = np.array([0.8, -0.5])
        p_d = arm.forward_kinematics(q)
        u = ctl.ctl(None, np.concatenate([p_d, q, np.zeros(2)]))
        np.testing.assert_allclose(u, arm.g(q))

    def test_task_impedance_closed_loop_qdq(self):
        diagram = closed_loop_qdq(TaskImpedance(TwoLinkManipulator()), TwoLinkManipulator())
        self.assertIn("mux", diagram.subsystems)
        self.assertEqual(
            diagram.connections["ctl"]["y"],
            ("mux", "y"),
        )


if __name__ == "__main__":
    unittest.main()
