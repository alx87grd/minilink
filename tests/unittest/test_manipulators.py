"""Cross-check all catalog manipulator classes."""

import unittest

import numpy as np

from minilink.dynamics.abstraction.manipulator import Manipulator
from minilink.dynamics.catalog.manipulators.arms import (
    FiveLinkPlanarManipulator,
    OneLinkManipulator,
    SpeedControlledManipulator,
    ThreeLinkManipulator3D,
    TwoLinkManipulator,
    _planar_joint_positions,
)

_MANIPULATORS = (
    OneLinkManipulator,
    TwoLinkManipulator,
    ThreeLinkManipulator3D,
    FiveLinkPlanarManipulator,
)


class TestManipulatorCatalog(unittest.TestCase):
    def test_all_subclass_manipulator_with_task_ports(self):
        for cls in _MANIPULATORS:
            with self.subTest(cls=cls.__name__):
                arm = cls()
                self.assertIsInstance(arm, Manipulator)
                for port in ("q", "dq", "p", "pdot", "y"):
                    self.assertIn(port, arm.outputs)

    def test_camera_scale_matches_reach_not_default(self):
        for cls in _MANIPULATORS:
            with self.subTest(cls=cls.__name__):
                arm = cls()
                self.assertLess(arm.camera_scale, 10.0)

        speed = SpeedControlledManipulator(2, 2)
        self.assertLess(speed.camera_scale, 10.0)

    def test_h_p_and_h_pdot_match_kinematics(self):
        for cls in _MANIPULATORS:
            with self.subTest(cls=cls.__name__):
                arm = cls()
                dof = arm.dof
                q = np.linspace(-0.4, 0.4, dof)
                dq = np.linspace(-0.3, 0.3, dof)
                x = arm.q2x(q, dq)
                np.testing.assert_allclose(
                    arm.h_p(x, np.zeros(dof)),
                    arm.forward_kinematics(q),
                )
                np.testing.assert_allclose(
                    arm.h_pdot(x, np.zeros(dof)),
                    arm.J(q) @ dq,
                )

    def test_jacobian_matches_numeric_forward_kinematics(self):
        eps = 1e-7
        for cls in _MANIPULATORS:
            with self.subTest(cls=cls.__name__):
                arm = cls()
                dof = arm.dof
                q = np.linspace(-0.3, 0.3, dof)
                J = arm.J(q)
                Jnum = np.column_stack(
                    [
                        (
                            arm.forward_kinematics(q + eps * e)
                            - arm.forward_kinematics(q - eps * e)
                        )
                        / (2 * eps)
                        for e in np.eye(dof)
                    ]
                )
                np.testing.assert_allclose(J, Jnum, atol=1e-5, rtol=1e-5)

    def test_two_link_fk_matches_planar_geometry_tip(self):
        arm = TwoLinkManipulator()
        q = np.array([0.2, -0.1])
        tip, _ = _planar_joint_positions(q, arm._lengths())
        np.testing.assert_allclose(arm.forward_kinematics(q), tip[-1])

    def test_three_link_fk_matches_tf_end_effector(self):
        arm = ThreeLinkManipulator3D()
        q = np.array([0.1, -0.2, 0.15])
        x = arm.q2x(q, np.zeros(3))
        fk = arm.forward_kinematics(q)
        tip = arm.tf(x, np.zeros(3))["joint3"][:3, 3]
        np.testing.assert_allclose(fk, tip)

    def test_five_link_uses_placeholder_inertia(self):
        arm = FiveLinkPlanarManipulator()
        np.testing.assert_allclose(arm.H(np.zeros(5)), np.eye(5))
        np.testing.assert_allclose(
            arm.inverse_dynamics(np.zeros(5), np.zeros(5), np.ones(5)),
            np.ones(5),
        )

    def test_torque_arrow_sign_matches_gravity_hold(self):
        """Static hold uses ``tau = g(q)``; arc sweep is ``-u`` (Pyro / double-pendulum)."""
        arm = TwoLinkManipulator()
        q = np.array([0.8, -0.5])
        x = arm.q2x(q, np.zeros(2))
        tau = arm.g(q)
        self.assertTrue(np.all(tau < 0.0))
        geom = arm.get_dynamic_geometry(x, tau)
        for i in range(arm.dof):
            self.assertGreater(geom[f"link{i}"][0].sweep, 0.0)
            np.testing.assert_allclose(
                geom[f"link{i}"][0].sweep,
                -tau[i] * (2.0 * np.pi / 3.0) / max(abs(arm.inputs["u"].upper_bound[i]), 1.0),
            )

    def test_from_manipulator_inherits_kinematics(self):
        source = TwoLinkManipulator()
        speed = SpeedControlledManipulator.from_manipulator(source)
        q = np.array([0.2, -0.1])
        np.testing.assert_allclose(speed.forward_kinematics(q), source.forward_kinematics(q))
        np.testing.assert_allclose(speed.J(q), source.J(q))
        self.assertEqual(speed.n, 2)
        np.testing.assert_allclose(speed.f(q, np.array([0.3, -0.2])), [0.3, -0.2])

    def test_speed_plant_p_port(self):
        speed = SpeedControlledManipulator.from_manipulator(TwoLinkManipulator())
        q = np.array([0.2, -0.1])
        np.testing.assert_allclose(
            speed.h_p(q, np.zeros(2)),
            speed.forward_kinematics(q),
        )


if __name__ == "__main__":
    unittest.main()
