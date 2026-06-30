import unittest

import numpy as np

from minilink.dynamics.abstraction.manipulator import Manipulator
from minilink.dynamics.abstraction.mechanical import MechanicalSystem


class TestMechanicalJointPorts(unittest.TestCase):
    def test_q_and_dq_ports_match_state(self):
        sys = MechanicalSystem(dof=2)
        x = np.array([0.1, -0.2, 0.3, 0.4])

        q = sys.h_q(x, np.zeros(sys.m))
        dq = sys.h_dq(x, np.zeros(sys.m))

        np.testing.assert_allclose(q, x[:2])
        np.testing.assert_allclose(dq, x[2:])
        self.assertEqual(sys.outputs["q"].dim, 2)
        self.assertEqual(sys.outputs["dq"].dim, 2)


class TestManipulator(unittest.TestCase):
    def test_default_kinematics_ports_are_zero(self):
        sys = Manipulator(dof=2, task_dim=2)
        x = np.array([0.2, -0.1, 0.0, 0.0])
        u = np.zeros(sys.m)

        p = sys.h_p(x, u)
        pdot = sys.h_pdot(x, u)

        np.testing.assert_allclose(p, np.zeros(2))
        np.testing.assert_allclose(pdot, np.zeros(2))

    def test_subclass_forward_kinematics_and_jacobian(self):
        class TwoLinkStub(Manipulator):
            def forward_kinematics(self, q, params=None):
                return np.array([q[0], q[0] + q[1]])

            def J(self, q, params=None):
                return np.array([[1.0, 0.0], [1.0, 1.0]])

        sys = TwoLinkStub(dof=2, task_dim=2)
        q = np.array([0.5, -0.25])
        dq = np.array([1.0, 2.0])
        x = sys.q2x(q, dq)
        u = np.zeros(sys.m)

        np.testing.assert_allclose(sys.h_p(x, u), np.array([0.5, 0.25]))
        np.testing.assert_allclose(sys.h_pdot(x, u), np.array([1.0, 3.0]))


if __name__ == "__main__":
    unittest.main()
