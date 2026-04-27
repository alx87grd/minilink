import unittest

import numpy as np

from minilink.dynamics.abstraction.mechanical import MechanicalSystem


class TestMechanicalSystem(unittest.TestCase):
    def test_dimensions_fully_actuated(self):
        sys = MechanicalSystem(dof=2)
        self.assertEqual(sys.dof, 2)
        self.assertEqual(sys.n, 4)
        self.assertEqual(sys.m, 2)
        self.assertEqual(sys.p, 4)

    def test_dimensions_custom_actuators(self):
        sys = MechanicalSystem(dof=3, actuators=2)
        self.assertEqual(sys.m, 2)
        self.assertEqual(sys.n, 6)

    def test_f_double_integrator_chain(self):
        """Default H=I, C=g=d=0, u=0 => ddq=0, dx = [dq; 0]."""
        sys = MechanicalSystem(dof=2)
        x = np.array([0.1, -0.2, 0.3, 0.4])
        u = np.zeros(2)
        dx = sys.f(x, u, t=0.0)
        self.assertEqual(dx.shape, (4,))
        np.testing.assert_allclose(dx[:2], x[2:4])
        np.testing.assert_allclose(dx[2:4], 0.0)

    def test_f_constant_acceleration(self):
        sys = MechanicalSystem(dof=1)
        x = np.array([0.0, 0.0])
        u = np.array([2.0])
        dx = sys.f(x, u)
        np.testing.assert_allclose(dx, np.array([0.0, 2.0]))

    def test_f_uses_explicit_params_in_matrix_hooks(self):
        class MassSystem(MechanicalSystem):
            def __init__(self):
                super().__init__(dof=1)
                self.params = {"mass": 2.0}

            def H(self, q, params=None):
                return np.array([[params["mass"]]])

        sys = MassSystem()
        x = np.array([0.0, 0.0])
        u = np.array([8.0])

        np.testing.assert_allclose(sys.f(x, u), np.array([0.0, 4.0]))
        np.testing.assert_allclose(
            sys.f(x, u, params={"mass": 4.0}), np.array([0.0, 2.0])
        )

    def test_inverse_dynamics_uses_explicit_params_in_matrix_hooks(self):
        class MassSystem(MechanicalSystem):
            def __init__(self):
                super().__init__(dof=1)
                self.params = {"mass": 2.0}

            def H(self, q, params=None):
                return np.array([[params["mass"]]])

        sys = MassSystem()
        q = np.array([0.0])
        dq = np.array([0.0])
        ddq = np.array([3.0])

        np.testing.assert_allclose(sys.actuator_forces(q, dq, ddq), np.array([6.0]))
        np.testing.assert_allclose(
            sys.actuator_forces(q, dq, ddq, params={"mass": 5.0}),
            np.array([15.0]),
        )

    def test_h_equals_state(self):
        sys = MechanicalSystem(dof=1)
        x = np.array([1.0, 2.0])
        y = sys.h(x, np.zeros(1))
        np.testing.assert_array_equal(y, x)

    def test_y_port_dependencies(self):
        sys = MechanicalSystem(dof=1)
        self.assertEqual(sys.outputs["y"].dependencies, ())


if __name__ == "__main__":
    unittest.main()
