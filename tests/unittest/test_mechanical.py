import unittest

import numpy as np
import pytest

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
        """Default H=I, C=g=d=0, u=0 => acceleration=0, dx = [dq; 0]."""
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
        acceleration = np.array([3.0])

        np.testing.assert_allclose(
            sys.inverse_dynamics(q, dq, acceleration), np.array([6.0])
        )
        np.testing.assert_allclose(
            sys.inverse_dynamics(q, dq, acceleration, params={"mass": 5.0}),
            np.array([15.0]),
        )

    def test_default_generalized_force_is_actuator_map(self):
        sys = MechanicalSystem(dof=2, actuators=1)
        q = np.array([0.0, 0.0])
        dq = np.array([0.0, 0.0])
        u = np.array([3.0])

        np.testing.assert_allclose(
            sys.generalized_force(q, dq, u),
            np.array([3.0, 0.0]),
        )

    def test_forward_and_inverse_dynamics_are_consistent(self):
        class MassSystem(MechanicalSystem):
            def H(self, q, params=None):
                return np.array([[2.0]])

            def d(self, q, dq, u=None, t=0.0, params=None):
                return np.array([0.5 * dq[0]])

        sys = MassSystem(dof=1)
        q = np.array([0.0])
        dq = np.array([4.0])
        u = np.array([10.0])

        acceleration = sys.forward_dynamics(q, dq, u)

        np.testing.assert_allclose(acceleration, np.array([4.0]))
        np.testing.assert_allclose(
            sys.inverse_dynamics(q, dq, acceleration, u),
            sys.generalized_force(q, dq, u),
        )

    def test_h_equals_state(self):
        sys = MechanicalSystem(dof=1)
        x = np.array([1.0, 2.0])
        y = sys.h(x, np.zeros(1))
        np.testing.assert_array_equal(y, x)

    def test_y_port_dependencies(self):
        sys = MechanicalSystem(dof=1)
        self.assertEqual(sys.outputs["y"].dependencies, ())

    @pytest.mark.optional
    @pytest.mark.jax
    def test_default_base_is_jax_traceable_if_available(self):
        jax = pytest.importorskip("jax")
        jnp = pytest.importorskip("jax.numpy")

        sys = MechanicalSystem(dof=2)
        x = jnp.array([0.1, -0.2, 0.3, 0.4])
        u = jnp.array([1.0, -2.0])

        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)
        np.testing.assert_allclose(
            np.asarray(sys.f(x, u)), np.array([0.3, 0.4, 1.0, -2.0])
        )


class TestJaxMechanicalInheritance(unittest.TestCase):
    def test_jax_mechanical_subclasses_numpy_mechanical(self):
        from minilink.dynamics.abstraction.mechanical import (
            JaxMechanicalSystem,
            MechanicalSystem,
        )

        self.assertTrue(issubclass(JaxMechanicalSystem, MechanicalSystem))

    def test_jax_mechanical_init_matches_numpy_layout(self):
        from minilink.dynamics.abstraction.mechanical import (
            JaxMechanicalSystem,
            MechanicalSystem,
        )

        a = MechanicalSystem(dof=3, actuators=2)
        b = JaxMechanicalSystem(dof=3, actuators=2)
        self.assertEqual((a.n, a.m, a.p), (b.n, b.m, b.p))
        self.assertEqual(a.state.labels, b.state.labels)


if __name__ == "__main__":
    unittest.main()
