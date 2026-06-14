import unittest

import numpy as np
import pytest

from minilink.dynamics.abstraction.generalized_mechanical import (
    GeneralizedMechanicalSystem,
)


class TestGeneralizedMechanicalSystem(unittest.TestCase):
    def test_dimensions_default_to_square_configuration_velocity_system(self):
        sys = GeneralizedMechanicalSystem(dof=2)

        self.assertEqual(sys.dof, 2)
        self.assertEqual(sys.pos, 2)
        self.assertEqual(sys.n, 4)
        self.assertEqual(sys.m, 2)
        self.assertEqual(sys.p, 4)

    def test_default_dynamics_are_double_integrator_chain(self):
        sys = GeneralizedMechanicalSystem(dof=2)
        x = np.array([0.1, -0.2, 0.3, 0.4])
        u = np.array([1.0, -2.0])

        dx = sys.f(x, u)

        np.testing.assert_allclose(dx, np.array([0.3, 0.4, 1.0, -2.0]))

    def test_nontrivial_kinematic_map(self):
        class PlanarBody(GeneralizedMechanicalSystem):
            def __init__(self):
                super().__init__(dof=3, pos=3, actuators=0)

            def N(self, q, params=None):
                theta = q[2]
                c, s = np.cos(theta), np.sin(theta)
                return np.array(
                    [
                        [c, -s, 0.0],
                        [s, c, 0.0],
                        [0.0, 0.0, 1.0],
                    ]
                )

        sys = PlanarBody()
        q = np.array([1.0, 2.0, np.pi / 2.0])
        v = np.array([1.0, 2.0, 3.0])
        x = sys.qv2x(q, v)
        u = np.zeros(0)

        dx = sys.f(x, u)

        np.testing.assert_allclose(dx[:3], np.array([-2.0, 1.0, 3.0]), atol=1e-12)
        np.testing.assert_allclose(dx[3:], np.zeros(3))

    def test_mixed_inputs_use_generalized_force_hook(self):
        class MixedInputBody(GeneralizedMechanicalSystem):
            def __init__(self):
                super().__init__(dof=2, pos=2, actuators=2)

            def generalized_force(self, q, v, u, t=0.0, params=None):
                thrust = u[0]
                angle = u[1]
                return np.array([thrust * np.cos(angle), thrust * np.sin(angle)])

        sys = MixedInputBody()
        x = np.zeros(4)
        u = np.array([2.0, np.pi / 2.0])

        dx = sys.f(x, u)

        np.testing.assert_allclose(dx[:2], np.zeros(2))
        np.testing.assert_allclose(dx[2:], np.array([0.0, 2.0]), atol=1e-12)

    def test_explicit_params_reach_matrix_hooks(self):
        class MassBody(GeneralizedMechanicalSystem):
            def __init__(self):
                super().__init__(dof=1)
                self.params = {"mass": 2.0}

            def M(self, q, params=None):
                return np.array([[params["mass"]]])

        sys = MassBody()
        x = np.array([0.0, 0.0])
        u = np.array([8.0])

        np.testing.assert_allclose(sys.f(x, u), np.array([0.0, 4.0]))
        np.testing.assert_allclose(
            sys.f(x, u, params={"mass": 4.0}), np.array([0.0, 2.0])
        )

    def test_default_generalized_force_is_actuator_map(self):
        sys = GeneralizedMechanicalSystem(dof=2, pos=2, actuators=1)
        q = np.array([0.0, 0.0])
        v = np.array([0.0, 0.0])
        u = np.array([3.0])

        np.testing.assert_allclose(
            sys.generalized_force(q, v, u),
            np.array([3.0, 0.0]),
        )

    def test_forward_and_inverse_dynamics_are_consistent(self):
        class MassBody(GeneralizedMechanicalSystem):
            def M(self, q, params=None):
                return np.array([[2.0]])

            def d(self, q, v, u=None, t=0.0, params=None):
                return np.array([0.5 * v[0]])

        sys = MassBody(dof=1)
        q = np.array([0.0])
        v = np.array([4.0])
        u = np.array([10.0])

        acceleration = sys.forward_dynamics(q, v, u)

        np.testing.assert_allclose(acceleration, np.array([4.0]))
        np.testing.assert_allclose(
            sys.inverse_dynamics(q, v, acceleration, u),
            sys.generalized_force(q, v, u),
        )

    def test_h_equals_state(self):
        sys = GeneralizedMechanicalSystem(dof=1)
        x = np.array([1.0, 2.0])
        y = sys.h(x, np.zeros(1))
        np.testing.assert_array_equal(y, x)

    @pytest.mark.optional
    @pytest.mark.jax
    def test_default_base_is_jax_traceable_if_available(self):
        jax = pytest.importorskip("jax")
        jnp = pytest.importorskip("jax.numpy")

        sys = GeneralizedMechanicalSystem(dof=2)
        x = jnp.array([0.1, -0.2, 0.3, 0.4])
        u = jnp.array([1.0, -2.0])

        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)
        np.testing.assert_allclose(
            np.asarray(sys.f(x, u)), np.array([0.3, 0.4, 1.0, -2.0])
        )


if __name__ == "__main__":
    unittest.main()
