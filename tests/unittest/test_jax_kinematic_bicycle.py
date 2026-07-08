"""Optional JAX smoke tests for kinematic bicycle twins (skip if jax missing)."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402

from minilink.core.backends import configure_jax  # noqa: E402
from minilink.dynamics.catalog.vehicles.steering import (  # noqa: E402
    JaxKinematicBicycle,
    JaxKinematicBicycleRateInputs,
    KinematicBicycle,
)


@pytest.mark.optional
@pytest.mark.jax
class TestJaxKinematicBicycle(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def test_regular_f_is_jaxpr_traceable(self):
        sys = JaxKinematicBicycle()
        x = jnp.zeros(3)
        u = jnp.zeros(2)
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)

    def test_regular_f_matches_numpy_nominal(self):
        sys_j = JaxKinematicBicycle()
        sys_n = KinematicBicycle()
        x = jnp.zeros(3)
        u = jnp.array([2.0, 0.0])
        dx_j = sys_j.f(x, u)
        dx_n = sys_n.f(np.zeros(3), np.array([2.0, 0.0]))
        np.testing.assert_allclose(np.asarray(dx_j), dx_n, rtol=1e-5, atol=1e-5)

    def test_regular_f_matches_numpy_nontrivial(self):
        sys_j = JaxKinematicBicycle()
        sys_n = KinematicBicycle()
        x = jnp.array([1.0, -0.5, 0.4])
        u = jnp.array([3.0, 0.25])
        dx_j = sys_j.f(x, u)
        dx_n = sys_n.f(np.array([1.0, -0.5, 0.4]), np.array([3.0, 0.25]))
        np.testing.assert_allclose(np.asarray(dx_j), dx_n, rtol=1e-5, atol=1e-5)

    def test_rate_f_is_jaxpr_traceable(self):
        sys = JaxKinematicBicycleRateInputs()
        x = jnp.zeros(5)
        u = jnp.zeros(2)
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)

    def test_rate_f_integrates_command_rates(self):
        sys = JaxKinematicBicycleRateInputs()
        x = jnp.array([0.0, 0.0, 0.0, 2.0, 0.1])
        u = jnp.array([0.5, -0.2])
        dx = np.asarray(sys.f(x, u))
        self.assertEqual(dx.shape, (5,))
        np.testing.assert_allclose(dx[3:], np.array([0.5, -0.2]))

    def test_rate_f_pose_derivative_matches_regular_model(self):
        sys_rate = JaxKinematicBicycleRateInputs()
        sys_reg = JaxKinematicBicycle()
        x_rate = jnp.array([1.0, 2.0, 0.3, 4.0, -0.15])
        u_rate = jnp.zeros(2)
        dx_rate = np.asarray(sys_rate.f(x_rate, u_rate))
        dx_reg = np.asarray(sys_reg.f(x_rate[:3], jnp.array([x_rate[3], x_rate[4]])))
        np.testing.assert_allclose(dx_rate[:3], dx_reg, rtol=1e-5, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
