"""Tests for JAX physics engine core (skip if jax is unavailable)."""

import unittest

import numpy as np
import pytest

try:
    import jax
    import jax.numpy as jnp

    from minilink.physics.engine_jax import (
        PlaneModel,
        SphereModel,
        make_world_model,
        pack_state,
        plane_contact_force,
        unpack_state,
        world_ode,
    )

    HAS_JAX = True
except ImportError:
    HAS_JAX = False


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(HAS_JAX, "jax not installed")
class TestPhysicsEngineJax(unittest.TestCase):
    def _single_world(self):
        return make_world_model(
            [SphereModel(mass=2.0, radius=0.5)],
            PlaneModel(normal=(0.0, 0.0, 1.0), offset=0.0),
            gravity=(0.0, 0.0, -9.81),
            k_contact=1000.0,
            c_contact=50.0,
        )

    def test_pack_unpack_roundtrip(self):
        pos = jnp.array([[0.0, 1.2, 0.0]])
        quat = jnp.array([[2.0, 0.0, 0.0, 0.0]])
        vel = jnp.array([[0.1, 0.2, 0.3]])
        omg = jnp.array([[0.3, -0.1, 0.2]])
        x = pack_state(pos, quat, vel, omg)
        p2, q2, v2, w2 = unpack_state(x, 1)
        np.testing.assert_allclose(np.asarray(p2), np.asarray(pos))
        np.testing.assert_allclose(np.asarray(q2), np.asarray(quat))
        np.testing.assert_allclose(np.asarray(v2), np.asarray(vel))
        np.testing.assert_allclose(np.asarray(w2), np.asarray(omg))

    def test_plane_contact_zero_without_penetration(self):
        world = self._single_world()
        pos = jnp.array([[0.0, 0.0, 1.0]])
        vel = jnp.array([[0.0, 0.0, 0.0]])
        f = plane_contact_force(world, pos, vel)
        np.testing.assert_allclose(np.asarray(f), np.zeros((1, 3)), atol=1e-12)

    def test_plane_contact_pushes_up_on_penetration(self):
        world = self._single_world()
        pos = jnp.array([[0.0, 0.0, 0.1]])  # penetration = 0.4 for r=0.5
        vel = jnp.array([[0.0, 0.0, 0.0]])
        f = plane_contact_force(world, pos, vel)
        self.assertGreater(float(f[0, 2]), 0.0)

    def test_world_ode_jax_traceable_and_shape(self):
        world = self._single_world()
        x = pack_state(
            jnp.array([[0.0, 0.0, 0.2]]),
            jnp.array([[1.0, 0.0, 0.0, 0.0]]),
            jnp.array([[0.0, 0.0, 0.0]]),
            jnp.array([[0.0, 0.0, 0.0]]),
        )
        u = jnp.zeros(6)
        dx = world_ode(world, x, u)
        self.assertEqual(dx.shape, (13,))
        jax.make_jaxpr(lambda xx, uu: world_ode(world, xx, uu))(x, u)


if __name__ == "__main__":
    unittest.main()
