"""Integration tests for physics DynamicSystem wrapper."""

import unittest

import numpy as np
import pytest

from minilink.core.diagram import DiagramSystem

try:
    import jax.numpy as jnp

    from minilink.physics import PhysicsWorldSystem
    from minilink.physics.engine_jax import PlaneModel, SphereModel, make_world_model

    HAS_JAX = True
except ImportError:
    HAS_JAX = False


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(HAS_JAX, "jax not installed")
class TestPhysicsSystemMinilink(unittest.TestCase):
    def _make_sys(self):
        world = make_world_model(
            [SphereModel(mass=1.0, radius=0.2), SphereModel(mass=1.5, radius=0.15)],
            PlaneModel(normal=(0.0, 0.0, 1.0), offset=0.0),
            gravity=(0.0, 0.0, -9.81),
            k_contact=5000.0,
            c_contact=80.0,
        )
        sys = PhysicsWorldSystem(world)
        # body0 above plane, body1 near plane
        sys.x0 = np.array(
            [
                0.0,
                0.0,
                1.0,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.2,
                0.0,
                0.1,
                1.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            dtype=float,
        )
        return sys

    def test_f_shape(self):
        sys = self._make_sys()
        dx = sys.f(sys.x0, np.zeros(sys.m), 0.0)
        self.assertEqual(dx.shape, (sys.n,))

    def test_geometry_transform_contract(self):
        sys = self._make_sys()
        prim = sys.get_kinematic_geometry()
        T = sys.get_kinematic_transforms(sys.x0, np.zeros(sys.m), 0.0)
        self.assertEqual(len(prim), len(T))

    def test_compile_jax_parity_one_step(self):
        sys = self._make_sys()
        diagram = DiagramSystem()
        diagram.add_subsystem(sys, "physics")
        x = jnp.asarray(sys.x0)
        u = jnp.zeros(diagram.m)

        eval_j = diagram.compile(backend="jax")
        dx_raw = jnp.asarray(diagram.f(np.asarray(sys.x0), np.zeros(diagram.m), 0.0))
        dx_cmp = eval_j.f(x, u, 0.0)
        np.testing.assert_allclose(np.asarray(dx_cmp), np.asarray(dx_raw), atol=1e-7)


if __name__ == "__main__":
    unittest.main()
