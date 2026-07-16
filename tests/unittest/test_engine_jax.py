"""Integration tests for physics DynamicSystem wrapper."""

import unittest
import numpy as np
import pytest
from minilink.core.diagram import DiagramSystem
from tests.unittest.graphics_contract_helpers import resolve_draw_frame

pytest.importorskip("jax")
import jax.numpy as jnp
from minilink.dynamics.engines.contact_jax import (
    PlaneModel,
    SphereModel,
    make_world_model,
)
from minilink.dynamics.engines.world import PhysicsWorldSystem


@pytest.mark.optional
@pytest.mark.jax
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
        frame = resolve_draw_frame(sys, sys.x0, np.zeros(sys.m), 0.0)
        self.assertEqual(len(frame["primitives"]), len(frame["transforms"]))

    def test_compile_jax_parity_one_step(self):
        sys = self._make_sys()
        diagram = DiagramSystem()
        diagram.add_subsystem(sys, "physics")
        x = jnp.asarray(sys.x0)
        u = jnp.zeros(diagram.m)
        eval_j = diagram.compile(backend="jax")
        dx_raw = jnp.asarray(diagram.f(np.asarray(sys.x0), np.zeros(diagram.m), 0.0))
        dx_cmp = eval_j.f(x, u, 0.0)
        np.testing.assert_allclose(np.asarray(dx_cmp), np.asarray(dx_raw), atol=1e-07)


pytest.importorskip("jax")
import jax
from minilink.dynamics.engines.contact_jax import (
    PlaneModel,
    SphereModel,
    make_world_model,
    pack_state,
    plane_contact_force,
    unpack_state,
    world_ode,
)


@pytest.mark.optional
@pytest.mark.jax
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
        pos = jnp.array([[0.0, 0.0, 0.1]])
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


pytest.importorskip("jax")
from minilink.dynamics.engines.ancf_tire_jax import (
    ANCFTireSystem,
    ancf_tire_initial_state,
    ancf_tire_ode,
    ancf_tire_potential_energy,
    make_ancf_tire_model,
    plane_contact_node_forces,
    unpack_ancf_state,
)
from minilink.graphical.animation.camera import resolve_camera_from_hints


@pytest.mark.optional
@pytest.mark.jax
class TestANCFTireJax(unittest.TestCase):
    def _model(self):
        return make_ancf_tire_model(
            n_nodes=5,
            radius=0.4,
            mass=4.0,
            k_stretch=1000.0,
            k_bend=5.0,
            k_area=100.0,
            k_slope=200.0,
            k_contact=2000.0,
            c_contact=20.0,
        )

    def test_initial_state_shape_and_energy(self):
        model = self._model()
        x = ancf_tire_initial_state(model, center=(0.0, 0.0, 1.0))
        self.assertEqual(x.shape, (12 * model.n_nodes,))
        q, _ = unpack_ancf_state(x, model.n_nodes)
        V = ancf_tire_potential_energy(model, q.reshape(-1))
        self.assertTrue(np.isfinite(float(V)))

    def test_contact_force_pushes_up_on_penetration(self):
        model = self._model()
        x = ancf_tire_initial_state(model, center=(0.0, 0.0, 0.25))
        q, v = unpack_ancf_state(x, model.n_nodes)
        f = plane_contact_node_forces(model, q.reshape(-1), v.reshape(-1))
        self.assertGreater(float(jnp.max(f[:, 2])), 0.0)

    def test_friction_pushes_forward_for_positive_spin(self):
        model = make_ancf_tire_model(
            n_nodes=8,
            radius=0.4,
            mass=4.0,
            k_contact=2000.0,
            c_contact=20.0,
            mu_static=0.9,
            mu_dynamic=0.75,
        )
        x = ancf_tire_initial_state(
            model,
            center=(0.0, 0.0, 0.4),
            linear_velocity=(0.0, 0.0, 0.0),
            angular_velocity=(0.0, 10.0, 0.0),
        )
        q, v = unpack_ancf_state(x, model.n_nodes)
        f = plane_contact_node_forces(model, q.reshape(-1), v.reshape(-1))
        self.assertGreater(float(jnp.max(f[:, 0])), 0.0)

    def test_ode_is_jax_traceable_and_jvp_compatible(self):
        model = self._model()
        x = ancf_tire_initial_state(model, center=(0.0, 0.0, 1.0))
        u = jnp.zeros(3 * model.n_nodes)
        dx = ancf_tire_ode(model, x, u)
        self.assertEqual(dx.shape, x.shape)
        jax.make_jaxpr(lambda xx, uu: ancf_tire_ode(model, xx, uu))(x, u)
        _, tangent = jax.jvp(
            lambda xx: ancf_tire_ode(model, xx, u), (x,), (jnp.ones_like(x) * 0.0001,)
        )
        self.assertEqual(tangent.shape, x.shape)

    def test_minilink_system_compile_jax_parity(self):
        model = self._model()
        sys = ANCFTireSystem(model, center=(0.0, 0.0, 1.0))
        x = jnp.asarray(sys.x0)
        u = jnp.zeros(sys.m)
        evaluator = sys.compile(backend="jax")
        dx_raw = jnp.asarray(sys.f(np.asarray(sys.x0), np.zeros(sys.m), 0.0))
        dx_cmp = evaluator.f(x, u, 0.0)
        np.testing.assert_allclose(
            np.asarray(dx_cmp), np.asarray(dx_raw), rtol=1e-05, atol=0.001
        )
        frame = resolve_draw_frame(sys, sys.x0, np.zeros(sys.m), 0.0)
        self.assertEqual(len(frame["primitives"]), len(frame["transforms"]))
        self.assertEqual(len(frame["primitives"]), 2 * model.n_nodes + 1)

    def test_contact_force_vectors_are_visible_in_geometry(self):
        model = make_ancf_tire_model(
            n_nodes=8,
            radius=0.4,
            mass=4.0,
            k_contact=2000.0,
            c_contact=20.0,
            mu_static=0.9,
            mu_dynamic=0.75,
        )
        sys = ANCFTireSystem(
            model,
            center=(0.0, 0.0, 0.4),
            angular_velocity=(0.0, 10.0, 0.0),
            contact_force_scale=0.01,
            contact_force_threshold=1.0,
        )
        dynamic = sys.get_dynamic_geometry(sys.x0, np.zeros(sys.m), 0.0)
        force_lines = dynamic["world"][model.n_nodes :]
        spans = [np.linalg.norm(line.pts[-1] - line.pts[0]) for line in force_lines]
        self.assertGreater(float(np.max(spans)), 0.0)

    def test_contact_force_vectors_hide_without_contact(self):
        model = make_ancf_tire_model(n_nodes=8, radius=0.4, mass=4.0)
        sys = ANCFTireSystem(
            model,
            center=(0.0, 0.0, 1.5),
            contact_force_scale=0.01,
            contact_force_threshold=1.0,
        )
        dynamic = sys.get_dynamic_geometry(sys.x0, np.zeros(sys.m), 0.0)
        self.assertEqual(len(dynamic["world"]), model.n_nodes)

    def test_camera_is_fixed_by_default_for_forward_motion(self):
        model = make_ancf_tire_model(n_nodes=8, radius=0.4, mass=4.0)
        sys = ANCFTireSystem(model, center=(0.0, 0.0, 1.0))
        x_shifted = np.asarray(sys.x0).copy()
        x_shifted[: 6 * model.n_nodes].reshape((model.n_nodes, 6))[:, 0] += 1.0
        camera0 = resolve_camera_from_hints(
            sys, sys.tf(sys.x0, np.zeros(sys.m), 0.0), sys.x0, np.zeros(sys.m), 0.0
        )
        camera1 = resolve_camera_from_hints(
            sys,
            sys.tf(x_shifted, np.zeros(sys.m), 0.0),
            x_shifted,
            np.zeros(sys.m),
            0.0,
        )
        self.assertAlmostEqual(float(camera0[0, 3]), float(camera1[0, 3]))
