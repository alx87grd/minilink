"""Tests for the differentiable ANCF tire-ring prototype."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402

from minilink.dynamics.engines.ancf_tire_jax import (  # noqa: E402
    ANCFTireSystem,
    ancf_tire_initial_state,
    ancf_tire_ode,
    ancf_tire_potential_energy,
    make_ancf_tire_model,
    plane_contact_node_forces,
    unpack_ancf_state,
)
from minilink.graphical.animation.camera import resolve_camera_from_hints
from tests.unittest.graphics_contract_helpers import resolve_draw_frame  # noqa: E402


@pytest.mark.optional
@pytest.mark.jax
class TestANCFTireJax(unittest.TestCase):
    def _model(self):
        return make_ancf_tire_model(
            n_nodes=5,
            radius=0.4,
            mass=4.0,
            k_stretch=1.0e3,
            k_bend=5.0,
            k_area=100.0,
            k_slope=200.0,
            k_contact=2.0e3,
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
            k_contact=2.0e3,
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
            lambda xx: ancf_tire_ode(model, xx, u),
            (x,),
            (jnp.ones_like(x) * 1.0e-4,),
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
            np.asarray(dx_cmp),
            np.asarray(dx_raw),
            rtol=1e-5,
            atol=1e-3,
        )

        frame = resolve_draw_frame(sys, sys.x0, np.zeros(sys.m), 0.0)
        self.assertEqual(len(frame["primitives"]), len(frame["transforms"]))
        self.assertEqual(len(frame["primitives"]), 2 * model.n_nodes + 1)

    def test_contact_force_vectors_are_visible_in_geometry(self):
        model = make_ancf_tire_model(
            n_nodes=8,
            radius=0.4,
            mass=4.0,
            k_contact=2.0e3,
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
        spans = [
            np.linalg.norm(line.pts[-1] - line.pts[0])
            for line in force_lines
        ]
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


if __name__ == "__main__":
    unittest.main()
