"""Optional JAX native-array coverage for ``UR5Manipulator``."""

from __future__ import annotations

import json
import time
import unittest
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("jax")
import jax
import jax.numpy as jnp

from minilink.core.backends import configure_jax
from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator
from minilink.simulation.simulator import Simulator

_FIXTURE_DIR = Path(__file__).resolve().parents[1] / "fixtures" / "ur5_sim_baseline"


def _load_baseline():
    manifest = json.loads((_FIXTURE_DIR / "manifest.json").read_text())
    data = np.load(_FIXTURE_DIR / "trajectory.npz")
    return manifest, data


def _scenario_arm(manifest):
    arm = UR5Manipulator()
    arm.params["gravity"] = float(manifest["gravity"])
    x0 = np.asarray(manifest["x0"], dtype=float)
    u0 = np.asarray(manifest["u"], dtype=float)
    arm.x0 = x0
    arm.inputs["u"].nominal_value = u0
    return arm, x0, u0


@pytest.mark.optional
@pytest.mark.jax
class TestUR5NativeArrayJax(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.arm = UR5Manipulator()
        self.q = np.linspace(-0.5, 0.4, 6)
        self.dq = np.linspace(0.2, -0.3, 6)
        self.u = np.linspace(-0.4, 0.5, 6)
        self.x = self.arm.q2x(self.q, self.dq)

    def test_matrix_hooks_match_numpy(self):
        q_j = jnp.asarray(self.q)
        dq_j = jnp.asarray(self.dq)
        np.testing.assert_allclose(np.asarray(self.arm.H(q_j)), self.arm.H(self.q))
        np.testing.assert_allclose(
            np.asarray(self.arm.g(q_j)), self.arm.g(self.q), atol=1e-12
        )
        np.testing.assert_allclose(
            np.asarray(self.arm.C(q_j, dq_j) @ dq_j),
            self.arm.C(self.q, self.dq) @ self.dq,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            np.asarray(self.arm.d(q_j, dq_j)), self.arm.d(self.q, self.dq)
        )
        np.testing.assert_allclose(
            np.asarray(self.arm.forward_kinematics(q_j)),
            self.arm.forward_kinematics(self.q),
        )
        np.testing.assert_allclose(
            np.asarray(self.arm.J(q_j)), self.arm.J(self.q), atol=1e-12
        )

    def test_f_matches_numpy(self):
        dx_n = self.arm.f(self.x, self.u)
        dx_j = np.asarray(self.arm.f(jnp.asarray(self.x), jnp.asarray(self.u)))
        np.testing.assert_allclose(dx_j, dx_n, rtol=1e-10, atol=1e-10)

    def test_tf_frames_match_numpy(self):
        frames_n = self.arm.tf(self.x, self.u)
        frames_j = self.arm.tf(jnp.asarray(self.x), jnp.asarray(self.u))
        self.assertEqual(set(frames_n), set(frames_j))
        for key, Tn in frames_n.items():
            np.testing.assert_allclose(np.asarray(frames_j[key]), Tn, atol=1e-12)

    def test_f_fk_tf_are_traceable_and_jittable(self):
        x = jnp.asarray(self.x)
        u = jnp.asarray(self.u)
        q = jnp.asarray(self.q)
        jax.make_jaxpr(lambda xx, uu: self.arm.f(xx, uu))(x, u)
        jax.make_jaxpr(lambda qq: self.arm.forward_kinematics(qq))(q)

        def stacked_tf(xx):
            frames = self.arm.tf(xx, u)
            return jnp.stack([frames[key] for key in sorted(frames)])

        jax.make_jaxpr(stacked_tf)(x)
        np.testing.assert_allclose(
            np.asarray(jax.jit(lambda xx, uu: self.arm.f(xx, uu))(x, u)),
            self.arm.f(self.x, self.u),
            rtol=1e-10,
            atol=1e-10,
        )

    def test_compile_backend_jax_matches_numpy_f(self):
        ev = self.arm.compile(backend="jax")
        dx = np.asarray(ev.f(jnp.asarray(self.x), jnp.asarray(self.u), 0.0))
        np.testing.assert_allclose(
            dx, self.arm.f(self.x, self.u), rtol=1e-10, atol=1e-10
        )

    def test_gradients_are_finite(self):
        x = jnp.asarray(self.x)
        u = jnp.asarray(self.u)
        q = jnp.asarray(self.q)
        Jf = jax.jacrev(lambda xx: self.arm.f(xx, u))(x)
        Jfk = jax.jacrev(self.arm.forward_kinematics)(q)
        self.assertEqual(np.asarray(Jf).shape, (12, 12))
        self.assertEqual(np.asarray(Jfk).shape, (3, 6))
        self.assertTrue(np.all(np.isfinite(Jf)))
        self.assertTrue(np.all(np.isfinite(Jfk)))

    def test_params_override_is_differentiable(self):
        q = jnp.asarray(self.q)

        def gravity_norm(gravity):
            params = dict(self.arm.params)
            params["gravity"] = gravity
            return jnp.linalg.norm(self.arm.g(q, params))

        g = jax.grad(gravity_norm)(jnp.asarray(9.81))
        self.assertTrue(np.isfinite(np.asarray(g)))
        self.assertNotEqual(float(g), 0.0)

    def test_simulation_parity_and_speed(self):
        manifest, _ = _load_baseline()
        arm, x0, _ = _scenario_arm(manifest)
        kwargs = dict(
            x0=x0,
            t0=0.0,
            tf=manifest["tf"],
            dt=manifest["dt"],
            solver=manifest["solver"],
            verbose=False,
        )
        sim_np = Simulator(arm, compile_backend="numpy", **kwargs)
        sim_jx = Simulator(arm, compile_backend="jax", **kwargs)
        sim_jx.solve()  # warmup / JIT

        t0 = time.perf_counter()
        traj_np = sim_np.solve()
        t_np = time.perf_counter() - t0

        t0 = time.perf_counter()
        traj_jx = sim_jx.solve()
        t_jx = time.perf_counter() - t0

        np.testing.assert_allclose(traj_np.t, traj_jx.t)
        np.testing.assert_allclose(
            traj_np.x, np.asarray(traj_jx.x), rtol=1e-09, atol=1e-09
        )
        self.assertTrue(np.all(np.isfinite(traj_np.x)))
        self.assertLess(t_jx, 0.8 * t_np)

    def test_matches_pre_impl_baseline(self):
        manifest, data = _load_baseline()
        arm, x0, _ = _scenario_arm(manifest)
        kwargs = dict(
            x0=x0,
            t0=0.0,
            tf=manifest["tf"],
            dt=manifest["dt"],
            solver=manifest["solver"],
            verbose=False,
        )
        traj_np = Simulator(arm, compile_backend="numpy", **kwargs).solve()
        traj_jx = Simulator(arm, compile_backend="jax", **kwargs).solve()

        np.testing.assert_allclose(traj_np.t, data["t"])
        np.testing.assert_allclose(traj_np.x, data["x"], rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(
            np.asarray(traj_jx.x), data["x"], rtol=1e-09, atol=1e-09
        )
        np.testing.assert_allclose(traj_np.u, data["u"])


if __name__ == "__main__":
    unittest.main()
