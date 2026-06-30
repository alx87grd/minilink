"""Deep contract checks on representative catalog plants (dynamics + graphics).

Broader catalog coverage lives in ``test_catalog_migration.py`` and render
smoke in ``test_kinematic_regression.py`` (manifest only — no stored PNGs).
This file keeps a small set of
reference-value and integration checks that are expensive to scatter.
"""

from __future__ import annotations

import unittest

import numpy as np

from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.dynamics.catalog.pendulum.double_pendulum import DoublePendulum
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle
from minilink.graphical.animation.camera import resolve_camera_from_hints
from minilink.graphical.animation.primitives import Arrow, Box, Rod
from tests.unittest.graphics_contract_helpers import geometry_smoke, resolve_draw_frame


class TestCartPole(unittest.TestCase):
    def test_dimensions_and_reference_matrices(self):
        sys = CartPole()
        self.assertEqual(sys.n, 4)
        self.assertEqual(sys.m, 1)
        self.assertEqual(sys.state.labels, ["x", "theta", "dx", "dtheta"])

        q = np.array([0.2, 0.3])
        dq = np.array([0.4, 0.5])
        H = sys.H(q)
        C = sys.C(q, dq)
        np.testing.assert_allclose(
            H,
            np.array(
                [
                    [1.1, 0.1 * 0.5 * np.cos(0.3)],
                    [0.1 * 0.5 * np.cos(0.3), 0.1 * 0.5**2],
                ]
            ),
        )
        self.assertAlmostEqual(C[0, 1], -0.1 * 0.5 * np.sin(0.3) * 0.5)
        np.testing.assert_allclose(sys.B(q), np.array([[1.0], [0.0]]))
        np.testing.assert_allclose(
            sys.g(q), np.array([0.0, 0.1 * 9.81 * 0.5 * np.sin(0.3)])
        )

    def test_compile_and_graphics_contract(self):
        sys = CartPole()
        x = np.zeros(sys.n)
        u = np.zeros(sys.m)
        np.testing.assert_allclose(sys.compile("numpy").f(x, u, 0.0), np.zeros(sys.n))

        frame = resolve_draw_frame(sys)
        cart_boxes = [
            primitive for primitive in frame["primitives"] if isinstance(primitive, Box)
        ]
        self.assertEqual(len(cart_boxes), 1)
        self.assertEqual(cart_boxes[0].length_z, sys.cart_depth)
        self.assertTrue(any(isinstance(p, Rod) for p in frame["primitives"]))
        geometry_smoke(sys)


class TestDoublePendulum(unittest.TestCase):
    def test_reference_matrices_and_compile(self):
        sys = DoublePendulum()
        q = np.array([0.2, 0.3])
        dq = np.array([0.4, 0.5])
        c2 = np.cos(q[1])
        s1, s12 = np.sin(q[0]), np.sin(q[0] + q[1])
        h = np.sin(q[1])

        np.testing.assert_allclose(
            sys.H(q),
            np.array([[3.0 + 2.0 * c2, 1.0 + c2], [1.0 + c2, 1.0]]),
        )
        np.testing.assert_allclose(
            sys.C(q, dq),
            np.array(
                [
                    [-h * dq[1], -h * (dq[0] + dq[1])],
                    [h * dq[0], 0.0],
                ]
            ),
        )
        np.testing.assert_allclose(
            sys.g(q), np.array([-19.62 * s1 - 9.81 * s12, -9.81 * s12])
        )

        x = np.zeros(sys.n)
        np.testing.assert_allclose(
            sys.compile("numpy").f(x, np.zeros(sys.m), 0.0), np.zeros(sys.n)
        )
        geometry_smoke(sys)


class TestDynamicBicycle(unittest.TestCase):
    def test_dynamics_and_camera_follow(self):
        sys = DynamicBicycle()
        self.assertEqual(sys.n, 6)
        self.assertEqual(sys.m, 2)

        x = np.array([10.0, 3.0, 0.25, 4.0, 0.0, 0.0])
        u = np.zeros(sys.m)
        sys.camera_target[:] = (1.0, -2.0, 0.5)
        sys.camera_scale = 7.0
        frames = sys.tf(x, u, 0.0)
        camera = resolve_camera_from_hints(sys, frames, x, u, 0.0)
        np.testing.assert_allclose(camera[:3, 3], np.array([11.0, 1.0, 0.5]))

        x = np.array([0.1, -0.2, 0.3, 5.0, 0.4, 0.05])
        dx = sys.f(x, np.array([20.0, 0.1]))
        np.testing.assert_allclose(dx[:3], sys.N(x[:3]) @ x[3:])

    def test_graphics_resolves_dynamic_arrows(self):
        sys = DynamicBicycle()
        x = np.zeros(sys.n)
        x[3] = 1.0
        frame = resolve_draw_frame(sys, x, np.array([10.0, 0.1]), 0.0)
        self.assertEqual(len(frame["primitives"]), 7)
        self.assertEqual(sum(isinstance(p, Arrow) for p in frame["primitives"]), 4)
        geometry_smoke(sys, x, np.array([10.0, 0.1]), 0.0)


if __name__ == "__main__":
    unittest.main()
