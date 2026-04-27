import unittest

import numpy as np

from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.dynamics.catalog.pendulum.double_pendulum import DoublePendulum
from minilink.graphical.primitives import Box


class TestCartPole(unittest.TestCase):
    def test_dimensions_and_labels(self):
        sys = CartPole()

        self.assertEqual(sys.n, 4)
        self.assertEqual(sys.m, 1)
        self.assertEqual(sys.p, 4)
        self.assertEqual(sys.state.labels, ["x", "theta", "dx", "dtheta"])
        self.assertEqual(sys.inputs["u"].labels, ["F"])

    def test_matrices_match_pyro_reference_values(self):
        sys = CartPole()
        q = np.array([0.2, 0.3])
        dq = np.array([0.4, 0.5])

        H = sys.H(q)
        C = sys.C(q, dq)
        B = sys.B(q)
        g = sys.g(q)

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
        np.testing.assert_allclose(B, np.array([[1.0], [0.0]]))
        np.testing.assert_allclose(g, np.array([0.0, 0.1 * 9.81 * 0.5 * np.sin(0.3)]))

    def test_compiled_equilibrium_and_explicit_params(self):
        sys = CartPole()
        x = np.zeros(sys.n)
        u = np.zeros(sys.m)

        np.testing.assert_allclose(sys.compile("numpy").f(x, u, 0.0), np.zeros(sys.n))
        np.testing.assert_allclose(
            sys.H(np.zeros(2), params={**sys.params, "m1": 2.0})[0, 0],
            2.1,
        )

    def test_graphics_geometry_and_transform_counts_match(self):
        sys = CartPole()
        primitives = sys.get_kinematic_geometry()
        transforms = sys.get_kinematic_transforms(np.zeros(sys.n), np.zeros(sys.m), 0.0)

        self.assertIsInstance(primitives[1], Box)
        self.assertEqual(primitives[1].length_z, sys.params["cart_depth"])
        self.assertGreater(
            transforms[4][2, 3], 0.5 * float(sys.params["cart_depth"])
        )
        self.assertEqual(len(primitives), len(transforms))
        for T in transforms:
            self.assertEqual(T.shape, (4, 4))


class TestDoublePendulum(unittest.TestCase):
    def test_dimensions_and_labels(self):
        sys = DoublePendulum()

        self.assertEqual(sys.n, 4)
        self.assertEqual(sys.m, 2)
        self.assertEqual(sys.p, 4)
        self.assertEqual(sys.state.labels, ["theta1", "theta2", "dtheta1", "dtheta2"])

    def test_matrices_match_pyro_reference_values(self):
        sys = DoublePendulum()
        q = np.array([0.2, 0.3])
        dq = np.array([0.4, 0.5])

        H = sys.H(q)
        C = sys.C(q, dq)
        g = sys.g(q)
        c2 = np.cos(q[1])
        s2 = np.sin(q[1])
        s1 = np.sin(q[0])
        s12 = np.sin(q[0] + q[1])
        h = s2

        np.testing.assert_allclose(
            H,
            np.array(
                [
                    [3.0 + 2.0 * c2, 1.0 + c2],
                    [1.0 + c2, 1.0],
                ]
            ),
        )
        np.testing.assert_allclose(
            C,
            np.array(
                [
                    [-h * dq[1], -h * (dq[0] + dq[1])],
                    [h * dq[0], 0.0],
                ]
            ),
        )
        np.testing.assert_allclose(g, np.array([-19.62 * s1 - 9.81 * s12, -9.81 * s12]))

    def test_compiled_equilibrium_and_explicit_params(self):
        sys = DoublePendulum()
        x = np.zeros(sys.n)
        u = np.zeros(sys.m)

        np.testing.assert_allclose(sys.compile("numpy").f(x, u, 0.0), np.zeros(sys.n))
        np.testing.assert_allclose(
            sys.d(np.zeros(2), np.array([2.0, 3.0]), params={**sys.params, "d2": 4.0}),
            np.array([0.0, 12.0]),
        )

    def test_graphics_geometry_and_transform_counts_match(self):
        sys = DoublePendulum()
        primitives = sys.get_kinematic_geometry()
        transforms = sys.get_kinematic_transforms(np.zeros(sys.n), np.zeros(sys.m), 0.0)

        self.assertEqual(len(primitives), len(transforms))
        for T in transforms:
            self.assertEqual(T.shape, (4, 4))


if __name__ == "__main__":
    unittest.main()
