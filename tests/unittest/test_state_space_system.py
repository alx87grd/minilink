import unittest

import numpy as np
import pytest

from minilink.dynamics.abstraction.state_space import StateSpaceSystem


class TestStateSpaceSystem(unittest.TestCase):
    def test_defaults_to_full_state_output(self):
        A = np.array([[0.0, 1.0], [-2.0, -3.0]])
        B = np.array([[0.0], [1.0]])
        sys = StateSpaceSystem(A, B)

        self.assertEqual(sys.n, 2)
        self.assertEqual(sys.m, 1)
        self.assertEqual(sys.p, 2)
        np.testing.assert_allclose(sys.C, np.eye(2))
        np.testing.assert_allclose(sys.D, np.zeros((2, 1)))
        self.assertEqual(sys.outputs["y"].dependencies, ())

        x = np.array([1.0, 2.0])
        u = np.array([0.5])
        np.testing.assert_allclose(sys.f(x, u), np.array([2.0, -7.5]))
        np.testing.assert_allclose(sys.h(x, u), x)

    def test_explicit_output_matrices(self):
        A = np.array([[0.0, 1.0], [-1.0, -0.2]])
        B = np.array([[0.0], [2.0]])
        C = np.array([[1.0, -1.0]])
        D = np.array([[3.0]])
        sys = StateSpaceSystem(A, B, C, D)

        self.assertIs(sys.A, A)
        self.assertIs(sys.B, B)
        self.assertIs(sys.C, C)
        self.assertIs(sys.D, D)
        self.assertEqual(sys.p, 1)
        self.assertEqual(sys.outputs["y"].dependencies, ("u",))

        x = np.array([2.0, -4.0])
        u = np.array([0.5])
        np.testing.assert_allclose(sys.f(x, u), np.array([-4.0, -0.2]))
        np.testing.assert_allclose(sys.h(x, u), np.array([7.5]))

    def test_params_are_ignored_for_signature_compatibility(self):
        sys = StateSpaceSystem(np.array([[2.0]]), np.array([[3.0]]))
        np.testing.assert_allclose(
            sys.f(np.array([4.0]), np.array([5.0]), params={"A": np.array([[0.0]])}),
            np.array([23.0]),
        )

    def test_rejects_invalid_dimensions(self):
        with self.assertRaisesRegex(ValueError, "A must be"):
            StateSpaceSystem(np.array([[1.0, 2.0]]), np.array([[1.0], [2.0]]))

        with self.assertRaisesRegex(ValueError, "B row count"):
            StateSpaceSystem(np.eye(2), np.ones((3, 1)))

        with self.assertRaisesRegex(ValueError, "C column count"):
            StateSpaceSystem(np.eye(2), np.ones((2, 1)), C=np.ones((1, 3)))

        with self.assertRaisesRegex(ValueError, "C must be"):
            StateSpaceSystem(np.eye(2), np.ones((2, 1)), C=np.ones(2))

        with self.assertRaisesRegex(ValueError, "D shape"):
            StateSpaceSystem(
                np.eye(2),
                np.ones((2, 1)),
                C=np.ones((1, 2)),
                D=np.ones((2, 1)),
            )

    @pytest.mark.optional
    @pytest.mark.jax
    def test_jax_arrays_are_preserved_and_traceable_if_available(self):
        jax = pytest.importorskip("jax")
        jnp = pytest.importorskip("jax.numpy")

        A = jnp.array([[0.0, 1.0], [-2.0, -3.0]])
        B = jnp.array([[0.0], [1.0]])
        sys = StateSpaceSystem(A, B)

        self.assertIs(sys.A, A)
        self.assertIs(sys.B, B)

        x = jnp.array([1.0, 2.0])
        u = jnp.array([0.5])
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)
        jax.make_jaxpr(lambda xx, uu: sys.h(xx, uu))(x, u)
        np.testing.assert_allclose(np.asarray(sys.f(x, u)), np.array([2.0, -7.5]))
        np.testing.assert_allclose(np.asarray(sys.h(x, u)), np.array([1.0, 2.0]))


if __name__ == "__main__":
    unittest.main()
