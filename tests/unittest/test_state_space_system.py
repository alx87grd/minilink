import unittest

import numpy as np
import pytest

from minilink.dynamics.abstraction.state_space import LTISystem, StateSpaceSystem


class TestLTISystem(unittest.TestCase):
    def test_defaults_to_full_state_output(self):
        A = np.array([[0.0, 1.0], [-2.0, -3.0]])
        B = np.array([[0.0], [1.0]])
        sys = LTISystem(A, B)

        self.assertEqual(sys.n, 2)
        self.assertEqual(sys.m, 1)
        self.assertEqual(sys.p, 2)
        np.testing.assert_allclose(sys.C(), np.eye(2))
        np.testing.assert_allclose(sys.D(), np.zeros((2, 1)))
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
        sys = LTISystem(A, B, C, D)

        self.assertIs(sys.A(), A)
        self.assertIs(sys.B(), B)
        self.assertIs(sys.C(), C)
        self.assertIs(sys.D(), D)
        self.assertEqual(sys.p, 1)
        self.assertEqual(sys.outputs["y"].dependencies, ("u",))

        x = np.array([2.0, -4.0])
        u = np.array([0.5])
        np.testing.assert_allclose(sys.f(x, u), np.array([-4.0, -0.2]))
        np.testing.assert_allclose(sys.h(x, u), np.array([7.5]))

    def test_constant_matrices_ignore_params(self):
        sys = LTISystem(np.array([[2.0]]), np.array([[3.0]]))
        np.testing.assert_allclose(
            sys.f(np.array([4.0]), np.array([5.0]), params={"A": np.array([[0.0]])}),
            np.array([23.0]),
        )

    def test_rejects_invalid_dimensions(self):
        with self.assertRaisesRegex(ValueError, "A must be"):
            LTISystem(np.array([[1.0, 2.0]]), np.array([[1.0], [2.0]]))

        with self.assertRaisesRegex(ValueError, "B row count"):
            LTISystem(np.eye(2), np.ones((3, 1)))

        with self.assertRaisesRegex(ValueError, "C column count"):
            LTISystem(np.eye(2), np.ones((2, 1)), C=np.ones((1, 3)))

        with self.assertRaisesRegex(ValueError, "C must be"):
            LTISystem(np.eye(2), np.ones((2, 1)), C=np.ones(2))

        with self.assertRaisesRegex(ValueError, "D shape"):
            LTISystem(
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
        sys = LTISystem(A, B)

        self.assertIs(sys.A(), A)
        self.assertIs(sys.B(), B)

        x = jnp.array([1.0, 2.0])
        u = jnp.array([0.5])
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)
        jax.make_jaxpr(lambda xx, uu: sys.h(xx, uu))(x, u)
        np.testing.assert_allclose(np.asarray(sys.f(x, u)), np.array([2.0, -7.5]))
        np.testing.assert_allclose(np.asarray(sys.h(x, u)), np.array([1.0, 2.0]))


class TestStateSpaceSystem(unittest.TestCase):
    def test_subclass_builds_matrices_from_params(self):
        class Gain(StateSpaceSystem):
            def __init__(self):
                super().__init__(n=1, m=1, p=1, name="Gain")
                self.params = {"a": -2.0, "b": 3.0}

            def A(self, t=0.0, params=None):
                params = self.params if params is None else params
                return np.array([[params["a"]]])

            def B(self, t=0.0, params=None):
                params = self.params if params is None else params
                return np.array([[params["b"]]])

        sys = Gain()
        x = np.array([4.0])
        u = np.array([5.0])
        np.testing.assert_allclose(sys.f(x, u), np.array([-8.0 + 15.0]))
        np.testing.assert_allclose(sys.h(x, u), x)
        np.testing.assert_allclose(
            sys.f(x, u, params={"a": 0.0, "b": 1.0}), np.array([5.0])
        )


if __name__ == "__main__":
    unittest.main()
