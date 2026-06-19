import unittest

import numpy as np
import pytest

from minilink.blocks.neural import NeuralNetwork


class TestNeuralNetwork(unittest.TestCase):
    def test_forward_equation_and_shape(self):
        net = NeuralNetwork(input_dim=2, output_dim=1, hidden_dim=3)
        params = {
            "W1": np.array(
                [
                    [1.0, 0.0],
                    [0.0, 1.0],
                    [-1.0, 1.0],
                ]
            ),
            "b1": np.array([0.0, 0.5, -0.5]),
            "W2": np.array([[2.0, -1.0, 0.5]]),
            "b2": np.array([0.25]),
        }
        u = np.array([0.2, -0.4])

        y = net.compute(np.array([]), u, params=params)
        expected = (
            params["W2"] @ np.tanh(params["W1"] @ u + params["b1"])
            + params["b2"]
        )

        self.assertEqual(y.shape, (1,))
        np.testing.assert_allclose(y, expected)

    def test_explicit_params_override_defaults(self):
        net = NeuralNetwork(input_dim=1, output_dim=1, hidden_dim=2)
        net.params = {
            "W1": np.array([[1.0], [1.0]]),
            "b1": np.array([0.0, 0.0]),
            "W2": np.array([[1.0, 1.0]]),
            "b2": np.array([0.0]),
        }
        override = {
            "W1": np.zeros((2, 1)),
            "b1": np.zeros(2),
            "W2": np.zeros((1, 2)),
            "b2": np.array([3.0]),
        }

        y_default = net.compute(np.array([]), np.array([1.0]))
        y_override = net.compute(np.array([]), np.array([1.0]), params=override)

        np.testing.assert_allclose(y_default, [2.0 * np.tanh(1.0)])
        np.testing.assert_allclose(y_override, [3.0])

    def test_compute_is_jax_traceable(self):
        jax = pytest.importorskip("jax")
        import jax.numpy as jnp

        net = NeuralNetwork(input_dim=2, output_dim=1, hidden_dim=3)
        params = {key: jnp.asarray(value) for key, value in net.params.items()}

        y = jax.jit(lambda u, params: net.compute([], u, params=params))(
            jnp.array([1.0, -1.0]),
            params,
        )

        self.assertEqual(np.asarray(y).shape, (1,))


if __name__ == "__main__":
    unittest.main()
