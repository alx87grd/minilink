"""Minimal neural-network static blocks.

These are ordinary :class:`~minilink.core.system.System` blocks: weights
are model parameters, inputs and outputs are ports, and training lives outside
the block.
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import System


class NeuralNetwork(System):
    """One-hidden-layer neural network ``y = W2 tanh(W1 u + b1) + b2``.

    Parameters
    ----------
    input_dim : int
        Dimension of the input port ``u``.
    output_dim : int
        Dimension of the output port ``y``.
    hidden_dim : int, optional
        Number of hidden units.
    seed : int, optional
        Seed for deterministic weight initialization.
    scale : float, optional
        Standard-deviation scale for the initial weights. Biases start at zero.
    """

    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        hidden_dim: int = 8,
        seed: int = 0,
        scale: float = 0.1,
    ):
        super().__init__()
        self.name = "Neural Network"

        input_dim = int(input_dim)
        output_dim = int(output_dim)
        hidden_dim = int(hidden_dim)
        if input_dim <= 0 or output_dim <= 0 or hidden_dim <= 0:
            raise ValueError("input_dim, output_dim, and hidden_dim must be positive")

        rng = np.random.default_rng(seed)
        self.params = {
            "W1": scale * rng.standard_normal((hidden_dim, input_dim)),
            "b1": np.zeros(hidden_dim),
            "W2": scale * rng.standard_normal((output_dim, hidden_dim)),
            "b2": np.zeros(output_dim),
        }

        self.add_input_port("u", dim=input_dim)
        self.add_output_port(
            "y", dim=output_dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        W1 = params["W1"]
        b1 = params["b1"]
        W2 = params["W2"]
        b2 = params["b2"]

        xp = array_module(u)

        a = xp.tanh(W1 @ u + b1)
        y = W2 @ a + b2

        return y


class MLP(System):
    """
    Multilayer perceptron ``y = W_L a_{L-1} + b_L`` with ``a_k = act(W_k a_{k-1} + b_k)``.

    Weights live in ``params`` as ``{"W0", "b0", "W1", "b1", ...}`` so training
    loops differentiate through the compiled diagram like any other parameter.

    Parameters
    ----------
    input_dim, output_dim : int
        Port dimensions.
    hidden : tuple of int
        Hidden layer widths; ``()`` gives an affine map.
    activation : {"tanh", "relu"}
        Hidden nonlinearity.
    seed : int
        Seed of the orthogonal initialization.
    output_gain : float
        Gain of the last layer's orthogonal init (``0.01`` for a policy mean
        that starts near zero, ``1.0`` for a value function).
    """

    def __init__(
        self,
        input_dim: int,
        output_dim: int,
        hidden=(64, 64),
        activation: str = "tanh",
        seed: int = 0,
        output_gain: float = 1.0,
    ):
        super().__init__()
        self.name = "MLP"
        if activation not in ("tanh", "relu"):
            raise ValueError(f"activation must be 'tanh' or 'relu', got {activation!r}")
        self.activation = activation
        sizes = (int(input_dim),) + tuple(int(h) for h in hidden) + (int(output_dim),)
        gains = [np.sqrt(2.0)] * len(hidden) + [float(output_gain)]
        rng = np.random.default_rng(seed)
        self.params = {}
        for k, (a, b, gain) in enumerate(zip(sizes[:-1], sizes[1:], gains)):
            self.params[f"W{k}"] = gain * orthogonal(rng, (b, a))
            self.params[f"b{k}"] = np.zeros(b)
        self.n_layers = len(sizes) - 1
        self.add_input_port("u", dim=sizes[0])
        self.add_output_port(
            "y", dim=sizes[-1], function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)
        act = xp.tanh if self.activation == "tanh" else (lambda v: xp.maximum(v, 0.0))
        a = u
        for k in range(self.n_layers - 1):
            a = act(params[f"W{k}"] @ a + params[f"b{k}"])
        last = self.n_layers - 1
        return params[f"W{last}"] @ a + params[f"b{last}"]


def orthogonal(rng, shape):
    """Orthogonal matrix of the given shape (QR of a Gaussian draw)."""
    rows, cols = shape
    a = rng.standard_normal((max(rows, cols), min(rows, cols)))
    q, r = np.linalg.qr(a)
    q = q * np.sign(np.diag(r))
    return q if rows >= cols else q.T
