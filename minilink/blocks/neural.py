"""Neural-network static blocks: the weights are parameters, the training lives outside."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


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

        # Hidden nonlinearity
        require_activation(activation)
        self.activation = activation

        # Parameters: orthogonal weights, of gain √2 on the hidden layers and
        # output_gain on the last one, and zero biases
        sizes = (int(input_dim),) + tuple(int(h) for h in hidden) + (int(output_dim),)
        gains = [np.sqrt(2.0)] * len(hidden) + [float(output_gain)]
        rng = np.random.default_rng(seed)
        self.params = {}
        for k, (a, b, gain) in enumerate(zip(sizes[:-1], sizes[1:], gains)):
            self.params[f"W{k}"] = gain * orthogonal(rng, (b, a))
            self.params[f"b{k}"] = np.zeros(b)
        self.n_layers = len(sizes) - 1

        # Ports
        self.add_input_port("u", dim=sizes[0])
        self.add_output_port(
            "y", dim=sizes[-1], function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        activation = self.activation
        n_layers = self.n_layers

        def relu(v):
            return xp.maximum(v, 0.0)

        act = xp.tanh if activation == "tanh" else relu

        # Hidden layers: a_k = act(W_k a_{k−1} + b_k), from a₀ = u
        a = u
        for k in range(n_layers - 1):
            W = params[f"W{k}"]
            b = params[f"b{k}"]
            a = act(W @ a + b)

        # Output layer, affine (no activation): y = W_L a_{L−1} + b_L
        L = n_layers - 1
        W = params[f"W{L}"]
        b = params[f"b{L}"]
        y = W @ a + b

        return y


###############################################################################
# Internal machinery
###############################################################################


def require_activation(activation):
    """Refuse a hidden nonlinearity other than ``"tanh"`` and ``"relu"``."""
    if activation not in ("tanh", "relu"):
        raise ValueError(f"activation must be 'tanh' or 'relu', got {activation!r}")


def orthogonal(rng, shape):
    """Orthogonal matrix of the given shape (QR of a Gaussian draw)."""
    rows, cols = shape

    # QR of a Gaussian matrix; the signs of diag(R) make Q uniformly distributed
    a = rng.standard_normal((max(rows, cols), min(rows, cols)))
    q, r = np.linalg.qr(a)
    q = q * np.sign(np.diag(r))

    # The tall factor as drawn, transposed for a wide shape
    Q = q if rows >= cols else q.T

    return Q
