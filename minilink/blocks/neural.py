"""Minimal neural-network static blocks.

These are ordinary :class:`~minilink.core.system.StaticSystem` blocks: weights
are model parameters, inputs and outputs are ports, and training lives outside
the block.
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import StaticSystem


class NeuralNetwork(StaticSystem):
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
