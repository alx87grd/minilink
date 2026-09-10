"""
Critics: value networks on the policy's features.

``ValueFunction`` estimates ``V(x)`` for on-policy methods (advantage
baselines); ``QFunction`` estimates ``Q(x, a)`` for off-policy ones. Both are
:class:`~minilink.blocks.neural.MLP` blocks whose weights are a params dict,
so a planner keeps them in its train state like the policy weights.
"""

from minilink.blocks.neural import MLP
from minilink.core.backends import require_jax_numpy

# Public API


class ValueFunction:
    """``V(x) = MLP(z(x))`` on the controller's features."""

    def __init__(self, observe, n_features, hidden=(64, 64), activation="tanh", seed=1):
        self.observe = observe
        self.mlp = MLP(n_features, 1, hidden, activation, seed=seed, output_gain=1.0)

    def init(self):
        return self.mlp.params

    def value(self, params, x):
        return self.mlp.compute(None, self.observe(x), params=params)[0]


class QFunction:
    """``Q(x, a) = MLP([z(x), a])`` for off-policy methods."""

    def __init__(
        self, observe, n_features, m, hidden=(64, 64), activation="tanh", seed=2
    ):
        self.observe = observe
        self.mlp = MLP(
            n_features + int(m), 1, hidden, activation, seed=seed, output_gain=1.0
        )

    def init(self):
        return self.mlp.params

    def value(self, params, x, a):
        jnp = require_jax_numpy()
        z = jnp.concatenate([self.observe(x), a])
        return self.mlp.compute(None, z, params=params)[0]
