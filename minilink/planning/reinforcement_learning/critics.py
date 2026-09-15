"""Critics: value networks on the policy's features."""

from minilink.blocks.neural import MLP
from minilink.core.backends import require_jax, require_jax_numpy
from minilink.planning.reinforcement_learning.policy import over_states

# Public API


class ValueFunction:
    """
    ``V_w(x) = MLP(z(x))`` on the policy's features, the baseline of on-policy methods.

    Its weights ``w`` are a params pytree, kept in the algorithm's train state
    like the policy's. ``value`` takes one state or a batch of states.
    """

    def __init__(
        self, features, n_features, hidden=(64, 64), activation="tanh", seed=1
    ):
        self.features = features
        self.n_features, self.hidden, self.activation, self.seed = (
            int(n_features),
            tuple(hidden),
            activation,
            int(seed),
        )
        self.mlp = self.network(self.seed)

    def init(self, seed=None):
        """Initial weights; another ``seed`` gives an independent draw (twin critics)."""
        jax, jnp = require_jax(), require_jax_numpy()
        mlp = self.mlp if seed is None else self.network(seed)
        return jax.tree_util.tree_map(jnp.asarray, mlp.params)

    @over_states
    def value(self, w, x):
        """The state value ``V_w(x)``."""
        return self.mlp.compute(None, self.features(x), params=w)[0]

    def network(self, seed):
        return MLP(
            self.n_features, 1, self.hidden, self.activation, seed=seed, output_gain=1.0
        )


class QFunction(ValueFunction):
    """``Q_w(x, a) = MLP([z(x), a])``, the action value of off-policy methods."""

    def __init__(
        self, features, n_features, m, hidden=(64, 64), activation="tanh", seed=2
    ):
        self.m = int(m)
        super().__init__(features, n_features, hidden, activation, seed)

    @over_states
    def value(self, w, x, a):
        """The action value ``Q_w(x, a)``."""
        jnp = require_jax_numpy()
        z = jnp.concatenate([self.features(x), a])
        return self.mlp.compute(None, z, params=w)[0]

    def network(self, seed):
        return MLP(
            self.n_features + self.m,
            1,
            self.hidden,
            self.activation,
            seed=seed,
            output_gain=1.0,
        )
