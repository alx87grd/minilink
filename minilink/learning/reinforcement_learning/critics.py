"""Critics: value networks on the policy's features."""

from minilink.blocks.neural import MLP
from minilink.core.backends import require_jax, require_jax_numpy
from minilink.learning.reinforcement_learning.policy import over_states

###############################################################################
# Public API
###############################################################################


class ValueFunction:
    """
    ``V_w(x) = MLP(z(x))`` on the policy's features, the baseline of on-policy methods.

    Its weights ``w`` are a params pytree, kept in the algorithm's train state
    like the policy's. ``value`` takes one state or a batch of states.
    """

    def __init__(
        self, features, n_features, hidden=(64, 64), activation="tanh", seed=1
    ):
        # The features z(x) the network reads
        self.features = features

        # Network shape and seed
        self.n_features = int(n_features)
        self.hidden = tuple(hidden)
        self.activation = activation
        self.seed = int(seed)

        # The network, its weights drawn from the seed
        self.mlp = self.network(self.seed)

    def init(self, seed=None):
        """Initial weights; another ``seed`` gives an independent draw (twin critics)."""
        jax = require_jax()
        jnp = require_jax_numpy()

        mlp = self.mlp if seed is None else self.network(seed)
        w = jax.tree_util.tree_map(jnp.asarray, mlp.params)

        return w

    @over_states
    def value(self, w, x):
        """The state value ``V_w(x)``."""
        mlp = self.mlp
        features = self.features

        # V_w(x) = MLP(z(x)), on the policy's features z
        z = features(x)
        V = mlp.compute(None, z, params=w)[0]

        return V

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def network(self, seed):
        """The MLP from the features to one value, its weights drawn from ``seed``."""
        return MLP(
            self.n_features, 1, self.hidden, self.activation, seed=seed, output_gain=1.0
        )


class QFunction(ValueFunction):
    """``Q_w(x, a) = MLP([z(x), a])``, the action value of off-policy methods."""

    def __init__(
        self, features, n_features, m, hidden=(64, 64), activation="tanh", seed=2
    ):
        # The action width comes first: the base constructor builds the network,
        # which reads it beside the features
        self.m = int(m)
        super().__init__(features, n_features, hidden, activation, seed)

    @over_states
    def value(self, w, x, a):
        """The action value ``Q_w(x, a)``."""
        jnp = require_jax_numpy()

        mlp = self.mlp
        features = self.features

        # Q_w(x, a) = MLP([z(x), a]), the action appended to the features
        z = jnp.concatenate([features(x), a])
        Q = mlp.compute(None, z, params=w)[0]

        return Q

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def network(self, seed):
        """The MLP from the features and the action to one value, its weights drawn from ``seed``."""
        return MLP(
            self.n_features + self.m,
            1,
            self.hidden,
            self.activation,
            seed=seed,
            output_gain=1.0,
        )
