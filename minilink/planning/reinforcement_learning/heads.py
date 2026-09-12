"""Stochastic policy heads: the exploration distribution on top of a mean network."""

import math

from minilink.blocks.neural import MLP
from minilink.core.backends import require_jax, require_jax_numpy

# Public API


class GaussianHead:
    """
    Diagonal Gaussian ``a ~ N(mu, exp(log_std)^2)`` with a learned, state-independent log-std.

    The head owns the distribution's parameters and the three operations a
    policy-gradient method needs on the normalized action ``a``: sample,
    log-density and entropy. The mean ``mu(x)`` comes from the policy block,
    and the plant receives ``clip(a, -1, 1)`` mapped to the port bounds.
    """

    def __init__(self, m, log_std_init=0.0):
        self.m = int(m)
        self.log_std_init = float(log_std_init)

    def init(self):
        """Initial parameters: one log-std per action component."""
        jnp = require_jax_numpy()
        return {"log_std": jnp.full(self.m, self.log_std_init)}

    def sample(self, params, mu, key, z=None):
        """A normalized action drawn around the mean; ``z`` is unused, the spread ignores the state."""
        jax, jnp = require_jax(), require_jax_numpy()

        # a = mu + sigma eps, eps ~ N(0, I)
        sigma = jnp.exp(params["log_std"])
        eps = jax.random.normal(key, mu.shape)
        return mu + sigma * eps

    def log_prob(self, params, mu, a):
        """Log-density of the action ``a`` around ``mu``, summed over its components."""
        jnp = require_jax_numpy()
        log_std = params["log_std"]

        # The unit-Gaussian draw that produced a, then log N = -eps^2/2 - log sigma - log(2 pi)/2
        eps = (a - mu) / jnp.exp(log_std)
        return jnp.sum(-0.5 * eps**2 - log_std - 0.5 * math.log(2.0 * math.pi))

    def entropy(self, params):
        """Entropy of the head, the same for every state."""
        jnp = require_jax_numpy()

        # H = sum_i (log sigma_i + log(2 pi e)/2)
        return jnp.sum(params["log_std"] + 0.5 * math.log(2.0 * math.pi * math.e))


class SquashedGaussianHead:
    """
    Tanh-squashed Gaussian with a state-dependent log-std (the SAC family).

    The head owns the log-std network on the policy's features ``z``. Its
    log-density includes the tanh change of variables, and
    ``sample_and_log_prob`` is the reparameterized draw the actor and
    temperature losses differentiate through.
    """

    def __init__(
        self,
        observe,
        n_features,
        m,
        hidden=(64, 64),
        activation="tanh",
        log_std_bounds=(-5.0, 2.0),
        seed=3,
    ):
        self.observe = observe
        self.m = int(m)
        self.log_std_bounds = tuple(float(b) for b in log_std_bounds)
        self.mlp = MLP(
            n_features, self.m, hidden, activation, seed=seed, output_gain=0.01
        )

    def init(self):
        """Initial parameters: the weights of the log-std network."""
        jax, jnp = require_jax(), require_jax_numpy()
        return {"log_std_mlp": jax.tree_util.tree_map(jnp.asarray, self.mlp.params)}

    def log_std(self, params, z):
        """State-dependent log-std on the features ``z``, clipped to its bounds."""
        jnp = require_jax_numpy()
        lo, hi = self.log_std_bounds
        return jnp.clip(self.mlp.compute(None, z, params=params["log_std_mlp"]), lo, hi)

    def sample_and_log_prob(self, params, mu, z, key):
        """Reparameterized squashed sample and its log-density."""
        jax, jnp = require_jax(), require_jax_numpy()
        log_std = self.log_std(params, z)

        # Reparameterized draw: a = tanh(mu + sigma eps), eps ~ N(0, I)
        eps = jax.random.normal(key, mu.shape)
        sigma = jnp.exp(log_std)
        a_gauss = mu + sigma * eps
        a = jnp.tanh(a_gauss)

        # Log-density: the Gaussian's, minus the tanh change of variables sum log(1 - a^2)
        log_gauss = jnp.sum(-0.5 * eps**2 - log_std - 0.5 * math.log(2.0 * math.pi))
        log_jacobian = jnp.sum(jnp.log(1.0 - a**2 + 1e-6))
        return a, log_gauss - log_jacobian

    def sample(self, params, mu, key, z=None):
        """A squashed normalized action drawn around the mean action."""
        return self.sample_and_log_prob(params, mu, z, key)[0]

    @staticmethod
    def deterministic(mu):
        """The squashed mean action, the law used once training is over."""
        jnp = require_jax_numpy()
        return jnp.tanh(mu)
