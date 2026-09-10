"""
Stochastic policy heads: the exploration distribution on top of a mean network.

A head owns the distribution's own parameters (a state-independent log-std for
the Gaussian family) and the three operations every policy-gradient method
needs on the normalized action ``a``: ``sample``, ``log_prob`` and ``entropy``.
The mean ``mu(x)`` comes from the policy block; the plant sees
``clip(a, -1, 1)`` mapped to the port bounds.
"""

import math

from minilink.core.backends import require_jax_numpy

# Public API


class GaussianHead:
    """Diagonal Gaussian ``a ~ N(mu, exp(log_std)^2)`` with a learned, state-independent log-std."""

    def __init__(self, m, log_std_init=0.0):
        self.m = int(m)
        self.log_std_init = float(log_std_init)

    def init(self):
        jnp = require_jax_numpy()
        return {"log_std": jnp.full(self.m, self.log_std_init)}

    def sample(self, params, mu, key, z=None):
        jnp = require_jax_numpy()
        import jax

        return mu + jnp.exp(params["log_std"]) * jax.random.normal(key, mu.shape)

    def log_prob(self, params, mu, a):
        jnp = require_jax_numpy()
        log_std = params["log_std"]
        z = (a - mu) / jnp.exp(log_std)
        return jnp.sum(-0.5 * z**2 - log_std - 0.5 * math.log(2.0 * math.pi))

    def entropy(self, params):
        jnp = require_jax_numpy()
        return jnp.sum(params["log_std"] + 0.5 * math.log(2.0 * math.pi * math.e))


class SquashedGaussianHead:
    """
    Tanh-squashed Gaussian with a state-dependent log-std (the SAC family).

    ``a = tanh(mu + exp(log_std(z)) * eps)`` with ``eps ~ N(0, I)``; the head
    owns the log-std network on the policy's features ``z``. The log-density
    includes the tanh change of variables, and ``sample_and_log_prob`` is the
    reparameterized draw actor and temperature losses differentiate through.
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
        from minilink.blocks.neural import MLP

        self.observe = observe
        self.m = int(m)
        self.log_std_bounds = tuple(float(b) for b in log_std_bounds)
        self.mlp = MLP(
            n_features, self.m, hidden, activation, seed=seed, output_gain=0.01
        )

    def init(self):
        jnp = require_jax_numpy()
        import jax

        return {"log_std_mlp": jax.tree_util.tree_map(jnp.asarray, self.mlp.params)}

    def log_std(self, params, z):
        jnp = require_jax_numpy()
        lo, hi = self.log_std_bounds
        return jnp.clip(self.mlp.compute(None, z, params=params["log_std_mlp"]), lo, hi)

    def sample_and_log_prob(self, params, mu, z, key):
        """Reparameterized squashed sample and its log-density."""
        jnp = require_jax_numpy()
        import jax

        log_std = self.log_std(params, z)
        eps = jax.random.normal(key, mu.shape)
        pre = mu + jnp.exp(log_std) * eps
        a = jnp.tanh(pre)
        gaussian = jnp.sum(-0.5 * eps**2 - log_std - 0.5 * math.log(2.0 * math.pi))
        squash = jnp.sum(jnp.log(1.0 - a**2 + 1e-6))
        return a, gaussian - squash

    def sample(self, params, mu, key, z=None):
        return self.sample_and_log_prob(params, mu, z, key)[0]

    @staticmethod
    def deterministic(mu):
        jnp = require_jax_numpy()
        return jnp.tanh(mu)
