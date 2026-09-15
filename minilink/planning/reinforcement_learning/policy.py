"""Stochastic policies for training: a controller block's mean action with an exploration head around it."""

import functools
import math

from minilink.blocks.neural import MLP
from minilink.core.backends import array_module, require_jax, require_jax_numpy


def over_states(method):
    """Let a per-state method ``(self, params, x, *per_state_args)`` take any batch of states."""

    @functools.wraps(method)
    def batched(self, params, x, *args):
        if x.ndim == 1:
            return method(self, params, x, *args)
        jax = require_jax()
        in_axes = (None, 0) + (0,) * len(args)
        return jax.vmap(lambda p, x, *a: batched(self, p, x, *a), in_axes=in_axes)(
            params, x, *args
        )

    return batched


# Public API


class StochasticPolicy:
    """
    ``pi_theta(a | x)``: the law of a controller block, made stochastic by an exploration head.

    The controller, a :class:`~minilink.control.neural.NeuralPolicyController`,
    gives the normalized mean action ``mu_theta(x) = MLP(z(x))`` and the map
    from a normalized action onto the plant's input port. The head gives the
    distribution around the mean: ``sample``, ``log_prob``, ``entropy``. The
    parameters ``theta = {"mlp", "head"}`` form one pytree, so an algorithm
    differentiates through both, and ``theta["mlp"]`` is what the controller
    block carries once training is over.

    Every method takes one state ``x`` or a batch of states (leading axes),
    with one key or one action per state.
    """

    def __init__(self, controller, head):
        self.controller = controller
        self.head = head
        self.m = int(head.m)

    def init(self):
        """Initial parameters: the controller's network and the head's own."""
        jax, jnp = require_jax(), require_jax_numpy()
        mlp = jax.tree_util.tree_map(jnp.asarray, self.controller.params["mlp"])
        return {"mlp": mlp, "head": self.head.init()}

    def features(self, x):
        """The policy's features ``z(x)``, the network's input."""
        return self.controller.observe(x)

    def input(self, a):
        """Plant input ``u = u_mid + u_half clip(a, -1, 1)`` of a normalized action."""
        xp = array_module(a)
        ctl = self.controller
        return ctl.u_mid + ctl.u_half * xp.clip(a, -1.0, 1.0)

    @property
    def deterministic(self):
        """The controller block's own law ``u = pi(x)``, the one used once training is over."""
        return self.controller

    def mean(self, theta, x):
        """Normalized mean action ``mu_theta(x)``."""
        return self.controller.mean_action(x, {"mlp": theta["mlp"]})

    @over_states
    def sample(self, theta, x, key):
        """A normalized action ``a ~ pi_theta(. | x)``."""
        return self.head.sample(
            theta["head"], self.mean(theta, x), self.features(x), key
        )

    @over_states
    def log_prob(self, theta, x, a):
        """Log-density ``ln pi_theta(a | x)`` of a normalized action."""
        return self.head.log_prob(
            theta["head"], self.mean(theta, x), self.features(x), a
        )

    @over_states
    def sample_and_log_prob(self, theta, x, key):
        """A reparameterized draw and its log-density, the pair that off-policy losses differentiate through."""
        return self.head.sample_and_log_prob(
            theta["head"], self.mean(theta, x), self.features(x), key
        )

    @over_states
    def entropy(self, theta, x):
        """Entropy ``H(pi_theta(. | x))``."""
        return self.head.entropy(theta["head"], self.features(x))


class GaussianHead:
    """
    Diagonal Gaussian ``a ~ N(mu, sigma^2)`` with a learned, state-independent log-std.

    The exploration of the policy-gradient family: the plant receives
    ``clip(a, -1, 1)`` while the density is the unclipped Gaussian's, as in the
    reference implementations. Features ``z`` are accepted and ignored: the
    spread does not depend on the state.
    """

    def __init__(self, m, log_std_init=0.0):
        self.m = int(m)
        self.log_std_init = float(log_std_init)

    def init(self):
        """Initial parameters: one log-std per action component."""
        jnp = require_jax_numpy()
        return {"log_std": jnp.full(self.m, self.log_std_init)}

    def sample(self, params, mu, z, key):
        """A normalized action drawn around the mean."""
        jax, jnp = require_jax(), require_jax_numpy()

        # a = mu + sigma eps, eps ~ N(0, I)
        sigma = jnp.exp(params["log_std"])
        eps = jax.random.normal(key, mu.shape)
        return mu + sigma * eps

    def log_prob(self, params, mu, z, a):
        """Log-density of ``a`` around ``mu``, summed over the components."""
        jnp = require_jax_numpy()
        log_std = params["log_std"]

        # The unit-Gaussian draw behind a, then ln N = -eps^2/2 - ln sigma - ln(2 pi)/2
        eps = (a - mu) / jnp.exp(log_std)
        return jnp.sum(-0.5 * eps**2 - log_std - 0.5 * math.log(2.0 * math.pi))

    def sample_and_log_prob(self, params, mu, z, key):
        """A draw and its log-density."""
        a = self.sample(params, mu, z, key)
        return a, self.log_prob(params, mu, z, a)

    def entropy(self, params, z):
        """Entropy of the head, the same in every state."""
        jnp = require_jax_numpy()

        # H = sum_i (ln sigma_i + ln(2 pi e)/2)
        return jnp.sum(params["log_std"] + 0.5 * math.log(2.0 * math.pi * math.e))


class SquashedGaussianHead:
    """
    Tanh-squashed Gaussian with a state-dependent log-std network (the SAC family).

    The head owns the log-std network on the policy's features ``z``. Its
    log-density includes the tanh change of variables, and
    ``sample_and_log_prob`` is the reparameterized draw the actor and
    temperature losses differentiate through. The entropy has no closed form.
    """

    def __init__(
        self,
        n_features,
        m,
        hidden=(64, 64),
        activation="tanh",
        log_std_bounds=(-5.0, 2.0),
        seed=3,
    ):
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

        # Log-density: the Gaussian's, minus the tanh change of variables sum ln(1 - a^2)
        log_gauss = jnp.sum(-0.5 * eps**2 - log_std - 0.5 * math.log(2.0 * math.pi))
        log_jacobian = jnp.sum(jnp.log(1.0 - a**2 + 1e-6))
        return a, log_gauss - log_jacobian

    def sample(self, params, mu, z, key):
        """A squashed normalized action drawn around the mean."""
        return self.sample_and_log_prob(params, mu, z, key)[0]

    def log_prob(self, params, mu, z, a):
        """Log-density of a squashed action ``a`` in ``(-1, 1)``."""
        jnp = require_jax_numpy()
        log_std = self.log_std(params, z)

        # Undo the squash, then the Gaussian's density minus the change of variables
        a_gauss = jnp.arctanh(jnp.clip(a, -1.0 + 1e-6, 1.0 - 1e-6))
        eps = (a_gauss - mu) / jnp.exp(log_std)
        log_gauss = jnp.sum(-0.5 * eps**2 - log_std - 0.5 * math.log(2.0 * math.pi))
        log_jacobian = jnp.sum(jnp.log(1.0 - a**2 + 1e-6))
        return log_gauss - log_jacobian

    def entropy(self, params, z):
        raise NotImplementedError(
            "a squashed Gaussian has no closed-form entropy; estimate it as -ln pi of samples"
        )
