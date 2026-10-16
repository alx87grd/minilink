"""Stochastic policies for training: a controller block's mean action with an exploration head around it."""

import functools
import math

from minilink.blocks.neural import MLP
from minilink.core.backends import array_module, require_jax, require_jax_numpy


# The batching decorator the classes below apply as they are defined, so it comes
# first; the critics share it
def over_states(method):
    """Let a per-state method ``(self, params, x, *per_state_args)`` take any batch of states."""

    @functools.wraps(method)
    def batched(self, params, x, *args):
        # One state: the method itself
        if x.ndim == 1:
            return method(self, params, x, *args)

        # A batch: map over the leading axis of the states and of the per-state
        # arguments, recursively until one state is left
        jax = require_jax()
        in_axes = (None, 0) + (0,) * len(args)
        per_state = jax.vmap(lambda p, x, *a: batched(self, p, x, *a), in_axes=in_axes)
        out = per_state(params, x, *args)

        return out

    return batched


###############################################################################
# Public API
###############################################################################


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

    @property
    def deterministic(self):
        """The controller block's own law ``u = pi(x)``, the one used once training is over."""
        return self.controller

    def init(self):
        """Initial parameters: the controller's network and the head's own."""
        jax, jnp = require_jax(), require_jax_numpy()

        mlp = jax.tree_util.tree_map(jnp.asarray, self.controller.params["mlp"])
        head = self.head.init()

        return {"mlp": mlp, "head": head}

    def features(self, x):
        """The policy's features ``z(x)``, the network's input."""
        return self.controller.observe(x)

    def input(self, a):
        """Plant input ``u = u_mid + u_half clip(a, -1, 1)`` of a normalized action."""
        xp = array_module(a)

        u_mid = self.controller.u_mid
        u_half = self.controller.u_half

        u = u_mid + u_half * xp.clip(a, -1.0, 1.0)

        return u

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

        log_std = jnp.full(self.m, self.log_std_init)

        return {"log_std": log_std}

    def sample(self, params, mu, z, key):
        """A normalized action drawn around the mean."""
        jax, jnp = require_jax(), require_jax_numpy()

        log_std = params["log_std"]

        # a = μ + σ ε, ε ~ N(0, I)
        sigma = jnp.exp(log_std)
        eps = jax.random.normal(key, mu.shape)
        a = mu + sigma * eps

        return a

    def log_prob(self, params, mu, z, a):
        """Log-density of ``a`` around ``mu``, summed over the components."""
        jnp = require_jax_numpy()

        log_std = params["log_std"]

        # The unit-Gaussian draw behind a, then ln N = Σᵢ (−εᵢ²/2 − ln σᵢ − ln(2π)/2)
        eps = (a - mu) / jnp.exp(log_std)
        logp = jnp.sum(-0.5 * eps**2 - log_std - 0.5 * math.log(2.0 * math.pi))

        return logp

    def sample_and_log_prob(self, params, mu, z, key):
        """A draw and its log-density."""
        a = self.sample(params, mu, z, key)
        logp = self.log_prob(params, mu, z, a)

        return a, logp

    def entropy(self, params, z):
        """Entropy of the head, the same in every state."""
        jnp = require_jax_numpy()

        log_std = params["log_std"]

        # H = Σᵢ (ln σᵢ + ln(2πe)/2)
        entropy = jnp.sum(log_std + 0.5 * math.log(2.0 * math.pi * math.e))

        return entropy


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
        # Action dimension and the clip of the log-std
        self.m = int(m)
        self.log_std_bounds = tuple(float(b) for b in log_std_bounds)

        # The log-std network on the features, its last layer near zero
        self.mlp = MLP(
            n_features, self.m, hidden, activation, seed=seed, output_gain=0.01
        )

    def init(self):
        """Initial parameters: the weights of the log-std network."""
        jax, jnp = require_jax(), require_jax_numpy()

        log_std_mlp = jax.tree_util.tree_map(jnp.asarray, self.mlp.params)

        return {"log_std_mlp": log_std_mlp}

    def log_std(self, params, z):
        """State-dependent log-std on the features ``z``, clipped to its bounds."""
        jnp = require_jax_numpy()

        mlp = self.mlp
        lo, hi = self.log_std_bounds

        log_std = jnp.clip(mlp.compute(None, z, params=params["log_std_mlp"]), lo, hi)

        return log_std

    def sample_and_log_prob(self, params, mu, z, key):
        """Reparameterized squashed sample and its log-density."""
        jax, jnp = require_jax(), require_jax_numpy()

        log_std = self.log_std(params, z)

        # Reparameterized draw: a = tanh(μ + σ ε), ε ~ N(0, I)
        eps = jax.random.normal(key, mu.shape)
        sigma = jnp.exp(log_std)
        a_gauss = mu + sigma * eps
        a = jnp.tanh(a_gauss)

        # Log-density: the Gaussian's, minus the tanh change of variables Σᵢ ln(1 − aᵢ²)
        log_gauss = jnp.sum(-0.5 * eps**2 - log_std - 0.5 * math.log(2.0 * math.pi))
        log_jacobian = jnp.sum(jnp.log(1.0 - a**2 + 1e-6))
        logp = log_gauss - log_jacobian

        return a, logp

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
        logp = log_gauss - log_jacobian

        return logp

    def entropy(self, params, z):
        """Refuse: a squashed Gaussian's entropy has no closed form."""
        raise NotImplementedError(
            "a squashed Gaussian has no closed-form entropy; estimate it as -ln pi of samples"
        )
