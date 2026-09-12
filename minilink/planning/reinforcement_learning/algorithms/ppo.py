"""Proximal Policy Optimization: the clipped-surrogate update on an on-policy batch."""

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.planning.reinforcement_learning.algorithms.base import Algorithm
from minilink.planning.reinforcement_learning.collect import gae
from minilink.planning.reinforcement_learning.optim import Adam

# Public API


class PPO(Algorithm):
    """
    Clipped-surrogate policy gradient with a learned value baseline.

    As in the reference implementations, the Gaussian sample is stored
    unclipped and its density is the unclipped one, while the plant receives
    ``clip(a, -1, 1)``; the squashed head of the SAC family is the alternative
    when that mismatch matters.

    Parameters
    ----------
    learning_rate, gamma, gae_lambda, clip_range, ent_coef, vf_coef,
    max_grad_norm, n_epochs, batch_size
        The usual PPO hyperparameters; ``gamma`` and ``gae_lambda`` also
        drive the advantage estimation of the collected batch. ``gamma=None``
        lets the planner resolve the discount from the task.
    optimizer : object, optional
        Optax-style ``init`` / ``update``; default the built-in :class:`Adam`.
    """

    on_policy = True

    def __init__(
        self,
        *,
        learning_rate=3e-4,
        gamma=None,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        vf_coef=0.5,
        max_grad_norm=0.5,
        n_epochs=10,
        batch_size=64,
        optimizer=None,
    ):
        self.gamma = None if gamma is None else float(gamma)
        self.gae_lambda = float(gae_lambda)
        self.clip_range = float(clip_range)
        self.ent_coef = float(ent_coef)
        self.vf_coef = float(vf_coef)
        self.n_epochs = int(n_epochs)
        self.batch_size = int(batch_size)
        self.optimizer = (
            Adam(learning_rate, max_grad_norm=max_grad_norm)
            if optimizer is None
            else optimizer
        )

    def init(self, key, params):
        """Train state: the weights and the optimizer's moments."""
        return {"params": params, "opt": self.optimizer.init(params)}

    def loss(self, params, minibatch):
        """Clipped surrogate, plus the weighted value loss, minus the entropy bonus, on one minibatch."""
        jax, jnp = require_jax(), require_jax_numpy()
        functions = self.functions
        x, a = minibatch["x"], minibatch["a"]

        # Current policy: mean action, log-density of the stored actions, baseline V(x)
        mu = jax.vmap(functions.mean, in_axes=(None, 0))(params["actor"], x)
        logp = jax.vmap(functions.head.log_prob, in_axes=(None, 0, 0))(
            params["head"], mu, a
        )
        value = jax.vmap(functions.value, in_axes=(None, 0))(params["critic"], x)

        # Normalized advantage, and the probability ratio pi(a|x) / pi_old(a|x)
        advantage = minibatch["advantage"]
        advantage = (advantage - advantage.mean()) / (advantage.std() + 1e-8)
        ratio = jnp.exp(logp - minibatch["logp"])

        # Clipped surrogate: -E[min(ratio A, clip(ratio, 1 - clip_range, 1 + clip_range) A)]
        clipped = jnp.clip(ratio, 1.0 - self.clip_range, 1.0 + self.clip_range)
        policy_loss = -jnp.mean(jnp.minimum(advantage * ratio, advantage * clipped))

        # Value regression E[(R - V)^2], and the entropy of the exploration head
        value_loss = jnp.mean((minibatch["return"] - value) ** 2)
        entropy = functions.head.entropy(params["head"])

        total = policy_loss + self.vf_coef * value_loss - self.ent_coef * entropy

        # Monitoring only: an estimate of how far the update moved the policy
        stats = {
            "policy_loss": policy_loss,
            "value_loss": value_loss,
            "entropy": entropy,
            "approx_kl": jnp.mean((ratio - 1.0) - jnp.log(ratio)),
        }
        return total, stats

    def update(self, train_state, batch, key):
        """Advantages of the batch, then ``n_epochs`` passes of minibatch gradient steps."""
        jax, jnp = require_jax(), require_jax_numpy()

        # Advantages and returns of the whole batch, by generalized advantage estimation
        advantage, returns = gae(
            batch, batch["last_value"], self.gamma, self.gae_lambda
        )

        # Time steps and plants flattened into one batch of samples
        n_steps, n_envs = batch["reward"].shape
        size = n_steps * n_envs
        if size % self.batch_size != 0:
            raise ValueError("n_steps * n_envs must be a multiple of batch_size")
        n_minibatches = size // self.batch_size
        flat = {
            "x": batch["x"].reshape((size, -1)),
            "a": batch["a"].reshape((size, -1)),
            "logp": batch["logp"].reshape(size),
            "advantage": advantage.reshape(size),
            "return": returns.reshape(size),
        }

        # Gradient steps on shuffled minibatches, n_epochs passes over the batch
        grad_fn = jax.value_and_grad(self.loss, has_aux=True)

        def minibatch_step(carry, idx):
            params, opt_state = carry
            minibatch = {k: v[idx] for k, v in flat.items()}
            (_, stats), grads = grad_fn(params, minibatch)
            params, opt_state = self.optimizer.update(grads, opt_state, params)
            return (params, opt_state), stats

        def epoch(carry, key):
            perm = jax.random.permutation(key, size).reshape(n_minibatches, -1)
            return jax.lax.scan(minibatch_step, carry, perm)

        keys = jax.random.split(key, self.n_epochs)
        (params, opt_state), stats = jax.lax.scan(
            epoch, (train_state["params"], train_state["opt"]), keys
        )
        stats = {k: jnp.mean(v) for k, v in stats.items()}
        return {"params": params, "opt": opt_state}, stats
