"""Proximal Policy Optimization: the clipped surrogate on an on-policy batch, several epochs of minibatches."""

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.planning.reinforcement_learning.algorithms.base import (
    Algorithm,
    optimizer_of,
)
from minilink.planning.reinforcement_learning.collect import advantages, flatten

###############################################################################
# Public API
###############################################################################


class PPO(Algorithm):
    """
    Clipped-surrogate policy gradient with a learned value baseline.

    The update of :class:`ActorCritic` with two safeguards: the probability
    ratio ``π_θ(a|x) / π_old(a|x)`` is clipped to ``[1 − ε, 1 + ε]``, so no
    minibatch step moves the policy far from the one that collected the batch
    (a trust region), and the batch is reused for ``n_epochs`` passes of
    shuffled minibatches.

    Parameters
    ----------
    learning_rate, gamma, gae_lambda, clip_range, ent_coef, vf_coef,
    max_grad_norm, n_epochs, batch_size
        The usual PPO hyperparameters; ``gamma=None`` lets the planner resolve
        the discount from the task.
    optimizer : object, optional
        Optax-style ``init`` / ``update``; default the built-in :class:`Adam`.
    """

    on_policy = True
    head_kind = "gaussian"
    critic_kind = "V"

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
        # Hyperparameters
        self.gamma = None if gamma is None else float(gamma)
        self.gae_lambda = float(gae_lambda)
        self.clip_range = float(clip_range)
        self.ent_coef = float(ent_coef)
        self.vf_coef = float(vf_coef)

        # Passes over the batch: n_epochs of shuffled minibatches of batch_size samples
        self.n_epochs = int(n_epochs)
        self.batch_size = int(batch_size)

        # Optimizer: the one given, or the built-in Adam
        self.optimizer = optimizer_of(optimizer, learning_rate, max_grad_norm)

    def init(self, key):
        """Train state: the weights and the optimizer's moments."""
        params = {"policy": self.policy.init(), "critic": self.critic.init()}
        opt_state = self.optimizer.init(params)

        return {"params": params, "opt": opt_state}

    def loss(self, params, minibatch):
        """Clipped surrogate, plus the weighted value loss, minus the entropy bonus, on one minibatch."""
        jnp = require_jax_numpy()

        pi = self.policy
        V = self.critic
        eps = self.clip_range
        c_v = self.vf_coef
        c_e = self.ent_coef
        theta = params["policy"]
        w = params["critic"]

        x = minibatch["x"]
        a = minibatch["a"]
        logp_old = minibatch["logp"]
        advantage = minibatch["advantage"]
        returns = minibatch["return"]

        # ratio = π_θ(a|x) / π_old(a|x), on the stored actions
        ratio = jnp.exp(pi.log_prob(theta, x, a) - logp_old)

        # Â = (A − mean A) / (std A + 10⁻⁸)
        advantage = (advantage - advantage.mean()) / (advantage.std() + 1e-8)

        # L_clip = −E[min(ratio Â, clip(ratio, 1 − ε, 1 + ε) Â)]
        clipped = jnp.clip(ratio, 1.0 - eps, 1.0 + eps)
        L_clip = -jnp.mean(jnp.minimum(ratio * advantage, clipped * advantage))

        # L_V = E[(R − V_w(x))²], and the entropy bonus H(π_θ)
        L_V = jnp.mean((returns - V.value(w, x)) ** 2)
        entropy = jnp.mean(pi.entropy(theta, x))

        # L = L_clip + c_v L_V − c_e H
        L = L_clip + c_v * L_V - c_e * entropy

        # Monitoring only: an estimate of how far the update moved the policy
        approx_kl = jnp.mean((ratio - 1.0) - jnp.log(ratio))
        stats = {
            "policy_loss": L_clip,
            "value_loss": L_V,
            "entropy": entropy,
            "approx_kl": approx_kl,
        }

        return L, stats

    def update(self, train_state, batch, key):
        """Advantages of the batch, then ``n_epochs`` passes of minibatch gradient steps."""
        jax = require_jax()
        jnp = require_jax_numpy()

        pi = self.policy
        V = self.critic
        optimizer = self.optimizer
        gamma = self.gamma
        lam = self.gae_lambda
        n_epochs = self.n_epochs
        batch_size = self.batch_size
        params = train_state["params"]
        opt_state = train_state["opt"]
        theta = params["policy"]
        w = params["critic"]

        # The batch as the collecting policy saw it: advantages, returns and ln π_old(a|x)
        advantage, returns = advantages(V, w, batch, gamma, lam)
        x = batch["x"]
        a = batch["a"]
        logp_old = pi.log_prob(theta, x, a)
        samples = flatten(
            {
                "x": x,
                "a": a,
                "logp": logp_old,
                "advantage": advantage,
                "return": returns,
            }
        )

        # The samples split into whole minibatches
        size = samples["x"].shape[0]
        require_whole_minibatches(size, batch_size)
        n_minibatches = size // batch_size

        # Gradient steps on shuffled minibatches, n_epochs passes over the batch
        grad_fn = jax.value_and_grad(self.loss, has_aux=True)

        def minibatch_step(carry, idx):
            params, opt_state = carry
            minibatch = {k: v[idx] for k, v in samples.items()}
            (_, stats), grads = grad_fn(params, minibatch)
            params, opt_state = optimizer.update(grads, opt_state, params)
            return (params, opt_state), stats

        def epoch(carry, key):
            perm = jax.random.permutation(key, size).reshape(n_minibatches, -1)
            return jax.lax.scan(minibatch_step, carry, perm)

        (params, opt_state), stats = jax.lax.scan(
            epoch, (params, opt_state), jax.random.split(key, n_epochs)
        )

        # Each statistic averaged over every minibatch step
        stats = {k: jnp.mean(v) for k, v in stats.items()}

        return {"params": params, "opt": opt_state}, stats


###############################################################################
# Internal machinery
###############################################################################


def require_whole_minibatches(size, batch_size):
    """Refuse a batch that does not split into whole minibatches."""
    if size % batch_size != 0:
        raise ValueError("n_steps * n_envs must be a multiple of batch_size")
