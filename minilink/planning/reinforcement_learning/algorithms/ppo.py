"""Proximal Policy Optimization: the clipped surrogate on an on-policy batch, several epochs of minibatches."""

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.planning.reinforcement_learning.algorithms.base import Algorithm
from minilink.planning.reinforcement_learning.collect import advantages, flatten
from minilink.planning.reinforcement_learning.optim import Adam

# Public API


class PPO(Algorithm):
    """
    Clipped-surrogate policy gradient with a learned value baseline.

    The update of :class:`ActorCritic` with two safeguards: the probability
    ratio ``rho = pi_theta(a|x) / pi_old(a|x)`` is clipped to
    ``[1 - eps, 1 + eps]``, so no minibatch step moves the policy far from
    the one that collected the batch (a trust region), and the batch is reused
    for ``n_epochs`` passes of shuffled minibatches.

    Parameters
    ----------
    learning_rate, gamma, gae_lambda, clip_range, ent_coef, vf_coef,
    max_grad_norm, n_epochs, batch_size
        The usual PPO hyperparameters; ``gamma=None`` lets the planner resolve
        the discount from the task.
    optimizer : object, optional
        Optax-style ``init`` / ``update``; default the built-in :class:`Adam`.
    """

    on_policy, head_kind, critic_kind = True, "gaussian", "V"

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

    def init(self, key):
        """Train state: the weights and the optimizer's moments."""
        params = {"policy": self.policy.init(), "critic": self.critic.init()}
        return {"params": params, "opt": self.optimizer.init(params)}

    def loss(self, params, minibatch):
        """Clipped surrogate, plus the weighted value loss, minus the entropy bonus, on one minibatch."""
        jnp = require_jax_numpy()
        pi, V = self.policy, self.critic
        theta, w = params["policy"], params["critic"]
        x, a = minibatch["x"], minibatch["a"]

        # Probability ratio rho = pi_theta(a|x) / pi_old(a|x) of the stored actions
        ratio = jnp.exp(pi.log_prob(theta, x, a) - minibatch["logp"])

        # Normalized advantage
        advantage = minibatch["advantage"]
        advantage = (advantage - advantage.mean()) / (advantage.std() + 1e-8)

        # Clipped surrogate: L_clip = E[min(rho A, clip(rho, 1 - eps, 1 + eps) A)]
        eps = self.clip_range
        clipped = jnp.clip(ratio, 1.0 - eps, 1.0 + eps)
        policy_loss = -jnp.mean(jnp.minimum(ratio * advantage, clipped * advantage))

        # Value regression E[(R - V_w(x))^2], and the entropy bonus H(pi)
        value_loss = jnp.mean((minibatch["return"] - V.value(w, x)) ** 2)
        entropy = jnp.mean(pi.entropy(theta, x))

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
        params, opt_state = train_state["params"], train_state["opt"]
        theta, w = params["policy"], params["critic"]

        # The batch as the collecting policy saw it: ln pi_old(a|x), advantages and returns
        advantage, returns = advantages(
            self.critic, w, batch, self.gamma, self.gae_lambda
        )
        samples = flatten(
            {
                "x": batch["x"],
                "a": batch["a"],
                "logp": self.policy.log_prob(theta, batch["x"], batch["a"]),
                "advantage": advantage,
                "return": returns,
            }
        )
        size = samples["x"].shape[0]
        if size % self.batch_size != 0:
            raise ValueError("n_steps * n_envs must be a multiple of batch_size")
        n_minibatches = size // self.batch_size

        # Gradient steps on shuffled minibatches, n_epochs passes over the batch
        grad_fn = jax.value_and_grad(self.loss, has_aux=True)

        def minibatch_step(carry, idx):
            params, opt_state = carry
            (_, stats), grads = grad_fn(params, {k: v[idx] for k, v in samples.items()})
            params, opt_state = self.optimizer.update(grads, opt_state, params)
            return (params, opt_state), stats

        def epoch(carry, key):
            perm = jax.random.permutation(key, size).reshape(n_minibatches, -1)
            return jax.lax.scan(minibatch_step, carry, perm)

        (params, opt_state), stats = jax.lax.scan(
            epoch, (params, opt_state), jax.random.split(key, self.n_epochs)
        )
        stats = {k: jnp.mean(v) for k, v in stats.items()}
        return {"params": params, "opt": opt_state}, stats
