"""Advantage actor-critic: a critic's temporal-difference errors guide a policy-gradient actor."""

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.planning.reinforcement_learning.algorithms.base import Algorithm
from minilink.planning.reinforcement_learning.collect import advantages
from minilink.planning.reinforcement_learning.optim import Adam

# Public API


class ActorCritic(Algorithm):
    """
    Advantage actor-critic: one gradient step per batch on the actor and the critic together.

    The critic ``V_w(x)`` is the baseline; its temporal-difference errors,
    combined by generalized advantage estimation, are the advantage ``A_k``
    the actor moves along: ``theta <- theta + eta E[A_k grad ln pi_theta(a_k|x_k)]``.
    ``gae_lambda = 0`` is the one-step TD error, ``1`` the Monte Carlo return
    minus the baseline. :class:`PPO` is this update with a clipped ratio and
    several passes over the batch.

    Parameters
    ----------
    learning_rate, gamma, gae_lambda, ent_coef, vf_coef, max_grad_norm
        Step size, discount (``None``: resolved by the planner), advantage
        mixing, entropy bonus, value-loss weight and gradient clip.
    normalize_advantage : bool
        Standardize the advantages of the batch before the step.
    optimizer : object, optional
        Optax-style ``init`` / ``update``; default the built-in :class:`Adam`.
    """

    on_policy, head_kind, critic_kind = True, "gaussian", "V"

    def __init__(
        self,
        *,
        learning_rate=1e-3,
        gamma=None,
        gae_lambda=0.95,
        ent_coef=0.0,
        vf_coef=0.5,
        max_grad_norm=0.5,
        normalize_advantage=True,
        optimizer=None,
    ):
        self.gamma = None if gamma is None else float(gamma)
        self.gae_lambda = float(gae_lambda)
        self.ent_coef = float(ent_coef)
        self.vf_coef = float(vf_coef)
        self.normalize_advantage = bool(normalize_advantage)
        self.optimizer = (
            Adam(learning_rate, max_grad_norm=max_grad_norm)
            if optimizer is None
            else optimizer
        )

    def init(self, key):
        """Train state: the weights and the optimizer's moments."""
        params = {"policy": self.policy.init(), "critic": self.critic.init()}
        return {"params": params, "opt": self.optimizer.init(params)}

    def loss(self, params, batch, advantage, returns):
        """Minus the advantage-weighted log-likelihood, plus the value regression, minus the entropy bonus."""
        jnp = require_jax_numpy()
        pi, V = self.policy, self.critic
        theta, w = params["policy"], params["critic"]
        x, a = batch["x"], batch["a"]

        # Actor: -E[A_k ln pi_theta(a_k | x_k)], whose gradient is the policy gradient
        policy_loss = -jnp.mean(advantage * pi.log_prob(theta, x, a))

        # Critic: E[(R_k - V_w(x_k))^2]; and the entropy bonus H(pi)
        value_loss = jnp.mean((returns - V.value(w, x)) ** 2)
        entropy = jnp.mean(pi.entropy(theta, x))

        total = policy_loss + self.vf_coef * value_loss - self.ent_coef * entropy
        stats = {
            "policy_loss": policy_loss,
            "value_loss": value_loss,
            "entropy": entropy,
        }
        return total, stats

    def update(self, train_state, batch, key):
        """Advantages of the batch under the current critic, then one gradient step on the whole batch."""
        jax = require_jax()
        params, opt_state = train_state["params"], train_state["opt"]

        # Advantages and returns are targets: computed once, held fixed during the step
        advantage, returns = advantages(
            self.critic, params["critic"], batch, self.gamma, self.gae_lambda
        )
        if self.normalize_advantage:
            advantage = (advantage - advantage.mean()) / (advantage.std() + 1e-8)

        (_, stats), grads = jax.value_and_grad(self.loss, has_aux=True)(
            params, batch, advantage, returns
        )
        params, opt_state = self.optimizer.update(grads, opt_state, params)
        return {"params": params, "opt": opt_state}, stats
