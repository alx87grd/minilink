"""Advantage actor-critic: a critic's temporal-difference errors guide a policy-gradient actor."""

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.planning.reinforcement_learning.algorithms.base import (
    Algorithm,
    optimizer_of,
)
from minilink.planning.reinforcement_learning.collect import advantages

###############################################################################
# Public API
###############################################################################


class ActorCritic(Algorithm):
    """
    Advantage actor-critic: one gradient step per batch on the actor and the critic together.

    The critic ``V_w(x)`` is the baseline; its temporal-difference errors,
    combined by generalized advantage estimation, are the advantage ``A_k``
    the actor moves along: ``θ ← θ + η E[A_k ∇ ln π_θ(a_k|x_k)]``.
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

    on_policy = True
    head_kind = "gaussian"
    critic_kind = "V"

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
        # Hyperparameters
        self.gamma = None if gamma is None else float(gamma)
        self.gae_lambda = float(gae_lambda)
        self.ent_coef = float(ent_coef)
        self.vf_coef = float(vf_coef)
        self.normalize_advantage = bool(normalize_advantage)

        # Optimizer: the one given, or the built-in Adam
        self.optimizer = optimizer_of(optimizer, learning_rate, max_grad_norm)

    def init(self, key):
        """Train state: the weights and the optimizer's moments."""
        params = {"policy": self.policy.init(), "critic": self.critic.init()}
        opt_state = self.optimizer.init(params)

        return {"params": params, "opt": opt_state}

    def loss(self, params, batch, advantage, returns):
        """Minus the advantage-weighted log-likelihood, plus the value regression, minus the entropy bonus."""
        jnp = require_jax_numpy()

        pi = self.policy
        V = self.critic
        c_v = self.vf_coef
        c_e = self.ent_coef
        theta = params["policy"]
        w = params["critic"]

        x = batch["x"]
        a = batch["a"]

        # L_π = −E[A_k ln π_θ(a_k|x_k)], whose gradient is the policy gradient
        L_pi = -jnp.mean(advantage * pi.log_prob(theta, x, a))

        # L_V = E[(R_k − V_w(x_k))²], and the entropy bonus H(π_θ)
        L_V = jnp.mean((returns - V.value(w, x)) ** 2)
        entropy = jnp.mean(pi.entropy(theta, x))

        # L = L_π + c_v L_V − c_e H
        L = L_pi + c_v * L_V - c_e * entropy

        stats = {
            "policy_loss": L_pi,
            "value_loss": L_V,
            "entropy": entropy,
        }

        return L, stats

    def update(self, train_state, batch, key):
        """Advantages of the batch under the current critic, then one gradient step on the whole batch."""
        jax = require_jax()

        V = self.critic
        optimizer = self.optimizer
        gamma = self.gamma
        lam = self.gae_lambda
        normalize = self.normalize_advantage
        params = train_state["params"]
        opt_state = train_state["opt"]
        w = params["critic"]

        # Advantages and returns are targets: computed once, held fixed during the step
        advantage, returns = advantages(V, w, batch, gamma, lam)

        # Â = (A − mean A) / (std A + 10⁻⁸)
        if normalize:
            advantage = (advantage - advantage.mean()) / (advantage.std() + 1e-8)

        # One gradient step on the whole batch
        grad_fn = jax.value_and_grad(self.loss, has_aux=True)
        (_, stats), grads = grad_fn(params, batch, advantage, returns)
        params, opt_state = optimizer.update(grads, opt_state, params)

        return {"params": params, "opt": opt_state}, stats
