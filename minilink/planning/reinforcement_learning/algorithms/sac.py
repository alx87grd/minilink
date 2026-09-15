"""Soft Actor-Critic: the off-policy update on replayed transitions."""

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.planning.reinforcement_learning.algorithms.base import Algorithm
from minilink.planning.reinforcement_learning.optim import Adam

# Public API


class SAC(Algorithm):
    """
    Soft Actor-Critic.

    Twin action-value critics with Polyak-averaged targets, a squashed
    Gaussian actor trained by the reparameterized gradient, and a learned
    temperature ``alpha`` that holds the policy entropy at a target.

    Parameters
    ----------
    learning_rate, gamma, tau, batch_size
        Step size (actor, critics, temperature), discount, Polyak rate, replay
        minibatch. ``gamma=None`` lets the planner resolve the discount from
        the task.
    buffer_size, learning_starts : int
        Replay capacity and the number of stored transitions before updates.
    gradient_steps : int, optional
        Updates per collected step (default one per environment step).
    target_entropy : float, optional
        Entropy target of the temperature loss; default ``-m``.
    """

    on_policy, head_kind, critic_kind = False, "squashed", "Q"

    def __init__(
        self,
        *,
        learning_rate=3e-4,
        gamma=None,
        tau=0.005,
        batch_size=256,
        buffer_size=200_000,
        learning_starts=2_000,
        gradient_steps=None,
        target_entropy=None,
        max_grad_norm=None,
    ):
        self.gamma = None if gamma is None else float(gamma)
        self.tau = float(tau)
        self.batch_size = int(batch_size)
        self.buffer_size = int(buffer_size)
        self.learning_starts = int(learning_starts)
        self.gradient_steps = gradient_steps
        self.target_entropy = target_entropy
        self.optimizer = Adam(learning_rate, max_grad_norm=max_grad_norm)

    def init(self, key):
        """Train state: weights, twin critics with their targets, log-temperature, one optimizer state each."""
        jnp = require_jax_numpy()
        pi, Q = self.policy, self.critic
        if self.target_entropy is None:
            self.target_entropy = -float(pi.m)
        theta = pi.init()
        w = {"q1": Q.init(), "q2": Q.init(Q.seed + 1)}
        return {
            "params": {"policy": theta, "critic": w},
            "target": w,
            "log_alpha": jnp.asarray(0.0),
            "opt": {
                "actor": self.optimizer.init(theta),
                "critic": self.optimizer.init(w),
                "alpha": self.optimizer.init(jnp.asarray(0.0)),
            },
        }

    def update(self, train_state, batch, key):
        """One SAC step on a replay minibatch ``{x, a, reward, x_next, terminated}``."""
        jax, jnp = require_jax(), require_jax_numpy()
        pi, Q = self.policy, self.critic
        params, opt = train_state["params"], train_state["opt"]
        theta, w, w_target = params["policy"], params["critic"], train_state["target"]
        x, a, r, x_next, terminated = (
            batch[k] for k in ("x", "a", "reward", "x_next", "terminated")
        )
        k_next, k_actor = jax.random.split(key)
        keys_next = jax.random.split(k_next, x.shape[0])
        keys_actor = jax.random.split(k_actor, x.shape[0])
        alpha = jnp.exp(train_state["log_alpha"])

        def twin_q(w, x, a):
            return Q.value(w["q1"], x, a), Q.value(w["q2"], x, a)

        # Soft Bellman target: y = r + gamma (1 - terminated) [min_i Q_i'(x', a') - alpha ln pi(a'|x')], a' ~ pi(.|x')
        a_next, logp_next = pi.sample_and_log_prob(theta, x_next, keys_next)
        q1_target, q2_target = twin_q(w_target, x_next, a_next)
        y = r + self.gamma * (1.0 - terminated) * (
            jnp.minimum(q1_target, q2_target) - alpha * logp_next
        )
        y = jax.lax.stop_gradient(y)

        # Critic loss: L_Q = E[(Q_1(x, a) - y)^2] + E[(Q_2(x, a) - y)^2]
        def critic_loss(w):
            q1, q2 = twin_q(w, x, a)
            return jnp.mean((q1 - y) ** 2) + jnp.mean((q2 - y) ** 2)

        # Actor loss, reparameterized: L_pi = E[alpha ln pi(a|x) - min_i Q_i(x, a)], a ~ pi(.|x)
        def actor_loss(theta):
            a_new, logp = pi.sample_and_log_prob(theta, x, keys_actor)
            q1, q2 = twin_q(w, x, a_new)
            return jnp.mean(alpha * logp - jnp.minimum(q1, q2)), logp

        # Temperature loss, holding the entropy at its target: L_alpha = -ln alpha E[ln pi + H_target]
        def alpha_loss(log_alpha, logp):
            return -log_alpha * jnp.mean(logp + self.target_entropy)

        # Gradients of the three losses at the current weights, then one Adam step each
        critic_grads = jax.grad(critic_loss)(w)
        (loss_pi, logp), actor_grads = jax.value_and_grad(actor_loss, has_aux=True)(
            theta
        )
        alpha_grad = jax.grad(alpha_loss)(train_state["log_alpha"], logp)
        w, opt_critic = self.optimizer.update(critic_grads, opt["critic"], w)
        theta, opt_actor = self.optimizer.update(actor_grads, opt["actor"], theta)
        log_alpha, opt_alpha = self.optimizer.update(
            alpha_grad, opt["alpha"], train_state["log_alpha"]
        )

        # Target critics, Polyak-averaged: Q' <- (1 - tau) Q' + tau Q
        w_target = jax.tree_util.tree_map(
            lambda q_target, q: (1.0 - self.tau) * q_target + self.tau * q, w_target, w
        )

        new_state = {
            "params": {"policy": theta, "critic": w},
            "target": w_target,
            "log_alpha": log_alpha,
            "opt": {"actor": opt_actor, "critic": opt_critic, "alpha": opt_alpha},
        }
        stats = {
            "critic_loss": critic_loss(w),
            "actor_loss": loss_pi,
            "alpha": alpha,
            "entropy": -jnp.mean(logp),
        }
        return new_state, stats
