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

    on_policy = False
    squash = "tanh"

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

    def init(self, key, params):
        """Train state: weights, target critics, log-temperature, and one optimizer state each."""
        jax, jnp = require_jax(), require_jax_numpy()
        if self.target_entropy is None:
            self.target_entropy = -float(self.functions.m)
        actor = {"actor": params["actor"], "head": params["head"]}
        critic = params["critic"]
        return {
            "params": params,
            "target": jax.tree_util.tree_map(lambda w: w, critic),
            "log_alpha": jnp.asarray(0.0),
            "opt": {
                "actor": self.optimizer.init(actor),
                "critic": self.optimizer.init(critic),
                "alpha": self.optimizer.init(jnp.asarray(0.0)),
            },
        }

    def update(self, train_state, batch, key):
        """One SAC step on a replay minibatch ``{x, a, reward, x_next, terminated}``."""
        jax, jnp = require_jax(), require_jax_numpy()
        functions = self.functions
        params = train_state["params"]
        x, a, r, x_next, done = (
            batch[k] for k in ("x", "a", "reward", "x_next", "terminated")
        )
        k_next, k_actor = jax.random.split(key)
        alpha = jnp.exp(train_state["log_alpha"])
        batch_size = x.shape[0]

        def policy_sample(actor, head, xs, keys):
            mu = jax.vmap(functions.mean, in_axes=(None, 0))(actor, xs)
            z = jax.vmap(functions.observe)(xs)
            return jax.vmap(
                functions.head.sample_and_log_prob, in_axes=(None, 0, 0, 0)
            )(head, mu, z, keys)

        def q_values(critic, xs, acts):
            q = jax.vmap(functions.q, in_axes=(None, 0, 0))
            return q(critic["q1"], xs, acts), q(critic["q2"], xs, acts)

        # Soft Bellman target: y = r + gamma (1 - done) [min_i Q_i'(x', a') - alpha log pi(a'|x')]
        a_next, logp_next = policy_sample(
            params["actor"],
            params["head"],
            x_next,
            jax.random.split(k_next, batch_size),
        )
        q1_t, q2_t = q_values(train_state["target"], x_next, a_next)
        y = r + self.gamma * (1.0 - done) * (
            jnp.minimum(q1_t, q2_t) - alpha * logp_next
        )
        y = jax.lax.stop_gradient(y)

        # Critic loss: L_Q = E[(Q_1(x, a) - y)^2] + E[(Q_2(x, a) - y)^2]
        def critic_loss(critic):
            q1, q2 = q_values(critic, x, a)
            return jnp.mean((q1 - y) ** 2) + jnp.mean((q2 - y) ** 2)

        # Actor loss, reparameterized: L_pi = E[alpha log pi(a|x) - min_i Q_i(x, a)]
        def actor_loss(actor_params):
            a_new, logp = policy_sample(
                actor_params["actor"],
                actor_params["head"],
                x,
                jax.random.split(k_actor, batch_size),
            )
            q1, q2 = q_values(params["critic"], x, a_new)
            return jnp.mean(alpha * logp - jnp.minimum(q1, q2)), logp

        # Temperature loss, holding the entropy at its target: L_alpha = -log alpha E[log pi + H_target]
        def alpha_loss(log_alpha, logp):
            return -log_alpha * jnp.mean(logp + self.target_entropy)

        opt = train_state["opt"]
        critic_grads = jax.grad(critic_loss)(params["critic"])
        critic, opt_critic = self.optimizer.update(
            critic_grads, opt["critic"], params["critic"]
        )

        actor_params = {"actor": params["actor"], "head": params["head"]}
        (loss_pi, logp), actor_grads = jax.value_and_grad(actor_loss, has_aux=True)(
            actor_params
        )
        actor_params, opt_actor = self.optimizer.update(
            actor_grads, opt["actor"], actor_params
        )

        alpha_grad = jax.grad(alpha_loss)(train_state["log_alpha"], logp)
        log_alpha, opt_alpha = self.optimizer.update(
            alpha_grad, opt["alpha"], train_state["log_alpha"]
        )

        # Target critics, Polyak-averaged: Q' = (1 - tau) Q' + tau Q
        target = jax.tree_util.tree_map(
            lambda q_target, q: (1.0 - self.tau) * q_target + self.tau * q,
            train_state["target"],
            critic,
        )
        new_state = {
            "params": {
                "actor": actor_params["actor"],
                "head": actor_params["head"],
                "critic": critic,
            },
            "target": target,
            "log_alpha": log_alpha,
            "opt": {"actor": opt_actor, "critic": opt_critic, "alpha": opt_alpha},
        }
        stats = {
            "critic_loss": critic_loss(critic),
            "actor_loss": loss_pi,
            "alpha": alpha,
            "entropy": -jnp.mean(logp),
        }
        return new_state, stats
