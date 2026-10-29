"""Soft Actor-Critic: the off-policy update on replayed transitions."""

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.learning.reinforcement_learning.algorithms.base import Algorithm
from minilink.optimization.optimizers.first_order import Adam

###############################################################################
# Public API
###############################################################################


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
    head_kind = "squashed"
    critic_kind = "Q"

    NUMBERS = ("gamma", "tau")

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
        # Hyperparameters
        self.gamma = None if gamma is None else float(gamma)
        self.tau = float(tau)

        # Replay: the minibatch, the capacity, the warm-up and the updates per step
        self.batch_size = int(batch_size)
        self.buffer_size = int(buffer_size)
        self.learning_starts = int(learning_starts)
        self.gradient_steps = gradient_steps

        # Temperature: the entropy target as given (None: −m of the bound policy, at
        # each update)
        self.target_entropy = target_entropy

        # Optimizer: one Adam for the actor, the critics and the temperature
        self.optimizer = Adam(learning_rate, max_grad_norm=max_grad_norm)

    def init(self, key):
        """Train state: weights, twin critics with their targets, log-temperature, one optimizer state each."""
        jnp = require_jax_numpy()

        pi = self.policy
        Q = self.critic
        optimizer = self.optimizer

        # The actor, the twin critics (independent draws) and their targets
        theta = pi.init()
        w = {"q1": Q.init(), "q2": Q.init(Q.seed + 1)}

        # The temperature starts at α = 1 (ln α = 0); each part has its own optimizer state
        log_alpha = jnp.asarray(0.0)
        opt = {
            "actor": optimizer.init(theta),
            "critic": optimizer.init(w),
            "alpha": optimizer.init(jnp.asarray(0.0)),
        }

        return {
            "params": {"policy": theta, "critic": w},
            "target": w,
            "log_alpha": log_alpha,
            "opt": opt,
        }

    def update(self, train_state, batch, key):
        """One SAC step on a replay minibatch ``{x, a, reward, x_next, terminated}``."""
        jax = require_jax()
        jnp = require_jax_numpy()

        pi = self.policy
        Q = self.critic
        optimizer = self.optimizer
        gamma = self.gamma
        tau = self.tau
        H_target = target_entropy_of(self.target_entropy, pi)
        params = train_state["params"]
        opt = train_state["opt"]
        theta = params["policy"]
        w = params["critic"]
        w_target = train_state["target"]

        x = batch["x"]
        a = batch["a"]
        r = batch["reward"]
        x_next = batch["x_next"]
        terminated = batch["terminated"]

        # One noise key per sample, for the next actions and for the actor's actions
        k_next, k_actor = jax.random.split(key)
        keys_next = jax.random.split(k_next, x.shape[0])
        keys_actor = jax.random.split(k_actor, x.shape[0])

        # α = exp(ln α), the current temperature
        log_alpha = train_state["log_alpha"]
        alpha = jnp.exp(log_alpha)

        def twin_q(w, x, a):
            return Q.value(w["q1"], x, a), Q.value(w["q2"], x, a)

        # y = r + γ (1 − terminated) [min_i Q'_i(x', a') − α ln π(a'|x')], a' ~ π(·|x'),
        # the soft Bellman target, held fixed
        a_next, logp_next = pi.sample_and_log_prob(theta, x_next, keys_next)
        q1_target, q2_target = twin_q(w_target, x_next, a_next)
        y = r + gamma * (1.0 - terminated) * (
            jnp.minimum(q1_target, q2_target) - alpha * logp_next
        )
        y = jax.lax.stop_gradient(y)

        # L_Q = E[(Q_1(x, a) − y)²] + E[(Q_2(x, a) − y)²]
        def critic_loss(w):
            q1, q2 = twin_q(w, x, a)
            L_Q = jnp.mean((q1 - y) ** 2) + jnp.mean((q2 - y) ** 2)
            return L_Q

        # L_π = E[α ln π(a|x) − min_i Q_i(x, a)], a ~ π(·|x) reparameterized
        def actor_loss(theta):
            a_new, logp = pi.sample_and_log_prob(theta, x, keys_actor)
            q1, q2 = twin_q(w, x, a_new)
            L_pi = jnp.mean(alpha * logp - jnp.minimum(q1, q2))
            return L_pi, logp

        # L_α = −ln α E[ln π + H_target], holding the entropy at its target
        def alpha_loss(log_alpha, logp):
            L_alpha = -log_alpha * jnp.mean(logp + H_target)
            return L_alpha

        # Gradients of the three losses at the current weights, then one Adam step each
        critic_grads = jax.grad(critic_loss)(w)
        (L_pi, logp), actor_grads = jax.value_and_grad(actor_loss, has_aux=True)(theta)
        alpha_grad = jax.grad(alpha_loss)(log_alpha, logp)
        w, opt_critic = optimizer.update(critic_grads, opt["critic"], w)
        theta, opt_actor = optimizer.update(actor_grads, opt["actor"], theta)
        log_alpha, opt_alpha = optimizer.update(alpha_grad, opt["alpha"], log_alpha)

        # Q' ← (1 − τ) Q' + τ Q, the Polyak-averaged target critics
        w_target = jax.tree_util.tree_map(
            lambda q_target, q: (1.0 - tau) * q_target + tau * q, w_target, w
        )

        new_state = {
            "params": {"policy": theta, "critic": w},
            "target": w_target,
            "log_alpha": log_alpha,
            "opt": {"actor": opt_actor, "critic": opt_critic, "alpha": opt_alpha},
        }

        # Monitoring: the critic loss at the new weights, and the entropy −E[ln π]
        L_Q = critic_loss(w)
        entropy = -jnp.mean(logp)
        stats = {
            "critic_loss": L_Q,
            "actor_loss": L_pi,
            "alpha": alpha,
            "entropy": entropy,
        }

        return new_state, stats


###############################################################################
# Internal machinery
###############################################################################


def target_entropy_of(target_entropy, pi) -> float:
    """The entropy target as given, else −m, minus the bound policy's action dimension."""
    if target_entropy is None:
        return -float(pi.m)

    return target_entropy
