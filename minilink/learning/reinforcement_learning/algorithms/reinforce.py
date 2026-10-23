"""REINFORCE: the Monte Carlo policy gradient, from complete episodes and nothing else."""

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.learning.reinforcement_learning.algorithms.base import (
    Algorithm,
    optimizer_of,
)
from minilink.learning.reinforcement_learning.collect import returns_to_go

###############################################################################
# Public API
###############################################################################


class REINFORCE(Algorithm):
    """
    Monte Carlo policy gradient.

    The policy gradient theorem, ``∇J = E[∇ ln π_θ(a|x) Q^π(x, a)]``, with the
    discounted return observed to the end of the episode standing in for
    ``Q^π``: no critic, no bootstrap, unbiased and noisy. The method learns
    from complete episodes, so the planner restarts every plant before a
    collection and runs it for one episode length. ``baseline`` subtracts the
    mean return of the batch, the one variance reduction that needs no critic.

    Parameters
    ----------
    learning_rate, gamma, max_grad_norm
        Step size, discount (``None``: resolved by the planner) and gradient clip.
    baseline : bool
        Subtract the batch's mean return from every return.
    optimizer : object, optional
        Optax-style ``init`` / ``update``; default the built-in :class:`Adam`.
    """

    on_policy = True
    head_kind = "gaussian"
    critic_kind = None
    episodic = True

    def __init__(
        self,
        *,
        learning_rate=1e-3,
        gamma=None,
        baseline=True,
        max_grad_norm=0.5,
        optimizer=None,
    ):
        # Hyperparameters
        self.gamma = None if gamma is None else float(gamma)
        self.baseline = bool(baseline)

        # Optimizer: the one given, or the built-in Adam
        self.optimizer = optimizer_of(optimizer, learning_rate, max_grad_norm)

    def init(self, key):
        """Train state: the policy weights and the optimizer's moments."""
        params = {"policy": self.policy.init()}
        opt_state = self.optimizer.init(params)

        return {"params": params, "opt": opt_state}

    def loss(self, params, batch, weight, complete):
        """Minus the return-weighted log-likelihood of the actions of the complete episodes."""
        jnp = require_jax_numpy()

        pi = self.policy
        theta = params["policy"]

        x = batch["x"]
        a = batch["a"]

        # L_π = −E[(R_k − b) ln π_θ(a_k|x_k)] over the complete episodes: its gradient
        # is the policy gradient estimate
        logp = pi.log_prob(theta, x, a)
        L_pi = -jnp.sum(complete * weight * logp) / jnp.sum(complete)

        return L_pi, {}

    def update(self, train_state, batch, key):
        """Returns of the episodes, then one gradient step on the whole batch."""
        jax = require_jax()
        jnp = require_jax_numpy()

        gamma = self.gamma
        baseline = self.baseline
        optimizer = self.optimizer
        params = train_state["params"]
        opt_state = train_state["opt"]

        done = batch["done"]
        reward = batch["reward"]

        # R_k = Σ_{j≥k} γ^{j−k} r_j, the Monte Carlo return to the end of each episode
        returns = returns_to_go(reward, done, gamma)

        # Only the first episode of each plant is complete; a restarted one is cut by the batch's end
        complete = ((jnp.cumsum(done, axis=0) - done) == 0).astype(float)

        # Baseline b: the mean return of the batch
        b = jnp.sum(complete * returns) / jnp.sum(complete) if baseline else 0.0

        # One gradient step on the surrogate, each step weighted by R_k − b
        grad_fn = jax.value_and_grad(self.loss, has_aux=True)
        (_, _), grads = grad_fn(params, batch, returns - b, complete)
        params, opt_state = optimizer.update(grads, opt_state, params)

        # Monitoring: the mean return and the length of the complete episodes
        return_mean = jnp.sum(complete * returns) / jnp.sum(complete)
        episode_steps = jnp.mean(jnp.sum(complete, axis=0))
        stats = {
            "return_mean": return_mean,
            "episode_steps": episode_steps,
        }

        return {"params": params, "opt": opt_state}, stats
