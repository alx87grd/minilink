"""REINFORCE: the Monte Carlo policy gradient, from complete episodes and nothing else."""

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.planning.reinforcement_learning.algorithms.base import Algorithm
from minilink.planning.reinforcement_learning.collect import returns_to_go
from minilink.planning.reinforcement_learning.optim import Adam

# Public API


class REINFORCE(Algorithm):
    """
    Monte Carlo policy gradient.

    The policy gradient theorem, ``grad J = E[grad ln pi_theta(a|x) Q^pi(x, a)]``,
    with the discounted return observed to the end of the episode standing in
    for ``Q^pi``: no critic, no bootstrap, unbiased and noisy. The method
    learns from complete episodes, so the planner restarts every plant before
    a collection and runs it for one episode length. ``baseline`` subtracts the
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

    on_policy, head_kind, critic_kind, episodic = True, "gaussian", None, True

    def __init__(
        self,
        *,
        learning_rate=1e-3,
        gamma=None,
        baseline=True,
        max_grad_norm=0.5,
        optimizer=None,
    ):
        self.gamma = None if gamma is None else float(gamma)
        self.baseline = bool(baseline)
        self.optimizer = (
            Adam(learning_rate, max_grad_norm=max_grad_norm)
            if optimizer is None
            else optimizer
        )

    def init(self, key):
        """Train state: the policy weights and the optimizer's moments."""
        params = {"policy": self.policy.init()}
        return {"params": params, "opt": self.optimizer.init(params)}

    def loss(self, params, batch, weight, complete):
        """Minus the return-weighted log-likelihood of the actions of the complete episodes."""
        jnp = require_jax_numpy()

        # Surrogate E[(R_k - b) ln pi_theta(a_k | x_k)]: its gradient is the policy gradient estimate
        logp = self.policy.log_prob(params["policy"], batch["x"], batch["a"])
        return -jnp.sum(complete * weight * logp) / jnp.sum(complete), {}

    def update(self, train_state, batch, key):
        """Returns of the episodes, then one gradient step on the whole batch."""
        jax, jnp = require_jax(), require_jax_numpy()
        params, opt_state = train_state["params"], train_state["opt"]
        done = batch["done"]

        # Monte Carlo return to the end of each episode: R_k = sum_j>=k gamma^(j-k) r_j
        returns = returns_to_go(batch["reward"], done, self.gamma)

        # Only the first episode of each plant is complete; a restarted one is cut by the batch's end
        complete = ((jnp.cumsum(done, axis=0) - done) == 0).astype(float)

        # Baseline b: the mean return of the batch
        b = jnp.sum(complete * returns) / jnp.sum(complete) if self.baseline else 0.0

        (_, _), grads = jax.value_and_grad(self.loss, has_aux=True)(
            params, batch, returns - b, complete
        )
        params, opt_state = self.optimizer.update(grads, opt_state, params)
        stats = {
            "return_mean": jnp.sum(complete * returns) / jnp.sum(complete),
            "episode_steps": jnp.mean(jnp.sum(complete, axis=0)),
        }
        return {"params": params, "opt": opt_state}, stats
