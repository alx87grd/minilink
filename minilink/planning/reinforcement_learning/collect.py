"""Experience: one rollout scan for both families, advantage estimation, and a replay buffer."""

import numpy as np

from minilink.core.backends import require_jax, require_jax_numpy

# Public API


def rollout(env, policy, theta, carry, key, *, n_steps, n_envs):
    """
    Step ``n_envs`` plants ``n_steps`` times under ``pi_theta``; return ``(batch, carry)``.

    ``batch`` holds one transition per step and plant, arrays of shape
    ``(n_steps, n_envs, ...)``: the state ``x``, the normalized action ``a``,
    the ``reward``, the next state ``x_next``, and how the step ended,
    ``terminated`` (the episode's cost fully counted) or ``done`` (terminated
    or truncated, the episode restarts). ``ep_return`` is the return of an
    episode that ended at that step (NaN elsewhere), for the logs. ``carry``
    is ``(x, t, ep_return, params)`` with a leading ``n_envs`` axis.
    """
    jax, jnp = require_jax(), require_jax_numpy()

    def env_step(carry, key):
        x, t, ep_return, params = carry
        k_sample, k_step, k_reset, k_params = jax.random.split(key, 4)

        # Policy: a ~ pi_theta(. | x); plant: one control period under u = u_mid + u_half clip(a)
        a = policy.sample(theta, x, k_sample)
        x_next, t_next, reward, terminated, truncated = env.step(
            x, t, policy.input(a), k_step, params
        )
        done = terminated | truncated
        ep_return = ep_return + reward

        transition = {
            "x": x,
            "a": a,
            "reward": reward,
            "x_next": x_next,
            "terminated": terminated.astype(float),
            "done": done.astype(float),
            "ep_return": jnp.where(done, ep_return, jnp.nan),
        }
        carry = restart_finished_episodes(
            env, x_next, t_next, ep_return, params, done, k_reset, k_params
        )
        return carry, transition

    def scan_step(carry, key):
        return jax.vmap(env_step)(carry, jax.random.split(key, n_envs))

    carry, batch = jax.lax.scan(scan_step, carry, jax.random.split(key, n_steps))
    return batch, carry


def advantages(critic, w, batch, gamma, lam):
    """Advantages and returns of an on-policy batch: the critic's TD errors, then generalized advantage estimation."""
    # TD error: delta_k = r_k + gamma V(x_k+1) - V(x_k), no value beyond a terminal state
    value = critic.value(w, batch["x"])
    value_next = (1.0 - batch["terminated"]) * critic.value(w, batch["x_next"])
    delta = batch["reward"] + gamma * value_next - value

    # Advantage A_k, and the return R_k = A_k + V(x_k) the critic regresses onto
    advantage = gae(delta, batch["done"], gamma, lam)
    return advantage, advantage + value


def gae(delta, done, gamma, lam):
    """Generalized advantage estimation ``A_k = delta_k + gamma lambda (1 - done_k) A_k+1``, backwards in time."""
    return discounted_sum(delta, done, gamma * lam)


def returns_to_go(reward, done, gamma):
    """Discounted return to the end of the episode, ``R_k = r_k + gamma (1 - done_k) R_k+1``."""
    return discounted_sum(reward, done, gamma)


def flatten(batch):
    """Merge the time and plant axes of a batch into one axis of samples."""
    return {
        key: value.reshape((value.shape[0] * value.shape[1],) + value.shape[2:])
        for key, value in batch.items()
    }


class ReplayBuffer:
    """
    Fixed-capacity transition store on device for off-policy methods.

    ``add(batch)`` writes a batch of transitions (leading axis) at the cursor,
    wrapping around; ``sample(key, n)`` draws uniformly among the stored ones.
    Fields: ``x, a, reward, x_next, terminated``.
    """

    def __init__(self, capacity, n, m):
        jnp = require_jax_numpy()
        self.capacity = int(capacity)
        self.data = {
            "x": jnp.zeros((self.capacity, n)),
            "a": jnp.zeros((self.capacity, m)),
            "reward": jnp.zeros(self.capacity),
            "x_next": jnp.zeros((self.capacity, n)),
            "terminated": jnp.zeros(self.capacity),
        }
        self.cursor = 0
        self.size = 0

    def add(self, batch):
        """Write a batch of transitions at the cursor, overwriting the oldest when full."""
        k = int(batch["x"].shape[0])
        idx = (self.cursor + np.arange(k)) % self.capacity
        self.data = {key: self.data[key].at[idx].set(batch[key]) for key in self.data}
        self.cursor = (self.cursor + k) % self.capacity
        self.size = min(self.size + k, self.capacity)

    def sample(self, key, n):
        """Draw ``n`` stored transitions uniformly."""
        jax = require_jax()
        idx = jax.random.randint(key, (int(n),), 0, self.size)
        return {key_: value[idx] for key_, value in self.data.items()}


# Internal machinery


def discounted_sum(increment, done, factor):
    """Backward recursion ``S_k = y_k + factor (1 - done_k) S_k+1`` along the time axis, zero past the end."""
    jax, jnp = require_jax(), require_jax_numpy()

    def step(s_next, inputs):
        y, d = inputs
        s = y + factor * (1.0 - d) * s_next
        return s, s

    _, s = jax.lax.scan(
        step, jnp.zeros_like(increment[0]), (increment, done), reverse=True
    )
    return s


def reset_carry(env, key, n_envs):
    """Initial ``(x, t, ep_return, params)`` carry for ``n_envs`` plants."""
    jax, jnp = require_jax(), require_jax_numpy()
    k_x, k_params = jax.random.split(key)
    x = jax.vmap(env.reset)(jax.random.split(k_x, n_envs))
    params = jax.vmap(env.sample_params)(jax.random.split(k_params, n_envs))
    return x, jnp.zeros(n_envs), jnp.zeros(n_envs), params


def restart_finished_episodes(
    env, x_next, t_next, ep_return, params, done, k_reset, k_params
):
    """
    The carry of the next control period, per plant.

    A plant whose episode ended restarts from a fresh start, at time zero, with
    a zero return and, when the problem randomizes them, fresh parameters; the
    others continue.
    """
    jax, jnp = require_jax(), require_jax_numpy()
    x_next = jnp.where(done, env.reset(k_reset), x_next)
    t_next = jnp.where(done, 0.0, t_next)
    ep_return = jnp.where(done, 0.0, ep_return)
    if env.randomizes_params:
        fresh = env.sample_params(k_params)
        params = jax.tree_util.tree_map(
            lambda old, new: jnp.where(done, new, old), params, fresh
        )
    return x_next, t_next, ep_return, params
