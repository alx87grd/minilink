"""Experience collectors: an on-policy rollout scan, and a replay buffer for off-policy methods."""

import numpy as np

from minilink.core.backends import require_jax, require_jax_numpy

# Public API


def rollout(env, functions, params, carry, key, *, n_steps, n_envs, gamma):
    """
    Collect ``n_steps`` on ``n_envs`` plants; return ``(batch, carry, last_value)``.

    One ``lax.scan`` samples the stochastic policy inside the loop and returns
    what on-policy methods consume: states, normalized actions, log-densities,
    values, rewards and episode ends. ``carry`` is ``(x, t, ep_return, params)``
    with a leading ``n_envs`` axis; ``functions`` exposes ``mean(actor, x)``,
    ``value(critic, x)`` and ``head``.
    """
    jax, jnp = require_jax(), require_jax_numpy()
    head = functions.head

    def env_step(carry, key):
        x, t, ep_return, theta = carry
        k_sample, k_step, k_reset, k_theta = jax.random.split(key, 4)

        # Policy: a ~ pi(.|x) around the mean action, its log-density, and V(x)
        mu = functions.mean(params["actor"], x)
        a = head.sample(params["head"], mu, k_sample)
        logp = head.log_prob(params["head"], mu, a)
        v = functions.value(params["critic"], x)

        # Plant: the normalized action mapped onto the port bounds, one control period
        u = env.u_mid + env.u_half * jnp.clip(a, -1.0, 1.0)
        x_next, t_next, reward, terminated, truncated = env.step(
            x, t, u, k_step, params=theta
        )
        done = terminated | truncated
        ep_return_next = ep_return + reward  # logged return: raw rewards only

        # A truncated episode bootstraps its value: r + gamma V(x')
        reward = reward + truncated * gamma * functions.value(params["critic"], x_next)

        sample = {
            "x": x,
            "a": a,
            "logp": logp,
            "value": v,
            "reward": reward,
            "done": done.astype(float),
            "ep_return": jnp.where(done, ep_return_next, jnp.nan),
        }
        carry = restart_finished_episodes(
            env, x_next, t_next, ep_return_next, theta, done, k_reset, k_theta
        )
        return carry, sample

    def scan_step(carry, key):
        keys = jax.random.split(key, n_envs)
        return jax.vmap(env_step)(carry, keys)

    keys = jax.random.split(key, n_steps)
    carry, batch = jax.lax.scan(scan_step, carry, keys)
    last_value = jax.vmap(functions.value, in_axes=(None, 0))(
        params["critic"], carry[0]
    )
    return batch, carry, last_value


def collect_transitions(env, functions, params, carry, key, *, n_steps, n_envs):
    """
    Step ``n_envs`` plants ``n_steps`` times with the stochastic policy; return
    ``(transitions, carry)`` with the flat transition batch off-policy methods
    store: ``x, a, reward, x_next, terminated`` (plus ``ep_return`` for logs).
    Truncation is not terminal: the replay keeps the true next state.
    """
    jax, jnp = require_jax(), require_jax_numpy()
    head = functions.head

    def env_step(carry, key):
        x, t, ep_return, theta = carry
        k_sample, k_step, k_reset, k_theta = jax.random.split(key, 4)

        # Policy: a ~ pi(.|x), then one control period of the plant
        mu = functions.mean(params["actor"], x)
        a = head.sample(params["head"], mu, k_sample, z=functions.observe(x))
        u = env.u_mid + env.u_half * jnp.clip(a, -1.0, 1.0)
        x_next, t_next, reward, terminated, truncated = env.step(
            x, t, u, k_step, params=theta
        )
        done = terminated | truncated
        ep_return_next = ep_return + reward

        # The transition keeps the true next state, even where the episode was cut
        transition = {
            "x": x,
            "a": a,
            "reward": reward,
            "x_next": x_next,
            "terminated": terminated.astype(float),
            "ep_return": jnp.where(done, ep_return_next, jnp.nan),
        }

        carry = restart_finished_episodes(
            env, x_next, t_next, ep_return_next, theta, done, k_reset, k_theta
        )
        return carry, transition

    def scan_step(carry, key):
        keys = jax.random.split(key, n_envs)
        return jax.vmap(env_step)(carry, keys)

    keys = jax.random.split(key, n_steps)
    carry, batch = jax.lax.scan(scan_step, carry, keys)
    flat = {k: v.reshape((n_steps * n_envs,) + v.shape[2:]) for k, v in batch.items()}
    return flat, carry


def gae(batch, last_value, gamma, lam):
    """Generalized advantage estimation, backwards in time, per plant."""
    jax = require_jax()

    def step(carry, sample):
        advantage_next, value_next = carry
        not_done = 1.0 - sample["done"]

        # TD error: delta_k = r_k + gamma V(x_k+1) - V(x_k), no bootstrap past an episode's end
        delta = sample["reward"] + gamma * value_next * not_done - sample["value"]

        # Advantage: A_k = delta_k + gamma lambda A_k+1
        advantage = delta + gamma * lam * not_done * advantage_next
        return (advantage, sample["value"]), advantage

    zeros = last_value * 0.0
    sequence = {k: batch[k] for k in ("reward", "value", "done")}
    _, advantage = jax.lax.scan(step, (zeros, last_value), sequence, reverse=True)

    # Returns: R_k = A_k + V(x_k)
    return advantage, advantage + batch["value"]


def reset_carry(env, key, n_envs):
    """Initial ``(x, t, ep_return, params)`` carry for ``n_envs`` plants."""
    jax, jnp = require_jax(), require_jax_numpy()
    k_x, k_theta = jax.random.split(key)
    x = jax.vmap(env.reset)(jax.random.split(k_x, n_envs))
    theta = jax.vmap(env.sample_params)(jax.random.split(k_theta, n_envs))
    return x, jnp.zeros(n_envs), jnp.zeros(n_envs), theta


def restart_finished_episodes(
    env, x_next, t_next, ep_return, theta, done, k_reset, k_theta
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
        fresh = env.sample_params(k_theta)
        theta = jax.tree_util.tree_map(
            lambda old, new: jnp.where(done, new, old), theta, fresh
        )
    return x_next, t_next, ep_return, theta


class ReplayBuffer:
    """
    Fixed-capacity transition store on device for off-policy methods.

    ``add(batch)`` writes a ``(k, ...)`` batch of transitions at the cursor
    (wrapping around); ``sample(key, n)`` draws uniformly among the stored
    ones. Fields: ``x, a, reward, x_next, terminated``.
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
