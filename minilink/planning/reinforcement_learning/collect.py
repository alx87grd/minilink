"""
Experience collectors: an on-policy rollout scan, and a replay buffer for off-policy methods.

``rollout`` simulates ``n_envs`` plants for ``n_steps`` control periods in one
``lax.scan`` with the stochastic policy sampled inside the loop, and returns
the batch on-policy methods consume (states, normalized actions, log-probs,
values, rewards, episode ends). ``gae`` turns it into advantages and returns.
``ReplayBuffer`` stores transitions for off-policy methods.
"""

import numpy as np

from minilink.core.backends import require_jax_numpy

# Public API


def rollout(env, functions, params, carry, key, *, n_steps, n_envs, gamma):
    """
    Collect ``n_steps`` on ``n_envs`` plants; return ``(batch, carry, last_value)``.

    ``carry`` is ``(x, t, ep_return)`` with a leading ``n_envs`` axis;
    ``functions`` exposes ``mean(actor, x)``, ``value(critic, x)`` and ``head``.
    Truncated episodes bootstrap their reward with ``gamma * V(x_next)``.
    """
    jnp = require_jax_numpy()
    import jax

    head = functions.head

    def env_step(carry, key):
        x, t, ep_return, theta = carry
        k_sample, k_step, k_reset, k_theta = jax.random.split(key, 4)

        mu = functions.mean(params["actor"], x)
        a = head.sample(params["head"], mu, k_sample)
        logp = head.log_prob(params["head"], mu, a)
        v = functions.value(params["critic"], x)

        u = env.u_mid + env.u_half * jnp.clip(a, -1.0, 1.0)
        x_next, t_next, reward, terminated, truncated = env.step(
            x, t, u, k_step, params=theta
        )
        done = terminated | truncated
        ep_return_next = ep_return + reward  # logged return: raw rewards only
        reward = reward + truncated * gamma * functions.value(params["critic"], x_next)

        x_next = jnp.where(done, env.reset(k_reset), x_next)
        t_next = jnp.where(done, 0.0, t_next)
        theta = new_episode_params(env, theta, done, k_theta)
        sample = {
            "x": x,
            "a": a,
            "logp": logp,
            "value": v,
            "reward": reward,
            "done": done.astype(float),
            "ep_return": jnp.where(done, ep_return_next, jnp.nan),
        }
        ep_return_next = jnp.where(done, 0.0, ep_return_next)
        return (x_next, t_next, ep_return_next, theta), sample

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
    jnp = require_jax_numpy()
    import jax

    head = functions.head

    def env_step(carry, key):
        x, t, ep_return, theta = carry
        k_sample, k_step, k_reset, k_theta = jax.random.split(key, 4)
        mu = functions.mean(params["actor"], x)
        a = head.sample(params["head"], mu, k_sample, z=functions.observe(x))
        u = env.u_mid + env.u_half * jnp.clip(a, -1.0, 1.0)
        x_next, t_next, reward, terminated, truncated = env.step(
            x, t, u, k_step, params=theta
        )
        done = terminated | truncated
        ep_return_next = ep_return + reward
        transition = {
            "x": x,
            "a": a,
            "reward": reward,
            "x_next": x_next,
            "terminated": terminated.astype(float),
            "ep_return": jnp.where(done, ep_return_next, jnp.nan),
        }
        x_next = jnp.where(done, env.reset(k_reset), x_next)
        t_next = jnp.where(done, 0.0, t_next)
        theta = new_episode_params(env, theta, done, k_theta)
        ep_return_next = jnp.where(done, 0.0, ep_return_next)
        return (x_next, t_next, ep_return_next, theta), transition

    def scan_step(carry, key):
        keys = jax.random.split(key, n_envs)
        return jax.vmap(env_step)(carry, keys)

    keys = jax.random.split(key, n_steps)
    carry, batch = jax.lax.scan(scan_step, carry, keys)
    flat = {k: v.reshape((n_steps * n_envs,) + v.shape[2:]) for k, v in batch.items()}
    return flat, carry


def gae(batch, last_value, gamma, lam):
    """Generalized advantage estimation, backwards in time, per plant."""
    import jax

    def step(carry, sample):
        adv, next_value = carry
        not_done = 1.0 - sample["done"]
        delta = sample["reward"] + gamma * next_value * not_done - sample["value"]
        adv = delta + gamma * lam * not_done * adv
        return (adv, sample["value"]), adv

    zeros = last_value * 0.0
    sequence = {k: batch[k] for k in ("reward", "value", "done")}
    _, advantage = jax.lax.scan(step, (zeros, last_value), sequence, reverse=True)
    return advantage, advantage + batch["value"]


def reset_carry(env, key, n_envs):
    """Initial ``(x, t, ep_return, params)`` carry for ``n_envs`` plants."""
    jnp = require_jax_numpy()
    import jax

    k_x, k_theta = jax.random.split(key)
    x = jax.vmap(env.reset)(jax.random.split(k_x, n_envs))
    theta = jax.vmap(env.sample_params)(jax.random.split(k_theta, n_envs))
    return x, jnp.zeros(n_envs), jnp.zeros(n_envs), theta


def new_episode_params(env, theta, done, key):
    """Keep the episode's plant parameters, or draw new ones when it ended."""
    jnp = require_jax_numpy()
    import jax

    if not env.randomizes_params:
        return theta
    fresh = env.sample_params(key)
    return jax.tree_util.tree_map(
        lambda old, new: jnp.where(done, new, old), theta, fresh
    )


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
        k = int(batch["x"].shape[0])
        idx = (self.cursor + np.arange(k)) % self.capacity
        self.data = {key: self.data[key].at[idx].set(batch[key]) for key in self.data}
        self.cursor = (self.cursor + k) % self.capacity
        self.size = min(self.size + k, self.capacity)

    def sample(self, key, n):
        import jax

        idx = jax.random.randint(key, (int(n),), 0, self.size)
        return {key_: value[idx] for key_, value in self.data.items()}
