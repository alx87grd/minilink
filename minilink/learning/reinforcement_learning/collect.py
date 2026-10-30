"""Experience: one rollout scan for both families, advantage estimation, and a replay buffer."""

import numpy as np

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.core.distributions import CounterKey, sample_index
from minilink.planning.environment import MAX_SEED

###############################################################################
# Public API
###############################################################################


def rollout(env, policy, theta, carry, key, *, n_steps, n_envs):
    """
    Step ``n_envs`` plants ``n_steps`` times under ``π_θ``; return ``(batch, carry)``.

    ``batch`` holds one transition per step and plant, arrays of shape
    ``(n_steps, n_envs, ...)``: the state ``x``, the normalized action ``a``,
    the ``reward``, the next state ``x_next``, and how the step ended,
    ``terminated`` (the episode's cost fully counted) or ``done`` (terminated
    or truncated, the episode restarts). ``ep_return`` is the return of an
    episode that ended at that step (NaN elsewhere), for the logs. ``carry``
    is ``(x, t, ep_return, params, seed)`` with a leading ``n_envs`` axis:
    ``seed`` is the episode's seed of the policy, which draws the action of
    tick ``k = ⌊t/dt⌋`` at ``CounterKey(seed, k)``, the draw the seeded policy
    block makes with that seed.
    """
    jax, jnp = require_jax(), require_jax_numpy()

    def env_step(carry, key):
        x, t, ep_return, params, seed = carry
        k_seed, k_step, k_reset, k_params = jax.random.split(key, 4)

        # Policy: a ~ π_θ(·|x) drawn at the episode's seed and the tick,
        # applied as u = u_mid + u_half clip(a)
        tick = sample_index(t, env.dt)
        a = policy.sample(theta, x, CounterKey(seed, tick))
        u = policy.input(a)

        # Plant: one control period, and the return of the running episode
        x_next, t_next, reward, terminated, truncated = env.step(
            x, t, u, k_step, params
        )
        done = terminated | truncated
        ep_return = ep_return + reward

        # The transition, with the return of an episode that ends here
        transition = {
            "x": x,
            "a": a,
            "reward": reward,
            "x_next": x_next,
            "terminated": terminated.astype(float),
            "done": done.astype(float),
            "ep_return": jnp.where(done, ep_return, jnp.nan),
        }

        # A plant whose episode ended restarts; the others continue
        carry = restart_finished_episodes(
            env,
            (x_next, t_next, ep_return, params, seed),
            done,
            k_reset,
            k_params,
            k_seed,
        )

        return carry, transition

    def scan_step(carry, key):
        return jax.vmap(env_step)(carry, jax.random.split(key, n_envs))

    # The n_steps control periods of the n_envs plants
    carry, batch = jax.lax.scan(scan_step, carry, jax.random.split(key, n_steps))

    return batch, carry


def advantages(critic, w, batch, gamma, lam):
    """Advantages and returns of an on-policy batch: the critic's TD errors, then generalized advantage estimation."""
    x = batch["x"]
    terminated = batch["terminated"]
    x_next = batch["x_next"]
    reward = batch["reward"]
    done = batch["done"]

    # δ_k = r_k + γ V(x_{k+1}) − V(x_k), no value beyond a terminal state
    value = critic.value(w, x)
    value_next = (1.0 - terminated) * critic.value(w, x_next)
    delta = reward + gamma * value_next - value

    # Advantage A_k, and the return R_k = A_k + V(x_k) the critic regresses onto
    advantage = gae(delta, done, gamma, lam)
    returns = advantage + value

    return advantage, returns


def gae(delta, done, gamma, lam):
    """Generalized advantage estimation ``A_k = δ_k + γ λ (1 − done_k) A_{k+1}``, backwards in time."""
    # The TD errors summed backwards at the rate γ λ
    advantage = discounted_sum(delta, done, gamma * lam)

    return advantage


def returns_to_go(reward, done, gamma):
    """Discounted return to the end of the episode, ``R_k = r_k + γ (1 − done_k) R_{k+1}``."""
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

        # Capacity
        self.capacity = int(capacity)

        # Storage: one row per transition, zeros until written
        self.data = {
            "x": jnp.zeros((self.capacity, n)),
            "a": jnp.zeros((self.capacity, m)),
            "reward": jnp.zeros(self.capacity),
            "x_next": jnp.zeros((self.capacity, n)),
            "terminated": jnp.zeros(self.capacity),
        }

        # The ring's write cursor and its fill level
        self.cursor = 0
        self.size = 0

    def add(self, batch):
        """Write a batch of transitions at the cursor, overwriting the oldest when full."""
        k = int(batch["x"].shape[0])

        data = self.data
        cursor = self.cursor
        size = self.size
        capacity = self.capacity

        # A ring: the k slots from the cursor on, wrapping around past the capacity
        idx = (cursor + np.arange(k)) % capacity
        data = {key: data[key].at[idx].set(batch[key]) for key in data}

        self.data = data
        self.cursor = (cursor + k) % capacity
        self.size = min(size + k, capacity)

    def sample(self, key, n):
        """Draw ``n`` stored transitions uniformly."""
        jax = require_jax()

        size = self.size
        data = self.data
        require_transitions(size)

        idx = jax.random.randint(key, (int(n),), 0, size)

        return {key_: value[idx] for key_, value in data.items()}


###############################################################################
# Internal machinery
###############################################################################


def restart_finished_episodes(env, carry, done, k_reset, k_params, k_seed):
    """
    The carry of the next control period, per plant.

    A plant whose episode ended restarts from a fresh start, at ``t_start``, with
    a zero return, a fresh policy seed and, when the problem randomizes them,
    fresh parameters; the others continue.
    """
    jax, jnp = require_jax(), require_jax_numpy()

    x_next, t_next, ep_return, params, seed = carry

    # A finished episode restarts from a fresh start, at t_start, with a zero return
    x_next = jnp.where(done, env.reset(k_reset), x_next)
    t_next = jnp.where(done, env.t_start, t_next)
    ep_return = jnp.where(done, 0.0, ep_return)
    seed = jnp.where(done, episode_policy_seed(k_seed), seed)

    # And with fresh parameters, when the problem randomizes them
    if env.randomizes_params:
        fresh = env.sample_params(k_params)
        params = jax.tree_util.tree_map(
            lambda old, new: jnp.where(done, new, old), params, fresh
        )

    return x_next, t_next, ep_return, params, seed


def discounted_sum(increment, done, factor):
    """Backward recursion ``S_k = y_k + factor (1 − done_k) S_{k+1}`` along the time axis, zero past the end."""
    jax, jnp = require_jax(), require_jax_numpy()

    def step(s_next, inputs):
        y, d = inputs

        s = y + factor * (1.0 - d) * s_next

        return s, s

    # Swept backward in time, from zero past the last step
    s_end = jnp.zeros_like(increment[0])
    _, s = jax.lax.scan(step, s_end, (increment, done), reverse=True)

    return s


def reset_carry(env, key, n_envs):
    """Initial ``(x, t, ep_return, params, seed)`` carry for ``n_envs`` plants."""
    jax, jnp = require_jax(), require_jax_numpy()

    k_x, k_params = jax.random.split(key)
    k_seed = jax.random.fold_in(key, 1)

    # A fresh start, fresh parameters and a fresh policy seed per plant, at t_start
    # with a zero return
    x = jax.vmap(env.reset)(jax.random.split(k_x, n_envs))
    params = jax.vmap(env.sample_params)(jax.random.split(k_params, n_envs))
    seed = jax.vmap(episode_policy_seed)(jax.random.split(k_seed, n_envs))
    t = jnp.full(n_envs, env.t_start)
    ep_return = jnp.zeros(n_envs)

    return x, t, ep_return, params, seed


def episode_policy_seed(key):
    """An episode's seed of the policy, a non-negative 31-bit integer drawn under ``key``."""
    jax = require_jax()

    return jax.random.randint(key, (), 0, MAX_SEED)


def require_transitions(size):
    """Refuse a draw from a replay buffer that holds no transition."""
    if size == 0:
        raise ValueError("the replay buffer is empty; add transitions before sampling")
