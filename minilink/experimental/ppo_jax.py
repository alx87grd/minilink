"""
PPO in pure JAX on a minilink plant (experimental, research lane).

Reinforcement learning without leaving the toolbox: the plant is compiled
with the JAX backend, the rollout is a ``jax.lax.scan`` over one RK4 step per
time step with the policy sampled inside the loop, and the PPO update (GAE,
clipped surrogate, value loss, Adam) is one jitted call. There is no
Gymnasium environment and no external RL library; the only dependency is JAX.

The problem is the one :class:`~minilink.interfaces.gymnasium.Sys2Gym` poses:

- ``x_{k+1} = rk4(x_k, u_k, t_k, dt)`` with the input held (ZOH);
- ``r_k = -g(x_k, u_k, t_k) * dt``, so maximizing the return minimizes the
  cost ``J = int g dt``;
- an episode truncates after ``tf`` seconds (the next state's value
  bootstraps the truncated reward) and, by default, when the state leaves
  its bounds; ``domain_exit="terminate"`` instead ends such an episode with
  the terminal cost ``h(x)`` and no bootstrap, which removes the incentive
  to escape the box for an extrapolated value.

The policy is a diagonal Gaussian ``a ~ N(mu_theta(x), sigma)`` on the
normalized action ``a in [-1, 1]`` (``u = u_mid + u_half * a`` spans the
input-port bounds), with an MLP mean and a state-independent log-std; the
critic is a second MLP. Sizes,
initialisation, and hyperparameters follow the usual PPO defaults (two
hidden layers of 64 tanh units, orthogonal init, 2048-step rollouts,
minibatches of 64, 10 epochs, lr 3e-4, gamma 0.99, lambda 0.95, clip 0.2),
so a training budget that works with the Gymnasium bridge should work here.

Usage::

    ppo = PPO(plant, cost, dt=0.05, x0_std=[5, 5, 1, 1, 1, 0.2])
    ppo.learn(100_000)
    cl_sys = ppo.controller @ plant

TODO: User Architectural Review — unvalidated prototype (TRL 2).
"""

import math
import time

import numpy as np

from minilink.core.backends import require_jax_numpy
from minilink.core.feedback import Controller

# Public API


class PPO:
    """
    Proximal Policy Optimization on a compiled minilink plant.

    Parameters
    ----------
    sys : DynamicSystem
        Plant with state and input bounds (``sys.state`` and
        ``sys.inputs["u"]``). Must trace under the JAX backend.
    cost : CostFunction
        Running cost ``g(x, u, t)``; the reward is ``-g * dt``.
    dt : float
        Control period (zero-order hold, one RK4 step per period).
    tf : float
        Episode duration (truncation).
    domain_exit : {"truncate", "terminate"}
        What leaving the state bounds does: ``"truncate"`` bootstraps with the
        critic (the Gymnasium bridge behaviour); ``"terminate"`` ends the
        episode with reward ``-h(x_next)`` and no bootstrap.
    reset_mode : {"gaussian", "uniform", "determinist"} or callable
        Initial-state distribution around ``sys.x0``, or a JAX-traceable
        ``reset(key) -> x0`` for task-specific starts (e.g. random points
        along a track).
    x0_std : array, optional
        Standard deviation for ``reset_mode="gaussian"`` (default: a tenth
        of the state range).
    x0_lb, x0_ub : array, optional
        Bounds for ``reset_mode="uniform"`` (default: ``x0 +- 0.1 * range``).
    n_envs : int
        Rollouts simulated in parallel (vmapped); ``n_steps`` is per env.
    seed : int
        Seed of the policy initialisation, exploration, and resets.
    features : callable, optional
        Observation map ``z = features(x)`` fed to both networks (default:
        the state itself). Use it to make angles periodic, e.g.
        ``lambda x: xp.array([x[0], cos(x[1]), sin(x[1]), x[2], x[3]])``;
        it must trace under JAX. The learned law stays ``u = pi(x)``.

    The remaining keyword arguments are the PPO hyperparameters, named as in
    the common implementations: ``hidden``, ``n_steps``, ``batch_size``,
    ``n_epochs``, ``learning_rate``, ``gamma``, ``gae_lambda``,
    ``clip_range``, ``ent_coef``, ``vf_coef``, ``max_grad_norm``, plus
    ``log_std_init`` (initial exploration log-std on the normalized action;
    ``0`` explores the full input range, lower values start gentler).

    Attributes
    ----------
    weights : dict
        Network parameters ``{"actor", "critic", "log_std"}`` (a pytree),
        updated in place by :meth:`learn`.
    controller : JaxPolicyController
        The deterministic policy ``u = mu_theta(x)`` as a feedback block.
    history : list of dict
        One record per training iteration (timesteps, mean episode return,
        losses, fps).
    """

    def __init__(
        self,
        sys,
        cost,
        dt=0.05,
        tf=10.0,
        reset_mode="gaussian",
        x0_std=None,
        x0_lb=None,
        x0_ub=None,
        n_envs=1,
        seed=0,
        hidden=(64, 64),
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        learning_rate=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.0,
        vf_coef=0.5,
        max_grad_norm=0.5,
        log_std_init=0.0,
        integrator="rk4",
        domain_exit="truncate",
        features=None,
        verbose=1,
    ):
        jnp = require_jax_numpy()
        import jax

        if domain_exit not in ("truncate", "terminate"):
            raise ValueError(
                f"domain_exit must be 'truncate' or 'terminate', got {domain_exit!r}"
            )
        self.domain_exit = domain_exit
        self.features = (lambda x: x) if features is None else features

        self.sys = sys
        self.cost = cost
        self.dt = float(dt)
        self.tf = float(tf)
        self.n_envs = int(n_envs)
        self.n_steps = int(n_steps)
        self.batch_size = int(batch_size)
        self.n_epochs = int(n_epochs)
        self.learning_rate = float(learning_rate)
        self.gamma = float(gamma)
        self.gae_lambda = float(gae_lambda)
        self.clip_range = float(clip_range)
        self.ent_coef = float(ent_coef)
        self.vf_coef = float(vf_coef)
        self.max_grad_norm = float(max_grad_norm)
        self.verbose = verbose
        self.num_timesteps = 0
        self.train_time = 0.0
        self.history = []

        if (self.n_steps * self.n_envs) % self.batch_size != 0:
            raise ValueError("n_steps * n_envs must be a multiple of batch_size")

        # Plant: one compiled step x_{k+1} = step(x_k, u_k, t_k, dt), traceable
        self.evaluator = sys.compile(backend="jax", verbose=False)
        if integrator == "rk4":
            self.step = self.evaluator.rk4_step_trace
        elif integrator == "euler":
            self.step = self.evaluator.euler_step_trace
        else:
            raise ValueError(f"integrator must be 'rk4' or 'euler', got {integrator!r}")

        # Bounds (state bounds truncate episodes, input bounds clip actions)
        self.x_lb = jnp.asarray(sys.state.lower_bound, dtype=float)
        self.x_ub = jnp.asarray(sys.state.upper_bound, dtype=float)
        self.u_lb = jnp.asarray(sys.inputs["u"].lower_bound, dtype=float)
        self.u_ub = jnp.asarray(sys.inputs["u"].upper_bound, dtype=float)
        self.n = int(sys.n)
        self.m = int(self.u_lb.shape[0])
        self.u_mid = 0.5 * (self.u_ub + self.u_lb)  # normalized action a in [-1, 1]
        self.u_half = 0.5 * (self.u_ub - self.u_lb)

        # Initial-state distribution
        self.x0 = jnp.asarray(sys.x0, dtype=float)
        x_range = self.x_ub - self.x_lb
        self.reset_mode = reset_mode
        self.x0_std = jnp.asarray(
            0.1 * x_range if x0_std is None else np.asarray(x0_std, dtype=float)
        )
        self.x0_lb = self.x0 + 0.1 * self.x_lb if x0_lb is None else jnp.asarray(x0_lb)
        self.x0_ub = self.x0 + 0.1 * self.x_ub if x0_ub is None else jnp.asarray(x0_ub)

        # Networks: actor mean, critic value, state-independent log-std
        n_features = int(jnp.asarray(self.features(self.x0)).shape[0])
        self.key = jax.random.PRNGKey(seed)
        self.key, k_actor, k_critic = jax.random.split(self.key, 3)
        self.weights = {
            "actor": mlp_init(k_actor, n_features, hidden, self.m, out_gain=0.01),
            "critic": mlp_init(k_critic, n_features, hidden, 1, out_gain=1.0),
            "log_std": jnp.full(self.m, float(log_std_init)),
        }
        self.opt_state = adam_init(self.weights)

        # Jitted training and evaluation pieces
        self.rollout_jit = jax.jit(self.rollout)
        self.update_jit = jax.jit(self.update)
        self.action_jit = jax.jit(self.action)

        # Per-env simulation memory (state, time, running episode return)
        self.key, k_reset = jax.random.split(self.key)
        self.x = jax.vmap(self.reset)(jax.random.split(k_reset, self.n_envs))
        self.t = jnp.zeros(self.n_envs)
        self.ep_return = jnp.zeros(self.n_envs)

        self.controller = JaxPolicyController(self, sys=sys)

    # --- policy ---

    def mean_action(self, weights, x):
        """Deterministic normalized action ``a = mu_theta(x)`` (unclipped)."""
        return mlp_apply(weights["actor"], self.features(x))

    def to_input(self, a):
        """Map a normalized action to the plant input, clipped to the bounds."""
        jnp = require_jax_numpy()
        return self.u_mid + self.u_half * jnp.clip(a, -1.0, 1.0)

    def value(self, weights, x):
        """Critic ``V_phi(x)``."""
        return mlp_apply(weights["critic"], self.features(x))[0]

    def log_prob(self, weights, x, a):
        """Log-density of the normalized action ``a`` under the policy at ``x``."""
        jnp = require_jax_numpy()
        mu = self.mean_action(weights, x)
        log_std = weights["log_std"]
        z = (a - mu) / jnp.exp(log_std)
        return jnp.sum(-0.5 * z**2 - log_std - 0.5 * math.log(2.0 * math.pi))

    def entropy(self, weights):
        """Entropy of the diagonal Gaussian policy (state independent)."""
        jnp = require_jax_numpy()
        return jnp.sum(weights["log_std"] + 0.5 * math.log(2.0 * math.pi * math.e))

    def predict(self, x, deterministic=True):
        """Return ``(u, None)`` for one state or a batch of states (rows)."""
        jnp = require_jax_numpy()
        import jax

        x = jnp.asarray(x, dtype=float)
        act = jax.vmap(self.action_jit, in_axes=(None, 0))
        u = act(self.weights, x) if x.ndim == 2 else self.action_jit(self.weights, x)
        return np.asarray(u), None

    def action(self, weights, x):
        """Deterministic plant input ``u = u_mid + u_half * clip(mu_theta(x))``."""
        return self.to_input(self.mean_action(weights, x))

    # --- environment (one step, one env) ---

    def reset(self, key):
        """Draw an initial state."""
        jnp = require_jax_numpy()
        import jax

        if callable(self.reset_mode):
            return jnp.asarray(self.reset_mode(key), dtype=float)
        if self.reset_mode == "gaussian":
            return self.x0 + self.x0_std * jax.random.normal(key, (self.n,))
        if self.reset_mode == "uniform":
            return jax.random.uniform(
                key, (self.n,), minval=self.x0_lb, maxval=self.x0_ub
            )
        return jnp.array(self.x0)

    def env_step(self, weights, carry, key):
        """Sample ``u ~ pi(x)``, integrate one period, reward, and truncate."""
        jnp = require_jax_numpy()
        import jax

        x, t, ep_return = carry
        k_sample, k_reset = jax.random.split(key)
        dt = self.dt

        # Explore: Gaussian sample stored unclipped, the plant sees the clipped input
        mu = self.mean_action(weights, x)
        a = mu + jnp.exp(weights["log_std"]) * jax.random.normal(k_sample, (self.m,))
        u_plant = self.to_input(a)
        logp = self.log_prob(weights, x, a)
        v = self.value(weights, x)

        # Plant and reward
        x_next = self.step(x, u_plant, t, dt)
        t_next = t + dt
        r = -self.cost.g(x, u_plant, t) * dt

        # Episode end: horizon (bootstrap) or domain exit (bootstrap or h)
        out_of_bounds = jnp.any(x_next < self.x_lb) | jnp.any(x_next > self.x_ub)
        terminated = out_of_bounds & (self.domain_exit == "terminate")
        truncated = (t_next > self.tf) | (out_of_bounds & ~terminated)
        done = terminated | truncated
        r = r - terminated * self.cost.h(x_next, t_next)
        ep_return_next = ep_return + r  # logged return: raw rewards only
        r = r + truncated * self.gamma * self.value(weights, x_next)
        x_reset = self.reset(k_reset)
        x_next = jnp.where(done, x_reset, x_next)
        t_next = jnp.where(done, 0.0, t_next)

        sample = {
            "x": x,
            "u": a,
            "logp": logp,
            "value": v,
            "reward": r,
            "done": done.astype(float),
            "ep_return": jnp.where(done, ep_return_next, jnp.nan),
        }
        ep_return_next = jnp.where(done, 0.0, ep_return_next)
        return (x_next, t_next, ep_return_next), sample

    # --- rollout and advantage estimation ---

    def rollout(self, weights, x, t, ep_return, key):
        """Simulate ``n_steps`` on ``n_envs`` plants; return the batch and the memory."""
        jnp = require_jax_numpy()
        import jax

        def scan_step(carry, key):
            keys = jax.random.split(key, self.n_envs)
            return jax.vmap(self.env_step, in_axes=(None, 0, 0))(weights, carry, keys)

        keys = jax.random.split(key, self.n_steps)
        (x, t, ep_return), batch = jax.lax.scan(scan_step, (x, t, ep_return), keys)

        # Generalized advantage estimation, backwards in time (per env)
        last_value = jax.vmap(self.value, in_axes=(None, 0))(weights, x)
        gamma, lam = self.gamma, self.gae_lambda

        def gae_step(carry, sample):
            gae, next_value = carry
            not_done = 1.0 - sample["done"]
            delta = sample["reward"] + gamma * next_value * not_done - sample["value"]
            gae = delta + gamma * lam * not_done * gae
            return (gae, sample["value"]), gae

        zeros = jnp.zeros(self.n_envs)
        _, advantage = jax.lax.scan(gae_step, (zeros, last_value), batch, reverse=True)
        batch["advantage"] = advantage
        batch["return"] = advantage + batch["value"]
        return batch, (x, t, ep_return)

    # --- PPO update ---

    def loss(self, weights, minibatch):
        """Clipped surrogate + value loss - entropy bonus on one minibatch."""
        jnp = require_jax_numpy()
        import jax

        x, u = minibatch["x"], minibatch["u"]
        logp = jax.vmap(self.log_prob, in_axes=(None, 0, 0))(weights, x, u)
        v = jax.vmap(self.value, in_axes=(None, 0))(weights, x)

        adv = minibatch["advantage"]
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        ratio = jnp.exp(logp - minibatch["logp"])
        clipped = jnp.clip(ratio, 1.0 - self.clip_range, 1.0 + self.clip_range)
        policy_loss = -jnp.mean(jnp.minimum(adv * ratio, adv * clipped))
        value_loss = jnp.mean((minibatch["return"] - v) ** 2)
        entropy = self.entropy(weights)

        total = policy_loss + self.vf_coef * value_loss - self.ent_coef * entropy
        stats = {
            "policy_loss": policy_loss,
            "value_loss": value_loss,
            "entropy": entropy,
            "approx_kl": jnp.mean((ratio - 1.0) - jnp.log(ratio)),
        }
        return total, stats

    def update(self, weights, opt_state, batch, key):
        """``n_epochs`` passes of minibatch Adam steps over the flattened batch."""
        jnp = require_jax_numpy()
        import jax

        size = self.n_steps * self.n_envs
        n_minibatches = size // self.batch_size
        flat = {
            k: batch[k].reshape((size,) + batch[k].shape[2:])
            for k in ("x", "u", "logp", "advantage", "return")
        }
        grad_fn = jax.value_and_grad(self.loss, has_aux=True)

        def minibatch_step(carry, idx):
            weights, opt_state = carry
            minibatch = {k: v[idx] for k, v in flat.items()}
            (_, stats), grads = grad_fn(weights, minibatch)
            grads = clip_by_global_norm(grads, self.max_grad_norm)
            weights, opt_state = adam_update(
                weights, grads, opt_state, self.learning_rate
            )
            return (weights, opt_state), stats

        def epoch(carry, key):
            perm = jax.random.permutation(key, size).reshape(n_minibatches, -1)
            return jax.lax.scan(minibatch_step, carry, perm)

        keys = jax.random.split(key, self.n_epochs)
        (weights, opt_state), stats = jax.lax.scan(epoch, (weights, opt_state), keys)
        stats = {k: jnp.mean(v) for k, v in stats.items()}
        return weights, opt_state, stats

    def learn(self, total_timesteps):
        """Train for ``total_timesteps`` plant steps (rounded up to whole rollouts)."""
        import jax

        n_iterations = math.ceil(total_timesteps / (self.n_steps * self.n_envs))
        for _ in range(n_iterations):
            self.key, k_roll, k_update = jax.random.split(self.key, 3)

            t0 = time.time()
            batch, (self.x, self.t, self.ep_return) = self.rollout_jit(
                self.weights, self.x, self.t, self.ep_return, k_roll
            )
            self.weights, self.opt_state, stats = self.update_jit(
                self.weights, self.opt_state, batch, k_update
            )
            jax.block_until_ready(self.weights)

            self.num_timesteps += self.n_steps * self.n_envs
            self.train_time += time.time() - t0
            ep_returns = np.asarray(batch["ep_return"])
            ep_returns = ep_returns[np.isfinite(ep_returns)]
            record = {
                "timesteps": self.num_timesteps,
                "ep_return_mean": float(ep_returns.mean())
                if ep_returns.size
                else np.nan,
                "n_episodes": int(ep_returns.size),
                "fps": self.n_steps * self.n_envs / (time.time() - t0),
                "elapsed": self.train_time,
                **{k: float(v) for k, v in stats.items()},
            }
            self.history.append(record)
            if self.verbose:
                print(
                    f"steps {record['timesteps']:>8d} | "
                    f"ep_return {record['ep_return_mean']:9.1f} | "
                    f"policy_loss {record['policy_loss']:7.4f} | "
                    f"value_loss {record['value_loss']:9.2f} | "
                    f"kl {record['approx_kl']:.4f} | "
                    f"fps {record['fps']:7.0f} | "
                    f"{record['elapsed']:.0f}s"
                )
        return self


class JaxPolicyController(Controller):
    """
    State-feedback block ``u = clip(mu_theta(x))`` for a :class:`PPO` policy.

    Reads the current ``ppo.weights`` at every call, so the block follows
    training in place; ``plot_control_law`` draws the same slice as the LQR
    / VI controllers. Traces under JAX (closed-loop compile) and runs on
    NumPy inputs (simulation, plots).
    """

    measurement_port = "x"
    ref_port = None
    control_port = "u"
    plot_space = "state"

    def __init__(self, ppo, sys=None, name="PPO Policy Controller (JAX)"):
        super().__init__()
        self.name = name
        self.ppo = ppo

        n, m = ppo.n, ppo.m
        x_labels = x_units = u_labels = u_units = None
        x_lb, x_ub = np.asarray(ppo.x_lb), np.asarray(ppo.x_ub)
        u_lb, u_ub = np.asarray(ppo.u_lb), np.asarray(ppo.u_ub)
        if sys is not None:
            x_labels = list(sys.state.labels)
            x_units = list(sys.state.units)
            u_labels = list(sys.inputs["u"].labels)
            u_units = list(sys.inputs["u"].units)

        self.add_input_port(
            "x",
            dim=n,
            labels=x_labels,
            units=x_units,
            lower_bound=x_lb,
            upper_bound=x_ub,
        )
        self.add_output_port(
            "u",
            dim=m,
            function=self.ctl,
            dependencies=("x",),
            labels=u_labels,
            units=u_units,
            lower_bound=u_lb,
            upper_bound=u_ub,
        )

    def action(self, x):
        """Deterministic clipped action at one state (NumPy in, NumPy out)."""
        return np.asarray(
            self.ppo.action_jit(self.ppo.weights, np.asarray(x, dtype=float))
        )

    def ctl(self, x, u, t=0, params=None):
        """State feedback; the ``x`` input port carries the plant state in ``u``."""
        if type(u).__module__.startswith("jax"):  # tracing: stay in the graph
            return self.ppo.action(self.ppo.weights, u)
        return self.action(u)


# Internal machinery: MLP and Adam without a neural-network library


def mlp_init(key, n_in, hidden, n_out, out_gain=1.0):
    """Orthogonal init: gain sqrt(2) on hidden tanh layers, ``out_gain`` on the last."""
    import jax

    sizes = (n_in,) + tuple(hidden) + (n_out,)
    gains = [math.sqrt(2.0)] * len(hidden) + [out_gain]
    layers = []
    for k, (a, b, gain) in enumerate(zip(sizes[:-1], sizes[1:], gains)):
        w = jax.nn.initializers.orthogonal(gain)(jax.random.fold_in(key, k), (a, b))
        layers.append({"W": w, "b": jax.numpy.zeros(b)})
    return layers


def mlp_apply(layers, x):
    """``tanh`` hidden layers, linear output."""
    jnp = require_jax_numpy()
    h = x
    for layer in layers[:-1]:
        h = jnp.tanh(h @ layer["W"] + layer["b"])
    return h @ layers[-1]["W"] + layers[-1]["b"]


def adam_init(weights):
    import jax

    zeros = jax.tree_util.tree_map(lambda w: w * 0.0, weights)
    return {"m": zeros, "v": zeros, "step": jax.numpy.asarray(0)}


def adam_update(weights, grads, state, lr, b1=0.9, b2=0.999, eps=1e-5):
    """One Adam step on a pytree (``eps`` as in the common PPO configs)."""
    import jax

    tree_map = jax.tree_util.tree_map
    step = state["step"] + 1
    m = tree_map(lambda m, g: b1 * m + (1 - b1) * g, state["m"], grads)
    v = tree_map(lambda v, g: b2 * v + (1 - b2) * g * g, state["v"], grads)
    lr_t = lr * (1 - b2**step) ** 0.5 / (1 - b1**step)
    weights = tree_map(lambda w, m, v: w - lr_t * m / (v**0.5 + eps), weights, m, v)
    return weights, {"m": m, "v": v, "step": step}


def clip_by_global_norm(grads, max_norm):
    import jax

    jnp = jax.numpy
    leaves = jax.tree_util.tree_leaves(grads)
    norm = jnp.sqrt(sum(jnp.sum(g**2) for g in leaves))
    scale = jnp.minimum(1.0, max_norm / (norm + 1e-6))
    return jax.tree_util.tree_map(lambda g: g * scale, grads)
