"""Optimizer pieces without a deep-learning library: Adam and gradient clipping on pytrees."""

from minilink.core.backends import require_jax, require_jax_numpy

###############################################################################
# Public API
###############################################################################


class Adam:
    """
    Adam on a pytree of weights (``eps`` as in the common PPO configs).

    Any object exposing Optax-style ``init(params)`` and
    ``update(grads, state, params)`` can replace it.
    """

    def __init__(
        self, learning_rate=3e-4, b1=0.9, b2=0.999, eps=1e-5, max_grad_norm=0.5
    ):
        # Hyperparameters: the step size η, the moment decay rates β₁ and β₂, and ε
        self.learning_rate = float(learning_rate)
        self.b1 = float(b1)
        self.b2 = float(b2)
        self.eps = float(eps)

        # Gradient clip: the largest global norm, or None for none
        self.max_grad_norm = None if max_grad_norm is None else float(max_grad_norm)

    def init(self, params):
        """Zero moment estimates and a step counter."""
        jax, jnp = require_jax(), require_jax_numpy()

        # m = v = 0 in the shape of the weights, at step k = 0
        zeros = jax.tree_util.tree_map(lambda w: w * 0.0, params)
        step = jnp.asarray(0)

        state = {"m": zeros, "v": zeros, "step": step}

        return state

    def update(self, grads, state, params):
        """Return ``(new_params, new_state)`` after one clipped Adam step."""
        jax = require_jax()
        tree_map = jax.tree_util.tree_map

        eta = self.learning_rate
        b1 = self.b1
        b2 = self.b2
        eps = self.eps
        max_norm = self.max_grad_norm

        # The gradients clipped to one global norm, then the step count k advanced
        if max_norm is not None:
            grads = clip_by_global_norm(grads, max_norm)
        step = state["step"] + 1

        # Moment estimates: m = β₁ m + (1 − β₁) g,  v = β₂ v + (1 − β₂) g²
        m = tree_map(lambda m, g: b1 * m + (1 - b1) * g, state["m"], grads)
        v = tree_map(lambda v, g: b2 * v + (1 - b2) * g * g, state["v"], grads)

        # Bias-corrected step: w = w − η_k m / (√v + ε), η_k = η √(1 − β₂ᵏ) / (1 − β₁ᵏ)
        eta_k = eta * (1 - b2**step) ** 0.5 / (1 - b1**step)
        params = tree_map(lambda w, m, v: w - eta_k * m / (v**0.5 + eps), params, m, v)

        state = {"m": m, "v": v, "step": step}

        return params, state


def clip_by_global_norm(grads, max_norm):
    """Scale a gradient pytree so its global L2 norm is at most ``max_norm``."""
    jax, jnp = require_jax(), require_jax_numpy()

    leaves = jax.tree_util.tree_leaves(grads)

    # The global norm ‖g‖ = √(Σ g²) over every leaf
    norm = jnp.sqrt(sum(jnp.sum(g**2) for g in leaves))

    # One common scale, min(1, max_norm / (‖g‖ + 10⁻⁶)), on every leaf
    scale = jnp.minimum(1.0, max_norm / (norm + 1e-6))
    clipped = jax.tree_util.tree_map(lambda g: g * scale, grads)

    return clipped
