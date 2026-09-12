"""Optimizer pieces without a deep-learning library: Adam and gradient clipping on pytrees."""

from minilink.core.backends import require_jax, require_jax_numpy

# Public API


class Adam:
    """
    Adam on a pytree of weights (``eps`` as in the common PPO configs).

    Any object exposing Optax-style ``init(params)`` and
    ``update(grads, state, params)`` can replace it.
    """

    def __init__(
        self, learning_rate=3e-4, b1=0.9, b2=0.999, eps=1e-5, max_grad_norm=0.5
    ):
        self.learning_rate = float(learning_rate)
        self.b1, self.b2, self.eps = float(b1), float(b2), float(eps)
        self.max_grad_norm = None if max_grad_norm is None else float(max_grad_norm)

    def init(self, params):
        """Zero moment estimates and a step counter."""
        jax = require_jax()
        zeros = jax.tree_util.tree_map(lambda w: w * 0.0, params)
        return {"m": zeros, "v": zeros, "step": jax.numpy.asarray(0)}

    def update(self, grads, state, params):
        """Return ``(new_params, new_state)`` after one clipped Adam step."""
        jax = require_jax()
        tree_map = jax.tree_util.tree_map
        if self.max_grad_norm is not None:
            grads = clip_by_global_norm(grads, self.max_grad_norm)
        b1, b2 = self.b1, self.b2
        step = state["step"] + 1

        # Moment estimates: m = b1 m + (1 - b1) g,  v = b2 v + (1 - b2) g^2
        m = tree_map(lambda m, g: b1 * m + (1 - b1) * g, state["m"], grads)
        v = tree_map(lambda v, g: b2 * v + (1 - b2) * g * g, state["v"], grads)

        # Bias-corrected step: w = w - lr m / (sqrt(v) + eps)
        lr = self.learning_rate * (1 - b2**step) ** 0.5 / (1 - b1**step)
        params = tree_map(
            lambda w, m, v: w - lr * m / (v**0.5 + self.eps), params, m, v
        )
        return params, {"m": m, "v": v, "step": step}


def clip_by_global_norm(grads, max_norm):
    """Scale a gradient pytree so its global L2 norm is at most ``max_norm``."""
    jax, jnp = require_jax(), require_jax_numpy()

    # One norm over every leaf, one common scale factor
    leaves = jax.tree_util.tree_leaves(grads)
    norm = jnp.sqrt(sum(jnp.sum(g**2) for g in leaves))
    scale = jnp.minimum(1.0, max_norm / (norm + 1e-6))
    return jax.tree_util.tree_map(lambda g: g * scale, grads)
