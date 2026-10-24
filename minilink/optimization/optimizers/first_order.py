"""The first-order backend: gradient descent and Adam on a program, projected on its box."""

import numpy as np

from minilink.core.backends import require_jax, require_jax_numpy
from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
from minilink.optimization.mathematical_program import OptimizationResult
from minilink.optimization.optimizers.optimizer_backend import (
    BackendIterateCallback,
    OptimizerBackend,
)

ALGORITHMS = ("gd", "adam")

###############################################################################
# Public API
###############################################################################


class FirstOrderOptimizer(OptimizerBackend):
    """
    Gradient descent or Adam on an unconstrained or box-bounded program.

    Each iteration reads ``J(z)`` and ``∇J(z)`` and steps, then projects on the
    box: ``z ← Π(z − η ∇J)`` for ``"gd"``, the bias-corrected moment step of
    :class:`Adam` for ``"adam"``. It stops after ``maxiter`` steps, or when
    ``‖∇J‖ ≤ gtol``.

    Parameters
    ----------
    algorithm : {"gd", "adam"}
        The step rule.
    options : dict, optional
        ``learning_rate`` (η, default ``1e-2``), ``maxiter`` (default 1000),
        ``gtol`` (default ``0``: run every step), and for Adam ``b1``, ``b2``,
        ``eps`` and ``max_grad_norm`` (a global gradient clip, ``None`` for none).
    """

    DEFAULTS = {
        "learning_rate": 1e-2,
        "maxiter": 1000,
        "gtol": 0.0,
        "b1": 0.9,
        "b2": 0.999,
        "eps": 1e-8,
        "max_grad_norm": None,
    }

    def __init__(self, algorithm="gd", options=None):
        require_algorithm(algorithm)
        self.algorithm = algorithm
        self.options = {**self.DEFAULTS, **(options or {})}

    def solve(
        self,
        program_evaluator: MathematicalProgramEvaluator,
        z0: np.ndarray,
        *,
        callback: BackendIterateCallback | None = None,
    ) -> OptimizationResult:
        """Run the first-order loop from ``z0`` and return a backend-neutral result."""
        require_gradient(program_evaluator)
        require_no_constraints(program_evaluator)
        opts = self.options

        eta = float(opts["learning_rate"])
        maxiter = int(opts["maxiter"])
        gtol = float(opts["gtol"])
        lower, upper = box_of(program_evaluator)

        # The step rule: η ∇J, or Adam's moments
        adam_step = None
        if self.algorithm == "adam":
            jax = require_jax()
            adam = Adam(
                eta,
                b1=opts["b1"],
                b2=opts["b2"],
                eps=opts["eps"],
                max_grad_norm=opts["max_grad_norm"],
            )
            state = adam.init(np.asarray(z0, dtype=float))
            adam_step = jax.jit(adam.update)  # One compiled update, not one per step

        z = np.clip(np.asarray(z0, dtype=float), lower, upper)
        converged = False
        for k in range(maxiter):
            # Stationary on the box: ‖z − Π(z − ∇J)‖ ≤ gtol (‖∇J‖ inside the box)
            grad = program_evaluator.gradient(z)
            grad_norm = float(np.linalg.norm(z - np.clip(z - grad, lower, upper)))
            if grad_norm <= gtol:
                converged = True
                break

            # z ← Π(z − η ∇J), or Π of Adam's step
            if adam_step is None:
                z_next = z - eta * grad
            else:
                z_next, state = adam_step(grad, state, z)
                z_next = np.asarray(z_next, dtype=float)
            z_next = np.clip(z_next, lower, upper)

            # Gradient descent at a fixed point in floating point: the step no longer
            # moves z (Adam's moments still evolve, so its step may move z again)
            if adam_step is None and np.array_equal(z_next, z):
                converged = True
                break
            z = z_next

            if callback is not None:
                callback(z.copy())
        else:
            k = maxiter
            grad = program_evaluator.gradient(z)
            grad_norm = float(np.linalg.norm(z - np.clip(z - grad, lower, upper)))

        J = program_evaluator.objective(z)
        message = (
            f"stationary (projected gradient {grad_norm:.3g})"
            if converged
            else f"{maxiter} steps taken"
        )

        return OptimizationResult(
            z=z,
            success=bool(converged and np.isfinite(J)),
            cost=float(J),
            message=message,
            stats={"nit": k, "grad_norm": grad_norm, "algorithm": self.algorithm},
        )


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


###############################################################################
# Internal machinery
###############################################################################


def require_algorithm(algorithm):
    """Refuse a step rule the backend does not have."""
    if algorithm not in ALGORITHMS:
        raise ValueError(f"algorithm must be one of {ALGORITHMS}, got {algorithm!r}")


def require_gradient(program_evaluator):
    """Refuse a program without a gradient: a first-order step needs one."""
    if not program_evaluator.has_gradient:
        raise ValueError(
            "a first-order method needs the objective's gradient: give the program "
            "grad_J, or compile it on JAX (compile_backend='jax')"
        )


def require_no_constraints(program_evaluator):
    """Refuse a program with constraint rows: the first-order step only projects on the box."""
    if program_evaluator.n_g or program_evaluator.n_h:
        raise ValueError(
            "gd and adam take an unconstrained or box-bounded program; this one has "
            "constraint rows (use 'scipy_slsqp', 'scipy_trust_constr' or 'ipopt')"
        )


def box_of(program_evaluator):
    """The box of ``z``, open sides at ∓∞."""
    n_z = program_evaluator.n_z
    program = program_evaluator.program

    lower = np.full(n_z, -np.inf) if program.lower is None else program.lower
    upper = np.full(n_z, np.inf) if program.upper is None else program.upper

    return lower, upper
