"""The first-order backend: gradient descent and Adam on a program, projected on its box."""

import numpy as np

from minilink.core.backends import array_module, require_jax, require_jax_numpy
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
    :class:`Adam` for ``"adam"``. :meth:`solve` stops after ``maxiter`` steps,
    or when ``‖∇J‖ ≤ gtol``; :meth:`run` takes exactly ``K`` steps, one
    ``lax.scan`` on JAX, pure and differentiable (R149). Both run the method's
    pure law (:meth:`law`, R132): ``init(z0) -> state`` and
    ``update(grad, state) -> state``, the gradient oracle beside it.

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

    def law(self, program_evaluator):
        """The method's pure law on the program's box: :class:`GradientDescentLaw` or :class:`AdamLaw`."""
        opts = self.options
        lower, upper = box_of(program_evaluator)
        eta = float(opts["learning_rate"])

        if self.algorithm == "gd":
            return GradientDescentLaw(eta, lower, upper)

        return AdamLaw(
            eta,
            lower,
            upper,
            b1=opts["b1"],
            b2=opts["b2"],
            eps=opts["eps"],
            max_grad_norm=opts["max_grad_norm"],
        )

    def solve(
        self,
        program_evaluator: MathematicalProgramEvaluator,
        z0: np.ndarray,
        *,
        callback: BackendIterateCallback | None = None,
    ) -> OptimizationResult:
        """Run the first-order loop from ``z0`` to its tolerance (``maxiter`` its cap) and return a backend-neutral result."""
        require_gradient(program_evaluator)
        require_no_constraints(program_evaluator)
        opts = self.options

        maxiter = int(opts["maxiter"])
        gtol = float(opts["gtol"])
        lower, upper = box_of(program_evaluator)

        # The law, Adam's update compiled once when JAX is at hand (not once per step)
        law = self.law(program_evaluator)
        update = law.update if self.algorithm == "gd" else compiled(law.update)
        state = law.init(np.asarray(z0, dtype=float))

        z = state["z"]
        converged = False
        for k in range(maxiter):
            # Stationary on the box: ‖z − Π(z − ∇J)‖ ≤ gtol (‖∇J‖ inside the box)
            grad = program_evaluator.gradient(z)
            grad_norm = float(np.linalg.norm(z - np.clip(z - grad, lower, upper)))
            if grad_norm <= gtol:
                converged = True
                break

            # z ← Π(z − η ∇J), or Π of Adam's step
            state = update(grad, state)
            z_next = np.asarray(state["z"], dtype=float)

            # Gradient descent at a fixed point in floating point: the step no longer
            # moves z (Adam's moments still evolve, so its step may move z again)
            if self.algorithm == "gd" and np.array_equal(z_next, z):
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

    def run(self, program_evaluator, z0, params, iterations):
        """
        Exactly ``iterations`` steps of the law from ``z0`` on the tree ``params``: ``(z_K, J(z_K))``.

        On a JAX program the steps are one ``lax.scan``: pure, so the solve
        traces under ``jit``, ``vmap`` and ``grad`` (R149); on NumPy, a loop of
        the same steps. No tolerance, no early stop: the count is the contract.
        """
        require_gradient(program_evaluator)
        require_no_constraints(program_evaluator)
        law = self.law(program_evaluator)
        gradient = program_evaluator.native_gradient
        K = int(iterations)

        # z_{k+1} = S(z_k, ∇J(z_k; p)), k = 0 … K − 1
        def body(state, _):
            grad = gradient(state["z"], params)
            return law.update(grad, state), None

        state = law.init(z0)
        if program_evaluator.backend == "jax":
            jax = require_jax()
            state, _ = jax.lax.scan(body, state, None, length=K)
        else:
            for _ in range(K):
                state, _ = body(state, None)

        z = state["z"]
        J = program_evaluator.native_objective(z, params)

        return z, J


class GradientDescentLaw:
    """Projected gradient descent as a pure law: ``z ← Π(z − η ∇J)`` on the array module of its input."""

    def __init__(self, learning_rate, lower, upper):
        self.learning_rate = float(learning_rate)
        self.lower = lower
        self.upper = upper

    def init(self, z0):
        """The state ``{"z": Π(z0)}``: the start projected on the box."""
        xp = array_module(z0)
        z = xp.clip(z0, self.lower, self.upper)

        return {"z": z}

    def update(self, grad, state):
        """One step: ``z ← Π(z − η ∇J)``."""
        xp = array_module(grad, state["z"])

        eta = self.learning_rate
        z = state["z"]

        # z ← Π(z − η ∇J)
        z_next = xp.clip(z - eta * grad, self.lower, self.upper)

        return {"z": z_next}


class AdamLaw:
    """Projected Adam on a flat ``z`` as a pure law, on the array module of its input: :class:`Adam`'s step, then ``Π``."""

    def __init__(
        self,
        learning_rate,
        lower,
        upper,
        b1=0.9,
        b2=0.999,
        eps=1e-8,
        max_grad_norm=None,
    ):
        # Hyperparameters: the step size η, the moment decay rates β₁ and β₂, and ε
        self.learning_rate = float(learning_rate)
        self.b1 = float(b1)
        self.b2 = float(b2)
        self.eps = float(eps)
        self.max_grad_norm = None if max_grad_norm is None else float(max_grad_norm)

        # The box the step is projected on
        self.lower = lower
        self.upper = upper

    def init(self, z0):
        """The state: ``z`` projected on the box, zero moments ``m`` and ``v``, the step count ``0``."""
        xp = array_module(z0)

        z = xp.clip(z0, self.lower, self.upper)
        zeros = z0 * 0.0
        step = xp.asarray(0)

        return {"z": z, "m": zeros, "v": zeros, "step": step}

    def update(self, grad, state):
        """One step: the clipped gradient's moments, the bias-corrected step, then ``Π``."""
        xp = array_module(grad, state["z"])

        eta = self.learning_rate
        b1 = self.b1
        b2 = self.b2
        eps = self.eps
        max_norm = self.max_grad_norm
        z = state["z"]

        # The gradient clipped to one norm, then the step count k advanced
        if max_norm is not None:
            norm = safe_root(xp.sum(grad**2), xp)
            grad = grad * xp.minimum(1.0, max_norm / (norm + 1e-6))
        step = state["step"] + 1

        # Moment estimates: m = β₁ m + (1 − β₁) g,  v = β₂ v + (1 − β₂) g²
        m = b1 * state["m"] + (1 - b1) * grad
        v = b2 * state["v"] + (1 - b2) * grad * grad

        # Bias-corrected step: z ← Π(z − η_k m / (√v + ε)), η_k = η √(1 − β₂ᵏ) / (1 − β₁ᵏ)
        eta_k = eta * (1 - b2**step) ** 0.5 / (1 - b1**step)
        z_next = xp.clip(
            z - eta_k * m / (safe_root(v, xp) + eps), self.lower, self.upper
        )

        return {"z": z_next, "m": m, "v": v, "step": step}


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


def safe_root(v, xp):
    """``√v`` with a finite derivative at ``v = 0`` (zero there): the same value, so ``grad`` through a step is never ``nan``."""
    positive = v > 0

    # √v where v > 0, 0 elsewhere, the root never taken at 0
    root = xp.where(positive, xp.where(positive, v, 1.0) ** 0.5, 0.0)

    return root


def compiled(update):
    """The law's update compiled once by ``jax.jit`` when JAX is at hand, else the update on NumPy."""
    try:
        jax = require_jax()
    except ImportError:
        return update

    return jax.jit(update)


def require_algorithm(algorithm):
    """Refuse a step rule the backend does not have."""
    if algorithm not in ALGORITHMS:
        raise ValueError(f"algorithm must be one of {ALGORITHMS}, got {algorithm!r}")


def require_gradient(program_evaluator):
    """Refuse a program without a gradient: a first-order step needs one."""
    if not program_evaluator.has_gradient:
        raise ValueError(
            "a first-order method needs the objective's gradient: give the program "
            "grad_J, or compile it on JAX (backend='jax')"
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
