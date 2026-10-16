"""Transcriptions: a planning problem as a finite-dimensional program, and back."""

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import (
    BACKEND_DIRECT,
    BACKEND_JAX,
    BACKEND_NUMPY,
    array_module,
    ensure_jax_x64,
    normalize_backend,
)
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.planning.problems import PlanningProblem

ConstraintFunction = Callable[[np.ndarray], np.ndarray]


###############################################################################
# Public API
###############################################################################


class Transcription(ABC):
    """
    Base class for finite-dimensional trajectory transcriptions.

    A transcription maps a continuous planning problem to a finite-dimensional
    mathematical program, then reconstructs a trajectory from the optimizer
    output.
    """

    #: True when :meth:`transcribe_parametric` builds a correct parametric
    #: program (the receding-horizon / MPC path). Checked by the planner
    #: instead of ``hasattr`` so a subclass cannot inherit a wrong one.
    supports_parametric = False

    @abstractmethod
    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str = BACKEND_NUMPY,
    ) -> MathematicalProgram:
        """Convert ``problem`` into a finite-dimensional mathematical program."""
        ...

    @abstractmethod
    def pack_initial_guess(
        self,
        problem: PlanningProblem,
        guess: np.ndarray | Trajectory | None,
    ) -> np.ndarray:
        """Pack a trajectory or array guess into the transcription decision vector."""
        ...

    @abstractmethod
    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        compile_backend: str = BACKEND_NUMPY,
    ) -> Trajectory:
        """Convert an optimizer result back into a trajectory."""
        ...

    @abstractmethod
    def initial_guess_time_grid(self, problem: PlanningProblem) -> np.ndarray:
        """Return the time grid used by generic trajectory guesses."""
        ...


@dataclass
class FixedGridOptions:
    """Uniform fixed-grid options — knot count only; horizon is ``problem.tf``."""

    n_steps: int

    def __post_init__(self) -> None:
        require_knot_count(self.n_steps)
        self.n_steps = int(self.n_steps)

    def t(self, problem: PlanningProblem) -> np.ndarray:
        """Time grid from finite ``problem.tf`` and ``n_steps``."""
        return fixed_grid_t(problem.require_finite_tf(), self.n_steps)

    def dt(self, problem: PlanningProblem) -> float:
        """Step size from finite ``problem.tf`` and ``n_steps``."""
        return fixed_grid_dt(problem.require_finite_tf(), self.n_steps)


# Shared transcription math: the textbook pieces every fixed-grid transcription
# needs (the trapezoidal quadrature of the running cost, the trapezoidal
# collocation defect, one RK4 step between input knots)


def trajectory_cost(cost, x, u, t, dt, params):
    """Sampled Bolza objective on a uniform grid.

    ``x`` and ``u`` are knot matrices with shape ``(dim, N)``; the running
    cost is integrated with the trapezoid rule.
    """
    # J = Σ dt/2 (g_k + g_{k+1}) + h(x_N, t_N)
    g = running_cost_samples(cost, x, u, t, params)
    running = trapezoid_integral(g, dt)
    terminal = cost.h(x[:, -1], t[-1], params=params)
    J = running + terminal

    return J


def running_cost_samples(cost, x, u, t, params):
    """Sample the running cost ``g(x_k, u_k, t_k)`` at every knot.

    NumPy inputs use a plain loop; JAX inputs vectorize with ``vmap`` so the
    whole-program jit stays compact.
    """
    # NumPy: one call per knot
    if array_module(x) is np:
        g = np.array(
            [
                cost.g(x[:, k], u[:, k], float(t[k]), params=params)
                for k in range(x.shape[1])
            ]
        )
        return g

    # JAX: one call mapped over the knots
    jax = ensure_jax_x64()
    g = jax.vmap(
        lambda x_k, u_k, t_k: cost.g(x_k, u_k, t_k, params=params),
        in_axes=(1, 1, 0),
    )(x, u, t)

    return g


def trapezoid_integral(values, dt):
    """Trapezoid rule on a uniform grid."""
    xp = array_module(values)

    # ∫ v dt ≈ Σ dt/2 (v_k + v_{k+1})
    integral = xp.sum(0.5 * dt * (values[:-1] + values[1:]))

    return integral


def trapezoidal_defect(x, dx, dt):
    """Trapezoidal collocation defect between neighboring knots.

    ``x`` and ``dx`` have shape ``(n, N)``; the result has shape ``(n, N-1)``.
    """
    # d_k = x_{k+1} − x_k − dt/2 (f_k + f_{k+1})
    defect = x[:, 1:] - x[:, :-1] - 0.5 * dt * (dx[:, :-1] + dx[:, 1:])

    return defect


def rk4_step_between_knots(f, x, u0, u1, t, dt):
    """One RK4 step from knot ``k`` to ``k+1`` with linear input interpolation."""
    # The input at mid-step, interpolated linearly between the knots
    umid = 0.5 * (u0 + u1)

    # The four slopes
    k1 = f(x, u0, t)
    k2 = f(x + 0.5 * dt * k1, umid, t + 0.5 * dt)
    k3 = f(x + 0.5 * dt * k2, umid, t + 0.5 * dt)
    k4 = f(x + dt * k3, u1, t + dt)

    # x_{k+1} = x_k + dt/6 (k₁ + 2 k₂ + 2 k₃ + k₄)
    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    return x_next


###############################################################################
# Internal machinery
###############################################################################


def program_backend_for_compile(compile_backend: str) -> str:
    """Return the mathematical-program evaluator backend for a transcription."""
    key = normalize_backend(compile_backend, allow_direct=True)
    return BACKEND_JAX if key == BACKEND_JAX else BACKEND_NUMPY


def dynamics_function(problem: PlanningProblem, compile_backend: str):
    """Return ``(x, u, t) -> f(x, u, t)`` for a transcription.

    ``"numpy"`` and ``"jax"`` use a compiled evaluator; system params from
    ``problem.params.system`` flow through the parametric tier ``f_p``.
    ``"direct"`` calls ``system.f`` without compiling (escape hatch for
    systems that cannot be compiled).
    """
    key = normalize_backend(compile_backend, allow_direct=True)
    params = problem.params.system

    # The escape hatch: the system's own f, uncompiled
    if key == BACKEND_DIRECT:
        return lambda x, u, t: problem.sys.f(x, u, t, params)

    # A compiled evaluator: its trace tier when it has one, the params when given
    evaluator = problem.sys.compile(backend=key, verbose=False)
    if getattr(evaluator, "has_trace_tier", False):
        if params is None:
            return lambda x, u, t: evaluator.f_trace(x, u, t)
        return lambda x, u, t: evaluator.f_trace_p(x, u, t, params)
    if params is None:
        return lambda x, u, t: evaluator.f(x, u, t)
    return lambda x, u, t: evaluator.f_p(x, u, t, params)


def native_concatenate(values, like):
    """Concatenate vector pieces with the array module used by ``like``.

    A scalar piece (a set margin written as a Python float) counts as one entry.
    """
    xp = array_module(like)

    # Each piece flattened, a scalar as one entry
    pieces = []
    for value in values:
        pieces.append(xp.reshape(xp.asarray(value), (-1,)))

    # No piece: the empty vector
    if not pieces:
        empty = xp.array([])
        return empty

    stacked = xp.concatenate(pieces)

    return stacked


def stack_constraints(
    constraints: list[ConstraintFunction],
) -> ConstraintFunction | None:
    """Return one native-array constraint vector from a list of vector functions.

    A constraint that returns a scalar (a terminal set margin written as a
    Python float) contributes one entry.
    """
    if not constraints:
        return None

    def stacked(z: np.ndarray) -> np.ndarray:
        # Each constraint evaluated, then flattened, in turn
        values = native_concatenate((constraint(z) for constraint in constraints), z)
        return values

    return stacked


def require_knot_count(n_steps):
    """Refuse a fixed grid of fewer than two knots."""
    if n_steps < 2:
        raise ValueError("n_steps must be at least 2")


def fixed_grid_t(tf: float, n_steps: int) -> np.ndarray:
    """Uniform time knots: ``linspace(0, tf, n_steps)``."""
    tf = float(tf)
    require_fixed_grid_horizon(tf)

    t = np.linspace(0.0, tf, int(n_steps))

    return t


def fixed_grid_dt(tf: float, n_steps: int) -> float:
    """Uniform step: ``tf / (n_steps - 1)``."""
    tf = float(tf)
    require_fixed_grid_horizon(tf)

    dt = tf / (int(n_steps) - 1)

    return dt


def require_fixed_grid_horizon(tf):
    """Refuse a fixed-grid horizon that is not positive and finite."""
    if not np.isfinite(tf) or tf <= 0.0:
        raise ValueError("fixed-grid tf must be positive and finite")
