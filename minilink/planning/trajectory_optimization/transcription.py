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
from minilink.planning.problems import PlanningProblem, nominal_plant_params

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

    #: The plant compiled for this transcription, ``(sys, backend, evaluator)``:
    #: a transcription is built once per planner and compiles its plant once
    held_plant = None

    def plant_evaluator(self, problem: PlanningProblem, backend: str):
        """The plant of ``problem`` compiled on ``backend``, held while the plant is the same object (``None`` in direct mode)."""
        key = normalize_backend(backend, allow_direct=True)
        if key == BACKEND_DIRECT:
            return None

        held = self.held_plant
        if held is None or held[0] is not problem.sys or held[1] != key:
            held = (problem.sys, key, problem.sys.compile(backend=key))
            self.held_plant = held

        return held[2]

    @abstractmethod
    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        backend: str = BACKEND_NUMPY,
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
        backend: str = BACKEND_NUMPY,
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
        """The time grid on the horizon ``[t_start, t_start + tf]``: ``t_start`` plus ``linspace(0, tf, n_steps)``."""
        t = problem.t_start + fixed_grid_t(problem.require_finite_tf(), self.n_steps)

        return t

    def offsets(self, problem: PlanningProblem) -> np.ndarray:
        """The knots' offsets from the horizon's start, ``linspace(0, tf, n_steps)``: ``t_k = t_start + offset_k``."""
        return fixed_grid_t(problem.require_finite_tf(), self.n_steps)

    def dt(self, problem: PlanningProblem) -> float:
        """Step size from finite ``problem.tf`` and ``n_steps``."""
        return fixed_grid_dt(problem.require_finite_tf(), self.n_steps)


# Shared transcription math: the textbook pieces every fixed-grid transcription
# needs (the trapezoidal quadrature of the running cost, the trapezoidal
# collocation defect, one RK4 step between input knots)


def trajectory_cost(cost, x, u, t, dt, params, *, discount_rate=0.0, t_start=0.0):
    """Sampled Bolza objective on a uniform grid, discounted from the horizon's start.

    ``x`` and ``u`` are knot matrices with shape ``(dim, N)``; the running
    cost is integrated with the trapezoid rule. The discount is the
    problem's, ``e^{−ρ(t − t_start)}``; ``ρ = 0`` is the plain sum.
    """
    xp = array_module(x, t)
    rho = discount_rate

    # w_k = e^{−ρ(t_k − t_start)}, the discount at each knot
    w = xp.exp(-rho * (t - t_start))

    # J = Σ dt/2 (w_k g_k + w_{k+1} g_{k+1}) + w_N h(x_N, t_N)
    g = w * running_cost_samples(cost, x, u, t, params)
    running = trapezoid_integral(g, dt)
    terminal = w[-1] * cost.h(x[:, -1], t[-1], params=params)
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


###############################################################################
# Internal machinery
###############################################################################


def program_backend_for_compile(backend: str) -> str:
    """Return the mathematical-program evaluator backend for a transcription."""
    key = normalize_backend(backend, allow_direct=True)
    return BACKEND_JAX if key == BACKEND_JAX else BACKEND_NUMPY


def dynamics_function(problem: PlanningProblem, backend: str, evaluator=None):
    """Return ``(x, u, t) -> f(x, u, t)`` for a transcription.

    ``"numpy"`` and ``"jax"`` use a compiled evaluator (``evaluator`` when
    given, the transcription's held one, else a new compile); system params from
    ``nominal_plant_params(problem)`` flow through the evaluator's ``params`` argument.
    ``"direct"`` calls ``system.f`` without compiling (escape hatch for
    systems that cannot be compiled).
    """
    key = normalize_backend(backend, allow_direct=True)
    params = nominal_plant_params(problem)

    # The escape hatch: the system's own f, uncompiled
    if key == BACKEND_DIRECT:
        return lambda x, u, t: problem.sys.f(x, u, t, params)

    # A compiled evaluator, on the params when given, else on the plant's numbers now
    if evaluator is None:
        evaluator = problem.sys.compile(backend=key)
    f = evaluator.f
    params = evaluator.call_params() if params is None else params

    return lambda x, u, t: f(x, u, t, params)


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


def stack_parametric(constraints):
    """One native constraint vector of ``(z, params)`` from a list of vector functions, ``None`` for none."""
    if not constraints:
        return None

    def stacked(z, params):
        values = native_concatenate(
            (constraint(z, params) for constraint in constraints), z
        )
        return values

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


def require_jax_compile_backend(backend):
    """Refuse a parametric program on any compile backend but JAX."""
    if normalize_backend(backend, allow_direct=True) != BACKEND_JAX:
        raise ValueError(
            f"a parametric program requires backend='jax' (got {backend!r})."
        )
