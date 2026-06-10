"""
Transcription contracts for trajectory optimization.

A transcription maps a continuous planning problem to a finite-dimensional
mathematical program, then reconstructs a trajectory from the optimizer
output. The shared math helpers below implement the textbook pieces every
fixed-grid transcription needs:

- trapezoidal quadrature of the running cost (Bolza objective),
- the trapezoidal collocation defect,
- one RK4 step between input knots.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import (
    BACKEND_DIRECT,
    BACKEND_JAX,
    BACKEND_NUMPY,
    array_module,
    normalize_backend,
)
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.planning.problems import PlanningProblem

ConstraintFunction = Callable[[np.ndarray], np.ndarray]


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

    if key == BACKEND_DIRECT:
        return lambda x, u, t: problem.sys.f(x, u, t, params)

    evaluator = problem.sys.compile(backend=key, verbose=False)
    if params is None:
        return lambda x, u, t: evaluator.f(x, u, t)
    return lambda x, u, t: evaluator.f_p(x, u, t, params)


def native_concatenate(values, like):
    """Concatenate vector pieces with the array module used by ``like``."""
    xp = array_module(like)
    pieces = []
    for value in values:
        pieces.append(value.reshape(-1))

    if not pieces:
        return xp.array([])

    return xp.concatenate(pieces)


# Shared Transcription Math


def trajectory_cost(cost, x, u, t, dt, params):
    """Sampled Bolza objective on a uniform grid.

        J = sum_k dt/2 (g_k + g_{k+1}) + h(x_N, t_N)

    ``x`` and ``u`` are knot matrices with shape ``(dim, N)``; the running
    cost is integrated with the trapezoid rule.
    """
    running = running_cost_samples(cost, x, u, t, params)
    return trapezoid_integral(running, dt) + cost.h(x[:, -1], t[-1], params=params)


def running_cost_samples(cost, x, u, t, params):
    """Sample the running cost ``g(x_k, u_k, t_k)`` at every knot.

    NumPy inputs use a plain loop; JAX inputs vectorize with ``vmap`` so the
    whole-program jit stays compact.
    """
    if array_module(x) is np:
        return np.array(
            [
                cost.g(x[:, k], u[:, k], float(t[k]), params=params)
                for k in range(x.shape[1])
            ]
        )

    import jax

    return jax.vmap(
        lambda x_k, u_k, t_k: cost.g(x_k, u_k, t_k, params=params),
        in_axes=(1, 1, 0),
    )(x, u, t)


def trapezoid_integral(values, dt):
    """Trapezoid rule on a uniform grid: ``∫ v dt ≈ Σ dt/2 (v_k + v_{k+1})``."""
    xp = array_module(values)
    return xp.sum(0.5 * dt * (values[:-1] + values[1:]))


def trapezoidal_defect(x, dx, dt):
    """Trapezoidal collocation defect between neighboring knots.

        d_k = x_{k+1} - x_k - dt/2 (f_k + f_{k+1})

    ``x`` and ``dx`` have shape ``(n, N)``; the result has shape ``(n, N-1)``.
    """
    return x[:, 1:] - x[:, :-1] - 0.5 * dt * (dx[:, :-1] + dx[:, 1:])


def rk4_step_between_knots(f, x, u0, u1, t, dt):
    """One RK4 step from knot ``k`` to ``k+1`` with linear input interpolation."""
    umid = 0.5 * (u0 + u1)

    k1 = f(x, u0, t)
    k2 = f(x + 0.5 * dt * k1, umid, t + 0.5 * dt)
    k3 = f(x + 0.5 * dt * k2, umid, t + 0.5 * dt)
    k4 = f(x + dt * k3, u1, t + dt)

    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


@dataclass
class FixedGridOptions:
    """Uniform fixed-time-grid options."""

    tf: float
    n_steps: int

    def __post_init__(self) -> None:
        tf = float(self.tf)
        if not np.isfinite(tf) or tf <= 0.0:
            raise ValueError("tf must be positive and finite")
        if self.n_steps < 2:
            raise ValueError("n_steps must be at least 2")
        self.tf = tf
        self.n_steps = int(self.n_steps)

    @property
    def t(self) -> np.ndarray:
        return np.linspace(0.0, self.tf, self.n_steps)

    @property
    def dt(self) -> float:
        return self.tf / (self.n_steps - 1)


class Transcription(ABC):
    """
    Base class for finite-dimensional trajectory transcriptions.
    """

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


def stack_constraints(
    constraints: list[ConstraintFunction],
) -> ConstraintFunction | None:
    """Return one native-array constraint vector from a list of vector functions."""
    if not constraints:
        return None

    def stacked(z: np.ndarray) -> np.ndarray:
        xp = array_module(z)
        values = []
        for constraint in constraints:
            values.append(constraint(z).reshape(-1))
        return xp.concatenate(values)

    return stacked
