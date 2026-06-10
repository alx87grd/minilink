"""Multiple-shooting trajectory optimization."""

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY, normalize_backend
from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.transcription import (
    ConstraintFunction,
    FixedGridOptions,
    dynamics_function,
    native_concatenate,
    program_backend_for_compile,
    rk4_step_between_knots,
    stack_constraints,
    trajectory_cost,
)


class MultipleShootingOptions(FixedGridOptions):
    """Grid options for fixed-step multiple shooting."""


class MultipleShootingTranscription(DirectCollocationTranscription):
    """
    Fixed-grid multiple-shooting transcription.

    The decision vector is the same as direct collocation:
    ``z = [x[0, :], ..., x[n-1, :], u[0, :], ..., u[m-1, :]]``.
    Dynamics are enforced by RK4 shooting defects between neighboring knots.
    """

    def __init__(self, options: MultipleShootingOptions):
        self.options = options

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str = BACKEND_NUMPY,
    ) -> MathematicalProgram:
        """Build the multiple-shooting nonlinear program."""
        if normalize_backend(compile_backend, allow_direct=True) == BACKEND_JAX:
            return self._transcribe_jax(problem, compile_backend=compile_backend)

        problem.require_cost()
        step = self._make_step(problem, compile_backend)
        equalities: list[ConstraintFunction] = [
            lambda z: self._dynamics_residual(z, problem, step)
        ]
        inequalities: list[ConstraintFunction] = []

        self._add_boundary_constraints(
            problem,
            equalities=equalities,
            inequalities=inequalities,
        )
        self._add_path_constraints(problem, inequalities=inequalities)
        lower, upper = self.decision_bounds(problem)

        return MathematicalProgram(
            n_z=self.decision_dimension(problem),
            J=lambda z: self._objective(z, problem),
            h=stack_constraints(equalities),
            g=stack_constraints(inequalities),
            lower=lower,
            upper=upper,
            backend=program_backend_for_compile(compile_backend),
            metadata={
                "transcription": "multiple_shooting",
                "compile_backend": compile_backend,
            },
        )

    def _transcribe_jax(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str,
    ) -> MathematicalProgram:
        """Build a JAX-vectorized multiple-shooting program."""
        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()
        t = jnp.asarray(self.options.t)
        dt = jnp.asarray(self.options.dt)
        n = int(problem.sys.n)
        m = int(problem.sys.m)
        n_steps = int(self.options.n_steps)
        system_params = problem.params.system
        cost_params = problem.params.cost

        def unpack_jax(z):
            split = n * n_steps
            x = z[:split].reshape(n, n_steps)
            u = z[split:].reshape(m, n_steps)
            return x, u

        def f(x, u, t_k):
            return problem.sys.f(x, u, t_k, system_params)

        def rk4_step(x_k, u0, u1, t_k):
            return rk4_step_between_knots(f, x_k, u0, u1, t_k, dt)

        def J(z):
            x, u = unpack_jax(z)
            return trajectory_cost(cost, x, u, t, dt, cost_params)

        def dynamics_residual(z):
            x, u = unpack_jax(z)
            x_next = jax.vmap(rk4_step, in_axes=(1, 1, 1, 0), out_axes=1)(
                x[:, :-1],
                u[:, :-1],
                u[:, 1:],
                t[:-1],
            )
            return (x[:, 1:] - x_next).reshape(-1)

        equalities: list[ConstraintFunction] = [dynamics_residual]
        inequalities: list[ConstraintFunction] = []

        self._add_boundary_constraints(
            problem,
            equalities=equalities,
            inequalities=inequalities,
        )
        self._add_path_constraints(problem, inequalities=inequalities)
        lower, upper = self.decision_bounds(problem)

        return MathematicalProgram(
            n_z=self.decision_dimension(problem),
            J=J,
            h=stack_constraints(equalities),
            g=stack_constraints(inequalities),
            lower=lower,
            upper=upper,
            backend=BACKEND_JAX,
            metadata={
                "transcription": "multiple_shooting",
                "compile_backend": compile_backend,
            },
        )

    def _make_step(self, problem: PlanningProblem, compile_backend: str):
        f = dynamics_function(problem, compile_backend)
        dt = self.options.dt

        def step(x, u0, u1, t):
            return rk4_step_between_knots(f, x, u0, u1, t, dt)

        return step

    def _dynamics_residual(self, z: np.ndarray, problem: PlanningProblem, step):
        x, u = self.unpack(z, problem)
        residuals = []
        for k, t_k in enumerate(self.options.t[:-1]):
            x_next = step(x[:, k], u[:, k], u[:, k + 1], float(t_k))
            residuals.append(x[:, k + 1] - x_next)

        return native_concatenate(residuals, z)
