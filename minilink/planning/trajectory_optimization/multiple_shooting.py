"""Multiple-shooting trajectory optimization."""

import numpy as np

from minilink.compile.backend_policy import BACKEND_JAX
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
    stack_constraints,
    transcription_backend_key,
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
        compile_backend: str | None = "numpy",
    ) -> MathematicalProgram:
        """Build the multiple-shooting nonlinear program."""
        if transcription_backend_key(compile_backend) == BACKEND_JAX:
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
            metadata={
                "transcription": "multiple_shooting",
                "compile_backend": compile_backend,
                "program_backend": program_backend_for_compile(compile_backend),
            },
        )

    def _transcribe_jax(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str | None,
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

        def rk4_step(x_k, u0, u1, t_k):
            umid = 0.5 * (u0 + u1)
            k1 = problem.sys.f(x_k, u0, t_k, system_params)
            k2 = problem.sys.f(
                x_k + 0.5 * dt * k1,
                umid,
                t_k + 0.5 * dt,
                system_params,
            )
            k3 = problem.sys.f(
                x_k + 0.5 * dt * k2,
                umid,
                t_k + 0.5 * dt,
                system_params,
            )
            k4 = problem.sys.f(
                x_k + dt * k3,
                u1,
                t_k + dt,
                system_params,
            )
            return x_k + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        def J(z):
            x, u = unpack_jax(z)
            running = jax.vmap(
                lambda x_k, u_k, t_k: cost.g(
                    x_k,
                    u_k,
                    t_k,
                    params=cost_params,
                ),
                in_axes=(1, 1, 0),
            )(x, u, t)
            integral = jnp.sum(0.5 * dt * (running[:-1] + running[1:]))
            terminal = cost.h(x[:, -1], t[-1], params=cost_params)
            return integral + terminal

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
            metadata={
                "transcription": "multiple_shooting",
                "compile_backend": compile_backend,
                "program_backend": BACKEND_JAX,
            },
        )

    def _make_step(self, problem: PlanningProblem, compile_backend: str | None):
        f = dynamics_function(problem, compile_backend)

        def step(x, u0, u1, t):
            dt = self.options.dt
            umid = 0.5 * (u0 + u1)
            k1 = f(x, u0, t)
            k2 = f(x + 0.5 * dt * k1, umid, t + 0.5 * dt)
            k3 = f(x + 0.5 * dt * k2, umid, t + 0.5 * dt)
            k4 = f(x + dt * k3, u1, t + dt)
            return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        return step

    def _dynamics_residual(self, z: np.ndarray, problem: PlanningProblem, step):
        x, u = self.unpack(z, problem)
        residuals = []
        for k, t_k in enumerate(self.options.t[:-1]):
            x_next = step(x[:, k], u[:, k], u[:, k + 1], float(t_k))
            residuals.append(x[:, k + 1] - x_next)

        return native_concatenate(residuals, z)
