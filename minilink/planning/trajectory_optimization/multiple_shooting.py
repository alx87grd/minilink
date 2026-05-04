"""Multiple-shooting trajectory optimization."""

import numpy as np

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
