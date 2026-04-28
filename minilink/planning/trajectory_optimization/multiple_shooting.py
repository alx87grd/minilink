"""Multiple-shooting trajectory optimization."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    EqualityConstraint,
    MathematicalProgram,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.transcription import (
    FixedGridOptions,
)


class MultipleShootingOptions(FixedGridOptions):
    """Grid options for fixed-step multiple shooting."""


@dataclass
class MultipleShootingTranscription(DirectCollocationTranscription):
    """
    Fixed-grid multiple-shooting transcription.

    The decision vector is the same as direct collocation:
    ``z = [x[0, :], ..., x[n-1, :], u[0, :], ..., u[m-1, :]]``.
    Dynamics are enforced by RK4 shooting defects between neighboring knots.
    """

    options: MultipleShootingOptions

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        compile_backend: str | None = "numpy",
    ) -> MathematicalProgram:
        """Build the multiple-shooting nonlinear program."""
        problem.require_cost()
        step = self._make_step(problem, compile_backend)
        equalities = [
            EqualityConstraint(
                h=lambda z: self._dynamics_residual(z, problem, step),
                name="multiple_shooting_dynamics",
            )
        ]
        inequalities = []

        self._add_boundary_constraints(
            problem,
            equalities=equalities,
            inequalities=inequalities,
        )
        self._add_path_constraints(problem, inequalities=inequalities)

        return MathematicalProgram(
            J=lambda z: self._objective(z, problem),
            z0=self._pack_initial_guess(problem, initial_guess),
            bounds=self.variable_bounds(problem),
            equalities=tuple(equalities),
            inequalities=tuple(inequalities),
            metadata={
                "transcription": "multiple_shooting",
                "compile_backend": compile_backend,
            },
        )

    def _make_step(self, problem: PlanningProblem, compile_backend: str | None):
        params = problem.params.system
        if params is not None or compile_backend is None or compile_backend == "direct":
            def f(x, u, t):
                return problem.sys.f(x, u, t, params)
        else:
            evaluator = problem.sys.compile(backend=compile_backend, verbose=False)
            f = evaluator.f

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
        residuals = np.zeros((problem.sys.n, self.options.n_steps - 1))

        for k, t_k in enumerate(self.options.t[:-1]):
            x_next = step(x[:, k], u[:, k], u[:, k + 1], float(t_k))
            residuals[:, k] = x[:, k + 1] - x_next

        return residuals.reshape(-1)
