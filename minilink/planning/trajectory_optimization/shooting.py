"""Single-shooting trajectory optimization."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from minilink.core.sets import BoxInputSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    EqualityConstraint,
    InequalityConstraint,
    MathematicalProgram,
    OptimizationResult,
    VariableBounds,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.transcription import (
    FixedGridOptions,
    Transcription,
    dynamics_function,
)


class ShootingOptions(FixedGridOptions):
    """Grid options for fixed-step single shooting."""


@dataclass
class ShootingTranscription(Transcription):
    """
    Fixed-grid single-shooting transcription.

    The decision vector contains input knots only:
    ``z = [u[0, :], ..., u[m-1, :]]``. States are reconstructed by RK4 rollout.
    """

    options: ShootingOptions

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        compile_backend: str | None = "numpy",
    ) -> MathematicalProgram:
        """Build the input-knot nonlinear program."""
        problem.require_cost()
        rollout = self._make_rollout(problem, compile_backend)
        equalities: list[EqualityConstraint] = []
        inequalities: list[InequalityConstraint] = []

        self._add_terminal_constraints(
            problem,
            rollout=rollout,
            equalities=equalities,
            inequalities=inequalities,
        )
        self._add_path_constraints(problem, rollout=rollout, inequalities=inequalities)

        return MathematicalProgram(
            J=lambda z: self._objective(z, problem, rollout),
            z0=self._pack_initial_guess(problem, initial_guess),
            bounds=self.variable_bounds(problem),
            equalities=tuple(equalities),
            inequalities=tuple(inequalities),
            metadata={
                "transcription": "shooting",
                "compile_backend": compile_backend,
            },
        )

    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        compile_backend: str | None = "numpy",
    ) -> Trajectory:
        """Roll out the optimized input sequence."""
        rollout = self._make_rollout(problem, compile_backend)
        dynamics = dynamics_function(problem, compile_backend)
        u = self.unpack(result.z, problem)
        x = rollout(u)
        dx = np.zeros_like(x)
        for k, t_k in enumerate(self.options.t):
            dx[:, k] = dynamics(x[:, k], u[:, k], float(t_k))

        traj = Trajectory(t=self.options.t, x=x, u=u, signals={"dx": dx})
        if problem.cost is not None:
            traj = problem.cost.evaluate_trajectory(traj, params=problem.params.cost)
        return traj

    def initial_guess_time_grid(self, problem: PlanningProblem) -> np.ndarray:
        """Return the shooting input-knot grid."""
        return self.options.t

    def decision_dimension(self, problem: PlanningProblem) -> int:
        """Return the input-only decision-vector dimension."""
        return int(problem.sys.m * self.options.n_steps)

    def pack(self, u: np.ndarray, problem: PlanningProblem) -> np.ndarray:
        """Pack sampled input knots into one decision vector."""
        return u.reshape(-1)

    def unpack(self, z: np.ndarray, problem: PlanningProblem) -> np.ndarray:
        """Unpack a decision vector into sampled input knots."""
        return z.reshape(problem.sys.m, self.options.n_steps)

    def variable_bounds(self, problem: PlanningProblem) -> VariableBounds:
        """Build box bounds for input-knot decision variables."""
        n_z = self.decision_dimension(problem)
        lower = np.full(n_z, -np.inf)
        upper = np.full(n_z, np.inf)
        if isinstance(problem.U, BoxInputSet):
            lower[:] = np.repeat(problem.U.box.lower, self.options.n_steps)
            upper[:] = np.repeat(problem.U.box.upper, self.options.n_steps)
        return VariableBounds(lower=lower, upper=upper)

    def _pack_initial_guess(
        self,
        problem: PlanningProblem,
        guess: np.ndarray | Trajectory | None,
    ) -> np.ndarray:
        if guess is None:
            raise ValueError("Shooting requires an initial input guess")

        if isinstance(guess, Trajectory):
            resampled = guess.resample(t_new=self.options.t)
            return self.pack(resampled.u, problem)

        return guess.reshape(-1)

    def _initial_state(self, problem: PlanningProblem) -> np.ndarray:
        if isinstance(problem.X0, SingletonSet):
            return problem.X0.point
        return problem.x_start

    def _make_rollout(
        self,
        problem: PlanningProblem,
        compile_backend: str | None,
    ) -> Callable[[np.ndarray], np.ndarray]:
        x0 = self._initial_state(problem)
        if (
            problem.params.system is not None
            or compile_backend is None
            or compile_backend == "direct"
        ):
            return lambda u: self._rollout_direct(problem, u, x0)

        evaluator = problem.sys.compile(backend=compile_backend, verbose=False)

        def rollout(u):
            x_samples = evaluator.rk4_rollout_forced(
                x0,
                u.T,
                float(self.options.t[0]),
                self.options.dt,
            )
            return np.asarray(x_samples).T

        return rollout

    def _rollout_direct(
        self,
        problem: PlanningProblem,
        u: np.ndarray,
        x0: np.ndarray,
    ) -> np.ndarray:
        n = int(problem.sys.n)
        n_steps = self.options.n_steps
        t = self.options.t
        dt = self.options.dt
        params = problem.params.system
        x = np.zeros((n, n_steps))
        x[:, 0] = x0
        for k in range(n_steps - 1):
            u0 = u[:, k]
            u1 = u[:, k + 1]
            umid = 0.5 * (u0 + u1)
            x_k = x[:, k]
            t_k = float(t[k])
            k1 = problem.sys.f(x_k, u0, t_k, params)
            k2 = problem.sys.f(
                x_k + 0.5 * dt * k1,
                umid,
                t_k + 0.5 * dt,
                params,
            )
            k3 = problem.sys.f(
                x_k + 0.5 * dt * k2,
                umid,
                t_k + 0.5 * dt,
                params,
            )
            k4 = problem.sys.f(
                x_k + dt * k3,
                u1,
                t_k + dt,
                params,
            )
            x[:, k + 1] = x_k + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        return x

    def _objective(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
        rollout: Callable[[np.ndarray], np.ndarray],
    ) -> float:
        cost = problem.require_cost()
        u = self.unpack(z, problem)
        x = rollout(u)
        t = self.options.t
        params = problem.params.cost

        total = 0.0
        for k in range(self.options.n_steps - 1):
            g0 = cost.g(x[:, k], u[:, k], float(t[k]), params=params)
            g1 = cost.g(x[:, k + 1], u[:, k + 1], float(t[k + 1]), params=params)
            total += 0.5 * self.options.dt * (g0 + g1)

        total += float(cost.h(x[:, -1], float(t[-1]), params=params))
        return float(total)

    def _add_terminal_constraints(
        self,
        problem: PlanningProblem,
        *,
        rollout: Callable[[np.ndarray], np.ndarray],
        equalities: list[EqualityConstraint],
        inequalities: list[InequalityConstraint],
    ) -> None:
        boundary = problem.Xf
        if boundary is None:
            return

        if isinstance(boundary, SingletonSet):

            def residual(z, boundary=boundary):
                x = rollout(self.unpack(z, problem))
                return boundary.residual(x[:, -1])

            equalities.append(EqualityConstraint(h=residual, name="terminal_state"))
            return

        def margin(z, boundary=boundary):
            x = rollout(self.unpack(z, problem))
            return boundary.margin(
                x[:, -1],
                t=float(self.options.t[-1]),
                params=problem.params.sets,
            )

        inequalities.append(InequalityConstraint(g=margin, name="terminal_state"))

    def _add_path_constraints(
        self,
        problem: PlanningProblem,
        *,
        rollout: Callable[[np.ndarray], np.ndarray],
        inequalities: list[InequalityConstraint],
    ) -> None:
        if problem.X is not None:

            def state_margins(z):
                x = rollout(self.unpack(z, problem))
                return np.concatenate(
                    [
                        problem.X.margin(
                            x[:, k],
                            t=float(t_k),
                            params=problem.params.sets,
                        ).reshape(-1)
                        for k, t_k in enumerate(self.options.t)
                    ]
                )

            inequalities.append(
                InequalityConstraint(g=state_margins, name="state_path")
            )

        if problem.U is not None and not isinstance(problem.U, BoxInputSet):

            def input_margins(z):
                u = self.unpack(z, problem)
                x = rollout(u)
                return np.concatenate(
                    [
                        problem.U.margin(
                            u[:, k],
                            x=x[:, k],
                            t=float(t_k),
                            params=problem.params.sets,
                        ).reshape(-1)
                        for k, t_k in enumerate(self.options.t)
                    ]
                )

            inequalities.append(
                InequalityConstraint(g=input_margins, name="input_path")
            )
