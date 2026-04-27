"""Single-shooting trajectory optimization."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

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
from minilink.planning.trajectory_optimization.fixed_grid import FixedGridOptions
from minilink.planning.trajectory_optimization.transcription import (
    Transcription,
    TranscriptionContext,
)


class ShootingOptions(FixedGridOptions):
    """Grid options for fixed-step single shooting."""


@dataclass(frozen=True)
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
        context: TranscriptionContext | None = None,
    ) -> MathematicalProgram:
        """Convert ``problem`` into an input-only nonlinear program."""
        problem.require_cost()
        context = TranscriptionContext() if context is None else context
        rollout = self._make_rollout(problem, context)
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
                "integration_scheme": "rk4",
                "tf": self.options.tf,
                "dt": self.options.dt,
                "n_steps": self.options.n_steps,
                "state_dim": int(problem.sys.n),
                "input_dim": int(problem.sys.m),
                "compile_backend": context.compile_backend,
            },
        )

    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        metadata: dict[str, Any] | None = None,
        context: TranscriptionContext | None = None,
    ) -> Trajectory:
        """Roll out the optimized input sequence into a trajectory."""
        context = TranscriptionContext() if context is None else context
        rollout = self._make_rollout(problem, context)
        dynamics = self._make_dynamics(problem, context)
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
        m = int(problem.sys.m)
        n_steps = self.options.n_steps
        u_arr = np.asarray(u, dtype=float)
        if u_arr.shape != (m, n_steps):
            raise ValueError(f"u must have shape ({m}, {n_steps})")
        return u_arr.reshape(-1)

    def unpack(self, z: np.ndarray, problem: PlanningProblem) -> np.ndarray:
        """Unpack a decision vector into sampled input knots."""
        m = int(problem.sys.m)
        n_steps = self.options.n_steps
        z_arr = np.asarray(z, dtype=float).reshape(-1)
        expected = self.decision_dimension(problem)
        if z_arr.size != expected:
            raise ValueError(f"z must have shape ({expected},)")
        return z_arr.reshape(m, n_steps)

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
            raise ValueError(
                "ShootingTranscription requires an initial guess prepared by the planner"
            )

        if isinstance(guess, Trajectory):
            resampled = guess.resample(t_new=self.options.t)
            return self.pack(resampled.u, problem)

        guess_arr = np.asarray(guess, dtype=float).reshape(-1)
        expected = self.decision_dimension(problem)
        if guess_arr.shape != (expected,):
            raise ValueError(f"initial_guess must have shape ({expected},)")
        return guess_arr.copy()

    def _initial_state(self, problem: PlanningProblem) -> np.ndarray:
        if isinstance(problem.X0, SingletonSet):
            return problem.X0.point.copy()
        return np.asarray(problem.x_start, dtype=float).reshape(problem.sys.n).copy()

    def _make_rollout(
        self,
        problem: PlanningProblem,
        context: TranscriptionContext,
    ) -> Callable[[np.ndarray], np.ndarray]:
        x0 = self._initial_state(problem)
        backend = context.compile_backend
        if problem.params.system is not None or backend is None or backend == "direct":
            return lambda u: self._rollout_direct(problem, u, x0)

        evaluator = problem.sys.compile(backend=backend, verbose=False)

        def rollout(u):
            x_samples = evaluator.rk4_rollout_forced(
                x0,
                np.asarray(u, dtype=float).T,
                float(self.options.t[0]),
                self.options.dt,
            )
            return np.asarray(x_samples, dtype=float).T

        return rollout

    def _make_dynamics(
        self,
        problem: PlanningProblem,
        context: TranscriptionContext,
    ) -> Callable[[np.ndarray, np.ndarray, float], np.ndarray]:
        backend = context.compile_backend
        params = problem.params.system
        if params is not None or backend is None or backend == "direct":
            return lambda x, u, t: np.asarray(
                problem.sys.f(x, u, t, params), dtype=float
            ).reshape(problem.sys.n)

        evaluator = problem.sys.compile(backend=backend, verbose=False)
        return lambda x, u, t: np.asarray(evaluator.f(x, u, t), dtype=float).reshape(
            problem.sys.n
        )

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
            k1 = np.asarray(problem.sys.f(x_k, u0, t_k, params), dtype=float).reshape(
                n
            )
            k2 = np.asarray(
                problem.sys.f(x_k + 0.5 * dt * k1, umid, t_k + 0.5 * dt, params),
                dtype=float,
            ).reshape(n)
            k3 = np.asarray(
                problem.sys.f(x_k + 0.5 * dt * k2, umid, t_k + 0.5 * dt, params),
                dtype=float,
            ).reshape(n)
            k4 = np.asarray(
                problem.sys.f(x_k + dt * k3, u1, t_k + dt, params),
                dtype=float,
            ).reshape(n)
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
            total += 0.5 * self.options.dt * (float(g0) + float(g1))

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
