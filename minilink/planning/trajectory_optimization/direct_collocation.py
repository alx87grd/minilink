"""Direct-collocation trajectory optimization."""

import numpy as np

from minilink.core.sets import BoxInputSet, BoxSet, SingletonSet
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


class DirectCollocationOptions(FixedGridOptions):
    """
    Grid options for direct-collocation transcriptions.
    """


class DirectCollocationTranscription(Transcription):
    """
    Fixed-time-grid trapezoidal direct-collocation transcription.

    The decision vector packs every sampled state followed by every sampled
    input, both in ``(dim, N)`` row-major order:

    ``z = [x[0, :], ..., x[n-1, :], u[0, :], ..., u[m-1, :]]``.
    """

    def __init__(self, options: DirectCollocationOptions):
        self.options = options

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        compile_backend: str | None = "numpy",
    ) -> MathematicalProgram:
        """Build the trapezoidal collocation nonlinear program."""
        problem.require_cost()
        dynamics = dynamics_function(problem, compile_backend)

        equalities = [
            EqualityConstraint(
                h=lambda z: self._dynamics_residual(z, problem, dynamics),
                name="direct_collocation_dynamics",
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
                "transcription": "direct_collocation",
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
        """Read ``(x, u)`` from the optimizer result."""
        x, u = self.unpack(result.z, problem)
        dynamics = dynamics_function(problem, compile_backend)
        dx = np.zeros_like(x)
        for k, t_k in enumerate(self.options.t):
            dx[:, k] = dynamics(x[:, k], u[:, k], float(t_k))

        traj = Trajectory(
            t=self.options.t,
            x=x,
            u=u,
            signals={
                "dx": dx,
            },
        )
        if problem.cost is not None:
            traj = problem.cost.evaluate_trajectory(traj, params=problem.params.cost)
        return traj

    def decision_dimension(self, problem: PlanningProblem) -> int:
        """Return the packed decision-vector dimension for ``problem``."""
        return int((problem.sys.n + problem.sys.m) * self.options.n_steps)

    def pack(
        self, x: np.ndarray, u: np.ndarray, problem: PlanningProblem
    ) -> np.ndarray:
        """Pack sampled state and input matrices into one decision vector."""
        return np.concatenate((x.reshape(-1), u.reshape(-1)))

    def unpack(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Unpack a decision vector into sampled ``(x, u)`` matrices."""
        n = int(problem.sys.n)
        m = int(problem.sys.m)
        n_steps = self.options.n_steps
        split = n * n_steps
        x = z[:split].reshape(n, n_steps)
        u = z[split:].reshape(m, n_steps)
        return x, u

    def _pack_initial_guess(
        self,
        problem: PlanningProblem,
        guess: np.ndarray | Trajectory | None,
    ) -> np.ndarray:
        """Return a packed initial guess for the mathematical program."""
        if guess is None:
            raise ValueError("Direct collocation requires an initial guess")

        if isinstance(guess, Trajectory):
            resampled = guess.resample(t_new=self.options.t)
            return self.pack(resampled.x, resampled.u, problem)

        return guess.reshape(-1)

    def initial_guess_time_grid(self, problem: PlanningProblem) -> np.ndarray:
        """Return the collocation time grid."""
        return self.options.t

    def variable_bounds(self, problem: PlanningProblem) -> VariableBounds:
        """Build box bounds on the packed decision vector when sets expose boxes."""
        n = int(problem.sys.n)
        n_steps = self.options.n_steps
        n_z = self.decision_dimension(problem)

        lower = np.full(n_z, -np.inf)
        upper = np.full(n_z, np.inf)

        if isinstance(problem.X, BoxSet):
            lower[: n * n_steps] = np.repeat(problem.X.lower, n_steps)
            upper[: n * n_steps] = np.repeat(problem.X.upper, n_steps)

        if isinstance(problem.U, BoxInputSet):
            start = n * n_steps
            lower[start:] = np.repeat(problem.U.box.lower, n_steps)
            upper[start:] = np.repeat(problem.U.box.upper, n_steps)

        return VariableBounds(lower=lower, upper=upper)

    def _objective(self, z: np.ndarray, problem: PlanningProblem) -> float:
        cost = problem.require_cost()
        x, u = self.unpack(z, problem)
        t = self.options.t
        params = problem.params.cost

        total = 0.0
        for k in range(self.options.n_steps - 1):
            g0 = cost.g(x[:, k], u[:, k], float(t[k]), params=params)
            g1 = cost.g(x[:, k + 1], u[:, k + 1], float(t[k + 1]), params=params)
            total += 0.5 * self.options.dt * (g0 + g1)

        total += float(cost.h(x[:, -1], float(t[-1]), params=params))
        return float(total)

    def _dynamics_residual(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
        dynamics,
    ) -> np.ndarray:
        x, u = self.unpack(z, problem)
        t = self.options.t
        residuals = np.zeros((problem.sys.n, self.options.n_steps - 1))

        for k in range(self.options.n_steps - 1):
            f0 = dynamics(x[:, k], u[:, k], float(t[k]))
            f1 = dynamics(x[:, k + 1], u[:, k + 1], float(t[k + 1]))
            dx_num = x[:, k + 1] - x[:, k]
            dx_col = 0.5 * self.options.dt * (f0 + f1)
            residuals[:, k] = dx_num - dx_col

        return residuals.reshape(-1)

    def _add_boundary_constraints(
        self,
        problem: PlanningProblem,
        *,
        equalities: list[EqualityConstraint],
        inequalities: list[InequalityConstraint],
    ) -> None:
        for name, boundary, index in (
            ("initial_state", problem.X0, 0),
            ("terminal_state", problem.Xf, -1),
        ):
            if boundary is None:
                continue
            t_i = float(self.options.t[index])

            if isinstance(boundary, SingletonSet):

                def residual(z, boundary=boundary, index=index):
                    x, _ = self.unpack(z, problem)
                    return boundary.residual(x[:, index])

                equalities.append(EqualityConstraint(h=residual, name=name))
            else:

                def margin(z, boundary=boundary, index=index, t_i=t_i):
                    x, _ = self.unpack(z, problem)
                    return boundary.margin(
                        x[:, index],
                        t=t_i,
                        params=problem.params.sets,
                    )

                inequalities.append(InequalityConstraint(g=margin, name=name))

    def _add_path_constraints(
        self,
        problem: PlanningProblem,
        *,
        inequalities: list[InequalityConstraint],
    ) -> None:
        if problem.X is not None and not isinstance(problem.X, BoxSet):

            def state_margins(z):
                x, _ = self.unpack(z, problem)
                margins = []
                for k, t_k in enumerate(self.options.t):
                    margin = problem.X.margin(
                        x[:, k],
                        t=float(t_k),
                        params=problem.params.sets,
                    )
                    margins.append(margin.reshape(-1))
                return np.concatenate(margins)

            inequalities.append(
                InequalityConstraint(g=state_margins, name="state_path")
            )

        if problem.U is not None and not isinstance(problem.U, BoxInputSet):

            def input_margins(z):
                x, u = self.unpack(z, problem)
                margins = []
                for k, t_k in enumerate(self.options.t):
                    margin = problem.U.margin(
                        u[:, k],
                        x=x[:, k],
                        t=float(t_k),
                        params=problem.params.sets,
                    )
                    margins.append(margin.reshape(-1))
                return np.concatenate(margins)

            inequalities.append(
                InequalityConstraint(g=input_margins, name="input_path")
            )
