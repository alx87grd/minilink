"""Direct-collocation trajectory optimization."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

from minilink.core.costs import CostFunction
from minilink.core.sets import BoxInputSet, BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    EqualityConstraint,
    InequalityConstraint,
    MathematicalProgram,
    OptimizationResult,
    VariableBounds,
)
from minilink.optimization.optimizers.optimizer import Optimizer
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.transcription import Transcription


@dataclass(frozen=True)
class DirectCollocationOptions:
    """
    Grid and optimizer options for direct-collocation transcriptions.

    These are solver options, not fields on :class:`PlanningProblem`.
    """

    tf: float
    n_steps: int
    compile_backend: str | None = "numpy"
    optimizer: Optimizer = field(default_factory=ScipyMinimizeOptimizer)
    initial_guess: np.ndarray | Trajectory | None = None

    def __post_init__(self) -> None:
        tf = float(self.tf)
        if not np.isfinite(tf) or tf <= 0.0:
            raise ValueError("tf must be a positive finite scalar")
        if isinstance(self.n_steps, bool) or int(self.n_steps) < 2:
            raise ValueError("n_steps must be an integer greater than or equal to 2")
        object.__setattr__(self, "tf", tf)
        object.__setattr__(self, "n_steps", int(self.n_steps))
        if self.compile_backend is not None:
            object.__setattr__(self, "compile_backend", str(self.compile_backend))

    @property
    def t(self) -> np.ndarray:
        """Uniform transcription time grid."""
        return np.linspace(0.0, self.tf, self.n_steps)

    @property
    def dt(self) -> float:
        """Uniform time step between collocation knots."""
        return self.tf / float(self.n_steps - 1)


@dataclass(frozen=True)
class DirectCollocationTranscription(Transcription):
    """
    Fixed-time-grid trapezoidal direct-collocation transcription.

    The decision vector packs every sampled state followed by every sampled
    input, both in ``(dim, N)`` row-major order:

    ``z = [x[0, :], ..., x[n-1, :], u[0, :], ..., u[m-1, :]]``.
    """

    options: DirectCollocationOptions

    def transcribe(self, problem: PlanningProblem) -> MathematicalProgram:
        """Convert ``problem`` into a finite-dimensional nonlinear program."""
        problem.require_cost()
        dynamics = self._make_dynamics(problem)

        equalities = [
            EqualityConstraint(
                h=lambda z: self._dynamics_residual(z, problem, dynamics),
                name="direct_collocation_dynamics",
                metadata={"scheme": "trapezoidal"},
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
            z0=self.initial_guess(problem),
            bounds=self.variable_bounds(problem),
            equalities=tuple(equalities),
            inequalities=tuple(inequalities),
            metadata={
                "transcription": "direct_collocation",
                "integration_scheme": "trapezoidal",
                "tf": self.options.tf,
                "dt": self.options.dt,
                "n_steps": self.options.n_steps,
                "state_dim": int(problem.sys.n),
                "input_dim": int(problem.sys.m),
            },
        )

    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        metadata: dict[str, Any] | None = None,
    ) -> Trajectory:
        """Convert an optimizer result back into a sampled trajectory."""
        x, u = self.unpack(result.z, problem)
        dynamics = self._make_dynamics(problem)
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

    def pack(self, x: np.ndarray, u: np.ndarray, problem: PlanningProblem) -> np.ndarray:
        """Pack sampled state and input matrices into one decision vector."""
        n = int(problem.sys.n)
        m = int(problem.sys.m)
        n_steps = self.options.n_steps
        x_arr = np.asarray(x, dtype=float)
        u_arr = np.asarray(u, dtype=float)
        if x_arr.shape != (n, n_steps):
            raise ValueError(f"x must have shape ({n}, {n_steps})")
        if u_arr.shape != (m, n_steps):
            raise ValueError(f"u must have shape ({m}, {n_steps})")
        return np.concatenate((x_arr.reshape(-1), u_arr.reshape(-1)))

    def unpack(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Unpack a decision vector into sampled ``(x, u)`` matrices."""
        n = int(problem.sys.n)
        m = int(problem.sys.m)
        n_steps = self.options.n_steps
        z_arr = np.asarray(z, dtype=float).reshape(-1)
        expected = self.decision_dimension(problem)
        if z_arr.size != expected:
            raise ValueError(f"z must have shape ({expected},)")
        split = n * n_steps
        x = z_arr[:split].reshape(n, n_steps)
        u = z_arr[split:].reshape(m, n_steps)
        return x, u

    def initial_guess(self, problem: PlanningProblem) -> np.ndarray:
        """Return a packed initial guess for the mathematical program."""
        guess = self.options.initial_guess
        if guess is None:
            raise ValueError(
                "DirectCollocationTranscription requires an initial guess prepared "
                "by the planner"
            )

        if isinstance(guess, Trajectory):
            resampled = guess.resample(t_new=self.options.t)
            return self.pack(resampled.x, resampled.u, problem)

        guess_arr = np.asarray(guess, dtype=float).reshape(-1)
        expected = self.decision_dimension(problem)
        if guess_arr.shape != (expected,):
            raise ValueError(f"initial_guess must have shape ({expected},)")
        return guess_arr.copy()

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

    def _make_dynamics(self, problem: PlanningProblem):
        params = problem.params.system
        backend = self.options.compile_backend

        if params is not None or backend is None or backend == "direct":
            return lambda x, u, t: np.asarray(
                problem.sys.f(x, u, t, params), dtype=float
            ).reshape(problem.sys.n)

        evaluator = problem.sys.compile(backend=backend, verbose=False)
        return lambda x, u, t: np.asarray(evaluator.f(x, u, t), dtype=float).reshape(
            problem.sys.n
        )

    def _objective(self, z: np.ndarray, problem: PlanningProblem) -> float:
        cost = problem.require_cost()
        x, u = self.unpack(z, problem)
        t = self.options.t
        params = problem.params.cost

        total = 0.0
        for k in range(self.options.n_steps - 1):
            g0 = cost.g(x[:, k], u[:, k], float(t[k]), params=params)
            g1 = cost.g(x[:, k + 1], u[:, k + 1], float(t[k + 1]), params=params)
            total += 0.5 * self.options.dt * (float(g0) + float(g1))

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
                x, u = self.unpack(z, problem)
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


def _with_default_initial_guess(
    problem: PlanningProblem,
    options: DirectCollocationOptions,
) -> DirectCollocationOptions:
    if options.initial_guess is not None:
        return options
    return replace(
        options,
        initial_guess=default_initial_trajectory(problem, options.t),
    )


class _DirectCollocationPlannerBase(Planner):
    """Shared orchestration for direct-collocation planner variants."""

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        options: DirectCollocationOptions,
        transcription_cls: type[DirectCollocationTranscription],
    ) -> None:
        super().__init__(problem)
        self.require_cost()
        self.options = _with_default_initial_guess(problem, options)
        self.transcription = transcription_cls(self.options)
        self.last_program: MathematicalProgram | None = None
        self.last_optimization_result: OptimizationResult | None = None

    def compute_solution(self) -> Trajectory:
        """Compute and store a direct-collocation trajectory."""
        program = self.transcription.transcribe(self.problem)
        optimization_result = self.options.optimizer.solve(program)
        trajectory = self.transcription.reconstruct_result(
            optimization_result,
            problem=self.problem,
            metadata=program.metadata,
        )

        self.last_program = program
        self.last_optimization_result = optimization_result
        return self._store_result(trajectory)


class DirectCollocationPlanner(_DirectCollocationPlannerBase):
    """
    Direct-collocation trajectory-optimization planner.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        tf: float,
        n_steps: int,
        optimizer: Optimizer | None = None,
        compile_backend: str | None = "numpy",
        initial_guess: np.ndarray | Trajectory | None = None,
    ) -> None:
        options = DirectCollocationOptions(
            tf=tf,
            n_steps=n_steps,
            compile_backend=compile_backend,
            optimizer=(ScipyMinimizeOptimizer() if optimizer is None else optimizer),
            initial_guess=initial_guess,
        )
        super().__init__(
            problem,
            options=options,
            transcription_cls=DirectCollocationTranscription,
        )

    @classmethod
    def from_system(
        cls,
        sys: Any,
        *,
        x_start: np.ndarray,
        x_goal: np.ndarray,
        cost: CostFunction,
        tf: float,
        n_steps: int,
        optimizer: Optimizer | None = None,
        compile_backend: str | None = "numpy",
        initial_guess: np.ndarray | Trajectory | None = None,
    ) -> DirectCollocationPlanner:
        """Convenience constructor building a :class:`PlanningProblem`."""
        problem = PlanningProblem(
            sys=sys,
            x_start=x_start,
            x_goal=x_goal,
            cost=cost,
        )
        return cls(
            problem,
            tf=tf,
            n_steps=n_steps,
            optimizer=optimizer,
            compile_backend=compile_backend,
            initial_guess=initial_guess,
        )
