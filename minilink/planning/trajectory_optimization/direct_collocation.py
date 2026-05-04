"""Direct-collocation trajectory optimization."""

import numpy as np

from minilink.core.sets import BoxInputSet, BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.transcription import (
    ConstraintFunction,
    FixedGridOptions,
    Transcription,
    dynamics_function,
    stack_constraints,
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
        compile_backend: str | None = "numpy",
    ) -> MathematicalProgram:
        """Build the trapezoidal collocation nonlinear program."""
        if compile_backend == "jax":
            return self._transcribe_jax(problem, compile_backend=compile_backend)

        problem.require_cost()
        dynamics = dynamics_function(problem, compile_backend)

        equalities: list[ConstraintFunction] = [
            lambda z: self._dynamics_residual(z, problem, dynamics)
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
                "transcription": "direct_collocation",
                "compile_backend": compile_backend,
                "program_backend": "numpy",
            },
        )

    def _transcribe_jax(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str | None,
    ) -> MathematicalProgram:
        """Build a JAX-traceable collocation program."""
        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()

        t = jnp.asarray(self.options.t)
        dt = jnp.asarray(self.options.dt)
        n = int(problem.sys.n)
        m = int(problem.sys.m)
        n_steps = int(self.options.n_steps)
        params = problem.params.system

        def f(x, u, t_k):
            return problem.sys.f(x, u, t_k, params)

        def unpack_jax(z):
            split = n * n_steps
            x = z[:split].reshape(n, n_steps)
            u = z[split:].reshape(m, n_steps)
            return x, u

        def J(z):
            x, u = unpack_jax(z)
            running = jax.vmap(
                lambda x_k, u_k, t_k: cost.g(
                    x_k,
                    u_k,
                    t_k,
                    params=problem.params.cost,
                ),
                in_axes=(1, 1, 0),
            )(x, u, t)
            integral = jnp.sum(0.5 * dt * (running[:-1] + running[1:]))
            terminal = cost.h(x[:, -1], t[-1], params=problem.params.cost)
            return integral + terminal

        def dynamics_residual(z):
            x, u = unpack_jax(z)
            dx = jax.vmap(f, in_axes=(1, 1, 0), out_axes=1)(x, u, t)
            residuals = x[:, 1:] - x[:, :-1] - 0.5 * dt * (dx[:, :-1] + dx[:, 1:])
            return residuals.reshape(-1)

        equalities = [dynamics_residual]
        inequalities = []
        for boundary, index in ((problem.X0, 0), (problem.Xf, -1)):
            if boundary is None:
                continue
            t_i = t[index]
            if isinstance(boundary, SingletonSet):

                def boundary_residual(z, boundary=boundary, index=index):
                    x, _ = unpack_jax(z)
                    return boundary.residual(x[:, index])

                equalities.append(boundary_residual)
            else:

                def boundary_margin(z, boundary=boundary, index=index, t_i=t_i):
                    x, _ = unpack_jax(z)
                    return boundary.margin(
                        x[:, index],
                        t=t_i,
                        params=problem.params.sets,
                    )

                inequalities.append(boundary_margin)

        if problem.X is not None and not isinstance(problem.X, BoxSet):

            def state_margins(z):
                x, _ = unpack_jax(z)
                margins = jax.vmap(
                    lambda x_k, t_k: problem.X.margin(
                        x_k,
                        t=t_k,
                        params=problem.params.sets,
                    )
                )(x.T, t)
                return margins.reshape(-1)

            inequalities.append(state_margins)

        if problem.U is not None and not isinstance(problem.U, BoxInputSet):

            def input_margins(z):
                x, u = unpack_jax(z)
                margins = jax.vmap(
                    lambda u_k, x_k, t_k: problem.U.margin(
                        u_k,
                        x=x_k,
                        t=t_k,
                        params=problem.params.sets,
                    )
                )(u.T, x.T, t)
                return margins.reshape(-1)

            inequalities.append(input_margins)

        def h(z):
            return jnp.concatenate([residual(z).reshape(-1) for residual in equalities])

        def g(z):
            return jnp.concatenate([margin(z).reshape(-1) for margin in inequalities])

        lower, upper = self.decision_bounds(problem)
        return MathematicalProgram(
            n_z=self.decision_dimension(problem),
            J=J,
            h=h,
            g=g if inequalities else None,
            lower=lower,
            upper=upper,
            metadata={
                "transcription": "direct_collocation",
                "compile_backend": compile_backend,
                "program_backend": "jax",
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

    def pack_initial_guess(
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

    def decision_bounds(self, problem: PlanningProblem) -> tuple[np.ndarray, np.ndarray]:
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

        return lower, upper

    def _objective(self, z: np.ndarray, problem: PlanningProblem):
        cost = problem.require_cost()
        x, u = self.unpack(z, problem)
        t = self.options.t
        params = problem.params.cost

        total = 0.0
        for k in range(self.options.n_steps - 1):
            g0 = cost.g(x[:, k], u[:, k], float(t[k]), params=params)
            g1 = cost.g(x[:, k + 1], u[:, k + 1], float(t[k + 1]), params=params)
            total += 0.5 * self.options.dt * (g0 + g1)

        total += cost.h(x[:, -1], float(t[-1]), params=params)
        return total

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
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
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

                equalities.append(residual)
            else:

                def margin(z, boundary=boundary, index=index, t_i=t_i):
                    x, _ = self.unpack(z, problem)
                    return boundary.margin(
                        x[:, index],
                        t=t_i,
                        params=problem.params.sets,
                    )

                inequalities.append(margin)

    def _add_path_constraints(
        self,
        problem: PlanningProblem,
        *,
        inequalities: list[ConstraintFunction],
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

            inequalities.append(state_margins)

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

            inequalities.append(input_margins)
