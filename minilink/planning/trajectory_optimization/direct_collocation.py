"""Direct-collocation trajectory optimization."""

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY, normalize_backend
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
    native_concatenate,
    program_backend_for_compile,
    stack_constraints,
    trajectory_cost,
    trapezoidal_defect,
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
        compile_backend: str = BACKEND_NUMPY,
    ) -> MathematicalProgram:
        """Build the trapezoidal collocation nonlinear program."""
        if normalize_backend(compile_backend, allow_direct=True) == BACKEND_JAX:
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
            backend=program_backend_for_compile(compile_backend),
            metadata={
                "transcription": "direct_collocation",
                "compile_backend": compile_backend,
            },
        )

    def _transcribe_jax(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str,
    ) -> MathematicalProgram:
        """Build a JAX-vectorized collocation program.

        Direct collocation is performance-sensitive because SLSQP repeatedly
        calls the objective, gradient, constraints, and constraint Jacobians.
        The whole mathematical program is already JIT/autodiff compiled by the
        JAX program evaluator, so this path uses raw traceable equations and
        vectorized knot evaluation instead of nesting precompiled dynamics
        evaluators inside the program.
        """
        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()
        t = jnp.asarray(self.options.t(problem))
        dt = jnp.asarray(self.options.dt(problem))
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

        def J(z):
            x, u = unpack_jax(z)
            return trajectory_cost(cost, x, u, t, dt, cost_params)

        def dynamics_residual(z):
            x, u = unpack_jax(z)
            dx = jax.vmap(
                lambda x_k, u_k, t_k: problem.sys.f(
                    x_k,
                    u_k,
                    t_k,
                    system_params,
                ),
                in_axes=(1, 1, 0),
                out_axes=1,
            )(x, u, t)
            return trapezoidal_defect(x, dx, dt).reshape(-1)

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
                "transcription": "direct_collocation",
                "compile_backend": compile_backend,
            },
        )

    def transcribe_parametric(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str = BACKEND_JAX,
    ):
        """Build a JAX parametric collocation program with runtime ``x0``.

        Equality constraints are ``h(z, x0)``: dynamics / terminal (fixed) plus
        the initial-state residual ``x[:, 0] - x0``. Requires singleton ``X0``.
        """
        from minilink.planning.trajectory_optimization.parametric_program import (
            ParametricMathematicalProgram,
        )

        if normalize_backend(compile_backend, allow_direct=True) != BACKEND_JAX:
            raise ValueError(
                "Parametric direct collocation currently supports "
                "compile_backend='jax' only."
            )
        if not isinstance(problem.X0, SingletonSet):
            raise ValueError(
                "Parametric direct collocation requires a singleton initial "
                "boundary X0."
            )

        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()
        t = jnp.asarray(self.options.t(problem))
        dt = jnp.asarray(self.options.dt(problem))
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

        def J(z):
            x, u = unpack_jax(z)
            return trajectory_cost(cost, x, u, t, dt, cost_params)

        def dynamics_residual(z):
            x, u = unpack_jax(z)
            dx = jax.vmap(
                lambda x_k, u_k, t_k: problem.sys.f(
                    x_k,
                    u_k,
                    t_k,
                    system_params,
                ),
                in_axes=(1, 1, 0),
                out_axes=1,
            )(x, u, t)
            return trapezoidal_defect(x, dx, dt).reshape(-1)

        def initial_residual(z, x0):
            x, _ = unpack_jax(z)
            return (x[:, 0] - x0).reshape(-1)

        fixed_equalities: list[ConstraintFunction] = [dynamics_residual]
        inequalities: list[ConstraintFunction] = []

        self._add_terminal_constraints(
            problem,
            equalities=fixed_equalities,
            inequalities=inequalities,
        )
        self._add_path_constraints(problem, inequalities=inequalities)
        lower, upper = self.decision_bounds(problem)

        fixed_h = stack_constraints(fixed_equalities)

        def h(z, x0):
            parts = [fixed_h(z), initial_residual(z, x0)]
            return jnp.concatenate(parts)

        return ParametricMathematicalProgram(
            n_z=self.decision_dimension(problem),
            n_x0=n,
            J=J,
            h=h,
            g=stack_constraints(inequalities),
            lower=lower,
            upper=upper,
            backend=BACKEND_JAX,
            metadata={
                "transcription": "direct_collocation_parametric",
                "compile_backend": compile_backend,
            },
        )

    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        compile_backend: str = BACKEND_NUMPY,
        dynamics=None,
    ) -> Trajectory:
        """Read ``(x, u)`` from the optimizer result.

        Prefer a cached ``dynamics`` callable (parametric compile path);
        otherwise build one via ``compile_backend``.
        """
        x, u = self.unpack(result.z, problem)
        # Public trajectories are NumPy — convert off any JAX optimizer arrays here.
        x = np.asarray(x, dtype=float)
        u = np.asarray(u, dtype=float)
        t = np.asarray(self.options.t(problem), dtype=float)
        if dynamics is None:
            dynamics = dynamics_function(problem, compile_backend)
        dx = np.zeros_like(x)
        for k, t_k in enumerate(t):
            dx[:, k] = np.asarray(dynamics(x[:, k], u[:, k], float(t_k)), dtype=float)

        traj = Trajectory(
            t=t,
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
            resampled = guess.resample(t_new=self.options.t(problem))
            return self.pack(resampled.x, resampled.u, problem)

        return guess.reshape(-1)

    def initial_guess_time_grid(self, problem: PlanningProblem) -> np.ndarray:
        """Return the collocation time grid."""
        return self.options.t(problem)

    def decision_bounds(
        self, problem: PlanningProblem
    ) -> tuple[np.ndarray, np.ndarray]:
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
        return trajectory_cost(
            cost,
            x,
            u,
            self.options.t(problem),
            self.options.dt(problem),
            problem.params.cost,
        )

    def _dynamics_residual(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
        dynamics,
    ) -> np.ndarray:
        x, u = self.unpack(z, problem)
        t = self.options.t(problem)

        dx = np.column_stack(
            [dynamics(x[:, k], u[:, k], float(t_k)) for k, t_k in enumerate(t)]
        )
        return trapezoidal_defect(x, dx, self.options.dt(problem)).reshape(-1)

    def _add_boundary_constraints(
        self,
        problem: PlanningProblem,
        *,
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
    ) -> None:
        self._add_point_boundary(
            problem,
            boundary=problem.X0,
            index=0,
            equalities=equalities,
            inequalities=inequalities,
        )
        self._add_terminal_constraints(
            problem,
            equalities=equalities,
            inequalities=inequalities,
        )

    def _add_terminal_constraints(
        self,
        problem: PlanningProblem,
        *,
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
    ) -> None:
        """Add terminal-boundary constraints (used by rebuild and parametric)."""
        self._add_point_boundary(
            problem,
            boundary=problem.Xf,
            index=-1,
            equalities=equalities,
            inequalities=inequalities,
        )

    def _add_point_boundary(
        self,
        problem: PlanningProblem,
        *,
        boundary,
        index: int,
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
    ) -> None:
        if boundary is None:
            return
        t_i = float(self.options.t(problem)[index])

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
                for k, t_k in enumerate(self.options.t(problem)):
                    margin = problem.X.margin(
                        x[:, k],
                        t=float(t_k),
                        params=problem.params.sets,
                    )
                    margins.append(margin.reshape(-1))
                return native_concatenate(margins, z)

            inequalities.append(state_margins)

        if problem.U is not None and not isinstance(problem.U, BoxInputSet):

            def input_margins(z):
                x, u = self.unpack(z, problem)
                margins = []
                for k, t_k in enumerate(self.options.t(problem)):
                    margin = problem.U.margin(
                        u[:, k],
                        x=x[:, k],
                        t=float(t_k),
                        params=problem.params.sets,
                    )
                    margins.append(margin.reshape(-1))
                return native_concatenate(margins, z)

            inequalities.append(input_margins)
