"""Single-shooting trajectory optimization."""

import numpy as np

from minilink.core.backends import (
    BACKEND_DIRECT,
    BACKEND_JAX,
    BACKEND_NUMPY,
    array_module,
    normalize_backend,
)
from minilink.core.sets import BoxInputSet, SingletonSet
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
    rk4_step_between_knots,
    stack_constraints,
    trajectory_cost,
)


class ShootingOptions(FixedGridOptions):
    """Grid options for fixed-step single shooting."""


class ShootingTranscription(Transcription):
    """
    Fixed-grid single-shooting transcription.

    The decision vector contains input knots only:
    ``z = [u[0, :], ..., u[m-1, :]]``. States are reconstructed by RK4 rollout.
    """

    def __init__(self, options: ShootingOptions):
        self.options = options

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str = BACKEND_NUMPY,
    ) -> MathematicalProgram:
        """Build the input-knot nonlinear program."""
        if normalize_backend(compile_backend, allow_direct=True) == BACKEND_JAX:
            return self._transcribe_jax(problem, compile_backend=compile_backend)

        problem.require_cost()
        rollout = self._make_rollout(problem, compile_backend)
        equalities: list[ConstraintFunction] = []
        inequalities: list[ConstraintFunction] = []

        self._add_terminal_constraints(
            problem,
            rollout=rollout,
            equalities=equalities,
            inequalities=inequalities,
        )
        self._add_path_constraints(problem, rollout=rollout, inequalities=inequalities)
        lower, upper = self.decision_bounds(problem)

        return MathematicalProgram(
            n_z=self.decision_dimension(problem),
            J=lambda z: self._objective(z, problem, rollout),
            h=stack_constraints(equalities),
            g=stack_constraints(inequalities),
            lower=lower,
            upper=upper,
            backend=program_backend_for_compile(compile_backend),
            metadata={
                "transcription": "shooting",
                "compile_backend": compile_backend,
            },
        )

    def _transcribe_jax(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str,
    ) -> MathematicalProgram:
        """Build a JAX scan-based single-shooting program."""
        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()
        t = jnp.asarray(self.options.t)
        dt = jnp.asarray(self.options.dt)
        n_steps = int(self.options.n_steps)
        m = int(problem.sys.m)
        x0 = jnp.asarray(self._initial_state(problem))
        system_params = problem.params.system
        cost_params = problem.params.cost

        def unpack_jax(z):
            return z.reshape(m, n_steps)

        def f(x, u_k, t_k):
            return problem.sys.f(x, u_k, t_k, system_params)

        def rollout(u):
            def body(x, inputs):
                u0, u1, t_k = inputs
                x_next = rk4_step_between_knots(f, x, u0, u1, t_k, dt)
                return x_next, x_next

            _, xs = jax.lax.scan(
                body,
                x0,
                (u[:, :-1].T, u[:, 1:].T, t[:-1]),
            )
            return jnp.concatenate((x0[:, None], xs.T), axis=1)

        def J(z):
            u = unpack_jax(z)
            x = rollout(u)
            return trajectory_cost(cost, x, u, t, dt, cost_params)

        equalities: list[ConstraintFunction] = []
        inequalities: list[ConstraintFunction] = []

        self._add_terminal_constraints(
            problem,
            rollout=rollout,
            equalities=equalities,
            inequalities=inequalities,
        )
        self._add_path_constraints(problem, rollout=rollout, inequalities=inequalities)
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
                "transcription": "shooting",
                "compile_backend": compile_backend,
            },
        )

    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        compile_backend: str = BACKEND_NUMPY,
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

    def decision_bounds(
        self, problem: PlanningProblem
    ) -> tuple[np.ndarray, np.ndarray]:
        """Build box bounds for input-knot decision variables."""
        n_z = self.decision_dimension(problem)
        lower = np.full(n_z, -np.inf)
        upper = np.full(n_z, np.inf)
        if isinstance(problem.U, BoxInputSet):
            lower[:] = np.repeat(problem.U.box.lower, self.options.n_steps)
            upper[:] = np.repeat(problem.U.box.upper, self.options.n_steps)
        return lower, upper

    def pack_initial_guess(
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
        compile_backend: str,
    ):
        x0 = self._initial_state(problem)
        key = normalize_backend(compile_backend, allow_direct=True)

        if key != BACKEND_DIRECT and problem.params.system is None:
            evaluator = problem.sys.compile(backend=key, verbose=False)

            def rollout(u):
                x_samples = evaluator.rk4_rollout_forced(
                    x0,
                    u.T,
                    float(self.options.t[0]),
                    self.options.dt,
                )
                return x_samples.T

            return rollout

        # Direct mode or explicit system params: step the dynamics per knot.
        f = dynamics_function(problem, compile_backend)
        return lambda u: self._rollout_loop(f, u, x0, problem)

    def _rollout_loop(self, f, u, x0, problem: PlanningProblem) -> np.ndarray:
        n = int(problem.sys.n)
        t = self.options.t
        dt = self.options.dt
        xp = array_module(u)

        x = xp.asarray(x0).reshape(n)
        x_samples = [x]
        for k in range(self.options.n_steps - 1):
            x = rk4_step_between_knots(f, x, u[:, k], u[:, k + 1], float(t[k]), dt)
            x_samples.append(x)
        return xp.stack(x_samples, axis=1)

    def _objective(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
        rollout,
    ):
        cost = problem.require_cost()
        u = self.unpack(z, problem)
        x = rollout(u)
        return trajectory_cost(
            cost, x, u, self.options.t, self.options.dt, problem.params.cost
        )

    def _add_terminal_constraints(
        self,
        problem: PlanningProblem,
        *,
        rollout,
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
    ) -> None:
        boundary = problem.Xf
        if boundary is None:
            return

        if isinstance(boundary, SingletonSet):

            def residual(z, boundary=boundary):
                x = rollout(self.unpack(z, problem))
                return boundary.residual(x[:, -1])

            equalities.append(residual)
            return

        def margin(z, boundary=boundary):
            x = rollout(self.unpack(z, problem))
            return boundary.margin(
                x[:, -1],
                t=float(self.options.t[-1]),
                params=problem.params.sets,
            )

        inequalities.append(margin)

    def _add_path_constraints(
        self,
        problem: PlanningProblem,
        *,
        rollout,
        inequalities: list[ConstraintFunction],
    ) -> None:
        if problem.X is not None:

            def state_margins(z):
                x = rollout(self.unpack(z, problem))
                margins = []
                for k, t_k in enumerate(self.options.t):
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
                u = self.unpack(z, problem)
                x = rollout(u)
                margins = []
                for k, t_k in enumerate(self.options.t):
                    margin = problem.U.margin(
                        u[:, k],
                        x=x[:, k],
                        t=float(t_k),
                        params=problem.params.sets,
                    )
                    margins.append(margin.reshape(-1))
                return native_concatenate(margins, z)

            inequalities.append(input_margins)
