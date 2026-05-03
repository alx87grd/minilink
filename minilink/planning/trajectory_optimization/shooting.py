"""Single-shooting trajectory optimization."""

import numpy as np

from minilink.core.costs import require_jax_traceable_cost
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
        compile_backend: str | None = "numpy",
    ) -> MathematicalProgram:
        """Build the input-knot nonlinear program."""
        if compile_backend == "jax":
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
            metadata={
                "transcription": "shooting",
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
        """Build a JAX-traceable single-shooting program."""
        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()
        require_jax_traceable_cost(cost)
        self._check_jax_supported_problem(problem)

        t = jnp.asarray(self.options.t)
        dt = jnp.asarray(self.options.dt)
        n_steps = int(self.options.n_steps)
        m = int(problem.sys.m)
        x0 = jnp.asarray(self._initial_state(problem))
        params = problem.params.system

        def unpack_jax(z):
            return z.reshape(m, n_steps)

        def rollout(u):
            def body(carry, u_pair):
                x, t_k = carry
                u0, u1 = u_pair
                umid = 0.5 * (u0 + u1)
                k1 = problem.sys.f(x, u0, t_k, params)
                k2 = problem.sys.f(x + 0.5 * dt * k1, umid, t_k + 0.5 * dt, params)
                k3 = problem.sys.f(x + 0.5 * dt * k2, umid, t_k + 0.5 * dt, params)
                k4 = problem.sys.f(x + dt * k3, u1, t_k + dt, params)
                x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
                return (x_next, t_k + dt), x_next

            (_, _), xs = jax.lax.scan(
                body,
                (x0, t[0]),
                (u[:, :-1].T, u[:, 1:].T),
            )
            return jnp.concatenate((x0[:, None], xs.T), axis=1)

        def J(z):
            u = unpack_jax(z)
            x = rollout(u)
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

        equalities = []
        if isinstance(problem.Xf, SingletonSet):
            target = jnp.asarray(problem.Xf.point)

            def terminal_residual(z):
                x = rollout(unpack_jax(z))
                return x[:, -1] - target

            equalities.append(terminal_residual)

        inequalities = []
        if isinstance(problem.Xf, BoxSet):
            lower = jnp.asarray(problem.Xf.lower)
            upper = jnp.asarray(problem.Xf.upper)

            def terminal_margin(z):
                x = rollout(unpack_jax(z))
                x_f = x[:, -1]
                return jnp.concatenate((x_f - lower, upper - x_f))

            inequalities.append(terminal_margin)

        if isinstance(problem.X, BoxSet):
            lower = jnp.asarray(problem.X.lower)
            upper = jnp.asarray(problem.X.upper)

            def state_margin(z):
                x = rollout(unpack_jax(z)).T
                return jnp.concatenate((x - lower, upper - x), axis=1).reshape(-1)

            inequalities.append(state_margin)

        def h(z):
            return jnp.concatenate([residual(z).reshape(-1) for residual in equalities])

        def g(z):
            return jnp.concatenate([margin(z).reshape(-1) for margin in inequalities])

        lower, upper = self.decision_bounds(problem)
        return MathematicalProgram(
            n_z=self.decision_dimension(problem),
            J=J,
            h=h if equalities else None,
            g=g if inequalities else None,
            lower=lower,
            upper=upper,
            metadata={
                "transcription": "shooting",
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

    def decision_bounds(self, problem: PlanningProblem) -> tuple[np.ndarray, np.ndarray]:
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
        compile_backend: str | None,
    ):
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
        rollout,
    ):
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

        total += cost.h(x[:, -1], float(t[-1]), params=params)
        return total

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
                return np.concatenate(margins)

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
                return np.concatenate(margins)

            inequalities.append(input_margins)

    @staticmethod
    def _check_jax_supported_problem(problem: PlanningProblem) -> None:
        if problem.Xf is not None and not isinstance(
            problem.Xf,
            (SingletonSet, BoxSet),
        ):
            raise NotImplementedError(
                "JAX shooting currently supports only SingletonSet or BoxSet "
                "terminal constraints"
            )

        if problem.X is not None and not isinstance(problem.X, BoxSet):
            raise NotImplementedError(
                "JAX shooting currently supports only BoxSet state path constraints"
            )

        if problem.U is not None and not isinstance(problem.U, BoxInputSet):
            raise NotImplementedError(
                "JAX shooting currently supports only BoxInputSet input constraints"
            )
