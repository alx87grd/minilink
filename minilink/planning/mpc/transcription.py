"""Beta direct-collocation transcription with parametric initial state."""

import numpy as np

from minilink.core.backends import BACKEND_JAX, normalize_backend
from minilink.core.sets import BoxInputSet, BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import OptimizationResult
from minilink.planning.mpc.parametric_program import ParametricMathematicalProgram
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.transcription import (
    ConstraintFunction,
    native_concatenate,
    stack_constraints,
    trajectory_cost,
    trapezoidal_defect,
)


class MPCDirectCollocationTranscription:
    """
    Beta direct collocation with runtime ``x0`` for receding-horizon MPC.

    Delegates pack/unpack/bounds to
    :class:`~minilink.planning.trajectory_optimization.direct_collocation.DirectCollocationTranscription`
    and builds a :class:`~minilink.planning.mpc.parametric_program.ParametricMathematicalProgram`
    whose equality constraints are ``h(z, x0)``.
    """

    def __init__(self, options: DirectCollocationOptions):
        self.options = options
        self._collocation = DirectCollocationTranscription(options)

    def transcribe_parametric(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str = BACKEND_JAX,
    ) -> ParametricMathematicalProgram:
        """Build a JAX parametric collocation program."""
        if normalize_backend(compile_backend, allow_direct=True) != BACKEND_JAX:
            raise ValueError(
                "Beta MPC transcription currently supports compile_backend='jax' only."
            )
        if not isinstance(problem.X0, SingletonSet):
            raise ValueError(
                "Beta MPC transcription requires a singleton initial boundary X0."
            )

        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()
        t = jnp.asarray(self.options.t)
        dt = jnp.asarray(self.options.dt)
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

        self._add_fixed_terminal_constraints(
            problem,
            equalities=fixed_equalities,
            inequalities=inequalities,
        )
        self._add_path_constraints(problem, inequalities=inequalities)
        lower, upper = self._collocation.decision_bounds(problem)

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
                "transcription": "mpc_direct_collocation",
                "compile_backend": compile_backend,
            },
        )

    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        dynamics,
    ) -> Trajectory:
        """Read ``(x, u)`` from the optimizer result using a cached dynamics fn."""
        x, u = self.unpack(result.z, problem)
        dx = np.zeros_like(x)
        for k, t_k in enumerate(self.options.t):
            dx[:, k] = dynamics(x[:, k], u[:, k], float(t_k))

        traj = Trajectory(
            t=self.options.t,
            x=x,
            u=u,
            signals={"dx": dx},
        )
        if problem.cost is not None:
            traj = problem.cost.evaluate_trajectory(traj, params=problem.params.cost)
        return traj

    def decision_dimension(self, problem: PlanningProblem) -> int:
        return self._collocation.decision_dimension(problem)

    def pack(
        self, x: np.ndarray, u: np.ndarray, problem: PlanningProblem
    ) -> np.ndarray:
        return self._collocation.pack(x, u, problem)

    def unpack(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
    ) -> tuple[np.ndarray, np.ndarray]:
        return self._collocation.unpack(z, problem)

    def pack_initial_guess(
        self,
        problem: PlanningProblem,
        guess: np.ndarray | Trajectory | None,
    ) -> np.ndarray:
        return self._collocation.pack_initial_guess(problem, guess)

    def initial_guess_time_grid(self, problem: PlanningProblem) -> np.ndarray:
        return self._collocation.initial_guess_time_grid(problem)

    def _add_fixed_terminal_constraints(
        self,
        problem: PlanningProblem,
        *,
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
    ) -> None:
        boundary = problem.Xf
        if boundary is None:
            return

        index = -1
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
                return native_concatenate(margins, z)

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
                return native_concatenate(margins, z)

            inequalities.append(input_margins)
