"""Multiple shooting: a trajectory transcribed as knots joined by RK4 shooting defects."""

import numpy as np

from minilink.core.backends import (
    BACKEND_JAX,
    BACKEND_NUMPY,
    ensure_jax_x64,
    normalize_backend,
)
from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.transcription import (
    ConstraintFunction,
    FixedGridOptions,
    dynamics_function,
    native_concatenate,
    program_backend_for_compile,
    rk4_step_between_knots,
    stack_constraints,
    trajectory_cost,
)

###############################################################################
# Public API
###############################################################################


class MultipleShootingOptions(FixedGridOptions):
    """Grid options for fixed-step multiple shooting."""


class MultipleShootingTranscription(DirectCollocationTranscription):
    """
    Fixed-grid multiple-shooting transcription.

    The decision vector is the same as direct collocation:
    ``z = [x[0, :], ..., x[n-1, :], u[0, :], ..., u[m-1, :]]``.
    Dynamics are enforced by RK4 shooting defects between neighboring knots.
    """

    # The collocation parametric build assumes collocation defects; multiple
    # shooting has no parametric (MPC) form.
    supports_parametric = False

    def __init__(self, options: MultipleShootingOptions):
        self.options = options

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str = BACKEND_NUMPY,
    ) -> MathematicalProgram:
        """Build the multiple-shooting nonlinear program."""
        # The JAX backend builds the same program, vectorized over the knots
        if normalize_backend(compile_backend, allow_direct=True) == BACKEND_JAX:
            return self.transcribe_jax(problem, compile_backend=compile_backend)

        problem.require_cost()
        step = self.make_step(problem, compile_backend)

        # Dynamics: the shooting defect of every interval vanishes
        equalities: list[ConstraintFunction] = [
            lambda z: self.dynamics_residual(z, problem, step)
        ]
        inequalities: list[ConstraintFunction] = []

        # The boundary and path constraints, and the box bounds on the knots
        self.add_boundary_constraints(
            problem,
            equalities=equalities,
            inequalities=inequalities,
        )
        self.add_path_constraints(problem, inequalities=inequalities)
        lower, upper = self.decision_bounds(problem)

        return MathematicalProgram(
            n_z=self.decision_dimension(problem),
            J=lambda z: self.objective(z, problem),
            h=stack_constraints(equalities),
            g=stack_constraints(inequalities),
            lower=lower,
            upper=upper,
            backend=program_backend_for_compile(compile_backend),
            metadata={
                "transcription": "multiple_shooting",
                "compile_backend": compile_backend,
            },
        )

    def transcribe_parametric(self, *args, **kwargs):
        """Refuse: multiple shooting has no parametric (MPC) form."""
        raise NotImplementedError(
            "MultipleShootingTranscription has no parametric (MPC) form; use "
            "transcription='direct_collocation' for compile_parametric_program"
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def transcribe_jax(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str,
    ) -> MathematicalProgram:
        """Build a JAX-vectorized multiple-shooting program."""
        jax = ensure_jax_x64()
        jnp = jax.numpy

        cost = problem.require_cost()
        t = jnp.asarray(self.options.t(problem))
        dt = jnp.asarray(self.options.dt(problem))
        n = int(problem.sys.n)
        m = int(problem.sys.m)
        n_steps = int(self.options.n_steps)
        system_params = problem.params.system
        cost_params = problem.params.cost

        # The knots packed in z: the states row by row, then the inputs
        def unpack_jax(z):
            split = n * n_steps
            x = z[:split].reshape(n, n_steps)
            u = z[split:].reshape(m, n_steps)
            return x, u

        # One RK4 step between neighbouring knots, the input linear in between
        def f(x, u, t_k):
            return problem.sys.f(x, u, t_k, system_params)

        def rk4_step(x_k, u0, u1, t_k):
            return rk4_step_between_knots(f, x_k, u0, u1, t_k, dt)

        # J = Σ dt/2 (g_k + g_{k+1}) + h(x_N, t_N)
        def J(z):
            x, u = unpack_jax(z)
            return trajectory_cost(cost, x, u, t, dt, cost_params)

        # d_k = x_{k+1} − RK4(x_k, u_k, u_{k+1}), every interval at once
        def dynamics_residual(z):
            x, u = unpack_jax(z)
            x_next = jax.vmap(rk4_step, in_axes=(1, 1, 1, 0), out_axes=1)(
                x[:, :-1],
                u[:, :-1],
                u[:, 1:],
                t[:-1],
            )
            defect = x[:, 1:] - x_next
            return defect.reshape(-1)

        equalities: list[ConstraintFunction] = [dynamics_residual]
        inequalities: list[ConstraintFunction] = []

        # The boundary and path constraints, and the box bounds on the knots
        self.add_boundary_constraints(
            problem,
            equalities=equalities,
            inequalities=inequalities,
        )
        self.add_path_constraints(problem, inequalities=inequalities)
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
                "transcription": "multiple_shooting",
                "compile_backend": compile_backend,
            },
        )

    def make_step(self, problem: PlanningProblem, compile_backend: str):
        """One RK4 step between knots, ``step(x, u0, u1, t)``, on the grid's ``dt``."""
        f = dynamics_function(problem, compile_backend)
        dt = self.options.dt(problem)

        def step(x, u0, u1, t):
            return rk4_step_between_knots(f, x, u0, u1, t, dt)

        return step

    def dynamics_residual(self, z: np.ndarray, problem: PlanningProblem, step):
        """Shooting defects of the packed knots, flattened."""
        x, u = self.unpack(z, problem)
        t = self.options.t(problem)

        # d_k = x_{k+1} − RK4(x_k, u_k, u_{k+1}) on every interval
        residuals = []
        for k, t_k in enumerate(t[:-1]):
            x_next = step(x[:, k], u[:, k], u[:, k + 1], float(t_k))
            d_k = x[:, k + 1] - x_next
            residuals.append(d_k)

        return native_concatenate(residuals, z)
