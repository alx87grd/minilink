"""Multiple shooting: a trajectory transcribed as knots joined by RK4 shooting defects."""

import numpy as np

from minilink.core.backends import (
    BACKEND_JAX,
    BACKEND_NUMPY,
    normalize_backend,
)
from minilink.core.compile.evaluators.stepping import step_body
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

    # The transcription's name in a program's metadata
    name = "multiple_shooting"

    def __init__(self, options: MultipleShootingOptions):
        self.options = options

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        backend: str = BACKEND_NUMPY,
    ) -> MathematicalProgram:
        """Build the multiple-shooting nonlinear program."""
        # The JAX backend builds the same program, vectorized over the knots
        if normalize_backend(backend, allow_direct=True) == BACKEND_JAX:
            return self.transcribe_jax(problem, backend=backend)

        problem.require_cost()
        step = self.make_step(problem, backend)

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
            backend=program_backend_for_compile(backend),
            metadata={
                "transcription": "multiple_shooting",
                "backend": backend,
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

    def jax_objective_and_defects(self, problem: PlanningProblem, jax):
        """The objective and the RK4 shooting defects as traceable JAX closures of ``(z, params)``.

        Returns ``(unpack_jax, J, dynamics_residual)`` as the collocation's does,
        the defect of each interval the RK4 step between its knots.
        """
        jnp = jax.numpy

        cost = problem.require_cost()
        grid = self.knot_offsets(problem, jnp)
        dt = jnp.asarray(self.options.dt(problem))
        unpack_jax = self.jax_unpacker(problem)

        # One RK4 step between neighbouring knots, the input linear in between
        def rk4_step(x_k, u0, u1, t_k, plant):
            def f(x, u, t):
                return problem.sys.f(x, u, t, plant)

            return step_body(f, x_k, u0, u1, t_k, dt, "rk4", 1)

        # J = Σ dt/2 (w_k g_k + w_{k+1} g_{k+1}) + w_N h(x_N, t_N), t_k = t_start + k dt
        def J(z, params):
            x, u = unpack_jax(z)
            t = params["t_start"] + grid
            return trajectory_cost(
                cost,
                x,
                u,
                t,
                dt,
                params["cost"],
                discount_rate=params["discount_rate"],
                t_start=params["t_start"],
            )

        # d_k = x_{k+1} − RK4(x_k, u_k, u_{k+1}), every interval at once
        def dynamics_residual(z, params):
            x, u = unpack_jax(z)
            t = params["t_start"] + grid
            x_next = jax.vmap(rk4_step, in_axes=(1, 1, 1, 0, None), out_axes=1)(
                x[:, :-1],
                u[:, :-1],
                u[:, 1:],
                t[:-1],
                params["plant"],
            )
            defect = x[:, 1:] - x_next
            return defect.reshape(-1)

        return unpack_jax, J, dynamics_residual

    def make_step(self, problem: PlanningProblem, backend: str):
        """One RK4 step between knots, ``step(x, u0, u1, t)``, on the grid's ``dt``."""
        f = dynamics_function(problem, backend)
        dt = self.options.dt(problem)

        def step(x, u0, u1, t):
            return step_body(f, x, u0, u1, t, dt, "rk4", 1)

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
