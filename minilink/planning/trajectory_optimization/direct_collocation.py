"""Direct collocation: a trajectory transcribed as knots joined by trapezoidal defects."""

import numpy as np

from minilink.core.backends import (
    BACKEND_JAX,
    BACKEND_NUMPY,
    ensure_jax_x64,
    normalize_backend,
)
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
    require_jax_compile_backend,
    stack_constraints,
    trajectory_cost,
    trapezoidal_defect,
)

###############################################################################
# Public API
###############################################################################


class DirectCollocationOptions(FixedGridOptions):
    """Grid options for direct-collocation transcriptions."""


class DirectCollocationTranscription(Transcription):
    """
    Fixed-time-grid trapezoidal direct-collocation transcription.

    The decision vector packs every sampled state followed by every sampled
    input, both in ``(dim, N)`` row-major order:

    ``z = [x[0, :], ..., x[n-1, :], u[0, :], ..., u[m-1, :]]``.
    """

    supports_parametric = True

    # The transcription's name in a program's metadata
    name = "direct_collocation"

    def __init__(self, options: DirectCollocationOptions):
        self.options = options

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str = BACKEND_NUMPY,
    ) -> MathematicalProgram:
        """Build the trapezoidal collocation nonlinear program."""
        # The JAX backend builds the same program, vectorized over the knots
        if normalize_backend(compile_backend, allow_direct=True) == BACKEND_JAX:
            return self.transcribe_jax(problem, compile_backend=compile_backend)

        problem.require_cost()
        dynamics = dynamics_function(problem, compile_backend)

        # Dynamics: the trapezoidal defect of every interval vanishes
        equalities: list[ConstraintFunction] = [
            lambda z: self.dynamics_residual(z, problem, dynamics)
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
        # Lazy import: the parametric program loads on first use, not with the
        # transcription
        from minilink.planning.trajectory_optimization.parametric_program import (
            ParametricMathematicalProgram,
        )

        require_jax_compile_backend(compile_backend)
        require_singleton_x0(problem)

        jax = ensure_jax_x64()
        jnp = jax.numpy

        unpack_jax, J, dynamics_residual = self.jax_objective_and_defects(problem, jax)
        n = int(problem.sys.n)

        # x_0 − x0 = 0: the measured initial state is the program's parameter
        def initial_residual(z, x0):
            x, _ = unpack_jax(z)
            residual = x[:, 0] - x0
            return residual.reshape(-1)

        # Dynamics and the terminal boundary, fixed at compile time
        fixed_equalities: list[ConstraintFunction] = [dynamics_residual]
        inequalities: list[ConstraintFunction] = []

        # The terminal and path constraints, and the box bounds on the knots
        self.add_terminal_constraints(
            problem,
            equalities=fixed_equalities,
            inequalities=inequalities,
        )
        self.add_path_constraints(problem, inequalities=inequalities)
        lower, upper = self.decision_bounds(problem)

        # h(z, x0) = [h_fixed(z); x_0 − x0]
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

        # Public trajectories are NumPy: convert off any JAX optimizer arrays here
        x = np.asarray(x, dtype=float)
        u = np.asarray(u, dtype=float)
        t = np.asarray(self.options.t(problem), dtype=float)
        if dynamics is None:
            dynamics = dynamics_function(problem, compile_backend)

        # f_k = f(x_k, u_k, t_k) at every knot
        dx = np.zeros_like(x)
        for k, t_k in enumerate(t):
            dx[:, k] = np.asarray(dynamics(x[:, k], u[:, k], float(t_k)), dtype=float)

        # The trajectory, with the cost signals when the problem has a cost
        traj = Trajectory(t=t, x=x, u=u, signals={"dx": dx})
        if problem.cost is not None:
            traj = problem.cost.evaluate_trajectory(traj, params=problem.params.cost)

        return traj

    def decision_dimension(self, problem: PlanningProblem) -> int:
        """Return the packed decision-vector dimension for ``problem``."""
        n = problem.sys.n
        m = problem.sys.m
        n_steps = self.options.n_steps

        n_z = int((n + m) * n_steps)

        return n_z

    def pack(
        self, x: np.ndarray, u: np.ndarray, problem: PlanningProblem
    ) -> np.ndarray:
        """Pack sampled state and input matrices into one decision vector."""
        z = np.concatenate((x.reshape(-1), u.reshape(-1)))

        return z

    def unpack(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Unpack a decision vector into sampled ``(x, u)`` matrices."""
        n = int(problem.sys.n)
        m = int(problem.sys.m)
        n_steps = self.options.n_steps

        # The states row by row, then the inputs
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
        require_initial_guess(guess)

        # A trajectory is resampled on the knots
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

        # The state box on every state knot
        X_box = problem.X.bounding_box()
        if X_box is not None:
            lower[: n * n_steps] = np.repeat(X_box.lower, n_steps)
            upper[: n * n_steps] = np.repeat(X_box.upper, n_steps)

        # The input box on every input knot
        U_box = problem.U.bounding_box()
        if U_box is not None:
            start = n * n_steps
            lower[start:] = np.repeat(U_box.lower, n_steps)
            upper[start:] = np.repeat(U_box.upper, n_steps)

        return lower, upper

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def transcribe_jax(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str,
    ) -> MathematicalProgram:
        """Build the JAX-vectorized program of :meth:`jax_objective_and_defects`.

        Direct collocation is performance-sensitive because SLSQP repeatedly
        calls the objective, gradient, constraints, and constraint Jacobians.
        The whole mathematical program is already JIT/autodiff compiled by the
        JAX program evaluator, so this path uses raw traceable equations and
        vectorized knot evaluation instead of nesting precompiled dynamics
        evaluators inside the program.
        """
        jax = ensure_jax_x64()

        _, J, dynamics_residual = self.jax_objective_and_defects(problem, jax)

        # Dynamics: the defect of every interval vanishes, by the transcription's
        # defect law
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
                "transcription": self.name,
                "compile_backend": compile_backend,
            },
        )

    def jax_objective_and_defects(self, problem: PlanningProblem, jax):
        """The objective and the defects as traceable JAX closures.

        Returns ``(unpack_jax, J, dynamics_residual)``: the knot unpacking
        ``unpack_jax(z) -> (x, u)``, the objective ``J(z)`` and the flattened
        trapezoidal defects ``dynamics_residual(z)``, the dynamics vectorized
        over the knots.
        """
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

        # The dynamics at one knot, vectorized over the knots below
        def f(x_k, u_k, t_k):
            return problem.sys.f(x_k, u_k, t_k, system_params)

        # J = Σ dt/2 (g_k + g_{k+1}) + h(x_N, t_N)
        def J(z):
            x, u = unpack_jax(z)
            return trajectory_cost(cost, x, u, t, dt, cost_params)

        # d_k = x_{k+1} − x_k − dt/2 (f_k + f_{k+1}), every interval at once
        def dynamics_residual(z):
            x, u = unpack_jax(z)
            dx = jax.vmap(f, in_axes=(1, 1, 0), out_axes=1)(x, u, t)
            defect = trapezoidal_defect(x, dx, dt)
            return defect.reshape(-1)

        return unpack_jax, J, dynamics_residual

    def dynamics_residual(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
        dynamics,
    ) -> np.ndarray:
        """Trapezoidal defects of the packed knots, flattened."""
        x, u = self.unpack(z, problem)
        t = self.options.t(problem)

        # f_k = f(x_k, u_k, t_k), one column per knot
        dx = np.column_stack(
            [dynamics(x[:, k], u[:, k], float(t_k)) for k, t_k in enumerate(t)]
        )

        # d_k = x_{k+1} − x_k − dt/2 (f_k + f_{k+1}) on every interval
        dt = self.options.dt(problem)
        defect = trapezoidal_defect(x, dx, dt)

        return defect.reshape(-1)

    def add_boundary_constraints(
        self,
        problem: PlanningProblem,
        *,
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
    ) -> None:
        """Add the initial and the terminal boundary constraints."""
        self.add_point_boundary(
            problem,
            boundary=problem.X0,
            index=0,
            equalities=equalities,
            inequalities=inequalities,
        )
        self.add_terminal_constraints(
            problem,
            equalities=equalities,
            inequalities=inequalities,
        )

    def add_point_boundary(
        self,
        problem: PlanningProblem,
        *,
        boundary,
        index: int,
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
    ) -> None:
        """Add one knot's boundary set: an equality for a singleton, its margins otherwise."""
        if boundary is None:
            return

        t_i = float(self.options.t(problem)[index])

        if isinstance(boundary, SingletonSet):
            # x_i − point = 0: the knot sits on the singleton's point
            def residual(z, boundary=boundary, index=index):
                x, _ = self.unpack(z, problem)
                return boundary.residual(x[:, index])

            equalities.append(residual)
        else:
            # margin(x_i, t_i) ≥ 0: the knot lies in the boundary set
            def margin(z, boundary=boundary, index=index, t_i=t_i):
                x, _ = self.unpack(z, problem)
                return boundary.margin(
                    x[:, index],
                    t=t_i,
                    params=problem.params.sets,
                )

            inequalities.append(margin)

    def add_terminal_constraints(
        self,
        problem: PlanningProblem,
        *,
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
    ) -> None:
        """Add terminal-boundary constraints (used by rebuild and parametric)."""
        self.add_point_boundary(
            problem,
            boundary=problem.Xf,
            index=-1,
            equalities=equalities,
            inequalities=inequalities,
        )

    def add_path_constraints(
        self,
        problem: PlanningProblem,
        *,
        inequalities: list[ConstraintFunction],
    ) -> None:
        """Add the state and input set margins at every knot."""
        # Lowering (RULES 4.3): a box is already in the decision bounds; any other
        # set contributes its margins as path inequalities
        if problem.X is not None and not isinstance(problem.X, BoxSet):
            # X.margin(x_k, t_k) ≥ 0 at every knot
            def state_margins(z):
                x, _ = self.unpack(z, problem)
                margins = []
                for k, t_k in enumerate(self.options.t(problem)):
                    margin = problem.X.margin(
                        x[:, k],
                        t=float(t_k),
                        params=problem.params.sets,
                    )
                    margins.append(margin)
                return native_concatenate(margins, z)

            inequalities.append(state_margins)

        if problem.U is not None and not isinstance(problem.U, BoxInputSet):
            # U.margin(u_k, x_k, t_k) ≥ 0 at every knot
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
                    margins.append(margin)
                return native_concatenate(margins, z)

            inequalities.append(input_margins)

    def objective(self, z: np.ndarray, problem: PlanningProblem):
        """Bolza objective ``J(z)`` of the packed knots."""
        cost = problem.require_cost()
        x, u = self.unpack(z, problem)
        t = self.options.t(problem)
        dt = self.options.dt(problem)
        cost_params = problem.params.cost

        # J = Σ dt/2 (g_k + g_{k+1}) + h(x_N, t_N)
        J = trajectory_cost(cost, x, u, t, dt, cost_params)

        return J


###############################################################################
# Internal machinery
###############################################################################


def require_singleton_x0(problem):
    """Refuse a parametric collocation program without a singleton initial set ``X0``."""
    if not isinstance(problem.X0, SingletonSet):
        raise ValueError(
            "Parametric direct collocation requires a singleton initial boundary X0."
        )


def require_initial_guess(guess):
    """Refuse a collocation solve without an initial guess."""
    if guess is None:
        raise ValueError("Direct collocation requires an initial guess")
