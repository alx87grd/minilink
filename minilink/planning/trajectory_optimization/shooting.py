"""Single shooting: a trajectory transcribed as input knots, its states rolled out by RK4."""

import numpy as np

from minilink.core.backends import (
    BACKEND_DIRECT,
    BACKEND_JAX,
    BACKEND_NUMPY,
    array_module,
    ensure_jax_x64,
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

###############################################################################
# Public API
###############################################################################


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
        # The JAX backend builds the same program, its rollout scanned over the knots
        if normalize_backend(compile_backend, allow_direct=True) == BACKEND_JAX:
            return self.transcribe_jax(problem, compile_backend=compile_backend)

        problem.require_cost()

        # The states are not decision variables: each function of z rolls them out
        rollout = self.make_rollout(problem, compile_backend)
        equalities: list[ConstraintFunction] = []
        inequalities: list[ConstraintFunction] = []

        # The terminal and path constraints on the rolled-out states, and the box
        # bounds on the input knots
        self.add_terminal_constraints(
            problem,
            rollout=rollout,
            equalities=equalities,
            inequalities=inequalities,
        )
        self.add_path_constraints(problem, rollout=rollout, inequalities=inequalities)
        lower, upper = self.decision_bounds(problem)

        return MathematicalProgram(
            n_z=self.decision_dimension(problem),
            J=lambda z: self.objective(z, problem, rollout),
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

    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        compile_backend: str = BACKEND_NUMPY,
    ) -> Trajectory:
        """Roll out the optimized input sequence."""
        # The program's rollout, and the dynamics for the state rates
        rollout = self.make_rollout(problem, compile_backend)
        dynamics = dynamics_function(problem, compile_backend)

        # The optimized input knots, and the states they roll out
        u = self.unpack(result.z, problem)
        x = rollout(u)

        # ẋ_k = f(x_k, u_k, t_k) at every knot
        dx = np.zeros_like(x)
        for k, t_k in enumerate(self.options.t(problem)):
            dx[:, k] = dynamics(x[:, k], u[:, k], float(t_k))

        # The trajectory, with the cost's signals when the problem has a cost
        traj = Trajectory(t=self.options.t(problem), x=x, u=u, signals={"dx": dx})
        if problem.cost is not None:
            traj = problem.cost.evaluate_trajectory(traj, params=problem.params.cost)

        return traj

    def initial_guess_time_grid(self, problem: PlanningProblem) -> np.ndarray:
        """Return the shooting input-knot grid."""
        return self.options.t(problem)

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
        n_steps = self.options.n_steps

        # Unbounded, unless the input set has a bounding box
        lower = np.full(n_z, -np.inf)
        upper = np.full(n_z, np.inf)

        # u_min ≤ u_k ≤ u_max at every knot: each input's bound along its row of z
        U_box = problem.U.bounding_box()
        if U_box is not None:
            lower[:] = np.repeat(U_box.lower, n_steps)
            upper[:] = np.repeat(U_box.upper, n_steps)

        return lower, upper

    def pack_initial_guess(
        self,
        problem: PlanningProblem,
        guess: np.ndarray | Trajectory | None,
    ) -> np.ndarray:
        """Return a packed input-knot guess for the mathematical program."""
        if guess is None:
            raise ValueError("Shooting requires an initial input guess")

        # A trajectory gives its inputs, resampled on the knots
        if isinstance(guess, Trajectory):
            resampled = guess.resample(t_new=self.options.t(problem))
            return self.pack(resampled.u, problem)

        return guess.reshape(-1)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def transcribe_jax(
        self,
        problem: PlanningProblem,
        *,
        compile_backend: str,
    ) -> MathematicalProgram:
        """Build a JAX scan-based single-shooting program."""
        jax = ensure_jax_x64()
        jnp = jax.numpy

        cost = problem.require_cost()
        t = jnp.asarray(self.options.t(problem))
        dt = jnp.asarray(self.options.dt(problem))
        n_steps = int(self.options.n_steps)
        m = int(problem.sys.m)
        x0 = jnp.asarray(self.initial_state(problem))
        system_params = problem.params.system
        cost_params = problem.params.cost

        # The input knots packed in z, one input's row after another
        def unpack_jax(z):
            return z.reshape(m, n_steps)

        # ẋ = f(x, u, t) with the problem's system params
        def f(x, u_k, t_k):
            return problem.sys.f(x, u_k, t_k, system_params)

        # x_{k+1} = RK4(x_k, u_k, u_{k+1}) from x_0, scanned over the intervals
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
            x = jnp.concatenate((x0[:, None], xs.T), axis=1)

            return x

        # J = Σ dt/2 (g_k + g_{k+1}) + h(x_N, t_N), on the rolled-out states
        def J(z):
            u = unpack_jax(z)
            x = rollout(u)
            return trajectory_cost(cost, x, u, t, dt, cost_params)

        equalities: list[ConstraintFunction] = []
        inequalities: list[ConstraintFunction] = []

        # The terminal and path constraints on the rolled-out states, and the box
        # bounds on the input knots
        self.add_terminal_constraints(
            problem,
            rollout=rollout,
            equalities=equalities,
            inequalities=inequalities,
        )
        self.add_path_constraints(problem, rollout=rollout, inequalities=inequalities)
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

    def make_rollout(
        self,
        problem: PlanningProblem,
        compile_backend: str,
    ):
        """``rollout(u) -> x``: the compiled RK4 integrator, or the knot loop (direct mode, system params)."""
        x0 = self.initial_state(problem)
        key = normalize_backend(compile_backend, allow_direct=True)

        # A compiled system without explicit params: its own integrator over the grid
        if key != BACKEND_DIRECT and problem.params.system is None:
            evaluator = problem.sys.compile(backend=key, verbose=False)

            def rollout(u):
                t0 = float(self.options.t(problem)[0])
                dt = self.options.dt(problem)

                # x_{k+1} = RK4(x_k, u_k, u_{k+1}), one row per knot, turned to columns
                x_samples = evaluator.rk4_integrate_linear(x0, u.T, t0, dt)
                x = x_samples.T

                return x

            return rollout

        # Direct mode or explicit system params: step the dynamics per knot
        f = dynamics_function(problem, compile_backend)
        return lambda u: self.rollout_loop(f, u, x0, problem)

    def initial_state(self, problem: PlanningProblem) -> np.ndarray:
        """The rollout's start: the singleton ``X0`` point, else ``x_start``."""
        if isinstance(problem.X0, SingletonSet):
            return problem.X0.point
        return problem.x_start

    def rollout_loop(self, f, u, x0, problem: PlanningProblem) -> np.ndarray:
        """The state knots of the input knots ``u`` from ``x0``, one RK4 step per interval."""
        xp = array_module(u)

        n = int(problem.sys.n)
        n_steps = self.options.n_steps
        t = self.options.t(problem)
        dt = self.options.dt(problem)

        # x_{k+1} = RK4(x_k, u_k, u_{k+1}) from x_0
        x_k = xp.asarray(x0).reshape(n)
        x_samples = [x_k]
        for k in range(n_steps - 1):
            x_k = rk4_step_between_knots(f, x_k, u[:, k], u[:, k + 1], float(t[k]), dt)
            x_samples.append(x_k)

        # The state knots, one column per knot
        x = xp.stack(x_samples, axis=1)

        return x

    def add_terminal_constraints(
        self,
        problem: PlanningProblem,
        *,
        rollout,
        equalities: list[ConstraintFunction],
        inequalities: list[ConstraintFunction],
    ) -> None:
        """Add the terminal set on the last rolled-out state."""
        boundary = problem.Xf
        if boundary is None:
            return

        # A point: the equality x_N − x_f = 0
        if isinstance(boundary, SingletonSet):

            def residual(z, boundary=boundary):
                x = rollout(self.unpack(z, problem))
                return boundary.residual(x[:, -1])

            equalities.append(residual)
            return

        # Any other set: its margins at the final knot, m(x_N, t_N) ≥ 0
        def margin(z, boundary=boundary):
            x = rollout(self.unpack(z, problem))
            return boundary.margin(
                x[:, -1],
                t=float(self.options.t(problem)[-1]),
                params=problem.params.sets,
            )

        inequalities.append(margin)

    def add_path_constraints(
        self,
        problem: PlanningProblem,
        *,
        rollout,
        inequalities: list[ConstraintFunction],
    ) -> None:
        """Add the state and input set margins at every knot of the rollout."""
        # m_X(x_k, t_k) ≥ 0 at every knot; the states are not decision variables, so
        # even a state box enters as margins
        if problem.X is not None:

            def state_margins(z):
                x = rollout(self.unpack(z, problem))
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

        # m_U(u_k, x_k, t_k) ≥ 0 at every knot. Lowering (RULES 4.3): an input box is
        # already in the decision bounds
        if problem.U is not None and not isinstance(problem.U, BoxInputSet):

            def input_margins(z):
                u = self.unpack(z, problem)
                x = rollout(u)
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

    def objective(
        self,
        z: np.ndarray,
        problem: PlanningProblem,
        rollout,
    ):
        """Bolza objective ``J(z)`` of the input knots, the states rolled out."""
        cost = problem.require_cost()
        u = self.unpack(z, problem)
        x = rollout(u)
        t = self.options.t(problem)
        dt = self.options.dt(problem)
        cost_params = problem.params.cost

        # J = Σ dt/2 (g_k + g_{k+1}) + h(x_N, t_N)
        J = trajectory_cost(cost, x, u, t, dt, cost_params)

        return J
