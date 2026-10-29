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
from minilink.planning.problems import PlanningProblem, nominal_plant_params
from minilink.planning.trajectory_optimization.transcription import (
    ConstraintFunction,
    FixedGridOptions,
    Transcription,
    dynamics_function,
    native_concatenate,
    program_backend_for_compile,
    require_jax_compile_backend,
    stack_constraints,
    stack_parametric,
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
        backend: str = BACKEND_NUMPY,
    ) -> MathematicalProgram:
        """Build the trapezoidal collocation nonlinear program."""
        # The JAX backend builds the same program, vectorized over the knots
        if normalize_backend(backend, allow_direct=True) == BACKEND_JAX:
            return self.transcribe_jax(problem, backend=backend)

        problem.require_cost()
        dynamics = dynamics_function(
            problem, backend, self.plant_evaluator(problem, backend)
        )

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
            backend=program_backend_for_compile(backend),
            metadata={
                "transcription": "direct_collocation",
                "backend": backend,
            },
        )

    def transcribe_parametric(
        self,
        problem: PlanningProblem,
        *,
        backend: str = BACKEND_JAX,
    ) -> MathematicalProgram:
        """The JAX program an MPC compiles once: :meth:`transcribe`'s, its start pinned from ``params["x_start"]``.

        Requires a singleton ``X0``, so the measured start binds the initial
        residual ``x[:, 0] − params["x_start"]``.
        """
        require_jax_compile_backend(backend)
        require_singleton_x0(problem)

        return self.transcribe_jax(
            problem, backend=backend, name="direct_collocation_parametric"
        )

    def reconstruct_result(
        self,
        result: OptimizationResult,
        *,
        problem: PlanningProblem,
        backend: str = BACKEND_NUMPY,
        dynamics=None,
        params=None,
    ) -> Trajectory:
        """Read ``(x, u)`` from the optimizer result.

        Prefer a cached ``dynamics`` callable (parametric compile path);
        otherwise build one via ``backend``. ``params``, the tree a compiled
        program was solved with, gives the cost's numbers and the discount of
        the cost signals; ``None`` reads the problem's.
        """
        x, u = self.unpack(result.z, problem)

        # Public trajectories are NumPy: convert off any JAX optimizer arrays here
        x = np.asarray(x, dtype=float)
        u = np.asarray(u, dtype=float)
        t = np.asarray(self.options.t(problem), dtype=float)
        if dynamics is None:
            dynamics = dynamics_function(
                problem, backend, self.plant_evaluator(problem, backend)
            )

        # f_k = f(x_k, u_k, t_k) at every knot
        dx = np.zeros_like(x)
        for k, t_k in enumerate(t):
            dx[:, k] = np.asarray(dynamics(x[:, k], u[:, k], float(t_k)), dtype=float)

        # The trajectory, with the cost signals when the problem has a cost
        traj = Trajectory(t=t, x=x, u=u, signals={"dx": dx})
        if problem.cost is not None:
            cost_params = problem.cost.params if params is None else params["cost"]
            rho = problem.discount_rate if params is None else params["discount_rate"]
            traj = traj.with_cost(
                problem.cost,
                discount_rate=rho,
                params=cost_params,
                t_start=problem.t_start,
            )

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
        self, problem: PlanningProblem, *, boxes_only=False
    ) -> tuple[np.ndarray, np.ndarray]:
        """Build box bounds on the packed decision vector when sets expose boxes.

        ``boxes_only``: the bounds of a true box only (the JAX program, whose
        other sets are leaves of its tree: a moved set's compile-time box would
        stay in the bounds).
        """
        n = int(problem.sys.n)
        n_steps = self.options.n_steps
        n_z = self.decision_dimension(problem)

        lower = np.full(n_z, -np.inf)
        upper = np.full(n_z, np.inf)

        # The state box on every state knot
        X_box = box_bounds(problem.X, BoxSet, boxes_only)
        if X_box is not None:
            lower[: n * n_steps] = np.repeat(X_box.lower, n_steps)
            upper[: n * n_steps] = np.repeat(X_box.upper, n_steps)

        # The input box on every input knot
        U_box = box_bounds(problem.U, BoxInputSet, boxes_only)
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
        backend: str,
        name: str | None = None,
    ) -> MathematicalProgram:
        """Build the JAX program on the problem's tree, ``J(z, params)``, ``h(z, params)``, ``g(z, params)``.

        The tree is :meth:`program_params`: the start, its time, the plant's,
        the cost's and the discount, and each set read as a constraint. One program is compiled once and solved for any value of
        them, a moved target or a new friction, without a rebuild. Direct
        collocation is performance-sensitive because SLSQP repeatedly calls the
        objective, gradient, constraints, and constraint Jacobians; the whole
        program is JIT/autodiff compiled by the JAX program evaluator, so this
        path uses raw traceable equations and knot evaluation vectorized with
        ``vmap``.
        """
        jax = ensure_jax_x64()

        unpack_jax, J, dynamics_residual = self.jax_objective_and_defects(problem, jax)

        # Dynamics: the defect of every interval vanishes, by the transcription's
        # defect law; then the boundary and path constraints
        equalities, inequalities = self.jax_constraints(problem, unpack_jax, jax)
        equalities = [dynamics_residual, *equalities]
        lower, upper = self.decision_bounds(problem, boxes_only=True)

        return MathematicalProgram(
            n_z=self.decision_dimension(problem),
            J=J,
            h=stack_parametric(equalities),
            g=stack_parametric(inequalities),
            lower=lower,
            upper=upper,
            backend=BACKEND_JAX,
            metadata={
                "transcription": self.name if name is None else name,
                "backend": backend,
            },
            params=self.program_params(problem),
        )

    def program_params(self, problem: PlanningProblem) -> dict:
        """
        The JAX program's tree: the problem's leaves it reads.

        ``x_start``, ``t_start``, the plant's (a random plant's at its mean),
        the cost's and the discount rate; each set it reads as a constraint by
        role (``X`` and ``U`` when not lowered to the decision bounds, ``X0``
        when not the start's singleton, ``Xf``), with its numbers. A box lowered
        to the bounds is the program's structure, not a leaf.
        """
        plant = nominal_plant_params(problem)
        tree = {
            "x_start": np.asarray(problem.x_start, dtype=float),
            "t_start": float(problem.t_start),
            "plant": problem.sys.params if plant is None else plant,
            "cost": problem.require_cost().params,
            "discount_rate": float(problem.discount_rate),
        }
        for role in constrained_roles(problem):
            tree[role] = getattr(problem, role).params

        return tree

    def jax_objective_and_defects(self, problem: PlanningProblem, jax):
        """The objective and the defects as traceable JAX closures of ``(z, params)``.

        Returns ``(unpack_jax, J, dynamics_residual)``: the knot unpacking
        ``unpack_jax(z) -> (x, u)``, the objective ``J(z, params)`` and the
        flattened trapezoidal defects ``dynamics_residual(z, params)``, the
        dynamics vectorized over the knots, every number read from ``params``.
        """
        jnp = jax.numpy

        cost = problem.require_cost()
        grid = self.knot_offsets(problem, jnp)
        dt = jnp.asarray(self.options.dt(problem))
        unpack_jax = self.jax_unpacker(problem)

        # The dynamics at one knot, vectorized over the knots below
        def f(x_k, u_k, t_k, plant):
            return problem.sys.f(x_k, u_k, t_k, plant)

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

        # d_k = x_{k+1} − x_k − dt/2 (f_k + f_{k+1}), every interval at once
        def dynamics_residual(z, params):
            x, u = unpack_jax(z)
            t = params["t_start"] + grid
            dx = jax.vmap(f, in_axes=(1, 1, 0, None), out_axes=1)(
                x, u, t, params["plant"]
            )
            defect = trapezoidal_defect(x, dx, dt)
            return defect.reshape(-1)

        return unpack_jax, J, dynamics_residual

    def jax_constraints(self, problem: PlanningProblem, unpack_jax, jax):
        """
        The boundary and path constraints as functions of ``(z, params)``: ``(equalities, inequalities)``.

        The start: ``x_0 − x_start = 0`` for the start's singleton ``X0`` (the
        start pinned from ``x_start``), else its margins; the terminal
        set: ``x_N − point = 0`` for a singleton, else its margins; then the
        margins of ``X`` and ``U`` at every knot when they are not lowered to the
        decision bounds. Each set reads its numbers from ``params`` by role.
        """
        jnp = jax.numpy
        grid = self.knot_offsets(problem, jnp)
        equalities = []
        inequalities = []

        # x_0 − x_start = 0: the start is the program's parameter
        if isinstance(problem.X0, SingletonSet):

            def initial_residual(z, params):
                x, _ = unpack_jax(z)
                residual = x[:, 0] - params["x_start"]
                return residual.reshape(-1)

            equalities.append(initial_residual)
        else:
            inequalities.append(knot_margin(problem.X0, "X0", 0, unpack_jax, grid))

        # The terminal set: x_N on its point, or inside it
        Xf = problem.Xf
        if isinstance(Xf, SingletonSet):

            def terminal_residual(z, params):
                x, _ = unpack_jax(z)
                return Xf.residual(x[:, -1], params["Xf"])

            equalities.append(terminal_residual)
        elif Xf is not None:
            inequalities.append(knot_margin(Xf, "Xf", -1, unpack_jax, grid))

        # The path: a set not lowered to the bounds contributes its margins
        roles = constrained_roles(problem)
        if "X" in roles:
            inequalities.append(state_margins(problem.X, unpack_jax, grid))
        if "U" in roles:
            inequalities.append(input_margins(problem.U, unpack_jax, grid))

        return equalities, inequalities

    def knot_offsets(self, problem: PlanningProblem, xp):
        """The knots' offsets from the horizon's start, ``linspace(0, tf, N)``: ``t_k = t_start + offset_k``."""
        return xp.asarray(self.options.offsets(problem))

    def jax_unpacker(self, problem: PlanningProblem):
        """``unpack_jax(z) -> (x, u)``: the knots packed in ``z``, the states row by row, then the inputs."""
        n = int(problem.sys.n)
        m = int(problem.sys.m)
        n_steps = int(self.options.n_steps)

        def unpack_jax(z):
            split = n * n_steps
            x = z[:split].reshape(n, n_steps)
            u = z[split:].reshape(m, n_steps)
            return x, u

        return unpack_jax

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
                    params=None,
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
                        params=None,
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
                        params=None,
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
        cost_params = problem.cost.params

        # J = Σ dt/2 (w_k g_k + w_{k+1} g_{k+1}) + w_N h(x_N, t_N), w_k = e^{−ρ(t_k − t_start)}
        J = trajectory_cost(
            cost,
            x,
            u,
            t,
            dt,
            cost_params,
            discount_rate=problem.discount_rate,
            t_start=problem.t_start,
        )

        return J


###############################################################################
# Internal machinery
###############################################################################


def box_bounds(set_, box_type, boxes_only):
    """The set's bounding box for the decision bounds: any set's, or with ``boxes_only`` a true box's (else ``None``)."""
    if boxes_only and not isinstance(set_, box_type):
        return None

    return set_.bounding_box()


def constrained_roles(problem) -> tuple[str, ...]:
    """The sets the JAX program reads as constraints, by role: ``X`` and ``U`` not lowered to the bounds, ``X0`` not the start's singleton, ``Xf``."""
    roles = []
    if not isinstance(problem.X, BoxSet):
        roles.append("X")
    if not isinstance(problem.U, BoxInputSet):
        roles.append("U")
    if not isinstance(problem.X0, SingletonSet):
        roles.append("X0")
    if problem.Xf is not None:
        roles.append("Xf")

    return tuple(roles)


def knot_margin(set_, role, index, unpack_jax, grid):
    """``margin(x_i, t_i) ≥ 0`` at one knot, the set's numbers from ``params[role]``."""

    def margin(z, params):
        x, _ = unpack_jax(z)
        t_i = params["t_start"] + grid[index]
        return set_.margin(x[:, index], t=t_i, params=params[role])

    return margin


def state_margins(X, unpack_jax, grid):
    """``X.margin(x_k, t_k) ≥ 0`` at every knot, ``X``'s numbers from ``params["X"]``."""

    def margins(z, params):
        x, _ = unpack_jax(z)
        t = params["t_start"] + grid
        values = [
            X.margin(x[:, k], t=t[k], params=params["X"]) for k in range(x.shape[1])
        ]
        return native_concatenate(values, z)

    return margins


def input_margins(U, unpack_jax, grid):
    """``U.margin(u_k, x_k, t_k) ≥ 0`` at every knot, ``U``'s numbers from ``params["U"]``."""

    def margins(z, params):
        x, u = unpack_jax(z)
        t = params["t_start"] + grid
        values = [
            U.margin(u[:, k], x=x[:, k], t=t[k], params=params["U"])
            for k in range(x.shape[1])
        ]
        return native_concatenate(values, z)

    return margins


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
