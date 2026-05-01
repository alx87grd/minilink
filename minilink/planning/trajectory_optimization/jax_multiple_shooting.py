"""JAX-backed multiple-shooting transcription."""

from dataclasses import dataclass

import numpy as np

from minilink.core.costs import CostFunction, require_jax_traceable_cost
from minilink.core.sets import BoxInputSet, BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    EqualityConstraint,
    MathematicalProgram,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.multiple_shooting import (
    MultipleShootingOptions,
    MultipleShootingTranscription,
)


@dataclass
class JaxMultipleShootingOptions(MultipleShootingOptions):
    """Fixed-grid multiple-shooting options with JAX derivative flags."""

    use_gradient: bool = True
    use_hessian: bool = False

    def __post_init__(self) -> None:
        super().__post_init__()
        self.use_gradient = bool(self.use_gradient)
        self.use_hessian = bool(self.use_hessian)
        if self.use_hessian and not self.use_gradient:
            raise ValueError("use_hessian requires use_gradient")


class JaxMultipleShootingTranscription(MultipleShootingTranscription):
    """Multiple shooting with JAX objective and constraint derivatives."""

    def __init__(self, options: JaxMultipleShootingOptions):
        self.options = options

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        compile_backend: str | None = "jax",
    ) -> MathematicalProgram:
        """Build the JAX-differentiable multiple-shooting program."""
        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()
        self._check_supported_problem(problem, cost)

        t = jnp.asarray(self.options.t)
        dt = jnp.asarray(self.options.dt)
        n = int(problem.sys.n)
        m = int(problem.sys.m)
        n_steps = int(self.options.n_steps)
        f = self._make_jax_dynamics(problem, compile_backend)

        def unpack_jax(z):
            split = n * n_steps
            x = z[:split].reshape(n, n_steps)
            u = z[split:].reshape(m, n_steps)
            return x, u

        def rk4_step(x_k, u0, u1, t_k):
            umid = 0.5 * (u0 + u1)
            k1 = f(x_k, u0, t_k)
            k2 = f(x_k + 0.5 * dt * k1, umid, t_k + 0.5 * dt)
            k3 = f(x_k + 0.5 * dt * k2, umid, t_k + 0.5 * dt)
            k4 = f(x_k + dt * k3, u1, t_k + dt)
            return x_k + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        def objective_jax(z):
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

        def dynamics_residual_jax(z):
            x, u = unpack_jax(z)
            x_next = jax.vmap(rk4_step, in_axes=(1, 1, 1, 0), out_axes=1)(
                x[:, :-1],
                u[:, :-1],
                u[:, 1:],
                t[:-1],
            )
            return (x[:, 1:] - x_next).reshape(-1)

        objective = jax.jit(objective_jax)
        residual = jax.jit(dynamics_residual_jax)
        objective_grad = (
            jax.jit(jax.grad(objective_jax)) if self.options.use_gradient else None
        )
        residual_jac = (
            jax.jit(jax.jacfwd(dynamics_residual_jax))
            if self.options.use_gradient
            else None
        )
        objective_hess = (
            jax.jit(jax.hessian(objective_jax)) if self.options.use_hessian else None
        )

        equalities = [
            EqualityConstraint(
                h=self._to_numpy_vector(residual, jnp),
                jac=(
                    self._to_numpy_matrix(residual_jac, jnp)
                    if residual_jac is not None
                    else None
                ),
                name="jax_multiple_shooting_dynamics",
            )
        ]
        self._add_boundary_equalities(problem, equalities, jax, jnp, unpack_jax)

        return MathematicalProgram(
            J=self._to_numpy_scalar(objective, jnp),
            z0=self._pack_initial_guess(problem, initial_guess),
            bounds=self.variable_bounds(problem),
            equalities=tuple(equalities),
            inequalities=(),
            metadata={
                "transcription": "jax_multiple_shooting",
                "use_gradient": self.options.use_gradient,
                "use_hessian": self.options.use_hessian,
                "compile_backend": compile_backend,
            },
            grad=(
                self._to_numpy_vector(objective_grad, jnp)
                if objective_grad is not None
                else None
            ),
            hess=(
                self._to_numpy_matrix(objective_hess, jnp)
                if objective_hess is not None
                else None
            ),
        )

    def _make_jax_dynamics(self, problem: PlanningProblem, compile_backend: str | None):
        params = problem.params.system
        if params is not None or compile_backend is None or compile_backend == "direct":
            return lambda x, u, t: problem.sys.f(x, u, t, params)

        if compile_backend == "jax":
            evaluator = problem.sys.compile(backend="jax", verbose=False)
            return lambda x, u, t: evaluator.f(x, u, t)

        raise ValueError(
            "JaxMultipleShootingTranscription requires compile_backend='jax', "
            "None, or 'direct'"
        )

    def _add_boundary_equalities(self, problem, equalities, jax, jnp, unpack_jax):
        for name, boundary, index in (
            ("initial_state", problem.X0, 0),
            ("terminal_state", problem.Xf, -1),
        ):
            if boundary is None:
                continue
            point = jnp.asarray(boundary.point)

            def residual_jax(z, index=index, point=point):
                x, _ = unpack_jax(z)
                return x[:, index] - point

            residual = jax.jit(residual_jax)
            residual_jac = (
                jax.jit(jax.jacfwd(residual_jax)) if self.options.use_gradient else None
            )
            equalities.append(
                EqualityConstraint(
                    h=self._to_numpy_vector(residual, jnp),
                    jac=(
                        self._to_numpy_matrix(residual_jac, jnp)
                        if residual_jac is not None
                        else None
                    ),
                    name=name,
                )
            )

    @staticmethod
    def _to_numpy_scalar(fn, jnp):
        return lambda z: float(np.asarray(fn(jnp.asarray(z))))

    @staticmethod
    def _to_numpy_vector(fn, jnp):
        return lambda z: np.asarray(fn(jnp.asarray(z)), dtype=float).reshape(-1)

    @staticmethod
    def _to_numpy_matrix(fn, jnp):
        return lambda z: np.asarray(fn(jnp.asarray(z)), dtype=float)

    @staticmethod
    def _check_supported_problem(problem: PlanningProblem, cost: CostFunction) -> None:
        require_jax_traceable_cost(cost)

        for label, boundary in (("X0", problem.X0), ("Xf", problem.Xf)):
            if boundary is not None and not isinstance(boundary, SingletonSet):
                raise NotImplementedError(
                    "JAX multiple shooting currently supports only SingletonSet "
                    f"boundaries; got {label}={type(boundary).__name__}"
                )

        if problem.X is not None and not isinstance(problem.X, BoxSet):
            raise NotImplementedError(
                "JAX multiple shooting currently supports only BoxSet state "
                "path constraints through variable bounds"
            )

        if problem.U is not None and not isinstance(problem.U, BoxInputSet):
            raise NotImplementedError(
                "JAX multiple shooting currently supports only BoxInputSet input "
                "path constraints through variable bounds"
            )
