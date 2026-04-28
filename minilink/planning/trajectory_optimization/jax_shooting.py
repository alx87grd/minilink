"""JAX-backed single-shooting transcription."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.costs import CostFunction, JaxQuadraticCost, QuadraticCost
from minilink.core.sets import BoxInputSet, BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    EqualityConstraint,
    InequalityConstraint,
    MathematicalProgram,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.shooting import (
    ShootingOptions,
    ShootingTranscription,
)


@dataclass
class JaxShootingOptions(ShootingOptions):
    """Fixed-grid shooting options with JAX derivative flags."""

    use_gradient: bool = True
    use_hessian: bool = False

    def __post_init__(self) -> None:
        super().__post_init__()
        self.use_gradient = bool(self.use_gradient)
        self.use_hessian = bool(self.use_hessian)
        if self.use_hessian and not self.use_gradient:
            raise ValueError("use_hessian requires use_gradient")


@dataclass
class JaxShootingTranscription(ShootingTranscription):
    """Single shooting with JAX objective and constraint derivatives."""

    options: JaxShootingOptions

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        compile_backend: str | None = "jax",
    ) -> MathematicalProgram:
        """Build the JAX-differentiable single-shooting program."""
        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()
        self._check_supported_problem(problem, cost)

        t = jnp.asarray(self.options.t)
        dt = jnp.asarray(self.options.dt)
        n_steps = int(self.options.n_steps)
        m = int(problem.sys.m)
        rollout = self._make_jax_rollout(problem, compile_backend, jax, jnp)

        def unpack_jax(z):
            return z.reshape(m, n_steps)

        def objective_jax(z):
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

        objective = jax.jit(objective_jax)
        objective_grad = (
            jax.jit(jax.grad(objective_jax)) if self.options.use_gradient else None
        )
        objective_hess = (
            jax.jit(jax.hessian(objective_jax))
            if self.options.use_hessian
            else None
        )

        equalities = self._terminal_equalities(
            problem,
            jax=jax,
            jnp=jnp,
            unpack_jax=unpack_jax,
            rollout=rollout,
        )
        inequalities = self._path_inequalities(
            problem,
            jax=jax,
            jnp=jnp,
            unpack_jax=unpack_jax,
            rollout=rollout,
        )

        return MathematicalProgram(
            J=self._to_numpy_scalar(objective, jnp),
            z0=self._pack_initial_guess(problem, initial_guess),
            bounds=self.variable_bounds(problem),
            equalities=tuple(equalities),
            inequalities=tuple(inequalities),
            metadata={
                "transcription": "jax_shooting",
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

    def _make_jax_rollout(
        self,
        problem: PlanningProblem,
        compile_backend: str | None,
        jax,
        jnp,
    ):
        x0 = jnp.asarray(self._initial_state(problem))
        t0 = jnp.asarray(float(self.options.t[0]))
        dt = jnp.asarray(self.options.dt)
        params = problem.params.system

        if params is not None or compile_backend is None or compile_backend == "direct":
            return lambda u: self._rollout_direct_jax(
                problem, u, x0, t0, dt, params, jax, jnp
            )

        if compile_backend == "jax":
            evaluator = problem.sys.compile(backend="jax", verbose=False)
            return lambda u: evaluator.rk4_rollout_forced(x0, u.T, t0, dt).T

        raise ValueError(
            "JaxShootingTranscription requires compile_backend='jax', None, or 'direct'"
        )

    def _rollout_direct_jax(self, problem, u, x0, t0, dt, params, jax, jnp):
        def body(carry, u_pair):
            x, t = carry
            u0, u1 = u_pair
            umid = 0.5 * (u0 + u1)
            k1 = problem.sys.f(x, u0, t, params)
            k2 = problem.sys.f(x + 0.5 * dt * k1, umid, t + 0.5 * dt, params)
            k3 = problem.sys.f(x + 0.5 * dt * k2, umid, t + 0.5 * dt, params)
            k4 = problem.sys.f(x + dt * k3, u1, t + dt, params)
            x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            return (x_next, t + dt), x_next

        (_, _), xs = jax.lax.scan(body, (x0, t0), (u[:, :-1].T, u[:, 1:].T))
        return jnp.concatenate((x0[:, None], xs.T), axis=1)

    def _terminal_equalities(
        self,
        problem: PlanningProblem,
        *,
        jax,
        jnp,
        unpack_jax,
        rollout,
    ) -> list[EqualityConstraint]:
        if not isinstance(problem.Xf, SingletonSet):
            return []

        target = jnp.asarray(problem.Xf.point)

        def residual_jax(z):
            x = rollout(unpack_jax(z))
            return x[:, -1] - target

        residual = jax.jit(residual_jax)
        residual_jac = (
            jax.jit(jax.jacfwd(residual_jax))
            if self.options.use_gradient
            else None
        )
        return [
            EqualityConstraint(
                h=self._to_numpy_vector(residual, jnp),
                jac=(
                    self._to_numpy_matrix(residual_jac, jnp)
                    if residual_jac is not None
                    else None
                ),
                name="terminal_state",
            )
        ]

    def _path_inequalities(
        self,
        problem: PlanningProblem,
        *,
        jax,
        jnp,
        unpack_jax,
        rollout,
    ) -> list[InequalityConstraint]:
        inequalities = []

        if isinstance(problem.Xf, BoxSet):
            lower = jnp.asarray(problem.Xf.lower)
            upper = jnp.asarray(problem.Xf.upper)

            def terminal_margin_jax(z):
                x = rollout(unpack_jax(z))
                x_f = x[:, -1]
                return jnp.concatenate((x_f - lower, upper - x_f))

            inequalities.append(
                self._jax_inequality(
                    terminal_margin_jax,
                    "terminal_state",
                    jax,
                    jnp,
                )
            )

        if isinstance(problem.X, BoxSet):
            lower = jnp.asarray(problem.X.lower)
            upper = jnp.asarray(problem.X.upper)

            def state_margin_jax(z):
                x = rollout(unpack_jax(z)).T
                return jnp.concatenate((x - lower, upper - x), axis=1).reshape(-1)

            inequalities.append(
                self._jax_inequality(state_margin_jax, "state_path", jax, jnp)
            )

        return inequalities

    def _jax_inequality(self, margin_jax, name: str, jax, jnp) -> InequalityConstraint:
        margin = jax.jit(margin_jax)
        margin_jac = (
            jax.jit(jax.jacfwd(margin_jax)) if self.options.use_gradient else None
        )
        return InequalityConstraint(
            g=self._to_numpy_vector(margin, jnp),
            jac=(
                self._to_numpy_matrix(margin_jac, jnp)
                if margin_jac is not None
                else None
            ),
            name=name,
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
        if isinstance(cost, QuadraticCost) and not isinstance(cost, JaxQuadraticCost):
            raise ValueError(
                "JAX shooting needs JAX-traceable cost functions in the objective; "
                "for quadratic costs use JaxQuadraticCost instead of QuadraticCost."
            )

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
