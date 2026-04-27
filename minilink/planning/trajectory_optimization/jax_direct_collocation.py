"""JAX-backed direct-collocation trajectory optimization.

This module builds a trapezoidal direct-collocation ``MathematicalProgram`` whose
objective and equality residuals are ``jax.jit``-compatible. SciPy remains the
outer optimizer; JAX supplies derivatives for the objective and constraints.

**Supported problems** are a strict subset of
:class:`~minilink.planning.trajectory_optimization.direct_collocation.DirectCollocationTranscription`:
use :class:`~minilink.core.sets.SingletonSet` for initial and terminal state
boundaries, and :class:`~minilink.core.sets.BoxSet` /
:class:`~minilink.core.sets.BoxInputSet` for path constraints expressed as
variable bounds. Pair with :class:`~minilink.core.costs.JaxQuadraticCost` (not
:class:`~minilink.core.costs.QuadraticCost`) for the usual quadratic cost.

**TODO: User Architectural Review** — set coverage and the SciPy+JAX split are
prototype choices pending approval. Configure process-wide JAX precision with
:func:`minilink.compile.jax_utils.configure_jax` before constructing JAX systems.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.costs import CostFunction, JaxQuadraticCost, QuadraticCost
from minilink.core.sets import BoxInputSet, BoxSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.optimization.mathematical_program import (
    EqualityConstraint,
    MathematicalProgram,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.transcription import (
    TranscriptionContext,
)


@dataclass(frozen=True)
class JaxDirectCollocationOptions(DirectCollocationOptions):
    """
    Options for JAX-backed direct collocation.

    ``use_gradient`` supplies SciPy with JAX objective/constraint Jacobians.
    ``use_hessian`` also attaches a dense JAX objective Hessian for SciPy
    methods that consume it (``dogleg``, ``trust-*``, ``trust-constr``;
    see :class:`~minilink.optimization.optimizers.scipy_minimize.ScipyMinimizeOptimizer`).

    Notes
    -----
    ``use_hessian`` builds a full :math:`n_z \times n_z` objective Hessian and is
    practical only for small decision vectors.
    """

    use_gradient: bool = True
    use_hessian: bool = False

    def __post_init__(self) -> None:
        super().__post_init__()
        object.__setattr__(self, "use_gradient", bool(self.use_gradient))
        object.__setattr__(self, "use_hessian", bool(self.use_hessian))
        if self.use_hessian and not self.use_gradient:
            raise ValueError("use_hessian requires use_gradient")


@dataclass(frozen=True)
class JaxDirectCollocationTranscription(DirectCollocationTranscription):
    """
    Direct-collocation transcription with JAX-derived first (and optional
    second) derivatives. See the module docstring for supported sets and the
    :class:`~minilink.core.costs.JaxQuadraticCost` requirement for quadratic
    running/terminal cost.
    """

    options: JaxDirectCollocationOptions

    @staticmethod
    def _validate_jax_traceable_quadratic_pairing(cost: CostFunction) -> None:
        if isinstance(cost, QuadraticCost) and not isinstance(cost, JaxQuadraticCost):
            raise ValueError(
                "JAX direct collocation needs JAX-traceable cost functions in the "
                "objective; for quadratic costs use JaxQuadraticCost instead of "
                "QuadraticCost."
            )

    def transcribe(
        self,
        problem: PlanningProblem,
        *,
        initial_guess: np.ndarray | Trajectory | None = None,
        context: TranscriptionContext | None = None,
    ) -> MathematicalProgram:
        """Convert ``problem`` into a differentiable mathematical program."""
        import jax
        import jax.numpy as jnp

        cost = problem.require_cost()
        self._validate_jax_traceable_quadratic_pairing(cost)
        self._check_supported_sets(problem)

        return self._transcribe_core(
            problem,
            jax,
            jnp,
            cost,
            problem.params.cost,
            initial_guess=initial_guess,
            context=TranscriptionContext("jax") if context is None else context,
        )

    def _transcribe_core(
        self,
        problem: PlanningProblem,
        jax,
        jnp,
        cost: CostFunction,
        cost_params,
        *,
        initial_guess: np.ndarray | Trajectory | None,
        context: TranscriptionContext,
    ) -> MathematicalProgram:
        t = jnp.asarray(self.options.t)
        dt = jnp.asarray(self.options.dt)
        n = int(problem.sys.n)
        n_steps = int(self.options.n_steps)
        f = self._make_jax_dynamics(problem, context.compile_backend)

        def unpack_jax(z):
            x = z[: n * n_steps].reshape(n, n_steps)
            u = z[n * n_steps :].reshape(problem.sys.m, n_steps)
            return x, u

        def objective_jax(z):
            x, u = unpack_jax(z)
            running = jax.vmap(
                lambda x_k, u_k, t_k: cost.g(
                    x_k,
                    u_k,
                    t_k,
                    params=cost_params,
                ),
                in_axes=(1, 1, 0),
            )(x, u, t)
            integral = jnp.sum(0.5 * dt * (running[:-1] + running[1:]))
            terminal = cost.h(x[:, -1], t[-1], params=cost_params)
            return integral + terminal

        def dynamics_residual_jax(z):
            x, u = unpack_jax(z)
            dx = jax.vmap(f, in_axes=(1, 1, 0), out_axes=1)(x, u, t)
            residuals = x[:, 1:] - x[:, :-1] - 0.5 * dt * (
                dx[:, :-1] + dx[:, 1:]
            )
            return residuals.reshape(-1)

        objective_jit = jax.jit(objective_jax)
        dynamics_residual_jit = jax.jit(dynamics_residual_jax)
        objective_grad_jit = (
            jax.jit(jax.grad(objective_jax)) if self.options.use_gradient else None
        )
        dynamics_jac_jit = (
            jax.jit(jax.jacfwd(dynamics_residual_jax))
            if self.options.use_gradient
            else None
        )
        objective_hess_jit = (
            jax.jit(jax.hessian(objective_jax))
            if self.options.use_hessian
            else None
        )

        equalities = [
            EqualityConstraint(
                h=self._to_numpy_vector(dynamics_residual_jit, jnp),
                jac=(
                    self._to_numpy_matrix(dynamics_jac_jit, jnp)
                    if dynamics_jac_jit is not None
                    else None
                ),
                name="jax_direct_collocation_dynamics",
                metadata={"scheme": "trapezoidal", "backend": "jax"},
            )
        ]
        self._add_jax_boundary_constraints(
            problem,
            equalities=equalities,
            jax=jax,
            jnp=jnp,
            unpack_jax=unpack_jax,
        )

        return MathematicalProgram(
            J=self._to_numpy_scalar(objective_jit, jnp),
            z0=self._pack_initial_guess(problem, initial_guess),
            bounds=self.variable_bounds(problem),
            equalities=tuple(equalities),
            inequalities=(),
            metadata={
                "transcription": "jax_direct_collocation",
                "integration_scheme": "trapezoidal",
                "tf": self.options.tf,
                "dt": self.options.dt,
                "n_steps": self.options.n_steps,
                "state_dim": int(problem.sys.n),
                "input_dim": int(problem.sys.m),
                "use_gradient": self.options.use_gradient,
                "use_hessian": self.options.use_hessian,
                "compile_backend": context.compile_backend,
            },
            grad=(
                self._to_numpy_vector(objective_grad_jit, jnp)
                if objective_grad_jit is not None
                else None
            ),
            hess=(
                self._to_numpy_matrix(objective_hess_jit, jnp)
                if objective_hess_jit is not None
                else None
            ),
        )

    def _make_jax_dynamics(self, problem: PlanningProblem, compile_backend: str | None):
        backend = compile_backend
        params = problem.params.system

        if backend == "jax":
            evaluator = problem.sys.compile(backend="jax", verbose=False)
            return lambda x, u, t: evaluator.f(x, u, t)

        if backend is None or backend == "direct":
            return lambda x, u, t: problem.sys.f(x, u, t, params)

        raise ValueError(
            "JaxDirectCollocationTranscription requires compile_backend='jax', "
            "None, or 'direct'"
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

    def _add_jax_boundary_constraints(
        self,
        problem: PlanningProblem,
        *,
        equalities: list[EqualityConstraint],
        jax,
        jnp,
        unpack_jax,
    ) -> None:
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

            residual_jit = jax.jit(residual_jax)
            residual_jac_jit = (
                jax.jit(jax.jacfwd(residual_jax))
                if self.options.use_gradient
                else None
            )
            equalities.append(
                EqualityConstraint(
                    h=self._to_numpy_vector(residual_jit, jnp),
                    jac=(
                        self._to_numpy_matrix(residual_jac_jit, jnp)
                        if residual_jac_jit is not None
                        else None
                    ),
                    name=name,
                    metadata={"backend": "jax"},
                )
            )

    @staticmethod
    def _check_supported_sets(problem: PlanningProblem) -> None:
        for label, boundary in (("X0", problem.X0), ("Xf", problem.Xf)):
            if boundary is not None and not isinstance(boundary, SingletonSet):
                raise NotImplementedError(
                    f"JAX direct collocation currently supports only SingletonSet "
                    f"boundaries; got {label}={type(boundary).__name__}"
                )

        if problem.X is not None and not isinstance(problem.X, BoxSet):
            raise NotImplementedError(
                "JAX direct collocation currently supports only BoxSet state "
                "path constraints through variable bounds"
            )

        if problem.U is not None and not isinstance(problem.U, BoxInputSet):
            raise NotImplementedError(
                "JAX direct collocation currently supports only BoxInputSet input "
                "path constraints through variable bounds"
            )
