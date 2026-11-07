"""The linear-quadratic regulator as a planner: the Riccati solution of a quadratic problem at its goal."""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np

from minilink.analysis.linearization import design_model
from minilink.analysis.riccati import care
from minilink.control.design import lqr_gain_schedule
from minilink.control.state import (
    StateFeedback,
    TimeVaryingFeedback,
)
from minilink.core.costs import QuadraticCost
from minilink.core.fields import QuadraticField
from minilink.core.roles import command_port
from minilink.core.sets import is_bounded
from minilink.planning.planner import (
    Planner,
    given_numbers,
    planner_at,
    planner_evidence,
)
from minilink.planning.problems import (
    PlanningProblem,
    nominal_plant_params,
    require_cost,
    require_finite_horizon,
)
from minilink.planning.results import PlanningSolution

###############################################################################
# Public API
###############################################################################


def lqr_matrices(
    sys, x_bar, u_bar, Q, R, S, *, t=0.0, params=None, method="auto", eps=1e-6
):
    """
    ``(A, B, Q, R, S)``: a quadratic problem lowered to the textbook LQR matrices at ``(x̄, ū, t)``.

    The plant linearized at the operating point, ``B`` over its command port
    alone (a disturbance ``d`` or a noise ``n`` is never decided), and the
    weights over that port: ``R[c, c]`` of a weight that spans every input.
    ``params`` are the plant's numbers to linearize with (its own for
    ``None``); ``method`` and ``eps`` the differentiation of
    :func:`~minilink.analysis.derivatives.jacobian`. The Riccati solvers of
    :mod:`minilink.control.design` take the result.
    """
    # A = ∂f/∂x and B = ∂f/∂u at (x̄, ū, t), B over the command port alone
    model = design_model(sys, x_bar, u_bar, t, params, method=method, eps=eps)
    A = model.A()
    B = model.B()

    # The input weight over the command port: R[c, c]
    R = command_block(sys, R)

    return A, B, Q, R, S


class LQRPlanner(Planner):
    """
    Linear-quadratic regulator design as a planner.

    Lowers a :class:`~minilink.planning.problems.PlanningProblem` whose cost is
    a :class:`~minilink.core.costs.QuadraticCost` to the matrices
    ``(A, B, Q, R)`` of the plant linearized at ``(x̄, ū)`` about its command
    port (a disturbance ``d`` or a noise ``n`` is never decided) and solves the
    Riccati equation with the factories of :mod:`minilink.control.design`. Its
    solution carries the law, ``u = ū − K (x − x̄)``, and what the method itself
    knows about it: the cost-to-go ``J(x) = (x − x̄)ᵀ P (x − x̄)`` of the linear
    model and the closed-loop poles. The problem's horizon picks the equation:
    infinite, the algebraic one; finite, the differential one swept backward
    from the terminal weight ``S``, and the law is then
    ``u = ū − K(t) (x − x̄)`` on the problem's clock ``t0 … t0 + horizon``, and
    past it the stationary law ``K∞`` at the operating point.

    Parameters
    ----------
    problem : PlanningProblem
        Its cost must be a :class:`~minilink.core.costs.QuadraticCost`; the
        operating point is the cost's ``(x_bar, u_bar)`` unless given here. A
        cost over every input stacked weighs the command by its block of
        ``R``.
        A finite ``horizon`` selects the finite-horizon design with the
        cost's terminal weight ``S``; an infinite or unset one the stationary
        design.
    x_bar, u_bar : array-like, optional
        Operating point of the linearization; default the cost's target.
    dt : float, optional
        Sampling period of the score's continuous loop (``solve(evaluate=True)``),
        the grid it is integrated and recorded on; a reporting choice, the law
        is continuous.
    n_steps : int, optional
        Samples of the finite-horizon gain schedule ``K(t)``.
    t : float, optional
        Time at which the Jacobians are evaluated.
    method : {"auto", "fd", "jax"}, optional
        Differentiation backend of :func:`~minilink.analysis.derivatives.jacobian`.
    eps : float, optional
        Central-difference step.
    """

    number_names = ("x_bar", "u_bar", "t")

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        x_bar=None,
        u_bar=None,
        dt: float = 0.01,
        n_steps: int = 1001,
        t: float = 0.0,
        method: str = "auto",
        eps: float = 1e-6,
    ) -> None:
        super().__init__(problem)

        # The quadratic cost: its weights and its target
        cost = require_cost(self.problem)
        require_quadratic_cost(cost)
        self.cost = cost

        # Operating point of the linearization, the cost's target by default
        self.x_bar = np.asarray(cost.x_bar if x_bar is None else x_bar, dtype=float)
        self.u_bar = np.asarray(cost.u_bar if u_bar is None else u_bar, dtype=float)
        self.given = given_numbers(x_bar=x_bar, u_bar=u_bar)

        # The rollout's period, the schedule's samples, the linearization options
        self.dt = float(dt)
        self.n_steps = int(n_steps)
        self.t = float(t)
        self.method = method
        self.eps = float(eps)

    def solve(self, *, params=None, evaluate=False, n_trials=50) -> PlanningSolution:
        """
        Solve the Riccati equation of the linearized problem.

        The solution's controller is the state-feedback block, its ``cost_to_go``
        the quadratic form of the linear model. ``evaluate=True`` also scores
        the law, as it is, on the nonlinear plant over the problem's draws (the
        continuous loop on the grid of ``dt``).
        ``params`` (the complete tree of :attr:`params`, a copy edited) solves
        the planner rebuilt there (:func:`~minilink.planning.planner.planner_at`), this one unchanged.
        """
        # At another tree: the planner there, its solution kept here
        if params is not None:
            solution = planner_at(self, params).solve(
                evaluate=evaluate, n_trials=n_trials
            )
            return self.store_solution(solution)

        problem = self.problem
        sys = problem.sys
        n_steps = self.n_steps
        dt = self.dt
        rho = problem.discount_rate

        # The problem lowered at the operating point: A = ∂f/∂x, B = ∂f/∂u, the
        # weights over the command port
        A, B, Q, R, S_f = self.lqr_matrices()

        # An e^{−ρt}-discounted cost is the undiscounted one on A_ρ = A − (ρ/2) I (in
        # x̃ = e^{−ρt/2} x, ũ = e^{−ρt/2} u, the same law); A_ρ = A undiscounted
        if rho > 0.0:
            warn_discounted_lqr(rho)
        A_rho = A - 0.5 * rho * np.eye(A.shape[0])

        # The operating point, ū over the command port: ū[c]
        x_bar = self.x_bar
        u_bar = command_block(sys, self.u_bar)

        if problem.horizon_kind() == "finite":
            T = require_finite_horizon(problem)

            # −Ṡ = A_ρᵀ S + S A_ρ − S B R⁻¹ Bᵀ S + Q, swept backward from S(T) = S_f,
            # and K(t) = R⁻¹ Bᵀ S(t), on the problem's clock t0 … t0 + T
            s, K, S = lqr_gain_schedule(A_rho, B, Q, R, S_f, T, n_steps=n_steps)
            t = problem.t0 + s

            # Past the horizon, the stationary law at the operating point: K∞ of
            # A_ρᵀ P + P A_ρ − P B R⁻¹ Bᵀ P + Q = 0 (K(T) held when there is none)
            K_after = stationary_gain(A_rho, B, Q, R)

            # The law u = ū − K(t) (x − x̄)
            controller = TimeVaryingFeedback(
                t, K, x_bar=x_bar, u_bar=u_bar, K_after=K_after
            )

            # Closed-loop poles at the horizon's start: eig(A − B K(t0))
            poles = np.linalg.eigvals(A - B @ K[0])

            # The record, and the cost-to-go J(x, t) = (x − x̄)ᵀ S(t) (x − x̄)
            record = RiccatiRecord(
                K=K[0],
                P=S[0],
                poles=poles,
                horizon=T,
                schedule=(t, K, S),
                U_ignored=is_bounded(problem.U.bounding_box()),
            )
            cost_to_go = QuadraticField(x_bar, S, t)
        else:
            # A_ρᵀ P + P A_ρ − P B R⁻¹ Bᵀ P + Q = 0
            P = care(A_rho, B, Q, R)

            # K = R⁻¹ Bᵀ P
            K = np.linalg.solve(R, B.T @ P)

            # The law u = ū − K (x − x̄)
            controller = StateFeedback(K, x_bar=x_bar, u_bar=u_bar)

            # Closed-loop poles: eig(A − B K)
            poles = np.linalg.eigvals(A - B @ K)

            # The record, and the cost-to-go J(x) = (x − x̄)ᵀ P (x − x̄)
            record = RiccatiRecord(
                K=K,
                P=P,
                poles=poles,
                horizon=np.inf,
                U_ignored=is_bounded(problem.U.bounding_box()),
            )
            cost_to_go = QuadraticField(x_bar, P)

        # Optionally, the law, as it is, scored over the problem's draws
        evaluation = None
        if evaluate:
            evaluation = planner_evidence(self, controller, dt=dt, n_trials=n_trials)

        # The solution: the law, the record, the quadratic cost-to-go, the score
        solution = PlanningSolution(
            problem=problem,
            controller=controller,
            record=record,
            cost_to_go=cost_to_go,
            evaluation=evaluation,
        )

        return self.store_solution(solution)

    def build_kwargs(self) -> dict:
        """The keywords that rebuild this planner on another problem: the schedule and the linearization, the operating point it was given."""
        kwargs = dict(
            dt=self.dt, n_steps=self.n_steps, t=self.t, method=self.method, eps=self.eps
        )

        return {**kwargs, **self.given}

    def linear_model(self):
        """``(A, B)``: the plant's Jacobians at the operating point, ``B`` over its command port."""
        A, B, _, _, _ = self.lqr_matrices()

        return A, B

    def lqr_matrices(self):
        """``(A, B, Q, R, S)`` of :func:`lqr_matrices` at this planner's operating point, the plant at its design numbers."""
        cost = self.cost
        params = nominal_plant_params(self.problem)

        return lqr_matrices(
            self.problem.sys,
            self.x_bar,
            self.u_bar,
            cost.Q,
            cost.R,
            cost.S,
            t=self.t,
            params=params,
            method=self.method,
            eps=self.eps,
        )


@dataclass(frozen=True)
class RiccatiRecord:
    """
    The Riccati solution: the gain, the cost-to-go matrix, the closed-loop poles.

    ``K`` and ``P`` are the values at the horizon's start; on a finite horizon
    ``schedule`` holds the whole sweep ``(t, K(t), S(t))``. ``success`` is
    the closed loop of the linear model being stable (every pole in the left
    half-plane) on the infinite horizon, a finite sweep otherwise.
    ``U_ignored`` is the problem declaring an input set that bounds the
    command: the Riccati law never reads it, and the printout says so.
    """

    K: np.ndarray
    P: np.ndarray
    poles: np.ndarray
    horizon: float
    schedule: tuple | None = None
    U_ignored: bool = False

    @property
    def success(self) -> bool:
        """A finite sweep on a finite horizon; a stable linear closed loop otherwise."""
        if np.isfinite(self.horizon):
            return bool(np.all(np.isfinite(self.schedule[2])))

        return bool(np.all(np.real(self.poles) < 0.0))

    # Display

    def __str__(self) -> str:
        K = np.array2string(np.round(self.K, 3), separator=", ").replace("\n", "")
        poles = ", ".join(f"{p:.3g}" for p in np.round(self.poles, 3))
        ignored = "; U ignored" if self.U_ignored else ""
        if np.isfinite(self.horizon):
            return (
                f"K(0) = {K} over {self.horizon:g} s, poles at t = 0: {poles}{ignored}"
            )

        return f"K = {K}, closed-loop poles {poles}{ignored}"


def stationary_gain(A, B, Q, R):
    """
    The infinite-horizon gain ``K∞ = R⁻¹ Bᵀ P`` of ``Aᵀ P + P A − P B R⁻¹ Bᵀ P + Q = 0``; ``None`` without a stabilizing solution.

    A finite horizon needs no stabilizability, so a pair the algebraic
    equation cannot solve keeps its last gain past the horizon instead.
    """
    try:
        P = care(A, B, Q, R)
    except (np.linalg.LinAlgError, ValueError):
        return None

    # K∞ = R⁻¹ Bᵀ P
    K = np.linalg.solve(R, B.T @ P)

    return K


###############################################################################
# Internal machinery
###############################################################################


def require_quadratic_cost(cost):
    """Refuse a cost that is not a ``QuadraticCost``, the only one the Riccati equation solves."""
    if not isinstance(cost, QuadraticCost):
        raise TypeError(
            "LQRPlanner needs a QuadraticCost, "
            "g = (x - x_bar)ᵀ Q (x - x_bar) + (u - u_bar)ᵀ R (u - u_bar); "
            f"got {type(cost).__name__}"
        )


def warn_discounted_lqr(rho):
    """Warn that the problem is discounted, unusual for LQR, and how it is solved."""
    warnings.warn(
        f"LQRPlanner on a discounted problem (discount_rate={rho:g}): solved as the "
        "undiscounted problem on A − (ρ/2) I, the textbook equivalent; the closed-loop "
        "poles are the plant's, eig(A − B K)",
        stacklevel=3,
    )


def command_block(sys, value):
    """The command port's block of a weight ``R`` or a point ``u_bar`` that spans every input of ``sys``.

    ``R[c, c]`` and ``u_bar[c]`` with ``c`` the command's components; a value
    already of the command's size is returned as is.
    """
    value = np.asarray(value, dtype=float)
    if value.shape[0] != sys.m:
        return value

    c = sys.get_input_port_slice(command_port(sys))

    return value[c, c] if value.ndim == 2 else value[c]


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.cartpole import CartPole

    plant = CartPole()
    x_bar = np.array([0.0, np.pi, 0.0, 0.0])  # Pole inverted, cart at origin
    x0 = np.array([-1.0, np.pi + 0.3, 0.0, 0.0])
    cost = QuadraticCost.from_system(
        plant, Q=np.diag([1.0, 10.0, 1.0, 1.0]), R=np.array([[0.1]]), x_bar=x_bar
    )
    problem = PlanningProblem(
        plant, x0=x0, cost=cost, horizon=np.inf, U=plant.input_box
    )

    solution = LQRPlanner(problem).solve(evaluate=True)
    print(solution)

    closed_loop = solution.controller @ plant
    plant.x0 = problem.x0
    closed_loop.compute_trajectory(tf=8.0)
    closed_loop.plot_trajectory()
