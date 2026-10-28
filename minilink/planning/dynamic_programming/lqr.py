"""The linear-quadratic regulator as a planner: the Riccati solution of a quadratic problem at its goal."""

from __future__ import annotations

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
from minilink.core.roles import command_port
from minilink.core.trajectory import interpolate_schedule
from minilink.planning.evaluator import nominal_trajectory
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem, nominal_plant_params
from minilink.planning.results import PlanningSolution

###############################################################################
# Public API
###############################################################################


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
    ``u = ū − K(t) (x − x̄)``.

    Parameters
    ----------
    problem : PlanningProblem
        Its cost must be a :class:`~minilink.core.costs.QuadraticCost`; the
        operating point is the cost's ``(xbar, ubar)`` unless given here. A
        cost over every input stacked weighs the command by its block of
        ``R``.
        A finite ``tf`` selects the finite-horizon design with the cost's
        terminal weight ``S``; an infinite or unset ``tf`` the stationary one.
    x_bar, u_bar : array-like, optional
        Operating point of the linearization; default the cost's target.
    dt : float, optional
        Sampling period of the nominal rollout and the Monte Carlo score
        (``solve(evaluate=True)``); a reporting choice, the law is continuous.
    n_steps : int, optional
        Samples of the finite-horizon gain schedule ``K(t)``.
    t : float, optional
        Time at which the Jacobians are evaluated.
    method : {"auto", "fd", "jax"}, optional
        Differentiation backend of :func:`~minilink.analysis.derivatives.jacobian`.
    eps : float, optional
        Central-difference step.
    """

    # The law is continuous: its score closes it as a continuous loop
    holds_policy = False

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
        cost = self.require_cost()
        require_quadratic_cost(cost)
        self.cost = cost

        # Operating point of the linearization, the cost's target by default
        self.x_bar = np.asarray(cost.xbar if x_bar is None else x_bar, dtype=float)
        self.u_bar = np.asarray(cost.ubar if u_bar is None else u_bar, dtype=float)

        # The rollout's period, the schedule's samples, the linearization options
        self.dt = float(dt)
        self.n_steps = int(n_steps)
        self.t = float(t)
        self.method = method
        self.eps = float(eps)

    def solve(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """
        Solve the Riccati equation of the linearized problem.

        The solution's policy is the state-feedback block, its ``cost_to_go``
        the quadratic form of the linear model. ``evaluate=True`` also rolls
        the law out from the problem's start on the nonlinear plant and scores
        it over the problem's draws.
        """
        problem = self.problem
        sys = problem.sys
        n_steps = self.n_steps
        dt = self.dt

        # The plant linearized at the operating point: A = ∂f/∂x, B = ∂f/∂u
        A, B = self.linear_model()

        # The weights and the operating point over the command port: R[c, c], ū[c]
        Q = self.cost.Q
        R = command_block(sys, self.cost.R)
        S_f = self.cost.S
        x_bar = self.x_bar
        u_bar = command_block(sys, self.u_bar)

        if problem.horizon_kind() == "finite":
            tf = problem.require_finite_tf()

            # −Ṡ = Aᵀ S + S A − S B R⁻¹ Bᵀ S + Q, swept backward from S(t_f) = S_f,
            # and K(t) = R⁻¹ Bᵀ S(t)
            t, K, S = lqr_gain_schedule(A, B, Q, R, S_f, tf, n_steps=n_steps)

            # The law u = ū − K(t) (x − x̄)
            policy = TimeVaryingFeedback(t, K, xbar=x_bar, ubar=u_bar)

            # Closed-loop poles at t = 0: eig(A − B K(0))
            poles = np.linalg.eigvals(A - B @ K[0])

            # The record, and the cost-to-go J(x, t) = (x − x̄)ᵀ S(t) (x − x̄)
            record = RiccatiRecord(
                K=K[0], P=S[0], poles=poles, horizon=tf, schedule=(t, K, S)
            )
            cost_to_go = quadratic_cost_to_go(x_bar, S, t)
        else:
            # Aᵀ P + P A − P B R⁻¹ Bᵀ P + Q = 0
            P = care(A, B, Q, R)

            # K = R⁻¹ Bᵀ P
            K = np.linalg.solve(R, B.T @ P)

            # The law u = ū − K (x − x̄)
            policy = StateFeedback(K, xbar=x_bar, ubar=u_bar)

            # Closed-loop poles: eig(A − B K)
            poles = np.linalg.eigvals(A - B @ K)

            # The record, and the cost-to-go J(x) = (x − x̄)ᵀ P (x − x̄)
            record = RiccatiRecord(K=K, P=P, poles=poles, horizon=np.inf)
            cost_to_go = quadratic_cost_to_go(x_bar, P)

        # Optionally, roll the law out from the problem's start and score it over
        # its draws
        trajectory = None
        evaluation = None
        if evaluate:
            trajectory = nominal_trajectory(problem, policy, dt=dt)
            evaluation = self.evaluate(policy, dt=dt, n_trials=n_trials)

        # The solution: the law, the record, the rollout, the quadratic cost-to-go
        solution = PlanningSolution(
            problem, policy, record, trajectory, evaluation, cost_to_go
        )

        return self.store_solution(solution)

    def solve_policy(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """The planner-family name of :meth:`solve`, answered by every policy planner."""
        return self.solve(evaluate=evaluate, n_trials=n_trials)

    def nominal_trajectory(self, tf=None):
        """The law from the problem's start on the nonlinear plant, sampled every ``dt``."""
        return nominal_trajectory(
            self.problem, self.get_controller(), dt=self.dt, tf=tf
        )

    def linear_model(self):
        """``(A, B)``: the plant's Jacobians at the operating point, ``B`` over its command port."""
        sys = self.problem.sys
        x_bar = self.x_bar
        u_bar = self.u_bar
        t = self.t
        params = nominal_plant_params(self.problem)
        method = self.method
        eps = self.eps

        # A = ∂f/∂x and B = ∂f/∂u at (x̄, ū, t), B over the command port alone
        model = design_model(sys, x_bar, u_bar, t, params, method=method, eps=eps)
        A = model.A()
        B = model.B()

        return A, B


@dataclass(frozen=True)
class RiccatiRecord:
    """
    The Riccati solution: the gain, the cost-to-go matrix, the closed-loop poles.

    ``K`` and ``P`` are the values at ``t = 0``; on a finite horizon
    ``schedule`` holds the whole sweep ``(t, K(t), S(t))``. ``success`` is
    the closed loop of the linear model being stable (every pole in the left
    half-plane) on the infinite horizon, a finite sweep otherwise.
    """

    K: np.ndarray
    P: np.ndarray
    poles: np.ndarray
    horizon: float
    schedule: tuple | None = None

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
        if np.isfinite(self.horizon):
            return f"K(0) = {K} over {self.horizon:g} s, poles at t = 0: {poles}"

        return f"K = {K}, closed-loop poles {poles}"


def quadratic_cost_to_go(x_bar, P, t_samples=None):
    """``J(x) = (x − x̄)ᵀ P (x − x̄)``; with a schedule ``P = S(t)`` on ``t_samples``, ``J(x, t)``."""
    x_bar = np.asarray(x_bar, dtype=float)

    if t_samples is None:

        def J(x, t=0.0):
            dx = np.asarray(x, dtype=float) - x_bar

            # J = dxᵀ P dx
            value = float(dx @ P @ dx)

            return value

    else:

        def J(x, t=0.0):
            dx = np.asarray(x, dtype=float) - x_bar
            S = interpolate_schedule(np, t_samples, P, float(t))

            # J(x, t) = dxᵀ S(t) dx
            value = float(dx @ S @ dx)

            return value

    return J


###############################################################################
# Internal machinery
###############################################################################


def require_quadratic_cost(cost):
    """Refuse a cost that is not a ``QuadraticCost``, the only one the Riccati equation solves."""
    if not isinstance(cost, QuadraticCost):
        raise TypeError(
            "LQRPlanner needs a QuadraticCost, "
            "g = (x - xbar)ᵀ Q (x - xbar) + (u - ubar)ᵀ R (u - ubar); "
            f"got {type(cost).__name__}"
        )


def command_block(sys, value):
    """The command port's block of a weight ``R`` or a point ``ubar`` that spans every input of ``sys``.

    ``R[c, c]`` and ``ubar[c]`` with ``c`` the command's components; a value
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
        plant, Q=np.diag([1.0, 10.0, 1.0, 1.0]), R=np.array([[0.1]]), xbar=x_bar
    )
    problem = PlanningProblem(
        plant, x_start=x0, cost=cost, tf=np.inf, U=plant.input_box
    )

    solution = LQRPlanner(problem).solve(evaluate=True)
    print(solution)

    loop = solution.policy @ plant
    plant.x0 = problem.x_start
    loop.compute_trajectory(tf=8.0)
    loop.plot_trajectory()
