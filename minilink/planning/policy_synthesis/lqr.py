"""
The linear-quadratic regulator as a planner: the Riccati solution of a quadratic problem at its goal.

:class:`LQRPlanner` lowers a :class:`~minilink.planning.problems.PlanningProblem`
whose cost is a :class:`~minilink.core.costs.QuadraticCost` to the matrices
``(A, B, Q, R)`` of the plant linearized at ``(x̄, ū)`` and solves the Riccati
equation with the factories of :mod:`minilink.control.lqr`. Its solution
carries the law, ``u = ū − K (x − x̄)``, and what the method itself knows
about it: the cost-to-go ``J(x) = (x − x̄)ᵀ P (x − x̄)`` of the linear model and
the closed-loop poles. The problem's horizon picks the equation: infinite,
the algebraic one; finite, the differential one swept backward from the
terminal weight ``S``, and the law is then ``u = ū − K(t) (x − x̄)``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_continuous_are

from minilink.control.lqr import lqr_gain_schedule
from minilink.control.state import (
    StateFeedbackController,
    TimeVaryingStateFeedbackController,
    interpolate_schedule,
)
from minilink.core.costs import QuadraticCost
from minilink.planning.evaluation import nominal_trajectory
from minilink.planning.planner import Planner
from minilink.planning.problems import PlanningProblem
from minilink.planning.results import PlanningSolution

# Public API


class LQRPlanner(Planner):
    """
    Linear-quadratic regulator design as a planner.

    Parameters
    ----------
    problem : PlanningProblem
        Its cost must be a :class:`~minilink.core.costs.QuadraticCost`; the
        operating point is the cost's ``(xbar, ubar)`` unless given here.
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
        cost = self.require_cost()
        if not isinstance(cost, QuadraticCost):
            raise TypeError(
                "LQRPlanner needs a QuadraticCost, "
                "g = (x - xbar)ᵀ Q (x - xbar) + (u - ubar)ᵀ R (u - ubar); "
                f"got {type(cost).__name__}"
            )
        self.cost = cost
        self.x_bar = np.asarray(cost.xbar if x_bar is None else x_bar, dtype=float)
        self.u_bar = np.asarray(cost.ubar if u_bar is None else u_bar, dtype=float)
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
        return self.solve_policy(evaluate=evaluate, n_trials=n_trials)

    def solve_policy(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """Policy-family name of :meth:`solve`."""
        A, B = self.linear_model()
        Q, R, S_f = self.cost.Q, self.cost.R, self.cost.S
        x_bar, u_bar = self.x_bar, self.u_bar

        if self.problem.horizon_kind() == "finite":
            tf = self.problem.require_finite_tf()
            t, K, S = lqr_gain_schedule(A, B, Q, R, S_f, tf, n_steps=self.n_steps)
            policy = TimeVaryingStateFeedbackController(t, K, xbar=x_bar, ubar=u_bar)
            record = RiccatiRecord(
                K=K[0],
                P=S[0],
                poles=np.linalg.eigvals(A - B @ K[0]),
                horizon=tf,
                schedule=(t, K, S),
            )
            cost_to_go = quadratic_cost_to_go(x_bar, S, t)
        else:
            P = solve_continuous_are(A, B, Q, R)
            K = np.linalg.solve(R, B.T @ P)
            policy = StateFeedbackController(K, xbar=x_bar, ubar=u_bar)
            record = RiccatiRecord(
                K=K, P=P, poles=np.linalg.eigvals(A - B @ K), horizon=np.inf
            )
            cost_to_go = quadratic_cost_to_go(x_bar, P)

        trajectory = evaluation = None
        if evaluate:
            trajectory = nominal_trajectory(self.problem, policy, dt=self.dt)
            evaluation = self.evaluate(policy, dt=self.dt, n_trials=n_trials)
        return self.store_solution(
            PlanningSolution(
                self.problem, policy, record, trajectory, evaluation, cost_to_go
            )
        )

    def nominal_trajectory(self, tf=None):
        """The law from the problem's start on the nonlinear plant, sampled every ``dt``."""
        return nominal_trajectory(
            self.problem, self.get_controller(), dt=self.dt, tf=tf
        )

    def linear_model(self):
        """``(A, B)``: the plant's Jacobians at the operating point."""
        from minilink.analysis.derivatives import jacobian

        sys, x_bar, u_bar, t = self.problem.sys, self.x_bar, self.u_bar, self.t
        params = self.problem.params.system
        method, eps = self.method, self.eps
        A = jacobian(sys, "f", "x", x_bar, u_bar, t, params, method=method, eps=eps)
        B = jacobian(sys, "f", "u", x_bar, u_bar, t, params, method=method, eps=eps)
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
        if np.isfinite(self.horizon):
            return bool(np.all(np.isfinite(self.schedule[2])))
        return bool(np.all(np.real(self.poles) < 0.0))

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


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.cartpole import CartPole

    plant = CartPole()
    x_bar = np.array([0.0, np.pi, 0.0, 0.0])  # pole inverted, cart at origin
    x0 = np.array([-1.0, np.pi + 0.3, 0.0, 0.0])
    cost = QuadraticCost.from_system(
        plant, Q=np.diag([1.0, 10.0, 1.0, 1.0]), R=np.array([[0.1]]), xbar=x_bar
    )
    problem = PlanningProblem(plant, x_start=x0, cost=cost, tf=np.inf)

    solution = LQRPlanner(problem).solve(evaluate=True)
    print(solution)

    loop = solution.policy @ plant
    plant.x0 = problem.x_start
    loop.compute_trajectory(tf=8.0)
    loop.plot_trajectory()
