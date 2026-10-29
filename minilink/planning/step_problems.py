"""Step planning problems: a task on a step system, a horizon of ticks, a stage cost."""

from __future__ import annotations

import numpy as np

from minilink.core.costs import SampledCost, StepCost, box_samples
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.params import merge_tree
from minilink.planning.problems import (
    PlanningProblem,
    fallback_bound,
    finite_box_or,
)

###############################################################################
# Public API
###############################################################################


class StepPlanningProblem:
    """
    A planning problem on a step system: ``x_{k+1} = step(x_k, u_k, k)``, ``N`` ticks, a discount ``γ`` per tick.

    The objective is ``J = Σ_{k<N} γᵏ c(x_k, u_k, k) + γᴺ h(x_N, N)``, ``c`` and
    ``h`` a :class:`~minilink.core.costs.StepCost`. ``N=None`` is the infinite
    horizon (no ``h``). The start, the sets and the laws are a
    :class:`~minilink.planning.problems.PlanningProblem`'s, shared by
    composition (:attr:`parts`). ``t_start`` is the tick offset, an integer,
    for a native step problem; a discretized problem
    (:meth:`PlanningProblem.discretize`) keeps ``(dt, t_start)`` in seconds, its plant a :class:`~minilink.core.discrete.sampled_system.SampledSystem`
    and its cost a :class:`~minilink.core.costs.SampledCost`. ``compile()``
    gives the step face (:class:`~minilink.planning.step_face.StepFace`), the
    one face value iteration, the RL environment and the judge read.
    """

    def __init__(
        self,
        sys,
        *,
        N=None,
        cost: StepCost | None = None,
        gamma: float = 1.0,
        x_start=None,
        X=None,
        U=None,
        X0=None,
        Xf=None,
        t_start=0,
        dt=None,
        penalize_failure=False,
        failure_cost=None,
        params=None,
        criterion="expectation",
    ):
        require_step_cost(cost)

        # The task: the step system, its horizon, its cost and discount per tick
        self.sys = sys
        self.N = coerce_horizon(N)
        self.cost = cost
        self.gamma = coerce_gamma(gamma)
        self.dt = None if dt is None else float(dt)
        self.t_start = coerce_tick_offset(t_start, self.dt)

        # The continuous problem it samples, for a discretized one (problem.discretize)
        self.origin = None

        # The start, the sets and the laws, shared with the continuous problem
        self.parts = PlanningProblem(
            sys,
            x_start=x_start,
            X=X,
            U=U,
            X0=X0,
            Xf=Xf,
            penalize_failure=penalize_failure,
            failure_cost=failure_cost,
            params=params,
            criterion=criterion,
        )

    # The shared parts

    @property
    def x_start(self):
        return self.parts.x_start

    @property
    def start_law(self):
        return self.parts.start_law

    @property
    def X(self):
        return self.parts.X

    @property
    def U(self):
        return self.parts.U

    @property
    def X0(self):
        return self.parts.X0

    @property
    def Xf(self):
        return self.parts.Xf

    @property
    def laws(self):
        return self.parts.laws

    @property
    def penalize_failure(self):
        return self.parts.penalize_failure

    @property
    def failure_cost(self):
        return self.parts.failure_cost

    @property
    def criterion(self):
        return self.parts.criterion

    @property
    def is_stochastic(self) -> bool:
        return self.parts.is_stochastic

    def sample_x0(self, key, n=None):
        """Draw starts from the start's law (the shared parts')."""
        return self.parts.sample_x0(key, n)

    @property
    def params(self) -> dict:
        """
        The live view of the problem's numbers, by role: ``plant``, ``cost``, the sets', ``x_start``, ``t_start``, ``gamma``.

        A law stands in place of each number the problem draws, as on
        :class:`~minilink.planning.problems.PlanningProblem`.
        """
        view = self.parts_view()
        if self.start_law is not None:
            view["x_start"] = self.start_law
        if self.laws:
            view = merge_tree(view, self.laws)

        return view

    def parts_view(self) -> dict:
        """The parts' numbers by role: :attr:`params` without the laws."""
        parts = self.parts.parts_view()
        view = {"plant": parts["plant"]}
        if self.cost is not None:
            view["cost"] = self.cost.params
        for role in ("X", "U", "X0", "Xf"):
            if role in parts:
                view[role] = parts[role]
        view["x_start"] = self.x_start
        view["t_start"] = self.t_start
        view["gamma"] = self.gamma

        return view

    def compile(self, controller=None):
        """The step face (:class:`~minilink.planning.step_face.StepFace`): ``transition(x, u, k, params) -> (x_next, c_k)``."""
        require_no_controller(controller)

        # Lazy import: the face imports this module
        from minilink.planning.step_face import StepFace

        return StepFace(self)

    def require_cost(self) -> StepCost:
        """Refuse a problem without a cost; return the cost."""
        if self.cost is None:
            raise ValueError("This planner requires problem.cost")

        return self.cost

    def horizon_kind(self) -> str:
        """``"finite"`` with a horizon ``N`` (``h`` is charged there), ``"infinite"`` without."""
        return "infinite" if self.N is None else "finite"

    def cost_bound(self, N=None, *, gamma=None, params=None):
        """
        An upper bound of the cost of any run that stays in the boxes: ``Σ_{k<N} γᵏ c_max + γᴺ h_max``.

        ``c_max`` and ``h_max`` are the stage and terminal costs' largest values
        on a quasi-random sample of the boxes of ``X`` and ``U`` (the declared
        sets', else the plant's ranges); the infinite horizon is
        ``c_max / (1 − γ)``. With ``γ = 1`` on an infinite horizon, or boxes
        with no finite bound, it falls back to ``1e6`` with a warning naming
        ``failure_cost``. A discretized problem's bound is the continuous
        problem's on its own sum at ``dt`` (the face's one number):
        ``g_max dt Σ_{k<N} γᵏ + γᴺ h_max``.
        """
        cost = self.require_cost()
        gamma = self.gamma if gamma is None else float(gamma)
        N = self.N if N is None else N

        # A discretized problem: the continuous problem's sum at dt, γ = e^{−ρ dt}
        if self.origin is not None:
            return sampled_cost_bound(self.origin, N, gamma, self.dt, params)

        # The boxes: the declared sets', else the plant's declared ranges
        X_box = finite_box_or(self.X.bounding_box(), self.sys.state.box)
        U_box = finite_box_or(self.U.bounding_box(), self.sys.input_box.box)
        if X_box is None or U_box is None:
            return fallback_bound("the state or input box has no finite bound")
        if N is None and gamma >= 1.0:
            return fallback_bound("an infinite undiscounted horizon has no cost bound")

        # c_max and h_max: the largest stage and terminal cost on the sampled boxes
        xs, us = box_samples(X_box, U_box)
        c_max = max(float(cost.c(x, u, 0, params)) for x, u in zip(xs, us))
        h_max = max(float(cost.h(x, 0, params)) for x in xs) if N is not None else 0.0

        # Σ_{k<N} γᵏ = (1 − γᴺ) / (1 − γ), or N when γ = 1; 1 / (1 − γ) when N = ∞
        if N is None:
            horizon = 1.0 / (1.0 - gamma)
        elif gamma < 1.0:
            horizon = (1.0 - gamma**N) / (1.0 - gamma)
        else:
            horizon = float(N)

        # J_max = c_max Σ γᵏ + γᴺ h_max
        bound = c_max * horizon + (gamma**N * h_max if N is not None else 0.0)
        if not np.isfinite(bound):
            return fallback_bound("the cost has no finite bound on the boxes")

        return float(bound)

    def failure_cost_at(self, N=None, *, gamma=None, params=None):
        """The cost a failure is charged: :attr:`failure_cost` when a number, else :meth:`cost_bound` (its arguments)."""
        failure_cost = self.failure_cost
        if failure_cost is None:
            return self.cost_bound(N, gamma=gamma, params=params)

        return failure_cost

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


def discretize_problem(problem, dt, *, solver="rk4", substeps=1) -> StepPlanningProblem:
    """
    The continuous problem as a step problem on its plant sampled at ``dt``: the explicit discretization.

    The plant becomes its :class:`~minilink.core.discrete.sampled_system.SampledSystem`,
    the cost a :class:`~minilink.core.costs.SampledCost` (the one-period
    integral of ``e^{−ρτ} g``), ``N = tf / dt``, ``γ = e^{−ρ dt}``,
    ``(dt, t_start)`` in seconds; the start, the sets and the laws as
    they are. Its face equals ``problem.compile(dt=dt, solver=, substeps=)``'s
    to the bit, its cost bound the continuous problem's on the same sum. A problem
    with disturbance sources is refused: the step problem feeds no source.
    """
    require_no_sources(problem)

    # Lazy imports: the sampled view and the evaluator's grid helpers load on use
    from minilink.core.discrete.sampled_system import SampledSystem
    from minilink.planning.evaluator import steps_of

    cost = problem.require_cost()
    dt = float(dt)
    rho = problem.discount_rate
    N = steps_of(problem.tf, dt) if problem.horizon_kind() == "finite" else None

    # γ = e^{−ρ dt}, the discount of one period
    gamma = float(np.exp(-rho * dt))

    step = StepPlanningProblem(
        SampledSystem(problem.sys, dt, solver=solver, substeps=substeps),
        N=N,
        cost=SampledCost(
            cost,
            problem.sys,
            dt,
            solver=solver,
            substeps=substeps,
            discount_rate=rho,
            t_start=problem.t_start,
        ),
        gamma=gamma,
        x_start=problem.start_law if problem.start_law is not None else problem.x_start,
        X=problem.X,
        U=problem.U,
        X0=None if problem.start_law is not None else problem.X0,
        Xf=problem.Xf,
        t_start=problem.t_start,
        dt=dt,
        penalize_failure=problem.penalize_failure,
        failure_cost=problem.failure_cost,
        params=problem.laws or None,
        criterion=problem.criterion,
    )

    # The continuous description it samples: its cost bound is the face's
    step.origin = problem

    return step


###############################################################################
# Internal machinery
###############################################################################


def sampled_cost_bound(problem, N, gamma, dt, params=None):
    """
    A continuous problem's cost bound on a stepped tool's sum: ``g_max dt Σ_{k<N} γᵏ + γᴺ h_max``.

    ``N`` ticks of ``dt`` (``None``: the geometric limit ``g_max dt / (1 − γ)``,
    no ``h``), the discount ``γ = e^{−ρ dt}`` per tick (``γ = 1`` undiscounted,
    ``γ = 0`` one tick counts).
    """
    tf = np.inf if N is None else N * dt

    # ρ = −ln(γ) / dt, so that e^{−ρ dt} = γ
    if gamma >= 1.0:
        rho = 0.0
    elif gamma > 0.0:
        rho = -np.log(gamma) / dt
    else:
        rho = np.inf

    return problem.cost_bound(tf, discount_rate=rho, dt=dt, params=params)


def coerce_horizon(N):
    """The horizon: ``None`` (infinite), or a positive whole number of ticks."""
    if N is None:
        return None
    if isinstance(N, bool) or int(N) != N or N < 1:
        raise ValueError(
            f"N must be a positive whole number of ticks or None, got {N!r}"
        )

    return int(N)


def coerce_gamma(gamma) -> float:
    """The discount per tick as a float in ``(0, 1]``."""
    gamma = float(gamma)
    if not 0.0 < gamma <= 1.0:
        raise ValueError(f"gamma must be in (0, 1], got {gamma!r}")

    return gamma


def coerce_tick_offset(t_start, dt):
    """The start: a whole tick offset for a native problem, seconds for a discretized one."""
    if dt is not None:
        return float(t_start)
    if int(t_start) != t_start:
        raise ValueError(
            f"a native step problem's t_start is a tick offset, a whole number; got {t_start!r}"
        )

    return int(t_start)


def require_step_cost(cost):
    """Refuse a cost that is not a stage cost: a continuous cost goes through ``discretize``."""
    if cost is not None and not isinstance(cost, StepCost):
        raise TypeError(
            f"a step problem's cost is a StepCost (c(x, u, k), h(x, k)), got "
            f"{type(cost).__name__}: a continuous cost goes through "
            "problem.discretize(dt), whose cost is its SampledCost"
        )


def require_no_controller(controller):
    """Refuse a controller on a step problem's compile: the step face scores a policy itself."""
    if controller is not None:
        raise NotImplementedError(
            "StepPlanningProblem.compile() takes no controller: it gives the step face; "
            "score a policy with face.run(policy, x_start)"
        )


def require_no_sources(problem):
    """Refuse to discretize a problem with disturbance sources: a step problem feeds none."""
    if problem.disturbances:
        raise ValueError(
            f"the problem has disturbance sources on {sorted(problem.disturbances)}: a "
            "step problem feeds no source; score it with problem.compile(controller) "
            "or train on it with the RL environment"
        )
