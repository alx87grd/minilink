"""Step planning problems: a task on a step system, a horizon of ticks, a stage cost."""

from __future__ import annotations

import numpy as np

from minilink.core.costs import SampledCost, StepCost, box_samples
from minilink.core.inspect import repr_pretty
from minilink.core.params import require_complete
from minilink.planning.problems import (
    PlanningProblem,
    coerce_laws,
    fallback_bound,
    finite_box_or,
    law_draws,
    planning_horizon_text,
    problem_text,
    require_cost,
    require_numbers,
    require_one_run,
    require_roles,
    stochastic_problem_text,
    stream_of,
    with_law_draws,
)

###############################################################################
# Public API
###############################################################################


class StepPlanningProblem:
    """
    A planning problem on a step system: ``x_{k+1} = step(x_k, u_k, k)``, ``N`` ticks, a discount ``γ`` per tick.

    The objective is ``J = Σ_{k<N} γᵏ c(x_k, u_k, k) + γᴺ h(x_N, N)``, ``c`` and
    ``h`` a :class:`~minilink.core.costs.StepCost`, with ``N`` the ``n_steps``
    ticks. ``n_steps=None`` is the infinite
    horizon (no ``h``). The start and the sets are a
    :class:`~minilink.planning.problems.PlanningProblem`'s, shared by
    composition (:attr:`parts`); the laws (``x0=`` a law, ``distributions=``)
    are over the problem's own tree (:attr:`params`), which :meth:`realize`
    draws. ``t0`` is the tick offset, an integer, for a native step problem:
    tick ``k`` of a run is ``t0 + k``, its stage cost ``c(x_k, u_k, t0 + k)``
    discounted by ``γᵏ``; a discretized problem
    (:meth:`PlanningProblem.discretize`) keeps ``(dt, t0)`` in seconds, its plant a :class:`~minilink.core.discrete.sampled_system.SampledSystem`
    and its cost a :class:`~minilink.core.costs.SampledCost`. ``compile()``
    gives the step face (:class:`~minilink.planning.step_face.StepFace`), the
    one face value iteration, the RL environment and the judge read.
    """

    def __init__(
        self,
        sys,
        *,
        n_steps=None,
        cost: StepCost | None = None,
        gamma: float = 1.0,
        x0=None,
        X=None,
        U=None,
        X0=None,
        Xf=None,
        t0=0,
        dt=None,
        penalize_failure=False,
        failure_cost=None,
        distributions=None,
        criterion="expectation",
    ):
        require_step_cost(cost)

        # The task: the step system, its horizon, its cost and discount per tick
        self.sys = sys
        self.n_steps = coerce_horizon(n_steps)
        self.cost = cost
        self.gamma = coerce_gamma(gamma)
        self.dt = None if dt is None else float(dt)
        self.t0 = coerce_tick_offset(t0, self.dt)

        # The continuous problem it samples, for a discretized one (problem.discretize)
        self.origin = None

        # The start and the sets, shared with the continuous problem
        self.parts = PlanningProblem(
            sys,
            x0=x0,
            X=X,
            U=U,
            X0=X0,
            Xf=Xf,
            penalize_failure=penalize_failure,
            failure_cost=failure_cost,
            criterion=criterion,
        )

        # The laws over the problem's own tree, the cost's numbers among them
        self.distributions = coerce_laws(distributions, self.parts_view())

    # The shared parts

    @property
    def x0(self):
        return self.parts.x0

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
    def penalize_failure(self):
        return self.parts.penalize_failure

    @property
    def failure_cost(self):
        return self.parts.failure_cost

    @property
    def criterion(self):
        return self.parts.criterion

    @property
    def disturbances(self) -> dict:
        """The source blocks on the plant's ports: none, a step problem feeds no source."""
        return {}

    @property
    def is_stochastic(self) -> bool:
        """``True`` when a part is random: a law on the start or on a number of the tree."""
        return self.start_law is not None or bool(self.distributions)

    def sample_x0(self, key, n=None):
        """Draw starts from the start's law (the shared parts')."""
        return self.parts.sample_x0(key, n)

    @property
    def params(self) -> dict:
        """
        The live view of the problem's numbers, by role: ``plant``, ``cost``, the sets', ``x0``, ``t0``, ``gamma``.

        Numbers only, as on :class:`~minilink.planning.problems.PlanningProblem`:
        the laws over them are :attr:`distributions`, and :meth:`realize`
        draws them into this tree. Assigning a complete tree routes each entry
        to its part, as on a continuous problem.
        """
        return self.parts_view()

    @params.setter
    def params(self, value):
        view = self.parts_view()
        require_roles(value, view)
        require_numbers(value or {})
        require_complete(value, view)

        for role, entry in value.items():
            if role == "plant":
                self.sys.params = entry
            elif role == "cost":
                self.cost.params = entry
            elif role == "x0":
                self.parts.x0 = entry
            elif role == "t0":
                self.t0 = coerce_tick_offset(entry, self.dt)
            elif role == "gamma":
                self.gamma = coerce_gamma(entry)
            else:
                # A set's numbers in place, its one home
                getattr(self, role).params = entry

    def parts_view(self) -> dict:
        """The parts' numbers by role: :attr:`params` without the laws."""
        parts = self.parts.parts_view()
        view = {"plant": parts["plant"]}
        if self.cost is not None:
            view["cost"] = self.cost.params
        for role in ("X", "U", "X0", "Xf"):
            if role in parts:
                view[role] = parts[role]
        view["x0"] = self.x0
        view["t0"] = self.t0
        view["gamma"] = self.gamma

        return view

    def realize(self, key=None, n=1):
        """
        ``n`` realizations of the problem's draws from one key: its own tree.

        The problem's numbers (:attr:`params`, its tick offset ``t0`` and its
        ``gamma`` among them), with ``"x0"`` of shape ``(n, nx)`` and each
        law's leaf drawn ``n`` times in its number's shape, each from the
        stream named by its path, as
        :meth:`PlanningProblem.realize <minilink.planning.problems.PlanningProblem.realize>`
        draws them; the other leaves are shared. A deterministic problem has
        one realization, its own start; ``key=None`` draws every law at its mean.
        """
        n = int(n)
        tree = self.parts_view()

        # A deterministic problem: its one start, the shared numbers
        if not self.is_stochastic:
            require_one_run(n)
            tree["x0"] = self.sample_x0(None, n)
            return tree

        # x(0) ~ the start's law, n times, from the stream named "x0"
        tree["x0"] = self.sample_x0(stream_of(key, "x0"), n)

        # Each law's leaf n times, in its number's shape, from the stream named by its path
        tree = with_law_draws(tree, self.distributions, key, n)

        return tree

    def sample_params(self, key):
        """One draw of each law, at its path in the problem's tree (empty when nothing is drawn)."""
        return law_draws(self.distributions, self.parts_view(), key)

    def compile(self, controller=None):
        """The step face (:class:`~minilink.planning.step_face.StepFace`): ``transition(x, u, k, params) -> (x_next, c_k)``."""
        require_no_controller(controller)

        # Lazy import: the face imports this module
        from minilink.planning.step_face import StepFace

        return StepFace(self)

    def horizon_kind(self) -> str:
        """``"finite"`` with a horizon ``N`` (``h`` is charged there), ``"infinite"`` without."""
        return "infinite" if self.n_steps is None else "finite"

    def cost_bound(self, n_steps=None, *, gamma=None, params=None):
        """
        An upper bound of the cost of any run that stays in the boxes: ``Σ_{k<N} γᵏ c_max + γᴺ h_max``.

        ``c_max`` and ``h_max`` are the stage and terminal costs' largest values
        on a quasi-random sample of the boxes of ``X`` and ``U`` (the declared
        sets', else the plant's ranges), read at the tick offset ``t0``; the
        infinite horizon is ``c_max / (1 − γ)``. With ``γ = 1`` on an infinite horizon, or boxes
        with no finite bound, it falls back to ``1e6`` with a warning naming
        ``failure_cost``. A discretized problem's bound is the continuous
        problem's on its own sum at ``dt`` (the face's one number):
        ``g_max dt Σ_{k<N} γᵏ + γᴺ h_max``.
        """
        cost = require_cost(self)
        gamma = self.gamma if gamma is None else float(gamma)
        N = self.n_steps if n_steps is None else n_steps

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

        # c_max and h_max: the largest stage and terminal cost on the sampled boxes,
        # read at the tick offset k₀ (a problem started at k₀ bounds as from 0)
        k0 = self.t0
        xs, us = box_samples(X_box, U_box)
        c_max = max(float(cost.c(x, u, k0, params)) for x, u in zip(xs, us))
        h_max = max(float(cost.h(x, k0, params)) for x in xs) if N is not None else 0.0

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

    def failure_cost_at(self, n_steps=None, *, gamma=None, params=None):
        """The cost a failure is charged: :attr:`failure_cost` when a number, else :meth:`cost_bound` (its arguments)."""
        failure_cost = self.failure_cost
        if failure_cost is None:
            return self.cost_bound(n_steps, gamma=gamma, params=params)

        return failure_cost

    # Display

    def __str__(self):
        horizon = f"n_steps={planning_horizon_text(self.n_steps)}, gamma={self.gamma:g}"
        if self.is_stochastic:
            return stochastic_problem_text(self, horizon)

        return problem_text(self, horizon)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


def discretize_problem(problem, dt, *, solver="rk4", substeps=1) -> StepPlanningProblem:
    """
    The continuous problem as a step problem on its plant sampled at ``dt``: the explicit discretization.

    The plant becomes its :class:`~minilink.core.discrete.sampled_system.SampledSystem`,
    the cost a :class:`~minilink.core.costs.SampledCost` (the one-period
    integral of ``e^{−ρτ} g``), ``N = tf / dt``, ``γ = e^{−ρ dt}``,
    ``(dt, t0)`` in seconds; the start, the sets and the laws as
    they are. Its face equals ``StepFace(problem, dt=dt, solver=, substeps=)``'s
    to the bit, its cost bound the continuous problem's on the same sum. A problem
    with disturbance sources is refused: the step problem feeds no source.
    """
    require_no_sources(problem)

    # Lazy imports: the sampled view and the evaluator's grid helpers load on use
    from minilink.core.discrete.sampled_system import SampledSystem
    from minilink.planning.evaluator import steps_of

    cost = require_cost(problem)
    dt = float(dt)
    rho = problem.discount_rate
    N = steps_of(problem.horizon, dt) if problem.horizon_kind() == "finite" else None

    # γ = e^{−ρ dt}, the discount of one period
    gamma = float(np.exp(-rho * dt))

    step = StepPlanningProblem(
        SampledSystem(problem.sys, dt, solver=solver, substeps=substeps),
        n_steps=N,
        cost=SampledCost(
            cost,
            problem.sys,
            dt,
            solver=solver,
            substeps=substeps,
            discount_rate=rho,
            t0=problem.t0,
        ),
        gamma=gamma,
        x0=problem.start_law if problem.start_law is not None else problem.x0,
        X=problem.X,
        U=problem.U,
        X0=None if problem.start_law is not None else problem.X0,
        Xf=problem.Xf,
        t0=problem.t0,
        dt=dt,
        penalize_failure=problem.penalize_failure,
        failure_cost=problem.failure_cost,
        distributions=problem.distributions or None,
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
            f"n_steps must be a positive whole number of ticks or None, got {N!r}"
        )

    return int(N)


def coerce_gamma(gamma) -> float:
    """The discount per tick as a float in ``(0, 1]``."""
    gamma = float(gamma)
    if not 0.0 < gamma <= 1.0:
        raise ValueError(f"gamma must be in (0, 1], got {gamma!r}")

    return gamma


def coerce_tick_offset(t0, dt):
    """The start: a whole tick offset for a native problem, seconds for a discretized one."""
    if dt is not None:
        return float(t0)
    if int(t0) != t0:
        raise ValueError(
            f"a native step problem's t0 is a tick offset, a whole number; got {t0!r}"
        )

    return int(t0)


def require_step_cost(cost):
    """Refuse a cost that is not a stage cost: a continuous cost goes through ``discretize``."""
    if cost is not None and not isinstance(cost, StepCost):
        raise TypeError(
            f"a step problem's cost is a StepCost (c(x, u, k), h(x, k)), got "
            f"{type(cost).__name__}: a continuous cost goes through "
            "problem.discretize(dt), whose cost is its SampledCost"
        )


def require_no_period_on_ticks(problem, dt):
    """Refuse a ``dt`` for a step problem: it ticks on its own clock."""
    if isinstance(problem, StepPlanningProblem) and dt is not None:
        raise ValueError(
            "a step problem ticks on its own clock: omit dt (its period, if it has "
            "one, is the problem's dt)"
        )


def require_no_controller(controller):
    """Refuse a controller on a step problem's compile: the step face scores a policy itself."""
    if controller is not None:
        raise NotImplementedError(
            "StepPlanningProblem.compile() takes no controller: it gives the step face; "
            "score a policy with face.run(policy, x0)"
        )


def require_no_sources(problem):
    """Refuse to discretize a problem with disturbance sources: a step problem feeds none."""
    if problem.disturbances:
        raise ValueError(
            f"the problem has disturbance sources on {sorted(problem.disturbances)}: a "
            "step problem feeds no source; score it with problem.compile(controller) "
            "or train on it with the RL environment"
        )
