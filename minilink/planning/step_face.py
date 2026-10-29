"""The compiled step face of a problem: ``transition(x, u, k, params) -> (x_next, c_k)``."""

from __future__ import annotations

import numpy as np

from minilink.core.backends import array_module
from minilink.core.costs import SampledCost
from minilink.planning.problems import PlanningProblem, nominal_plant_params
from minilink.planning.step_problems import StepPlanningProblem, sampled_cost_bound

###############################################################################
# Public API
###############################################################################


class StepFace:
    """
    The step face of a problem: one tick of its plant and its stage cost, the one face the step tools read.

    ``transition(x, u, k, params) -> (x_next, c_k)`` advances the plant one
    tick from ``x`` under ``u`` and returns the stage cost of the tick:

    - for a continuous problem compiled at ``dt`` (``problem.compile(dt=)``),
      the plant's step model over one period, the input held, with
      ``c_k = ∫₀^dt e^{−ρτ} g dτ`` integrated beside the state by the same
      scheme and substeps (under one Euler step ``g dt`` to the last bit);
    - for a :class:`~minilink.planning.step_problems.StepPlanningProblem`, its
      system's ``step`` and its cost's ``c`` (a discretized problem's
      :class:`~minilink.core.costs.SampledCost` fused into the same one step,
      so its face equals the continuous problem's).

    ``J = Σ_{k<N} γᵏ c_k + γᴺ h(x_N)``. :attr:`N` is the horizon in ticks
    (``None`` infinite), :attr:`gamma` the discount per tick, :attr:`params`
    the tree every call reads (``params=None``: these numbers),
    :meth:`time_of` the time of tick ``k`` (``t_start + k dt``, or the tick for
    a native problem), :meth:`terminal` the terminal cost, :attr:`X`,
    :attr:`U` and :attr:`Xf` the sets. Value iteration, the RL environment and
    the judge read this face only.
    """

    def __init__(self, problem, *, dt=None, solver="rk4", substeps=1):
        self.problem = problem
        self.sys = problem.sys
        self.n = int(problem.sys.n)
        self.m = int(problem.sys.m)

        # A step problem: its own horizon, discount and stage
        if isinstance(problem, StepPlanningProblem):
            self.dt = problem.dt
            self.N = problem.N
            self.gamma = problem.gamma
            self.solver = None
            self.substeps = None
            cost = problem.cost
            self.stage = cost if isinstance(cost, SampledCost) else None

        # A continuous problem at dt: N = tf / dt, γ = e^{−ρ dt}, the stage its SampledCost
        else:
            require_period(dt)
            # Lazy import: the evaluator's grid helper loads with the first face
            from minilink.planning.evaluator import steps_of

            self.dt = float(dt)
            self.solver = solver
            self.substeps = int(substeps)
            rho = problem.discount_rate
            finite = problem.horizon_kind() == "finite"
            self.N = steps_of(problem.tf, self.dt) if finite else None
            self.gamma = float(np.exp(-rho * self.dt))
            self.stage = None
            if problem.cost is not None:
                self.stage = SampledCost(
                    problem.cost,
                    problem.sys,
                    self.dt,
                    solver=solver,
                    substeps=substeps,
                    discount_rate=rho,
                    t_start=problem.t_start,
                )

    @property
    def params(self) -> dict:
        """The numbers every call reads: ``plant`` (a random plant at its mean), ``cost``, ``x_start``, ``t_start``, and the discount."""
        problem = self.problem
        plant = nominal_plant_params(problem) if is_continuous(problem) else None
        tree = {
            "plant": problem.sys.params if plant is None else plant,
            "cost": None if problem.cost is None else problem.cost.params,
            "x_start": np.asarray(problem.x_start, dtype=float),
            "t_start": problem.t_start,
        }
        if is_continuous(problem):
            tree["discount_rate"] = problem.discount_rate

        return tree

    @property
    def X(self):
        return self.problem.X

    @property
    def U(self):
        return self.problem.U

    @property
    def Xf(self):
        return self.problem.Xf

    @property
    def t_start(self):
        return self.problem.t_start

    def time_of(self, k, params=None):
        """The time of tick ``k``: ``t_start + k dt`` (seconds), or ``t_start + k`` (a native problem's ticks)."""
        t_start = self.t_start if params is None else params["t_start"]
        dt = self.dt

        # A native problem's tick: t_k = t_start + k
        if dt is None:
            t_k = t_start + k
            return t_k

        # t_k = t_start + k dt
        t_k = t_start + k * dt

        return t_k

    def transition(self, x, u, k, params=None):
        """``(x_{k+1}, c_k)``: tick ``k`` of the plant and its stage cost."""
        t_k = self.time_of(k, params)

        return self.transition_at(x, u, t_k, params)

    def transition_at(self, x, u, t, params=None):
        """``(x_next, c)`` of the tick that starts at time ``t`` (a native problem's tick index)."""
        params = self.params if params is None else params
        self.problem.require_cost()

        # A continuous plant: the plant and its stage cost in one step
        if self.stage is not None:
            rho = params.get("discount_rate")
            return self.stage.stage_at(
                x, u, t, params["cost"], params["plant"], discount_rate=rho
            )

        # A native step system: its step and its cost's c
        x_next = self.sys.step(x, u, t, params["plant"])
        c = self.problem.cost.c(x, u, t, params["cost"])

        return x_next, c

    def step_at(self, x, u, t, plant_params=None):
        """``x_next`` alone of the tick that starts at ``t``: the plant's step model (no cost evaluated), on ``plant_params`` (``None``: the face's)."""
        plant = self.params["plant"] if plant_params is None else plant_params

        # A native step system: its own step at the tick
        if self.stage is None and not is_continuous(self.problem):
            return self.sys.step(x, u, t, plant)

        # A continuous plant (a discretized problem's too): the step model over one
        # period from the absolute time t, the input held
        # Lazy import: the step model loads with the first step
        from minilink.core.compile.evaluators.stepping import step_body

        if self.stage is not None:
            f, dt = self.stage.plant.f, self.stage.dt
            solver, substeps = self.stage.solver, self.stage.substeps
        else:
            f, dt, solver, substeps = self.sys.f, self.dt, self.solver, self.substeps

        def dynamics(x, u, tau):
            return f(x, u, tau, plant)

        return step_body(dynamics, x, u, u, t, dt, solver, substeps)

    def horizon_end(self):
        """The time of the horizon's end, ``t_start + tf`` (ticks: ``t_start + N``); ``None`` for an infinite one."""
        if self.N is None:
            return None
        t_start = self.t_start

        # t_end = t_start + tf
        if is_continuous(self.problem):
            tf = self.problem.tf
            t_end = t_start + tf
            return t_end

        t_end = self.time_of(self.N)

        return t_end

    def stage_cost_at(self, x, u, t, params=None):
        """``c`` alone of the tick that starts at ``t``: :meth:`transition_at`'s cost, the plant not stepped where the scheme allows."""
        params = self.params if params is None else params
        self.problem.require_cost()

        if self.stage is not None:
            rho = params.get("discount_rate")
            return self.stage.stage_cost_at(
                x, u, t, params["cost"], params["plant"], discount_rate=rho
            )

        return self.problem.cost.c(x, u, t, params["cost"])

    def terminal(self, x, t=None, params=None):
        """The terminal cost ``h(x)`` at the horizon's end (or at ``t``); ``0`` on an infinite horizon."""
        params = self.params if params is None else params
        xp = array_module(x)
        if self.N is None and t is None:
            return xp.asarray(0.0)

        t = self.time_of(self.N, params) if t is None else t

        # A sampled stage: the continuous cost's h at the time t
        if self.stage is not None:
            h_N = self.stage.terminal_at(x, t, params["cost"])
            return h_N

        # A native step cost: its h at the tick t
        h_N = self.problem.cost.h(x, t, params["cost"])

        return h_N

    def gamma_at(self, params=None):
        """The discount per tick the tree gives: ``e^{−ρ dt}`` at its ``discount_rate`` (a continuous problem), else :attr:`gamma`."""
        params = self.params if params is None else params
        rho = params.get("discount_rate")
        if rho is None:
            return self.gamma
        xp = array_module(rho)
        dt = self.dt

        # γ = e^{−ρ dt}
        gamma = xp.exp(-rho * dt)

        return gamma

    @property
    def cost_bound(self) -> float:
        """
        The face's one cost bound: its own sum with the maxima, ``g_max dt Σ_{k<N} γᵏ + γᴺ h_max``.

        It bounds every stage, ``c_k ≤ g_max dt``; the infinite horizon is its
        geometric limit ``g_max dt / (1 − γ)``, no ``h``; a native step
        problem's is ``Σ γᵏ c_max + γᴺ h_max``. The judge on the face, the RL
        environment and value iteration read it.
        """
        return self.cost_bound_over(self.N, self.gamma)

    @property
    def failure_cost(self) -> float:
        """What a failure is charged on the face: the problem's ``failure_cost`` when a number, else :attr:`cost_bound`."""
        return self.failure_cost_over(self.N, self.gamma)

    def cost_bound_over(self, N, gamma) -> float:
        """
        The face's sum with the maxima over ``N`` ticks (``None``: its geometric limit) at the discount ``gamma`` per tick.

        One formula, each tool over the horizon its own sum runs: the RL
        environment over its episode, value iteration over its sweeps (to
        convergence: the geometric limit), the face's own over :attr:`N`.
        """
        problem = self.problem

        # A continuous problem, or a discretized one: the continuous cost's sum at dt
        if is_continuous(problem):
            return sampled_cost_bound(problem, N, gamma, self.dt)
        if problem.origin is not None:
            return sampled_cost_bound(problem.origin, N, gamma, self.dt)

        # A native step problem: Σ_{k<N} γᵏ c_max + γᴺ h_max (N = None: its own horizon)
        return problem.cost_bound(N, gamma=gamma)

    def failure_cost_over(self, N, gamma) -> float:
        """What a failure is charged over ``N`` ticks at ``gamma``: the problem's ``failure_cost`` when a number, else :meth:`cost_bound_over`."""
        failure_cost = self.problem.failure_cost
        if failure_cost is None:
            return self.cost_bound_over(N, gamma)

        return failure_cost

    def run(self, policy, x_start=None, params=None, n_steps=None):
        """
        The face's rollout under a law ``u = policy(x, t)``: ``(xs, us, cs, J)``, ``J = Σ γᵏ c_k + γᴺ h``.

        ``n_steps`` defaults to :attr:`N`. The states ``(n_steps + 1, n)``, the
        inputs and stage costs ``(n_steps,)``-long, on the array module of the
        start: the one sum every tool's score is.
        """
        params = self.params if params is None else params
        x = np.asarray(params["x_start"] if x_start is None else x_start, dtype=float)
        N = self.N if n_steps is None else int(n_steps)
        require_horizon(N)
        gamma = float(self.gamma_at(params))
        m = self.m

        xs, us, cs = [x], [], []
        J = 0.0
        for k in range(N):
            t_k = self.time_of(k, params)
            u = np.asarray(policy(x, t_k), dtype=float).reshape(m)

            # x_{k+1}, c_k = transition(x_k, u_k, k); J += γᵏ c_k
            x, c = self.transition_at(x, u, t_k, params)
            J = J + gamma**k * float(c)
            xs.append(np.asarray(x, dtype=float))
            us.append(u)
            cs.append(float(c))

        # J += γᴺ h(x_N)
        J = J + gamma**N * float(self.terminal(x, params=params))

        return np.stack(xs), np.stack(us), np.asarray(cs), J


###############################################################################
# Internal machinery
###############################################################################


def is_continuous(problem) -> bool:
    """Whether a problem is a continuous one (``PlanningProblem``), compiled at a period."""
    return isinstance(problem, PlanningProblem)


def require_period(dt):
    """Refuse a continuous problem's face without its period."""
    if dt is None:
        raise ValueError(
            "a continuous problem's step face needs its period: problem.compile(dt=...)"
        )


def require_horizon(N):
    """Refuse a rollout with no number of ticks."""
    if N is None:
        raise ValueError("an infinite horizon needs n_steps= for a rollout")
