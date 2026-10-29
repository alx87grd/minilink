"""Path integral control (MPPI): sample input sequences, roll them out, weigh them by their cost, average."""

from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module, require_jax, require_jax_numpy
from minilink.core.params import merge_known
from minilink.core.roles import command_port
from minilink.core.trajectory import Trajectory
from minilink.planning.environment import RolloutEnvironment
from minilink.planning.planner import Planner
from minilink.planning.results import PlanningSolution

###############################################################################
# Public API
###############################################################################


class MPPIPlanner(Planner):
    """
    Model predictive path integral control (Williams et al. 2017): a sampling planner, no gradients.

    The plan is a held input sequence ``U = (u_0, …, u_{N−1})`` on a grid of
    ``dt`` over the problem's horizon. One solve samples ``K`` perturbed
    sequences ``V^k = U + ε^k``, rolls each out on the plant, scores it with
    the problem's cost (its stage cost ``c_k`` over each period, discounted,
    the terminal cost, the price of leaving ``X``) plus the information term
    ``λ (1 − α) Σ_t u_tᵀ Σ⁻¹ ε^k_t``, weighs the samples by the softmin
    ``w^k ∝ exp(−(S^k − S_min) / λ)`` and moves ``U`` by ``Σ_k w^k ε^k``.
    ``ModelPredictiveController(MPPIPlanner(...)) % dt`` is MPPI in the loop.

    ``dt``, ``n_samples``, ``n_iterations`` and ``solver`` are structure, fixed
    at build; ``temperature`` (``λ``), ``noise`` (``Σ``) and ``control_cost``
    (``α``) are the leaves of ``params["method"]`` and ``seed`` of
    ``params["seed"]``, so a solve can rebind them. JAX only: the solve is one
    jitted function of ``(params, z)``. A deterministic problem is the
    textbook algorithm, the noise on the inputs only.

    Parameters
    ----------
    problem : PlanningProblem
        A finite horizon, a cost, an input box ``U``.
    dt : float
        Control period: ``N = tf / dt`` held inputs.
    n_samples : int
        ``K``, the sampled sequences per pass.
    temperature : float
        ``λ``: small trusts the best samples, large averages them all.
    noise : float or array, optional
        ``Σ``: a float (every input), an ``(m,)`` diagonal or an ``(m, m)``
        covariance; default ``(u_half / 2)²`` from the input box.
    control_cost : float
        ``α``: 0 charges the information term, 1 turns it off.
    n_iterations : int
        Passes of sample, roll out, weigh, update per solve.
    solver : {"rk4", "euler"}
        The plant's integration over each period.
    seed : int
        The draws' seed; a solve at a later tick folds the tick in.
    """

    def __init__(
        self,
        problem,
        *,
        dt=0.05,
        n_samples=1024,
        temperature=1.0,
        noise=None,
        control_cost=0.0,
        n_iterations=1,
        solver="rk4",
        seed=0,
    ):
        require_jax()
        super().__init__(problem)
        tf = problem.require_finite_tf()

        # The structure: the grid, the sample count, the passes, the scheme
        self.dt = float(dt)
        self.n_steps = steps_of(tf, self.dt)
        self.n_samples = int(n_samples)
        self.n_iterations = int(n_iterations)
        self.solver = solver

        # The input box of the action port, and the noise's default from it
        lower, upper = action_box(problem)
        self.u_lower = lower
        self.u_upper = upper
        Sigma = covariance_of(noise, lower, upper)

        # The method's numbers: the defaults of params["method"] and params["seed"]
        self.method = {
            "temperature": float(temperature),
            "noise": Sigma,
            "control_cost": float(control_cost),
        }
        self.seed = int(seed)

    @property
    def params(self) -> dict:
        """The tree the plan depends on: the start, its time, the plant's numbers, the method's and the seed."""
        problem = self.problem

        return {
            "x_start": np.asarray(problem.x_start, dtype=float),
            "t_start": float(problem.t_start),
            "plant": copy.deepcopy(problem.sys.params or {}),
            "method": copy.deepcopy(self.method),
            "seed": self.seed,
        }

    def compile(self, backend=None):
        """The planner's pure face, ``pv``: its solve as one jitted function of ``(params, z)``."""
        return CompiledMPPIPlanner(self)

    def solve(self, params=None, *, guess=None, evaluate=False, n_trials=50):
        """
        One path-integral solve from the problem's start: a :class:`~minilink.planning.results.PlanningSolution`.

        ``params`` overrides leaves of :attr:`params`; ``guess`` (a trajectory or
        a solution) seeds ``U``, else the ports' nominal input. The policy is the
        held schedule ``u = π(t)``; ``evaluate`` scores it with ``evaluate``.
        """
        pv = self.compile()
        z0 = pv.initial(params) if guess is None else pv.pack(guess, params)

        z, status = pv.solve(params, z0)
        solution = pv.solution(z, params, status)

        if evaluate:
            evaluation = self.evaluate(solution.policy, dt=self.dt, n_trials=n_trials)
            solution = PlanningSolution(
                solution.problem,
                solution.policy,
                solution.solver,
                solution.trajectory,
                evaluation,
            )

        return self.store_solution(solution)

    def solve_trajectory(self, **kwargs) -> PlanningSolution:
        """The trajectory family's offline solve: :meth:`solve`."""
        return self.solve(**kwargs)


@dataclass(frozen=True)
class MPPIRecord:
    """
    The account of one path-integral solve: what a user tunes ``λ`` and ``Σ`` against.

    ``effective_samples`` is ``1 / Σ (w^k)²``: a few means ``λ`` is too small
    (one sample decides), close to ``K`` too large (an unweighted average).
    ``success`` is "the nominal rollout of the plan stays in ``X``".
    """

    cost: float
    S_min: float
    S_weighted: float
    effective_samples: float
    exit_rate: float
    iterations: int
    success: bool

    def __str__(self) -> str:
        return (
            f"MPPI: cost {self.cost:.6g} (nominal plan), S_min {self.S_min:.6g}, "
            f"weighted {self.S_weighted:.6g}, effective samples "
            f"{self.effective_samples:.1f}, exit rate {self.exit_rate:.0%}, "
            f"{'stays in X' if self.success else 'leaves X'}"
        )


class CompiledMPPIPlanner:
    """
    The pure face of an :class:`MPPIPlanner`: its solve as a function of ``(params, z)``.

    ``z`` is the held sequence ``U`` row by row (``N m`` numbers) with the origin
    ``t_start`` it was solved from appended. :meth:`solve`, :meth:`shift` and
    :meth:`command` are pure and trace; :meth:`solution` rebuilds the rich
    :class:`~minilink.planning.results.PlanningSolution` on the host. The plan
    carries no states (``predicts`` is false): the MPC's ``x_ref`` comes from
    a planner that does.
    """

    predicts = False

    def __init__(self, planner):
        self.jax = require_jax()
        self.jnp = require_jax_numpy()
        problem = planner.problem

        # The structure: the grid, the sizes, the input box
        self.planner = planner
        self.problem = problem
        self.backend = "jax"
        self.dt = planner.dt
        self.knot_spacing = planner.dt
        self.N = planner.n_steps
        self.n = int(problem.sys.n)
        self.K = planner.n_samples
        self.n_iterations = planner.n_iterations
        self.offsets = planner.dt * np.arange(self.N)
        self.u_lower = planner.u_lower
        self.u_upper = planner.u_upper
        self.m = int(self.u_lower.size)
        self.n_z = self.N * self.m + 1

        # The step and its cost: the problem sampled at dt, the one owner of the exit
        # rule, the price and the terminal cost
        self.env = RolloutEnvironment(
            problem, dt=planner.dt, solver=planner.solver, backend="jax"
        )

        # One compiled function per face: the solve, and the nominal rollout
        self.solve_fn = self.jax.jit(self.solve_body)
        self.nominal_fn = self.jax.jit(self.nominal_rollout)
        self.samples_fns = {}

    @property
    def params(self) -> dict:
        """The compiled defaults: the planner's tree."""
        return self.planner.params

    def initial(self, params=None):
        """The default plan in ``z`` form: the action port's nominal input held over the horizon, at ``params["t_start"]``."""
        P = self.merged(params)
        u_nominal = nominal_action(self.problem, self.u_lower, self.u_upper)

        U = np.tile(u_nominal, (self.N, 1))

        return np.concatenate([U.reshape(-1), [float(P["t_start"])]])

    def solve(self, params=None, z0=None):
        """
        The solve as a function: ``(z, status)`` from the plan ``z0`` (``None``: :meth:`initial`) on ``params``.

        ``status`` is ``{"success", "iterations", "cost"}`` plus the softmin's
        account (``S_min``, ``S_weighted``, ``effective_samples``, ``exit_rate``).
        """
        P = self.merged(params)
        z0 = self.initial(P) if z0 is None else z0

        return self.solve_fn(P, self.jnp.asarray(z0, dtype=float))

    def solve_body(self, P, z0):
        """One path-integral solve from ``P["x_start"]``: :meth:`passes`, then the plan's own rollout and the status."""
        jnp = self.jnp
        path_cost = self.path_cost

        x0 = P["x_start"]
        t0 = P["t_start"]

        # U after n_iterations passes of sample, roll out, weigh, update
        U, last = self.passes(P, z0)

        # The plan's own rollout: its cost and whether it stays in X
        cost, left, _ = path_cost(x0, t0, U, P)
        S = last["S"]
        w = last["w"]
        status = {
            "success": ~left,
            "iterations": self.n_iterations,
            "cost": cost,
            "S_min": jnp.min(S),
            "S_weighted": w @ S,
            "effective_samples": 1.0 / jnp.sum(w**2),
            "exit_rate": jnp.mean(last["left_X"]),
        }
        z = jnp.concatenate([jnp.reshape(U, (-1,)), jnp.reshape(t0, (1,))])

        return z, status

    def passes(self, P, z0):
        """``(U, last)``: ``U`` after ``n_iterations`` passes from the plan ``z0``, and the last pass's samples (``S``, ``w``, ``X``, ``left_X``)."""
        jax = self.jax
        jnp = self.jnp
        N = self.N
        m = self.m
        K = self.K
        dt = self.dt
        t_origin = self.problem.t_start
        u_lower = self.u_lower
        u_upper = self.u_upper
        path_cost = self.path_cost

        # The start, the method's numbers, the plan
        x0 = P["x_start"]
        t0 = P["t_start"]
        lam = P["method"]["temperature"]
        alpha = P["method"]["control_cost"]
        Sigma = P["method"]["noise"]
        Sigma_inv = jnp.linalg.inv(Sigma)
        L = jnp.linalg.cholesky(Sigma)
        U = jnp.reshape(z0[:-1], (N, m))
        key = tick_key(jax, P["seed"], t0, t_origin, dt)

        for i in range(self.n_iterations):
            # Sample: V^k = U + ε^k, ε^k_t ~ N(0, Σ), clipped to the input box
            eps = jax.random.normal(jax.random.fold_in(key, i), (K, N, m)) @ L.T
            V = jnp.clip(U + eps, u_lower, u_upper)

            # Roll out and score: S^k = Σ_t γ^t c_t + γ^N h(x_N) + the price of leaving X
            S_path, left_X, X = jax.vmap(lambda V_k: path_cost(x0, t0, V_k, P))(V)

            # The information term: λ (1 − α) Σ_t u_tᵀ Σ⁻¹ ε^k_t
            S = S_path + lam * (1.0 - alpha) * jnp.einsum(
                "tm,mn,ktn->k", U, Sigma_inv, eps
            )

            # Weigh: w^k = exp(−(S^k − S_min) / λ), normalized
            S_min = jnp.min(S)
            w = jnp.exp(-(S - S_min) / lam)
            w = w / jnp.sum(w)

            # Update: U ← U + Σ_k w^k ε^k, kept in the input box
            U = jnp.clip(U + jnp.einsum("k,ktm->tm", w, eps), u_lower, u_upper)

        last = {"S": S, "w": w, "X": X, "left_X": left_X}

        return U, last

    def samples(self, params=None, z0=None, n_shown=32):
        """
        The fan of a solve: the ``n_shown`` heaviest sampled rollouts of its last pass, their weights, and the updated plan's rollout.

        The same passes as :meth:`solve` on the same key (the seed and the
        tick), so the fan is the solve's own: ``{"t", "futures" (n_shown,
        N + 1, n), "weights" (n_shown,), "plan" (N + 1, n), "inputs" (N, m)}``,
        the heaviest first. The MPC's overlays rebuild each tick's fan with it after a run.
        """
        P = self.merged(params)
        z0 = self.initial(P) if z0 is None else z0
        n_shown = min(int(n_shown), self.K)
        if n_shown not in self.samples_fns:
            self.samples_fns[n_shown] = self.jax.jit(
                lambda P, z0: self.samples_body(P, z0, n_shown)
            )

        fan = self.samples_fns[n_shown](P, self.jnp.asarray(z0, dtype=float))

        return {name: np.asarray(value) for name, value in fan.items()}

    def samples_body(self, P, z0, n_shown):
        """:meth:`samples` as a pure function of ``(P, z0)``."""
        jax = self.jax

        U, last = self.passes(P, z0)

        # The heaviest samples of the last pass, and the plan they moved U to
        weights, index = jax.lax.top_k(last["w"], n_shown)
        futures = last["X"][index]
        plan = self.nominal_rollout(P, U)

        return {
            "t": P["t_start"],
            "futures": futures,
            "weights": weights,
            "plan": plan,
            "inputs": U,
        }

    def shift(self, z, t):
        """The plan moved to the origin ``t``: each held input read at ``t + j dt``, the last one repeated."""
        xp = array_module(z, t)

        U = self.sequence(z)
        origin = z[-1]

        # u_new_j = U[⌊(t + j dt − origin) / dt⌋], clamped to the plan's last input
        j = xp.floor((t - origin) / self.dt + 1e-9) + xp.arange(self.N)
        U_new = U[xp.clip(j, 0, self.N - 1).astype(int)]

        return xp.concatenate(
            [xp.reshape(U_new, (-1,)), xp.reshape(xp.asarray(t, dtype=float), (1,))]
        )

    def command(self, z, x, t, params=None):
        """The input the plan holds at ``t`` (an open-loop plan ignores ``x``)."""
        xp = array_module(z, t)

        U = self.sequence(z)
        origin = z[-1]

        # u(t) = U[⌊(t − origin) / dt⌋], held, clamped to the plan's ends
        j = xp.floor((t - origin) / self.dt + 1e-9)
        u = U[xp.clip(j, 0, self.N - 1).astype(int)]

        return u

    def solution(self, z, params=None, status=None) -> PlanningSolution:
        """The host-side :class:`~minilink.planning.results.PlanningSolution` of ``z``: the held schedule, its nominal rollout, the record."""
        status = {} if status is None else status

        # The plan's nominal rollout on the grid, the last input held at the end
        trajectory = self.planned_trajectory(z, params)

        record = MPPIRecord(
            cost=float(status.get("cost", np.nan)),
            S_min=float(status.get("S_min", np.nan)),
            S_weighted=float(status.get("S_weighted", np.nan)),
            effective_samples=float(status.get("effective_samples", np.nan)),
            exit_rate=float(status.get("exit_rate", np.nan)),
            iterations=int(status.get("iterations", self.n_iterations)),
            success=bool(status.get("success", True)),
        )
        policy = self.planner.open_loop_policy(trajectory, interpolation="previous")

        return PlanningSolution(self.problem, policy, record, trajectory, None)

    def planned_trajectory(self, z, params=None) -> Trajectory:
        """The plan as a trajectory: ``U`` held from its origin, its states rolled out on the nominal plant from ``params["x_start"]``."""
        P = self.merged(params)
        U = np.asarray(self.sequence(np.asarray(z)), dtype=float)
        t0 = float(z[-1])

        xs = np.asarray(self.nominal_fn(P, self.jnp.asarray(U)), dtype=float)
        t = t0 + self.dt * np.arange(self.N + 1)
        u = np.concatenate([U, U[-1:]], axis=0)

        return Trajectory(t=t, x=xs.T, u=u.T)

    def pack(self, guess, params=None):
        """A guess in ``z`` form: a trajectory or a solution's input read at the knots."""
        P = self.merged(params)
        trajectory = getattr(guess, "trajectory", guess)
        t0 = float(P["t_start"])
        knots = t0 + self.offsets

        u = np.asarray(trajectory.u, dtype=float)[: self.m]
        U = np.stack([np.interp(knots, trajectory.t, row) for row in u], axis=1)
        U = np.clip(U, self.u_lower, self.u_upper)

        return np.concatenate([U.reshape(-1), [t0]])

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def merged(self, params):
        """The defaults with ``params`` merged over them, checked against the tree."""
        if not params:
            return self.params

        return merge_known(self.params, params)

    def sequence(self, z):
        """``U``, the plan's ``(N, m)`` held inputs."""
        xp = array_module(z)

        return xp.reshape(z[:-1], (self.N, self.m))

    def path_cost(self, x0, t0, V, P):
        """``(S, left_X, x_path)`` of one held sequence ``V`` from ``x0`` at ``t0``: the discounted cost, whether it left ``X``, the states ``(N + 1, n)``."""
        jax = self.jax
        jnp = self.jnp
        env = self.env
        gamma = env.face.gamma
        plant = {"plant": P["plant"]}

        def period(carry, v):
            x, t, S, alive, discount = carry
            x_next, t_next, reward, terminated, _ = env.step(
                x, t, v, None, plant, t_origin=t0
            )
            # S += γ^t c_t while alive; a failure is charged once, its state then frozen
            S = S - jnp.where(alive, discount * reward, 0.0)
            ended = alive & terminated
            x_next = jnp.where(alive & ~terminated, x_next, x)
            return (x_next, t_next, S, alive & ~ended, discount * gamma), (
                ended,
                x_next,
            )

        x_start = jnp.asarray(x0, dtype=float)
        start = (x_start, t0, 0.0, True, 1.0)
        (x_N, t_N, S, _, _), (ended, xs) = jax.lax.scan(period, start, V)
        x_path = jnp.concatenate([x_start[None, :], xs], axis=0)

        # Left X: an end before the horizon's, or a last state outside X
        inside = jnp.all(env.X.margin(x_N, t_N, env.set_params) >= 0.0)
        left_X = jnp.any(ended[:-1]) | ~inside

        return S, left_X, x_path

    def nominal_rollout(self, P, U):
        """The states ``(N + 1, n)`` of the plan held from ``P["x_start"]``: the plan's evidence."""
        jax = self.jax
        jnp = self.jnp
        env = self.env
        plant = {"plant": P["plant"]}

        def period(carry, u):
            x, t = carry
            x_next, t_next, _, _, _ = env.step(
                x, t, u, None, plant, t_origin=P["t_start"]
            )
            return (x_next, t_next), x_next

        x0 = jnp.asarray(P["x_start"], dtype=float)
        _, xs = jax.lax.scan(period, (x0, P["t_start"]), U)

        return jnp.concatenate([x0[None, :], xs], axis=0)


def steps_of(tf, dt) -> int:
    """``N = tf / dt``, refused unless whole."""
    N = int(round(tf / dt))
    if N < 1 or abs(N * dt - tf) > 1e-9 * max(1.0, tf):
        raise ValueError(
            f"the horizon tf = {tf:g} s is not a whole number of periods dt = {dt:g} s"
        )

    return N


def action_box(problem):
    """``(lower, upper)`` of the action port in the problem's input box, refused unless finite."""
    sys = problem.sys
    box = problem.U.bounding_box()
    if box is None:
        raise ValueError("MPPI samples inside an input box: give the problem a boxed U")
    sl = sys.get_input_port_slice(command_port(sys))
    lower = np.asarray(box.lower, dtype=float)[sl]
    upper = np.asarray(box.upper, dtype=float)[sl]
    if not (np.all(np.isfinite(lower)) and np.all(np.isfinite(upper))):
        raise ValueError(
            "MPPI samples inside an input box: bound every input of the action port"
        )

    return lower, upper


def covariance_of(noise, lower, upper):
    """``Σ`` from ``noise``: a float, an ``(m,)`` diagonal or an ``(m, m)`` matrix; ``None``: ``(u_half / 2)²``."""
    m = lower.size
    if noise is None:
        sigma = 0.25 * (upper - lower)
        return np.diag(sigma**2)

    noise = np.asarray(noise, dtype=float)
    if noise.ndim == 0:
        return np.eye(m) * float(noise)
    if noise.ndim == 1:
        return np.diag(noise)

    return noise


def nominal_action(problem, lower, upper):
    """The action port's nominal input, kept in the box."""
    sys = problem.sys
    port = sys.inputs[command_port(sys)]

    return np.clip(np.asarray(port.nominal_value, dtype=float), lower, upper)


def tick_key(jax, seed, t0, t_origin, dt):
    """The draws' key of a solve at ``t0``: the seed with the tick ``round((t0 − t_origin) / dt)`` folded in."""
    jnp = jax.numpy

    tick = jnp.round((t0 - t_origin) / dt).astype(jnp.int32)
    key = jax.random.PRNGKey(jnp.asarray(seed, dtype=jnp.int32))

    return jax.random.fold_in(key, tick)
