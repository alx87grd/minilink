"""Monte Carlo evaluation: a policy's cost on a planning problem, over one trajectory or the problem's draws."""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import jax_installed, require_jax, require_jax_numpy
from minilink.core.options import require_one_of
from minilink.core.trajectory import Trajectory
from minilink.planning.problems import as_stochastic

###############################################################################
# Public API
###############################################################################


class MonteCarloEvaluator:
    """
    Score a policy over the draws of a planning problem.

    Parameters
    ----------
    problem : StochasticPlanningProblem
        Task (plant, cost, constraint set and its price, horizon, distributions). A
        deterministic :class:`~minilink.planning.problems.PlanningProblem` is
        scored from its single start.
    dt : float
        Control period (the law is held over it) and sampling step.
    n_trials : int
        Number of draws.
    episode_length : float, optional
        Duration of each trial for an infinite-horizon problem (default 10 s).
    backend : {"auto", "jax", "numpy", "simulator"}
        ``"jax"`` vmaps the held-input rollout over the trials (static laws),
        ``"numpy"`` runs the same trials one at a time, ``"simulator"``
        integrates the continuous-time loop (any controller, no parameter or
        disturbance draws). ``"auto"`` (default) is JAX when it is installed
        and the plant, the law, ``X`` and the cost trace on it, NumPy otherwise
        (a lookup table's law, say); that fallback warns when it changes the draws.
    seed : int
        Seed of the draws (the same seed gives the same draws on the JAX and
        NumPy backends only through a :class:`~minilink.core.distributions.Particles`
        start distribution; the two random streams differ otherwise).
    record : bool
        Keep the trial trajectories (NumPy and simulator backends).
    """

    BACKENDS = ("auto", "jax", "numpy", "simulator")

    def __init__(
        self,
        problem,
        *,
        dt=0.05,
        n_trials=100,
        episode_length=None,
        backend="auto",
        seed=0,
        record=False,
    ):
        # The task, its draws included (one start for a deterministic problem)
        self.problem = as_stochastic(problem)

        # The trials: control period, number, duration, backend, seed, recording
        self.dt = float(dt)
        self.n_trials = int(n_trials)
        self.episode_length = episode_length
        self.backend = backend
        self.seed = int(seed)
        self.record = bool(record)

        require_one_of("backend", backend, self.BACKENDS)

    @property
    def tf(self) -> float:
        """Duration of a trial: the finite horizon, else ``episode_length`` (10 s by default)."""
        problem = self.problem
        if problem.horizon_kind() == "finite":
            return float(problem.tf)

        return 10.0 if self.episode_length is None else float(self.episode_length)

    def evaluate(self, controller) -> Evaluation:
        """Return the :class:`Evaluation` of a policy block (or of a solution's policy) on the problem."""
        # The law: the block itself, or a solution's policy
        controller = policy_of(controller)

        # The backend: "auto" is JAX when the whole loop traces on it
        backend = self.backend
        if backend == "auto":
            backend = self.auto_backend(controller)

        # The trials on that backend
        if backend == "jax":
            return self.evaluate_jax(controller)
        if backend == "numpy":
            return self.evaluate_numpy(controller)

        return self.evaluate_simulator(controller)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def auto_backend(self, *controllers) -> str:
        """``"jax"`` when JAX is installed and the plant, every law, ``X`` and the cost trace on it, else ``"numpy"``."""
        # Lazy import: the compiler loads on first use, not with the evaluator
        from minilink.core.compile.compiler import compile_auto

        problem = self.problem
        seed = self.seed
        n_trials = self.n_trials

        if not jax_installed():
            return "numpy"

        # The blocks of the loop and the problem's terms that do not trace on JAX, by
        # name (an open-loop source is a time table, never traced)
        feedback = [controller for controller in controllers if int(controller.m) > 0]
        loop = [problem.sys, *feedback]
        numpy_only = [block.name for block in loop if compile_auto(block)[0] != "jax"]
        numpy_only += numpy_only_terms(problem)
        if not numpy_only:
            return "jax"

        # One seed, two random streams: random trials differ from a JAX-scored law's
        culprits = ", ".join(dict.fromkeys(numpy_only))  # Two lookup tables named once
        x0s = problem.sample_x0(np.random.default_rng(seed), n=n_trials)
        random_starts = bool(np.any(x0s != problem.x_start))
        if random_starts or problem.params_distribution or problem.disturbances:
            warnings.warn(
                f"{culprits} does not trace on JAX: scored on NumPy, whose "
                "draws of the starts, parameters and disturbances differ from the JAX "
                "backend's for the same seed; pass backend='numpy' to score every "
                "policy on the same draws",
                stacklevel=3,
            )

        return "numpy"

    def evaluate_jax(self, controller) -> Evaluation:
        """The trials vmapped on JAX, each a scan of held-input steps on its own key."""
        jax = require_jax()
        jnp = require_jax_numpy()
        # A real cycle: reinforcement_learning imports Evaluation from this module
        from minilink.learning.reinforcement_learning.environment import (
            RolloutEnvironment,
        )

        seed = self.seed
        n_trials = self.n_trials

        # The trial's environment: plant step, draws, cost, price of an exit
        env = RolloutEnvironment(self.problem, dt=self.dt, episode_length=self.tf)
        dt = env.dt
        rho = env.discount_rate
        g = env.cost.g
        h = env.cost.h
        X = env.X
        set_params = env.set_params
        n_steps = env.n_steps_per_episode
        finite_horizon = env.finite_horizon

        # The law on the trial's control grid
        law = control_law(controller, dt * np.arange(n_steps + 1), "jax")

        def trial(key):
            # One key each for the start, the parameters and the steps
            k0, k_params, k_steps = jax.random.split(key, 3)
            x0 = env.reset(k0)
            params = env.sample_params(k_params)

            def body(carry, key):
                x, t, J, alive, failed = carry

                # u_k = π(x_k, t_k) held over the period, one plant step to x_{k+1}
                u = law(x, t)
                u_full = env.input_vector(u, key)
                x_next = env.plant_step(x, u_full, t, params)
                t_next = t + dt

                # J += ½ (e^{−ρ t_k} g_k + e^{−ρ t_{k+1}} g_{k+1}) dt, while alive
                g_k = jnp.exp(-rho * t) * g(x, u, t)
                g_next = jnp.exp(-rho * t_next) * g(x_next, law(x_next, t_next), t_next)
                J = J + alive * 0.5 * (g_k + g_next) * dt

                # A failure (x_{k+1} left X) is the last sample counted, and is charged
                out_of_X = ~jnp.all(X.margin(x_next, t_next, set_params) >= 0.0)
                diverged = ~jnp.all(jnp.isfinite(x_next))
                out = out_of_X | diverged
                exits = alive & out
                J = J + jnp.where(exits, env.infeasible_penalty(x_next, t_next), 0.0)
                failed = failed | exits
                alive = alive & ~out

                return (x_next, t_next, J, alive, failed), None

            # The steps, one key each, from x0 at t = 0 with J = 0
            init = (x0, 0.0, 0.0, jnp.bool_(True), jnp.bool_(False))
            step_keys = jax.random.split(k_steps, n_steps)
            (x, t, J, alive, failed), _ = jax.lax.scan(body, init, step_keys)

            # A trial alive at a finite horizon adds h(x(t_f), t_f)
            if finite_horizon:
                J = J + alive * h(x, t)

            return J, failed, x0

        # Every trial at once, one key each
        keys = jax.random.split(jax.random.PRNGKey(seed), n_trials)
        J, failed, x0 = jax.jit(jax.vmap(trial))(keys)

        # The trials back on the host
        J = np.asarray(J)
        failed = np.asarray(failed)
        x0 = np.asarray(x0)

        return Evaluation(J, failed, x0)

    def evaluate_numpy(self, controller) -> Evaluation:
        """The trials one at a time on NumPy, the law held over each period."""
        # A real cycle: reinforcement_learning imports Evaluation from this module
        from minilink.learning.reinforcement_learning.environment import (
            RolloutEnvironment,
        )

        problem = self.problem
        seed = self.seed
        n_trials = self.n_trials
        record = self.record

        # The trial's environment on NumPy, and the law on its control grid
        env = RolloutEnvironment(
            problem, dt=self.dt, episode_length=self.tf, backend="numpy"
        )
        rng = np.random.default_rng(seed)
        n_steps = env.n_steps_per_episode
        dt = env.dt
        law = control_law(controller, dt * np.arange(n_steps + 1), "numpy")

        # The starts, drawn first from the one random stream
        J = np.zeros(n_trials)
        failed = np.zeros(n_trials, dtype=bool)
        x0s = np.asarray(problem.sample_x0(rng, n=n_trials), dtype=float)
        trajectories = [] if record else None

        for i, x0 in enumerate(x0s):
            # The trial's parameters, its time grid, its states and inputs from x0
            params = env.sample_params(rng)
            t = dt * np.arange(n_steps + 1)
            xs = np.zeros((env.n, n_steps + 1))
            us = np.zeros((env.m, n_steps + 1))
            xs[:, 0] = x0

            # u_k = π(x_k, t_k) held over the period, one plant step to x_{k+1}; the
            # last sample takes its input only
            for k in range(n_steps + 1):
                us[:, k] = law(xs[:, k], t[k])
                if k == n_steps:
                    break
                u_full = env.input_vector(us[:, k], rng)
                xs[:, k + 1] = env.plant_step(xs[:, k], u_full, t[k], params)

            # The trial scored under the problem's contract, at the environment's price
            traj = Trajectory(t=t, x=xs, u=us)
            J[i], failed[i] = score_trajectory(
                problem, traj, infeasible_cost=env.infeasible_cost
            )

            if trajectories is not None:
                trajectories.append(traj)

        return Evaluation(J, failed, x0s, trajectories)

    def evaluate_simulator(self, controller) -> Evaluation:
        """The trials integrated by the simulator: the continuous-time loop from each drawn start."""
        # A real cycle: reinforcement_learning imports Evaluation from this module
        from minilink.learning.reinforcement_learning.environment import (
            RolloutEnvironment,
        )

        problem = self.problem
        sys = problem.sys
        seed = self.seed
        n_trials = self.n_trials
        record = self.record

        # The simulator draws the starts only
        if problem.params_distribution or problem.disturbances:
            warnings.warn(
                "the simulator backend ignores parameter and disturbance draws; "
                "use backend='jax' or 'numpy' for a randomized plant",
                stacklevel=2,
            )

        # The starts
        rng = np.random.default_rng(seed)
        J = np.zeros(n_trials)
        failed = np.zeros(n_trials, dtype=bool)
        x0s = np.asarray(problem.sample_x0(rng, n=n_trials), dtype=float)
        trajectories = [] if record else None

        # The plant's own start, restored after the trials, and the port the cost reads
        x0_saved = np.asarray(sys.x0, dtype=float).copy()
        action_port = env_action_port(sys)

        # The price of an exit: the NumPy backend's, from the same trial environment
        env = RolloutEnvironment(
            problem, dt=self.dt, episode_length=self.tf, backend="numpy"
        )
        price = env.infeasible_cost

        try:
            for i, x0 in enumerate(x0s):
                sys.x0 = np.asarray(x0, dtype=float)

                # An open-loop source drives the plant in series; a feedback block closes the loop
                cl_sys = (
                    controller >> sys if int(controller.m) == 0 else controller @ sys
                )
                cl_traj = cl_sys.compute_trajectory(
                    tf=self.tf, dt=self.dt, verbose=False
                )

                # The plant as it ran in the loop; the cost reads its action port
                plant_traj = cl_sys.trajectory_of(sys, cl_traj)
                u = sys.get_port_values_from_u(plant_traj.u, action_port)
                traj = Trajectory(t=plant_traj.t, x=plant_traj.x, u=u)

                # The trial scored under the problem's contract, at the environment's price
                J[i], failed[i] = score_trajectory(problem, traj, infeasible_cost=price)

                if trajectories is not None:
                    trajectories.append(traj)
        finally:
            sys.x0 = x0_saved

        return Evaluation(J, failed, x0s, trajectories)


@dataclass
class Evaluation:
    """
    The cost of a policy on the problem's draws: per-trial ``J`` and its summary.

    ``failed`` marks the trials that left the allowed box; ``x0`` holds the
    starts; ``trajectories`` the recorded trials when asked for. One trial is
    a deterministic problem's whole evaluation.
    """

    J: np.ndarray
    failed: np.ndarray
    x0: np.ndarray
    trajectories: list | None = None

    @classmethod
    def of_trajectory(cls, problem, traj: Trajectory) -> Evaluation:
        """One trial: a sampled trajectory scored under the problem's contract."""
        J, failed = score_trajectory(problem, traj)

        # The one trial's cost, failure flag, start and trajectory
        J_trials = np.array([J])
        failed_trials = np.array([failed])
        x0 = traj.x[:, :1].T.copy()
        trajectories = [traj]

        return cls(J_trials, failed_trials, x0, trajectories)

    @property
    def n_trials(self) -> int:
        """Number of trials."""
        return int(np.size(self.J))

    @property
    def mean(self) -> float:
        """Mean cost over the trials."""
        return float(np.mean(self.J))

    @property
    def std(self) -> float:
        """Standard deviation of the cost over the trials."""
        return float(np.std(self.J))

    @property
    def worst(self) -> float:
        """Largest cost over the trials."""
        return float(np.max(self.J))

    @property
    def failure_rate(self) -> float:
        """Fraction of the trials that left the allowed box."""
        return float(np.mean(self.failed))

    def value(self, criterion="expectation") -> float:
        """The problem's criterion: ``"expectation"`` (mean) or ``"worst_case"`` (max)."""
        require_criterion(criterion, ("expectation", "worst_case"))

        if criterion == "expectation":
            return self.mean

        return self.worst

    # Display

    def __str__(self) -> str:
        if self.n_trials == 1:
            note = ", left the box" if bool(self.failed[0]) else ""
            return f"J = {self.mean:.2f}{note}"

        return (
            f"J over {self.n_trials} trials: mean {self.mean:.2f}, std {self.std:.2f}, "
            f"worst {self.worst:.2f}, failure rate {100 * self.failure_rate:.0f}%"
        )


def score_trajectory(problem, traj: Trajectory, params=None, infeasible_cost=np.inf):
    """
    Return ``(J, failed)`` of a sampled closed-loop trajectory under the problem's contract.

    The one scoring contract every tool reports with: the discounted trapezoid
    of ``g``, cut at the first exit from ``X``, plus ``h`` at a reached finite
    horizon.

    ``traj`` carries the states and the applied inputs on a time grid;
    ``params`` are the cost parameters. ``failed`` is ``True`` when the
    trajectory left ``X`` (read at each sample time on the problem's set
    parameters) before the end of the grid; the trial is then charged
    the problem's ``infeasible_cost``, else ``infeasible_cost`` given here (an
    evaluator passes the bound its environment derived; ``+inf`` otherwise).
    """
    cost = problem.require_cost()
    X = problem.X
    set_params = problem.params.sets

    t = traj.t
    x = traj.x
    u = traj.u
    N = traj.n_samples

    # x(t_k) ∈ X at every sample time, on the problem's set parameters
    inside = np.array([X.contains(x[:, k], float(t[k]), set_params) for k in range(N)])

    # A failure ends the trial at its first exit: the samples after it are dropped
    failed = not bool(inside.all())
    last = int(np.argmin(inside)) if failed else N - 1
    kept = Trajectory(t=t[: last + 1], x=x[:, : last + 1], u=u[:, : last + 1])

    # J = ∫ e^{−ρt} g(x, u, t) dt by the trapezoid over the kept samples
    rho = problem.discount_rate
    _, running = cost.integrate(kept, discount_rate=rho, params=params)
    J = float(running[-1])

    # An exit is charged its price; a reached finite horizon adds h(x(t_f), t_f)
    if failed:
        price = problem.infeasible_cost
        J += float(infeasible_cost if price is None or price is True else price)
    elif problem.horizon_kind() == "finite" and t[-1] >= problem.tf - 1e-6:
        J += float(cost.h(x[:, -1], float(t[-1]), params=params))

    return J, failed


def nominal_trajectory(problem, policy, *, dt, tf=None) -> Trajectory:
    """
    The policy from the problem's start on a control grid, nominal parameters and disturbances.

    One recorded trial of the NumPy evaluator on the certainty-equivalent
    problem: the trajectory a feedback planner reports beside its policy.
    """
    # The certainty-equivalent problem: the mean start, no draws
    nominal = problem.nominal() if problem.is_stochastic else problem

    # One recorded trial on NumPy
    evaluator = MonteCarloEvaluator(
        nominal, dt=dt, n_trials=1, episode_length=tf, backend="numpy", record=True
    )
    evaluation = evaluator.evaluate(policy)

    return evaluation.trajectories[0]


###############################################################################
# Internal machinery
###############################################################################


def policy_of(controller):
    """The block itself, or the policy of a :class:`~minilink.planning.results.PlanningSolution`."""
    # A real cycle: results imports Evaluation from this module
    from minilink.planning.results import PlanningSolution

    if isinstance(controller, PlanningSolution):
        return controller.policy

    return controller


def numpy_only_terms(problem) -> list:
    """
    The problem's terms a JAX trial calls that do not trace on JAX, by name.

    ``X.margin`` on the set parameters, the running cost ``g``, the terminal
    cost ``h`` on a finite horizon and a callable ``infeasible_cost``, each
    traced once on a state, an action and a time.
    """
    jax = require_jax()
    jnp = require_jax_numpy()

    sys = problem.sys
    X = problem.X
    cost = problem.require_cost()
    set_params = problem.params.sets
    price = problem.infeasible_cost

    # The terms a trial calls, each as a function of (x, u, t)
    terms = {
        "the constraint set X": lambda x, u, t: X.margin(x, t, set_params),
        "the cost": lambda x, u, t: cost.g(x, u, t),
    }
    if problem.horizon_kind() == "finite":
        terms["the terminal cost"] = lambda x, u, t: cost.h(x, t)
    if callable(price):
        terms["the infeasible_cost"] = lambda x, u, t: price(x, t)

    # A state and an action to trace them on
    x = jnp.zeros(int(sys.n))
    u = jnp.zeros(int(sys.inputs[env_action_port(sys)].dim))

    # The terms whose trace fails
    numpy_only = []
    for name, term in terms.items():
        try:
            jax.make_jaxpr(term)(x, u, 0.0)
        except Exception:
            numpy_only.append(name)

    return numpy_only


def env_action_port(sys) -> str:
    """The plant's command port: the action a trial applies and the cost reads."""
    # Lazy import: the loop roles load on first use, not with the evaluator
    from minilink.core.feedback import command_port

    return command_port(sys)


def control_law(controller, t, backend="jax"):
    """``u = pi(x, t)`` of a policy block: a source by its time table, a feedback block by its state law."""
    if int(controller.m) == 0:
        return time_law(controller, t, backend)

    return static_law(controller, backend)


def time_law(source, t, backend="jax"):
    """``u = pi(t)`` of a source block, sampled on the trial grid ``t`` and held over each period."""
    empty = np.zeros(0)

    # u_k = π(t_k): the source's output at each sample of the grid
    table = np.stack(
        [np.asarray(source.h(empty, empty, float(t_k)), dtype=float) for t_k in t]
    )
    dt = float(t[1] - t[0])

    # u(t) = u_k at the nearest sample, k = round(t / dt)
    if backend == "jax":
        jnp = require_jax_numpy()
        table = jnp.asarray(table)
        return lambda x, t: table[jnp.rint(t / dt).astype(int)]

    return lambda x, t: table[int(round(t / dt))]


def static_law(controller, backend="jax"):
    """
    ``u = pi(x, t)`` of any static feedback block, on the NumPy or JAX backend.

    Compiles the block and evaluates its control port with the measurement
    port fed the plant state and the other inputs at their nominal values
    (a reference port left unconnected holds its set point, as in a diagram).
    The time reaches the block, so a time-varying law (a finite-horizon LQR
    gain ``K(t)``) is scored with the gain of each instant.
    """
    # Lazy import: the loop roles load on first use, not with the evaluator
    from minilink.core.feedback import block_roles

    roles = block_roles(controller)
    require_feedback_roles(controller, roles)
    require_static_law(controller)

    # The block compiled on the backend
    evaluator = build_law_evaluator(controller, backend)

    # The measurement's slice of the block's stacked input vector
    start = 0
    for port_id, port in controller.inputs.items():
        if port_id == roles.measurement:
            measurement = slice(start, start + port.dim)
        start += port.dim

    # u = π(x, t): the state on the measurement port, every other input at its nominal
    if backend == "jax":
        jnp = require_jax_numpy()

        u_nominal = jnp.asarray(controller.get_u_from_input_ports(), dtype=float)
        x_ctl = jnp.zeros(0)

        def law(x, t=0.0):
            u_in = u_nominal.at[measurement].set(x)
            u = evaluator.outputs_trace(x_ctl, u_in, t)[roles.command]
            return u

    else:
        u_nominal = np.asarray(controller.get_u_from_input_ports(), dtype=float)
        x_ctl = np.zeros(0)

        def law(x, t=0.0):
            u_in = u_nominal.copy()
            u_in[measurement] = x
            u = np.asarray(evaluator.outputs(x_ctl, u_in, float(t))[roles.command])
            return u

    return law


def require_feedback_roles(controller, roles):
    """Refuse a block that reads no state or commands no ``u``."""
    if roles.measurement is None or roles.command is None:
        raise ValueError(
            "the controller must read the state on an input named 'x' (or 'y') "
            f"and command 'u'; {controller.name!r} reads {list(controller.inputs)} "
            f"and gives {list(controller.outputs)}"
        )


def require_static_law(controller):
    """Refuse a controller with internal state: a trial holds none."""
    if int(controller.n) > 0:
        raise ValueError(
            "static laws u = pi(x) only; use backend='simulator' for a controller "
            "with internal state"
        )


def build_law_evaluator(controller, backend):
    """The block compiled on ``backend``, a law that does not trace on JAX refused in plain words."""
    try:
        evaluator = controller.compile(backend=backend, verbose=False)
    except RuntimeError as exc:
        if "JAX-traceable" not in str(exc):
            raise
        raise RuntimeError(
            f"the law of {controller.name!r} is NumPy-only (not JAX-traceable): "
            "pass backend='numpy' to score it"
        ) from exc

    return evaluator


def require_criterion(criterion, criteria):
    """Refuse a criterion that is not one of ``criteria``."""
    if criterion not in criteria:
        raise ValueError(f"unknown criterion {criterion!r}")
