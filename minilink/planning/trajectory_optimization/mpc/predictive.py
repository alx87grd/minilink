"""Model predictive control: a planner's compiled face closed on the loop, one solve per tick."""

from __future__ import annotations

import copy
import time

import numpy as np

from minilink.blocks.step import UnitDelay
from minilink.core.backends import array_module
from minilink.core.compile.evaluators.base import assigned
from minilink.core.discrete.sampled_diagram import SampledDiagramSystem
from minilink.core.discrete.sampled_system import NativeSampledSystem
from minilink.core.params import lookup, with_leaf
from minilink.core.system import System
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationTranscription,
)

# The default binding: the measured state on the input port y is the plan's start
DEFAULT_INPUTS = {"x_start": "y"}

###############################################################################
# Public API
###############################################################################


def ModelPredictiveController(
    planner,
    *,
    inputs=None,
    delay=True,
    warm_start=True,
    lookahead=0.0,
    backend=None,
    verbose=False,
):
    """
    Model predictive control on a planner's compiled face: re-plan from the measured state at each tick, apply the plan.

    ``mpc % dt @ plant`` closes it on a clock of period ``dt``. Three forms:

    - **delayed** (``delay=True``, the default): a native sampled law whose
      state is the plan ``z``. At tick ``k`` it applies the plan solved at the
      previous tick, ``u_k = command(z_k, x̂_k, t_k)``, and solves the next one,
      ``z_{k+1} = solve(P_k, z0=shift(z_k, t_k))``: one period of computation
      delay, as on a real computer, and no algebraic loop. A solve that returns
      no usable plan (a non-finite cost or plan) keeps ``z_k``, the next tick
      reading the old plan further in; a solver stopped at its iteration limit
      still returns its last iterate, which is used.
    - **zero delay, warm-started** (``delay=False``): at tick ``k`` it solves
      from the measurement, warm-started from the last plan shifted one period
      with its first state knot at the measurement, ``z*_k = solve(P_k,
      z0=warm(z_{k-1}, x̂_k, t_k))``, and applies it at once, ``u_k =
      command(z*_k, x̂_k, t_k)``: the solve takes no time, the classic textbook
      MPC. It is a small sampled diagram of pure blocks, the solve, the
      command and a unit delay holding the last plan, so a tick solves once.
    - **pure** (``delay=False, warm_start=False``): a static law, ``u_k =
      command(solve(P_k), x̂_k, t_k)``, the first action of a solve from the
      default guess at every tick: the ideal law, a reference for studies.

    ``P_k`` is the planner's tree (the block's ``params``, so the loop's
    ``controller.cost.Q`` tunes the plan) with each leaf named in ``inputs``
    read from its input port (default ``{"x_start": "y"}``: the measured state
    is the plan's start) and ``t_start`` the tick's time plus ``lookahead``.

    Parameters
    ----------
    planner : TrajectoryOptimizationPlanner
        The planner solved at each tick; its problem's start is a point.
    inputs : dict, optional
        Leaf path → input port: ``{"x_start": "x_hat", "plant.m": "m_hat"}``.
        A bound leaf stays in the tree as the value used when its port is not
        wired.
    delay : bool
        The delayed form (default), or a zero-delay one.
    warm_start : bool
        With ``delay=False``: warm-start each solve from the last plan
        (default), or solve cold from the default guess (the pure form). The
        delayed form always warm-starts.
    lookahead : float
        Delay compensation, in seconds: the plan starts at ``t_k + lookahead``
        from the measured start predicted that far ahead under the planner's
        model with the command the block applies now held, so the plan the
        next tick applies starts where the plant will be; ``0`` solves from
        the measurement at its own time.
    backend : str, optional
        The face's backend (default the planner's).
    verbose : bool
        Print one line per solve: the tick's time, success, iterations, cost
        and the solve's wall time (default off; host runs only, a traced run
        prints nothing). A read-back that solves again (the plant's input read
        after a zero-delay run) prints again.

    Ports: inputs per ``inputs``; outputs ``u`` (the command), and on the
    delayed and zero-delay forms ``z`` (the plan, for dual rate and logging);
    on the delayed form, when the plan carries states, ``x_ref`` and ``u_ref``
    (the planned state and input now).
    """
    pv = planner.compile(backend)
    require_point_start(planner.problem)
    bindings = bindings_of(inputs)

    if delay:
        return DelayedPredictiveController(pv, bindings, lookahead, verbose)
    if warm_start:
        return WarmStartedPredictiveController(pv, bindings, lookahead, verbose)

    return PurePredictiveController(pv, bindings, lookahead, verbose)


class DelayedPredictiveController(NativeSampledSystem):
    """
    The delayed MPC: a native sampled law whose state is the plan ``z``, ``n = pv.n_z``.

    Its state update solves the next plan from the bound leaves, warm-started
    from the stored plan rebased to the plan's origin, and keeps the stored
    plan when the solve returns none usable; its command reads the stored plan
    at the tick's time. Built by :func:`ModelPredictiveController`.
    """

    def __init__(self, pv, bindings, lookahead=0.0, verbose=False):
        super().__init__(n=pv.n_z)
        self.name = "ModelPredictiveController"

        # The face, the bindings and the plan's start at tick 0: the default guess
        self.pv = pv
        self.bindings = bindings
        self.lookahead = float(lookahead)
        self.verbose = bool(verbose)
        self.params = copy.deepcopy(pv.params)
        self.x0 = np.asarray(pv.initial(), dtype=float)
        self.state.labels = [f"z[{i}]" for i in range(pv.n_z)]

        # Ports: the bound leaves in, the command, the plan and the references out
        declare_bound_inputs(self, pv, bindings)
        self.add_output_port("u", dim=pv.m, function=self.compute_u, dependencies=())
        self.add_output_port(
            "z", dim=pv.n_z, function=self.compute_state, dependencies=()
        )
        if predicts(pv):
            self.add_output_port(
                "x_ref", dim=pv.n, function=self.compute_x_ref, dependencies=()
            )
            self.add_output_port(
                "u_ref", dim=pv.m, function=self.compute_u_ref, dependencies=()
            )

    def update(self, x, u, t, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        pv = self.pv
        lookahead = self.lookahead
        bindings = self.bindings
        z = x
        t_plan = t + lookahead

        # P_k: the planner's tree with the bound leaves read from their ports, the
        # plan's origin t_start = t_k + lookahead, its start predicted to that time
        P = bound_tree(self, params, u, t_plan)
        if lookahead > 0.0 and "x_start" in bindings:
            u_k = pv.command(z, P["x_start"], t)
            x_ahead = predicted_start(pv, P["x_start"], u_k, t, lookahead, P)
            P = with_leaf(P, "x_start", x_ahead)

        # z_{k+1} = solve(P_k, z0 = shift(z_k, t_k + lookahead)), z_k kept when the
        # solve returns no usable plan
        z_guess = pv.shift(z, t_plan)
        started = time.perf_counter()
        z_new, status = pv.solve(P, z0=z_guess)
        report_solve(self, t, status, started)
        cost = status["cost"]
        finite_cost = True if cost is None else xp.isfinite(cost)
        usable = xp.all(xp.isfinite(z_new)) & finite_cost
        z_next = xp.where(usable, z_new, z)

        return z_next

    def compute_u(self, x, u, t=0, params=None):
        """The command: the stored plan read at the tick's time."""
        pv = self.pv
        x_hat = measured_start(self, u)

        # u_k = command(z_k, x̂_k, t_k)
        u_k = pv.command(x, x_hat, t)

        return u_k

    def compute_x_ref(self, x, u, t=0, params=None):
        """The planned state at ``t``."""
        pv = self.pv

        x_ref, _ = pv.prediction(x, t)

        return x_ref

    def compute_u_ref(self, x, u, t=0, params=None):
        """The planned input at ``t``."""
        pv = self.pv

        _, u_ref = pv.prediction(x, t)

        return u_ref


class WarmStartedPredictiveController(SampledDiagramSystem):
    """
    The zero-delay MPC: a small sampled diagram, the planner's solve, its command and a unit delay holding the last plan.

    ::

        y ──► [ solve: (y, z_prev) → z* ] ──► [ command: z* → u ] ──► u
                       ▲                   │
                       └────── [ z⁻¹ ] ◄───┘

    At each tick the solve block solves from the bound leaves, warm-started
    from the last plan the delay holds, and the command block applies the new
    plan at once; the delay keeps it for the next tick. Every block is pure and
    the loop computes each signal once per tick, so a tick solves once. Built
    by :func:`ModelPredictiveController` with ``delay=False``; ``mpc % dt``
    puts its three blocks on the clock (a step diagram's ``%``).

    Ports: inputs per ``inputs``; outputs ``u`` (the command) and ``z`` (the
    plan solved at this tick, for dual rate and logging).
    """

    def __init__(self, pv, bindings, lookahead=0.0, verbose=False):
        super().__init__()
        self.name = "ModelPredictiveController"

        # The face and the bindings, shared by the blocks
        self.pv = pv
        self.bindings = bindings
        self.lookahead = float(lookahead)
        self.verbose = bool(verbose)

        # The blocks: the solve, the command and the plan's memory, which starts at
        # the default guess (no plan yet); mpc % dt clocks each of them
        solve = PlanSolver(pv, bindings, lookahead, verbose)
        command = PlanCommand(pv)
        memory = PlanMemory(pv)
        self.add_subsystem(solve, "solve")
        self.add_subsystem(command, "command")
        self.add_subsystem(memory, "memory")

        # The wiring: the bound leaves in, the last plan back into the solve, the
        # new plan to the command and to the memory
        for port in bindings.values():
            bound = solve.inputs[port]
            self.add_input_port(port, dim=bound.dim, nominal_value=bound.nominal_value)
            self.connect("input", port, "solve", port)
        self.connect("memory", "y", "solve", "z_prev")
        self.connect("solve", "z", "command", "z")
        self.connect("solve", "z", "memory", "u")
        self.connect_new_output_port("command", "u", "u")
        self.connect_new_output_port("solve", "z", "z")


class PlanMemory(UnitDelay):
    """The zero-delay MPC's memory: a unit delay holding the last plan, ``z_prev,k+1 = z*_k``, from the default guess (no plan yet)."""

    def __init__(self, pv):
        super().__init__(dim=pv.n_z)
        self.name = "PlanMemory"

        self.pv = pv
        self.x0 = np.asarray(pv.initial(), dtype=float)


class PlanSolver(System):
    """
    The zero-delay MPC's solve: the plan solved from the bound leaves, warm-started from the last plan on ``z_prev``.

    A static law, ``z*_k = solve(P_k, z0=warm(z_prev, x̂_k))``: the last plan
    rebased to the plan's origin with its first state knot at the measurement,
    or the default guess at ``P_k`` while ``z_prev`` is no plan yet (the
    default guess at the planner's own start); ``z_prev`` is kept when the
    solve returns no usable plan.
    """

    def __init__(self, pv, bindings, lookahead=0.0, verbose=False):
        super().__init__(0)
        self.name = "PlanSolver"

        # The face, the bindings, the planner's tree and the default guess
        self.pv = pv
        self.bindings = bindings
        self.lookahead = float(lookahead)
        self.verbose = bool(verbose)
        self.params = copy.deepcopy(pv.params)
        self.default_guess = np.asarray(pv.initial(), dtype=float)

        # Ports: the bound leaves and the last plan in, the new plan out
        declare_bound_inputs(self, pv, bindings)
        self.add_input_port("z_prev", dim=pv.n_z, nominal_value=self.default_guess)
        ports = (*bindings.values(), "z_prev")
        self.add_output_port(
            "z", dim=pv.n_z, function=self.compute_z, dependencies=ports
        )

    def compute_z(self, x, u, t=0, params=None):
        """The plan solved at this tick."""
        params = self.params if params is None else params
        xp = array_module(u)

        pv = self.pv
        lookahead = self.lookahead
        z_default = self.default_guess

        z_prev = u[self.get_input_port_slice("z_prev")]
        x_hat = measured_start(self, u)

        # P_k: the planner's tree with the bound leaves read from their ports and
        # the plan's origin t_start = t_k + lookahead
        P = bound_tree(self, params, u, t + lookahead)

        # z*_k = solve(P_k, z0 = shift(z_prev, t_k + lookahead) with x_0 = x̂_k), from
        # the default guess at P_k while z_prev is no plan yet; z_prev kept when the
        # solve returns no usable plan
        z_guess = warm_guess(pv, z_prev, x_hat, P, z_default)
        started = time.perf_counter()
        z_new, status = pv.solve(P, z0=z_guess)
        report_solve(self, t, status, started)
        cost = status["cost"]
        finite_cost = True if cost is None else xp.isfinite(cost)
        usable = xp.all(xp.isfinite(z_new)) & finite_cost
        z_star = xp.where(usable, z_new, z_prev)

        return z_star


class PurePredictiveController(System):
    """
    The pure MPC: a static law, the first action of a solve from the default guess at every tick.

    Built by :func:`ModelPredictiveController` with ``delay=False``; closed on
    a clock (``mpc % dt``), one solve per tick.
    """

    def __init__(self, pv, bindings, lookahead=0.0, verbose=False):
        super().__init__(0)
        self.name = "ModelPredictiveController"

        self.pv = pv
        self.bindings = bindings
        self.lookahead = float(lookahead)
        self.verbose = bool(verbose)
        self.params = copy.deepcopy(pv.params)

        declare_bound_inputs(self, pv, bindings)
        self.add_output_port("u", dim=pv.m, function=self.compute_u, dependencies="all")

    def compute_u(self, x, u, t=0, params=None):
        """The first action of a cold solve from the tick's bound leaves."""
        params = self.params if params is None else params

        pv = self.pv
        lookahead = self.lookahead

        # z* = solve(P_k) from the default guess, read at the tick's time
        P = bound_tree(self, params, u, t + lookahead)
        started = time.perf_counter()
        z, status = pv.solve(P)
        report_solve(self, t, status, started)
        x_hat = measured_start(self, u)

        # u_k = command(z*, x̂_k, t_k)
        u_k = pv.command(z, x_hat, t)

        return u_k


class PlanFollower(System):
    """
    The plan applied at its own rate: the command of the plan received on its input, read at its own ticks.

    The fast half of a dual-rate loop, ``(mpc % 0.05) >> (PlanFollower(planner) % 0.01)``:
    the plan solved from ``y_j`` reaches the follower at the next slow tick, one
    slow period of delay. ``planner`` is the MPC's (the same plan layout).
    """

    def __init__(self, planner, backend=None):
        super().__init__(0)
        self.name = "PlanFollower"

        declare_follower(self, planner.compile(backend))

    def compute_u(self, x, u, t=0, params=None):
        """The received plan read at ``t``."""
        pv = self.pv

        # u(t) = command(z, t), an open-loop plan reading no state
        u_t = pv.command(u, None, t)

        return u_t


class PlanCommand(PlanFollower):
    """The command of the plan on its input, on a face already compiled: the zero-delay MPC's command block."""

    def __init__(self, pv):
        System.__init__(self, 0)
        self.name = "PlanCommand"

        declare_follower(self, pv)


###############################################################################
# Internal machinery
###############################################################################


def bindings_of(inputs):
    """The leaf path → port bindings, ``{"x_start": "y"}`` by default."""
    bindings = dict(DEFAULT_INPUTS if inputs is None else inputs)
    require_unique_ports(bindings)

    return bindings


def declare_bound_inputs(block, pv, bindings):
    """One input port per bound leaf, sized as the leaf."""
    defaults = pv.params
    for path, port in bindings.items():
        leaf = lookup(defaults, path)
        require_bindable_leaf(path, leaf)
        value = np.asarray(leaf, dtype=float).reshape(-1)
        block.add_input_port(port, dim=int(value.size), nominal_value=value)


def bound_tree(block, params, u, t_start):
    """``P``: ``params`` with each bound leaf read from its port and ``t_start`` set."""
    xp = array_module(u)

    P = params
    for path, port in block.bindings.items():
        value = u[block.get_input_port_slice(port)]
        shape = xp.shape(lookup(block.params, path))
        P = with_leaf(P, path, xp.reshape(value, shape))
    P = with_leaf(P, "t_start", t_start)

    return P


def predicted_start(pv, x, u_k, t, lookahead, params):
    """``x̂(t + lookahead)``: the planner's model from ``x`` at ``t`` with the applied command ``u_k`` held, RK4 steps a quarter of a knot apart."""
    # Lazy import: the step model loads with the first prediction
    from minilink.core.compile.evaluators.stepping import step_body

    f = pv.problem.sys.f
    plant = params["plant"] or None
    spacing = float(pv.offsets[1] - pv.offsets[0])
    n_sub = max(1, int(np.ceil(4.0 * lookahead / spacing)))

    def dynamics(x, u, tau):
        return f(x, u, tau, plant)

    # x̂(t + Δ) = Φ(x, u_k, t), the command held over the lookahead as the plant sees it
    x_ahead = step_body(dynamics, x, u_k, u_k, t, lookahead, "rk4", n_sub)

    return x_ahead


def warm_guess(pv, z, x, params, z_default):
    """
    The warm start of a solve on ``params`` from the last plan ``z``: ``z`` rebased to the plan's origin, its first state knot at the measured ``x``.

    While ``z`` is still ``z_default`` (the block's start, no plan yet) the
    guess is the default guess at ``params``, from the measured start.
    """
    t = params["t_start"]
    xp = array_module(z, t)

    # No plan yet: the default guess from the measured start
    fresh = xp.all(z == z_default)
    if is_concrete(fresh):
        if bool(fresh):
            return pv.initial(params)
        return pinned_shift(pv, z, x, t)

    return xp.where(fresh, pv.initial(params), pinned_shift(pv, z, x, t))


def pinned_shift(pv, z, x, t):
    """The plan rebased to ``t`` (``pv.shift``), its first state knot at ``x`` when the plan carries states."""
    z_shifted = pv.shift(z, t)
    if x is None or not predicts(pv):
        return z_shifted

    # The state rows come first in z, N knots each: x_i's first knot at i·N
    xp = array_module(z_shifted, x)
    first_knots = xp.arange(pv.n) * pv.offsets.size
    z_guess = assigned(xp, z_shifted, first_knots, xp.reshape(x, (pv.n,)))

    return z_guess


def report_solve(block, t, status, started):
    """One line for a verbose block's solve: its tick's time, success, iterations, cost and wall time (host runs only)."""
    if not block.verbose or not is_concrete(status["success"]):
        return
    elapsed = time.perf_counter() - started
    cost = status["cost"]
    cost_text = "n/a" if cost is None else f"{float(cost):.6g}"

    print(
        f"MPC @ t={float(t):.2f}s  success={bool(status['success'])}  "
        f"iterations={status['iterations']}  J={cost_text}  solve={elapsed:.3f}s"
    )


def is_concrete(value) -> bool:
    """Whether ``value`` has a value on the host (a traced one has none)."""
    try:
        bool(value)
    except TypeError:  # a tracer cannot be concretized
        return False

    return True


def declare_follower(block, pv):
    """A follower's face and ports: the plan ``z`` in, its command ``u`` out."""
    block.pv = pv
    block.add_input_port("z", dim=pv.n_z)
    block.add_output_port("u", dim=pv.m, function=block.compute_u, dependencies=("z",))


def measured_start(block, u):
    """The plan's start read from its port (the measured state), ``None`` when unbound."""
    port = block.bindings.get("x_start")
    if port is None:
        return None

    return u[block.get_input_port_slice(port)]


def predicts(pv) -> bool:
    """Whether the plan carries its states, so the face predicts ``x`` (collocation)."""
    return isinstance(pv.transcription, DirectCollocationTranscription)


def require_point_start(problem):
    """Refuse a planner whose problem starts from a set or a law: the MPC binds a point."""
    if problem.start_law is not None:
        raise ValueError(
            "the MPC binds the measured state to the plan's start: give the planner's "
            "problem a point start (x_start), not a set or a law"
        )


def require_unique_ports(bindings):
    """Refuse two leaves bound to one port."""
    ports = list(bindings.values())
    if len(set(ports)) != len(ports):
        raise ValueError(f"each bound leaf needs its own port, got {bindings}")


def require_bindable_leaf(path, leaf):
    """Refuse a binding that names a subtree: a port carries one leaf."""
    if isinstance(leaf, dict):
        raise ValueError(
            f"{path!r} names a subtree of the planner's tree ({sorted(leaf)}): bind "
            "each leaf to its own port"
        )
