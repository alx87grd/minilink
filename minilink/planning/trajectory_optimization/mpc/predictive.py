"""Model predictive control: a planner's compiled face closed on the loop, one solve per tick."""

from __future__ import annotations

import copy
import time

import numpy as np

from minilink.blocks.step import UnitDelay
from minilink.core.backends import array_module
from minilink.core.discrete.sampled_diagram import SampledDiagramSystem
from minilink.core.params import lookup, with_leaf
from minilink.core.system import System

# The default binding: the measured state on the input port y is the plan's start
DEFAULT_INPUTS = {"x_start": "y"}

# The status keys every face's solve reports; a planner's own account comes after them
STANDARD_STATUS = ("success", "iterations", "cost")

###############################################################################
# Public API
###############################################################################


def ModelPredictiveController(
    planner,
    *,
    inputs=None,
    delay=False,
    warm_start=True,
    backend=None,
    verbose=False,
):
    """
    Model predictive control on a planner's compiled face: re-plan from the measured state at each tick, apply the plan.

    ``(mpc % dt) @ plant`` closes it on a clock of period ``dt``. Every MPC is
    one architecture, three pure blocks on the computer::

        y ──► [ solve: (y, z_prev) → z* ] ──┬──────────────► [ command: z → u ] ──► u
                       ▲                     │   (delay: z⁻¹)
                       └───── [ memory: z⁻¹ ] ◄┘

    the planner's solve (``z*_k = solve(P_k, z0)``), the command (``u_k =
    command(z_k, t_k)``) and the plan's memory (a unit delay holding the last
    plan, from the default guess). The two switches are independent:

    - ``warm_start=True`` (the default): each solve starts from the last plan
      shifted to the tick's time, its first state knot at the measurement
      (``pv.shift(z_prev, t_k, x_start=x̂_k)``; the default guess at ``P_k``
      while no plan exists yet); ``False`` solves from the default guess at
      every tick.
    - ``delay=True``: one period of computation delay, a visible unit delay
      ``delay`` on the plan before the command, so tick ``k`` applies the plan
      solved at tick ``k − 1``, as on a real computer; ``False`` (the default)
      applies the plan at once, the solve taking no time (the textbook MPC).

    A solve that returns no usable plan (a non-finite cost or plan) keeps the
    last one. ``P_k`` is the planner's tree (the solve block's ``params``, so
    the loop's ``controller.solve.cost.R`` tunes the plan) with each leaf
    named in ``inputs`` read from its input port (default ``{"x_start": "y"}``:
    the measured state is the plan's start) and ``t_start`` the tick's time.
    Delay compensation is the user's: a predictor block (``pv.predict_start``)
    on the port that binds ``x_start``.

    Parameters
    ----------
    planner : TrajectoryOptimizationPlanner or MPPIPlanner
        The planner solved at each tick; its problem's start is a point.
    inputs : dict, optional
        Leaf path → input port: ``{"x_start": "x_hat", "plant.m": "m_hat"}``.
        A bound leaf stays in the tree as the value used when its port is not
        wired.
    delay : bool
        One period of computation delay before the command (default off).
    warm_start : bool
        Warm-start each solve from the last plan (default on).
    backend : str, optional
        The face's backend (default the planner's).
    verbose : bool
        Print one line per solve: the tick's time, success, iterations, cost
        and the solve's wall time (default off; host runs only, a traced run
        prints nothing).

    Ports: inputs per ``inputs``; outputs ``u`` (the command), ``z`` (the plan
    solved at this tick, for dual rate and logging) and, when the plan carries
    states, ``x_ref`` and ``u_ref`` (the applied plan's state and input now).
    """
    pv = planner.compile(backend)
    require_point_start(planner.problem)
    bindings = bindings_of(inputs)

    return PredictiveController(
        pv, bindings, delay=delay, warm_start=warm_start, verbose=verbose
    )


class PredictiveController(SampledDiagramSystem):
    """
    The MPC box: the planner's solve, the command and the plan's memory, with a unit delay before the command when ``delay``.

    Every block is pure and the loop computes each signal once per tick, so a
    tick solves once. Built by :func:`ModelPredictiveController`; ``mpc % dt``
    puts its blocks on the clock (a step diagram's ``%``).
    """

    def __init__(self, pv, bindings, *, delay=False, warm_start=True, verbose=False):
        super().__init__()
        self.name = "ModelPredictiveController"

        # The face, the bindings and the switches, shared by the blocks
        self.pv = pv
        self.bindings = bindings
        self.delay = bool(delay)
        self.warm_start = bool(warm_start)
        self.verbose = bool(verbose)

        # The blocks: the solve, the command and the plan's memory, which starts at
        # the default guess (no plan yet); the delay, a unit delay on the plan
        solve = PlanSolver(pv, bindings, warm_start=warm_start, verbose=verbose)
        command = PlanCommand(pv)
        memory = PlanMemory(pv)
        self.add_subsystem(solve, "solve")
        self.add_subsystem(command, "command")
        self.add_subsystem(memory, "memory")
        if delay:
            computation = UnitDelay(dim=pv.n_z)
            computation.x0 = np.asarray(pv.initial(), dtype=float)
            self.add_subsystem(computation, "delay")

        # The wiring: the bound leaves in, the last plan back into the solve, the new
        # plan to the memory and (through the delay) to the command
        for port in bindings.values():
            bound = solve.inputs[port]
            self.add_input_port(port, dim=bound.dim, nominal_value=bound.nominal_value)
            self.connect("input", port, "solve", port)
        self.connect("memory", "y", "solve", "z_prev")
        self.connect("solve", "z", "memory", "u")
        if delay:
            self.connect("solve", "z", "delay", "u")
            self.connect("delay", "y", "command", "z")
        else:
            self.connect("solve", "z", "command", "z")
        self.connect_new_output_port("command", "u", "u")
        self.connect_new_output_port("solve", "z", "z")
        if pv.predicts:
            self.connect_new_output_port("command", "x_ref", "x_ref")
            self.connect_new_output_port("command", "u_ref", "u_ref")


class PlanMemory(UnitDelay):
    """The MPC's memory: a unit delay holding the last plan, ``z_prev,k+1 = z*_k``, from the default guess (no plan yet)."""

    def __init__(self, pv):
        super().__init__(dim=pv.n_z)
        self.name = "PlanMemory"

        self.pv = pv
        self.x0 = np.asarray(pv.initial(), dtype=float)


class PlanSolver(System):
    """
    The MPC's solve: the plan solved from the bound leaves, warm-started from the last plan on ``z_prev``.

    A static law, ``z*_k = solve(P_k, z0)``: ``z0`` the last plan rebased to
    the tick's time with its first state knot at the measurement, or the
    default guess at ``P_k`` while ``z_prev`` is no plan yet (and at every tick
    when the solve is cold); ``z_prev`` is kept when the solve returns no
    usable plan.
    """

    def __init__(self, pv, bindings, *, warm_start=True, verbose=False):
        super().__init__(0)
        self.name = "PlanSolver"

        # The face, the bindings, the planner's tree and the default guess
        self.pv = pv
        self.bindings = bindings
        self.warm_start = bool(warm_start)
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
        xp = array_module(u)

        pv = self.pv
        z_prev = u[self.get_input_port_slice("z_prev")]

        # P_k and the guess
        P, z_guess = self.solve_inputs(x, u, t, params)

        # z*_k = solve(P_k, z0), z_prev kept when the solve returns no usable plan
        started = time.perf_counter()
        z_new, status = pv.solve(P, z0=z_guess)
        report_solve(self, t, status, started)
        cost = status["cost"]
        finite_cost = True if cost is None else xp.isfinite(cost)
        usable = xp.all(xp.isfinite(z_new)) & finite_cost
        z_star = xp.where(usable, z_new, z_prev)

        return z_star

    def solve_inputs(self, x, u, t, params=None):
        """``(P_k, z0)`` of the tick's solve: the tree with the bound leaves read from their ports, and the guess (the last plan shifted, else the default guess at ``P_k``)."""
        params = self.params if params is None else params

        pv = self.pv
        z_default = self.default_guess

        z_prev = u[self.get_input_port_slice("z_prev")]
        x_hat = measured_start(self, u)

        # P_k: the planner's tree with the bound leaves read from their ports, the
        # plan's origin t_start = t_k
        P = bound_tree(self, params, u, t)
        if not self.warm_start:
            return P, pv.initial(P)

        # z0 = shift(z_prev, t_k) with x_0 = x̂, the default guess at P_k while
        # z_prev is no plan yet
        x_plan = P["x_start"] if x_hat is not None else None
        z_guess = warm_guess(pv, z_prev, x_plan, P, z_default)

        return P, z_guess


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
    """The command of the plan on its input, on a face already compiled: the MPC's command block, with the plan's state and input now when the plan carries states."""

    def __init__(self, pv):
        System.__init__(self, 0)
        self.name = "PlanCommand"

        declare_follower(self, pv)
        if pv.predicts:
            self.add_output_port(
                "x_ref", dim=pv.n, function=self.compute_x_ref, dependencies=("z",)
            )
            self.add_output_port(
                "u_ref", dim=pv.m, function=self.compute_u_ref, dependencies=("z",)
            )

    def compute_x_ref(self, x, u, t=0, params=None):
        """The applied plan's state at ``t``."""
        pv = self.pv

        x_ref, _ = pv.prediction(u, t)

        return x_ref

    def compute_u_ref(self, x, u, t=0, params=None):
        """The applied plan's input at ``t``."""
        pv = self.pv

        _, u_ref = pv.prediction(u, t)

        return u_ref


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
        return pv.shift(z, t, x_start=x)

    return xp.where(fresh, pv.initial(params), pv.shift(z, t, x_start=x))


def report_solve(block, t, status, started):
    """
    One line for a verbose block's solve: its tick's time, success, iterations, cost and wall time (host runs only).

    The planner's own account follows, any other number its status carries
    (MPPI: ``S_min``, ``S_weighted``, ``effective_samples`` of ``K``, ``exit_rate``).
    """
    values = [t, *(value for value in status.values() if value is not None)]
    if not block.verbose or not all(is_concrete(value) for value in values):
        return
    elapsed = time.perf_counter() - started
    cost = status["cost"]
    cost_text = "n/a" if cost is None else f"{float(cost):.6g}"

    # The planner's account beyond the four the face promises, the effective samples
    # out of the K drawn
    samples = getattr(block.pv, "K", None)
    extras = []
    for name, value in status.items():
        if name in STANDARD_STATUS or value is None:
            continue
        of_K = f"/{samples}" if name == "effective_samples" and samples else ""
        extras.append(f"{name}={float(value):.4g}{of_K}")

    print(
        f"MPC @ t={float(t):.2f}s  success={bool(status['success'])}  "
        f"iterations={status['iterations']}  J={cost_text}  solve={elapsed:.3f}s"
        + "".join(f"  {extra}" for extra in extras)
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
    """Whether the plan carries its states, so the face predicts ``x`` (its own word)."""
    return pv.predicts


def require_point_start(problem):
    """Refuse a planner whose problem starts from a set or a law: the MPC binds a point."""
    # Lazy import: the sets load with the core, this module with the planners
    from minilink.core.sets import SingletonSet

    if problem.start_law is not None or not isinstance(problem.X0, SingletonSet):
        raise ValueError(
            "the MPC binds the measured state to the plan's start: give the planner's "
            "problem a point start (x_start), not a set or a law (its X0 is "
            f"{type(problem.X0).__name__}; a plan would start anywhere in it)"
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
