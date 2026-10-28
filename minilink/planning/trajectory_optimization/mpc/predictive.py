"""Model predictive control: a planner's compiled face closed on the loop, one solve per tick."""

from __future__ import annotations

import numpy as np

from minilink.core.backends import array_module
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
    planner, *, inputs=None, delay=True, lookahead=0.0, backend=None
):
    """
    Model predictive control on a planner's compiled face: re-plan from the measured state at each tick, apply the plan.

    ``mpc % dt @ plant`` closes it on a clock of period ``dt``. Two forms:

    - **delayed** (``delay=True``, the default): a native sampled law whose
      state is the plan ``z``. At tick ``k`` it applies the plan solved at the
      previous tick, ``u_k = command(z_k, x̂_k, t_k)``, and solves the next one,
      ``z_{k+1} = solve(P_k, z0=shift(z_k, t_k))``: one period of computation
      delay, as on a real computer, and no algebraic loop. A failed solve keeps
      ``z_k``, the next tick reading the old plan further in.
    - **pure** (``delay=False``): a static law, ``u_k = command(solve(P_k), x̂_k,
      t_k)``, the first action of a solve from the default guess at every tick:
      the ideal law, a reference for studies.

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
        The delayed form (default) or the pure one.
    lookahead : float
        Delay compensation, in seconds: the plan starts at ``t_k + lookahead``
        from the measured start predicted that far ahead under the stored plan
        (the planner's model, the plan's commands applied), so the plan the
        next tick applies starts where the plant will be; ``0`` solves from
        the measurement at its own time.
    backend : str, optional
        The face's backend (default the planner's).

    Ports: inputs per ``inputs``; outputs ``u`` (the command), and on the
    delayed form ``z`` (the plan, for dual rate and logging) and, when the plan
    carries states, ``x_ref`` and ``u_ref`` (the planned state and input now).
    """
    pv = planner.compile(backend)
    require_point_start(planner.problem)
    bindings = bindings_of(inputs)

    if delay:
        return DelayedPredictiveController(pv, bindings, lookahead)

    return PurePredictiveController(pv, bindings, lookahead)


class DelayedPredictiveController(NativeSampledSystem):
    """
    The delayed MPC: a native sampled law whose state is the plan ``z``, ``n = pv.n_z``.

    ``update(z, y, t) = solve(P(y, t), z0=shift(z, t + lookahead))``, the old
    plan kept on a failed solve; ``u = command(z, x̂, t)``. Built by
    :func:`ModelPredictiveController`.
    """

    def __init__(self, pv, bindings, lookahead=0.0):
        super().__init__(n=pv.n_z)
        self.name = "ModelPredictiveController"

        # The face, the bindings and the plan's start at tick 0: the default guess
        self.pv = pv
        self.bindings = bindings
        self.lookahead = float(lookahead)
        self.params = dict(pv.params)
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
        z = x

        # P_k: the planner's tree with the bound leaves read from their ports, the
        # plan's origin t_start = t_k + lookahead, its start predicted to that time
        P = bound_tree(self, params, u, t + lookahead)
        if lookahead > 0.0 and "x_start" in self.bindings:
            x_ahead = predicted_start(pv, z, P["x_start"], t, lookahead, P)
            P = with_leaf(P, "x_start", x_ahead)

        # z_{k+1} = solve(P_k, z0 = shift(z_k, t_k + lookahead)), z_k kept on a failure
        z_guess = pv.shift(z, t + lookahead)
        z_new, status = pv.solve(P, z0=z_guess)
        z_next = xp.where(status["success"], z_new, z)

        return z_next

    def compute_u(self, x, u, t=0, params=None):
        """``u_k = command(z_k, x̂_k, t_k)``: the stored plan read at the tick's time."""
        u_k = self.pv.command(x, measured_start(self, u), t)
        return u_k

    def compute_x_ref(self, x, u, t=0, params=None):
        """The planned state at ``t``."""
        x_ref, _ = self.pv.prediction(x, t)
        return x_ref

    def compute_u_ref(self, x, u, t=0, params=None):
        """The planned input at ``t``."""
        _, u_ref = self.pv.prediction(x, t)
        return u_ref


class PurePredictiveController(System):
    """
    The pure MPC: a static law, ``u_k = command(solve(P_k), x̂_k, t_k)`` from the default guess every tick.

    Built by :func:`ModelPredictiveController` with ``delay=False``; closed on
    a clock (``mpc % dt``), one solve per tick.
    """

    def __init__(self, pv, bindings, lookahead=0.0):
        super().__init__(0)
        self.name = "ModelPredictiveController"

        self.pv = pv
        self.bindings = bindings
        self.lookahead = float(lookahead)
        self.params = dict(pv.params)

        declare_bound_inputs(self, pv, bindings)
        self.add_output_port("u", dim=pv.m, function=self.compute_u, dependencies="all")

    def compute_u(self, x, u, t=0, params=None):
        """``u_k = command(solve(P_k), x̂_k, t_k)``, the first action of a cold solve."""
        params = self.params if params is None else params

        pv = self.pv
        lookahead = self.lookahead

        # z* = solve(P_k) from the default guess, read at the tick's time
        P = bound_tree(self, params, u, t + lookahead)
        z, _ = pv.solve(P)
        u_k = pv.command(z, measured_start(self, u), t)

        return u_k


class PlanFollower(System):
    """
    The plan applied at its own rate: ``u = command(z, t)`` of the plan ``z`` received on its input.

    The fast half of a dual-rate loop, ``(mpc % 0.05) >> (PlanFollower(planner) % 0.01)``:
    the plan solved from ``y_j`` reaches the follower at the next slow tick, one
    slow period of delay. ``planner`` is the MPC's (the same plan layout).
    """

    def __init__(self, planner, backend=None):
        super().__init__(0)
        self.name = "PlanFollower"

        self.pv = planner.compile(backend)

        self.add_input_port("z", dim=self.pv.n_z)
        self.add_output_port(
            "u", dim=self.pv.m, function=self.compute_u, dependencies=("z",)
        )

    def compute_u(self, x, u, t=0, params=None):
        """``u(t) = command(z, t)``: the received plan read at ``t``."""
        u_t = self.pv.command(u, None, t)
        return u_t


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
        leaf = np.asarray(lookup(defaults, path), dtype=float)
        block.add_input_port(port, dim=int(leaf.size))


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


def predicted_start(pv, z, x, t, lookahead, params):
    """``x̂(t + lookahead)``: the planner's model from ``x`` at ``t`` under the stored plan's commands, RK4 steps a quarter of a knot apart, the command linear over each."""
    # Lazy import: the step model loads with the first prediction
    from minilink.core.compile.evaluators.stepping import step_body

    f = pv.problem.sys.f
    plant = params["plant"] or None
    spacing = float(pv.offsets[1] - pv.offsets[0])
    n_sub = max(1, int(np.ceil(4.0 * lookahead / spacing)))
    h = lookahead / n_sub

    def dynamics(x, u, tau):
        return f(x, u, tau, plant)

    # x_{j+1} = Φ(x_j, u(τ), τ_j), the plan's command linear from u(τ_j) to u(τ_j + h)
    for j in range(n_sub):
        tau = t + j * h
        u_j = pv.command(z, x, tau)
        u_next = pv.command(z, x, tau + h)
        x = step_body(dynamics, x, u_j, u_next, tau, h, "rk4", 1)

    return x


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
