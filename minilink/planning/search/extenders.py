"""Trajectory extenders: the candidate edges the RRT family grows toward a sample."""

import itertools
from abc import ABC, abstractmethod
from collections.abc import Iterable

import numpy as np

from minilink.planning.search.edge import Edge

###############################################################################
# Public API
###############################################################################


class TrajectoryExtender(ABC):
    """
    Propose candidate trajectory segments toward a target state.

    ``propose(from_state, toward_state, problem, rng)`` returns candidate
    :class:`~minilink.planning.search.edge.Edge` segments grown from
    ``from_state``. Extenders are metric-free and collision-free: they only
    propose, and the planner selects the best collision-free candidate.
    :class:`KinodynamicExtender` forward-integrates ``problem.sys.f`` under each
    proposed control (any system); :class:`SteeringExtender` makes one exact
    candidate through a ``SteeringFunction``.
    """

    @abstractmethod
    def propose(self, from_state, toward_state, problem, rng) -> Iterable[Edge]:
        """Return candidate edges grown from ``from_state`` toward ``toward_state``."""
        ...


class KinodynamicExtender(TrajectoryExtender):
    """
    Forward-integration extender: one fixed-time rollout per control.

    Works with any system: each candidate integrates ``problem.sys.f`` under
    one held control.

    Parameters
    ----------
    controls : sequence of array_like, int, or ``"bang-bang"``
        An explicit list of control inputs (motion primitives); an int ``n``
        meaning "``n`` random samples of ``problem.U`` per extension"; or
        ``"bang-bang"`` (default): the corners of the box ``problem.U`` plus
        its centre, so every planner call works from the input bounds alone.
    horizon : float
        Edge duration; the control is held over ``n_substeps`` of ``dt = horizon / n_substeps``.
    n_substeps : int
        Integration steps per edge.
    """

    def __init__(
        self, controls="bang-bang", *, horizon: float = 0.3, n_substeps: int = 6
    ) -> None:
        # The controls, and the edge's duration split into n_substeps steps of dt
        self.controls = controls
        self.horizon = float(horizon)
        self.n_substeps = int(n_substeps)
        self.dt = self.horizon / self.n_substeps

        # Memory: the NumPy evaluator of the last system, compiled once per system
        self._evaluator = None
        self._sys = None

    def propose(self, from_state, toward_state, problem, rng) -> Iterable[Edge]:
        evaluator = self.compiled(problem.sys)
        controls = self.candidate_controls(problem, rng)

        # One fixed-time rollout per control
        edges = [
            self.rollout(evaluator, from_state, np.asarray(u, dtype=float))
            for u in controls
        ]

        return edges

    def rollout(self, evaluator, from_state, u) -> Edge:
        """Hold ``u`` over the horizon: ``n_substeps`` RK4 steps of ``dt``."""
        N = self.n_substeps
        dt = self.dt
        T = self.horizon

        # x_{k+1} = RK4(x_k, u, dt), the control held over every step
        x = np.asarray(from_state, dtype=float)
        xs = [x]
        for _ in range(N):
            x = np.asarray(evaluator.rk4_step(x, u, 0.0, dt), dtype=float)
            xs.append(x)

        # The knots, the held input on each step, the knot times, and the
        # minimum-time cost: the duration T
        states = np.asarray(xs)
        inputs = np.tile(u, (N, 1))
        times = np.arange(N + 1) * dt
        cost = T

        return Edge(states=states, inputs=inputs, times=times, cost=cost)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def compiled(self, sys):
        """The NumPy evaluator of ``sys``, compiled once per system."""
        if self._sys is not sys:
            self._evaluator = sys.compile(backend="numpy", verbose=False)
            self._sys = sys

        return self._evaluator

    def candidate_controls(self, problem, rng):
        """The controls of one extension: ``n`` draws of ``problem.U``, the bang-bang set, or the list."""
        controls = self.controls
        if isinstance(controls, int):
            return [problem.U.sample(rng) for _ in range(controls)]

        if isinstance(controls, str):
            require_known_preset(controls)
            return bang_bang_controls(problem.U)

        return list(controls)

    _controls = candidate_controls  # ripple


class SteeringExtender(TrajectoryExtender):
    """
    Exact-connection extender: one candidate via a `SteeringFunction`.

    Parameters
    ----------
    steering : SteeringFunction
        Local connector whose `connect` returns a feasible ``(states, inputs,
        times, cost)`` for the model — must match ``problem.sys``.
    max_distance : float
        Arc length the edge is truncated to.
    resolution : float
        Arc-length spacing of the sampled edge (collision-check density).
    """

    def __init__(
        self, steering, *, max_distance: float = 0.5, resolution: float = 0.05
    ) -> None:
        # The connector, and the edge's length and sample spacing
        self.steering = steering
        self.max_distance = float(max_distance)
        self.resolution = float(resolution)

    def propose(self, from_state, toward_state, problem, rng) -> Iterable[Edge]:
        steering = self.steering
        max_distance = self.max_distance
        resolution = self.resolution

        # One exact connection, truncated to max_distance; none when it fails
        result = steering.connect(
            from_state,
            toward_state,
            max_distance=max_distance,
            resolution=resolution,
        )
        if result is None:
            return []

        states, inputs, times, cost = result
        edges = [Edge(states=states, inputs=inputs, times=times, cost=cost)]

        return edges


def bang_bang_controls(U):
    """Corners of the box input set *U* plus its centre (axis extremes above 3 inputs)."""
    box = require_box_input_set(U)
    lower = np.asarray(box.lower, dtype=float)
    upper = np.asarray(box.upper, dtype=float)
    require_finite_input_bounds(lower, upper)

    # The centre, then every corner of the box (above 3 inputs, each axis at
    # either bound with the others at the centre)
    centre = 0.5 * (lower + upper)
    controls = [centre]
    if lower.size <= 3:
        for corner in itertools.product(*zip(lower, upper)):
            controls.append(np.asarray(corner, dtype=float))
    else:
        for i in range(lower.size):
            for value in (lower[i], upper[i]):
                u = centre.copy()
                u[i] = value
                controls.append(u)

    # Each distinct control once
    unique = []
    for u in controls:
        if not any(np.allclose(u, v) for v in unique):
            unique.append(u)

    return unique


###############################################################################
# Internal machinery
###############################################################################


def require_known_preset(controls):
    """Refuse a controls preset other than ``"bang-bang"``."""
    if controls != "bang-bang":
        raise ValueError(f"unknown controls preset {controls!r}; use 'bang-bang'")


def require_box_input_set(U):
    """Refuse an input set without a box (input port bounds); return its box."""
    box = getattr(U, "box", None)
    if box is None:
        raise ValueError(
            "the 'bang-bang' preset needs a box input set (input port bounds); "
            "pass KinodynamicExtender(controls=[...]) for other input sets"
        )

    return box


def require_finite_input_bounds(lower, upper):
    """Refuse an input box with an infinite bound."""
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise ValueError("the 'bang-bang' preset needs finite input bounds")
