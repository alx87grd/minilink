"""
Trajectory extenders: the one connector seam of the RRT family.

`propose(from_state, toward_state, problem, rng)` returns candidate
:class:`~minilink.planning.search.edge.Edge` segments grown from ``from_state``.
Extenders are metric-free and collision-free — they only propose; the
orchestrator selects the best collision-free candidate.

- :class:`KinodynamicExtender` forward-integrates ``problem.sys.f`` under each
  proposed control (works with any system).
- :class:`SteeringExtender` makes one exact candidate via a `SteeringFunction`.
"""

import itertools
from abc import ABC, abstractmethod
from collections.abc import Iterable

import numpy as np

from minilink.planning.search.edge import Edge

# Public API


class TrajectoryExtender(ABC):
    """Propose candidate trajectory segments toward a target state."""

    @abstractmethod
    def propose(self, from_state, toward_state, problem, rng) -> Iterable[Edge]:
        """Return candidate edges grown from ``from_state`` toward ``toward_state``."""
        ...


class KinodynamicExtender(TrajectoryExtender):
    """
    Forward-integration extender: one fixed-time rollout per control.

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
        self.controls = controls
        self.horizon = float(horizon)
        self.n_substeps = int(n_substeps)
        self.dt = self.horizon / self.n_substeps
        self._evaluator = None
        self._sys = None

    def propose(self, from_state, toward_state, problem, rng) -> Iterable[Edge]:
        evaluator = self._compiled(problem.sys)
        return [
            self._rollout(evaluator, from_state, np.asarray(u, dtype=float))
            for u in self._controls(problem, rng)
        ]

    def _controls(self, problem, rng):
        if isinstance(self.controls, int):
            return [problem.U.sample(rng)[0] for _ in range(self.controls)]
        if isinstance(self.controls, str):
            if self.controls != "bang-bang":
                raise ValueError(
                    f"unknown controls preset {self.controls!r}; use 'bang-bang'"
                )
            return bang_bang_controls(problem.U)
        return list(self.controls)

    def _rollout(self, evaluator, from_state, u) -> Edge:
        x = np.asarray(from_state, dtype=float)
        states = [x]
        for _ in range(self.n_substeps):
            x = np.asarray(evaluator.rk4_step(x, u, 0.0, self.dt), dtype=float)
            states.append(x)

        # minimum-time edge cost is the duration
        return Edge(
            states=np.asarray(states),
            inputs=np.tile(u, (self.n_substeps, 1)),
            times=np.arange(self.n_substeps + 1) * self.dt,
            cost=self.horizon,
        )

    def _compiled(self, sys):
        if self._sys is not sys:
            self._evaluator = sys.compile(backend="numpy", verbose=False)
            self._sys = sys
        return self._evaluator


def bang_bang_controls(U):
    """Corners of the box input set *U* plus its centre (axis extremes above 3 inputs)."""
    box = getattr(U, "box", None)
    if box is None:
        raise ValueError(
            "the 'bang-bang' preset needs a box input set (input port bounds); "
            "pass KinodynamicExtender(controls=[...]) for other input sets"
        )
    lower = np.asarray(box.lower, dtype=float)
    upper = np.asarray(box.upper, dtype=float)
    if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)):
        raise ValueError("the 'bang-bang' preset needs finite input bounds")
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
    unique = []
    for u in controls:
        if not any(np.allclose(u, v) for v in unique):
            unique.append(u)
    return unique


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
        self.steering = steering
        self.max_distance = float(max_distance)
        self.resolution = float(resolution)

    def propose(self, from_state, toward_state, problem, rng) -> Iterable[Edge]:
        result = self.steering.connect(
            from_state,
            toward_state,
            max_distance=self.max_distance,
            resolution=self.resolution,
        )
        if result is None:
            return []

        states, inputs, times, cost = result
        return [Edge(states=states, inputs=inputs, times=times, cost=cost)]
