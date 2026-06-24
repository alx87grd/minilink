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
    controls : sequence of array_like, or int
        Either an explicit list of control inputs (motion primitives), or an int
        ``n`` meaning "``n`` random samples of ``problem.U`` per extension".
    horizon : float
        Edge duration; the control is held over ``n_substeps`` of ``dt = horizon / n_substeps``.
    n_substeps : int
        Integration steps per edge.
    """

    def __init__(self, controls, *, horizon: float = 0.5, n_substeps: int = 5) -> None:
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
