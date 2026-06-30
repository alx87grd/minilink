"""
Steering functions: exact two-point connectors with inverse dynamics.

A :class:`SteeringFunction` connects two states with a dynamically-feasible
segment for a specific kinematic model — it returns the ``states``, ``inputs``,
and ``times`` that realise the path under that model's ``f``, so the result must
match ``problem.sys`` (re-simulating ``inputs`` reproduces ``states``). It also
exposes :meth:`distance`, the natural nearest-neighbour metric for that model.

:class:`StraightLineSteering` connects a single integrator ``dx = u`` (holonomic
point); :class:`DubinsSteering` connects a forward-only kinematic bicycle along
the shortest Dubins curve.
"""

from abc import ABC, abstractmethod

import numpy as np

from minilink.planning.search.dubins import shortest_path


class SteeringFunction(ABC):
    """Exact local connector for a kinematic model."""

    @abstractmethod
    def connect(self, x0, x1, *, max_distance, resolution):
        """
        Return ``(states, inputs, times, cost)`` from ``x0`` toward ``x1``.

        The segment is truncated to arc length ``max_distance`` and sampled at
        spacing ``resolution``. Returns ``None`` when ``x0`` and ``x1`` coincide.
        """
        ...

    def distance(self, a, b) -> float:
        """Nearest-neighbour distance under this model (default Euclidean)."""
        return float(
            np.linalg.norm(np.asarray(b, dtype=float) - np.asarray(a, dtype=float))
        )


class StraightLineSteering(SteeringFunction):
    """
    Constant-velocity straight line for a single integrator ``dx = u``.

    The input is the velocity, so a unit-speed line from ``x0`` toward ``x1`` is
    exactly feasible: ``u = speed * direction`` reproduces the sampled states.
    """

    def __init__(self, speed: float = 1.0) -> None:
        self.speed = float(speed)

    def connect(self, x0, x1, *, max_distance, resolution):
        x0 = np.asarray(x0, dtype=float)
        x1 = np.asarray(x1, dtype=float)
        delta = x1 - x0
        dist = float(np.linalg.norm(delta))
        if dist < 1e-12:
            return None

        reach = min(dist, float(max_distance))
        direction = delta / dist
        n_seg = max(1, int(np.ceil(reach / float(resolution))))

        arc = np.linspace(0.0, reach, n_seg + 1)
        states = x0[None, :] + arc[:, None] * direction[None, :]
        inputs = np.tile(self.speed * direction, (n_seg, 1))
        times = np.linspace(0.0, reach / self.speed, n_seg + 1)
        return states, inputs, times, reach


class DubinsSteering(SteeringFunction):
    """
    Forward Dubins-car connector for a kinematic bicycle.

    Connects ``(x, y, theta)`` poses along the shortest forward Dubins curve of
    minimum turning radius ``wheelbase / tan(max_steering)``, at constant
    ``speed``. The inputs are ``(speed, steering)`` with ``steering`` in
    ``{+max_steering, 0, -max_steering}`` — exactly the bicycle controls that
    trace each circular arc, so the segment is feasible for the matching
    ``KinematicBicycle``/``KinematicCar``.
    """

    def __init__(
        self, *, wheelbase: float, max_steering: float, speed: float = 1.0
    ) -> None:
        self.wheelbase = float(wheelbase)
        self.max_steering = float(max_steering)
        self.speed = float(speed)
        self.radius = self.wheelbase / np.tan(self.max_steering)

    def connect(self, x0, x1, *, max_distance, resolution):
        path = shortest_path(x0, x1, self.radius)
        if path is None:
            return None
        _, segments, _ = path

        states, signs, steps = self._sample(
            np.asarray(x0, dtype=float), segments, max_distance, resolution
        )
        if len(steps) < 1:
            return None

        inputs = np.array([[self.speed, sign * self.max_steering] for sign in signs])
        times = np.concatenate([[0.0], np.cumsum(steps) / self.speed])
        cost = float(np.sum(steps))
        return np.asarray(states), inputs, times, cost

    def distance(self, a, b) -> float:
        path = shortest_path(a, b, self.radius)
        return float("inf") if path is None else float(path[2])

    def _sample(self, q0, segments, max_distance, resolution):
        """Walk the Dubins segments, sampling exact bicycle arcs at ``resolution``."""
        x, y, theta = float(q0[0]), float(q0[1]), float(q0[2])
        states = [np.array([x, y, theta])]
        signs: list[float] = []
        steps: list[float] = []
        remaining = float(max_distance)

        for sign, arc in segments:
            arc = min(arc, remaining)
            if arc <= 1e-12:
                continue
            curvature = sign / self.radius
            n = max(1, int(np.ceil(arc / float(resolution))))
            ds = arc / n
            for _ in range(n):
                theta_next = theta + curvature * ds
                if abs(curvature) < 1e-12:
                    x += ds * np.cos(theta)
                    y += ds * np.sin(theta)
                else:
                    x += (np.sin(theta_next) - np.sin(theta)) / curvature
                    y += -(np.cos(theta_next) - np.cos(theta)) / curvature
                theta = theta_next
                states.append(np.array([x, y, theta]))
                signs.append(sign)
                steps.append(ds)
            remaining -= arc
            if remaining <= 1e-12:
                break

        return states, signs, steps
