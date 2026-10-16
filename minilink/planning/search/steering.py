"""Steering functions: exact two-point connectors with inverse dynamics."""

from abc import ABC, abstractmethod

import numpy as np

from minilink.planning.search.dubins import shortest_path

###############################################################################
# Public API
###############################################################################


class SteeringFunction(ABC):
    """
    Exact local connector for a kinematic model.

    :meth:`connect` returns the ``states``, ``inputs`` and ``times`` that
    realise a dynamically-feasible segment under that model's ``f``, so the
    result must match ``problem.sys`` (re-simulating ``inputs`` reproduces
    ``states``). :meth:`distance` is the natural nearest-neighbour metric for
    that model.
    """

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
        b = np.asarray(b, dtype=float)
        a = np.asarray(a, dtype=float)

        dist = float(np.linalg.norm(b - a))

        return dist


class StraightLineSteering(SteeringFunction):
    """
    Constant-velocity straight line for a single integrator ``dx = u``.

    The input is the velocity, so a unit-speed line from ``x0`` toward ``x1`` is
    exactly feasible: ``u = speed * direction`` reproduces the sampled states.
    """

    def __init__(self, speed: float = 1.0) -> None:
        self.speed = float(speed)

    def connect(self, x0, x1, *, max_distance, resolution):
        v = self.speed

        x0 = np.asarray(x0, dtype=float)
        x1 = np.asarray(x1, dtype=float)

        # The distance to x1; coincident states have no segment
        delta = x1 - x0
        dist = float(np.linalg.norm(delta))
        if dist < 1e-12:
            return None

        # The arc length the segment reaches, its unit direction d, and the number of
        # samples that keeps the spacing under the resolution
        reach = min(dist, float(max_distance))
        direction = delta / dist
        n_seg = max(1, int(np.ceil(reach / float(resolution))))

        # x(s) = x0 + s d on s ∈ [0, reach], driven by the constant input u = v d,
        # so that t = s / v
        arc = np.linspace(0.0, reach, n_seg + 1)
        states = x0[None, :] + arc[:, None] * direction[None, :]
        inputs = np.tile(v * direction, (n_seg, 1))
        times = np.linspace(0.0, reach / v, n_seg + 1)
        cost = reach

        return states, inputs, times, cost


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
        # Parameters
        self.wheelbase = float(wheelbase)
        self.max_steering = float(max_steering)
        self.speed = float(speed)

        # The minimum turning radius r = L / tan(δ_max)
        L = self.wheelbase
        delta_max = self.max_steering
        self.radius = L / np.tan(delta_max)

    def connect(self, x0, x1, *, max_distance, resolution):
        v = self.speed
        delta_max = self.max_steering
        radius = self.radius

        # The shortest Dubins word from x0 to x1
        path = shortest_path(x0, x1, radius)
        if path is None:
            return None
        _, segments, _ = path

        # Its arcs walked exactly, up to max_distance
        states, signs, steps = self.sample_arcs(
            np.asarray(x0, dtype=float), segments, max_distance, resolution
        )
        if len(steps) < 1:
            return None

        # u = (v, σ δ_max) on each step, σ ∈ {+1, 0, −1}: full left, straight, full right
        inputs = np.array([[v, sign * delta_max] for sign in signs])

        # t = s / v at constant speed, and the cost is the arc length s
        times = np.concatenate([[0.0], np.cumsum(steps) / v])
        cost = float(np.sum(steps))

        states = np.asarray(states)

        return states, inputs, times, cost

    def distance(self, a, b) -> float:
        """The length of the shortest Dubins path from ``a`` to ``b``, ``inf`` when there is none."""
        path = shortest_path(a, b, self.radius)
        return float("inf") if path is None else float(path[2])

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def sample_arcs(self, q0, segments, max_distance, resolution):
        """Walk the Dubins segments, sampling exact bicycle arcs at ``resolution``."""
        radius = self.radius

        x = float(q0[0])
        y = float(q0[1])
        theta = float(q0[2])

        states = [np.array([x, y, theta])]
        signs: list[float] = []
        steps: list[float] = []
        remaining = float(max_distance)

        for sign, arc in segments:
            # The segment, cut at the length that remains; an empty one adds nothing
            arc = min(arc, remaining)
            if arc <= 1e-12:
                continue

            # κ = σ / r, walked in equal steps ds no longer than the resolution
            curvature = sign / radius
            n_steps = max(1, int(np.ceil(arc / float(resolution))))
            ds = arc / n_steps

            for _ in range(n_steps):
                # One exact step of length ds along the arc of curvature κ (a straight
                # step when κ = 0)
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
