"""
State-domain fields: ``value(x)`` exported as a set or a cost.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from minilink.core.backends import array_module
from minilink.core.costs import CostFunction
from minilink.core.kinematics import apply
from minilink.core.sets import Set
from minilink.planning.spatial.robot import (
    RobotBody,
    collision_spheres,
)
from minilink.planning.spatial.scene import Scene

if TYPE_CHECKING:
    from minilink.planning.spatial.track import ReferenceTrack

# Public API


class StateField(ABC):
    """Scalar ``value(x)``; export with :meth:`as_constraint` or :meth:`as_cost`."""

    @abstractmethod
    def value(self, x, u=None, t=0.0, params=None): ...

    def as_constraint(
        self, *, lower: float | None = 0.0, upper: float | None = None
    ) -> Set:
        return FieldSet(self, lower, upper)

    def as_cost(
        self,
        *,
        weight: float = 1.0,
        shaping: Callable | None = None,
    ) -> CostFunction:
        return FieldCost(self, float(weight), shaping)


@dataclass(frozen=True)
class ClearanceField(StateField):
    """``value(x) = min_p ( clearance(world_p) - r_p )``; nonnegative when free."""

    scene: Scene
    robot: RobotBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)
        scene = self.scene
        robot = self.robot
        c = []
        for shape, T in zip(robot.shapes, robot.body_poses(x, u, t, params)):
            for center, radius in collision_spheres(shape):
                world = apply(T, center)
                c.append(scene.clearance(world, t=t, params=params) - radius)

        return xp.min(xp.stack(c))


@dataclass(frozen=True)
class CostDensityField(StateField):
    """``value(x) = max_p cost_density(world_p)`` over body probes."""

    scene: Scene
    robot: RobotBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)
        scene = self.scene
        robot = self.robot
        d = []
        for shape, T in zip(robot.shapes, robot.body_poses(x, u, t, params)):
            for center, _ in collision_spheres(shape):
                world = apply(T, center)
                d.append(scene.cost_density(world, t=t, params=params))

        return xp.max(xp.stack(d))


@dataclass(frozen=True)
class PathDistanceField(StateField):
    """``value(x) = min_p distance(world_p)`` to the reference centerline."""

    track: ReferenceTrack
    robot: RobotBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)
        track = self.track
        robot = self.robot
        d = []
        for shape, T in zip(robot.shapes, robot.body_poses(x, u, t, params)):
            for center, radius in collision_spheres(shape):
                world = apply(T, center)
                d.append(track.distance(world, t=t, params=params) - radius)

        return xp.min(xp.stack(d))


@dataclass(frozen=True)
class CorridorMarginField(StateField):
    """``value(x) = min_p (half_width - distance(world_p))``; nonnegative in the tube."""

    track: ReferenceTrack
    robot: RobotBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)
        track = self.track
        robot = self.robot
        m = []
        for shape, T in zip(robot.shapes, robot.body_poses(x, u, t, params)):
            for center, radius in collision_spheres(shape):
                world = apply(T, center)
                m.append(
                    track.half_width
                    - track.distance(world, t=t, params=params)
                    - radius
                )

        return xp.min(xp.stack(m))


@dataclass(frozen=True)
class FieldSet(Set):
    """Feasible where ``lower <= value <= upper``."""

    field: StateField
    lower: float | None = 0.0
    upper: float | None = None

    def __post_init__(self) -> None:
        if self.lower is None and self.upper is None:
            raise ValueError("FieldSet requires at least one of lower or upper")

    def margin(self, z, t=0.0, params=None):
        xp = array_module(z)
        field = self.field
        lower = self.lower
        upper = self.upper
        v = field.value(z, None, t=t, params=params)

        bounds = []
        if lower is not None:
            bounds.append(v - lower)
        if upper is not None:
            bounds.append(upper - v)
        return xp.stack(bounds)


@dataclass(frozen=True)
class FieldCost(CostFunction):
    """``g = weight * shaping(value)``; ``shaping=None`` uses ``value`` directly."""

    field: StateField
    weight: float = 1.0
    shaping: Callable | None = None

    def g(self, x, u, t=0.0, params=None):
        field = self.field
        shaping = self.shaping
        weight = self.weight
        v = field.value(x, u, t=t, params=params)
        shaped = v if shaping is None else shaping(v)

        return weight * shaped

    def h(self, x, t=0.0, params=None):
        return 0.0
