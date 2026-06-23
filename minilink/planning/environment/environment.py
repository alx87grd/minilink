"""
Obstacle environments and their state-field exports.

An :class:`Environment` is the single source of truth for a scene: hard obstacle
:class:`~minilink.core.geometry.Shape` solids and soft traversability
:class:`~minilink.planning.environment.features.ScalarField` sources. It answers
workspace-point queries — :meth:`clearance` and :meth:`cost_density` — and lifts
them to the robot through :meth:`clearance_field` and :meth:`cost_field`.

Export obstacles and soft fields **separately**; compose at
:class:`~minilink.planning.problems.PlanningProblem`:

``X = bounds & env.clearance_field(robot).as_constraint()``

``cost = base + w * env.cost_field(robot).as_cost()``
"""

from dataclasses import dataclass

from minilink.core.backends import array_module
from minilink.core.geometry import Union
from minilink.planning.environment.fields import StateField
from minilink.planning.environment.robot import (
    RobotBody,
    apply_transform,
    collision_spheres,
)


# Public API


@dataclass(frozen=True)
class Environment:
    """
    A scene of hard obstacles and soft traversability fields.

    Parameters
    ----------
    obstacles : tuple of Shape
        Hard obstacle solids. Inflate individual shapes with
        :meth:`~minilink.core.geometry.Shape.inflate` for safety margins.
    fields : tuple of ScalarField
        Soft traversability penalty sources summed by :meth:`cost_density`.
    workspace_dim : int
        Spatial dimension of the scene (2 or 3).
    """

    obstacles: tuple = ()
    fields: tuple = ()
    workspace_dim: int = 2

    def __post_init__(self) -> None:
        obstacles = tuple(self.obstacles)
        fields = tuple(self.fields)
        object.__setattr__(self, "obstacles", obstacles)
        object.__setattr__(self, "fields", fields)
        union = Union(obstacles) if obstacles else None
        object.__setattr__(self, "_obstacle_field", union)

    def clearance(self, p, t=0.0, params=None):
        """Return the signed distance from ``p`` to the nearest obstacle."""
        union = self._obstacle_field
        if union is None:
            raise ValueError("Environment has no obstacles; clearance is undefined")
        return union.sdf(p, t=t, params=params)

    def cost_density(self, p, t=0.0, params=None):
        """Return the summed soft penalty density at workspace point ``p``."""
        xp = array_module(p)
        if not self.fields:
            return xp.asarray(0.0)

        density = self.fields[0].density(p, t=t, params=params)
        for field in self.fields[1:]:
            density = density + field.density(p, t=t, params=params)
        return density

    def clearance_field(self, robot: RobotBody) -> StateField:
        """Return the robot's obstacle-clearance field (free-space export)."""
        if self._obstacle_field is None:
            raise ValueError("Environment has no obstacles to build a clearance field")
        return ClearanceField(self, robot)

    def cost_field(self, robot: RobotBody) -> StateField:
        """Return the robot's traversability density field (soft-cost export)."""
        return CostDensityField(self, robot)


@dataclass(frozen=True)
class ClearanceField(StateField):
    """
    Per-part signed clearance of a robot body against the obstacles.

    ``value(x)`` holds one component per body collision sphere: the
    environment clearance at the placed sphere center minus its radius.
    Feasible (collision-free) states have every component nonnegative; a
    safety margin is an ``as_constraint(lower=...)`` bound or an obstacle
    ``inflate``.
    """

    env: Environment
    robot: RobotBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)

        clearances = []
        for shape, transform in zip(self.robot.shapes, self.robot.body_poses(x, u, t, params)):
            for center, radius in collision_spheres(shape):
                world = apply_transform(transform, center)
                clearances.append(self.env.clearance(world, t=t, params=params) - radius)
        return xp.stack(clearances)


@dataclass(frozen=True)
class CostDensityField(StateField):
    """
    Per-part traversability density of a robot body in the scene.

    ``value(x)`` holds one component per body collision sphere: the
    environment cost density at the placed sphere center. Feasible when
    ``as_constraint(upper=...)`` or penalized by ``as_cost()``.
    """

    env: Environment
    robot: RobotBody

    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)

        densities = []
        for shape, transform in zip(self.robot.shapes, self.robot.body_poses(x, u, t, params)):
            for center, _ in collision_spheres(shape):
                world = apply_transform(transform, center)
                densities.append(self.env.cost_density(world, t=t, params=params))
        return xp.stack(densities)
