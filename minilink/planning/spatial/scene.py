"""
Spatial workspace scenes and state-field factories.

Compose at :class:`~minilink.planning.problems.PlanningProblem`:

``X = bounds & scene.clearance_field(robot).as_constraint()``

``cost = base + w * scene.cost_field(robot).as_cost()``
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from minilink.core.backends import array_module
from minilink.core.geometry import Union
from minilink.planning.spatial.robot import RobotBody

if TYPE_CHECKING:
    from minilink.planning.spatial.state_fields import StateField

# Public API


@dataclass(frozen=True)
class Scene:
    """
    Hard ``Shape`` obstacles and soft ``WorkspaceField`` sources on the workspace.
    """

    obstacles: tuple = ()
    workspace_fields: tuple = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "obstacles", tuple(self.obstacles))
        object.__setattr__(self, "workspace_fields", tuple(self.workspace_fields))

    def clearance(self, p, t=0.0, params=None):
        """Signed distance from ``p`` to the nearest obstacle."""
        if not self.obstacles:
            raise ValueError("Scene has no obstacles; clearance is undefined")
        return Union(self.obstacles).sdf(p, t=t, params=params)

    def cost_density(self, p, t=0.0, params=None):
        """Sum of workspace penalty densities at ``p``."""
        xp = array_module(p)
        fields = self.workspace_fields
        if not fields:
            return xp.asarray(0.0)

        density = fields[0].density(p, t=t, params=params)
        for field in fields[1:]:
            density = density + field.density(p, t=t, params=params)
        return density

    def clearance_field(self, robot: RobotBody) -> StateField:
        if not self.obstacles:
            raise ValueError("Scene has no obstacles to build a clearance field")
        from minilink.planning.spatial.state_fields import ClearanceField

        return ClearanceField(self, robot)

    def cost_field(self, robot: RobotBody) -> StateField:
        from minilink.planning.spatial.state_fields import CostDensityField

        return CostDensityField(self, robot)

    def plot(self, **kwargs):
        """Plot obstacles and workspace fields (lazy matplotlib import)."""
        from minilink.planning.spatial.plotting import plot_scene

        return plot_scene(self, **kwargs)

    def as_visualizer(self, **kwargs):
        """Return a time-only overlay that draws this scene's obstacle skin."""
        from minilink.graphical.animation.drawables import scene_as_visualizer

        return scene_as_visualizer(self, **kwargs)
