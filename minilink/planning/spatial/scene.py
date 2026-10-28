"""Spatial scenes: hard obstacles and soft workspace fields on the workspace."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from minilink.core.backends import array_module
from minilink.core.geometry import Union
from minilink.planning.spatial.collision import CollisionBody

if TYPE_CHECKING:
    from minilink.core.fields import Field


###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class Scene:
    """
    Hard ``Shape`` obstacles and soft ``WorkspaceField`` sources on the workspace.

    A scene composes into a :class:`~minilink.planning.problems.PlanningProblem`
    through its state fields::

        X = bounds & scene.clearance_field(body).as_set()
        cost = base + w * scene.cost_field(body).as_cost()
    """

    obstacles: tuple = ()
    workspace_fields: tuple = ()

    def __post_init__(self) -> None:
        # The sources, stored as tuples on the frozen dataclass
        object.__setattr__(self, "obstacles", tuple(self.obstacles))
        object.__setattr__(self, "workspace_fields", tuple(self.workspace_fields))

    def clearance(self, p, t=0.0, params=None):
        """Signed distance from ``p`` to the nearest obstacle."""
        obstacles = self.obstacles
        require_obstacles_for_clearance(obstacles)

        # The signed distance to the union of the obstacles
        clearance = Union(obstacles).sdf(p, t=t, params=params)

        return clearance

    def cost_density(self, p, t=0.0, params=None):
        """Sum of workspace penalty densities at ``p``."""
        xp = array_module(p)

        fields = self.workspace_fields

        # density(p) = Σᵢ densityᵢ(p), summed in the fields' order; 0 with no field
        if not fields:
            density = xp.asarray(0.0)
        else:
            density = fields[0].density(p, t=t, params=params)
            for field in fields[1:]:
                density = density + field.density(p, t=t, params=params)

        return density

    def clearance_field(self, body: CollisionBody) -> Field:
        """The clearance of ``body``'s probes to the obstacles, as a field over the state."""
        require_obstacles_for_field(self.obstacles)

        # A real cycle: state_fields imports Scene from this module
        from minilink.planning.spatial.state_fields import ClearanceField

        return ClearanceField(self, body)

    def cost_field(self, body: CollisionBody) -> Field:
        """The largest cost density over ``body``'s probes, as a field over the state."""
        # A real cycle: state_fields imports Scene from this module
        from minilink.planning.spatial.state_fields import CostDensityField

        return CostDensityField(self, body)

    def plot(self, **kwargs):
        """Plot obstacles and workspace fields (lazy matplotlib import)."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.planning.spatial.plotting import plot_scene

        return plot_scene(self, **kwargs)

    def as_visualizer(self, **kwargs):
        """Return a time-only overlay that draws this scene's obstacle skin."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation.drawables import scene_as_visualizer

        return scene_as_visualizer(self, **kwargs)


###############################################################################
# Internal machinery
###############################################################################


def require_obstacles_for_clearance(obstacles):
    """Refuse a clearance query on a scene with no obstacles."""
    if not obstacles:
        raise ValueError("Scene has no obstacles; clearance is undefined")


def require_obstacles_for_field(obstacles):
    """Refuse a clearance field on a scene with no obstacles."""
    if not obstacles:
        raise ValueError("Scene has no obstacles to build a clearance field")
