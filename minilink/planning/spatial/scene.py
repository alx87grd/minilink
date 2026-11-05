"""Spatial scenes: hard obstacles and soft workspace fields on the workspace."""

from __future__ import annotations

from typing import TYPE_CHECKING

from minilink.core.backends import array_module
from minilink.core.geometry import Union
from minilink.core.params import held_params
from minilink.planning.spatial.collision import CollisionBody

if TYPE_CHECKING:
    from minilink.core.fields import Field


###############################################################################
# Public API
###############################################################################


class Scene:
    """
    Hard ``Shape`` obstacles and soft ``WorkspaceField`` sources on the workspace.

    A scene composes into a :class:`~minilink.planning.problems.PlanningProblem`
    through its state fields::

        X = bounds & scene.clearance_field(body).as_set()
        cost = base + w * scene.cost_field(body).as_cost()

    ``params`` holds ``{"obstacles": each obstacle's own numbers by position,
    "workspace_fields": each field's}``: ``params=None`` reads them.
    """

    params = held_params(
        '``{"obstacles", "workspace_fields"}``: each obstacle\'s and each field\'s numbers by position.'
    )

    def __init__(self, obstacles=(), workspace_fields=()):
        self.obstacles = tuple(obstacles)
        self.workspace_fields = tuple(workspace_fields)
        self.params = {
            "obstacles": {
                str(i): shape.params for i, shape in enumerate(self.obstacles)
            },
            "workspace_fields": {
                str(i): field.params for i, field in enumerate(self.workspace_fields)
            },
        }

    def param_parts(self) -> dict:
        """The obstacles and the fields by position: the parts an assignment routes to."""
        return {
            "obstacles": {str(i): shape for i, shape in enumerate(self.obstacles)},
            "workspace_fields": {
                str(i): field for i, field in enumerate(self.workspace_fields)
            },
        }

    def clearance(self, p, t=0.0, params=None):
        """Signed distance from ``p`` to the nearest obstacle."""
        params = self.params if params is None else params

        obstacles = self.obstacles
        obstacle_params = params["obstacles"]
        require_obstacles_for_clearance(obstacles)

        # The signed distance to the union of the obstacles
        clearance = Union(obstacles).sdf(p, t=t, params=obstacle_params)

        return clearance

    def cost_density(self, p, t=0.0, params=None):
        """Sum of workspace penalty densities at ``p``."""
        params = self.params if params is None else params
        xp = array_module(p)

        fields = self.workspace_fields
        field_params = params["workspace_fields"]

        # density(p) = Σᵢ densityᵢ(p), summed in the fields' order; 0 with no field
        if not fields:
            density = xp.asarray(0.0)
        else:
            density = fields[0].density(p, t=t, params=field_params["0"])
            for i, field in enumerate(fields[1:], start=1):
                density = density + field.density(p, t=t, params=field_params[str(i)])

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
