"""
Mission graphics layered around a system's vehicle skin during animation.

An :class:`SceneOverlay` collects the *world-space* extras an MPC or planning
demo wants on screen — track corridor, obstacles, a reference line, the executed
trail, and the receding-horizon plan — without subclassing the plant. The
animator draws the overlay's static and time-varying primitives before and after
the vehicle, so a demo writes one ``SceneOverlay(...)`` instead of overriding
``get_kinematic_geometry`` and hand-counting transforms.

Static parts (corridor, obstacles, reference line) carry world coordinates and
an identity transform; time-varying parts (:class:`TrajectoryPolyline`,
:class:`HorizonPolyline`) rebuild themselves each frame from the playback time
(see :class:`~minilink.graphical.animation.primitives.DynamicPrimitive`).

Z-order around the vehicle::

    [track, obstacles, reference line, executed trail] + VEHICLE + [mpc horizon]
"""

import numpy as np

from minilink.core.geometry import Sphere
from minilink.graphical.animation.primitives import (
    Circle,
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)

# Public API


class SceneOverlay:
    """World-space scene layers merged around a base system's vehicle skin."""

    def __init__(
        self,
        *,
        track=None,
        scene=None,
        obstacles=None,
        ref_line=None,
        mpc_plans=None,
        executed=None,
        track_samples=200,
    ):
        self._before = []
        self._after = []

        # Static world geometry, drawn behind the vehicle.
        if track is not None:
            center, upper, lower = track.sample_boundaries(track_samples)
            self._before.append(CustomLine(_to3(upper), color="#98df8a", linewidth=1.2))
            self._before.append(CustomLine(_to3(lower), color="#98df8a", linewidth=1.2))
            self._before.append(
                CustomLine(_to3(center), color="#2ca02c", linewidth=2.0)
            )
        if scene is not None:
            self._before.extend(scene_obstacle_primitives(scene))
        if obstacles is not None:
            for center_xy, radius in obstacles:
                cx, cy = float(center_xy[0]), float(center_xy[1])
                self._before.append(
                    Circle(
                        radius=radius, center=(cx, cy, 0.0), color="tab:red", fill=True
                    )
                )
        if ref_line is not None:
            (x0, y0), (x1, y1) = ref_line
            self._before.append(
                CustomLine(
                    np.array([[x0, y0, 0.0], [x1, y1, 0.0]]),
                    color="k",
                    linewidth=1.0,
                    style="--",
                )
            )

        # Time-varying executed trail, still behind the vehicle.
        if executed is not None:
            self._before.append(
                TrajectoryPolyline(
                    executed, window="prefix", color="b", style="--", linewidth=1.0
                )
            )

        # Time-varying horizon plan, drawn on top of the vehicle.
        if mpc_plans is not None:
            self._after.append(
                HorizonPolyline(
                    mpc_plans, color="tab:orange", linewidth=2.0, style="--"
                )
            )

    def before_primitives(self):
        """Primitives drawn behind the vehicle."""
        return list(self._before)

    def after_primitives(self):
        """Primitives drawn in front of the vehicle."""
        return list(self._after)

    def before_transforms(self):
        """Identity transform per ``before`` primitive (all world-space)."""
        return [np.eye(4) for _ in self._before]

    def after_transforms(self):
        """Identity transform per ``after`` primitive (all world-space)."""
        return [np.eye(4) for _ in self._after]


def shape_to_primitive(shape, *, color="tab:red", fill=True):
    """Convert a 2-D planning :class:`~minilink.core.geometry.Shape` to a primitive."""
    if isinstance(shape, Sphere) and shape.dim == 2:
        cx, cy = float(shape.center[0]), float(shape.center[1])
        return Circle(radius=shape.radius, center=(cx, cy, 0.0), color=color, fill=fill)
    raise NotImplementedError(
        f"No graphical primitive adapter for {type(shape).__name__} (2-D only for now)"
    )


def scene_obstacle_primitives(scene, **kwargs):
    """Graphical primitives for every obstacle in a planning ``Scene``."""
    return [shape_to_primitive(shape, **kwargs) for shape in scene.obstacles]


# Internal helpers


def _to3(pts_xy):
    """Lift an ``(n, 2)`` world polyline to ``(n, 3)`` at ``z = 0``."""
    pts = np.asarray(pts_xy, dtype=float)
    return np.column_stack([pts[:, 0], pts[:, 1], np.zeros(pts.shape[0])])
