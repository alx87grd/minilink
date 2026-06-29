"""Time-only overlay drawables for composition at ``animate(overlays=[...])``.

Overlays reuse the same three drawable hooks as :class:`~minilink.core.system.System`
but are driven by playback time *t* only (no ``x`` / ``u``). The animator resolves
each overlay with ``tf(t)``, cached ``get_kinematic_geometry()``, and
``get_dynamic_geometry(t)``, then concatenates the flat draw lists after the
primary plant.
"""

from __future__ import annotations

import numpy as np

from minilink.core.kinematics import translation
from minilink.graphical.animation.primitives import (
    Box,
    Circle,
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)
from minilink.graphical.animation.visualization import WORLD

__all__ = [
    "Overlay",
    "Replay",
    "SceneHistory",
    "SceneVisualizer",
    "scene_as_visualizer",
    "validate_overlay",
]


def validate_overlay(overlay) -> None:
    """Reject drawable mistakes at the animation boundary."""
    from minilink.core.system import System
    from minilink.planning.spatial.scene import Scene

    if isinstance(overlay, Scene):
        raise TypeError(
            "Pass scene.as_visualizer(), not a raw planning.Scene, into overlays=[...]."
        )
    if isinstance(overlay, System):
        raise TypeError(
            "Wrap a secondary System in Replay(drawable, trajectory); "
            "bare systems are not valid overlays."
        )
    for method in ("tf", "get_kinematic_geometry", "get_dynamic_geometry"):
        if not callable(getattr(overlay, method, None)):
            raise TypeError(
                f"overlay {type(overlay).__name__!r} is missing callable {method}()."
            )


class Overlay:
    """Base overlay: time-only drawable hooks with empty defaults."""

    def tf(self, t=0.0, params=None):
        return {}

    def get_kinematic_geometry(self):
        return {}

    def get_dynamic_geometry(self, t=0.0, params=None):
        return {}


class SceneHistory(Overlay):
    """Time-indexed plan/trail/reference polylines (graphics-only, not collision).

    Static layers (e.g. a fixed reference :class:`~minilink.graphical.animation.primitives.CustomLine`)
    live in kinematic geometry. Layers with :meth:`~HorizonPolyline.points_at` /
    :meth:`~TrajectoryPolyline.points_at` are rebuilt each frame in
    ``get_dynamic_geometry(t)``.
    """

    def __init__(self, **layers):
        self._kinematic = []
        self._dynamic_sources = []
        for primitive in layers.values():
            if primitive is None:
                continue
            if isinstance(primitive, (HorizonPolyline, TrajectoryPolyline)):
                self._dynamic_sources.append(primitive)
            else:
                self._kinematic.append(primitive)

    def tf(self, t=0.0, params=None):
        return {}

    def get_kinematic_geometry(self):
        if not self._kinematic:
            return {}
        return {WORLD: list(self._kinematic)}

    def get_dynamic_geometry(self, t=0.0, params=None):
        if not self._dynamic_sources:
            return {}
        lines = []
        for source in self._dynamic_sources:
            pts = source.points_at(t)
            lines.append(
                CustomLine(
                    pts,
                    color=source.color,
                    linewidth=source.linewidth,
                    style=source.style,
                )
            )
        return {WORLD: lines}


class Replay(Overlay):
    """Full-skin ghost: forward another drawable's hooks at ``trajectory`` samples."""

    def __init__(self, drawable, trajectory):
        self.drawable = drawable
        self.trajectory = trajectory
        self._kinematic = None

    def _sample(self, t):
        x = self.trajectory.t2x(t)
        u = self.trajectory.t2u(t)
        return x, u

    def tf(self, t=0.0, params=None):
        x, u = self._sample(t)
        return self.drawable.tf(x, u, t, params=params)

    def get_kinematic_geometry(self):
        if self._kinematic is None:
            self._kinematic = self.drawable.get_kinematic_geometry()
        return self._kinematic

    def get_dynamic_geometry(self, t=0.0, params=None):
        x, u = self._sample(t)
        return self.drawable.get_dynamic_geometry(x, u, t, params=params)


def _shape_to_primitives(shape, *, color, opacity):
    """Map a planning :class:`~minilink.core.geometry.Shape` to graphic primitives."""
    from minilink.core.geometry import Box as GeomBox
    from minilink.core.geometry import Inflated, Union
    from minilink.core.geometry import Sphere as GeomSphere

    if isinstance(shape, GeomSphere):
        center = np.asarray(shape.center, dtype=float)
        if center.size == 2:
            circle = Circle(radius=float(shape.radius), color=color, fill=True)
            circle.local_transform = translation(center[0], center[1], 0.0)
            return [circle]
        sphere = Circle(radius=float(shape.radius), color=color, fill=True)
        sphere.local_transform = translation(center[0], center[1], center[2])
        return [sphere]

    if isinstance(shape, GeomBox):
        lower = np.asarray(shape.lower, dtype=float)
        upper = np.asarray(shape.upper, dtype=float)
        center = 0.5 * (lower + upper)
        lengths = upper - lower
        box = Box(
            length_x=float(lengths[0]),
            length_y=float(lengths[1]),
            length_z=float(lengths[2]) if lengths.size > 2 else 0.1,
            color=color,
            opacity=opacity,
        )
        box.local_transform = translation(center[0], center[1], center[2] if center.size > 2 else 0.0)
        return [box]

    if isinstance(shape, Inflated):
        if isinstance(shape.base, GeomSphere):
            inflated = GeomSphere(
                center=shape.base.center,
                radius=float(shape.base.radius) + float(shape.radius),
            )
            return _shape_to_primitives(inflated, color=color, opacity=opacity)
        return _shape_to_primitives(shape.base, color=color, opacity=opacity)

    if isinstance(shape, Union):
        out = []
        for member in shape.shapes:
            out.extend(_shape_to_primitives(member, color=color, opacity=opacity))
        return out

    raise TypeError(f"unsupported obstacle shape for visualization: {type(shape).__name__}")


class SceneVisualizer(Overlay):
    """Static obstacle skin exported from a collision-first :class:`~minilink.planning.spatial.scene.Scene`."""

    def __init__(self, scene, *, color="#c44e52", opacity=0.45):
        primitives = []
        for obstacle in scene.obstacles:
            primitives.extend(
                _shape_to_primitives(obstacle, color=color, opacity=opacity)
            )
        self._geometry = {WORLD: primitives} if primitives else {}

    def tf(self, t=0.0, params=None):
        return {}

    def get_kinematic_geometry(self):
        return dict(self._geometry)


def scene_as_visualizer(scene, **kwargs):
    """Build a :class:`SceneVisualizer` overlay for *scene*."""
    return SceneVisualizer(scene, **kwargs)
