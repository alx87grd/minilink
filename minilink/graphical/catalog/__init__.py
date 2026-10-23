"""Graphical catalog: the shapes, skins, overlays and cameras systems are drawn with."""

from __future__ import annotations

from minilink.core.facade import lazy_facade

_SHAPES = "minilink.graphical.catalog.shapes"
_SKINS = "minilink.graphical.catalog.skins"
_CAMERA = "minilink.graphical.animation.camera"
_DRAWABLES = "minilink.graphical.animation.drawables"

# The one-stop import surface for demos and student plants, mirroring
# ``dynamics.catalog`` for plants, e.g.
#
#     from minilink.graphical.catalog import Box, Circle, Arrow, car_skin_3d
#     from minilink.graphical.catalog import camera_matrix
#
# The classes and functions live in the internal ``graphical/animation/`` band; this
# package is the friendly re-export, resolved on first use. Each exported name maps to
# its (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    # Shapes
    "Arrow": (_SHAPES, "Arrow"),
    "Box": (_SHAPES, "Box"),
    "Circle": (_SHAPES, "Circle"),
    "ExtrudedPolygon": (_SHAPES, "ExtrudedPolygon"),
    "HorizonPolyline": (_SHAPES, "HorizonPolyline"),
    "Line": (_SHAPES, "Line"),
    "Plane": (_SHAPES, "Plane"),
    "Point": (_SHAPES, "Point"),
    "Rod": (_SHAPES, "Rod"),
    "Sphere": (_SHAPES, "Sphere"),
    "TorqueArrow": (_SHAPES, "TorqueArrow"),
    "TrajectoryPolyline": (_SHAPES, "TrajectoryPolyline"),
    "ground_line": (_SHAPES, "ground_line"),
    "spring_line": (_SHAPES, "spring_line"),
    "spring_between": (_SHAPES, "spring_between"),
    "line_segment": (_SHAPES, "line_segment"),
    "segment_pose_2d": (_SHAPES, "segment_pose_2d"),
    "link_pose_3d": (_SHAPES, "link_pose_3d"),
    "plane_airframe_3d": (_SHAPES, "plane_airframe_3d"),
    "point_pose": (_SHAPES, "point_pose"),
    "vehicle_body": (_SHAPES, "vehicle_body"),
    "wheel_box": (_SHAPES, "wheel_box"),
    # Overlays
    "SceneHistory": (_DRAWABLES, "SceneHistory"),
    # Skins
    "car_skin_2d": (_SKINS, "car_skin_2d"),
    "car_skin_3d": (_SKINS, "car_skin_3d"),
    "plane_skin_3d": (_SKINS, "plane_skin_3d"),
    "ur5_skin": (_SKINS, "ur5_skin"),
    # Camera
    "camera_matrix": (_CAMERA, "camera_matrix"),
    "world_to_camera": (_CAMERA, "world_to_camera"),
}

__all__, __getattr__, __dir__ = lazy_facade(
    globals(), _EXPORTS, modules={"shapes": _SHAPES, "skins": _SKINS}
)
