"""Public graphical catalog: curated shapes, skins, and camera factories.

The one-stop import surface for demos and student plants, mirroring
``dynamics.catalog`` for plants:

    from minilink.graphical.catalog import Box, Circle, Arrow, car_skin_3d
    from minilink.graphical.catalog import follow_frame_camera, fixed_camera

Classes and functions live in the internal ``graphical/animation/`` band; this
package is the friendly re-export, resolved on first use.
"""

from __future__ import annotations

from minilink.core.facade import lazy_facade

_SHAPES = "minilink.graphical.catalog.shapes"
_SKINS = "minilink.graphical.catalog.skins"
_CAMERA = "minilink.graphical.animation.camera"
_DRAWABLES = "minilink.graphical.animation.drawables"

# name -> (module path, attribute)
_EXPORTS: dict[str, tuple[str, str]] = {
    # shapes
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
    # overlays
    "Replay": (_DRAWABLES, "Replay"),
    "SceneHistory": (_DRAWABLES, "SceneHistory"),
    # skins
    "car_skin_2d": (_SKINS, "car_skin_2d"),
    "car_skin_3d": (_SKINS, "car_skin_3d"),
    "debug_state_skin": (_SKINS, "debug_state_skin"),
    "merge_skins": (_SKINS, "merge_skins"),
    "plane_skin_3d": (_SKINS, "plane_skin_3d"),
    "ur5_skin": (_SKINS, "ur5_skin"),
    # camera
    "camera_matrix": (_CAMERA, "camera_matrix"),
    "fixed_camera": (_CAMERA, "fixed_camera"),
    "follow_frame_camera": (_CAMERA, "follow_frame_camera"),
    "world_to_camera": (_CAMERA, "world_to_camera"),
}

__all__, __getattr__, __dir__ = lazy_facade(
    globals(), _EXPORTS, modules={"shapes": _SHAPES, "skins": _SKINS}
)
