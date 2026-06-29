"""Public graphical catalog: curated shapes, skins, and camera factories.

The one-stop import surface for demos and student plants, mirroring
``dynamics.catalog`` for plants:

    from minilink.graphical.catalog import Box, Circle, Arrow, car_skin_3d
    from minilink.graphical.catalog import follow_frame_camera, fixed_camera

Classes and functions live in the internal ``graphical/animation/`` band; this
package is the friendly re-export. The honest ``Arrow`` / ``TorqueArrow`` (v2)
are exposed here under those names — never the legacy column-norm-scaling ones.
"""

from minilink.graphical.animation.camera import (
    camera_matrix,
    fixed_camera,
    follow_frame_camera,
    world_to_camera,
)
from minilink.graphical.catalog.shapes import (
    Arrow,
    Box,
    Circle,
    ExtrudedPolygon,
    HorizonPolyline,
    Line,
    Plane,
    Point,
    Rod,
    Sphere,
    TorqueArrow,
    TrajectoryPolyline,
    ground_line,
    spring_line,
    vehicle_body,
    wheel_box,
)
from minilink.graphical.catalog.skins import (
    car_skin_2d,
    car_skin_3d,
    debug_state_skin,
    merge_skins,
)

__all__ = [
    # shapes
    "Arrow",
    "Box",
    "Circle",
    "ExtrudedPolygon",
    "HorizonPolyline",
    "Line",
    "Plane",
    "Point",
    "Rod",
    "Sphere",
    "TorqueArrow",
    "TrajectoryPolyline",
    "ground_line",
    "spring_line",
    "vehicle_body",
    "wheel_box",
    # skins
    "car_skin_2d",
    "car_skin_3d",
    "debug_state_skin",
    "merge_skins",
    # camera
    "camera_matrix",
    "fixed_camera",
    "follow_frame_camera",
    "world_to_camera",
]
