import numpy as np

from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    CustomLine,
    Point,
    Rod,
    Sphere,
    TorqueArrow,
    arrow_transform,
    camera_matrix,
    empty_transform,
    follow_xy_camera,
    ground_line,
    identity_matrix,
    line_between_transform,
    point_transform,
    pose2d_matrix,
    rod_between_transform,
    scale_pose2d_matrix,
    spring_line,
    torque_pose2d_matrix,
    translation_matrix,
)


def wheel_box(length=0.45, width=0.16):
    return Box(
        length_x=length, length_y=width, length_z=0.08, color="black", opacity=0.9
    )


def vehicle_body(length=1.0, width=0.5, color="blue", opacity=0.85):
    pts = np.array(
        [
            [-0.5 * length, -0.5 * width, 0.0],
            [0.3 * length, -0.5 * width, 0.0],
            [0.5 * length, 0.0, 0.0],
            [0.3 * length, 0.5 * width, 0.0],
            [-0.5 * length, 0.5 * width, 0.0],
            [-0.5 * length, -0.5 * width, 0.0],
        ]
    )
    return CustomLine(pts, color=color, linewidth=2)


def boat_body(length=2.0, width=0.8):
    pts = np.array(
        [
            [-0.5 * length, -0.5 * width, 0.0],
            [0.35 * length, -0.5 * width, 0.0],
            [0.5 * length, 0.0, 0.0],
            [0.35 * length, 0.5 * width, 0.0],
            [-0.5 * length, 0.5 * width, 0.0],
            [-0.5 * length, -0.5 * width, 0.0],
        ]
    )
    return CustomLine(pts, color="blue", linewidth=2)


def drone_body(width=1.0, height=0.2):
    return Box(
        length_x=width, length_y=height, length_z=0.08, color="black", opacity=0.9
    )


def rocket_body(width=0.4, height=2.0):
    pts = np.array(
        [
            [-0.5 * width, -0.5 * height, 0.0],
            [-0.5 * width, 0.35 * height, 0.0],
            [0.0, 0.5 * height, 0.0],
            [0.5 * width, 0.35 * height, 0.0],
            [0.5 * width, -0.5 * height, 0.0],
            [-0.5 * width, -0.5 * height, 0.0],
        ]
    )
    return CustomLine(pts, color="blue", linewidth=2)


def manipulator_links(lengths, color="blue"):
    return [
        Rod(length=float(length), radius=0.03 * float(length), color=color, linewidth=2)
        for length in lengths
    ]


__all__ = [
    "Arrow",
    "Box",
    "Circle",
    "CustomLine",
    "Point",
    "Rod",
    "Sphere",
    "TorqueArrow",
    "arrow_transform",
    "boat_body",
    "camera_matrix",
    "drone_body",
    "empty_transform",
    "follow_xy_camera",
    "ground_line",
    "identity_matrix",
    "line_between_transform",
    "manipulator_links",
    "point_transform",
    "pose2d_matrix",
    "rocket_body",
    "rod_between_transform",
    "scale_pose2d_matrix",
    "spring_line",
    "torque_pose2d_matrix",
    "translation_matrix",
    "vehicle_body",
    "wheel_box",
]
