import numpy as np

from minilink.graphical.animation.primitives import (Arrow, Box, Circle,
                                                     CustomLine, Point, Rod,
                                                     Sphere, TorqueArrow,
                                                     camera_matrix,
                                                     pose2d_matrix,
                                                     scale_pose2d_matrix,
                                                     torque_pose2d_matrix,
                                                     translation_matrix)


def identity_matrix():
    return np.eye(4)


def empty_transform():
    return translation_matrix(0.0, 0.0, -1000.0)


def follow_xy_camera(x, y, scale):
    return camera_matrix(target=(x, y, 0.0), plot_axes=(0, 1), scale=scale)


def heading_from_vector(vx, vy):
    return np.arctan2(vy, vx)


def arrow_transform(x, y, vx, vy, scale=1.0):
    length = scale * np.hypot(vx, vy)
    if length < 1e-12:
        return scale_pose2d_matrix(x, y, 0.0, 0.0)
    return scale_pose2d_matrix(x, y, heading_from_vector(vx, vy), length)


def line_between_transform(p0, p1):
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    delta = p1 - p0
    return scale_pose2d_matrix(
        p0[0],
        p0[1],
        heading_from_vector(delta[0], delta[1]),
        np.hypot(delta[0], delta[1]),
    )


def rod_between_transform(p0, p1):
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    delta = p1 - p0
    length = np.linalg.norm(delta)
    T = np.eye(4)
    T[:3, 3] = p0
    if length < 1e-12:
        return T

    y_axis = -delta / length
    reference = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(y_axis, reference)) > 0.95:
        reference = np.array([1.0, 0.0, 0.0])
    x_axis = np.cross(reference, y_axis)
    x_axis = x_axis / np.linalg.norm(x_axis)
    z_axis = np.cross(x_axis, y_axis)
    T[:3, 0] = x_axis
    T[:3, 1] = y_axis
    T[:3, 2] = z_axis
    return T


def point_transform(point):
    point = np.asarray(point, dtype=float)
    return translation_matrix(point[0], point[1], point[2] if point.size > 2 else 0.0)


def unit_line(color="black", linewidth=2, style="-"):
    return CustomLine(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        color=color,
        linewidth=linewidth,
        style=style,
    )


def rectangle_outline(length=1.0, width=0.5, color="blue", linewidth=2, style="-"):
    half_l = 0.5 * length
    half_w = 0.5 * width
    return CustomLine(
        [
            [-half_l, -half_w, 0.0],
            [half_l, -half_w, 0.0],
            [half_l, half_w, 0.0],
            [-half_l, half_w, 0.0],
            [-half_l, -half_w, 0.0],
        ],
        color=color,
        linewidth=linewidth,
        style=style,
    )


def ground_line(length=20.0, y=0.0, color="black", style="--"):
    return CustomLine(
        [[-0.5 * length, y, 0.0], [0.5 * length, y, 0.0]],
        color=color,
        linewidth=1,
        style=style,
    )


def mass_box(size=0.5, color="blue", opacity=0.9):
    return Box(length_x=size, length_y=size, length_z=0.2 * size, color=color, opacity=opacity)


def spring_line(coils=6, amplitude=0.12, color="black", linewidth=1):
    pts = [[0.0, 0.0, 0.0], [0.15, 0.0, 0.0]]
    xs = np.linspace(0.2, 0.8, 2 * coils + 1)
    for i, x in enumerate(xs):
        y = amplitude if i % 2 else -amplitude
        pts.append([x, y, 0.0])
    pts.append([0.85, 0.0, 0.0])
    pts.append([1.0, 0.0, 0.0])
    return CustomLine(pts, color=color, linewidth=linewidth)


def wheel_box(length=0.45, width=0.16):
    return Box(length_x=length, length_y=width, length_z=0.08, color="black", opacity=0.9)


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
    return Box(length_x=width, length_y=height, length_z=0.08, color="black", opacity=0.9)


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
    "mass_box",
    "point_transform",
    "pose2d_matrix",
    "rectangle_outline",
    "rocket_body",
    "rod_between_transform",
    "scale_pose2d_matrix",
    "spring_line",
    "torque_pose2d_matrix",
    "translation_matrix",
    "unit_line",
    "vehicle_body",
    "wheel_box",
]
