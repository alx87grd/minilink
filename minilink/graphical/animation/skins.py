"""
Reusable skin factories: static geometry keyed by standard frame names.

Frame vocabulary (vehicles): ``body``, ``axle_front``, ``axle_rear``,
``wheel_fl``, ``wheel_fr``, ``wheel_rl``, ``wheel_rr``.
"""

import numpy as np

from minilink.graphical.animation.primitives import (
    Box,
    CustomLine,
    Plane,
    Rod,
    wheel_box,
)


def merge_skins(*skins):
    """Merge skin dicts, concatenating primitive lists on shared keys."""
    merged = {}
    for skin in skins:
        for key, prims in skin.items():
            merged.setdefault(key, []).extend(prims)
    return merged


def _car_outline(length, width):
    return np.array(
        [
            [-0.5 * length, -0.5 * width, 0.0],
            [0.3 * length, -0.5 * width, 0.0],
            [0.5 * length, 0.0, 0.0],
            [0.3 * length, 0.5 * width, 0.0],
            [-0.5 * length, 0.5 * width, 0.0],
            [-0.5 * length, -0.5 * width, 0.0],
        ]
    )


def car_skin_2d(length, width, color="blue"):
    """Bicycle centerline skin: body + axle markers."""
    return {
        "body": [CustomLine(_car_outline(length, width), color=color, linewidth=2)],
        "axle_rear": [wheel_box()],
        "axle_front": [wheel_box()],
    }


def car_skin_3d(length, width, track, *, body_height=0.22, ground_size=120.0, color="#151922"):
    """Four-wheel 3D car skin."""
    bl = length
    bw = 0.72 * track
    bh = body_height
    ground = Plane(
        normal=[0.0, 0.0, 1.0],
        offset=0.0,
        size=ground_size,
        thickness=0.04,
        color=[0.72, 0.74, 0.78],
        opacity=0.5,
    )
    body = Box(
        length_x=bl,
        length_y=bw,
        length_z=bh,
        center=(0.0, 0.0, 0.0),
        color=color,
        opacity=1.0,
    )
    wheel = Rod(length=0.2, radius=0.08, color="#0a0a0a", opacity=1.0)
    return {
        "world": [ground],
        "body": [body],
        "wheel_rl": [wheel],
        "wheel_rr": [wheel],
        "wheel_fl": [wheel],
        "wheel_fr": [wheel],
    }
