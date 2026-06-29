"""Skin factory functions: ``(plant) -> dict[str, list[primitive]]``.

A *skin* is the swappable visual look of a plant — pure point geometry keyed to
the plant's ``tf`` frame vocabulary, with **no** state math (placement comes from
``tf``). Plants opt in via ``skin = car_skin_2d`` and swap a look with one
assignment (``car.skin = car_skin_3d``); same ``f``, same ``tf``. These are plain
functions (no dataclasses) so a custom skin is just another ``(plant) -> dict``.

Authored against the target frame vocabulary (vehicles: ``body`` and
``axle_front`` for the 2-D centerline; ``body`` plus steered ``wheel_fl`` /
``wheel_fr`` for the 3-D four-wheel look). They read plant geometry from
``plant.params`` and visual attributes with sensible fallbacks, so a bare plant
still skins. Not wired into the render pipeline (or pixel-validated) until the
Phase 3 catalog migration.
"""

import numpy as np

from minilink.core.kinematics import translation
from minilink.graphical.catalog.shapes import Box, Line, Plane, Point, Rod


def _wheel_rectangle(wl, ww, color="black", linewidth=1):
    """Closed wheel-outline polyline (x forward, y lateral) as a :class:`Line`."""
    h, w = 0.5 * wl, 0.5 * ww
    pts = np.array(
        [
            [h, w, 0.0],
            [h, -w, 0.0],
            [-h, -w, 0.0],
            [-h, w, 0.0],
            [h, w, 0.0],
        ]
    )
    return Line(pts, color=color, linewidth=linewidth)


def car_skin_2d(plant, color="blue"):
    """2-D centerline car look: chassis line + two axle wheels.

    Frame keys: ``body`` (chassis along the wheelbase plus the rear wheel outline;
    the rear axle offset is baked into the wheel ``local_transform``), and
    ``axle_front`` (steered front wheel). The placing ``tf`` supplies world pose
    and the front steer angle.
    """
    a = plant.params["a"]
    b = plant.params["b"]
    wl = getattr(plant, "wheel_len", 0.6)
    ww = getattr(plant, "wheel_width", 0.2)

    chassis = Line(np.array([[-b, 0.0, 0.0], [a, 0.0, 0.0]]), color=color, linewidth=2)
    rear_wheel = _wheel_rectangle(wl, ww)
    rear_wheel.local_transform = translation(-b, 0.0, 0.0)
    return {
        "body": [chassis, rear_wheel],
        "axle_front": [_wheel_rectangle(wl, ww)],
    }


def car_skin_3d(plant, color="#151922"):
    """3-D four-wheel car look: ground plane, body box, four wheel rods.

    Frame keys: ``world`` (ground), ``body`` (chassis box plus the two rear wheel
    rods; fixed hub offsets are baked into each rod ``local_transform``), and
    ``wheel_fl`` / ``wheel_fr`` (steered front rods placed by ``tf``).
    """
    a = plant.params["a"]
    b = plant.params["b"]
    r_f = plant.params["r_f"]
    r_r = plant.params["r_r"]

    track = getattr(plant, "track", 1.6)
    body_height = getattr(plant, "body_height", 0.22)
    body_width_ratio = getattr(plant, "body_width_ratio", 0.72)
    body_length_overhang = getattr(plant, "body_length_overhang", 0.26)
    body_ground_clearance = getattr(plant, "body_ground_clearance", 0.003)
    ground_size = getattr(plant, "ground_plane_size", 120.0)
    tire_radius_ratio = getattr(plant, "_visual_tire_radius_ratio", 0.58)
    wheel_width = getattr(plant, "_visual_wheel_width", 0.2)

    body_width = body_width_ratio * track
    body_length = (a + b) + body_length_overhang
    tire_radius = max(0.045, tire_radius_ratio * min(r_f, r_r))

    ground = Plane(
        normal=[0.0, 0.0, 1.0],
        offset=0.0,
        size=ground_size,
        thickness=0.04,
        color=[0.72, 0.74, 0.78],
        opacity=0.5,
    )
    body = Box(
        length_x=body_length,
        length_y=body_width,
        length_z=body_height,
        center=(0.0, 0.0, 0.0),
        color=color,
        opacity=1.0,
    )
    # Fixed offset within the body frame: lift to ride height and shift to the
    # geometric center of the wheelbase (CG sits at the origin of ``body``).
    cx_body = 0.5 * (a - b)
    z_body = r_r + body_ground_clearance + 0.5 * body_height
    body.local_transform = translation(cx_body, 0.0, z_body)

    def wheel():
        return Rod(length=wheel_width, radius=tire_radius, color="#0a0a0a", opacity=1.0)

    wheel_rl = wheel()
    wheel_rr = wheel()
    wheel_rl.local_transform = translation(-b, 0.5 * track, r_r)
    wheel_rr.local_transform = translation(-b, -0.5 * track, r_r)

    return {
        "world": [ground],
        "body": [body, wheel_rl, wheel_rr],
        "wheel_fl": [wheel()],
        "wheel_fr": [wheel()],
    }


def merge_skins(*skins):
    """Compose skin factories into one ``(plant) -> dict`` that merges their output.

    Primitive lists are concatenated per frame key, so several looks stack on a
    single plant (e.g. a base body plus a decal layer).
    """

    def merged(plant):
        out: dict[str, list] = {}
        for skin in skins:
            for key, primitives in skin(plant).items():
                out.setdefault(key, []).extend(primitives)
        return out

    return merged


def debug_state_skin(plant):
    """Opt-in schematic dashboard: one marker per state and input, in a column.

    A quick look for non-spatial plants (chemical reactors, filters, …) that have
    no natural geometry. Markers are keyed to ``world`` and offset vertically via
    ``local_transform`` (blue dots for states, red crosses for inputs).
    """

    def column_offset(row):
        T = np.eye(4)
        T[1, 3] = float(row)
        return T

    markers = []
    for i in range(plant.n):
        marker = Point(color="blue", marker="o")
        marker.local_transform = column_offset(i)
        markers.append(marker)
    for i in range(getattr(plant, "m", 0)):
        marker = Point(color="red", marker="x")
        marker.local_transform = column_offset(-i - 1)
        markers.append(marker)
    return {"world": markers}
