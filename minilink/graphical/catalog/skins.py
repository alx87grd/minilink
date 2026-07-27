"""Skin factory functions: ``(plant) -> dict[str, list[primitive]]``.

A *skin* is the swappable visual look of a plant — pure point geometry keyed to
the plant's ``tf`` frame vocabulary, with **no** state math (placement comes from
``tf``). Plants opt in via ``skin = car_skin_2d`` and swap a look with one
assignment (``car.skin = car_skin_3d``); same ``f``, same ``tf``. These are plain
functions (no dataclasses) so a custom skin is just another ``(plant) -> dict``.

Authored against the target frame vocabulary (vehicles: ``body`` and
``axle_front`` for the 2-D centerline; ``body`` plus steered ``wheel_fl`` /
``wheel_fr`` for the 3-D four-wheel look). They read axle offsets from
``plant.a`` / ``plant.b`` when present, else ``plant.params``, so a bare plant
still skins.
"""

import numpy as np

from minilink.core.kinematics import translation
from minilink.graphical.catalog.shapes import (
    Box,
    Line,
    Plane,
    Point,
    Rod,
    Sphere,
    link_pose_3d,
    plane_airframe_3d,
)


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


def _axle_offsets(plant):
    """Front/rear CG→axle distances for skins (attrs preferred over params)."""
    if hasattr(plant, "a") and hasattr(plant, "b"):
        return float(plant.a), float(plant.b)
    params = getattr(plant, "params", {}) or {}
    if "a" in params and "b" in params:
        return float(params["a"]), float(params["b"])
    length = float(params.get("length", 2.0))
    return 0.5 * length, 0.5 * length


def car_skin_2d(plant, color="#1a1a1a"):
    """2-D centerline car look: chassis line + two axle wheels.

    Frame keys: ``body`` (chassis along the wheelbase plus the rear wheel outline;
    the rear axle offset is baked into the wheel ``local_transform``), and
    ``axle_front`` (steered front wheel). The placing ``tf`` supplies world pose
    and the front steer angle.
    """
    a, b = _axle_offsets(plant)
    wl = getattr(plant, "wheel_len", 0.6)
    ww = getattr(plant, "wheel_width", 0.2)

    chassis = Line(
        np.array([[-b, 0.0, 0.0], [a, 0.0, 0.0]]), color=color, linewidth=2.5
    )
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
    a, b = _axle_offsets(plant)
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


def plane_skin_3d(plant, color="#2f6fed"):
    """3-D aircraft look: ground plane plus solid fuselage / wing / tails.

    Frame keys: ``world`` (ground) and ``body`` (airframe solids with the c.g.
    at the body-frame origin). Reads length / span / lever arms from plant
    graphic attrs and ``params`` when present.
    """
    params = getattr(plant, "params", {}) or {}
    length = float(getattr(plant, "length", params.get("length", 2.0)))
    width = float(getattr(plant, "width", length / 10.0))
    l_cg = float(getattr(plant, "l_cg", 0.6 * length))
    span = float(getattr(plant, "span", params.get("b_w", max(1.5 * length, 1.0))))
    l_w = float(params.get("l_w", 0.0))
    l_t = float(params.get("l_t", 0.45 * length))
    s_w = float(params.get("S_w", 0.2))
    s_t = float(params.get("S_t", 0.05))
    ar = float(params.get("AR", 5.0))
    chord_w = float(np.sqrt(max(s_w / max(ar, 1e-6), 1e-6)))
    chord_t = float(np.sqrt(max(s_t / max(ar, 1e-6), 1e-6)))
    # Visual chord a bit larger than aero chord so the wing reads clearly.
    chord_w = max(chord_w, 0.12 * length)
    chord_t = max(chord_t, 0.08 * length)
    ground_size = float(getattr(plant, "ground_plane_size", 200.0))

    ground = Plane(
        normal=[0.0, 0.0, 1.0],
        offset=0.0,
        size=ground_size,
        thickness=0.04,
        color=[0.72, 0.74, 0.78],
        opacity=0.5,
    )
    airframe = plane_airframe_3d(
        length=length,
        width=width,
        l_cg=l_cg,
        span=span,
        chord_w=chord_w,
        chord_t=chord_t,
        span_t=0.35 * span,
        fin_height=0.22 * length,
        l_w=l_w,
        l_t=l_t,
        color=color,
    )
    center = Sphere(radius=0.04 * length, color="black", opacity=1.0)
    return {
        "world": [ground],
        "body": airframe + [center],
    }


def ur5_skin(plant):
    """Schematic UR5 look made from link and joint cylinders.

    Frame keys are ``base``, ``link0`` … ``link5``, ``joint0`` …
    ``joint6``, and ``tool``. The plant's ``tf`` places every moving part.
    """
    a = np.asarray(plant.params["a"], dtype=float)
    d = np.asarray(plant.params["d"], dtype=float)
    lengths = np.hypot(a, d)

    base = Rod(length=0.12, radius=0.09, color="#a6a8ab")
    base.local_transform = link_pose_3d([0.0, 0.0, -0.12], [0.0, 0.0, 0.0])

    geometry = {"base": [base]}
    for i, length in enumerate(lengths):
        radius = 0.055 if i < 3 else 0.04
        geometry[f"link{i}"] = [
            Rod(length=length, radius=radius, color="#c7c9cb", linewidth=3)
        ]

    for i in range(7):
        radius = 0.075 if i < 4 else 0.055
        housing = Rod(length=2.0 * radius, radius=0.8 * radius, color="#2b73b6")
        housing.local_transform = link_pose_3d([0.0, 0.0, radius], [0.0, 0.0, -radius])
        geometry[f"joint{i}"] = [
            housing,
            Sphere(radius=0.72 * radius, color="#d5d7d8", opacity=1.0),
        ]

    geometry["tool"] = [Sphere(radius=0.035, color="#3f4245", opacity=1.0)]
    return geometry


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
