"""Two cartoon looks for the 3-D car: an open-wheel formula car and an off-road side-by-side."""

from __future__ import annotations

import numpy as np

from minilink.graphical.animation.primitives import Sphere
from minilink.graphical.catalog.skins import terrain_surface
from minilink.graphical.meshes import (
    TriangleMesh,
    axis_angle_matrix,
    box_mesh,
    cylinder_mesh,
    merge_meshes,
    ring_mesh,
    transform_mesh,
    translation_matrix,
)

FORMULA_COLORS = {
    "body": "#f2f3f5",
    "stripe": "#1b1d22",
    "accent": "#7d848f",
    "carbon": "#17181c",
    "cockpit": "#0b0b0d",
    "tire": "#141414",
    "rim": "#aeb3bb",
    "hub": "#1b1d22",
    "light": "#ef4444",
    "ground": "#9aa3ad",
}

OFFROAD_COLORS = {
    "body": "#e0a21b",
    "trim": "#2b2f36",
    "cage": "#15171b",
    "seat": "#3b4048",
    "light": "#fff3a3",
    "lamp": "#f8fafc",
    "tire": "#1c1c1c",
    "rim": "#3a3f47",
    "beadlock": "#d4d4d8",
    "hub": "#e0a21b",
    "ground": "#b08a5a",
}

###############################################################################
# Public API
###############################################################################


def formula_car_skin_3d(plant, colors=None):
    """An open-wheel formula car: a slim monocoque, an airbox, sidepods, two wings, slicks.

    A skin of the 3-D car: the body on its ``body`` frame (the centre of gravity at
    the origin, the ground ``h_cg`` below), each wheel on its own ``wheel_*`` frame
    at the hub, so the wheels steer, spin and ride the suspension. The proportions
    follow the plant's ``a``, ``b``, ``track``, ``h_cg`` and wheel radii. On a
    bicycle rung (no rear wheel frames, the body frame on the ground) the rear wheels
    ride on the body. A generic,
    cartoon racer: no real car's livery or shape.
    """
    colors = {**FORMULA_COLORS, **(colors or {})}
    a, b, track, h, r_f, r_r = chassis(plant)
    g = -h  # The ground, in the body frame

    # The monocoque: a slim nose rising to the cockpit, the airbox over the driver's
    # head, the engine cover falling to the gearbox
    # (x, half width, bottom, top), the heights above the ground
    tub = loft(
        [
            (a + 1.00, 0.05, 0.17, 0.24),
            (a + 0.60, 0.10, 0.15, 0.32),
            (a + 0.10, 0.16, 0.12, 0.42),
            (a - 0.55, 0.24, 0.08, 0.52),
            (0.60, 0.27, 0.06, 0.58),
            (0.02, 0.27, 0.06, 0.60),
            (-0.22, 0.20, 0.06, 0.93),
            (-0.75, 0.17, 0.08, 0.78),
            (-b + 0.30, 0.14, 0.10, 0.52),
            (-b - 0.22, 0.09, 0.17, 0.36),
        ],
        z0=g,
    )
    stripe = loft(
        [
            (a + 0.98, 0.022, 0.235, 0.245),
            (a + 0.60, 0.045, 0.315, 0.325),
            (a + 0.10, 0.065, 0.415, 0.425),
            (a - 0.55, 0.080, 0.515, 0.525),
            (0.66, 0.080, 0.570, 0.582),
        ],
        z0=g,
    )

    # The engine cover's fin, and the airbox inlet above the driver
    fin = loft(
        [
            (-0.30, 0.014, 0.60, 0.94),
            (-0.90, 0.014, 0.55, 0.80),
            (-b + 0.05, 0.014, 0.45, 0.62),
        ],
        z0=g,
    )
    inlet = box_mesh((0.04, 0.20, 0.13), (-0.10, 0.0, g + 0.83))

    # The sidepods, clear of the tub's flanks at the front and drawn in to the rear:
    # the bottle shape, their inlets dark
    pods = merge_meshes(
        *[
            loft(
                [
                    (a - 0.90, 0.17, 0.16, 0.50),
                    (0.50, 0.21, 0.10, 0.52),
                    (-0.30, 0.19, 0.09, 0.44),
                    (-0.85, 0.12, 0.09, 0.30),
                    (-b + 0.40, 0.05, 0.10, 0.20),
                ],
                z0=g,
                y0=side * 0.50,
            )
            for side in (1.0, -1.0)
        ]
    )
    pod_inlets = merge_meshes(
        *[
            box_mesh((0.03, 0.27, 0.24), (a - 0.905, side * 0.50, g + 0.33))
            for side in (1.0, -1.0)
        ]
    )

    # The floor: narrow under the nose, wide between the wheels, drawn in at the
    # rear tires, and the cockpit opening
    floor = loft(
        [
            (a - 0.25, 0.22, 0.045, 0.075),
            (a - 0.80, 0.74, 0.045, 0.075),
            (-b + 0.50, 0.76, 0.045, 0.075),
            (-b + 0.38, 0.42, 0.045, 0.075),
            (-b - 0.30, 0.42, 0.045, 0.12),
        ],
        z0=g,
    )
    opening = loft(
        [
            (0.62, 0.11, 0.572, 0.592),
            (0.35, 0.17, 0.585, 0.605),
            (0.04, 0.16, 0.592, 0.612),
        ],
        z0=g,
        power=2.5,
    )

    # The front wing: a main plane under the nose and two flaps each side of it,
    # each pitched more than the last, between painted endplates
    span = track + 0.30
    flap_span = 0.5 * span - 0.14
    flap_y = 0.14 + 0.5 * flap_span
    front_wing = merge_meshes(
        pitched(box_mesh((0.30, span, 0.02), (a + 0.92, 0.0, g + 0.085)), 0.05),
        *[
            pitched(
                box_mesh((chord, flap_span, 0.016), (x, side * flap_y, g + z)), tilt
            )
            for chord, x, z, tilt in (
                (0.20, a + 0.76, 0.15, 0.25),
                (0.15, a + 0.64, 0.22, 0.45),
            )
            for side in (1.0, -1.0)
        ],
        *[
            box_mesh((0.16, 0.02, 0.09), (a + 0.90, side * 0.05, g + 0.125))
            for side in (1.0, -1.0)
        ],
    )
    front_plates = merge_meshes(
        *[
            box_mesh((0.50, 0.018, 0.22), (a + 0.82, side * 0.5 * span, g + 0.16))
            for side in (1.0, -1.0)
        ]
    )

    # The rear wing over the rear axle: a main plane and a flap between painted
    # endplates, on a central pylon, and a beam wing under them
    rear_x = -b - 0.22
    rear_wing = merge_meshes(
        pitched(box_mesh((0.32, 0.98, 0.025), (rear_x, 0.0, g + 0.88)), 0.18),
        pitched(box_mesh((0.18, 0.98, 0.02), (rear_x - 0.17, 0.0, g + 0.96)), 0.55),
        box_mesh((0.22, 0.90, 0.02), (rear_x + 0.02, 0.0, g + 0.50)),
        box_mesh((0.09, 0.035, 0.52), (rear_x + 0.08, 0.0, g + 0.61)),
    )
    rear_plates = merge_meshes(
        *[
            box_mesh((0.46, 0.02, 0.40), (rear_x - 0.05, side * 0.50, g + 0.82))
            for side in (1.0, -1.0)
        ]
    )
    rain_light = box_mesh((0.03, 0.07, 0.10), (-b - 0.235, 0.0, g + 0.27))

    # The wishbones, two a side at each hub at rest, a track rod, and the halo
    arms = merge_meshes(
        *[
            tube(
                (x_hub + dx, side * y_tub, g + z + 0.04),
                (x_hub, side * (0.5 * track - 0.17), g + z),
                0.014,
            )
            for x_hub, y_tub, r in ((a, 0.15, r_f), (-b, 0.13, r_r))
            for side in (1.0, -1.0)
            for z in (r - 0.11, r + 0.10)
            for dx in (0.30, -0.26)
        ],
        *[
            tube(
                (a + 0.20, side * 0.14, g + r_f + 0.02),
                (a + 0.10, side * (0.5 * track - 0.17), g + r_f),
                0.011,
            )
            for side in (1.0, -1.0)
        ],
    )
    halo = merge_meshes(
        tube((0.86, 0.0, g + 0.555), (0.70, 0.0, g + 0.78), 0.024),
        *[
            merge_meshes(
                tube((0.70, 0.0, g + 0.78), (0.30, side * 0.25, g + 0.80), 0.024),
                tube(
                    (0.30, side * 0.25, g + 0.80), (-0.06, side * 0.22, g + 0.70), 0.024
                ),
            )
            for side in (1.0, -1.0)
        ],
    )

    # The mirrors on their stalks, and the camera pod on the airbox
    details = merge_meshes(
        *[
            merge_meshes(
                box_mesh((0.05, 0.13, 0.065), (0.86, side * 0.40, g + 0.62)),
                tube(
                    (0.86, side * 0.24, g + 0.52), (0.86, side * 0.36, g + 0.61), 0.008
                ),
            )
            for side in (1.0, -1.0)
        ],
        box_mesh((0.14, 0.035, 0.035), (-0.20, 0.0, g + 0.955)),
    )

    body = [
        TriangleMesh(*tub, color=colors["body"], name="monocoque", max_edges=0),
        TriangleMesh(*stripe, color=colors["stripe"], name="stripe", max_edges=0),
        TriangleMesh(*fin, color=colors["accent"], name="fin", max_edges=0),
        TriangleMesh(*inlet, color=colors["cockpit"], name="airbox", max_edges=0),
        TriangleMesh(*pods, color=colors["body"], name="sidepods", max_edges=0),
        TriangleMesh(*pod_inlets, color=colors["cockpit"], name="inlets", max_edges=0),
        TriangleMesh(*floor, color=colors["carbon"], name="floor", max_edges=0),
        TriangleMesh(*opening, color=colors["cockpit"], name="cockpit", max_edges=0),
        TriangleMesh(*front_wing, color=colors["carbon"], name="front_wing"),
        TriangleMesh(*front_plates, color=colors["accent"], name="front_plates"),
        TriangleMesh(*rear_wing, color=colors["carbon"], name="rear_wing"),
        TriangleMesh(*rear_plates, color=colors["accent"], name="rear_plates"),
        TriangleMesh(*rain_light, color=colors["light"], name="rain_light"),
        TriangleMesh(*arms, color=colors["carbon"], name="arms", max_edges=0),
        TriangleMesh(*halo, color=colors["carbon"], name="halo", max_edges=0),
        TriangleMesh(*details, color=colors["accent"], name="details", max_edges=0),
    ]

    # The wheels: wide slicks, the rear wider
    look = {"body": body}
    for wheel in ("fl", "fr", "rl", "rr"):
        front = wheel[0] == "f"
        side = 1.0 if wheel[1] == "l" else -1.0
        look[f"wheel_{wheel}"] = slick_wheel(
            r_f if front else r_r, 0.30 if front else 0.38, side, colors
        )

    # The ground under it, when the plant asks for a scene; the rear wheels on the
    # body when the plant is a bicycle
    look["world"] = terrain_surface(plant, colors["ground"])
    mount_rear_wheels(look, plant)

    return look


def offroad_car_skin_3d(plant, colors=None):
    """An off-road side-by-side: a tall tub, a roll cage, two seats, knobby tires.

    A skin of the 3-D car, as :func:`formula_car_skin_3d`: the body on ``body``, each
    wheel on its own ``wheel_*`` frame. A generic, cartoon utility buggy: no real
    vehicle's shape or badge.
    """
    colors = {**OFFROAD_COLORS, **(colors or {})}
    a, b, track, h, r_f, r_r = chassis(plant)
    g = -h  # The ground, in the body frame
    half = 0.5 * track - 0.12

    # The tub, high off the ground, the hood sloping down to the nose
    tub = loft(
        [
            (a + 0.62, 0.42, 0.42, 0.62),
            (a + 0.30, 0.58, 0.36, 0.86),
            (a - 0.30, 0.66, 0.32, 0.96),
            (0.05, 0.68, 0.30, 0.80),
            (-b + 0.25, 0.68, 0.32, 0.80),
            (-b - 0.42, 0.60, 0.40, 0.92),
        ],
        z0=g,
        power=6.0,
    )

    # The cargo bed and its rail, the seats, the dash
    bed = box_mesh((0.70, 1.20, 0.06), (-b - 0.05, 0.0, g + 0.95))
    seats = merge_meshes(
        *[
            merge_meshes(
                box_mesh((0.50, 0.46, 0.12), (-0.30, side * 0.32, g + 0.86)),
                box_mesh((0.12, 0.46, 0.62), (-0.55, side * 0.32, g + 1.18)),
            )
            for side in (1.0, -1.0)
        ]
    )
    dash = box_mesh((0.18, 1.20, 0.22), (a - 0.55, 0.0, g + 1.02))

    # The fenders over each wheel, and the bumpers and the skid plate
    fenders = merge_meshes(
        *[
            arc_band(
                (x, side * 0.5 * track, g + r),
                r + 0.10,
                0.36,
                np.radians(15),
                np.radians(165),
            )
            for x, r in ((a, r_f), (-b, r_r))
            for side in (1.0, -1.0)
        ]
    )
    bumpers = merge_meshes(
        tube((a + 0.72, 0.62, g + 0.55), (a + 0.72, -0.62, g + 0.55), 0.045),
        tube((a + 0.72, 0.55, g + 0.55), (a + 0.45, 0.45, g + 0.75), 0.04),
        tube((a + 0.72, -0.55, g + 0.55), (a + 0.45, -0.45, g + 0.75), 0.04),
        tube((-b - 0.50, 0.60, g + 0.50), (-b - 0.50, -0.60, g + 0.50), 0.045),
        box_mesh((1.80, 0.80, 0.03), (0.5 * (a - b), 0.0, g + 0.30)),
    )

    # The roll cage: two hoops, the roof rails and their braces
    roof = g + 1.85
    front_hoop = [
        (a - 0.45, half, g + 0.95),
        (-0.05, half - 0.05, roof),
        (-0.05, -half + 0.05, roof),
        (a - 0.45, -half, g + 0.95),
    ]
    rear_hoop = [
        (-b + 0.35, half, g + 0.82),
        (-0.80, half - 0.05, roof),
        (-0.80, -half + 0.05, roof),
        (-b + 0.35, -half, g + 0.82),
    ]
    cage = merge_meshes(
        *[tube(p0, p1, 0.035) for p0, p1 in zip(front_hoop, front_hoop[1:])],
        *[tube(p0, p1, 0.035) for p0, p1 in zip(rear_hoop, rear_hoop[1:])],
        tube(front_hoop[1], rear_hoop[1], 0.035),
        tube(front_hoop[2], rear_hoop[2], 0.035),
        tube((-0.05, 0.0, roof), (-0.80, 0.0, roof), 0.03),
        tube(rear_hoop[0], (-b - 0.35, 0.55, g + 1.00), 0.03),
        tube(rear_hoop[3], (-b - 0.35, -0.55, g + 1.00), 0.03),
        tube((-0.80, half - 0.05, roof), (-b - 0.35, 0.55, g + 1.00), 0.03),
        tube((-0.80, -half + 0.05, roof), (-b - 0.35, -0.55, g + 1.00), 0.03),
    )

    # The light bar on the roof, the headlamps on the nose
    bar = box_mesh((0.08, 1.00, 0.08), (-0.02, 0.0, roof + 0.07))
    lamps = [
        Sphere(radius=0.05, center=(0.03, y, roof + 0.07), color=colors["light"])
        for y in np.linspace(-0.40, 0.40, 5)
    ] + [
        Sphere(radius=0.075, center=(a + 0.62, y, g + 0.68), color=colors["lamp"])
        for y in (0.30, -0.30)
    ]

    body = [
        TriangleMesh(*tub, color=colors["body"], name="tub", max_edges=0),
        TriangleMesh(*fenders, color=colors["trim"], name="fenders", max_edges=0),
        TriangleMesh(*bed, color=colors["trim"], name="bed"),
        TriangleMesh(*seats, color=colors["seat"], name="seats"),
        TriangleMesh(*dash, color=colors["trim"], name="dash"),
        TriangleMesh(*bumpers, color=colors["cage"], name="bumpers", max_edges=0),
        TriangleMesh(*cage, color=colors["cage"], name="cage", max_edges=0),
        TriangleMesh(*bar, color=colors["cage"], name="light_bar"),
        *lamps,
    ]

    # The wheels: tall knobby tires on beadlock rims
    look = {"body": body}
    for wheel in ("fl", "fr", "rl", "rr"):
        radius = r_f if wheel[0] == "f" else r_r
        side = 1.0 if wheel[1] == "l" else -1.0
        look[f"wheel_{wheel}"] = knobby_wheel(radius, 0.30, side, colors)

    # The ground under it, when the plant asks for a scene; the rear wheels on the
    # body when the plant is a bicycle
    look["world"] = terrain_surface(plant, colors["ground"])
    mount_rear_wheels(look, plant)

    return look


###############################################################################
# Internal machinery
###############################################################################


def chassis(plant):
    """The sizes a car look follows: ``a``, ``b``, track, ride height, wheel radii [m].

    The 3-D car keeps them all in ``params``. A bicycle rung has no ``track`` or
    ``h_cg`` there: its track is a drawing attribute and its body frame sits on the
    ground (a ride height of 0); a rung without wheel radii uses its drawn one.
    """
    p = plant.params
    r_wheel = getattr(plant, "wheel_radius", 0.3)

    a = p["a"]
    b = p["b"]
    track = p["track"] if "track" in p else plant.track
    h = p.get("h_cg", 0.0)
    r_f = p.get("r_f", r_wheel)
    r_r = p.get("r_r", r_wheel)

    return a, b, track, h, r_f, r_r


def mount_rear_wheels(look, plant):
    """Move the rear wheels onto the body for a plant with no rear wheel frames."""
    if "rl" in getattr(plant, "wheels", ()):
        return
    a, b, track, h, r_f, r_r = chassis(plant)

    # The hub in the body frame: behind by b, out by half the track, up by r_r − h
    for wheel, side in (("rl", 1.0), ("rr", -1.0)):
        hub = translation_matrix((-b, side * 0.5 * track, r_r - h))
        for shape in look.pop(f"wheel_{wheel}"):
            shape.local_transform = hub @ shape.local_transform
            look["body"].append(shape)


def loft(sections, z0=0.0, y0=0.0, power=4.0, n=28):
    """A closed body through superellipse sections ``(x, half width, bottom, top)``.

    Each section is the superellipse ``|y/w|^power + |z/h|^power = 1`` (rounded boxes
    for ``power`` near 4), its heights measured from ``z0``; the sections are joined
    by bands of triangles and the two ends capped.
    """
    angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    c = np.cos(angles)
    s = np.sin(angles)
    ey = np.sign(c) * np.abs(c) ** (2.0 / power)
    ez = np.sign(s) * np.abs(s) ** (2.0 / power)

    rings = []
    for x, half_width, bottom, top in sections:
        z_mid = z0 + 0.5 * (bottom + top)
        half_height = 0.5 * (top - bottom)
        ring = np.column_stack(
            [np.full(n, x), y0 + half_width * ey, z_mid + half_height * ez]
        )
        rings.append(ring)

    vertices = np.vstack(rings + [rings[0].mean(axis=0), rings[-1].mean(axis=0)])
    first_cap = len(rings) * n
    last_cap = first_cap + 1

    # Each pair of rings joined by a band; each end closed by a fan
    faces = []
    for k in range(len(rings) - 1):
        for i in range(n):
            j = (i + 1) % n
            a0, a1 = k * n + i, k * n + j
            b0, b1 = (k + 1) * n + i, (k + 1) * n + j
            faces += [(a0, b0, b1), (a0, b1, a1)]
    for i in range(n):
        j = (i + 1) % n
        faces.append((first_cap, i, j))
        faces.append((last_cap, (len(rings) - 1) * n + j, (len(rings) - 1) * n + i))

    return vertices, np.asarray(faces, dtype=np.uint32)


def tube(p0, p1, radius, n=12):
    """A closed cylinder of ``radius`` from ``p0`` to ``p1``."""
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    axis = p1 - p0
    length = np.linalg.norm(axis)

    # The cylinder along z, turned onto the segment and moved to its start
    mesh = cylinder_mesh(radius, 0.0, length, n=n)
    z = np.array([0.0, 0.0, 1.0])
    turn = np.cross(z, axis / length)
    angle = np.arctan2(np.linalg.norm(turn), np.dot(z, axis / length))
    R = axis_angle_matrix(turn, angle) if np.linalg.norm(turn) > 1e-12 else np.eye(4)
    if np.linalg.norm(turn) <= 1e-12 and axis[2] < 0.0:
        R = axis_angle_matrix((1.0, 0.0, 0.0), np.pi)

    return transform_mesh(mesh, translation_matrix(p0) @ R)


def pitched(mesh, angle):
    """A wing element pitched leading edge down by ``angle`` [rad] about its spanwise centreline."""
    vertices, _ = mesh
    centre = 0.5 * (vertices.min(axis=0) + vertices.max(axis=0))
    turn = (
        translation_matrix(centre)
        @ axis_angle_matrix((0.0, 1.0, 0.0), angle)
        @ translation_matrix(-centre)
    )

    return transform_mesh(mesh, turn)


def arc_band(center, radius, width, angle0, angle1, thickness=0.04, n=14):
    """A fender: a curved strip of ``width`` along y, over the arc ``angle0..angle1`` in x-z."""
    center = np.asarray(center, dtype=float)
    angles = np.linspace(angle0, angle1, n)

    # Four edges along the arc: inner and outer radius, each side of the strip
    rings = []
    for r_k in (radius, radius + thickness):
        for y in (-0.5 * width, 0.5 * width):
            rings.append(
                np.column_stack(
                    [
                        center[0] + r_k * np.cos(angles),
                        np.full(n, center[1] + y),
                        center[2] + r_k * np.sin(angles),
                    ]
                )
            )
    vertices = np.vstack(rings)

    # The four faces of the strip: inner, outer, and the two sides
    faces = []
    for e0, e1 in ((0, 1), (2, 3), (0, 2), (1, 3)):
        for i in range(n - 1):
            a0, a1 = e0 * n + i, e0 * n + i + 1
            b0, b1 = e1 * n + i, e1 * n + i + 1
            faces += [(a0, b0, b1), (a0, b1, a1)]

    return vertices, np.asarray(faces, dtype=np.uint32)


def on_axle(mesh, side):
    """A mesh built about local z, turned so z lies along the axle, outward (``side`` ±1 along y)."""
    return transform_mesh(mesh, axis_angle_matrix((1.0, 0.0, 0.0), -side * 0.5 * np.pi))


def slick_wheel(radius, width, side, colors):
    """A slick: a tire with rounded shoulders, a rim with five spokes, a painted nut."""
    half = 0.5 * width
    tire = merge_meshes(
        ring_mesh(0.66 * radius, radius, -half + 0.03, half - 0.03, n=40),
        ring_mesh(0.66 * radius, 0.95 * radius, -half, half, n=40),
    )
    rim = ring_mesh(0.60 * radius, 0.66 * radius, -half + 0.02, half - 0.02, n=32)
    spokes = merge_meshes(
        *[
            transform_mesh(
                box_mesh(
                    (0.56 * radius, 0.07 * radius, 0.06),
                    (0.3 * radius, 0.0, half - 0.06),
                ),
                axis_angle_matrix((0.0, 0.0, 1.0), k * 2.0 * np.pi / 5.0),
            )
            for k in range(5)
        ]
    )
    nut = cylinder_mesh(0.14 * radius, half - 0.08, half - 0.01, n=16)

    return [
        TriangleMesh(
            *on_axle(tire, side), color=colors["tire"], name="slick", max_edges=0
        ),
        TriangleMesh(*on_axle(rim, side), color=colors["rim"], name="rim", max_edges=0),
        TriangleMesh(*on_axle(spokes, side), color=colors["rim"], name="spokes"),
        TriangleMesh(*on_axle(nut, side), color=colors["hub"], name="nut", max_edges=0),
    ]


def knobby_wheel(radius, width, side, colors):
    """A knobby: a tire with tread lugs, a beadlock rim with six spokes."""
    half = 0.5 * width
    carcass = ring_mesh(0.62 * radius, 0.92 * radius, -half, half, n=40)
    lugs = merge_meshes(
        *[
            transform_mesh(
                box_mesh(
                    (0.10 * radius, 0.24 * radius, 0.42 * width),
                    (0.95 * radius, 0.0, (0.25 if k % 2 else -0.25) * width),
                ),
                axis_angle_matrix((0.0, 0.0, 1.0), k * 2.0 * np.pi / 22.0),
            )
            for k in range(22)
        ]
    )
    rim = ring_mesh(0.54 * radius, 0.62 * radius, -half + 0.02, half - 0.02, n=32)
    beadlock = ring_mesh(0.56 * radius, 0.66 * radius, half - 0.03, half, n=32)
    spokes = merge_meshes(
        *[
            transform_mesh(
                box_mesh(
                    (0.50 * radius, 0.09 * radius, 0.05),
                    (0.27 * radius, 0.0, half - 0.07),
                ),
                axis_angle_matrix((0.0, 0.0, 1.0), k * np.pi / 3.0),
            )
            for k in range(6)
        ]
    )
    hub = cylinder_mesh(0.16 * radius, half - 0.09, half - 0.02, n=16)

    return [
        TriangleMesh(
            *on_axle(carcass, side), color=colors["tire"], name="carcass", max_edges=0
        ),
        TriangleMesh(
            *on_axle(lugs, side), color=colors["tire"], name="lugs", max_edges=0
        ),
        TriangleMesh(*on_axle(rim, side), color=colors["rim"], name="rim", max_edges=0),
        TriangleMesh(
            *on_axle(beadlock, side),
            color=colors["beadlock"],
            name="beadlock",
            max_edges=0,
        ),
        TriangleMesh(*on_axle(spokes, side), color=colors["rim"], name="spokes"),
        TriangleMesh(*on_axle(hub, side), color=colors["hub"], name="hub", max_edges=0),
    ]
