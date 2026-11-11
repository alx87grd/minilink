"""Two cartoon looks for the 3-D car: an open-wheel formula car and an off-road side-by-side."""

from __future__ import annotations

import numpy as np

from minilink.graphical.animation.primitives import Sphere
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
    "body": "#1d4ed8",
    "stripe": "#f5f5f5",
    "accent": "#f97316",
    "carbon": "#17181c",
    "cockpit": "#0b0b0d",
    "helmet": "#facc15",
    "visor": "#111827",
    "tire": "#141414",
    "rim": "#c0c4cc",
    "hub": "#f97316",
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
    """An open-wheel formula car: a slim monocoque, sidepods, two wings, slick tires.

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

    # The monocoque: a nose rising to the cockpit, the engine cover, the tail
    # (x, half width, bottom, top), the heights above the ground
    tub = loft(
        [
            (a + 1.05, 0.06, 0.13, 0.20),
            (a + 0.55, 0.14, 0.11, 0.34),
            (a + 0.05, 0.21, 0.09, 0.46),
            (a - 0.60, 0.33, 0.07, 0.58),
            (0.25, 0.40, 0.06, 0.64),
            (-0.45, 0.33, 0.07, 0.86),
            (-b + 0.35, 0.22, 0.10, 0.62),
            (-b - 0.25, 0.10, 0.18, 0.38),
        ],
        z0=g,
    )
    stripe = loft(
        [
            (a + 1.02, 0.025, 0.205, 0.215),
            (a + 0.55, 0.05, 0.345, 0.355),
            (a + 0.05, 0.07, 0.466, 0.476),
            (a - 0.55, 0.08, 0.585, 0.595),
        ],
        z0=g,
    )

    # The sidepods, either side of the cockpit
    pods = merge_meshes(
        *[
            loft(
                [
                    (a - 0.55, 0.10, 0.10, 0.34),
                    (a - 0.95, 0.26, 0.08, 0.46),
                    (-0.35, 0.24, 0.08, 0.42),
                    (-b + 0.45, 0.10, 0.12, 0.26),
                ],
                z0=g,
                y0=side * 0.52,
            )
            for side in (1.0, -1.0)
        ]
    )

    # The floor, wider than the tub, and the cockpit opening
    floor = box_mesh((a + b + 0.6, 1.25, 0.03), (0.5 * (a - b), 0.0, g + 0.055))
    opening = box_mesh((0.80, 0.42, 0.04), (-0.05, 0.0, g + 0.645))

    # The front wing: two planes, the endplates painted
    span = track + 0.30
    front_wing = merge_meshes(
        box_mesh((0.36, span, 0.025), (a + 0.92, 0.0, g + 0.09)),
        box_mesh((0.22, span - 0.10, 0.02), (a + 0.80, 0.0, g + 0.16)),
    )
    front_plates = merge_meshes(
        *[
            box_mesh((0.48, 0.02, 0.18), (a + 0.86, side * 0.5 * span, g + 0.13))
            for side in (1.0, -1.0)
        ]
    )

    # The rear wing on its pylon, the endplates painted
    rear_x = -b - 0.40
    rear_wing = merge_meshes(
        box_mesh((0.34, 1.00, 0.03), (rear_x, 0.0, g + 0.86)),
        box_mesh((0.20, 1.00, 0.025), (rear_x + 0.04, 0.0, g + 0.97)),
        box_mesh((0.10, 0.04, 0.60), (rear_x + 0.10, 0.0, g + 0.56)),
    )
    rear_plates = merge_meshes(
        *[
            box_mesh((0.50, 0.025, 0.45), (rear_x, side * 0.51, g + 0.82))
            for side in (1.0, -1.0)
        ]
    )

    # The suspension arms, from the tub to each hub at rest, and the halo
    arms = merge_meshes(
        *[
            tube(
                (x_body, side * 0.30, g + z), (x_hub, side * 0.5 * track, g + r), 0.022
            )
            for x_body, x_hub, r in ((a - 0.15, a, r_f), (-b + 0.15, -b, r_r))
            for side in (1.0, -1.0)
            for z in (0.20, 0.36)
        ]
    )
    halo = merge_meshes(
        tube((0.10, 0.20, g + 0.66), (0.35, 0.0, g + 0.86), 0.025),
        tube((0.10, -0.20, g + 0.66), (0.35, 0.0, g + 0.86), 0.025),
        tube((0.35, 0.0, g + 0.86), (0.62, 0.0, g + 0.62), 0.025),
    )

    body = [
        TriangleMesh(*tub, color=colors["body"], name="monocoque", max_edges=0),
        TriangleMesh(*stripe, color=colors["stripe"], name="stripe", max_edges=0),
        TriangleMesh(*pods, color=colors["body"], name="sidepods", max_edges=0),
        TriangleMesh(*floor, color=colors["carbon"], name="floor", max_edges=0),
        TriangleMesh(*opening, color=colors["cockpit"], name="cockpit", max_edges=0),
        TriangleMesh(*front_wing, color=colors["carbon"], name="front_wing"),
        TriangleMesh(*front_plates, color=colors["accent"], name="front_plates"),
        TriangleMesh(*rear_wing, color=colors["carbon"], name="rear_wing"),
        TriangleMesh(*rear_plates, color=colors["accent"], name="rear_plates"),
        TriangleMesh(*arms, color=colors["carbon"], name="arms", max_edges=0),
        TriangleMesh(*halo, color=colors["carbon"], name="halo", max_edges=0),
        Sphere(radius=0.13, center=(-0.02, 0.0, g + 0.74), color=colors["helmet"]),
        Sphere(radius=0.085, center=(0.06, 0.0, g + 0.75), color=colors["visor"]),
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


def terrain_surface(plant, color, step=0.25):
    """The plant's terrain as one mesh over ``plant.scene_bounds``; none without bounds.

    ``scene_bounds`` ``(x_min, x_max, y_min, y_max)`` [m] is the opt-in, as for the
    racecar's scene. The height comes from ``plant.terrain`` (flat when the plant has
    none), sampled every ``step`` metres.
    """
    bounds = getattr(plant, "scene_bounds", None)
    if bounds is None:
        return []
    x_min, x_max, y_min, y_max = bounds

    # A grid of heights, two triangles per cell
    xs = np.arange(x_min, x_max + 1e-9, step)
    ys = np.arange(y_min, y_max + 1e-9, step)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    terrain = getattr(plant, "terrain", None)
    Z = np.zeros_like(X) if terrain is None else np.asarray(terrain.height(X, Y))
    Z = np.broadcast_to(Z, X.shape)
    vertices = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
    n_y = ys.size
    i, j = np.meshgrid(np.arange(xs.size - 1), np.arange(n_y - 1), indexing="ij")
    k = (i * n_y + j).ravel()
    faces = np.concatenate(
        [
            np.column_stack([k, k + n_y, k + n_y + 1]),
            np.column_stack([k, k + n_y + 1, k + 1]),
        ]
    ).astype(np.uint32)

    ground = TriangleMesh(vertices, faces, color=color, name="terrain", max_edges=0)
    ground.camera_fit = False

    return [ground]


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
