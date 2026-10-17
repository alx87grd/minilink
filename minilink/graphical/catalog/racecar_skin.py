"""Racecar skins: the 1/10 racecar's link frames and its looks, in 3-D and from the top."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, SE3, Rx, Ry, Rz, translation
from minilink.graphical.catalog import (
    Arrow,
    Box,
    Circle,
    ExtrudedPolygon,
    Line,
    Rod,
    link_pose_3d,
)
from minilink.graphical.meshes import (
    TriangleMesh,
    box_mesh,
    cylinder_mesh,
    face_normals,
    load_stl,
    merge_meshes,
    ring_mesh,
    transform_mesh,
    weld,
)

# The CAD meshes ship with the package
ASSETS = Path(__file__).resolve().parents[1] / "assets" / "racecar"
MESH_DIR = ASSETS / "meshes"

# The links: :func:`racecar_frames` places each one, and the looks draw on them
WHEEL_LINKS = (
    "left_rear_wheel",
    "right_rear_wheel",
    "left_front_wheel",
    "right_front_wheel",
)
LINKS = (
    "base_footprint",
    "base_link",
    "chassis",
    "chassis_inertia",
    "left_rear_wheel",
    "right_rear_wheel",
    "left_steering_hinge",
    "right_steering_hinge",
    "left_front_wheel",
    "right_front_wheel",
    "base_laser",
    "camera_link",
    "camera_optical_link",
    "imu_link",
)

# 1/10 geometry [m] and joint rpy [rad]
WHEELBASE = 0.34
HALF_TRACK = 0.10
AXLE_HEIGHT = 0.05
HALF_PI = 1.5708
LASER_XYZ = (0.1387, 0.0, 0.166)
LASER_YAW = 3.14159
CAMERA_XYZ = (0.21, 0.0, 0.13)
CAMERA_OPTICAL_RPY = (-1.570796327, 0.0, -1.570796327)
IMU_XYZ = (0.3, -0.115, 0.0)
CHASSIS_VISUAL_XYZ = (0.0, 0.0, 0.05)
CAMERA_BOX = (0.033, 0.03, 0.03)
WHEEL_AXIS = (0.0, 0.0, -1.0)
HINGE_AXIS = (-1.0, 0.0, 0.0)
HINGE_RPY = (0.0, HALF_PI, 0.0)
WHEEL_RPY = (HALF_PI, 0.0, 0.0)

# The assembled platform: a 1/10-scale truck chassis in black plastic, black
# tires on black split-spoke wheels with a light bronze beadlock ring, clear acrylic
# decks, a black RPLidar A2M8 with its red ring, a black RaspiCam, a red emergency
# stop, a silver single-board-computer case and a black Arduino shield.
REAL_PALETTE = {
    "chassis": "#2b2d31",
    "tire": "#18181a",
    "rim": "#0c0c0d",
    "spokes": "#4b4e54",
    "beadlock": "#bba47c",
    "hub": "#c9ccd1",
    "hinge": "#1d1e21",
    "knuckle": "#232427",
    "acrylic": "#cfe3ee",
    "acrylic_opacity": 0.32,
    "standoff": "#b9bcc1",
    "lidar": "#1c1c1f",
    "lidar_ring": "#a3181c",
    "lidar_window": "#050507",
    "camera": "#18181a",
    "lens": "#0a0a0c",
    "glass": "#2b3b5c",
    "estop": "#d11f1f",
    "estop_collar": "#c9ccd1",
    "estop_base": "#d9d3c2",
    "powerbank": "#1f2023",
    "rpi": "#a9adb3",
    "arduino": "#1d5d8c",
    "shield": "#1a1a1c",
    "shock": "#1f2124",
    "spring": "#34373d",
    "deck_line": "#6b9fc4",
    "bumper": "#151517",
    "shadow": "#000000",
    "floor": "#c4c4c2",
    "floor_alt": "#b6b6b4",
    "wall": "#d9d0bf",
    "wall_opacity": 0.5,
}


###############################################################################
# Public API
###############################################################################


# Placement: pure kinematics, called from a plant's tf


def racecar_frames(
    X,
    Y,
    psi,
    delta,
    phi_rear=0.0,
    phi_front=None,
    ref_to_rear_axle=0.0,
    wheelbase=None,
    wheel_radius=None,
):
    """World poses of the racecar links for a planar pose, steering and wheel angles.

    The placement of the looks: planar pose, Ackermann steering and wheel roll. A
    plant calls it from its ``tf``.

    Parameters
    ----------
    X, Y : float
        World position [m] of the plant's reference point (on the ground).
    psi : float
        Heading [rad], counter-clockwise from world X.
    delta : float
        Bicycle-model steering angle [rad], positive to the left. The two hinges
        get the Ackermann angles of :func:`ackermann_angles`.
    phi_rear, phi_front : float
        Rolling angles of the rear and front wheels [rad], positive when rolling
        forward (``phi = distance / wheel radius`` without slip). ``phi_front``
        defaults to ``phi_rear``.
    ref_to_rear_axle : float
        Distance [m] from the reference point forward of the rear axle: 0 for a
        rear-axle model, ``b`` for a model carrying its pose at the centre of gravity.
    wheelbase : float, optional
        Moves the steering hinges to this distance from the rear axle (default
        0.34 m).
    wheel_radius : float, optional
        Raises ``base_link`` (the axles) to this height, so a tread spinning at
        the model's wheel rate rolls on the ground without sliding (default
        0.05 m).

    Returns
    -------
    dict[str, (4, 4) array]
        ``body`` (the plant's reference point on the ground, heading ``psi``),
        ``axle_front`` (the front axle centre on the ground, turned by ``delta``),
        and every link of :data:`LINKS`: ``base_footprint``, ``base_link``,
        ``chassis``, ``chassis_inertia``, ``left/right_rear_wheel``,
        ``left/right_steering_hinge``, ``left/right_front_wheel``, ``base_laser``,
        ``camera_link``, ``camera_optical_link`` and ``imu_link``.
    """
    L = WHEELBASE if wheelbase is None else wheelbase
    r = AXLE_HEIGHT if wheel_radius is None else wheel_radius
    phi_front = phi_rear if phi_front is None else phi_front

    # Each hinge's steer angle, from the bicycle angle δ
    delta_left, delta_right = ackermann_angles(delta, L, 2.0 * HALF_TRACK)

    # The chassis: the footprint behind the reference point, the axles at height r
    T_body = SE2(X, Y, psi)
    T_footprint = T_body @ translation(-ref_to_rear_axle, 0.0, 0.0)
    T_base = join(T_footprint, (0.0, 0.0, r))
    T_chassis = join(T_base, (0.0, 0.0, 0.0))

    # The rear wheels roll by φ_rear; the hinges turn by their Ackermann angles
    T_left_rear = join(
        T_chassis, (0.0, HALF_TRACK, 0.0), WHEEL_RPY, phi_rear, WHEEL_AXIS
    )
    T_right_rear = join(
        T_chassis, (0.0, -HALF_TRACK, 0.0), WHEEL_RPY, phi_rear, WHEEL_AXIS
    )
    T_left_hinge = join(
        T_chassis, (L, HALF_TRACK, 0.0), HINGE_RPY, delta_left, HINGE_AXIS
    )
    T_right_hinge = join(
        T_chassis, (L, -HALF_TRACK, 0.0), HINGE_RPY, delta_right, HINGE_AXIS
    )

    # The sensors
    T_laser = join(T_chassis, LASER_XYZ, (0.0, 0.0, LASER_YAW))
    T_camera = join(T_chassis, CAMERA_XYZ)
    T_imu = join(T_base, IMU_XYZ)

    # The front axle centre, turned by the bicycle angle δ
    T_axle_front = T_body @ SE2(L - ref_to_rear_axle, 0.0, delta)

    # The front wheels roll by φ_front on their hinges; the camera's optical frame
    T_left_front = join(T_left_hinge, (0.0, 0.0, 0.0), WHEEL_RPY, phi_front, WHEEL_AXIS)
    T_right_front = join(
        T_right_hinge, (0.0, 0.0, 0.0), WHEEL_RPY, phi_front, WHEEL_AXIS
    )
    T_optical = join(T_camera, (0.0, 0.0, 0.0), CAMERA_OPTICAL_RPY)

    return {
        "base_footprint": T_footprint,
        "base_link": T_base,
        "chassis": T_chassis,
        "chassis_inertia": T_chassis,
        "left_rear_wheel": T_left_rear,
        "right_rear_wheel": T_right_rear,
        "left_steering_hinge": T_left_hinge,
        "right_steering_hinge": T_right_hinge,
        "base_laser": T_laser,
        "camera_link": T_camera,
        "imu_link": T_imu,
        "body": T_body,
        "axle_front": T_axle_front,
        "left_front_wheel": T_left_front,
        "right_front_wheel": T_right_front,
        "camera_optical_link": T_optical,
    }


def ackermann_angles(delta, wheelbase, track):
    """Left and right wheel steer angles of an ideal Ackermann linkage.

    Both front wheels turn about the same centre as the bicycle model with steer
    angle *delta* (positive = left turn), for a *wheelbase* ``L`` and a *track*
    ``w`` between the steering axes. The inner wheel turns more.
    """
    xp = array_module(delta)

    L = wheelbase
    w = track

    t = xp.tan(delta)

    # tan δ_left = L tan δ / (L − (w/2) tan δ)
    # tan δ_right = L tan δ / (L + (w/2) tan δ)
    delta_left = xp.arctan2(L * t, L - 0.5 * w * t)
    delta_right = xp.arctan2(L * t, L + 0.5 * w * t)

    return delta_left, delta_right


# The looks


def racecar_skin_3d(
    plant=None,
    palette="real",
    equipment=True,
    scene=True,
    wheel_radius=None,
):
    """The racecar in 3-D: chassis, tires and hinges, plus decks and equipment.

    A look, ``(plant) -> dict[frame, list[primitive]]``, with no state math.
    Chassis, tires and hinges are the CAD triangle meshes; decks, lidar, camera,
    electronics, shocks and bumpers are minilink primitives.

    Parameters
    ----------
    plant : System, optional
        Read for the scene (``scene_bounds`` and friends, :func:`scene_from`) and
        for ``wheel_radius`` [m] when it has one.
    palette : {"real"} or dict
        Colours of the assembled vehicle; a dict overrides single entries of
        :data:`REAL_PALETTE`.
    equipment : bool
        Decks, electronics, e-stop, camera bracket, shocks, bumpers, wheel detail.
    scene : bool
        The ground the plant asks for (``world``) and the contact shadow.
    wheel_radius : float, optional
        Draws the tires at this radius (scaled about the axle), so that a wheel
        spinning at the model's rate rolls without sliding on the floor.
        ``plant.wheel_radius`` when omitted, else 0.05 m.
        :func:`racecar_frames` must get the same ``wheel_radius`` so the tires
        touch the ground.

    Returns
    -------
    dict[str, list]
        Primitives keyed to the frames of :func:`racecar_frames`.
    """
    colors = palette_colors(palette)
    r = drawn_wheel_radius(plant, wheel_radius)

    # The CAD chassis, and the lidar on its mount
    geometry = {
        "chassis": [
            mesh_primitive(
                "chassis.STL",
                colors["chassis"],
                name="chassis",
                xyz=CHASSIS_VISUAL_XYZ,
            )
        ],
        "base_laser": rplidar(colors),
    }

    # The tires, scaled about their axles to the drawn radius, with their detail
    for link in WHEEL_LINKS:
        tire = mesh_primitive(
            f"{link}.STL",
            colors["tire"],
            name=link,
            radial_scale=r / GEOMETRY["wheel_radius"],
        )
        details = wheel_details(link, colors, r) if equipment else []
        geometry[link] = [tire] + details

    # The steering hinges, with their knuckles
    for link in ("left_steering_hinge", "right_steering_hinge"):
        hinge = mesh_primitive(f"{link}.STL", colors["hinge"], name=link)
        geometry[link] = [hinge] + (steering_knuckle(link, colors) if equipment else [])

    # The decks and electronics, or a plain camera box
    if equipment:
        geometry["chassis"] += chassis_equipment(colors)
        geometry["camera_link"] = raspicam(colors)
    else:
        lx, ly, lz = CAMERA_BOX
        geometry["camera_link"] = [
            Box(length_x=lx, length_y=ly, length_z=lz, color=colors["camera"])
        ]

    # The contact shadow, and the ground the plant asks for
    if scene:
        geometry["base_footprint"] = [contact_shadow(colors)]
        world = scene_from(plant, palette)
        if world:
            geometry["world"] = world

    return geometry


def racecar_skin_2d(
    plant=None,
    palette="real",
    equipment=True,
    scene=True,
    heading_arrow=False,
    wheel_radius=None,
):
    """Top view of the racecar for fast matplotlib animations.

    The chassis outline is the top face of the chassis mesh; the four tires
    sit at the wheel joints (front ones on the steered hinges, Ackermann);
    the RPLidar is a filled disk with its red ring; the decks, electronics,
    e-stop and bumpers follow the assembled vehicle. Lines and circles only, so
    minilink's stock 2-D matplotlib renderer draws it fast — this is the look to
    animate with, since a GIF of the 3-D look comes out as a wireframe top view.

    Parameters are those of :func:`racecar_skin_3d`, plus *heading_arrow*
    (an arrow on ``body``).
    """
    colors = palette_colors(palette)

    # The chassis outline: the top face of the chassis mesh, at its visual origin
    vertices, faces = stl_mesh("chassis.STL")
    outline = outline_xy(*transform_mesh((vertices, faces), origin(CHASSIS_VISUAL_XYZ)))
    half_track = 0.5 * GEOMETRY["wheel_track"]
    r = drawn_wheel_radius(plant, wheel_radius)

    # The chassis hatched around the lidar and the e-stop, outlined, and the rear tires
    lidar_x = LASER_XYZ[0]
    holes = [(lidar_x, 0.0, 0.041), (ESTOP_XY[0], ESTOP_XY[1], 0.022)]
    chassis = [
        Line(hatch, color="#9a9ca1", linewidth=0.6)
        for hatch in hatch_lines(outline, 0.012, holes)
    ]
    chassis.append(Line(closed(outline), color=colors["chassis"], linewidth=1.8))
    for y in (-half_track, half_track):
        chassis += wheel_2d(y, colors, r)

    # The front tires, on the steered hinges
    geometry = {"chassis": chassis}
    for link in ("left_steering_hinge", "right_steering_hinge"):
        s = -outward_sign(link)
        parts = wheel_2d(s * 0.5 * GEOMETRY["wheel_width"], colors, r)
        for part in parts:
            part.local_transform = aligned()
        geometry[link] = parts

    # The decks, the Arduino shield, the power bank (dashed), the bumpers, the e-stop
    # and the camera
    if equipment:
        geometry["chassis"] += [
            Line(closed(LOWER_DECK), color=colors["deck_line"], linewidth=1.0),
            Line(closed(UPPER_DECK), color=colors["deck_line"], linewidth=1.0),
            Line(
                closed(rectangle(-0.050 - 0.0508, -0.050 + 0.0508, -0.0267, 0.0267)),
                color=colors["shield"],
                linewidth=1.2,
            ),
            Line(
                closed(rectangle(0.04, 0.17, -0.034, 0.034)),
                color="#555555",
                linewidth=0.8,
                style="--",
            ),
            Line(
                np.array(
                    [
                        [0.37, 0.065, 0],
                        [0.478, 0.065, 0],
                        [0.478, -0.065, 0],
                        [0.37, -0.065, 0],
                    ]
                ),
                color=colors["bumper"],
                linewidth=2.2,
            ),
            Line(
                closed(
                    rectangle(*REAR_BUMPER_X[:2], -REAR_BUMPER_X[2], REAR_BUMPER_X[2])
                ),
                color=colors["bumper"],
                linewidth=2.2,
            ),
        ]
        estop = Circle(
            radius=0.02, center=(*ESTOP_XY, 0.0), color=colors["estop"], fill=True
        )
        geometry["chassis"].append(estop)
        geometry["camera_link"] = [
            Line(
                closed(rectangle(-0.008, 0.008, -0.015, 0.015)),
                color=colors["camera"],
                linewidth=1.4,
            ),
            Line(
                np.array([[0.008, 0.0, 0.0], [0.022, 0.0, 0.0]]),
                color=colors["lens"],
                linewidth=3.0,
            ),
        ]

    # The lidar: a filled disk with its red ring
    geometry["base_laser"] = [
        Circle(radius=0.038, color=colors["lidar"], fill=True),
        Circle(radius=0.0385, color=colors["lidar_ring"]),
    ]

    if heading_arrow:
        geometry["body"] = [
            Arrow(
                base=(0.0, 0.0), vector=(0.25, 0.0), color="tab:orange", linewidth=1.5
            )
        ]

    # The ground the plant asks for, in lines
    if scene:
        world = scene_lines_from(plant, colors)
        if world:
            geometry["world"] = world

    return geometry


def frame_axes_skin(plant=None, length=0.04, links=None):
    """Debug look: an RGB triad (x red, y green, z blue) at every link frame.

    Stack it on a look with ``merge_skins(racecar_skin_3d, frame_axes_skin)`` to
    check that meshes sit in their joint frames.
    """
    geometry = {}
    for link in links or LINKS:
        geometry[link] = [
            Line(np.array([[0.0, 0.0, 0.0], axis]), color=color, linewidth=2.0)
            for axis, color in zip(
                length * np.eye(3), ("#d62728", "#2ca02c", "#1f77b4")
            )
        ]

    return geometry


# The scene: the ground the car drives on


def track_scene(
    bounds=(-1.0, 12.0, -1.0, 1.0),
    walls=False,
    wall_height=0.3,
    tile=0.5,
    path=None,
    palette="real",
):
    """World geometry for a vehicle animation: tiled floor, walls, painted line.

    Parameters
    ----------
    bounds : (x_min, x_max, y_min, y_max)
        Extent of the floor [m]. Round it out to whole tiles for a clean edge.
    walls : bool
        Two see-through walls along the ``y`` bounds, which turns the floor into a
        corridor (a lane to drive down, or the sides of an indoor track).
    wall_height : float
        Height of those walls [m].
    tile : float
        Side of a floor tile [m]; the tiles alternate in colour, which is what makes
        motion legible in a chase view.
    path : array of shape (N, 2), optional
        A line painted flat on the floor, e.g. the waypoints the vehicle is tracking.
    palette : {"real"} or dict
        Same colours as the vehicle looks.

    Returns
    -------
    list
        Primitives for the ``world`` key. Backdrops only (``camera_fit = False``),
        like minilink's ground planes, so they never enlarge an auto-fit view.
    """
    colors = palette_colors(palette)
    x_min, x_max, y_min, y_max = (float(v) for v in bounds)

    # The floor: two checkerboard meshes of alternating colour
    floor_a, floor_b = floor_tiles(x_min, x_max, y_min, y_max, tile)
    parts = [
        TriangleMesh(*floor_a, color=colors["floor"], name="floor", max_edges=0),
        TriangleMesh(*floor_b, color=colors["floor_alt"], name="floor", max_edges=0),
    ]

    # The see-through walls, in half-tile panels
    if walls:
        for y in (y_min, y_max):
            wall = wall_panels(x_min, x_max, y, wall_height, 0.5 * tile)
            parts.append(
                TriangleMesh(
                    *wall,
                    color=colors["wall"],
                    opacity=colors["wall_opacity"],
                    name="wall",
                    max_edges=0,
                )
            )

    # The painted line
    if path is not None:
        parts.append(painted_line(path, colors))

    # Backdrops only: never fitted by the camera
    for primitive in parts:
        primitive.camera_fit = False

    return parts


def track_lines(bounds, walls=False, tile=0.5, path=None, colors=None):
    """Top view of :func:`track_scene`: tile joints, walls and the painted line.

    One serpentine polyline for the joints, so the stock 2-D renderer draws the whole
    floor grid in a single artist.
    """
    colors = REAL_PALETTE if colors is None else colors
    x_min, x_max, y_min, y_max = (float(v) for v in bounds)

    # The tile joints, as one serpentine up and down the floor
    joints = []
    for k, x in enumerate(np.arange(x_min, x_max + 1e-9, tile)):
        ya, yb = (y_min, y_max) if k % 2 == 0 else (y_max, y_min)
        joints += [[x, ya, 0.0], [x, yb, 0.0]]
    lines = [Line(np.array(joints), color="#d9d9d9", linewidth=0.7)]

    # The painted line, and the walls
    if path is not None:
        lines.append(painted_line(path, colors))
    if walls:
        for y in (y_min, y_max):
            lines.append(
                Line(
                    np.array([[x_min, y, 0.0], [x_max, y, 0.0]]),
                    color="#6d6d6d",
                    linewidth=3.0,
                )
            )

    # Backdrops only: never fitted by the camera
    for line in lines:
        line.camera_fit = False

    return lines


# Dimensions


def racecar_geometry():
    """Vehicle dimensions [m], with tire size read from the wheel mesh.

    ``wheelbase`` (rear axle to the steering hinges), ``kingpin_track`` (between
    the hinges), ``wheel_track`` (between the tire mid-planes), ``axle_height``
    (``base_footprint`` to ``base_link``), ``wheel_radius`` and ``wheel_width``
    (wheel mesh), ``lidar_height`` (scan plane above the ground).
    """
    wheel = stl_mesh("left_rear_wheel.STL")[0]

    # The tire's width along its axle, the mesh's z
    width = float(np.ptp(wheel[:, 2]))

    # The tracks between the hinges and the tire mid-planes, the tire's largest
    # radius about its axle, and the scan plane above the ground
    kingpin_track = 2.0 * HALF_TRACK
    wheel_track = 2.0 * HALF_TRACK + width
    wheel_radius = float(np.max(np.hypot(wheel[:, 0], wheel[:, 1])))
    lidar_height = AXLE_HEIGHT + LASER_XYZ[2]

    return {
        "wheelbase": WHEELBASE,
        "kingpin_track": kingpin_track,
        "wheel_track": wheel_track,
        "axle_height": AXLE_HEIGHT,
        "wheel_radius": wheel_radius,
        "wheel_width": width,
        "lidar_height": lidar_height,
    }


###############################################################################
# Internal machinery
###############################################################################


# Link frames


def join(parent, xyz, rpy=(0.0, 0.0, 0.0), q=None, axis=None):
    """A child link's pose: its joint origin, then its joint (fixed when *q* is ``None``)."""
    # T_child = T_parent T_origin(xyz, rpy) T_joint(axis, q)
    T = parent @ origin(xyz, rpy)
    if q is not None:
        T = T @ axis_rotation(axis, q)

    return T


def origin(xyz, rpy=(0.0, 0.0, 0.0)):
    """4x4 pose of a joint or visual origin."""
    T = SE3(rpy_matrix(rpy), np.asarray(xyz, dtype=float))

    return T


def rpy_matrix(rpy):
    """The rotation of fixed-axis roll, pitch and yaw angles [rad]."""
    roll, pitch, yaw = (float(a) for a in rpy)

    R = np.asarray(Rz(yaw) @ Ry(pitch) @ Rx(roll))

    return R


def axis_rotation(axis, q):
    """4x4 rotation by *q* about the unit *axis* (Rodrigues; *q* may be a JAX tracer)."""
    xp = array_module(q)

    k = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    kx = k[0]
    ky = k[1]
    kz = k[2]

    # K = [k]ₓ, the cross-product matrix of the unit axis
    # fmt: off
    K = np.array([
        [0.0, -kz, ky],
        [kz, 0.0, -kx],
        [-ky, kx, 0.0],
    ])
    # fmt: on

    # R = I + sin q K + (1 − cos q) K²
    R = np.eye(3) + xp.sin(q) * K + (1.0 - xp.cos(q)) * (K @ K)
    T = SE3(R, 0.0)

    return T


def aligned():
    """Constant offset that re-expresses a steering hinge frame with its parent's axes.

    The steering hinge frames are rotated by the joint rpy ``(0, 1.5708, 0)``;
    geometry authored in chassis-like axes (x forward, y left, z up) at the hinge
    uses ``local_transform = aligned() @ T``.
    """
    T = SE3(rpy_matrix(HINGE_RPY).T, 0.0)

    return T


# Palette, meshes and the drawn wheel


def palette_colors(palette):
    """Skin colours: ``"real"`` is :data:`REAL_PALETTE`; a dict overrides its entries."""
    if isinstance(palette, dict):
        return {**REAL_PALETTE, **palette}
    if palette == "real":
        return dict(REAL_PALETTE)
    raise ValueError(f"palette must be 'real' or a dict, got {palette!r}")


def drawn_wheel_radius(plant, wheel_radius):
    """Drawn wheel radius [m]: the argument, else ``plant.wheel_radius``, else the mesh."""
    if wheel_radius is None:
        wheel_radius = getattr(plant, "wheel_radius", None)

    return GEOMETRY["wheel_radius"] if wheel_radius is None else float(wheel_radius)


def mesh_primitive(filename, color, opacity=1.0, name=None, radial_scale=1.0, xyz=None):
    """An STL as a :class:`TriangleMesh` at its visual origin.

    The wheel meshes are smooth (no crease above 40 deg): their wireframe uses a
    15 deg crease angle, which keeps the two tire-shoulder rings. *radial_scale*
    scales a wheel about its axle (the link's z axis).
    """
    vertices, faces = stl_mesh(filename)
    if radial_scale != 1.0:
        vertices = vertices * np.array([radial_scale, radial_scale, 1.0])

    crease_deg = 15.0 if name in WHEEL_LINKS else 40.0
    mesh = TriangleMesh(
        vertices, faces, color=color, opacity=opacity, name=name, crease_deg=crease_deg
    )
    if xyz is not None:
        mesh.local_transform = origin(xyz)

    return mesh


@lru_cache(maxsize=None)
def stl_mesh(filename, scale=(1.0, 1.0, 1.0)):
    """``(vertices, faces)`` of a chassis / wheel / hinge STL, cached."""
    return load_stl(MESH_DIR / Path(filename).name, scale=np.asarray(scale))


# The car's dimensions, read once at import from the wheel mesh
GEOMETRY = racecar_geometry()


# Real-car detail: wheels


def outward_sign(link):
    """+1 when the link's outward direction is its local +z (right wheels), else -1."""
    return 1.0 if link.startswith("right") else -1.0


def wheel_details(link, colors, r):
    """Tread lugs, rim, spokes, beadlock ring and hub nut in the wheel frame.

    The wheel frame's z is the axle; the tire mesh spans ``s z in [0, width]``
    outward, with ``s = outward_sign(link)``, and has radius *r*. Everything turns
    with the wheel, so the spokes and the tread show the rolling angle. The lugs
    stand 0.5 mm proud of the tire, so they do not sink into the floor.
    """
    s = outward_sign(link)
    w = GEOMETRY["wheel_width"]

    def z(d0, d1):
        return s * d0, s * d1

    # The tread: two rows of n lugs, the second row staggered by half a pitch
    lugs = []
    n = 18
    for row, d in enumerate((0.26 * w, 0.74 * w)):
        for k in range(n):
            angle = 2.0 * np.pi * (k + 0.5 * row) / n
            lug = box_mesh((0.0045, 0.0105, 0.36 * w), center=(r - 0.0018, 0.0, s * d))
            lugs.append(transform_mesh(lug, SE3(Rz(angle), 0.0)))
    tread = merge_meshes(*lugs)

    # The rim and the beadlock ring, on the outer face
    rim = ring_mesh(0.0, 0.72 * r, *z(w - 0.002, w + 0.0015))
    ring = ring_mesh(0.71 * r, 0.83 * r, *z(w - 0.0005, w + 0.003))

    # Five split spokes and the hub, then the hub nut
    spokes = []
    for k in range(5):
        for offset in (-0.11, 0.11):  # Split spokes
            angle = 2.0 * np.pi * k / 5 + offset
            spoke = box_mesh(
                (0.52 * r, 0.0042, 0.003), center=(0.45 * r, 0.0, s * (w + 0.003))
            )
            spokes.append(transform_mesh(spoke, SE3(Rz(angle), 0.0)))
    spokes.append(cylinder_mesh(0.2 * r, *z(w + 0.0015, w + 0.0055), n=16))
    nut = cylinder_mesh(0.09 * r, *z(w + 0.0055, w + 0.0095), n=6)

    details = [
        TriangleMesh(*tread, color=colors["tire"], name=f"{link}_tread", max_edges=0),
        TriangleMesh(*rim, color=colors["rim"], name=f"{link}_rim"),
        TriangleMesh(*ring, color=colors["beadlock"], name=f"{link}_beadlock"),
        TriangleMesh(
            *merge_meshes(*spokes),
            color=colors["spokes"],
            name=f"{link}_spokes",
            max_edges=60,
        ),
        TriangleMesh(*nut, color=colors["hub"], name=f"{link}_hub"),
    ]

    return details


def steering_knuckle(link, colors):
    """Black steering knuckle (C-hub) and steering arm around the hinge."""
    s = -outward_sign(link)  # +1 on the left: outward is +y in chassis axes

    knuckle = Box(
        length_x=0.016, length_y=0.012, length_z=0.046, color=colors["knuckle"]
    )
    knuckle.local_transform = aligned() @ translation(0.0, s * 0.004, 0.0)
    arm = Box(length_x=0.028, length_y=0.006, length_z=0.005, color=colors["knuckle"])
    arm.local_transform = aligned() @ translation(-0.016, -s * 0.004, 0.016)

    return [knuckle, arm]


# Real-car detail: decks, electronics, lidar, camera


def rounded_rectangle(x0, x1, half_width, chamfer):
    """Convex octagon (chamfered rectangle) in local XY."""
    c = chamfer

    octagon = np.array(
        [
            [x0 + c, -half_width],
            [x1 - c, -half_width],
            [x1, -half_width + c],
            [x1, half_width - c],
            [x1 - c, half_width],
            [x0 + c, half_width],
            [x0, half_width - c],
            [x0, -half_width + c],
        ]
    )

    return octagon


# The decks in the chassis frame (z = 0 at the axles, ground at -axle_height): their
# heights, their thickness and their outlines, built once at import
LOWER_DECK_Z = 0.088
UPPER_DECK_Z = 0.134
DECK_THICKNESS = 0.004
LOWER_DECK = rounded_rectangle(-0.11, 0.41, 0.118, 0.035)
UPPER_DECK = rounded_rectangle(-0.105, 0.19, 0.078, 0.015)
ESTOP_XY = (0.30, 0.056)  # Emergency stop, front left of the lower deck

# Shocks (chassis frame): (x of the lower end, fore-aft lean of the top), rear pair
# 3 cm ahead of the rear axle, front pair 3 cm behind the front one. Lower ends
# (|y|, z) behind the tires; tops just outside the chassis plate (|y| <= 0.1,
# 0.02 <= z <= 0.08), so the shocks show between the tires and the lower deck as on
# the assembled vehicle. The top leans 12 mm fore / aft.
SHOCK_X_LEAN = ((0.03, 1.0), (0.305, -1.0))
SHOCK_BOTTOM_YZ = (0.1176, -0.012)
SHOCK_TOP_YZ = (0.108, 0.080)
REAR_BUMPER_X = (-0.074, -0.058, 0.065)  # Back face, front face, half width


def chassis_equipment(colors):
    """Acrylic decks, electronics, e-stop, shocks and bumpers (chassis frame)."""
    acrylic = colors["acrylic"]
    alpha = colors["acrylic_opacity"]
    parts = []

    # The two acrylic decks, and the four standoffs between them
    for outline, z in ((LOWER_DECK, LOWER_DECK_Z), (UPPER_DECK, UPPER_DECK_Z)):
        deck = ExtrudedPolygon(
            outline, height=DECK_THICKNESS, color=acrylic, opacity=alpha
        )
        deck.local_transform = translation(0.0, 0.0, z)
        parts.append(deck)
    z_low = LOWER_DECK_Z + 0.5 * DECK_THICKNESS
    z_up = UPPER_DECK_Z - 0.5 * DECK_THICKNESS
    for x in (-0.095, 0.178):
        for y in (-0.066, 0.066):
            parts.append(
                rod_between((x, y, z_low), (x, y, z_up), 0.003, colors["standoff"])
            )

    # Between the decks: 5 V power bank (front) and single-board computer (rear)
    parts.append(
        box_at((0.13, 0.068, 0.028), (0.105, 0.0, z_low + 0.014), colors["powerbank"])
    )
    parts.append(
        box_at((0.094, 0.066, 0.030), (-0.058, -0.004, z_low + 0.015), colors["rpi"])
    )

    # On the upper deck: Arduino Mega under its black interface shield
    z_top = UPPER_DECK_Z + 0.5 * DECK_THICKNESS
    parts.append(
        box_at(
            (0.1016, 0.0533, 0.0016), (-0.050, 0.0, z_top + 0.004), colors["arduino"]
        )
    )
    parts.append(
        box_at((0.098, 0.0533, 0.0016), (-0.050, 0.0, z_top + 0.016), colors["shield"])
    )
    for y in (-0.022, 0.022):
        parts.append(
            box_at((0.086, 0.0045, 0.010), (-0.050, y, z_top + 0.010), colors["shield"])
        )

    # Emergency stop, front left of the lower deck
    x, y = ESTOP_XY
    parts.append(
        box_at((0.028, 0.028, 0.020), (x, y, z_low + 0.010), colors["estop_base"])
    )
    parts.append(
        rod_between(
            (x, y, z_low + 0.020), (x, y, z_low + 0.031), 0.0115, colors["estop_collar"]
        )
    )
    parts.append(
        rod_between(
            (x, y, z_low + 0.031), (x, y, z_low + 0.043), 0.020, colors["estop"]
        )
    )
    parts.append(
        rod_between(
            (x, y, z_low + 0.043), (x, y, z_low + 0.046), 0.016, colors["estop"]
        )
    )

    # Shocks at the axles: the front pair just behind the front axle, the rear pair
    # just ahead of the rear axle, leaning in toward their towers; the lower ends
    # hide behind the wheels
    for x, lean in SHOCK_X_LEAN:
        for side in (-1.0, 1.0):
            bottom = np.array([x, side * SHOCK_BOTTOM_YZ[0], SHOCK_BOTTOM_YZ[1]])
            top = np.array([x + lean * 0.012, side * SHOCK_TOP_YZ[0], SHOCK_TOP_YZ[1]])
            parts.append(rod_between(bottom, top, 0.0065, colors["shock"]))
            parts.append(
                rod_between(
                    bottom + 0.2 * (top - bottom),
                    bottom + 0.76 * (top - bottom),
                    0.0095,
                    colors["spring"],
                )
            )

    # Traxxas front bumper: two tubular side loops joined by cross bars, each loop an
    # elliptic arc of centre (cx, cz) and semi-axes (ax, az)
    arc = np.radians(np.linspace(-110.0, 110.0, 13))
    cx = 0.40
    ax = 0.078
    cz = 0.045
    az = 0.05
    half_width = 0.065
    loop = [(cx + ax * np.cos(a), cz + az * np.sin(a)) for a in arc]
    for y in (-half_width, half_width):
        parts += polyline_rods([(x, y, z) for x, z in loop], 0.0055, colors["bumper"])
    for x, z in loop[3:10:3]:
        parts.append(
            rod_between(
                (x, -half_width, z), (x, half_width, z), 0.0055, colors["bumper"]
            )
        )

    # Slash rear bumper: a low, short bar under the back of the chassis, on two struts
    x0, x1, half_width = REAR_BUMPER_X
    parts.append(
        box_at(
            (x1 - x0, 2.0 * half_width, 0.014),
            (0.5 * (x0 + x1), 0.0, 0.002),
            colors["bumper"],
        )
    )
    for y in (-0.042, 0.042):
        parts.append(
            rod_between(
                (-0.032, y, 0.022), (x1 + 0.004, y, 0.006), 0.005, colors["bumper"]
            )
        )

    return parts


def box_at(size, center, color, opacity=1.0):
    """A box of full lengths *size*, centred at *center*."""
    box = Box(
        length_x=size[0],
        length_y=size[1],
        length_z=size[2],
        color=color,
        opacity=opacity,
    )
    box.local_transform = translation(*center)

    return box


def rod_between(p0, p1, radius, color, opacity=1.0):
    """A rod of *radius* from point *p0* to point *p1*."""
    length = float(np.linalg.norm(np.subtract(p1, p0)))
    rod = Rod(length=length, radius=radius, color=color, opacity=opacity)
    rod.local_transform = np.asarray(
        link_pose_3d(np.asarray(p0, float), np.asarray(p1, float))
    )

    return rod


def polyline_rods(points, radius, color):
    """One rod per segment of the polyline *points*."""
    rods = [
        rod_between(p0, p1, radius, color) for p0, p1 in zip(points[:-1], points[1:])
    ]

    return rods


def rplidar(colors):
    """RPLidar A2M8 (76 mm x 41 mm, black, red ring) with its scan plane at ``base_laser``.

    ``base_laser`` is yawed by pi; the window is turned to face the front of the car.
    """
    face_forward = SE3(Rz(np.pi), 0.0)

    # The base, the red ring, the body and the cap, then the window's frame and glass
    parts = [
        (cylinder_mesh(0.038, -0.029, -0.010, n=40), colors["lidar"]),
        (ring_mesh(0.0375, 0.0385, -0.0105, -0.0080, n=40), colors["lidar_ring"]),
        (cylinder_mesh(0.035, -0.0080, 0.0105, n=40), colors["lidar"]),
        (cylinder_mesh(0.031, 0.0105, 0.0125, n=40), colors["lidar"]),
        (
            box_mesh((0.004, 0.030, 0.010), center=(0.0335, 0.0, 0.0010)),
            colors["lidar_ring"],
        ),
        (
            box_mesh((0.005, 0.026, 0.007), center=(0.0340, 0.0, 0.0010)),
            colors["lidar_window"],
        ),
    ]

    meshes = []
    for mesh, color in parts:
        primitive = TriangleMesh(*mesh, color=color, name="rplidar", max_edges=48)
        primitive.local_transform = face_forward
        meshes.append(primitive)

    return meshes


def raspicam(colors):
    """RaspiCam on its acrylic bracket, at the camera box (``camera_link``)."""
    lx, ly, lz = CAMERA_BOX

    body = Box(length_x=0.5 * lx, length_y=ly, length_z=lz, color=colors["camera"])
    lens = rod_between(
        (0.25 * lx, 0.0, 0.0), (0.25 * lx + 0.012, 0.0, 0.0), 0.0065, colors["lens"]
    )
    glass = rod_between(
        (0.25 * lx + 0.012, 0.0, 0.0),
        (0.25 * lx + 0.0135, 0.0, 0.0),
        0.0045,
        colors["glass"],
    )
    bracket = box_at(
        (0.004, 0.034, 0.036),
        (-0.25 * lx - 0.002, 0.0, -0.024),
        colors["acrylic"],
        colors["acrylic_opacity"],
    )

    return [body, lens, glass, bracket]


def contact_shadow(colors):
    """Soft contact shadow on the ground (``base_footprint``, z = 0)."""
    shadow = ExtrudedPolygon(
        rounded_rectangle(-0.09, 0.43, 0.13, 0.05),
        height=0.0008,
        color=colors["shadow"],
        opacity=0.2,
    )
    shadow.local_transform = translation(0.0, 0.0, 0.0015)
    shadow.camera_fit = False

    return shadow


# Scene


def scene_from(plant, palette):
    """The scene a plant asks for, through its optional drawing-only attributes.

    ``scene_bounds`` ``(x_min, x_max, y_min, y_max)`` [m] is the one that switches the
    ground on; ``scene_walls``, ``scene_wall_height``, ``scene_tile`` and ``scene_path``
    are optional. A plant that sets none of them is drawn without a floor.
    """
    bounds = getattr(plant, "scene_bounds", None)
    if bounds is None:
        return []

    return track_scene(
        bounds=bounds,
        walls=getattr(plant, "scene_walls", False),
        wall_height=getattr(plant, "scene_wall_height", 0.3),
        tile=getattr(plant, "scene_tile", 0.5),
        path=getattr(plant, "scene_path", None),
        palette=palette,
    )


def floor_tiles(x0, x1, y0, y1, size):
    """Two checkerboard meshes of flat floor tiles (z = 0)."""
    xs = np.arange(x0, x1 + 1e-9, size)
    ys = np.arange(y0, y1 + 1e-9, size)

    # Each tile a quad of two triangles, sorted by the parity of its cell
    even = []
    odd = []
    for i, (xa, xb) in enumerate(zip(xs[:-1], xs[1:])):
        for j, (ya, yb) in enumerate(zip(ys[:-1], ys[1:])):
            quad = np.array(
                [[xa, ya, 0.0], [xb, ya, 0.0], [xb, yb, 0.0], [xa, yb, 0.0]]
            )
            mesh = (quad, np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32))
            (even if (i + j) % 2 == 0 else odd).append(mesh)

    even_tiles = merge_meshes(*even)
    odd_tiles = merge_meshes(*odd)

    return even_tiles, odd_tiles


def wall_panels(x0, x1, y, height, panel):
    """A vertical wall at *y* made of *panel*-wide quads (so views can crop it)."""
    xs = np.arange(x0, x1 + 1e-9, panel)

    quads = []
    for xa, xb in zip(xs[:-1], xs[1:]):
        quad = np.array([[xa, y, 0.0], [xb, y, 0.0], [xb, y, height], [xa, y, height]])
        quads.append((quad, np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)))
    wall = merge_meshes(*quads)

    return wall


def painted_line(path, colors, z=0.002):
    """The line the vehicle is asked to hold, painted flat on the floor.

    A loop given without its first point repeated (the usual way to sample a circuit,
    since a zero-length closing segment has no tangent) is closed again for drawing;
    an open path is left open. The test is the gap against the sample spacing.
    """
    xy = np.asarray(path, dtype=float)[:, :2]

    # Closed again when the end-to-start gap is at most 1.5 sample spacings
    gap = float(np.linalg.norm(xy[0] - xy[-1]))
    if gap <= 1.5 * float(np.max(np.linalg.norm(np.diff(xy, axis=0), axis=1))):
        xy = np.vstack([xy, xy[:1]])

    line = Line(
        np.column_stack([xy, np.full(len(xy), z)]),
        color=colors["deck_line"],
        linewidth=1.2,
        style="--",
    )

    return line


def scene_lines_from(plant, colors):
    """The same scene as :func:`scene_from`, drawn with lines for the top view."""
    bounds = getattr(plant, "scene_bounds", None)
    if bounds is None:
        return []

    return track_lines(
        bounds,
        walls=getattr(plant, "scene_walls", False),
        tile=getattr(plant, "scene_tile", 0.5),
        path=getattr(plant, "scene_path", None),
        colors=colors,
    )


# 2-D top view


def outline_xy(vertices, faces):
    """Top-view outline of a mesh: boundary loop of its upward faces, else convex hull."""
    vertices, faces = weld(vertices, faces)

    # The boundary of the upward faces: the edges only one of them uses
    up = faces[face_normals(vertices, faces)[:, 2] > 0.5]
    edges = np.sort(up[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    unique, count = np.unique(edges, axis=0, return_counts=True)
    boundary = unique[count == 1]

    # Each boundary vertex's neighbours along the boundary
    neighbours = {}
    for a, b in boundary:
        neighbours.setdefault(a, []).append(b)
        neighbours.setdefault(b, []).append(a)

    # A single loop (two neighbours each): walk it, and keep its corners
    if boundary.size and all(len(n) == 2 for n in neighbours.values()):
        start = boundary[0, 0]
        loop = [start]
        previous = None
        current = start
        while True:
            nxt = [n for n in neighbours[current] if n != previous][0]
            if nxt == start:
                break
            loop.append(nxt)
            previous, current = current, nxt
        if len(loop) == len(neighbours):
            xy = vertices[loop, :2]
            keep = np.ones(len(xy), dtype=bool)  # Drop collinear points
            for i in range(len(xy)):
                a, b, c = xy[i - 1], xy[i], xy[(i + 1) % len(xy)]
                keep[i] = abs(np.cross(b - a, c - b)) > 1e-9
            outline = xy[keep]

            return outline

    # Otherwise, the convex hull of the top view
    from scipy.spatial import ConvexHull

    hull = ConvexHull(vertices[:, :2])
    outline = vertices[hull.vertices, :2]

    return outline


def closed(xy, z=0.0):
    """The polygon *xy* as a closed 3-D polyline at height *z*."""
    xy = np.asarray(xy, dtype=float)

    polyline = np.column_stack([np.vstack([xy, xy[:1]]), np.full(len(xy) + 1, z)])

    return polyline


def hatch_lines(xy, spacing, holes=(), z=0.0):
    """Hatch filling an x-monotone polygon: scanlines at constant x, as polylines.

    Consecutive scanlines are joined into a serpentine (one line to draw); the
    discs ``holes = [(cx, cy, r), ...]`` are left empty, so filled circles drawn
    by the renderer (under the lines) stay clean.
    """
    xy = np.asarray(xy, dtype=float)
    edges = list(zip(xy, np.roll(xy, -1, axis=0)))
    polylines = []
    serpentine = []

    xs = np.arange(xy[:, 0].min() + 0.5 * spacing, xy[:, 0].max(), spacing)
    for k, x in enumerate(xs):
        # Where the scanline crosses the polygon's edges
        ys = sorted(
            p[1] + (x - p[0]) * (q[1] - p[1]) / (q[0] - p[0])
            for p, q in edges
            if (p[0] - x) * (q[0] - x) < 0.0
        )
        if len(ys) < 2:
            continue

        # The span inside the polygon, with the discs cut out of it
        pieces = [(ys[0], ys[-1])]
        for cx, cy, r in holes:
            if abs(x - cx) < r:
                h = np.sqrt(r**2 - (x - cx) ** 2)
                pieces = [
                    part
                    for y0, y1 in pieces
                    for part in ((y0, min(y1, cy - h)), (max(y0, cy + h), y1))
                    if part[1] - part[0] > 1e-4
                ]

        # One piece extends the serpentine, alternating its direction; several end it
        # and are drawn one by one
        if len(pieces) == 1:
            y0, y1 = pieces[0] if k % 2 == 0 else pieces[0][::-1]
            serpentine += [[x, y0, z], [x, y1, z]]
            continue
        if serpentine:
            polylines.append(np.asarray(serpentine))
            serpentine = []
        polylines += [np.array([[x, y0, z], [x, y1, z]]) for y0, y1 in pieces]

    if serpentine:
        polylines.append(np.asarray(serpentine))

    return polylines


def rectangle(x0, x1, y0, y1):
    """The four corners of an axis-aligned rectangle, counter-clockwise."""
    corners = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])

    return corners


def wheel_2d(center_y, colors, r):
    """Top view of a tire of radius *r*: outline plus a serpentine tread (x forward, y lateral)."""
    w = GEOMETRY["wheel_width"]

    outline = rectangle(-r, r, center_y - 0.5 * w, center_y + 0.5 * w)
    tread = [
        Line(line, color=colors["tire"], linewidth=1.1)
        for line in hatch_lines(outline, 0.009)
    ]
    wheel = [Line(closed(outline), color=colors["tire"], linewidth=1.6)] + tread

    return wheel
