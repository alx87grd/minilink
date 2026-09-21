"""UdeS-Racecar skins for minilink, built on the racecar URDF.

The geometry comes from ``racecar_description`` (URDF + meshes, shipped in
``minilink/graphical/assets/racecar_description``): link frames, joint origins and axes, visual
origins, meshes and materials. The colours and the parts the URDF leaves out
(acrylic decks, RPLidar A2M8, RaspiCam, emergency stop, single-board computer,
Arduino Mega, shocks, bumpers, rim and tread detail) were placed by eye from
published pictures of the assembled platform; they are decoration, not measurements.

Two pieces, following minilink's skin contract (DESIGN.md, ``graphical/catalog/skins.py``):

- :func:`racecar_frames` is the placement: pure forward kinematics of the URDF
  for a planar pose, a bicycle steering angle (split into Ackermann angles) and
  wheel rolling angles. A plant calls it from its ``tf``.
- :func:`racecar_skin_3d`, :func:`racecar_skin_2d` and :func:`urdf_skin` are the
  looks: ``(plant) -> dict[frame, list[primitive]]`` with no state math, keyed to
  the URDF link names that :func:`racecar_frames` returns.

Frame vocabulary (world poses, 4x4): ``body`` (the plant's reference point on the
ground, heading ``psi``), ``axle_front`` (front axle centre on the ground turned
by ``delta``, as in minilink's bicycles), and every URDF link:
``base_footprint``, ``base_link``, ``chassis``, ``left/right_rear_wheel``,
``left/right_steering_hinge``, ``left/right_front_wheel``, ``base_laser``,
``camera_link``, ``camera_optical_link``, ``imu_link``, ``chassis_inertia``.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, SE3, Rz, translation
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
    load_collada,
    load_stl,
    merge_meshes,
    ring_mesh,
    transform_mesh,
    weld,
)
from minilink.graphical.urdf import (
    forward_kinematics,
    load_urdf,
    origin_transform,
    resolve_mesh_path,
    rpy_matrix,
)

ASSETS = Path(__file__).resolve().parents[1] / "assets" / "racecar_description"
URDF_FILE = ASSETS / "urdf" / "racecar.xacro"
MESH_DIR = ASSETS / "meshes"

HINGE_JOINTS = ("left_steering_hinge_joint", "right_steering_hinge_joint")
WHEEL_LINKS = (
    "left_rear_wheel",
    "right_rear_wheel",
    "left_front_wheel",
    "right_front_wheel",
)


@lru_cache(maxsize=None)
def racecar_urdf():
    """The racecar URDF (xacro expanded without ROS), parsed once."""
    return load_urdf(URDF_FILE)


@lru_cache(maxsize=None)
def _mesh(filename, scale=(1.0, 1.0, 1.0)):
    """``(vertices, faces)`` of a URDF mesh file (STL), cached."""
    return load_stl(resolve_mesh_path(filename, MESH_DIR), scale=np.asarray(scale))


def racecar_geometry():
    """Dimensions read from the URDF and its meshes [m].

    ``wheelbase`` (rear axle to the steering hinges), ``kingpin_track`` (between
    the hinges), ``wheel_track`` (between the tire mid-planes), ``axle_height``
    (``base_footprint`` to ``base_link``), ``wheel_radius`` and ``wheel_width``
    (wheel mesh), ``lidar_height`` (scan plane above the ground).
    """
    urdf = racecar_urdf()
    joints = urdf["joints"]
    hinge = joints["left_steering_hinge_joint"]["xyz"]
    wheel = _mesh(
        urdf["links"]["left_rear_wheel"]["visuals"][0]["geometry"]["filename"]
    )[0]
    width = float(np.ptp(wheel[:, 2]))
    frames = forward_kinematics(urdf)
    return {
        "wheelbase": float(hinge[0]),
        "kingpin_track": float(2.0 * hinge[1]),
        "wheel_track": float(2.0 * hinge[1] + width),
        "axle_height": float(joints["base_footprint_link_joint"]["xyz"][2]),
        "wheel_radius": float(np.max(np.hypot(wheel[:, 0], wheel[:, 1]))),
        "wheel_width": width,
        "lidar_height": float(frames["base_laser"][2, 3]),
    }


GEOMETRY = racecar_geometry()


# Placement: pure kinematics, called from a plant's tf


def ackermann_angles(delta, wheelbase, track):
    """Left and right wheel steer angles of an ideal Ackermann linkage.

    Both front wheels turn about the same centre as the bicycle model with steer
    angle *delta* (positive = left turn): ``tan(delta_left) = L tan(delta) / (L -
    w/2 tan(delta))`` and ``tan(delta_right) = L tan(delta) / (L + w/2 tan(delta))``,
    with *wheelbase* ``L`` and *track* ``w`` between the steering axes. The inner
    wheel turns more.
    """
    xp = array_module(delta)
    t = xp.tan(delta)
    delta_left = xp.arctan2(wheelbase * t, wheelbase - 0.5 * track * t)
    delta_right = xp.arctan2(wheelbase * t, wheelbase + 0.5 * track * t)
    return delta_left, delta_right


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
        Moves the steering hinges to this distance from the rear axle, for a model
        whose wheelbase is not the URDF's 0.34 m (the default). The generic racecar
        plant of :mod:`minilink.dynamics.catalog.vehicles.racecar` uses the URDF value.
    wheel_radius : float, optional
        Raises ``base_link`` (the axles) to this height, for a look drawn at the same
        radius, so that a tread spinning at the model's wheel rate rolls on the ground
        without sliding. The URDF's 0.05 m by default, which is also what the generic
        plant rolls on.

    Returns
    -------
    dict[str, (4, 4) array]
        ``body``, ``axle_front`` and every URDF link (module docstring).
    """
    xp = array_module(X, Y, psi, delta, phi_rear)
    urdf = racecar_urdf()
    L = GEOMETRY["wheelbase"] if wheelbase is None else wheelbase
    phi_front = phi_rear if phi_front is None else phi_front
    delta_left, delta_right = ackermann_angles(delta, L, GEOMETRY["kingpin_track"])

    q = {
        "left_steering_hinge_joint": delta_left,
        "right_steering_hinge_joint": delta_right,
        "left_rear_wheel_joint": phi_rear,
        "right_rear_wheel_joint": phi_rear,
        "left_front_wheel_joint": phi_front,
        "right_front_wheel_joint": phi_front,
    }
    origins = {}
    if wheelbase is not None:
        origins = {
            name: xp.asarray([L, *urdf["joints"][name]["xyz"][1:]])
            for name in HINGE_JOINTS
        }
    if wheel_radius is not None:
        origins["base_footprint_link_joint"] = xp.asarray([0.0, 0.0, wheel_radius])

    T_body = SE2(X, Y, psi)
    T_footprint = T_body @ translation(-ref_to_rear_axle, 0.0, 0.0)
    frames = forward_kinematics(urdf, q, T_root=T_footprint, origins=origins)
    frames["body"] = T_body
    frames["axle_front"] = T_body @ SE2(L - ref_to_rear_axle, 0.0, delta)
    return frames


# Colours


def _rgb(rgba):
    return tuple(float(c) for c in np.asarray(rgba)[:3])


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


def urdf_palette():
    """Colours of the URDF materials (``materials.xacro``, rviz look), same keys."""
    urdf = racecar_urdf()
    materials = urdf["materials"]
    palette = dict(REAL_PALETTE)
    palette.update(
        chassis=_rgb(urdf["links"]["chassis"]["visuals"][0]["material"]),
        tire=_rgb(materials["Black"]),
        rim=_rgb(materials["Black"]),
        hinge=_rgb(materials["DarkGrey"]),
        camera=_rgb(urdf["links"]["camera_link"]["visuals"][0]["material"]),
    )
    return palette


def _palette(palette):
    if isinstance(palette, dict):
        return {**REAL_PALETTE, **palette}
    if palette == "urdf":
        return urdf_palette()
    if palette == "real":
        return dict(REAL_PALETTE)
    raise ValueError(f"palette must be 'real', 'urdf' or a dict, got {palette!r}")


# URDF visuals


def _joint_to(child):
    return next(j for j in racecar_urdf()["joints"].values() if j["child"] == child)


def _aligned(link):
    """Constant offset that re-expresses a link frame with its parent's axes.

    The steering hinge frames are rotated by the joint ``rpy`` (0, pi/2, 0);
    geometry authored in chassis-like axes (x forward, y left, z up) at the hinge
    uses ``local_transform = _aligned(link) @ T``.
    """
    return SE3(rpy_matrix(_joint_to(link)["rpy"]).T, 0.0)


def _mesh_primitive(visual, color, opacity=1.0, name=None, radial_scale=1.0):
    """A URDF ``<mesh>`` visual as a :class:`TriangleMesh` at its visual origin.

    The wheel meshes are smooth (no crease above 40 deg): their wireframe uses a
    15 deg crease angle, which keeps the two tire-shoulder rings. *radial_scale*
    scales a wheel about its axle (the link's z axis).
    """
    geometry = visual["geometry"]
    vertices, faces = _mesh(geometry["filename"], tuple(geometry["scale"]))
    if radial_scale != 1.0:
        vertices = vertices * np.array([radial_scale, radial_scale, 1.0])
    crease_deg = 15.0 if name in WHEEL_LINKS else 40.0
    mesh = TriangleMesh(
        vertices, faces, color=color, opacity=opacity, name=name, crease_deg=crease_deg
    )
    mesh.local_transform = origin_transform(visual["xyz"], visual["rpy"])
    return mesh


def _hokuyo(visual=None):
    """The URDF laser (``hokuyo.dae``), one mesh per COLLADA material."""
    visual = visual or racecar_urdf()["links"]["base_laser"]["visuals"][0]
    path = resolve_mesh_path(visual["geometry"]["filename"], MESH_DIR)
    parts = []
    for vertices, faces, rgba in load_collada(path, scale=visual["geometry"]["scale"]):
        mesh = TriangleMesh(vertices, faces, color=_rgb(rgba), name="base_laser")
        mesh.local_transform = origin_transform(visual["xyz"], visual["rpy"])
        parts.append(mesh)
    return parts


def urdf_skin(plant=None, palette="urdf"):
    """The URDF visuals alone, as rviz shows them: every link's ``<visual>``.

    Chassis, wheel and hinge meshes, the Hokuyo mesh and the camera box at their
    URDF visual origins with the URDF materials (``palette="real"`` recolours
    them). Keyed to the link frames of :func:`racecar_frames`.
    """
    colors = _palette(palette)
    role = {
        "chassis": "chassis",
        "left_steering_hinge": "hinge",
        "right_steering_hinge": "hinge",
    }
    geometry = {}
    for link, data in racecar_urdf()["links"].items():
        for visual in data["visuals"]:
            kind = visual["geometry"]["type"]
            if link == "base_laser":
                parts = _hokuyo(visual)
            elif kind == "mesh":
                color = colors[role.get(link, "tire")]
                parts = [_mesh_primitive(visual, color, name=link)]
            elif kind == "box":
                lx, ly, lz = visual["geometry"]["size"]
                box = Box(length_x=lx, length_y=ly, length_z=lz, color=colors["camera"])
                box.local_transform = origin_transform(visual["xyz"], visual["rpy"])
                parts = [box]
            else:
                continue
            geometry.setdefault(link, []).extend(parts)
    return geometry


# Real-car detail: wheels


def _side(link):
    """+1 when the link's outward direction is its local +z (right wheels), else -1."""
    return 1.0 if link.startswith("right") else -1.0


def _wheel_details(link, colors, r):
    """Tread lugs, rim, spokes, beadlock ring and hub nut in the wheel frame.

    The wheel frame's z is the axle; the URDF tire spans ``s z in [0, width]``
    outward, with ``s = _side(link)``, and has radius *r*. Everything turns with
    the wheel, so the spokes and the tread show the rolling angle. The lugs
    stand 0.5 mm proud of the tire, so they do not sink into the floor.
    """
    s = _side(link)
    w = GEOMETRY["wheel_width"]

    def z(d0, d1):
        return s * d0, s * d1

    lugs = []
    n = 18
    for row, d in enumerate((0.26 * w, 0.74 * w)):
        for k in range(n):
            angle = 2.0 * np.pi * (k + 0.5 * row) / n
            lug = box_mesh((0.0045, 0.0105, 0.36 * w), center=(r - 0.0018, 0.0, s * d))
            lugs.append(transform_mesh(lug, SE3(Rz(angle), 0.0)))
    tread = merge_meshes(*lugs)

    rim = ring_mesh(0.0, 0.72 * r, *z(w - 0.002, w + 0.0015))
    ring = ring_mesh(0.71 * r, 0.83 * r, *z(w - 0.0005, w + 0.003))
    spokes = []
    for k in range(5):
        for offset in (-0.11, 0.11):  # split spokes
            angle = 2.0 * np.pi * k / 5 + offset
            spoke = box_mesh(
                (0.52 * r, 0.0042, 0.003), center=(0.45 * r, 0.0, s * (w + 0.003))
            )
            spokes.append(transform_mesh(spoke, SE3(Rz(angle), 0.0)))
    spokes.append(cylinder_mesh(0.2 * r, *z(w + 0.0015, w + 0.0055), n=16))
    nut = cylinder_mesh(0.09 * r, *z(w + 0.0055, w + 0.0095), n=6)

    return [
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


def _knuckle(link, colors):
    """Black steering knuckle (C-hub) and steering arm around the hinge."""
    s = -_side(link)  # +1 on the left: outward is +y in chassis axes
    knuckle = Box(
        length_x=0.016, length_y=0.012, length_z=0.046, color=colors["knuckle"]
    )
    knuckle.local_transform = _aligned(link) @ translation(0.0, s * 0.004, 0.0)
    arm = Box(length_x=0.028, length_y=0.006, length_z=0.005, color=colors["knuckle"])
    arm.local_transform = _aligned(link) @ translation(-0.016, -s * 0.004, 0.016)
    return [knuckle, arm]


# Real-car detail: decks, electronics, lidar, camera


def _rounded_rectangle(x0, x1, half_width, chamfer):
    """Convex octagon (chamfered rectangle) in local XY."""
    c = chamfer
    return np.array(
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


# Deck heights in the chassis frame (z = 0 at the axles, ground at -axle_height).
LOWER_DECK_Z = 0.088
UPPER_DECK_Z = 0.134
DECK_THICKNESS = 0.004
LOWER_DECK = _rounded_rectangle(-0.11, 0.41, 0.118, 0.035)
UPPER_DECK = _rounded_rectangle(-0.105, 0.19, 0.078, 0.015)
ESTOP_XY = (0.30, 0.056)  # emergency stop, front left of the lower deck
# shocks (chassis frame): (x of the lower end, fore-aft lean of the top), rear pair
# 3 cm ahead of the rear axle, front pair 3 cm behind the front one. Lower ends
# (|y|, z) behind the tires; tops just outside the URDF chassis block (|y| <= 0.1,
# 0.02 <= z <= 0.08), so the shocks show between the tires and the lower deck as on
# the assembled vehicle. The top leans 12 mm fore / aft.
SHOCK_X_LEAN = ((0.03, 1.0), (0.305, -1.0))
SHOCK_BOTTOM_YZ = (0.1176, -0.012)
SHOCK_TOP_YZ = (0.108, 0.080)
REAR_BUMPER_X = (-0.074, -0.058, 0.065)  # back face, front face, half width


def _box(size, center, color, opacity=1.0):
    box = Box(
        length_x=size[0],
        length_y=size[1],
        length_z=size[2],
        color=color,
        opacity=opacity,
    )
    box.local_transform = translation(*center)
    return box


def _rod(p0, p1, radius, color, opacity=1.0):
    length = float(np.linalg.norm(np.subtract(p1, p0)))
    rod = Rod(length=length, radius=radius, color=color, opacity=opacity)
    rod.local_transform = np.asarray(
        link_pose_3d(np.asarray(p0, float), np.asarray(p1, float))
    )
    return rod


def _polyline_rods(points, radius, color):
    return [_rod(p0, p1, radius, color) for p0, p1 in zip(points[:-1], points[1:])]


def _chassis_equipment(colors):
    """Acrylic decks, electronics, e-stop, shocks and bumpers (chassis frame)."""
    acrylic, alpha = colors["acrylic"], colors["acrylic_opacity"]
    parts = []

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
            parts.append(_rod((x, y, z_low), (x, y, z_up), 0.003, colors["standoff"]))

    # between the decks: 5 V power bank (front) and single-board computer (rear)
    parts.append(
        _box((0.13, 0.068, 0.028), (0.105, 0.0, z_low + 0.014), colors["powerbank"])
    )
    parts.append(
        _box((0.094, 0.066, 0.030), (-0.058, -0.004, z_low + 0.015), colors["rpi"])
    )
    # on the upper deck: Arduino Mega under its black interface shield
    z_top = UPPER_DECK_Z + 0.5 * DECK_THICKNESS
    parts.append(
        _box((0.1016, 0.0533, 0.0016), (-0.050, 0.0, z_top + 0.004), colors["arduino"])
    )
    parts.append(
        _box((0.098, 0.0533, 0.0016), (-0.050, 0.0, z_top + 0.016), colors["shield"])
    )
    for y in (-0.022, 0.022):
        parts.append(
            _box((0.086, 0.0045, 0.010), (-0.050, y, z_top + 0.010), colors["shield"])
        )

    # emergency stop, front left of the lower deck
    x, y = ESTOP_XY
    parts.append(
        _box((0.028, 0.028, 0.020), (x, y, z_low + 0.010), colors["estop_base"])
    )
    parts.append(
        _rod(
            (x, y, z_low + 0.020), (x, y, z_low + 0.031), 0.0115, colors["estop_collar"]
        )
    )
    parts.append(
        _rod((x, y, z_low + 0.031), (x, y, z_low + 0.043), 0.020, colors["estop"])
    )
    parts.append(
        _rod((x, y, z_low + 0.043), (x, y, z_low + 0.046), 0.016, colors["estop"])
    )

    # shocks at the axles: the front pair just behind the
    # front axle, the rear pair just ahead of the rear axle, leaning in toward
    # their towers; the lower ends hide behind the wheels
    for x, lean in SHOCK_X_LEAN:
        for side in (-1.0, 1.0):
            bottom = np.array([x, side * SHOCK_BOTTOM_YZ[0], SHOCK_BOTTOM_YZ[1]])
            top = np.array([x + lean * 0.012, side * SHOCK_TOP_YZ[0], SHOCK_TOP_YZ[1]])
            parts.append(_rod(bottom, top, 0.0065, colors["shock"]))
            parts.append(
                _rod(
                    bottom + 0.2 * (top - bottom),
                    bottom + 0.76 * (top - bottom),
                    0.0095,
                    colors["spring"],
                )
            )

    # Traxxas front bumper: two tubular side loops joined by cross bars
    arc = np.radians(np.linspace(-110.0, 110.0, 13))
    cx, ax, cz, az, half_width = 0.40, 0.078, 0.045, 0.05, 0.065
    loop = [(cx + ax * np.cos(a), cz + az * np.sin(a)) for a in arc]
    for y in (-half_width, half_width):
        parts += _polyline_rods([(x, y, z) for x, z in loop], 0.0055, colors["bumper"])
    for x, z in loop[3:10:3]:
        parts.append(
            _rod((x, -half_width, z), (x, half_width, z), 0.0055, colors["bumper"])
        )

    # Slash rear bumper: a low, short bar under the back of the chassis, on two struts
    x0, x1, half_width = REAR_BUMPER_X
    parts.append(
        _box(
            (x1 - x0, 2.0 * half_width, 0.014),
            (0.5 * (x0 + x1), 0.0, 0.002),
            colors["bumper"],
        )
    )
    for y in (-0.042, 0.042):
        parts.append(
            _rod((-0.032, y, 0.022), (x1 + 0.004, y, 0.006), 0.005, colors["bumper"])
        )
    return parts


def _rplidar(colors):
    """RPLidar A2M8 (76 mm x 41 mm, black, red ring) with its scan plane at ``base_laser``.

    ``base_laser`` is yawed by pi in the URDF; the window is turned to face the
    front of the car.
    """
    face_forward = SE3(Rz(np.pi), 0.0)
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


def _raspicam(colors):
    """RaspiCam on its acrylic bracket, at the URDF camera box (``camera_link``)."""
    lx, ly, lz = racecar_urdf()["links"]["camera_link"]["visuals"][0]["geometry"][
        "size"
    ]
    body = Box(length_x=0.5 * lx, length_y=ly, length_z=lz, color=colors["camera"])
    lens = _rod(
        (0.25 * lx, 0.0, 0.0), (0.25 * lx + 0.012, 0.0, 0.0), 0.0065, colors["lens"]
    )
    glass = _rod(
        (0.25 * lx + 0.012, 0.0, 0.0),
        (0.25 * lx + 0.0135, 0.0, 0.0),
        0.0045,
        colors["glass"],
    )
    bracket = _box(
        (0.004, 0.034, 0.036),
        (-0.25 * lx - 0.002, 0.0, -0.024),
        colors["acrylic"],
        colors["acrylic_opacity"],
    )
    return [body, lens, glass, bracket]


def _shadow(colors):
    """Soft contact shadow on the ground (``base_footprint``, z = 0)."""
    shadow = ExtrudedPolygon(
        _rounded_rectangle(-0.09, 0.43, 0.13, 0.05),
        height=0.0008,
        color=colors["shadow"],
        opacity=0.2,
    )
    shadow.local_transform = translation(0.0, 0.0, 0.0015)
    shadow.camera_fit = False
    return shadow


# Scene: the ground the car drives on


def _tiles(x0, x1, y0, y1, size):
    """Two checkerboard meshes of flat floor tiles (z = 0)."""
    xs = np.arange(x0, x1 + 1e-9, size)
    ys = np.arange(y0, y1 + 1e-9, size)
    even, odd = [], []
    for i, (xa, xb) in enumerate(zip(xs[:-1], xs[1:])):
        for j, (ya, yb) in enumerate(zip(ys[:-1], ys[1:])):
            quad = np.array(
                [[xa, ya, 0.0], [xb, ya, 0.0], [xb, yb, 0.0], [xa, yb, 0.0]]
            )
            mesh = (quad, np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32))
            (even if (i + j) % 2 == 0 else odd).append(mesh)
    return merge_meshes(*even), merge_meshes(*odd)


def _wall(x0, x1, y, height, panel):
    """A vertical wall at *y* made of *panel*-wide quads (so views can crop it)."""
    xs = np.arange(x0, x1 + 1e-9, panel)
    quads = []
    for xa, xb in zip(xs[:-1], xs[1:]):
        quad = np.array([[xa, y, 0.0], [xb, y, 0.0], [xb, y, height], [xa, y, height]])
        quads.append((quad, np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)))
    return merge_meshes(*quads)


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
    palette : {"real", "urdf"} or dict
        Same colours as the vehicle looks.

    Returns
    -------
    list
        Primitives for the ``world`` key. Backdrops only (``camera_fit = False``),
        like minilink's ground planes, so they never enlarge an auto-fit view.
    """
    colors = _palette(palette)
    x_min, x_max, y_min, y_max = (float(v) for v in bounds)
    floor_a, floor_b = _tiles(x_min, x_max, y_min, y_max, tile)
    parts = [
        TriangleMesh(*floor_a, color=colors["floor"], name="floor", max_edges=0),
        TriangleMesh(*floor_b, color=colors["floor_alt"], name="floor", max_edges=0),
    ]
    if walls:
        for y in (y_min, y_max):
            wall = _wall(x_min, x_max, y, wall_height, 0.5 * tile)
            parts.append(
                TriangleMesh(
                    *wall,
                    color=colors["wall"],
                    opacity=colors["wall_opacity"],
                    name="wall",
                    max_edges=0,
                )
            )
    if path is not None:
        parts.append(_painted_line(path, colors))
    for primitive in parts:
        primitive.camera_fit = False
    return parts


def _painted_line(path, colors, z=0.002):
    """The line the vehicle is asked to hold, painted flat on the floor.

    A loop given without its first point repeated (the usual way to sample a circuit,
    since a zero-length closing segment has no tangent) is closed again for drawing;
    an open path is left open. The test is the gap against the sample spacing.
    """
    xy = np.asarray(path, dtype=float)[:, :2]
    gap = float(np.linalg.norm(xy[0] - xy[-1]))
    if gap <= 1.5 * float(np.max(np.linalg.norm(np.diff(xy, axis=0), axis=1))):
        xy = np.vstack([xy, xy[:1]])
    return Line(
        np.column_stack([xy, np.full(len(xy), z)]),
        color=colors["deck_line"],
        linewidth=1.2,
        style="--",
    )


def _scene_from(plant, palette):
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


# The skins


def _wheel_radius(plant, wheel_radius):
    """Drawn wheel radius [m]: the argument, else ``plant.wheel_radius``, else the URDF's."""
    if wheel_radius is None:
        wheel_radius = getattr(plant, "wheel_radius", None)
    return GEOMETRY["wheel_radius"] if wheel_radius is None else float(wheel_radius)


def racecar_skin_3d(
    plant=None,
    palette="real",
    lidar="rplidar",
    equipment=True,
    scene=True,
    wheel_radius=None,
):
    """The racecar in 3-D: the URDF meshes plus the equipment of the built vehicle.

    Parameters
    ----------
    plant : System, optional
        Read for the scene (``scene_bounds`` and friends, :func:`_scene_from`) and
        for ``wheel_radius`` [m] when it has one.
    palette : {"real", "urdf"} or dict
        Colours of the assembled vehicle or of the URDF materials; a dict
        overrides single entries of :data:`REAL_PALETTE`.
    lidar : {"rplidar", "hokuyo"}
        The RPLidar A2M8 of the built vehicle, or the URDF's Hokuyo mesh.
    equipment : bool
        Decks, electronics, e-stop, camera bracket, shocks, bumpers, wheel detail.
    scene : bool
        The ground the plant asks for (``world``) and the contact shadow.
    wheel_radius : float, optional
        Draws the tires at this radius (scaled about the axle), so that a wheel
        spinning at the model's rate rolls without sliding on the floor.
        ``plant.wheel_radius`` when omitted, else the URDF's 0.05 m.
        :func:`racecar_frames` must get the same ``wheel_radius`` so the tires
        touch the ground.

    Returns
    -------
    dict[str, list]
        Primitives keyed to the frames of :func:`racecar_frames`.
    """
    colors = _palette(palette)
    urdf = racecar_urdf()["links"]
    r = _wheel_radius(plant, wheel_radius)
    geometry = {
        "chassis": [
            _mesh_primitive(
                urdf["chassis"]["visuals"][0], colors["chassis"], name="chassis"
            )
        ],
        "base_laser": _rplidar(colors) if lidar == "rplidar" else _hokuyo(),
    }
    for link in WHEEL_LINKS:
        tire = _mesh_primitive(
            urdf[link]["visuals"][0],
            colors["tire"],
            name=link,
            radial_scale=r / GEOMETRY["wheel_radius"],
        )
        details = _wheel_details(link, colors, r) if equipment else []
        geometry[link] = [tire] + details
    for link in ("left_steering_hinge", "right_steering_hinge"):
        hinge = _mesh_primitive(urdf[link]["visuals"][0], colors["hinge"], name=link)
        geometry[link] = [hinge] + (_knuckle(link, colors) if equipment else [])

    if equipment:
        geometry["chassis"] += _chassis_equipment(colors)
        geometry["camera_link"] = _raspicam(colors)
    else:
        lx, ly, lz = urdf["camera_link"]["visuals"][0]["geometry"]["size"]
        geometry["camera_link"] = [
            Box(length_x=lx, length_y=ly, length_z=lz, color=colors["camera"])
        ]

    if scene:
        geometry["base_footprint"] = [_shadow(colors)]
        world = _scene_from(plant, palette)
        if world:
            geometry["world"] = world
    return geometry


# 2-D top view


def _outline_xy(vertices, faces):
    """Top-view outline of a mesh: boundary loop of its upward faces, else convex hull."""
    vertices, faces = weld(vertices, faces)
    up = faces[face_normals(vertices, faces)[:, 2] > 0.5]
    edges = np.sort(up[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    unique, count = np.unique(edges, axis=0, return_counts=True)
    boundary = unique[count == 1]
    neighbours = {}
    for a, b in boundary:
        neighbours.setdefault(a, []).append(b)
        neighbours.setdefault(b, []).append(a)
    if boundary.size and all(len(n) == 2 for n in neighbours.values()):
        start = boundary[0, 0]
        loop, previous, current = [start], None, start
        while True:
            nxt = [n for n in neighbours[current] if n != previous][0]
            if nxt == start:
                break
            loop.append(nxt)
            previous, current = current, nxt
        if len(loop) == len(neighbours):
            xy = vertices[loop, :2]
            keep = np.ones(len(xy), dtype=bool)  # drop collinear points
            for i in range(len(xy)):
                a, b, c = xy[i - 1], xy[i], xy[(i + 1) % len(xy)]
                keep[i] = abs(np.cross(b - a, c - b)) > 1e-9
            return xy[keep]
    from scipy.spatial import ConvexHull

    hull = ConvexHull(vertices[:, :2])
    return vertices[hull.vertices, :2]


def _closed(xy, z=0.0):
    xy = np.asarray(xy, dtype=float)
    return np.column_stack([np.vstack([xy, xy[:1]]), np.full(len(xy) + 1, z)])


def _hatch(xy, spacing, holes=(), z=0.0):
    """Hatch filling an x-monotone polygon: scanlines at constant x, as polylines.

    Consecutive scanlines are joined into a serpentine (one line to draw); the
    discs ``holes = [(cx, cy, r), ...]`` are left empty, so filled circles drawn
    by the renderer (under the lines) stay clean.
    """
    xy = np.asarray(xy, dtype=float)
    edges = list(zip(xy, np.roll(xy, -1, axis=0)))
    polylines, serpentine = [], []
    xs = np.arange(xy[:, 0].min() + 0.5 * spacing, xy[:, 0].max(), spacing)
    for k, x in enumerate(xs):
        ys = sorted(
            p[1] + (x - p[0]) * (q[1] - p[1]) / (q[0] - p[0])
            for p, q in edges
            if (p[0] - x) * (q[0] - x) < 0.0
        )
        if len(ys) < 2:
            continue
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


def _rectangle(x0, x1, y0, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])


def _wheel_2d(center_y, colors, r):
    """Top view of a tire of radius *r*: outline plus a serpentine tread (x forward, y lateral)."""
    w = GEOMETRY["wheel_width"]
    outline = _rectangle(-r, r, center_y - 0.5 * w, center_y + 0.5 * w)
    tread = [
        Line(line, color=colors["tire"], linewidth=1.1)
        for line in _hatch(outline, 0.009)
    ]
    return [Line(_closed(outline), color=colors["tire"], linewidth=1.6)] + tread


def racecar_skin_2d(
    plant=None,
    palette="real",
    equipment=True,
    scene=True,
    heading_arrow=False,
    wheel_radius=None,
):
    """Top view of the racecar for fast matplotlib animations.

    The chassis outline is the top face of the URDF chassis mesh; the four tires
    sit at the URDF wheel joints (front ones on the steered hinges, Ackermann);
    the RPLidar is a filled disk with its red ring; the decks, electronics,
    e-stop and bumpers follow the assembled vehicle. Lines and circles only, so
    minilink's stock 2-D matplotlib renderer draws it fast — this is the look to
    animate with, since a GIF of the 3-D look comes out as a wireframe top view.

    Parameters are those of :func:`racecar_skin_3d`, plus *heading_arrow*
    (an arrow on ``body``).
    """
    colors = _palette(palette)
    chassis_visual = racecar_urdf()["links"]["chassis"]["visuals"][0]
    vertices, faces = _mesh(chassis_visual["geometry"]["filename"])
    outline = _outline_xy(
        *transform_mesh(
            (vertices, faces),
            origin_transform(chassis_visual["xyz"], chassis_visual["rpy"]),
        )
    )
    half_track = 0.5 * GEOMETRY["wheel_track"]
    r = _wheel_radius(plant, wheel_radius)

    lidar_x = forward_kinematics(racecar_urdf())["base_laser"][0, 3]
    holes = [(lidar_x, 0.0, 0.041), (ESTOP_XY[0], ESTOP_XY[1], 0.022)]
    chassis = [
        Line(hatch, color="#9a9ca1", linewidth=0.6)
        for hatch in _hatch(outline, 0.012, holes)
    ]
    chassis.append(Line(_closed(outline), color=colors["chassis"], linewidth=1.8))
    for y in (-half_track, half_track):
        chassis += _wheel_2d(y, colors, r)

    geometry = {"chassis": chassis}
    for link in ("left_steering_hinge", "right_steering_hinge"):
        s = -_side(link)
        parts = _wheel_2d(s * 0.5 * GEOMETRY["wheel_width"], colors, r)
        for part in parts:
            part.local_transform = _aligned(link)
        geometry[link] = parts

    if equipment:
        geometry["chassis"] += [
            Line(_closed(LOWER_DECK), color=colors["deck_line"], linewidth=1.0),
            Line(_closed(UPPER_DECK), color=colors["deck_line"], linewidth=1.0),
            Line(
                _closed(_rectangle(-0.050 - 0.0508, -0.050 + 0.0508, -0.0267, 0.0267)),
                color=colors["shield"],
                linewidth=1.2,
            ),
            Line(
                _closed(_rectangle(0.04, 0.17, -0.034, 0.034)),
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
                _closed(
                    _rectangle(*REAR_BUMPER_X[:2], -REAR_BUMPER_X[2], REAR_BUMPER_X[2])
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
                _closed(_rectangle(-0.008, 0.008, -0.015, 0.015)),
                color=colors["camera"],
                linewidth=1.4,
            ),
            Line(
                np.array([[0.008, 0.0, 0.0], [0.022, 0.0, 0.0]]),
                color=colors["lens"],
                linewidth=3.0,
            ),
        ]
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

    if scene:
        world = _scene_lines_from(plant, colors)
        if world:
            geometry["world"] = world
    return geometry


def _scene_lines_from(plant, colors):
    """The same scene as :func:`_scene_from`, drawn with lines for the top view."""
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


def track_lines(bounds, walls=False, tile=0.5, path=None, colors=None):
    """Top view of :func:`track_scene`: tile joints, walls and the painted line.

    One serpentine polyline for the joints, so the stock 2-D renderer draws the whole
    floor grid in a single artist.
    """
    colors = REAL_PALETTE if colors is None else colors
    x_min, x_max, y_min, y_max = (float(v) for v in bounds)
    joints = []
    for k, x in enumerate(np.arange(x_min, x_max + 1e-9, tile)):
        ya, yb = (y_min, y_max) if k % 2 == 0 else (y_max, y_min)
        joints += [[x, ya, 0.0], [x, yb, 0.0]]
    lines = [Line(np.array(joints), color="#d9d9d9", linewidth=0.7)]
    if path is not None:
        lines.append(_painted_line(path, colors))
    if walls:
        for y in (y_min, y_max):
            lines.append(
                Line(
                    np.array([[x_min, y, 0.0], [x_max, y, 0.0]]),
                    color="#6d6d6d",
                    linewidth=3.0,
                )
            )
    for line in lines:
        line.camera_fit = False
    return lines


def frame_axes_skin(plant=None, length=0.04, links=None):
    """Debug look: an RGB triad (x red, y green, z blue) at every URDF link frame.

    Stack it on a look with ``merge_skins(urdf_skin, frame_axes_skin)`` to check
    that meshes sit in their joint frames.
    """
    geometry = {}
    for link in links or racecar_urdf()["links"]:
        geometry[link] = [
            Line(np.array([[0.0, 0.0, 0.0], axis]), color=color, linewidth=2.0)
            for axis, color in zip(
                length * np.eye(3), ("#d62728", "#2ca02c", "#1f77b4")
            )
        ]
    return geometry
