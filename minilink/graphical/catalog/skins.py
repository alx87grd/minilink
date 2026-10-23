"""Skins: a plant's swappable look, as primitives keyed to the frames of its ``tf``."""

import numpy as np

from minilink.core.kinematics import translation
from minilink.graphical.catalog.shapes import (
    Box,
    Line,
    Plane,
    Rod,
    Sphere,
    link_pose_3d,
    plane_airframe_3d,
)

###############################################################################
# Public API
###############################################################################


def car_skin_2d(plant, color="#1a1a1a"):
    """2-D centerline car look: chassis line + two axle wheels.

    A skin is a plant's swappable look, ``(plant) -> dict[frame, list[primitive]]``:
    point geometry keyed to the frames of the plant's ``tf``, with no state math
    (``tf`` places every frame). A plant opts in with ``skin = car_skin_2d`` and
    swaps its look with one assignment (``car.skin = car_skin_3d``), keeping the
    same ``f`` and the same ``tf``. Skins are plain functions, so a custom skin is
    just another ``(plant) -> dict``.

    Frame keys: ``body`` (chassis along the wheelbase plus the rear wheel outline;
    the rear axle offset is baked into the wheel ``local_transform``), and
    ``axle_front`` (steered front wheel). The placing ``tf`` supplies world pose
    and the front steer angle.
    """
    # The axle offsets, and the wheel outline's length and width
    a, b = axle_offsets(plant)
    wl = getattr(plant, "wheel_len", 0.6)
    ww = getattr(plant, "wheel_width", 0.2)

    # The chassis, from the rear axle at x = −b to the front axle at x = a
    chassis_points = np.array([[-b, 0.0, 0.0], [a, 0.0, 0.0]])
    chassis = Line(chassis_points, color=color, linewidth=2.5)

    # The rear wheel, fixed on the rear axle in the body frame
    rear_wheel = wheel_rectangle(wl, ww)
    rear_wheel.local_transform = translation(-b, 0.0, 0.0)

    # The front wheel, placed and steered by its own frame
    front_wheel = wheel_rectangle(wl, ww)

    return {
        "body": [chassis, rear_wheel],
        "axle_front": [front_wheel],
    }


def car_skin_3d(plant, color="#151922"):
    """3-D four-wheel car look: ground plane, body box, four wheel rods.

    Frame keys: ``world`` (ground), ``body`` (chassis box plus the two rear wheel
    rods; fixed hub offsets are baked into each rod ``local_transform``), and
    ``wheel_fl`` / ``wheel_fr`` (steered front rods placed by ``tf``).
    """
    # The axle offsets and the tire radii: the params', else the plant's, else 0.3 m
    a, b = axle_offsets(plant)
    r_f = plant.params.get("r_f", getattr(plant, "r_f", 0.3))
    r_r = plant.params.get("r_r", getattr(plant, "r_r", 0.3))

    # The drawing attributes, with the defaults of a bare plant
    track = getattr(plant, "track", 1.6)
    body_height = getattr(plant, "body_height", 0.22)
    body_width_ratio = getattr(plant, "body_width_ratio", 0.72)
    body_length_overhang = getattr(plant, "body_length_overhang", 0.26)
    body_ground_clearance = getattr(plant, "body_ground_clearance", 0.003)
    ground_size = getattr(plant, "ground_plane_size", 120.0)
    tire_radius_ratio = getattr(plant, "_visual_tire_radius_ratio", 0.58)
    wheel_width = getattr(plant, "_visual_wheel_width", 0.2)

    # The body spans the wheelbase plus an overhang; the drawn tire is a fraction
    # of the smaller tire radius, never under 4.5 cm
    body_width = body_width_ratio * track
    body_length = (a + b) + body_length_overhang
    tire_radius = max(0.045, tire_radius_ratio * min(r_f, r_r))

    # The ground, in the world frame
    ground = ground_plane(ground_size)

    # The body box, at a fixed offset within the body frame (the c.g. sits at its
    # origin): lifted to ride height and shifted to the wheelbase's geometric center
    body = Box(
        length_x=body_length,
        length_y=body_width,
        length_z=body_height,
        center=(0.0, 0.0, 0.0),
        color=color,
        opacity=1.0,
    )
    cx_body = 0.5 * (a - b)
    z_body = r_r + body_ground_clearance + 0.5 * body_height
    body.local_transform = translation(cx_body, 0.0, z_body)

    # Every wheel: a dark cylinder of the drawn tire radius, as long as the wheel is wide
    def wheel():
        return Rod(length=wheel_width, radius=tire_radius, color="#0a0a0a", opacity=1.0)

    # The rear wheels, fixed at their hubs in the body frame
    wheel_rl = wheel()
    wheel_rr = wheel()
    wheel_rl.local_transform = translation(-b, 0.5 * track, r_r)
    wheel_rr.local_transform = translation(-b, -0.5 * track, r_r)

    # The front wheels, placed and steered by their own frames
    wheel_fl = wheel()
    wheel_fr = wheel()

    return {
        "world": [ground],
        "body": [body, wheel_rl, wheel_rr],
        "wheel_fl": [wheel_fl],
        "wheel_fr": [wheel_fr],
    }


def plane_skin_3d(plant, color="#2f6fed"):
    """3-D aircraft look: ground plane plus solid fuselage / wing / tails.

    Frame keys: ``world`` (ground) and ``body`` (airframe solids with the c.g.
    at the body-frame origin). Reads length / span / lever arms from plant
    graphic attrs and ``params`` when present.
    """
    # The drawing attributes, then the aerodynamic parameters, with the defaults
    # of a bare plant
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

    # The aerodynamic chords c = √(S / AR), with AR and c² floored at 10⁻⁶
    chord_w_aero = float(np.sqrt(max(s_w / max(ar, 1e-6), 1e-6)))
    chord_t_aero = float(np.sqrt(max(s_t / max(ar, 1e-6), 1e-6)))

    # The drawn chords, a bit larger than the aerodynamic ones so the wing reads clearly
    chord_w = max(chord_w_aero, 0.12 * length)
    chord_t = max(chord_t_aero, 0.08 * length)

    # The ground, in the world frame
    ground_size = float(getattr(plant, "ground_plane_size", 200.0))
    ground = ground_plane(ground_size)

    # The airframe solids, and a dot at the c.g.
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
    # The DH lengths a and offsets d of each joint
    a = np.asarray(plant.params["a"], dtype=float)
    d = np.asarray(plant.params["d"], dtype=float)

    # Link i spans its joint's offsets: ℓᵢ = √(aᵢ² + dᵢ²)
    lengths = np.hypot(a, d)

    # The base, a pedestal below the first joint
    base = Rod(length=0.12, radius=0.09, color="#a6a8ab")
    base.local_transform = link_pose_3d([0.0, 0.0, -0.12], [0.0, 0.0, 0.0])
    geometry = {"base": [base]}

    # The links, the first three thicker than the wrist's
    for i, length in enumerate(lengths):
        radius = 0.055 if i < 3 else 0.04
        link = Rod(length=length, radius=radius, color="#c7c9cb", linewidth=3)
        geometry[f"link{i}"] = [link]

    # The joints, the first four larger: a housing centered on the joint, and a hub
    for i in range(7):
        radius = 0.075 if i < 4 else 0.055
        housing = Rod(length=2.0 * radius, radius=0.8 * radius, color="#2b73b6")
        housing.local_transform = link_pose_3d([0.0, 0.0, radius], [0.0, 0.0, -radius])
        hub = Sphere(radius=0.72 * radius, color="#d5d7d8", opacity=1.0)
        geometry[f"joint{i}"] = [housing, hub]

    # The tool tip
    geometry["tool"] = [Sphere(radius=0.035, color="#3f4245", opacity=1.0)]

    return geometry


###############################################################################
# Internal machinery
###############################################################################


def axle_offsets(plant):
    """Front/rear CG→axle distances ``(a, b)`` of the car skins.

    ``plant.a`` / ``plant.b`` when the plant has both, else ``params["a"]`` /
    ``params["b"]`` when its ``params`` hold both, else half of
    ``params["length"]`` each (2 m by default), so a bare plant still skins.
    """
    if hasattr(plant, "a") and hasattr(plant, "b"):
        return float(plant.a), float(plant.b)
    params = getattr(plant, "params", {}) or {}
    if "a" in params and "b" in params:
        return float(params["a"]), float(params["b"])
    length = float(params.get("length", 2.0))
    return 0.5 * length, 0.5 * length


def wheel_rectangle(wl, ww, color="black", linewidth=1):
    """Closed wheel-outline polyline (x forward, y lateral) as a :class:`Line`."""
    # Half the wheel's length along x and half its width along y
    h = 0.5 * wl
    w = 0.5 * ww

    # The four corners, closed on the first
    # fmt: off
    corners = np.array([
        [ h,  w, 0.0],
        [ h, -w, 0.0],
        [-h, -w, 0.0],
        [-h,  w, 0.0],
        [ h,  w, 0.0],
    ])
    # fmt: on

    return Line(corners, color=color, linewidth=linewidth)


def ground_plane(size):
    """The ground of the 3-D skins: the plane z = 0 as a grey ``size × size`` patch."""
    return Plane(
        normal=[0.0, 0.0, 1.0],
        offset=0.0,
        size=size,
        thickness=0.04,
        color=[0.72, 0.74, 0.78],
        opacity=0.5,
    )
