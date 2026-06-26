"""
Pre-migration rendering helpers (NumPy only).

These reproduce the exact ``scale_pose2d_matrix`` / ``Arrow`` / ``TorqueArrow``
placement used before the kinematic-contract upgrade.  Attach via
``primitive.local_transform`` on ``"world"`` (identity in :meth:`tf`).
"""

from __future__ import annotations

import numpy as np

from minilink.graphical.animation.primitives import Arrow, CustomLine, TorqueArrow


def scale_pose2d_matrix(x=0.0, y=0.0, theta=0.0, scale=1.0):
    """2-D rotation *theta*, uniform *scale* on rotation columns, translation *(x, y)*."""
    T = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    T[0, 0] = scale * c
    T[0, 1] = scale * (-s)
    T[1, 0] = scale * s
    T[1, 1] = scale * c
    T[0, 3] = x
    T[1, 3] = y
    return T


def heading_from_vector(vx, vy):
    return float(np.arctan2(vy, vx))


def arrow_transform(x, y, vx, vy, scale=1.0):
    """World 4×4 for a unit :class:`Arrow` aligned with *(vx, vy)*."""
    length = scale * np.hypot(vx, vy)
    if length < 1e-12:
        return scale_pose2d_matrix(x, y, 0.0, 0.0)
    return scale_pose2d_matrix(x, y, heading_from_vector(vx, vy), length)


def force_arrow_transform(x, force):
    """Horizontal force arrow at world *x* (mass-spring convention)."""
    if abs(force) < 1e-12:
        return scale_pose2d_matrix(x, 0.0, 0.0, 0.0)
    theta = 0.0 if force >= 0.0 else np.pi
    return scale_pose2d_matrix(x, 0.0, theta, 0.3 * abs(force))


def legacy_arrow(
    transform,
    *,
    color="red",
    linewidth=2,
    origin="base",
    z=None,
):
    """Unit :class:`Arrow` with ``local_transform`` set to *transform*."""
    arr = Arrow(color=color, linewidth=linewidth, origin=origin)
    T = np.asarray(transform, dtype=float).copy()
    if z is not None:
        T[2, 3] = float(z)
    arr.local_transform = T
    return arr


def legacy_arrow_world(x, y, theta, scale, **kwargs):
    return legacy_arrow(scale_pose2d_matrix(x, y, theta, scale), **kwargs)


def legacy_arrow_vector(x, y, vx, vy, scale=1.0, **kwargs):
    return legacy_arrow(arrow_transform(x, y, vx, vy, scale), **kwargs)


def legacy_body_arrow(T_body, lx, ly, theta, scale, **kwargs):
    """``T_body @ scale_pose2d_matrix(lx, ly, theta, scale)`` for body-fixed arrows."""
    T = np.asarray(T_body, dtype=float) @ scale_pose2d_matrix(lx, ly, theta, scale)
    return legacy_arrow(T, **kwargs)


def legacy_torque_world(
    x,
    y,
    start_angle,
    sweep,
    radius,
    *,
    head_ratio=0.4,
    color="red",
    linewidth=2,
):
    """World-frame torque arc matching ``TorqueArrow`` + ``torque_pose2d_matrix``."""
    ta = TorqueArrow(
        radius=radius,
        head_ratio=head_ratio,
        color=color,
        linewidth=linewidth,
    )
    pts = ta.compute_pts(float(sweep))
    c, s = np.cos(start_angle), np.sin(start_angle)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    world = (rot @ pts.T).T
    world[:, 0] += x
    world[:, 1] += y
    return CustomLine(world, color=color, linewidth=linewidth)


def bicycle_sl_world_arrows(x, u, t, sys):
    """Velocity + tire-force arrows for :class:`DynamicBicycleMagicForces` models."""
    X, Y, Theta = float(x[0]), float(x[1]), float(x[2])
    _, vb = sys.x2q(x)
    u_in = sys.get_u_int(x, u)
    a, b = sys.a, sys.b
    uu, vv, wr = float(vb[0]), float(vb[1]), float(vb[2])
    v_f_loc = np.array([uu, vv + a * wr])
    v_r_loc = np.array([uu, vv - b * wr])
    c, s = np.cos(Theta), np.sin(Theta)
    rx = X + c * (-b) - s * 0.0
    ry = Y + s * (-b) + c * 0.0
    fx = X + c * a - s * 0.0
    fy = Y + s * a + c * 0.0
    vfx, vfy = c * v_f_loc[0] - s * v_f_loc[1], s * v_f_loc[0] + c * v_f_loc[1]
    vrx, vry = c * v_r_loc[0] - s * v_r_loc[1], s * v_r_loc[0] + c * v_r_loc[1]
    fx_f_b, fy_f_b, fx_r_b, fy_r_b = sys.tire_forces_body_frame(vb, u_in)
    ffx_w = c * fx_f_b - s * fy_f_b
    ffy_w = s * fx_f_b + c * fy_f_b
    frx_w = c * fx_r_b - s * fy_r_b
    fry_w = s * fx_r_b + c * fy_r_b
    v_scale = 0.2
    f_scale = 0.001

    def _vel(dx, dy, px, py):
        mag = v_scale * np.hypot(dx, dy)
        if mag < 1e-9:
            mag = 1e-9
        th = np.arctan2(dy, dx)
        return legacy_arrow_world(px, py, th, mag, color="blue", linewidth=2)

    def _force(Fx, Fy, px, py):
        mag = f_scale * np.hypot(Fx, Fy)
        if mag < 1e-12:
            mag = 1e-12
        th = np.arctan2(Fy, Fx)
        return legacy_arrow_world(px, py, th, mag, color="red", linewidth=2)

    return [
        _vel(vrx, vry, rx, ry),
        _vel(vfx, vfy, fx, fy),
        _force(frx_w, fry_w, rx, ry),
        _force(ffx_w, ffy_w, fx, fy),
    ]


def bicycle_world_arrows(x, u, t, sys):
    """Velocity + tire-force arrows for planar dynamic bicycle models."""
    params = sys.params
    a = params["a"]
    b = params["b"]
    X, Y, Theta = float(x[0]), float(x[1]), float(x[2])
    vb = x[3:6]
    u_in = sys._bicycle_actuators(x, u)
    delta = float(u_in[1])

    v_scale = 0.2
    f_scale = 0.001

    uu, vv, wr = float(vb[0]), float(vb[1]), float(vb[2])
    v_f_loc = np.array([uu, vv + a * wr])
    v_r_loc = np.array([uu, vv - b * wr])

    c, s = np.cos(Theta), np.sin(Theta)
    rx = X + c * (-b) - s * 0.0
    ry = Y + s * (-b) + c * 0.0
    fx = X + c * a - s * 0.0
    fy = Y + s * a + c * 0.0

    vfx, vfy = c * v_f_loc[0] - s * v_f_loc[1], s * v_f_loc[0] + c * v_f_loc[1]
    vrx, vry = c * v_r_loc[0] - s * v_r_loc[1], s * v_r_loc[0] + c * v_r_loc[1]

    fx_f, fy_f, fx_r, fy_r = sys.compute_tire_physics(vb, u_in, params)
    cd, sd = np.cos(delta), np.sin(delta)
    fxf_b = fx_f * cd - fy_f * sd
    fyf_b = fx_f * sd + fy_f * cd
    ffx_w = c * fxf_b - s * fyf_b
    ffy_w = s * fxf_b + c * fyf_b
    frx_w = c * fx_r - s * fy_r
    fry_w = s * fx_r + c * fy_r

    def _vel(dx, dy, px, py):
        mag = v_scale * np.hypot(dx, dy)
        if mag < 1e-9:
            mag = 1e-9
        th = np.arctan2(dy, dx)
        return legacy_arrow_world(px, py, th, mag, color="blue", linewidth=2)

    def _force(Fx, Fy, px, py):
        mag = f_scale * np.hypot(Fx, Fy)
        if mag < 1e-12:
            mag = 1e-12
        th = np.arctan2(Fy, Fx)
        return legacy_arrow_world(px, py, th, mag, color="red", linewidth=2)

    return [
        _vel(vrx, vry, rx, ry),
        _vel(vfx, vfy, fx, fy),
        _force(frx_w, fry_w, rx, ry),
        _force(ffx_w, ffy_w, fx, fy),
    ]


def bicycle_car3d_world_arrows(x, u, t, sys):
    """Eight velocity/force arrows at left/right wheel contact points (3-D car)."""
    params = sys.params
    a = params["a"]
    b = params["b"]
    r_f = params["r_f"]
    r_r = params["r_r"]
    X, Y, Theta = float(x[0]), float(x[1]), float(x[2])
    vb = x[3:6]
    u_in = sys._bicycle_actuators(x, u)
    delta = float(u_in[1])
    tr = sys.track

    v_scale = 0.2
    f_scale = 0.001

    uu, vv, wr = float(vb[0]), float(vb[1]), float(vb[2])
    v_f_loc = np.array([uu, vv + a * wr])
    v_r_loc = np.array([uu, vv - b * wr])

    c, s = np.cos(Theta), np.sin(Theta)
    vfx, vfy = c * v_f_loc[0] - s * v_f_loc[1], s * v_f_loc[0] + c * v_f_loc[1]
    vrx, vry = c * v_r_loc[0] - s * v_r_loc[1], s * v_r_loc[0] + c * v_r_loc[1]

    fx_f, fy_f, fx_r, fy_r = sys.compute_tire_physics(vb, u_in, params)
    cd, sd = np.cos(delta), np.sin(delta)
    fxf_b = fx_f * cd - fy_f * sd
    fyf_b = fx_f * sd + fy_f * cd
    ffx_w = c * fxf_b - s * fyf_b
    ffy_w = s * fxf_b + c * fyf_b
    frx_w = c * fx_r - s * fy_r
    fry_w = s * fx_r + c * fy_r

    def _body_to_world(bx, by, bz):
        wx = X + c * bx - s * by
        wy = Y + s * bx + c * by
        return wx, wy, bz

    def _vel(dx, dy, bx, by, bz):
        mag = v_scale * np.hypot(dx, dy)
        if mag < 1e-9:
            mag = 1e-9
        th = np.arctan2(dy, dx)
        px, py, pz = _body_to_world(bx, by, bz)
        return legacy_arrow_world(px, py, th, mag, color="blue", linewidth=2, z=pz)

    def _force(Fx, Fy, bx, by, bz):
        mag = f_scale * np.hypot(Fx, Fy)
        if mag < 1e-12:
            mag = 1e-12
        th = np.arctan2(Fy, Fx)
        px, py, pz = _body_to_world(bx, by, bz)
        return legacy_arrow_world(px, py, th, mag, color="red", linewidth=2, z=pz)

    return [
        _vel(vrx, vry, -b, 0.5 * tr, r_r),
        _vel(vrx, vry, -b, -0.5 * tr, r_r),
        _vel(vfx, vfy, a, 0.5 * tr, r_f),
        _vel(vfx, vfy, a, -0.5 * tr, r_f),
        _force(frx_w, fry_w, -b, 0.5 * tr, r_r),
        _force(frx_w, fry_w, -b, -0.5 * tr, r_r),
        _force(ffx_w, ffy_w, a, 0.5 * tr, r_f),
        _force(ffx_w, ffy_w, a, -0.5 * tr, r_f),
    ]
