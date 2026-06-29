"""
Graphical primitives and 4x4 transform helpers for system animation.

A system's visualization is a dict of named **frames** to **primitives** (shapes
defined in their own local frame) plus, at every instant, one 4x4 homogeneous
**transform** per frame placing it in the world (see
:meth:`minilink.core.system.System.get_kinematic_geometry`,
:meth:`~minilink.core.system.System.get_dynamic_geometry`, and
:meth:`~minilink.core.system.System.tf`). Renderers draw the primitives; they
never know about states or inputs.

Primitives are **honest**: they bake their full point geometry at real world
size in their own local frame, so the animator just draws them at
``frames[key] @ local_transform`` like any other polyline — no column-norm
scale, no ``T[3, 3]`` side-channel. The scaling helpers
(:func:`scale_pose2d_matrix`, :func:`line_between_transform`) stretch a
unit-length primitive (e.g. :class:`CustomLine` springs) to a world span by
posing it with an anisotropic frame, which is a real geometric transform rather
than a renderer side-channel.

This module is NumPy-only and safe to import from core kinematic hooks: it
never pulls in matplotlib or other rendering libraries.
"""

import numpy as np

# Primitive shapes (local-frame geometry)


class GraphicPrimitive:
    """Base class for all geometric objects rendered by the animator engine.

    ``local_transform`` is a fixed 4x4 graphical offset baked into the primitive
    (a constant pose within its frame). The v2 animator poses a primitive at
    ``frames[key] @ local_transform``; the legacy pipeline ignores it. Defaults
    to identity, so existing primitives are unaffected.
    """

    def __init__(self, color="blue", linewidth=1, style="-", local_transform=None):
        self.color = color
        self.linewidth = linewidth
        self.style = style
        self.local_transform = (
            np.eye(4) if local_transform is None else np.asarray(local_transform, float)
        )


class CustomLine(GraphicPrimitive):
    """A generic sequence of connected line segments."""

    def __init__(self, pts, color="blue", linewidth=1, style="-"):
        """
        Parameters
        ----------
        pts : list or np.ndarray
            Nx2 or Nx3 array of points connected sequentially.
        """
        super().__init__(color, linewidth, style)
        self.pts = np.array(pts)

    def points_at(self, t):
        """Nx3 polyline points at playback time *t* (static: time-independent)."""
        return self.pts


class Point(GraphicPrimitive):
    """An individual point marker."""

    def __init__(self, pt=(0, 0, 0), color="red", marker="o", size=5):
        super().__init__(color)
        self.pt = np.array(pt)
        self.marker = marker
        self.size = size


class Circle(GraphicPrimitive):
    """A basic circle primitive. Lives in the XY plane by default."""

    def __init__(self, radius=1.0, center=(0, 0, 0), color="blue", fill=False):
        super().__init__(color)
        self.radius = radius
        self.center = np.array(center)
        self.fill = fill


class Sphere(GraphicPrimitive):
    """A 3D sphere primitive centered at ``center`` in local frame."""

    def __init__(self, radius=1.0, center=(0, 0, 0), color="blue", opacity=1.0):
        super().__init__(color)
        self.radius = radius
        self.center = np.array(center)
        self.opacity = opacity


class Rod(GraphicPrimitive):
    """A slender rigid rod primitive aligned with local -Y axis."""

    def __init__(
        self,
        length=1.0,
        radius=0.05,
        color="blue",
        opacity=1.0,
        linewidth=2,
        style="-",
    ):
        super().__init__(color=color, linewidth=linewidth, style=style)
        self.length = float(length)
        self.radius = float(radius)
        self.opacity = float(opacity)


class Plane(GraphicPrimitive):
    """
    A finite square patch representing a plane: n·x = offset.

    The patch is centered at ``offset * normal`` and spans ``size x size``.
    """

    def __init__(
        self,
        normal=(0, 1, 0),
        offset=0.0,
        size=10.0,
        thickness=0.02,
        color="lightgray",
        opacity=0.65,
    ):
        super().__init__(color)
        self.normal = np.array(normal, dtype=float)
        self.offset = float(offset)
        self.size = float(size)
        self.thickness = float(thickness)
        self.opacity = float(opacity)


class Box(GraphicPrimitive):
    """Axis-aligned rectangular solid in local frame (centered at ``center``).

    Dimensions are **full** lengths along local X, Y, Z. Used for simple vehicle
    bodies and blocks in 3D renderers (MeshCat box geometry).
    """

    def __init__(
        self,
        length_x: float = 1.0,
        length_y: float = 1.0,
        length_z: float = 1.0,
        center=(0.0, 0.0, 0.0),
        color="gray",
        opacity: float = 1.0,
    ):
        super().__init__(color)
        self.length_x = float(length_x)
        self.length_y = float(length_y)
        self.length_z = float(length_z)
        self.center = np.asarray(center, dtype=float).reshape(3)
        self.opacity = float(opacity)


class ExtrudedPolygon(GraphicPrimitive):
    """Convex polygon in local XY, extruded symmetrically along local Z.

    This is useful for light-weight 3D body shells such as vehicle noses,
    cabins, side pods, and tapered covers without introducing a full mesh
    asset pipeline.
    """

    def __init__(
        self,
        pts_xy,
        height: float = 1.0,
        center=(0.0, 0.0, 0.0),
        color="gray",
        opacity: float = 1.0,
    ):
        super().__init__(color)
        pts = np.asarray(pts_xy, dtype=float).reshape(-1, 2)
        if pts.shape[0] < 3:
            raise ValueError("ExtrudedPolygon requires at least 3 XY points")
        if np.allclose(pts[0], pts[-1]):
            pts = pts[:-1]
        self.pts_xy = pts
        self.height = float(height)
        self.center = np.asarray(center, dtype=float).reshape(3)
        self.opacity = float(opacity)

    def vertices_local(self) -> np.ndarray:
        """Return local vertices with shape ``(2*n, 3)``."""
        n = self.pts_xy.shape[0]
        z0 = self.center[2] - 0.5 * self.height
        z1 = self.center[2] + 0.5 * self.height
        xy = self.pts_xy + self.center[:2]
        bottom = np.column_stack((xy, np.full(n, z0)))
        top = np.column_stack((xy, np.full(n, z1)))
        return np.vstack((bottom, top))

    def edges(self) -> tuple[tuple[int, int], ...]:
        """Return wireframe edges as vertex-index pairs."""
        n = self.pts_xy.shape[0]
        edges = []
        for i in range(n):
            j = (i + 1) % n
            edges.append((i, j))
            edges.append((i + n, j + n))
            edges.append((i, i + n))
        return tuple(edges)

    def mesh_data(self) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(vertices, faces)`` for triangle-mesh renderers."""
        n = self.pts_xy.shape[0]
        vertices = self.vertices_local()
        faces: list[list[int]] = []

        for i in range(1, n - 1):
            faces.append([0, i + 1, i])
            faces.append([n, n + i, n + i + 1])

        for i in range(n):
            j = (i + 1) % n
            faces.append([i, j, n + i])
            faces.append([j, n + j, n + i])

        return vertices, np.asarray(faces, dtype=np.uint32)


def arrow_pts(base, vector, scale=1.0, head_ratio=0.15):
    """Polyline (Nx3) of a straight arrow from *base* along *vector*, in local XY.

    The arrow is drawn at its **true length** ``scale * |vector|`` — the geometry
    is honest, so no column-norm scaling of the placing transform is needed. A
    near-zero vector collapses to a single point (nothing visible).

    Parameters
    ----------
    base : array-like, length 2
        Tail of the arrow in the local frame.
    vector : array-like, length 2
        Direction (and, with *scale*, length) of the shaft.
    scale : float
        Multiplies *vector* to set the drawn length (e.g. a velocity gain).
    head_ratio : float
        Chevron barb length as a fraction of the shaft length.
    """
    base = np.asarray(base, dtype=float).reshape(2)
    shaft = scale * np.asarray(vector, dtype=float).reshape(2)
    length = float(np.hypot(shaft[0], shaft[1]))
    if length < 1e-12:
        return np.array([[base[0], base[1], 0.0]])

    tip = base + shaft
    direction = shaft / length
    d = head_ratio * length
    back = direction * d
    perp = np.array([-direction[1], direction[0]]) * d

    barb1 = tip - back + perp
    barb2 = tip - back - perp
    return np.array(
        [
            [base[0], base[1], 0.0],
            [tip[0], tip[1], 0.0],
            [barb1[0], barb1[1], 0.0],
            [tip[0], tip[1], 0.0],
            [barb2[0], barb2[1], 0.0],
        ]
    )


def torque_arc_pts(sweep, radius=1.0, head_ratio=0.4, n_arc_pts=40):
    """Polyline (Nx3) of a circular torque arc + chevron, centered at the origin.

    The arc is baked at the given *sweep* (radians, + = CCW) and *radius*, so the
    sweep rides in the geometry rather than a transform side-channel.
    """
    sweep = float(sweep)
    r = radius
    d = r * head_ratio

    if abs(sweep) < 1e-6:
        return np.zeros((1, 3))

    n_pts = max(3, int(abs(sweep) / (2 * np.pi) * n_arc_pts))
    angles = np.linspace(0.0, sweep, n_pts)
    arc = np.column_stack([r * np.cos(angles), r * np.sin(angles), np.zeros(n_pts)])

    tip_c, tip_s = np.cos(sweep), np.sin(sweep)
    tip = np.array([r * tip_c, r * tip_s, 0.0])
    if sweep > 0:
        barb1 = tip + np.array(
            [-d / 2 * tip_c + d / 2 * tip_s, -d / 2 * tip_s - d / 2 * tip_c, 0.0]
        )
        barb2 = tip + np.array(
            [d / 2 * tip_c + d / 2 * tip_s, d / 2 * tip_s - d / 2 * tip_c, 0.0]
        )
    else:
        barb1 = tip + np.array(
            [-d / 2 * tip_c - d / 2 * tip_s, -d / 2 * tip_s + d / 2 * tip_c, 0.0]
        )
        barb2 = tip + np.array(
            [d / 2 * tip_c - d / 2 * tip_s, d / 2 * tip_s + d / 2 * tip_c, 0.0]
        )
    return np.vstack([arc, np.array([barb1, tip, barb2])])


class Arrow(GraphicPrimitive):
    """A 2-D arrow at honest world size — shaft from *base* along *vector*.

    The geometry is baked at its true length ``scale * |vector|``; the renderer
    simply draws ``self.pts`` posed by the frame. Pass body-frame ``vector`` and
    key the primitive to that body frame so its orientation comes from ``tf`` (no
    manual ``cos``/``sin`` rotation).
    """

    def __init__(
        self,
        base=(0.0, 0.0),
        vector=(1.0, 0.0),
        scale=1.0,
        head_ratio=0.15,
        color="red",
        linewidth=2,
        style="-",
    ):
        super().__init__(color, linewidth, style)
        self.base = np.asarray(base, dtype=float).reshape(2)
        self.vector = np.asarray(vector, dtype=float).reshape(2)
        self.scale = float(scale)
        self.head_ratio = head_ratio
        self.pts = arrow_pts(self.base, self.vector, self.scale, head_ratio)

    def points_at(self, t):
        """Nx3 arrow points at playback time *t* (geometry baked at construction)."""
        return self.pts


class TorqueArrow(GraphicPrimitive):
    """A curved torque arc at honest geometry — *sweep* baked into the points.

    The arc for the given *sweep* and *radius* is built at construction, centered
    at the local origin; the placing frame supplies world position and the start
    orientation. Rebuild it each frame in ``get_dynamic_geometry`` with the
    instantaneous torque.

    Parameters
    ----------
    sweep : float
        Arc sweep angle in radians (positive = CCW, negative = CW).
    radius : float
        Radius of the arc in world units.
    head_ratio : float
        Chevron barb length as a fraction of the radius.
    n_arc_pts : int
        Number of sample points used for a full-circle arc (subsampled
        proportionally for smaller sweeps).
    """

    def __init__(
        self,
        sweep,
        radius=1.0,
        head_ratio=0.4,
        n_arc_pts=40,
        color="red",
        linewidth=2,
        style="-",
    ):
        super().__init__(color, linewidth, style)
        self.sweep = float(sweep)
        self.radius = radius
        self.head_ratio = head_ratio
        self.n_arc_pts = n_arc_pts
        self.pts = torque_arc_pts(self.sweep, radius, head_ratio, n_arc_pts)

    def points_at(self, t):
        """Nx3 arc points at playback time *t* (geometry baked at construction)."""
        return self.pts


class HorizonPolyline(GraphicPrimitive):
    """World-frame polyline of the active receding-horizon plan at time *t*.

    ``plans`` is a sequence of ``(t_solve, trajectory)`` pairs with world-frame
    ``trajectory.x`` and ``trajectory.t``. At playback time *t*, the primitive
    draws the latest plan with ``t_solve <= t`` over samples with
    ``trajectory.t >= t``.

    Geometry is rebuilt each frame through :meth:`points_at` (an overlay's
    ``get_dynamic_geometry(t)`` bakes the world polyline into a
    :class:`CustomLine`).
    """

    def __init__(
        self,
        plans,
        *,
        color="tab:orange",
        linewidth=2,
        style="--",
    ):
        super().__init__(color, linewidth, style)
        self.plans = list(plans)

    def compute_pts(self, t_now):
        """Return Nx3 world-frame polyline points for the active plan tail."""
        t_now = float(t_now)
        active = None
        for t_solve, plan in self.plans:
            if t_solve <= t_now + 1e-9:
                active = plan
        if active is None:
            return np.zeros((1, 3))
        mask = active.t >= t_now - 1e-9
        if np.count_nonzero(mask) < 2:
            return np.zeros((1, 3))
        xy = active.x[:2, mask]
        return np.column_stack([xy[0], xy[1], np.zeros(xy.shape[1])])

    def points_at(self, t):
        """Nx3 world-frame points of the active plan tail at playback time *t*."""
        return self.compute_pts(t)


class TrajectoryPolyline(GraphicPrimitive):
    """World-frame XY polyline sampled from a :class:`~minilink.core.trajectory.Trajectory`.

    At playback time *t*:

    ``window="prefix"``
        Samples with ``trajectory.t <= t`` — a growing executed trail.
    ``window="suffix"``
        Samples with ``trajectory.t >= t``.
    ``window="all"``
        Full trajectory polyline (time-independent geometry).

    Geometry is rebuilt each frame through :meth:`compute_pts`, like
    :class:`HorizonPolyline`.
    """

    _WINDOWS = ("prefix", "suffix", "all")

    def __init__(
        self,
        trajectory,
        *,
        window="prefix",
        color="tab:blue",
        linewidth=2,
        style="-",
    ):
        super().__init__(color, linewidth, style)
        self.trajectory = trajectory
        if window not in self._WINDOWS:
            raise ValueError(f"window must be one of {self._WINDOWS}, got {window!r}")
        self.window = window

    def compute_pts(self, t_now):
        """Return Nx3 world-frame polyline points for the selected time window."""
        t_now = float(t_now)
        traj = self.trajectory
        if self.window == "all":
            mask = np.ones(traj.n_samples, dtype=bool)
        elif self.window == "prefix":
            mask = traj.t <= t_now + 1e-9
        else:
            mask = traj.t >= t_now - 1e-9
        if np.count_nonzero(mask) < 2:
            return np.zeros((1, 3))
        xy = traj.x[:2, mask]
        return np.column_stack([xy[0], xy[1], np.zeros(xy.shape[1])])

    def points_at(self, t):
        """Nx3 world-frame points of the selected time window at playback time *t*."""
        return self.compute_pts(t)


# Transformation Matrix Helpers


def translation_matrix(dx=0.0, dy=0.0, dz=0.0):
    """
    Generate a 4x4 pure translation matrix.

    Parameters
    ----------
    dx, dy, dz : float
        Translation along the x, y, and z axes.

    Returns
    -------
    np.ndarray
        A 4x4 transformation matrix.
    """
    T = np.eye(4)
    T[0, 3] = dx
    T[1, 3] = dy
    T[2, 3] = dz
    return T


def pose2d_matrix(x=0.0, y=0.0, theta=0.0):
    """
    Generate a 4x4 transformation matrix for a 2D pose (XY plane).

    Parameters
    ----------
    x, y : float
        Translation in the XY plane.
    theta : float
        Rotation around the Z axis in radians.

    Returns
    -------
    np.ndarray
        A 4x4 transformation matrix.
    """
    T = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    T[0, 3] = x
    T[1, 3] = y
    return T


def rotation_matrix_x(theta=0.0):
    """Generate a 4x4 rotation matrix about the X axis."""
    T = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    T[1, 1] = c
    T[1, 2] = -s
    T[2, 1] = s
    T[2, 2] = c
    return T


def rotation_matrix_y(theta=0.0):
    """Generate a 4x4 rotation matrix about the Y axis."""
    T = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    T[0, 0] = c
    T[0, 2] = s
    T[2, 0] = -s
    T[2, 2] = c
    return T


def rotation_matrix_z(theta=0.0):
    """Generate a 4x4 rotation matrix about the Z axis."""
    T = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    return T


def scale_pose2d_matrix(x=0.0, y=0.0, theta=0.0, scale=1.0):
    """4x4 matrix: 2-D rotation *theta*, uniform *scale*, translation *(x, y)*.

    Multiplying the rotation columns by *scale* lets renderers stretch
    unit-length primitives (e.g. :class:`Arrow`) to the desired world size
    while preserving position and orientation.
    """
    T = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    T[0, 0] = scale * c
    T[0, 1] = scale * (-s)
    T[1, 0] = scale * s
    T[1, 1] = scale * c
    T[0, 3] = x
    T[1, 3] = y
    return T


def camera_matrix(target=(0.0, 0.0, 0.0), plot_axes=(0, 1), scale=10.0):
    """Standard 4x4 camera transform.

    The matrix doubles as the camera's pose in the world frame **and** the
    renderer projection knob. The ``T[3, 3]`` slot carries the view scale
    (orthographic half-extent for matplotlib / pygame; camera distance for
    meshcat).

    Slot meaning
    ------------
    * ``T[:3, 3]`` — look-at target in world (point at the center of the view).
    * ``T[:3, 0]`` — world direction shown as **plot horizontal** axis.
    * ``T[:3, 1]`` — world direction shown as **plot vertical** axis.
    * ``T[:3, 2]`` — camera view-out direction (projected away in 2D / ortho;
      eye-out in 3D perspective).
    * ``T[3, 3]`` — view scale (orthographic half-extent in world units for
      matplotlib / pygame; perspective camera distance for meshcat).

    Parameters
    ----------
    target : array-like of length 3, optional
        Look-at point in world coordinates.
    plot_axes : tuple of two ints in {0, 1, 2}, optional
        World axis indices used as plot-X (``i``) and plot-Y (``j``).
        ``R`` is built so its columns are ``(e_i, e_j, e_i x e_j)``.
        Default ``(0, 1)`` is the canonical top-down view.
    scale : float, optional
        View half-extent (orthographic) or camera distance (perspective).
    Returns
    -------
    np.ndarray
        4x4 camera transform.
    """
    T = np.eye(4)
    i, j = plot_axes
    if i == j or i not in (0, 1, 2) or j not in (0, 1, 2):
        raise ValueError(
            "plot_axes must be two distinct world axis indices in {0, 1, 2}; "
            f"got {plot_axes!r}."
        )
    e = np.eye(3)
    T[:3, 0] = e[i]
    T[:3, 1] = e[j]
    T[:3, 2] = np.cross(e[i], e[j])
    T[:3, 3] = np.asarray(target, dtype=float).reshape(3)
    T[3, 3] = float(scale)
    return T


def world_to_camera(camera):
    """Return the world-to-camera (view) 4x4 matrix.

    Inverts the rigid part of *camera* (target translation + ``R``); the
    amplitude channel ``T[3, 3]`` is reset to 1 so the result is a regular
    rigid transform suitable for pre-multiplying body transforms before
    orthographic 2D rendering.
    """
    R = camera[:3, :3]
    target = camera[:3, 3]
    W = np.eye(4)
    W[:3, :3] = R.T
    W[:3, 3] = -R.T @ target
    return W


def identity_matrix():
    """4x4 identity transform (primitive drawn at the world origin)."""
    return np.eye(4)


def empty_transform():
    """Transform that parks a primitive far off-screen (used to hide it)."""
    return translation_matrix(0.0, 0.0, -1000.0)


def follow_xy_camera(x, y, scale):
    """Top-down camera centered on world point *(x, y)* with view half-extent *scale*."""
    return camera_matrix(target=(x, y, 0.0), plot_axes=(0, 1), scale=scale)


def heading_from_vector(vx, vy):
    """Planar heading angle of the vector *(vx, vy)*."""
    return np.arctan2(vy, vx)


def line_between_transform(p0, p1):
    """Place a unit :class:`CustomLine` so it spans from *p0* to *p1* in the plane."""
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
    """Pose a unit :class:`Rod` (length along local -y) from *p0* to *p1* in 3-D.

    Builds an orthonormal frame whose y-axis points from *p0* toward *p1*;
    a reference axis is swapped when nearly parallel to keep the cross
    products well conditioned.
    """
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
    """Translation transform placing a primitive at *point* (z defaults to 0)."""
    point = np.asarray(point, dtype=float)
    return translation_matrix(point[0], point[1], point[2] if point.size > 2 else 0.0)


# Ready-Made Shapes And Poses


def ground_line(length=20.0, y=0.0, color="black", style="--"):
    """Horizontal reference line of span *length* at height *y* (e.g. ground)."""
    return CustomLine(
        [[-0.5 * length, y, 0.0], [0.5 * length, y, 0.0]],
        color=color,
        linewidth=1,
        style=style,
    )


def spring_line(coils=6, amplitude=0.12, color="black", linewidth=1):
    """Unit-length zig-zag spring along local +X (lead-in, *coils* coils, lead-out).

    Drawn from x=0 to x=1; pair with a transform that spans the two endpoints.
    """
    pts = [[0.0, 0.0, 0.0], [0.15, 0.0, 0.0]]
    xs = np.linspace(0.2, 0.8, 2 * coils + 1)
    for i, x in enumerate(xs):
        y = amplitude if i % 2 else -amplitude
        pts.append([x, y, 0.0])
    pts.append([0.85, 0.0, 0.0])
    pts.append([1.0, 0.0, 0.0])
    return CustomLine(pts, color=color, linewidth=linewidth)


def wheel_box(length=0.45, width=0.16):
    """Small flat box used as a wheel/contact patch in vehicle diagrams."""
    return Box(
        length_x=length, length_y=width, length_z=0.08, color="black", opacity=0.9
    )


def vehicle_body(length=1.0, width=0.5, color="blue", opacity=0.85):
    """Planar car/robot shell (arrow-shaped outline pointing along +X)."""
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
