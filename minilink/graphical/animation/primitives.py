"""
Graphical primitives and 4x4 transform helpers for system animation.

A system's visualization is keyed **skin** geometry
(:meth:`minilink.core.system.System.get_kinematic_geometry`) plus world
**frames** from :meth:`~minilink.core.system.System.tf`. Renderers draw
primitives at ``frames[key] @ primitive.local_transform``.

Rigid transform builders live in :mod:`minilink.core.kinematics`; camera
matrices in :mod:`minilink.graphical.animation.camera`.
"""

import numpy as np

# Primitive shapes (local-frame geometry)


class GraphicPrimitive:
    """Base class for all geometric objects rendered by the animator engine."""

    def __init__(self, color="blue", linewidth=1, style="-"):
        self.color = color
        self.linewidth = linewidth
        self.style = style
        self.local_transform = np.eye(4)


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


class Arrow(GraphicPrimitive):
    """A 2-D arrow rendered as a polyline (shaft + chevron head).

    The arrow is defined in a **unit local frame** along +X: base at the
    origin, tip at (1, 0).  Renderers apply the accompanying 4x4
    transform whose **column-norm scaling** controls the displayed length
    and whose rotation/translation sets the world pose.

    Parameters
    ----------
    head_ratio : float
        Head barb length as a fraction of the shaft (default 0.15).
    origin : str
        ``'base'`` places the local origin at the tail;
        ``'tip'`` places it at the arrow head.
    """

    def __init__(
        self,
        head_ratio=0.15,
        origin="base",
        color="red",
        linewidth=2,
        style="-",
    ):
        super().__init__(color, linewidth, style)
        self.head_ratio = head_ratio
        self.origin = origin
        self.pts = _arrow_local_pts(head_ratio, origin)


def _arrow_local_pts(head_ratio=0.15, origin="base"):
    """Unit-length arrow polyline (5 pts) along +X in the local frame."""
    d = head_ratio
    if origin == "tip":
        return np.array(
            [
                [-1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [-d, d, 0.0],
                [0.0, 0.0, 0.0],
                [-d, -d, 0.0],
            ]
        )
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0 - d, d, 0.0],
            [1.0, 0.0, 0.0],
            [1.0 - d, -d, 0.0],
        ]
    )


class TorqueArrow(GraphicPrimitive):
    """Curved arc arrow for visualizing torques around a joint.

    The arc is generated dynamically via :meth:`compute_pts` because its
    shape (sweep length) varies with the torque magnitude.

    **Legacy primitive** — prefer :func:`~minilink.graphical.animation.builders.torque_arc_line`
    in :meth:`~minilink.core.system.System.get_dynamic_geometry` for new plants.

    Parameters
    ----------
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
        radius=1.0,
        head_ratio=0.4,
        n_arc_pts=40,
        color="red",
        linewidth=2,
        style="-",
    ):
        super().__init__(color, linewidth, style)
        self.radius = radius
        self.head_ratio = head_ratio
        self.n_arc_pts = n_arc_pts

    def compute_pts(self, sweep):
        """Return Nx3 arc + chevron polyline in **local frame** (centered at origin).

        Parameters
        ----------
        sweep : float
            Arc sweep angle in radians (positive = CCW).
        """
        r = self.radius
        d = r * self.head_ratio

        if abs(sweep) < 1e-6:
            return np.zeros((1, 3))

        n_pts = max(3, int(abs(sweep) / (2 * np.pi) * self.n_arc_pts))
        angles = np.linspace(0, sweep, n_pts)
        arc = np.column_stack(
            [
                r * np.cos(angles),
                r * np.sin(angles),
                np.zeros(n_pts),
            ]
        )

        tip_c = np.cos(sweep)
        tip_s = np.sin(sweep)
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


class HorizonPolyline(GraphicPrimitive):
    """World-frame polyline of the active receding-horizon plan at time *t*.

    ``plans`` is a sequence of ``(t_solve, trajectory)`` pairs with world-frame
    ``trajectory.x`` and ``trajectory.t``. At playback time *t*, call
    :meth:`points_at` and draw via :meth:`~minilink.core.system.System.get_dynamic_geometry`
    as a :class:`CustomLine` keyed to ``"world"``.

    Geometry is rebuilt each frame through :meth:`compute_pts`, like
    :class:`TorqueArrow`.
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
        return self.points_at(t_now)

    def points_at(self, t_now):
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


class TrajectoryPolyline(GraphicPrimitive):
    """World-frame XY polyline sampled from a :class:`~minilink.core.trajectory.Trajectory`.

    At playback time *t*, call :meth:`points_at` and emit a :class:`CustomLine`
    from :meth:`~minilink.core.system.System.get_dynamic_geometry`.

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
        return self.points_at(t_now)

    def points_at(self, t_now):
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


# Transform helpers (re-exported from core kinematics and camera)

from minilink.core.kinematics import (  # noqa: E402
    apply_transform,
    identity_matrix,
    point_transform,
    pose2d_matrix,
    rod_between_transform,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
    translation_matrix,
)
from minilink.graphical.animation.camera import (  # noqa: E402
    camera_matrix,
    follow_xy_camera,
    world_to_camera,
)


def empty_transform():
    """Transform that parks a primitive far off-screen (used to hide it)."""
    return translation_matrix(0.0, 0.0, -1000.0)


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
