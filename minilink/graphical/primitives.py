import numpy as np


class GraphicPrimitive:
    """Base class for all geometric objects rendered by the animator engine."""

    def __init__(self, color="blue", linewidth=1, style="-"):
        self.color = color
        self.linewidth = linewidth
        self.style = style


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

    def __init__(self, pt=[0, 0, 0], color="red", marker="o", size=5):
        super().__init__(color)
        self.pt = np.array(pt)
        self.marker = marker
        self.size = size


class Circle(GraphicPrimitive):
    """A basic circle primitive. Lives in the XY plane by default."""

    def __init__(self, radius=1.0, center=[0, 0, 0], color="blue", fill=False):
        super().__init__(color)
        self.radius = radius
        self.center = np.array(center)
        self.fill = fill


class Sphere(GraphicPrimitive):
    """A 3D sphere primitive centered at ``center`` in local frame."""

    def __init__(self, radius=1.0, center=[0, 0, 0], color="blue", opacity=1.0):
        super().__init__(color)
        self.radius = radius
        self.center = np.array(center)
        self.opacity = opacity


class Plane(GraphicPrimitive):
    """
    A finite square patch representing a plane: n·x = offset.

    The patch is centered at ``offset * normal`` and spans ``size x size``.
    """

    def __init__(
        self,
        normal=[0, 1, 0],
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


class Rectangle(GraphicPrimitive):
    """A standard axis-aligned rectangle in local frame (XY plane by default)."""

    def __init__(
        self, width=1.0, height=1.0, center=[0, 0, 0], color="blue", fill=False
    ):
        super().__init__(color)
        self.width = width
        self.height = height
        self.center = np.array(center)
        self.fill = fill


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
        return np.array([
            [-1.0,  0.0, 0.0],
            [ 0.0,  0.0, 0.0],
            [ -d,    d,  0.0],
            [ 0.0,  0.0, 0.0],
            [ -d,   -d,  0.0],
        ])
    return np.array([
        [0.0,    0.0, 0.0],
        [1.0,    0.0, 0.0],
        [1.0-d,   d,  0.0],
        [1.0,    0.0, 0.0],
        [1.0-d,  -d,  0.0],
    ])


class TorqueArrow(GraphicPrimitive):
    """Curved arc arrow for visualizing torques around a joint.

    The arc is generated dynamically via :meth:`compute_pts` because its
    shape (sweep length) varies with the torque magnitude.

    **Transform convention** — the ``T[3, 3]`` slot of the 4×4 matrix
    carries the **amplitude** (here, the sweep angle in radians).  In a
    standard homogeneous matrix ``T[3, 3]`` is always 1; a non-unit value
    is therefore an unambiguous side-channel that renderers extract before
    applying the rigid part.  See :ref:`amplitude-channel` and
    :func:`torque_pose2d_matrix`.

    * Translation ``T[0:2, 3]`` — centre of the arc (joint world position).
    * Rotation (upper-left 2×2) — starting angle of the arc (typically
      the rod direction so the arrow originates on the link).
    * ``T[3, 3]`` — **sweep angle in radians** (positive = CCW, negative
      = CW).

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
        arc = np.column_stack([
            r * np.cos(angles),
            r * np.sin(angles),
            np.zeros(n_pts),
        ])

        tip_c = np.cos(sweep)
        tip_s = np.sin(sweep)
        tip = np.array([r * tip_c, r * tip_s, 0.0])

        if sweep > 0:
            barb1 = tip + np.array([-d/2*tip_c + d/2*tip_s,
                                    -d/2*tip_s - d/2*tip_c, 0.0])
            barb2 = tip + np.array([ d/2*tip_c + d/2*tip_s,
                                     d/2*tip_s - d/2*tip_c, 0.0])
        else:
            barb1 = tip + np.array([-d/2*tip_c - d/2*tip_s,
                                    -d/2*tip_s + d/2*tip_c, 0.0])
            barb2 = tip + np.array([ d/2*tip_c - d/2*tip_s,
                                     d/2*tip_s + d/2*tip_c, 0.0])

        return np.vstack([arc, np.array([barb1, tip, barb2])])


######################################################################
# Transformation Matrix Helpers
######################################################################


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


def torque_pose2d_matrix(x=0.0, y=0.0, start_angle=0.0, sweep=0.0):
    """4x4 matrix for :class:`TorqueArrow`.

    * Rotation (upper-left 2×2) = *start_angle* — orients the arc so it
      begins along this direction (typically the rod angle so the arrow
      originates on the link).
    * Translation = *(x, y)* — arc centre (joint position in world).
    * ``T[3, 3]`` = *sweep* — arc sweep in radians (+ CCW, − CW),
      passed through the amplitude channel.
    """
    T = pose2d_matrix(x, y, start_angle)
    T[3, 3] = sweep
    return T


def extract_amplitude(T):
    """Read and consume the amplitude channel from a 4×4 transform.

    In a standard homogeneous matrix ``T[3, 3] == 1``.  Systems that need
    to pass a scalar amplitude to the renderer (e.g. sweep angle, force
    magnitude) store it in this slot via helpers like
    :func:`torque_pose2d_matrix`.

    Returns ``(amplitude, T_clean)`` where *T_clean* has ``T[3, 3]``
    restored to 1 so it can be used as a normal rigid transform.
    """
    amplitude = T[3, 3]
    T_clean = T.copy()
    T_clean[3, 3] = 1.0
    return amplitude, T_clean
