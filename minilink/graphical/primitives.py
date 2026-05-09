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


def camera_matrix(target=(0.0, 0.0, 0.0), plot_axes=(0, 1), scale=10.0, R=None):
    """Standard 4x4 camera transform.

    The matrix doubles as the camera's pose in the world frame **and** the
    renderer projection knob. The amplitude channel ``T[3, 3]`` carries the
    view scale (same side-channel convention as :func:`torque_pose2d_matrix`
    and :func:`extract_amplitude`).

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
    R : array-like 3x3, optional
        Explicit rotation; columns must be the world directions of camera-X,
        camera-Y, camera-Z. Overrides ``plot_axes`` when given (use this for
        orbit, isometric, or arbitrary view orientations).

    Returns
    -------
    np.ndarray
        4x4 camera transform.
    """
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = np.asarray(R, dtype=float).reshape(3, 3)
    else:
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


def attach_standard_camera(system, **kwargs):
    """
    Update :class:`~minilink.core.system.System` camera fields used by the default
    :meth:`~minilink.core.system.System.get_camera_transform`.

    Arguments match :func:`camera_matrix`. Omitted keys leave stored fields unchanged.
    Passing ``plot_axes`` clears :attr:`~minilink.core.system.System.camera_R`
    unless ``R`` is included in the same call.

    Parameters
    ----------
    system
        A :class:`~minilink.core.system.System` instance (has ``camera_*`` fields).
    kwargs
        Subset of ``target``, ``plot_axes``, ``scale``, ``R`` for :func:`camera_matrix`.

    Notes
    -----
    Subclasses that override ``get_camera_transform`` are unaffected unless they
    call ``super()`` or read the ``camera_*`` attributes.
    """
    if not hasattr(system, "camera_scale"):
        raise AttributeError(
            "attach_standard_camera expects a System with camera_target / "
            "camera_plot_axes / camera_scale / camera_R attributes."
        )
    if "target" in kwargs:
        system.camera_target = np.asarray(kwargs["target"], dtype=float).reshape(3)
    if "scale" in kwargs:
        system.camera_scale = float(kwargs["scale"])
    if "plot_axes" in kwargs:
        system.camera_plot_axes = tuple(kwargs["plot_axes"])
    if "R" in kwargs:
        R = kwargs["R"]
        system.camera_R = (
            None if R is None else np.asarray(R, dtype=float).reshape(3, 3)
        )
    elif "plot_axes" in kwargs:
        system.camera_R = None


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
