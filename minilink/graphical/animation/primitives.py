"""Graphical primitives: shapes in their own local frame, and the 4x4 camera transforms."""

# NumPy only: the core kinematic hooks import this module, so it never pulls in
# matplotlib or another rendering library.

import numpy as np

###############################################################################
# Public API
###############################################################################


# Primitive shapes (local-frame geometry)


class GraphicPrimitive:
    """Base class for all geometric objects rendered by the animator engine.

    A system's visualization is a dict of named **frames** to **primitives**
    (shapes defined in their own local frame) plus, at every instant, one 4x4
    homogeneous **transform** per frame placing it in the world (see
    :meth:`minilink.core.system.System.get_kinematic_geometry`,
    :meth:`~minilink.core.system.System.get_dynamic_geometry`, and
    :meth:`~minilink.core.system.System.tf`). Renderers draw the primitives;
    they never know about states or inputs.

    Primitives are **honest**: they bake their full point geometry at real world
    size in their own local frame. ``local_transform`` is a fixed 4x4 graphical
    offset baked into the primitive (a constant pose within its frame), so the
    animator poses a primitive at ``frames[key] @ local_transform``; it defaults
    to identity. Placement helpers for spans and coils live in
    :mod:`minilink.graphical.catalog.shapes`; rigid pose math lives in
    :mod:`minilink.core.kinematics`.
    """

    def __init__(self, color="blue", linewidth=1, style="-", local_transform=None):
        # Style
        self.color = color
        self.linewidth = linewidth
        self.style = style

        # Pose within its frame
        self.local_transform = (
            np.eye(4) if local_transform is None else np.asarray(local_transform, float)
        )

        # False for backdrops (ground lines, planes) and force glyphs (arrows):
        # they never drive the auto-fit camera
        self.camera_fit = True


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

        # Shape
        self.pts = np.array(pts)

    def points_at(self, t):
        """Nx3 polyline points at playback time *t* (static: time-independent)."""
        return self.pts


class Point(GraphicPrimitive):
    """An individual point marker."""

    def __init__(self, pt=(0, 0, 0), color="red", marker="o", size=5):
        super().__init__(color)

        # Marker
        self.pt = np.array(pt)
        self.marker = marker
        self.size = size


class Circle(GraphicPrimitive):
    """A basic circle primitive. Lives in the XY plane by default."""

    def __init__(self, radius=1.0, center=(0, 0, 0), color="blue", fill=False):
        super().__init__(color)

        # Shape
        self.radius = radius
        self.center = np.array(center)
        self.fill = fill


class Sphere(GraphicPrimitive):
    """A 3D sphere primitive centered at ``center`` in local frame."""

    def __init__(self, radius=1.0, center=(0, 0, 0), color="blue", opacity=1.0):
        super().__init__(color)

        # Shape
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

        # Shape
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

        # A backdrop: it never drives the auto-fit camera
        self.camera_fit = False

        # Shape: the patch of n·x = offset
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

        # Shape
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

        # Outline: at least three XY points, a closing repeat of the first dropped
        pts = np.asarray(pts_xy, dtype=float).reshape(-1, 2)
        require_polygon(pts)
        if np.allclose(pts[0], pts[-1]):
            pts = pts[:-1]
        self.pts_xy = pts

        # Extrusion
        self.height = float(height)
        self.center = np.asarray(center, dtype=float).reshape(3)
        self.opacity = float(opacity)

    def vertices_local(self) -> np.ndarray:
        """Return local vertices with shape ``(2*n, 3)``."""
        pts_xy = self.pts_xy
        center = self.center
        height = self.height

        n = pts_xy.shape[0]

        # The outline on the bottom and top faces, z = c_z ∓ h/2
        z0 = center[2] - 0.5 * height
        z1 = center[2] + 0.5 * height
        xy = pts_xy + center[:2]
        bottom = np.column_stack((xy, np.full(n, z0)))
        top = np.column_stack((xy, np.full(n, z1)))

        vertices = np.vstack((bottom, top))

        return vertices

    def edges(self) -> tuple[tuple[int, int], ...]:
        """Return wireframe edges as vertex-index pairs."""
        n = self.pts_xy.shape[0]

        # Each side i → j: its bottom edge, its top edge, and the upright at i
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

        # The two caps: a fan of triangles from their first vertex, the bottom one
        # wound to face down
        for i in range(1, n - 1):
            faces.append([0, i + 1, i])
            faces.append([n, n + i, n + i + 1])

        # The sides: two triangles per quad
        for i in range(n):
            j = (i + 1) % n
            faces.append([i, j, n + i])
            faces.append([j, n + j, n + i])

        triangles = np.asarray(faces, dtype=np.uint32)

        return vertices, triangles


class Arrow(GraphicPrimitive):
    """A straight arrow at honest world size — shaft from *base* along *vector*.

    Accepts planar (length-2) or spatial (length-3) *base* / *vector*. Geometry
    is baked at true length ``scale * |vector|``; the renderer simply draws
    ``self.pts`` posed by the frame. Pass body-frame ``vector`` and key the
    primitive to that body frame so its orientation comes from ``tf`` (no
    manual ``cos``/``sin`` rotation). For world-aligned task forces, key to a
    translation-only frame and pass the force components directly as *vector*.
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

        # Shaft: both planar or both spatial
        base = np.asarray(base, dtype=float).reshape(-1)
        vector = np.asarray(vector, dtype=float).reshape(-1)
        require_arrow_vectors(base, vector)
        self.base = base
        self.vector = vector
        self.scale = float(scale)
        self.head_ratio = head_ratio

        # Geometry, baked at its true length
        self.pts = arrow_pts(self.base, self.vector, self.scale, head_ratio)

        # A force glyph, not a body: it never drives the auto-fit camera
        self.camera_fit = False

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

        # Arc
        self.sweep = float(sweep)
        self.radius = radius
        self.head_ratio = head_ratio
        self.n_arc_pts = n_arc_pts

        # Geometry, baked at the given sweep
        self.pts = torque_arc_pts(self.sweep, radius, head_ratio, n_arc_pts)

        # A force glyph, not a body: it never drives the auto-fit camera
        self.camera_fit = False

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

        # The (t_solve, trajectory) pairs
        self.plans = list(plans)

    def compute_pts(self, t_now):
        """Return Nx3 world-frame polyline points for the active plan tail."""
        plans = self.plans

        t_now = float(t_now)

        # The active plan: the latest one solved by t_now, within 10⁻⁹
        active = None
        for t_solve, plan in plans:
            if t_solve <= t_now + 1e-9:
                active = plan
        if active is None:
            return np.zeros((1, 3))

        # Its tail: the samples from t_now on
        mask = active.t >= t_now - 1e-9
        pts = xy_polyline(active, mask)

        return pts

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

        # The trajectory and its time window
        self.trajectory = trajectory
        require_one_of("window", window, self._WINDOWS)
        self.window = window

    def compute_pts(self, t_now):
        """Return Nx3 world-frame polyline points for the selected time window."""
        traj = self.trajectory
        window = self.window

        t_now = float(t_now)

        # The samples in the window: all of them, those up to t_now, or those from
        # t_now on, within 10⁻⁹
        if window == "all":
            mask = np.ones(traj.n_samples, dtype=bool)
        elif window == "prefix":
            mask = traj.t <= t_now + 1e-9
        else:
            mask = traj.t >= t_now - 1e-9
        pts = xy_polyline(traj, mask)

        return pts

    def points_at(self, t):
        """Nx3 world-frame points of the selected time window at playback time *t*."""
        return self.compute_pts(t)


# Arrow and torque-arc geometry (baked into Arrow and TorqueArrow)


def arrow_pts(base, vector, scale=1.0, head_ratio=0.15):
    """Polyline (Nx3) of a straight arrow from *base* along *vector*.

    Accepts matching length-2 or length-3 *base* / *vector*. Planar arrows stay
    in local XY (``z = 0``); spatial arrows build the chevron in a plane spanned
    by the shaft and a deterministic perpendicular. Drawn length is
    ``scale * |vector|``. A near-zero vector collapses to a single point.

    Parameters
    ----------
    base : array-like, length 2 or 3
        Tail of the arrow in the local frame.
    vector : array-like, length 2 or 3
        Direction (and, with *scale*, length) of the shaft.
    scale : float
        Multiplies *vector* to set the drawn length (e.g. a force gain).
    head_ratio : float
        Chevron barb length as a fraction of the shaft length.
    """
    base = np.asarray(base, dtype=float).reshape(-1)
    vector = np.asarray(vector, dtype=float).reshape(-1)
    require_arrow_vectors(base, vector)

    # The tail p and the shaft s = scale v, lifted to z = 0 when planar
    planar = base.size == 2
    if planar:
        base3 = np.array([base[0], base[1], 0.0])
        shaft = scale * np.array([vector[0], vector[1], 0.0])
    else:
        base3 = base.copy()
        shaft = scale * vector

    # A vanishing shaft collapses to its tail
    length = float(np.linalg.norm(shaft))
    if length < 1e-12:
        return base3.reshape(1, 3)

    # The tip p + s, and the barbs tip − d ŝ ± d n̂, with d = head_ratio ‖s‖ and n̂ ⟂ ŝ
    tip = base3 + shaft
    direction = shaft / length
    d = head_ratio * length
    back = direction * d
    perp = arrow_perp(direction[:2] if planar else direction) * d

    barb1 = tip - back + perp
    barb2 = tip - back - perp

    # Tail to tip, then the chevron
    pts = np.array([base3, tip, barb1, tip, barb2])

    return pts


def torque_arc_pts(sweep, radius=1.0, head_ratio=0.4, n_arc_pts=40):
    """Polyline (Nx3) of a circular torque arc + chevron, centered at the origin.

    The arc is baked at the given *sweep* (radians, + = CCW) and *radius*, so the
    sweep rides in the geometry rather than a transform side-channel.
    """
    sweep = float(sweep)
    r = radius
    d = r * head_ratio

    # A vanishing sweep collapses to the centre
    if abs(sweep) < 1e-6:
        return np.zeros((1, 3))

    # The arc r (cos φ, sin φ), φ from 0 to the sweep θ, sampled in proportion to
    # the fraction of a full turn
    n_pts = max(3, int(abs(sweep) / (2 * np.pi) * n_arc_pts))
    angles = np.linspace(0.0, sweep, n_pts)
    arc = np.column_stack([r * np.cos(angles), r * np.sin(angles), np.zeros(n_pts)])

    # The tip r (cos θ, sin θ), and the barbs tip + (d/2)(∓r̂ − ŵ), with r̂ the
    # radial (cos θ, sin θ) and ŵ = ±(−sin θ, cos θ) the direction of travel
    tip_c = np.cos(sweep)
    tip_s = np.sin(sweep)
    tip = np.array([r * tip_c, r * tip_s, 0.0])
    if sweep > 0:
        barb1_x = -d / 2 * tip_c + d / 2 * tip_s
        barb1_y = -d / 2 * tip_s - d / 2 * tip_c
        barb2_x = d / 2 * tip_c + d / 2 * tip_s
        barb2_y = d / 2 * tip_s - d / 2 * tip_c
    else:
        barb1_x = -d / 2 * tip_c - d / 2 * tip_s
        barb1_y = -d / 2 * tip_s + d / 2 * tip_c
        barb2_x = d / 2 * tip_c - d / 2 * tip_s
        barb2_y = d / 2 * tip_s + d / 2 * tip_c

    barb1 = tip + np.array([barb1_x, barb1_y, 0.0])
    barb2 = tip + np.array([barb2_x, barb2_y, 0.0])

    # The arc, then the chevron at its tip
    pts = np.vstack([arc, np.array([barb1, tip, barb2])])

    return pts


# Transformation matrix helpers (camera and identity only; pose math → core.kinematics)


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
    i, j = plot_axes
    require_plot_axes(plot_axes, i, j)

    # The rotation's columns: plot-X e_i, plot-Y e_j, and the view-out axis e_i × e_j
    e = np.eye(3)
    T = np.eye(4)
    T[:3, 0] = e[i]
    T[:3, 1] = e[j]
    T[:3, 2] = np.cross(e[i], e[j])

    # The look-at target, and the view scale in the T[3, 3] slot
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

    # W = [Rᵀ, −Rᵀ p; 0, 1], the inverse of the rigid pose (R, p)
    W = np.eye(4)
    W[:3, :3] = R.T
    W[:3, 3] = -R.T @ target

    return W


def identity_matrix():
    """4x4 identity transform (primitive drawn at the world origin)."""
    return np.eye(4)


def empty_transform():
    """Transform that parks a primitive far off-screen (used to hide it)."""
    # Lazy import: the core kinematics load on first use, not with the primitives
    from minilink.core.kinematics import translation

    return translation(0.0, 0.0, -1000.0)


# Ready-made shapes and poses


def ground_line(length=20.0, y=0.0, color="black", style="--"):
    """Horizontal reference line of span *length* at height *y* (e.g. ground)."""
    line = CustomLine(
        [[-0.5 * length, y, 0.0], [0.5 * length, y, 0.0]],
        color=color,
        linewidth=1,
        style=style,
    )

    # A backdrop: it never drives the auto-fit camera
    line.camera_fit = False

    return line


def spring_line(coils=6, amplitude=0.12, color="black", linewidth=1):
    """Unit-length zig-zag spring along local +X (lead-in, *coils* coils, lead-out).

    Drawn from x=0 to x=1; pair with a transform that spans the two endpoints.
    """
    # Lead-in
    pts = [[0.0, 0.0, 0.0], [0.15, 0.0, 0.0]]

    # The coils: 2·coils + 1 vertices over x ∈ [0.2, 0.8], alternating at ∓amplitude
    xs = np.linspace(0.2, 0.8, 2 * coils + 1)
    for i, x in enumerate(xs):
        y = amplitude if i % 2 else -amplitude
        pts.append([x, y, 0.0])

    # Lead-out
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


# Extents (used by the auto-fit camera)


def bounding_points(primitive):
    """Local points that bound the primitive, plus a radial pad, or ``None`` when excluded.

    Returns ``(pts, pad)``: an ``(N, 3)`` array of local-frame points and a
    scalar margin to add around them (a rod's radius, a sphere's radius). The
    auto-fit camera transforms the points into the world and pads the box.
    Backdrops and force glyphs (``primitive.camera_fit`` is ``False``) return ``None``;
    unknown primitives count as their origin only.
    """
    if not getattr(primitive, "camera_fit", True):
        return None
    origin = np.zeros((1, 3))

    # A sphere: its centre, padded by its radius
    if isinstance(primitive, Sphere):
        pts = point3(primitive.center).reshape(1, 3)
        pad = float(primitive.radius)
        return pts, pad

    # A circle: the four points of its rim on the local X and Y axes
    if isinstance(primitive, Circle):
        c = point3(primitive.center)
        r = float(primitive.radius)
        pts = c + np.array([[r, 0, 0], [-r, 0, 0], [0, r, 0], [0, -r, 0]])
        return pts, 0.0

    # A rod: its two ends along −Y, padded by its radius
    if isinstance(primitive, Rod):
        pts = np.array([[0.0, 0.0, 0.0], [0.0, -primitive.length, 0.0]])
        pad = float(primitive.radius)
        return pts, pad

    # A box: its eight corners, the centre ± half the full lengths
    if isinstance(primitive, Box):
        half = 0.5 * np.array(
            [primitive.length_x, primitive.length_y, primitive.length_z]
        )
        signs = np.array(
            [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        )
        pts = point3(primitive.center) + signs * half
        return pts, 0.0

    # An extruded polygon: its outline on both faces, z = ±h/2 about the centre
    if isinstance(primitive, ExtrudedPolygon):
        xy = np.asarray(primitive.pts_xy, dtype=float)
        h = 0.5 * float(primitive.height)
        corners = np.vstack(
            [np.column_stack([xy, np.full(len(xy), z)]) for z in (-h, h)]
        )
        pts = point3(primitive.center) + corners
        return pts, 0.0

    # A point marker: the point itself
    if isinstance(primitive, Point):
        pts = point3(primitive.pt).reshape(1, 3)
        return pts, 0.0

    # Any primitive with baked points: those points, lifted to 3D
    pts = getattr(primitive, "pts", None)
    if pts is not None:
        pts = np.asarray(pts, dtype=float)
        if pts.size:
            pts = pts.reshape(len(pts), -1)
            if pts.shape[1] == 2:
                pts = np.column_stack([pts, np.zeros(len(pts))])
            return pts[:, :3], 0.0

    # Anything else: its origin only
    return origin, 0.0


def bounding_radius(primitive):
    """Radius of a ball about the local origin containing the primitive (``None`` when excluded)."""
    bounds = bounding_points(primitive)
    if bounds is None:
        return None

    pts, pad = bounds

    # r = maxᵢ ‖pᵢ‖ + pad
    radius = float(np.max(np.linalg.norm(pts, axis=1)) + pad)

    return radius


###############################################################################
# Internal machinery
###############################################################################


def require_polygon(pts):
    """Refuse an outline of fewer than three XY points."""
    if pts.shape[0] < 3:
        raise ValueError("ExtrudedPolygon requires at least 3 XY points")


def require_arrow_vectors(base, vector):
    """Refuse a base and a vector that are not both of length 2 or both of length 3."""
    if base.size not in (2, 3) or vector.size not in (2, 3) or base.size != vector.size:
        raise ValueError(
            f"base and vector must both have length 2 or 3, got {base.size} and "
            f"{vector.size}"
        )


def xy_polyline(trajectory, mask):
    """The XY rows of ``trajectory.x`` under ``mask``, as an Nx3 polyline at ``z = 0``.

    Fewer than two samples draw no line: the polyline is then the origin alone.
    """
    if np.count_nonzero(mask) < 2:
        return np.zeros((1, 3))

    xy = trajectory.x[:2, mask]
    pts = np.column_stack([xy[0], xy[1], np.zeros(xy.shape[1])])

    return pts


def require_one_of(name, value, choices):
    """Refuse an option outside its known choices, naming them."""
    if value not in choices:
        raise ValueError(f"{name} must be one of {choices}, got {value!r}")


def arrow_perp(direction):
    """Stable unit perpendicular for a 2-D or 3-D arrow shaft."""
    direction = np.asarray(direction, dtype=float).reshape(-1)

    # In the plane: the shaft turned a quarter turn, (−d_y, d_x, 0)
    if direction.size == 2:
        perp = np.array([-direction[1], direction[0], 0.0])
        return perp

    # In space: n̂ = (e × d) / (‖e × d‖ + 10⁻¹²), against e = e_z, or e_y when the
    # shaft is nearly vertical
    reference = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(direction, reference)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    normal = np.cross(reference, direction)
    perp = normal / (np.linalg.norm(normal) + 1e-12)

    return perp


def require_plot_axes(plot_axes, i, j):
    """Refuse plot axes that are not two distinct world axes in ``{0, 1, 2}``."""
    if i == j or i not in (0, 1, 2) or j not in (0, 1, 2):
        raise ValueError(
            "plot_axes must be two distinct world axis indices in {0, 1, 2}; "
            f"got {plot_axes!r}."
        )


def point3(value):
    """A local point as a length-3 float array (planar ``(x, y)`` gets ``z = 0``)."""
    v = np.asarray(value, dtype=float).reshape(-1)
    if v.size == 2:
        v = np.append(v, 0.0)
    return v[:3]
