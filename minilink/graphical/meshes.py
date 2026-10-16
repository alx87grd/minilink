"""Triangle meshes for minilink skins: STL / COLLADA readers and a mesh primitive."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from minilink.graphical.animation.primitives import ExtrudedPolygon

###############################################################################
# Public API
###############################################################################


# The primitive


class TriangleMesh(ExtrudedPolygon):
    """A triangle mesh at real size in its local frame (a URDF ``<mesh>`` visual).

    minilink has no mesh primitive of its own: its renderers dispatch on the
    primitive class, and the one class every renderer already knows as a triangle
    mesh is :class:`~minilink.graphical.animation.primitives.ExtrudedPolygon`.
    ``TriangleMesh`` subclasses it and overrides the members they read, so a URDF
    mesh plugs into every backend unchanged:

    - meshcat draws ``mesh_data() -> (vertices, faces)`` as a
      ``TriangularMeshGeometry`` (a lit solid);
    - matplotlib and plotly draw a wireframe from ``vertices_local()`` and
      ``edges()``;
    - the auto-fit camera bounds it with ``pts_xy``, ``height`` and ``center``.

    The wireframe keeps the feature edges only (creases and open borders), which
    keeps the matplotlib path readable; a smooth mesh (a tire, a hinge sphere)
    relaxes the crease angle so that it still shows. The shaded matplotlib backend
    of this package draws it as filled, lit triangles.

    Parameters
    ----------
    vertices : array-like, shape (k, 3)
        Vertex positions [m] in the primitive's local frame.
    faces : array-like, shape (n, 3)
        Triangle vertex indices.
    color, opacity :
        Flat material, as for the other minilink solids.
    crease_deg, max_edges :
        Wireframe selection (see :func:`wireframe_edges`). A smooth mesh (tire,
        sphere) relaxes *crease_deg* until it shows; ``max_edges=0`` draws no
        wireframe (floor tiles, tread lugs).
    name : str, optional
        Label (the URDF link name) for inspection.
    """

    def __init__(
        self,
        vertices,
        faces,
        color="gray",
        opacity=1.0,
        crease_deg=40.0,
        max_edges=160,
        name=None,
    ):
        vertices = np.asarray(vertices, dtype=float).reshape(-1, 3)
        faces = np.asarray(faces, dtype=np.uint32).reshape(-1, 3)

        # The bounding box, as the extruded footprint the auto-fit camera reads
        lo = vertices.min(axis=0)
        hi = vertices.max(axis=0)
        footprint = np.array(
            [[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]]
        )
        super().__init__(
            footprint,
            height=float(hi[2] - lo[2]),
            center=(0.0, 0.0, 0.5 * float(lo[2] + hi[2])),
            color=color,
            opacity=opacity,
        )
        self.name = name

        # The mesh
        self.vertices = vertices
        self.faces = faces

        # The wireframe: open borders and creases only
        self._wire_vertices, self._wire_edges = wireframe_edges(
            vertices, faces, crease_deg=crease_deg, max_edges=max_edges
        )

        # Memory
        self._lod = {}  # Decimated meshes by cell size, built on first use

    def mesh_data(self):
        """``(vertices, faces)`` for triangle-mesh renderers (meshcat)."""
        return self.vertices, self.faces

    def vertices_local(self):
        """Wireframe vertices (welded mesh vertices)."""
        return self._wire_vertices

    def edges(self):
        """Wireframe edges: open borders and creases only."""
        return tuple(map(tuple, self._wire_edges))

    def simplified(self, cell):
        """``(vertices, faces)`` decimated on a *cell* grid [m] (cached)."""
        if cell not in self._lod:
            self._lod[cell] = simplify(*weld(self.vertices, self.faces), cell)

        return self._lod[cell]


# Mesh files
#
# The readers need NumPy and the standard library only (no trimesh, no pycollada):
# binary and ASCII STL, and COLLADA ``triangles`` / ``polylist`` / ``polygons``.


def load_stl(path, scale=1.0):
    """Read a binary or ASCII STL file.

    Parameters
    ----------
    path : str or Path
        STL file.
    scale : float or array-like of 3
        URDF ``<mesh scale>`` applied to the vertices.

    Returns
    -------
    vertices : np.ndarray, shape (3 n, 3)
        One vertex per triangle corner (STL is a triangle soup), in file units.
    faces : np.ndarray, shape (n, 3), uint32
        Vertex indices, ``faces[k] = (3k, 3k + 1, 3k + 2)``.
    """
    data = Path(path).read_bytes()

    # Binary: an 80-byte header, the triangle count n, then n records of 50 bytes
    n = int(np.frombuffer(data[80:84], dtype="<u4")[0]) if len(data) >= 84 else -1
    if len(data) == 84 + 50 * n:
        record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
        triangles = np.frombuffer(data[84:], dtype=record, count=n)["v"]
    else:
        # ASCII: the three coordinates after every ``vertex`` keyword
        words = data.decode("ascii", errors="ignore").split()
        coords = [
            [float(words[i + 1]), float(words[i + 2]), float(words[i + 3])]
            for i, word in enumerate(words)
            if word == "vertex"
        ]
        triangles = np.asarray(coords, dtype=float).reshape(-1, 3, 3)

    # One vertex per triangle corner, scaled: triangle k is (3k, 3k + 1, 3k + 2)
    vertices = triangles.reshape(-1, 3).astype(float) * np.asarray(scale, dtype=float)
    faces = np.arange(len(vertices), dtype=np.uint32).reshape(-1, 3)

    return vertices, faces


def load_collada(path, scale=1.0):
    """Read the triangles of a COLLADA (``.dae``) file, one part per material.

    Node transforms (``matrix``, ``translate``, ``rotate``, ``scale``) and the
    ``<unit meter>`` factor are applied, so the result is in metres in the file's
    scene frame. Polygons are fan-triangulated.

    Returns
    -------
    list of (vertices, faces, rgba)
        ``vertices`` (k, 3) float, ``faces`` (n, 3) uint32 and the diffuse colour
        ``rgba`` of the bound material (``None`` when the file gives none).
    """
    root = ET.parse(path).getroot()
    ns = root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""

    def find(node, tag):
        return node.find(ns + tag)

    def children(node, tag):
        return node.findall(ns + tag)

    # The file's length unit, in metres
    unit = root.find(f"{ns}asset/{ns}unit")
    meter = float(unit.get("meter", 1.0)) if unit is not None else 1.0

    # Material id → diffuse RGBA, through the material's effect
    effects = {}
    for effect in root.iter(ns + "effect"):
        diffuse = effect.find(f".//{ns}diffuse/{ns}color")
        if diffuse is not None:
            effects[effect.get("id")] = np.array(diffuse.text.split(), dtype=float)

    materials = {}
    for material in root.iter(ns + "material"):
        instance = find(material, "instance_effect")
        if instance is not None:
            materials[material.get("id")] = effects.get(instance.get("url", "#")[1:])

    geometries = {g.get("id"): g for g in root.iter(ns + "geometry")}

    # A node's own 4×4: the product of its transform items, in document order
    def node_transform(node):
        T = np.eye(4)
        for item in node:
            tag = item.tag.replace(ns, "")
            values = np.array(item.text.split(), dtype=float) if item.text else None
            if tag == "matrix":
                T = T @ values.reshape(4, 4)
            elif tag == "translate":
                T = T @ translation_matrix(values)
            elif tag == "rotate":
                T = T @ axis_angle_matrix(values[:3], np.radians(values[3]))
            elif tag == "scale":
                T = T @ np.diag([*values, 1.0])
        return T

    # The scene graph, depth first: every geometry instance posed by its node chain
    parts = []

    def visit(node, T_parent):
        T = T_parent @ node_transform(node)
        for instance in children(node, "instance_geometry"):
            symbols = {
                im.get("symbol"): im.get("target", "#")[1:]
                for im in instance.iter(ns + "instance_material")
            }
            geometry = geometries[instance.get("url")[1:]]
            for vertices, faces, symbol in collada_mesh(geometry, ns):
                rgba = materials.get(symbols.get(symbol, symbol))
                world = vertices @ T[:3, :3].T + T[:3, 3]
                parts.append((world * meter * np.asarray(scale, float), faces, rgba))
        for child in children(node, "node"):
            visit(child, T)

    for scene in root.iter(ns + "visual_scene"):
        for node in children(scene, "node"):
            visit(node, np.eye(4))

    return parts


# Mesh processing


def weld(vertices, faces, tol=1e-6):
    """Merge the vertices that round to one point of a *tol* grid (STL soups share none)."""
    # The grid key of every vertex
    keys = np.round(np.asarray(vertices) / tol).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)

    # The first vertex of each key, and the faces renumbered onto them
    welded_vertices = np.asarray(vertices)[first]
    welded_faces = inverse.reshape(-1)[faces].astype(np.uint32)

    return welded_vertices, welded_faces


def simplify(vertices, faces, cell):
    """Vertex-clustering decimation on a grid of size *cell* [m].

    Every vertex moves to the mean of its grid cell and collapsed triangles are
    dropped. Crude but robust: enough for the small parts of a 1/10 car drawn by
    matplotlib, where every triangle is a Python-side polygon.
    """
    vertices = np.asarray(vertices, dtype=float)

    # One cluster per occupied grid cell
    keys = np.floor(vertices / cell).astype(np.int64)
    _, cluster = np.unique(keys, axis=0, return_inverse=True)
    cluster = cluster.reshape(-1)
    n = cluster.max() + 1

    # Each cluster's vertex: the mean of its members
    counts = np.bincount(cluster, minlength=n)[:, None]
    means = np.stack(
        [np.bincount(cluster, weights=vertices[:, k], minlength=n) for k in range(3)],
        axis=1,
    )
    means /= np.maximum(counts, 1)

    # The faces on the clusters, without the collapsed ones and the repeats
    f = cluster[faces]
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = np.unique(np.sort(f[keep], axis=1), axis=0) if keep.any() else f[:0]
    simplified_faces = f.astype(np.uint32)

    return means, simplified_faces


def refine(vertices, faces, max_edge):
    """Midpoint-subdivide triangles until every edge is shorter than *max_edge* [m].

    Returns a triangle soup ``(vertices, faces)``. Used by painter's-algorithm
    renderers, where one large triangle sorted by its centroid can cover the
    small parts next to it.
    """
    triangles = np.asarray(vertices, dtype=float)[np.asarray(faces)]
    done = []

    # Split every triangle with a long edge into four, until none is left
    while len(triangles):
        # The longest edge of each triangle sets it aside, or splits it
        lengths = np.linalg.norm(triangles - np.roll(triangles, 1, axis=1), axis=2)
        longest = lengths.max(axis=1)
        small = longest <= max_edge
        done.append(triangles[small])
        big = triangles[~small]
        if not len(big):
            break

        # The edge midpoints cut each big triangle into four
        a = big[:, 0]
        b = big[:, 1]
        c = big[:, 2]
        ab = 0.5 * (a + b)
        bc = 0.5 * (b + c)
        ca = 0.5 * (c + a)
        triangles = np.concatenate(
            [
                np.stack(t, axis=1)
                for t in ((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca))
            ]
        )

    # The finished triangles as a soup
    soup = np.concatenate(done) if done else np.zeros((0, 3, 3))
    soup_vertices = soup.reshape(-1, 3)
    soup_faces = np.arange(3 * len(soup), dtype=np.uint32).reshape(-1, 3)

    return soup_vertices, soup_faces


def face_normals(vertices, faces):
    """Unit normals of the triangles (right-hand rule on the vertex order)."""
    v = np.asarray(vertices, dtype=float)[faces]

    # n = (v₁ − v₀) × (v₂ − v₀), over its norm (floored at 10⁻¹⁵: a degenerate triangle)
    n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    normals = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-15)

    return normals


def feature_edges(vertices, faces, crease_deg=40.0, max_edges=None):
    """Edges worth drawing in a wireframe: open borders and creases.

    An edge is kept when it bounds one triangle only, or when its two triangles
    meet at more than *crease_deg*. With *max_edges*, the longest ones are kept.
    """
    vertices, faces = weld(vertices, faces)
    normals = face_normals(vertices, faces)

    # Every triangle's three edges, sorted, grouped, and each with its triangle
    edges = np.sort(faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    owner = np.repeat(np.arange(len(faces)), 3)
    order = np.lexsort((edges[:, 1], edges[:, 0]))
    edges = edges[order]
    owner = owner[order]
    unique, start, count = np.unique(
        edges, axis=0, return_index=True, return_counts=True
    )

    # An open border bounds one triangle; a crease joins two at |n₀ · n₁| < cos θ_crease
    keep = count == 1
    shared = count == 2
    n0 = normals[owner[start[shared]]]
    n1 = normals[owner[start[shared] + 1]]
    cos_angle = np.abs(np.sum(n0 * n1, axis=1))
    crease = np.zeros_like(keep)
    crease[np.flatnonzero(shared)] = cos_angle < np.cos(np.radians(crease_deg))
    selected = unique[keep | crease]

    # At most max_edges of them: the longest
    if max_edges is not None and len(selected) > max_edges:
        length = np.linalg.norm(
            vertices[selected[:, 0]] - vertices[selected[:, 1]], axis=1
        )
        selected = selected[np.argsort(-length)[:max_edges]]

    return vertices, selected


def wireframe_edges(
    vertices, faces, crease_deg=40.0, max_edges=None, min_edges=12, min_crease_deg=5.0
):
    """:func:`feature_edges` that keeps smooth meshes visible.

    A smooth mesh (a tire, a sphere) has no crease sharper than *crease_deg*,
    so it would vanish from a wireframe. While fewer than *min_edges* edges
    pass (or *max_edges*, if smaller), *crease_deg* is halved, down to
    *min_crease_deg*. ``max_edges=0`` asks for no wireframe and is kept.
    """
    wire_vertices, edges = feature_edges(vertices, faces, crease_deg, max_edges)

    # Halve the crease angle while too few edges show
    wanted = min_edges if max_edges is None else min(min_edges, max_edges)
    while len(edges) < wanted and crease_deg > min_crease_deg:
        crease_deg = max(0.5 * crease_deg, min_crease_deg)
        wire_vertices, edges = feature_edges(vertices, faces, crease_deg, max_edges)

    return wire_vertices, edges


def split_by_radius(vertices, faces, axis, radius):
    """Split a wheel mesh into ``(inner, outer)`` parts by face-centroid radius.

    *axis* is the wheel spin axis (unit vector, local frame) through the origin.
    Used to colour the rim and the tire separately.
    """
    v = np.asarray(vertices, dtype=float)
    centroid = v[faces].mean(axis=1)
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)

    # The centroid's distance to the axis, ‖c − (c · a) a‖, against the radius
    radial = centroid - np.outer(centroid @ axis, axis)
    inner = np.linalg.norm(radial, axis=1) < radius

    inner_part = (v, faces[inner])
    outer_part = (v, faces[~inner])

    return inner_part, outer_part


# Solid builders: (vertices, faces) pairs, outward winding, in metres


def box_mesh(size, center=(0.0, 0.0, 0.0)):
    """Axis-aligned box of full lengths ``size = (lx, ly, lz)`` centred at *center*."""
    half = 0.5 * np.asarray(size, dtype=float)
    signs = np.array(
        [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    )
    vertices = signs * half + np.asarray(center, dtype=float)

    # The six sides as quads, wound outward; corner index = 4 ix + 2 iy + iz
    quads = [
        (0, 1, 3, 2),
        (4, 6, 7, 5),
        (0, 4, 5, 1),
        (2, 3, 7, 6),
        (0, 2, 6, 4),
        (1, 5, 7, 3),
    ]

    # Each quad (a, b, c, d) as the triangles (a, b, c) and (a, c, d)
    abc = [(a, b, c) for a, b, c, d in quads]
    acd = [(a, c, d) for a, b, c, d in quads]
    faces = np.asarray(abc + acd, dtype=np.uint32)

    return vertices, faces


def ring_mesh(r_in, r_out, z0, z1, n=32):
    """Annular cylinder about local z, radii *r_in* < *r_out*, from *z0* to *z1*.

    ``r_in = 0`` gives a closed disk (a short cylinder).
    """
    angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    circle = np.column_stack([np.cos(angles), np.sin(angles)])

    # Four rings of n vertices: outer bottom, outer top, inner top, inner bottom (the
    # inner radius floored at 10⁻⁹)
    rings = [(r_out, z0), (r_out, z1), (max(r_in, 1e-9), z1), (max(r_in, 1e-9), z0)]
    vertices = np.vstack(
        [np.column_stack([r * circle, np.full(n, z)]) for r, z in rings]
    )

    # Each ring joined to the next by two triangles per segment: the outer wall, the
    # top annulus, the inner wall and the bottom annulus
    triangles = []
    for k in range(4):
        a = k * n
        b = ((k + 1) % 4) * n
        for i in range(n):
            j = (i + 1) % n
            triangles += [(a + i, a + j, b + j), (a + i, b + j, b + i)]

    faces = np.asarray(triangles, dtype=np.uint32)

    return vertices, faces


def cylinder_mesh(radius, z0, z1, n=32):
    """Closed cylinder about local z from *z0* to *z1*."""
    return ring_mesh(0.0, radius, z0, z1, n=n)


def transform_mesh(mesh, T):
    """Apply the 4x4 pose *T* to a ``(vertices, faces)`` pair."""
    vertices, faces = mesh
    T = np.asarray(T, dtype=float)

    # x' = R x + p, row by row
    moved = np.asarray(vertices) @ T[:3, :3].T + T[:3, 3]

    return moved, faces


def merge_meshes(*meshes):
    """Concatenate ``(vertices, faces)`` pairs into one mesh."""
    vertices = []
    faces = []
    offset = 0

    # Each mesh's faces shifted past the vertices before it
    for v, f in meshes:
        vertices.append(np.asarray(v, dtype=float))
        faces.append(np.asarray(f, dtype=np.int64) + offset)
        offset += len(v)

    merged_vertices = np.vstack(vertices)
    merged_faces = np.vstack(faces).astype(np.uint32)

    return merged_vertices, merged_faces


###############################################################################
# Internal machinery
###############################################################################


def translation_matrix(p):
    """The 4×4 pure translation by ``p``."""
    T = np.eye(4)
    T[:3, 3] = p

    return T


def axis_angle_matrix(axis, angle):
    """The 4×4 rotation of ``angle`` about ``axis`` (Rodrigues' formula)."""
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)

    kx = axis[0]
    ky = axis[1]
    kz = axis[2]

    # K = [k]ₓ, the cross-product matrix of the unit axis
    # fmt: off
    K = np.array([
        [0.0, -kz, ky],
        [kz, 0.0, -kx],
        [-ky, kx, 0.0],
    ])
    # fmt: on

    # R = I + sin θ K + (1 − cos θ) K²
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * K @ K

    return T


def collada_mesh(geometry, ns):
    """Yield ``(vertices, faces, material_symbol)`` per primitive list of a geometry."""
    mesh = geometry.find(ns + "mesh")

    # The float sources, each one reshaped by its accessor's stride
    sources = {}
    for source in mesh.findall(ns + "source"):
        array = source.find(ns + "float_array")
        accessor = source.find(f"{ns}technique_common/{ns}accessor")
        stride = int(accessor.get("stride", 3)) if accessor is not None else 3
        sources[source.get("id")] = np.array(array.text.split(), dtype=float).reshape(
            -1, stride
        )

    # The source of each <vertices> element's positions
    vertex_source = {}
    for vertices in mesh.findall(ns + "vertices"):
        for inp in vertices.findall(ns + "input"):
            if inp.get("semantic") == "POSITION":
                vertex_source[vertices.get("id")] = inp.get("source")[1:]

    for prim in mesh:
        kind = prim.tag.replace(ns, "")
        if kind not in ("triangles", "polylist", "polygons"):
            continue

        # The positions, and where the vertex index sits in each interleaved tuple
        inputs = prim.findall(ns + "input")
        n_offsets = 1 + max(int(i.get("offset", 0)) for i in inputs)
        vertex_input = next(i for i in inputs if i.get("semantic") == "VERTEX")
        offset = int(vertex_input.get("offset", 0))
        positions = sources[vertex_source[vertex_input.get("source")[1:]]][:, :3]

        # The polygons' corner indices: one <p> per polygon, or one <p> cut by vcount
        if kind == "polygons":
            polygons = [
                np.array(p.text.split(), dtype=int).reshape(-1, n_offsets)[:, offset]
                for p in prim.findall(ns + "p")
            ]
        else:
            indices = np.array(prim.find(ns + "p").text.split(), dtype=int)
            corners = indices.reshape(-1, n_offsets)[:, offset]
            if kind == "triangles":
                counts = np.full(len(corners) // 3, 3)
            else:
                counts = np.array(prim.find(ns + "vcount").text.split(), dtype=int)
            polygons = np.split(corners, np.cumsum(counts)[:-1])

        # Fan triangulation: (p₀, p_k, p_{k+1}) for k = 1, …, len − 2
        triangles = [
            (polygon[0], polygon[k], polygon[k + 1])
            for polygon in polygons
            for k in range(1, len(polygon) - 1)
        ]
        faces = np.asarray(triangles, dtype=np.uint32)

        yield positions, faces, prim.get("material")
