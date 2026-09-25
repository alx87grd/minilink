"""Triangle meshes for minilink skins: STL / COLLADA readers and a mesh primitive.

minilink has no mesh primitive yet. Its renderers dispatch on the primitive class,
and the one class every renderer already knows as a triangle mesh is
:class:`~minilink.graphical.animation.primitives.ExtrudedPolygon`:

- meshcat draws it from ``mesh_data() -> (vertices, faces)`` as a
  ``TriangularMeshGeometry`` (a lit solid);
- matplotlib and plotly draw it as a wireframe from ``vertices_local()`` and
  ``edges()``;
- the auto-fit camera bounds it with ``pts_xy``, ``height`` and ``center``.

:class:`TriangleMesh` subclasses it and overrides those four members, so a URDF
mesh plugs into every minilink backend unchanged. The wireframe uses feature
edges only (creases and open borders), which keeps the matplotlib path readable;
smooth meshes (tires, the hinge spheres) relax the crease angle so they still show.

The readers need NumPy and the standard library only (no trimesh, no pycollada):
binary and ASCII STL, and COLLADA ``triangles`` / ``polylist`` / ``polygons`` with
node transforms, ``<unit meter>`` scaling and per-material diffuse colours.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from minilink.graphical.animation.primitives import ExtrudedPolygon

# Mesh files


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
    n = int(np.frombuffer(data[80:84], dtype="<u4")[0]) if len(data) >= 84 else -1
    if len(data) == 84 + 50 * n:
        record = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
        triangles = np.frombuffer(data[84:], dtype=record, count=n)["v"]
    else:
        words = data.decode("ascii", errors="ignore").split()
        coords = [
            [float(words[i + 1]), float(words[i + 2]), float(words[i + 3])]
            for i, word in enumerate(words)
            if word == "vertex"
        ]
        triangles = np.asarray(coords, dtype=float).reshape(-1, 3, 3)
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

    unit = root.find(f"{ns}asset/{ns}unit")
    meter = float(unit.get("meter", 1.0)) if unit is not None else 1.0

    # material id -> diffuse rgba
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

    def node_transform(node):
        T = np.eye(4)
        for item in node:
            tag = item.tag.replace(ns, "")
            values = np.array(item.text.split(), dtype=float) if item.text else None
            if tag == "matrix":
                T = T @ values.reshape(4, 4)
            elif tag == "translate":
                T = T @ _translation(values)
            elif tag == "rotate":
                T = T @ _axis_angle(values[:3], np.radians(values[3]))
            elif tag == "scale":
                T = T @ np.diag([*values, 1.0])
        return T

    parts = []

    def visit(node, T_parent):
        T = T_parent @ node_transform(node)
        for instance in children(node, "instance_geometry"):
            symbols = {
                im.get("symbol"): im.get("target", "#")[1:]
                for im in instance.iter(ns + "instance_material")
            }
            geometry = geometries[instance.get("url")[1:]]
            for vertices, faces, symbol in _collada_mesh(geometry, ns):
                rgba = materials.get(symbols.get(symbol, symbol))
                world = vertices @ T[:3, :3].T + T[:3, 3]
                parts.append((world * meter * np.asarray(scale, float), faces, rgba))
        for child in children(node, "node"):
            visit(child, T)

    for scene in root.iter(ns + "visual_scene"):
        for node in children(scene, "node"):
            visit(node, np.eye(4))
    return parts


def _collada_mesh(geometry, ns):
    """Yield ``(vertices, faces, material_symbol)`` per primitive list of a geometry."""
    mesh = geometry.find(ns + "mesh")
    sources = {}
    for source in mesh.findall(ns + "source"):
        array = source.find(ns + "float_array")
        accessor = source.find(f"{ns}technique_common/{ns}accessor")
        stride = int(accessor.get("stride", 3)) if accessor is not None else 3
        sources[source.get("id")] = np.array(array.text.split(), dtype=float).reshape(
            -1, stride
        )
    vertex_source = {}
    for vertices in mesh.findall(ns + "vertices"):
        for inp in vertices.findall(ns + "input"):
            if inp.get("semantic") == "POSITION":
                vertex_source[vertices.get("id")] = inp.get("source")[1:]

    for prim in mesh:
        kind = prim.tag.replace(ns, "")
        if kind not in ("triangles", "polylist", "polygons"):
            continue
        inputs = prim.findall(ns + "input")
        n_offsets = 1 + max(int(i.get("offset", 0)) for i in inputs)
        vertex_input = next(i for i in inputs if i.get("semantic") == "VERTEX")
        offset = int(vertex_input.get("offset", 0))
        positions = sources[vertex_source[vertex_input.get("source")[1:]]][:, :3]

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

        faces = [
            (polygon[0], polygon[k], polygon[k + 1])
            for polygon in polygons
            for k in range(1, len(polygon) - 1)
        ]
        yield positions, np.asarray(faces, dtype=np.uint32), prim.get("material")


def _translation(p):
    T = np.eye(4)
    T[:3, 3] = p
    return T


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    K = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * K @ K
    return T


# Mesh processing


def weld(vertices, faces, tol=1e-6):
    """Merge vertices closer than *tol* (STL soups share no vertices)."""
    keys = np.round(np.asarray(vertices) / tol).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    return np.asarray(vertices)[first], inverse.reshape(-1)[faces].astype(np.uint32)


def simplify(vertices, faces, cell):
    """Vertex-clustering decimation on a grid of size *cell* [m].

    Every vertex moves to the mean of its grid cell and collapsed triangles are
    dropped. Crude but robust: enough for the small parts of a 1/10 car drawn by
    matplotlib, where every triangle is a Python-side polygon.
    """
    vertices = np.asarray(vertices, dtype=float)
    keys = np.floor(vertices / cell).astype(np.int64)
    _, cluster = np.unique(keys, axis=0, return_inverse=True)
    cluster = cluster.reshape(-1)
    n = cluster.max() + 1
    counts = np.bincount(cluster, minlength=n)[:, None]
    means = np.stack(
        [np.bincount(cluster, weights=vertices[:, k], minlength=n) for k in range(3)],
        axis=1,
    )
    means /= np.maximum(counts, 1)
    f = cluster[faces]
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = np.unique(np.sort(f[keep], axis=1), axis=0) if keep.any() else f[:0]
    return means, f.astype(np.uint32)


def refine(vertices, faces, max_edge):
    """Midpoint-subdivide triangles until every edge is shorter than *max_edge* [m].

    Returns a triangle soup ``(vertices, faces)``. Used by painter's-algorithm
    renderers, where one large triangle sorted by its centroid can cover the
    small parts next to it.
    """
    triangles = np.asarray(vertices, dtype=float)[np.asarray(faces)]
    done = []
    while len(triangles):
        edges = np.linalg.norm(triangles - np.roll(triangles, 1, axis=1), axis=2).max(
            axis=1
        )
        small = edges <= max_edge
        done.append(triangles[small])
        big = triangles[~small]
        if not len(big):
            break
        a, b, c = big[:, 0], big[:, 1], big[:, 2]
        ab, bc, ca = 0.5 * (a + b), 0.5 * (b + c), 0.5 * (c + a)
        triangles = np.concatenate(
            [
                np.stack(t, axis=1)
                for t in ((a, ab, ca), (ab, b, bc), (ca, bc, c), (ab, bc, ca))
            ]
        )
    soup = np.concatenate(done) if done else np.zeros((0, 3, 3))
    return soup.reshape(-1, 3), np.arange(3 * len(soup), dtype=np.uint32).reshape(-1, 3)


def face_normals(vertices, faces):
    """Unit normals of the triangles (right-hand rule on the vertex order)."""
    v = np.asarray(vertices, dtype=float)[faces]
    n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-15)


def feature_edges(vertices, faces, crease_deg=40.0, max_edges=None):
    """Edges worth drawing in a wireframe: open borders and creases.

    An edge is kept when it bounds one triangle only, or when its two triangles
    meet at more than *crease_deg*. With *max_edges*, the longest ones are kept.
    """
    vertices, faces = weld(vertices, faces)
    normals = face_normals(vertices, faces)
    edges = np.sort(faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    owner = np.repeat(np.arange(len(faces)), 3)
    order = np.lexsort((edges[:, 1], edges[:, 0]))
    edges, owner = edges[order], owner[order]
    unique, start, count = np.unique(
        edges, axis=0, return_index=True, return_counts=True
    )

    keep = count == 1
    shared = count == 2
    n0 = normals[owner[start[shared]]]
    n1 = normals[owner[start[shared] + 1]]
    cos_angle = np.abs(np.sum(n0 * n1, axis=1))
    crease = np.zeros_like(keep)
    crease[np.flatnonzero(shared)] = cos_angle < np.cos(np.radians(crease_deg))
    selected = unique[keep | crease]

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
    wanted = min_edges if max_edges is None else min(min_edges, max_edges)
    while len(edges) < wanted and crease_deg > min_crease_deg:
        crease_deg = max(0.5 * crease_deg, min_crease_deg)
        wire_vertices, edges = feature_edges(vertices, faces, crease_deg, max_edges)
    return wire_vertices, edges


# Solid builders: (vertices, faces) pairs, outward winding, in metres


def box_mesh(size, center=(0.0, 0.0, 0.0)):
    """Axis-aligned box of full lengths ``size = (lx, ly, lz)`` centred at *center*."""
    half = 0.5 * np.asarray(size, dtype=float)
    signs = np.array(
        [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    )
    vertices = signs * half + np.asarray(center, dtype=float)
    # corner index = 4 * ix + 2 * iy + iz
    quads = [
        (0, 1, 3, 2),
        (4, 6, 7, 5),
        (0, 4, 5, 1),
        (2, 3, 7, 6),
        (0, 2, 6, 4),
        (1, 5, 7, 3),
    ]
    faces = [(a, b, c) for a, b, c, d in quads] + [(a, c, d) for a, b, c, d in quads]
    return vertices, np.asarray(faces, dtype=np.uint32)


def ring_mesh(r_in, r_out, z0, z1, n=32):
    """Annular cylinder about local z, radii *r_in* < *r_out*, from *z0* to *z1*.

    ``r_in = 0`` gives a closed disk (a short cylinder).
    """
    angles = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    circle = np.column_stack([np.cos(angles), np.sin(angles)])
    rings = [(r_out, z0), (r_out, z1), (max(r_in, 1e-9), z1), (max(r_in, 1e-9), z0)]
    vertices = np.vstack(
        [np.column_stack([r * circle, np.full(n, z)]) for r, z in rings]
    )
    faces = []
    for k in range(4):  # outer wall, top annulus, inner wall, bottom annulus
        a, b = k * n, ((k + 1) % 4) * n
        for i in range(n):
            j = (i + 1) % n
            faces += [(a + i, a + j, b + j), (a + i, b + j, b + i)]
    return vertices, np.asarray(faces, dtype=np.uint32)


def cylinder_mesh(radius, z0, z1, n=32):
    """Closed cylinder about local z from *z0* to *z1*."""
    return ring_mesh(0.0, radius, z0, z1, n=n)


def transform_mesh(mesh, T):
    """Apply the 4x4 pose *T* to a ``(vertices, faces)`` pair."""
    vertices, faces = mesh
    T = np.asarray(T, dtype=float)
    return np.asarray(vertices) @ T[:3, :3].T + T[:3, 3], faces


def merge_meshes(*meshes):
    """Concatenate ``(vertices, faces)`` pairs into one mesh."""
    vertices, faces, offset = [], [], 0
    for v, f in meshes:
        vertices.append(np.asarray(v, dtype=float))
        faces.append(np.asarray(f, dtype=np.int64) + offset)
        offset += len(v)
    return np.vstack(vertices), np.vstack(faces).astype(np.uint32)


# The primitive


class TriangleMesh(ExtrudedPolygon):
    """A triangle mesh at real size in its local frame (a URDF ``<mesh>`` visual).

    Subclasses :class:`ExtrudedPolygon` so every minilink renderer draws it with
    no change to minilink (see the module docstring): meshcat as a lit solid,
    matplotlib / plotly as a feature-edge wireframe. The shaded matplotlib
    backend of this package draws it as filled, lit triangles.

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
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
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
        self.vertices = vertices
        self.faces = faces
        self._wire_vertices, self._wire_edges = wireframe_edges(
            vertices, faces, crease_deg=crease_deg, max_edges=max_edges
        )
        self._lod = {}

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


def split_by_radius(vertices, faces, axis, radius):
    """Split a wheel mesh into ``(inner, outer)`` parts by face-centroid radius.

    *axis* is the wheel spin axis (unit vector, local frame) through the origin.
    Used to colour the rim and the tire separately.
    """
    v = np.asarray(vertices, dtype=float)
    centroid = v[faces].mean(axis=1)
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    radial = centroid - np.outer(centroid @ axis, axis)
    inner = np.linalg.norm(radial, axis=1) < radius
    return (v, faces[inner]), (v, faces[~inner])
