"""Boat skins: the planar boat's top-view silhouette and its 3-D hull on the water."""

from __future__ import annotations

import numpy as np

from minilink.core.kinematics import translation
from minilink.graphical.animation.primitives import (
    Box,
    ExtrudedPolygon,
    Plane,
    Point,
)
from minilink.graphical.meshes import TriangleMesh, face_normals, weld

# The 3-D look's colours by part; ``boat_skin_3d(palette=...)`` overrides any of them
PALETTE = {
    "hull": "#f4f1ea",
    "deck": "#c8a46a",
    "cabin": "#e9ecef",
    "trim": "#1f3a5f",
    "pod": "#222222",
    "water": "#2a6f97",
}


###############################################################################
# Public API
###############################################################################


def boat_skin_2d(plant, color="blue"):
    """Top view: the hull silhouette in ``body`` and a dot at the c.g. in ``center``.

    Like every skin, a ``(plant) -> dict[frame, list[primitive]]`` with no state
    math: the plant's ``tf`` places the frames. ``body`` has the c.g. at its
    origin and the bow along local +X.
    """
    return {
        "body": [plant.body_shape()],
        "center": [Point(color=color, marker="o", size=5)],
    }


def boat_skin_3d(plant, palette=None, water=True):
    """The boat in 3-D: a lofted hull with deck, cabin, mast and thruster pod, on the water.

    Reads the hull size from the plant's drawing attributes (``body_length``,
    ``body_width``) and the thruster lever arm ``l_t`` from its ``params``; the
    water plane spans ``water_size`` [m] when the plant sets it, else six hull
    lengths. Frame keys: ``body`` (the c.g. at the origin, the bow along local
    +X, the waterline at local z = 0) and ``world`` (the water).
    """
    colors = dict(PALETTE, **(palette or {}))
    length = float(plant.body_length)
    beam = float(plant.body_width)
    l_t = float(plant.params["l_t"])

    # The hull's depth below the waterline and its height above it, in hull lengths
    draft = 0.12 * length
    freeboard = 0.09 * length

    # The hull, lofted from stern to bow
    hull = TriangleMesh(
        *hull_mesh(length, beam, draft, freeboard), color=colors["hull"], max_edges=0
    )

    # A teak deck inset from the gunwale
    deck = ExtrudedPolygon(
        deck_outline(0.96 * length, 0.86 * beam),
        height=0.01 * length,
        center=(0.0, 0.0, freeboard + 0.005 * length),
        color=colors["deck"],
    )

    # The wheelhouse, a tapered box amidships on the deck, and its roof
    # fmt: off
    cabin_outline = 0.5 * np.array([
        [-0.30 * length, -0.62 * beam],
        [ 0.14 * length, -0.62 * beam],
        [ 0.22 * length, -0.40 * beam],
        [ 0.22 * length,  0.40 * beam],
        [ 0.14 * length,  0.62 * beam],
        [-0.30 * length,  0.62 * beam],
    ])
    # fmt: on
    cabin = ExtrudedPolygon(
        cabin_outline,
        height=0.11 * length,
        center=(0.0, 0.0, freeboard + 0.065 * length),
        color=colors["cabin"],
    )
    roof = Box(
        length_x=0.28 * length,
        length_y=0.66 * beam,
        length_z=0.012 * length,
        center=(-0.04 * length, 0.0, freeboard + 0.126 * length),
        color=colors["trim"],
    )

    # The mast, standing on the roof
    mast = Box(
        length_x=0.015 * length,
        length_y=0.015 * length,
        length_z=0.22 * length,
        center=(-0.14 * length, 0.0, freeboard + 0.24 * length),
        color=colors["trim"],
    )

    # The stern thruster, l_t aft of the c.g., below the waterline
    pod = Box(
        length_x=0.08 * length,
        length_y=0.3 * beam,
        length_z=0.5 * draft,
        center=(0.0, 0.0, -0.75 * draft),
        color=colors["pod"],
    )
    pod.local_transform = translation(-l_t, 0.0, 0.0)

    geometry = {"body": [hull, deck, cabin, roof, mast, pod]}

    # The water, in the world frame
    if water:
        size = float(getattr(plant, "water_size", 6.0 * length))
        geometry["world"] = [
            Plane(
                normal=[0.0, 0.0, 1.0],
                offset=-0.02,
                size=size,
                thickness=0.04,
                color=colors["water"],
                opacity=0.8,
            )
        ]

    return geometry


###############################################################################
# Internal machinery
###############################################################################


def hull_mesh(length, beam, draft, freeboard, n_stations=28, n_section=11):
    """``(vertices, faces)`` of a round-bilge hull lofted from stern to bow.

    Stations run along +X from the transom at ``-length / 2`` to the stem at
    ``+length / 2``; each is a half-ellipse section from the port deck edge
    through the keel to the starboard deck edge, with the deck at ``+freeboard``
    and the keel at ``-draft``. The waterline is z = 0.
    """
    # Stations: s from 0 at the transom to 1 at the stem, at x = (s − ½) L
    s = np.linspace(0.0, 1.0, n_stations)
    x = (s - 0.5) * length
    half_beam = half_beam_profile(s, beam)

    # Keel depth d(s) = draft √(1 − s²), deepest at the transom
    depth = draft * np.sqrt(np.clip(1.0 - s**2, 0.0, 1.0))

    # Sections, from the port deck edge through the keel to the starboard one:
    # y = b cos φ, z = f − (f + d) sin φ, for φ ∈ [0, π]
    phi = np.linspace(0.0, np.pi, n_section)
    sections = []
    for xi, b, d in zip(x, half_beam, depth):
        y = b * np.cos(phi)
        z = freeboard - (freeboard + d) * np.sin(phi)
        sections.append(np.column_stack([np.full(n_section, xi), y, z]))
    vertices = np.vstack(sections)

    # Vertex j of station i
    def node(i, j):
        return i * n_section + j

    faces = []
    for i in range(n_stations - 1):
        # The shell: two triangles per quad between stations i and i + 1
        for j in range(n_section - 1):
            a = node(i, j)
            b = node(i, j + 1)
            c = node(i + 1, j + 1)
            d = node(i + 1, j)
            faces += [[a, b, c], [a, c, d]]

        # The deck, port edge to starboard edge
        a = node(i, 0)
        b = node(i + 1, 0)
        c = node(i + 1, n_section - 1)
        d = node(i, n_section - 1)
        faces += [[a, c, b], [a, d, c]]

    # The transom, a fan from the port deck edge
    for j in range(1, n_section - 1):
        faces.append([node(0, 0), node(0, j + 1), node(0, j)])

    # Coincident vertices merged: the stem's section closes onto its centreline
    vertices, faces = weld(vertices, np.asarray(faces, dtype=np.uint32))

    # Every face turned away from the hull's centre c: a face whose normal nᵢ has
    # nᵢ · (vᵢ − c) < 0, vᵢ its first vertex, has its winding reversed
    centre = vertices.mean(axis=0)
    normals = face_normals(vertices, faces)
    offsets = vertices[faces[:, 0]] - centre
    outward = np.einsum("ij,ij->i", normals, offsets)
    faces[outward < 0] = faces[outward < 0][:, ::-1]

    return vertices, faces


def half_beam_profile(s, beam):
    """The hull's half-beam: a linear rise aft, then an elliptic taper to the stem.

    ``s`` runs from 0 at the transom to 1 at the stem. Aft of ``s = 0.35`` the
    half-beam grows linearly from 85 % of ``beam / 2`` to all of it; forward of
    it, an ellipse closes it to zero at the stem.
    """
    s = np.asarray(s, dtype=float)

    # b(s) / (B/2): a linear rise from 0.85 to 1 aft, an ellipse down to 0 forward
    aft = 0.85 + 0.15 * s / 0.35
    forward = np.sqrt(np.clip(1.0 - ((s - 0.35) / 0.65) ** 2, 0.0, 1.0))
    shape = np.where(s < 0.35, aft, forward)

    half_beam = 0.5 * beam * shape

    return half_beam


def deck_outline(length, beam, n=16):
    """The deck edge as a closed polygon ``(2 n, 2)``, port side stern to stem then starboard back."""
    # Stations: s from 0 at the transom to 1 at the stem, at x = (s − ½) L
    s = np.linspace(0.0, 1.0, n)
    x = (s - 0.5) * length
    b = half_beam_profile(s, beam)

    # The port edge from stern to stem, then the starboard edge back
    port = np.column_stack([x, b])
    starboard = np.column_stack([x[::-1], -b[::-1]])
    outline = np.vstack([port, starboard])

    return outline
