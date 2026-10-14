"""Looks for the planar boat: the top-view silhouette and a 3-D hull on the water.

:func:`boat_skin_2d` is the catalog's top view, :func:`boat_skin_3d` a lofted hull
with deck, cabin, mast and the stern thruster pod, on a water plane. Both are
``(plant) -> dict[frame, list[primitive]]`` keyed to the boat's ``body`` frame
(the c.g. at the origin, bow along local +X, waterline at local z = 0) with no
state math; the plant's ``tf`` places them.
"""

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

PALETTE = {
    "hull": "#f4f1ea",
    "deck": "#c8a46a",
    "cabin": "#e9ecef",
    "trim": "#1f3a5f",
    "pod": "#222222",
    "water": "#2a6f97",
}


def half_beam_profile(s, beam):
    """Half-beam along the hull: full aft of midships, an elliptic taper to the stem.

    ``s`` runs from 0 at the transom to 1 at the stem.
    """
    s = np.asarray(s, dtype=float)
    half_beam = (
        0.5
        * beam
        * np.where(
            s < 0.35,
            0.85 + 0.15 * s / 0.35,
            np.sqrt(np.clip(1.0 - ((s - 0.35) / 0.65) ** 2, 0.0, 1.0)),
        )
    )
    return half_beam


def deck_outline(length, beam, n=16):
    """The deck edge as a closed polygon ``(2 n, 2)``, port side stern to stem then starboard back."""
    s = np.linspace(0.0, 1.0, n)
    x = (s - 0.5) * length
    b = half_beam_profile(s, beam)
    port = np.column_stack([x, b])
    starboard = np.column_stack([x[::-1], -b[::-1]])
    return np.vstack([port, starboard])


def hull_mesh(length, beam, draft, freeboard, n_stations=28, n_section=11):
    """``(vertices, faces)`` of a round-bilge hull lofted from stern to bow.

    Stations run along +X from the transom at ``-length / 2`` to the stem at
    ``+length / 2``; each is a half-ellipse section from the port deck edge
    through the keel to the starboard deck edge, with the deck at ``+freeboard``
    and the keel at ``-draft``. The waterline is z = 0.
    """
    s = np.linspace(0.0, 1.0, n_stations)  # 0 at the transom, 1 at the stem
    x = (s - 0.5) * length
    half_beam = half_beam_profile(s, beam)
    depth = draft * np.sqrt(np.clip(1.0 - s**2, 0.0, 1.0))  # deepest at the transom

    # Sections: y = b cos φ, z = f − (f + d) sin φ from port deck edge to starboard
    phi = np.linspace(0.0, np.pi, n_section)
    vertices = []
    for xi, b, d in zip(x, half_beam, depth):
        y = b * np.cos(phi)
        z = freeboard - (freeboard + d) * np.sin(phi)
        vertices.append(np.column_stack([np.full(n_section, xi), y, z]))
    vertices = np.vstack(vertices)

    def node(i, j):
        return i * n_section + j

    faces = []
    for i in range(n_stations - 1):
        for j in range(n_section - 1):  # the shell
            a, b, c, d = node(i, j), node(i, j + 1), node(i + 1, j + 1), node(i + 1, j)
            faces += [[a, b, c], [a, c, d]]
        a, b = node(i, 0), node(i + 1, 0)  # the deck, port edge to starboard edge
        c, d = node(i + 1, n_section - 1), node(i, n_section - 1)
        faces += [[a, c, b], [a, d, c]]
    for j in range(1, n_section - 1):  # the transom, a fan from the port deck edge
        faces.append([node(0, 0), node(0, j + 1), node(0, j)])
    vertices, faces = weld(vertices, np.asarray(faces, dtype=np.uint32))

    # Every face turned to point away from the hull's centre
    centre = vertices.mean(axis=0)
    outward = np.einsum(
        "ij,ij->i", face_normals(vertices, faces), vertices[faces[:, 0]] - centre
    )
    faces[outward < 0] = faces[outward < 0][:, ::-1]

    return vertices, faces


def boat_skin_2d(plant, color="blue"):
    """Top view: the hull silhouette and a dot at the c.g., both in ``body``."""
    return {
        "body": [plant.body_shape()],
        "center": [Point(color=color, marker="o", size=5)],
    }


def boat_skin_3d(plant, palette=None, water=True):
    """The boat in 3-D: a lofted hull with deck, cabin, mast and thruster pod, on the water.

    Reads the hull size from the plant's drawing attributes (``body_length``,
    ``body_width``) and the thruster lever arm ``l_t`` from its ``params``; the
    water plane spans ``water_size`` [m] when the plant sets it, else six hull
    lengths. Frame keys: ``body`` and ``world`` (the water).
    """
    colors = dict(PALETTE, **(palette or {}))
    length = float(plant.body_length)
    beam = float(plant.body_width)
    l_t = float(plant.params["l_t"])
    draft, freeboard = 0.12 * length, 0.09 * length

    hull = TriangleMesh(
        *hull_mesh(length, beam, draft, freeboard), color=colors["hull"], max_edges=0
    )
    deck = ExtrudedPolygon(  # a teak deck inset from the gunwale
        deck_outline(0.96 * length, 0.86 * beam),
        height=0.01 * length,
        center=(0.0, 0.0, freeboard + 0.005 * length),
        color=colors["deck"],
    )
    cabin = ExtrudedPolygon(  # the wheelhouse, a tapered box amidships
        0.5
        * np.array(
            [
                [-0.30 * length, -0.62 * beam],
                [0.14 * length, -0.62 * beam],
                [0.22 * length, -0.40 * beam],
                [0.22 * length, 0.40 * beam],
                [0.14 * length, 0.62 * beam],
                [-0.30 * length, 0.62 * beam],
            ]
        ),
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
    mast = Box(
        length_x=0.015 * length,
        length_y=0.015 * length,
        length_z=0.22 * length,
        center=(-0.14 * length, 0.0, freeboard + 0.24 * length),
        color=colors["trim"],
    )
    pod = Box(  # the stern thruster, l_t aft of the c.g., below the waterline
        length_x=0.08 * length,
        length_y=0.3 * beam,
        length_z=0.5 * draft,
        center=(0.0, 0.0, -0.75 * draft),
        color=colors["pod"],
    )
    pod.local_transform = translation(-l_t, 0.0, 0.0)
    geometry = {"body": [hull, deck, cabin, roof, mast, pod]}

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
