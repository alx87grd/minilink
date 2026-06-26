"""
Capture helpers for kinematic visual-regression baselines.

Serializes flattened draw geometry (primitive type + world vertices) and optional
matplotlib Agg PNG hashes for parity checks across the kinematic contract upgrade.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    CustomLine,
    ExtrudedPolygon,
    HorizonPolyline,
    Plane,
    Point,
    Rod,
    Sphere,
    TorqueArrow,
    TrajectoryPolyline,
)

BASELINE_DIR = Path(__file__).resolve().parent / "kinematic_baseline"


def _homogenize_pts(pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    if pts.ndim != 2:
        raise ValueError("pts must be 2-D")
    if pts.shape[1] == 2:
        pts = np.hstack((pts, np.zeros((pts.shape[0], 1))))
    return np.hstack((pts, np.ones((pts.shape[0], 1))))


def _transform_pts(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    hom = _homogenize_pts(pts)
    world = (np.asarray(T, dtype=float) @ hom.T).T
    return world[:, :3]


def _circle_boundary_pts(radius: float, center: np.ndarray, n: int = 32) -> np.ndarray:
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    cx, cy = float(center[0]), float(center[1])
    cz = float(center[2]) if center.size > 2 else 0.0
    return np.column_stack(
        (
            cx + radius * np.cos(th),
            cy + radius * np.sin(th),
            np.full(n, cz),
        )
    )


def primitive_world_vertices(primitive, transform_matrix: np.ndarray, t=0.0) -> dict[str, Any]:
    """Return serializable geometry for one primitive at a world transform."""
    T = np.asarray(transform_matrix, dtype=float)
    ptype = type(primitive).__name__

    if isinstance(primitive, Point):
        local = np.append(np.asarray(primitive.pt, dtype=float).reshape(3), 1.0)
        world = T @ local
        return {"type": ptype, "vertices": world[:3].reshape(1, 3).tolist()}

    if isinstance(primitive, (CustomLine, Arrow)):
        pts = primitive.pts if isinstance(primitive, CustomLine) else primitive.pts
        return {"type": ptype, "vertices": _transform_pts(T, pts).tolist()}

    if isinstance(primitive, (TorqueArrow, HorizonPolyline, TrajectoryPolyline)):
        if hasattr(primitive, "points_at"):
            pts = primitive.points_at(t)
        else:
            pts = primitive.compute_pts(t)
        return {"type": type(primitive).__name__, "vertices": _transform_pts(T, pts).tolist()}

    if isinstance(primitive, Circle):
        center = np.asarray(primitive.center, dtype=float)
        local_center = np.zeros(3)
        local_center[: center.size] = center
        wc = T @ np.append(local_center, 1.0)
        boundary = _circle_boundary_pts(primitive.radius, primitive.center)
        return {
            "type": ptype,
            "center": wc[:3].tolist(),
            "vertices": _transform_pts(T, boundary).tolist(),
        }

    if isinstance(primitive, Sphere):
        center = np.asarray(primitive.center, dtype=float)
        local_center = np.zeros(3)
        local_center[: center.size] = center
        wc = T @ np.append(local_center, 1.0)
        boundary = _circle_boundary_pts(primitive.radius, primitive.center)
        return {
            "type": ptype,
            "center": wc[:3].tolist(),
            "vertices": _transform_pts(T, boundary).tolist(),
        }

    if isinstance(primitive, Rod):
        length = float(primitive.length)
        local = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.0, -length, 0.0],
            ]
        )
        return {"type": ptype, "vertices": _transform_pts(T, local).tolist()}

    if isinstance(primitive, Box):
        lx, ly, lz = primitive.length_x, primitive.length_y, primitive.length_z
        c = np.asarray(primitive.center, dtype=float).reshape(3)
        corners = np.array(
            [
                c + np.array([-lx / 2, -ly / 2, -lz / 2]),
                c + np.array([lx / 2, -ly / 2, -lz / 2]),
                c + np.array([lx / 2, ly / 2, -lz / 2]),
                c + np.array([-lx / 2, ly / 2, -lz / 2]),
                c + np.array([-lx / 2, -ly / 2, lz / 2]),
                c + np.array([lx / 2, -ly / 2, lz / 2]),
                c + np.array([lx / 2, ly / 2, lz / 2]),
                c + np.array([-lx / 2, ly / 2, lz / 2]),
            ]
        )
        return {"type": ptype, "vertices": _transform_pts(T, corners).tolist()}

    if isinstance(primitive, Plane):
        n = np.asarray(primitive.normal, dtype=float)
        n = n / (np.linalg.norm(n) + 1e-12)
        offset = float(primitive.offset)
        size = float(primitive.size)
        center = offset * n
        if abs(n[2]) < 0.9:
            u = np.cross(n, np.array([0.0, 0.0, 1.0]))
        else:
            u = np.cross(n, np.array([1.0, 0.0, 0.0]))
        u = u / (np.linalg.norm(u) + 1e-12)
        v = np.cross(n, u)
        half = 0.5 * size
        corners = np.array(
            [
                center + half * (u + v),
                center + half * (u - v),
                center + half * (-u - v),
                center + half * (-u + v),
            ]
        )
        return {"type": ptype, "vertices": _transform_pts(T, corners).tolist()}

    if isinstance(primitive, ExtrudedPolygon):
        verts = primitive.vertices_local()
        return {"type": ptype, "vertices": _transform_pts(T, verts).tolist()}

    return {"type": ptype, "vertices": []}


def capture_draw_list(sys, x, u, t=0.0) -> list[dict[str, Any]]:
    """Capture flattened draw geometry for regression."""
    if hasattr(sys, "tf"):
        geom = sys.get_kinematic_geometry()
        if isinstance(geom, dict):
            return capture_draw_list_dict(sys, x, u, t)
    geometry = sys.get_kinematic_geometry()
    transforms = sys.get_kinematic_transforms(x, u, t)
    if len(geometry) != len(transforms):
        raise ValueError(
            f"geometry/transform count mismatch: {len(geometry)} vs {len(transforms)}"
        )
    return [
        primitive_world_vertices(prim, T)
        for prim, T in zip(geometry, transforms)
    ]


def capture_draw_list_dict(sys, x, u, t=0.0, params=None) -> list[dict[str, Any]]:
    """Capture geometry using the upgraded dict API (post-upgrade)."""
    from minilink.graphical.animation.visualization import flatten_draw_list

    frames = sys.tf(x, u, t, params)
    static = sys.get_kinematic_geometry()
    dynamic = sys.get_dynamic_geometry(x, u, t, params)
    draw_list = flatten_draw_list(frames, static, dynamic)
    return [
        primitive_world_vertices(prim, T, t=t)
        for prim, T in draw_list
    ]


def capture_frame_png_hash(sys, x, u, t=0.0, *, is_3d: bool = False) -> str:
    """Render one frame with matplotlib Agg and return SHA-256 of PNG bytes."""
    from minilink.graphical.animation.animator import Animator

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from minilink.graphical.common.matplotlib_style import DPI_FIGURE, FIGSIZE_ANIMATION
    animator = Animator(sys)
    frame = animator._prepare_transforms(x, u, t)

    fig = plt.figure(figsize=FIGSIZE_ANIMATION, dpi=DPI_FIGURE)
    if is_3d:
        ax = fig.add_subplot(111, projection="3d")
    else:
        ax = fig.add_subplot(111)

    from minilink.graphical.animation.renderers.matplotlib_renderer import (
        MatplotlibCanvas,
    )

    canvas = MatplotlibCanvas(ax, is_3d=is_3d)
    for prim, T in zip(frame["primitives"], frame["transforms"]):
        canvas.draw_primitive(prim, T)

    fig.canvas.draw()
    png_bytes = fig.canvas.buffer_rgba().tobytes()
    plt.close(fig)
    return hashlib.sha256(png_bytes).hexdigest()


def save_baseline(name: str, cases: list[dict[str, Any]]) -> Path:
    BASELINE_DIR.mkdir(parents=True, exist_ok=True)
    path = BASELINE_DIR / f"{name}.json"
    path.write_text(json.dumps(cases, indent=2))
    return path


def load_baseline(name: str) -> list[dict[str, Any]]:
    path = BASELINE_DIR / f"{name}.json"
    return json.loads(path.read_text())


def compare_draw_lists(
    actual: list[dict[str, Any]],
    expected: list[dict[str, Any]],
    *,
    rtol: float = 1e-10,
    atol: float = 1e-12,
) -> None:
    assert len(actual) == len(expected), (
        f"primitive count {len(actual)} != {len(expected)}"
    )
    for i, (a, e) in enumerate(zip(actual, expected)):
        assert a["type"] == e["type"], f"primitive {i}: type {a['type']} != {e['type']}"
        if "center" in e:
            np.testing.assert_allclose(
                a.get("center", []), e["center"], rtol=rtol, atol=atol,
                err_msg=f"primitive {i} center",
            )
        if e.get("vertices"):
            np.testing.assert_allclose(
                a["vertices"], e["vertices"], rtol=rtol, atol=atol,
                err_msg=f"primitive {i} vertices",
            )
