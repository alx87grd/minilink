"""
Animation draw-list flattening and drawable composition.

TODO: User Architectural Review — extend for :class:`Scene` /
:class:`SceneHistory` / :class:`Replay` drawables (Phase 3).
"""

from __future__ import annotations

import numpy as np

from minilink.core.kinematics import identity_matrix, translation_matrix
from minilink.graphical.animation.camera import resolve_camera_from_hints


def merge_frame_dicts(*frame_dicts):
    """Merge ``tf()`` frame dicts (later keys override on collision)."""
    merged = {}
    for frames in frame_dicts:
        merged.update(frames)
    return merged


def merge_geometry_dicts(*geometry_dicts):
    """Merge skin/dynamic geometry dicts, concatenating lists per key."""
    merged = {}
    for geometry in geometry_dicts:
        for key, prims in geometry.items():
            merged.setdefault(key, []).extend(prims)
    return merged


def prefix_geometry(geometry, prefix):
    """Prefix frame keys for diagram subsystems."""
    if not prefix:
        return dict(geometry)
    return {f"{prefix}:{key}": prims for key, prims in geometry.items()}


def prefix_frames(frames, prefix):
    """Prefix frame keys for diagram subsystems."""
    if not prefix:
        return dict(frames)
    return {f"{prefix}:{key}": T for key, T in frames.items()}


def ensure_world_frame(frames):
    """Inject ``world: I`` if missing."""
    if "world" not in frames:
        frames = dict(frames)
        frames["world"] = identity_matrix()
    return frames


def flatten_draw_list(frames, static_geometry, dynamic_geometry):
    """
    Flatten skin + dynamic geometry into ``(primitive, world_4x4)`` pairs.

    Validates that every geometry key exists in *frames*.
    """
    frames = ensure_world_frame(frames)
    draw_list = []
    for geometry in (static_geometry, dynamic_geometry):
        for key, prims in geometry.items():
            if key not in frames:
                raise KeyError(
                    f"geometry frame {key!r} missing from tf(); "
                    f"available: {sorted(frames)}"
                )
            T = frames[key]
            for prim in prims:
                local = getattr(prim, "local_transform", None)
                if local is None:
                    local = identity_matrix()
                world_T = T @ local
                draw_list.append((prim, world_T))
    return draw_list


def select_camera_source(drawables, primary_index=0):
    """
    Pick the drawable whose camera hints drive the view.

    Priority: highest ``camera_priority``, then non-empty skin, then primary.
    """
    if not drawables:
        raise ValueError("select_camera_source requires at least one drawable")
    ranked = []
    for index, drawable in enumerate(drawables):
        skin = drawable.get_kinematic_geometry()
        has_skin = bool(skin)
        priority = float(getattr(drawable, "camera_priority", 0.0))
        ranked.append((priority, has_skin, index == primary_index, index, drawable))
    ranked.sort(reverse=True)
    return ranked[0][4]


def resolve_camera(camera_override, source, frames, x, u, t):
    """Resolve the 4x4 camera matrix for one frame."""
    if camera_override is not None:
        if callable(camera_override):
            return camera_override(frames, x, u, t)
        return np.asarray(camera_override, dtype=float)
    return resolve_camera_from_hints(source, frames, t)


def debug_state_skin(sys):
    """Opt-in per-state/input point skin for quick dashboards."""
    from minilink.graphical.animation.primitives import Point

    geometry = {}
    for i in range(sys.n):
        geometry[f"x{i}"] = [Point(color="blue", marker="o")]
    for i in range(sys.m):
        geometry[f"u{i}"] = [Point(color="red", marker="x")]
    return geometry


def debug_state_tf(sys, x, u, t=0.0):
    """Matching frame dict for :func:`debug_state_skin`."""
    frames = {}
    for i in range(sys.n):
        frames[f"x{i}"] = translation_matrix(dx=x[i], dy=float(i))
    for i in range(sys.m):
        frames[f"u{i}"] = translation_matrix(dx=u[i], dy=float(-i - 1))
    return frames
