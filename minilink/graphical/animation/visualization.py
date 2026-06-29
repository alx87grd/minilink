"""Frame-keyed geometry → flat draw-list adapter (animator internals).

The drawable contract returns **frame-keyed dicts**: ``tf`` gives
``dict[key, 4x4 world]`` and the geometry hooks give ``dict[key, list[prim]]``.
Renderers consume a **flat** ``(primitive, world 4x4)`` list
("draw this shape at this pose"). :func:`flatten_draw_list` is the adapter that
bridges the two, plus the ``local_transform`` compose and the static/dynamic merge.

It is **signature-agnostic** (it never sees ``x, u, t``): the animator resolves
each drawable's hooks — primary with ``(x, u, t)``, overlays with ``t`` only —
and hands the three already-resolved dicts here, so the primary and the overlays
share one flattening path. This module is animator-only (not public).
"""

import numpy as np

# Frame key for world-fixed geometry; always present even when ``tf`` is empty.
WORLD = "world"


def ensure_world_frame(frames):
    """Return *frames* with an implicit identity ``world`` root frame if absent."""
    frames = dict(frames)
    frames.setdefault(WORLD, np.eye(4))
    return frames


def merge_geometry(*geometries):
    """Merge frame-keyed geometry dicts, concatenating primitive lists per key.

    ``merge_geometry(kinematic, dynamic)`` is how the static skin and the
    per-frame dynamic geometry are combined before flattening. Later dicts append
    after earlier ones at a shared key.
    """
    merged: dict[str, list] = {}
    for geometry in geometries:
        for key, primitives in geometry.items():
            merged.setdefault(key, []).extend(primitives)
    return merged


def prefix_keys(mapping, prefix, sep=":"):
    """Return *mapping* with every key namespaced as ``f"{prefix}{sep}{key}"``.

    Used to namespace a subsystem's frames/geometry inside a diagram
    (``"plant:body"``), keeping keys unique when drawables are concatenated.
    """
    return {f"{prefix}{sep}{key}": value for key, value in mapping.items()}


def namespace_subsystem_frames(sub_frames, sys_id, sep=":"):
    """Prefix articulated subsystem frames; omit ``world`` (shared diagram root)."""
    return {
        f"{sys_id}{sep}{key}": value
        for key, value in sub_frames.items()
        if key != WORLD
    }


def merge_subsystem_geometry(merged, subsystem_geometry, sys_id, sep=":"):
    """Merge one subsystem's geometry into *merged*; ``world`` stays unprefixed."""
    for key, primitives in subsystem_geometry.items():
        if key == WORLD:
            merged.setdefault(WORLD, []).extend(primitives)
        else:
            merged.setdefault(f"{sys_id}{sep}{key}", []).extend(primitives)
    return merged


def flatten_draw_list(frames, kinematic, dynamic=None):
    """Flatten frame-keyed geometry into a ``[(primitive, world 4x4), …]`` list.

    Parameters
    ----------
    frames : dict[str, 4x4]
        World pose per frame key (a resolved ``tf``). ``"world"`` is injected as
        the identity if absent, so world-fixed geometry always has a frame.
    kinematic : dict[str, list[primitive]]
        Static skin (cached once across frames).
    dynamic : dict[str, list[primitive]], optional
        Per-frame geometry (rebuilt each frame); merged onto *kinematic*.

    Returns
    -------
    list[tuple[primitive, np.ndarray]]
        For each primitive at key ``k``: ``(prim, frames[k] @ prim.local_transform)``.

    Raises
    ------
    KeyError
        If geometry is keyed to a frame ``tf`` did not provide (fail loudly —
        ``"world"`` is always treated as present).
    """
    frames = ensure_world_frame(frames)

    geometry = merge_geometry(kinematic or {}, dynamic or {})

    draw_list = []
    for key, primitives in geometry.items():
        if key not in frames:
            raise KeyError(
                f"geometry keyed to unknown frame {key!r}; "
                f"tf provides {sorted(frames)} (and implicit 'world')."
            )
        W_T_key = frames[key]
        for primitive in primitives:
            local = getattr(primitive, "local_transform", None)
            W_T_prim = W_T_key if local is None else W_T_key @ local
            draw_list.append((primitive, W_T_prim))
    return draw_list
