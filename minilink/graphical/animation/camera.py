"""Camera resolution: hints → 4x4, plus ready-made factories.

The camera is a **view hint**, not a kinematic frame, so it lives in the
graphical band (out of ``tf`` / ``core/kinematics.py``) and keeps its view scale
in the ``T[3, 3]`` slot of :func:`camera_matrix`. Three of the four design layers
live here:

- **Layer 2 — default resolver.** :func:`resolve_camera_from_hints` turns a
  drawable's plain ``camera_*`` attributes plus the resolved ``tf`` frames into a
  4x4 each frame (following a frame when ``camera_follow_frame`` is set).
- **Layer 3 — override.** The same function honors an explicit ``animate(camera=…)``
  override: a constant 4x4 ``ndarray`` or a callable ``camera(frames, x, u, t) -> 4x4``.
  :func:`follow_frame_camera` / :func:`fixed_camera` build such callables.

Layer 1 (the ``camera_*`` attributes on ``System``) and Layer 4 (multi-source
selection) live elsewhere. The matrix builders :func:`camera_matrix` /
:func:`world_to_camera` are **re-exported** from ``primitives.py`` (physically
relocated here at the Phase 5 cutover; re-exported now so import sites stay valid).
"""

import numpy as np

from minilink.graphical.animation.primitives import (  # noqa: F401  (re-export)
    camera_matrix,
    world_to_camera,
)

# Defaults mirror System.camera_* so a hint-less drawable still resolves.
_DEFAULT_SCALE = 10.0
_DEFAULT_PLOT_AXES = (0, 1)


def resolve_camera_from_hints(source, frames, x=None, u=None, t=0.0, *, override=None):
    """Resolve the 4x4 camera for one frame from a drawable's hints.

    Parameters
    ----------
    source : object
        The drawable supplying ``camera_*`` attributes (Layer 1).
    frames : dict[str, 4x4]
        Resolved ``tf`` for this instant (used when following a frame).
    x, u, t : optional
        Playback state/input/time, forwarded to a callable *override*.
    override : ndarray or callable, optional
        Layer-3 ``animate(camera=…)`` override. A constant 4x4 is returned as-is;
        a callable is invoked as ``override(frames, x, u, t)``.

    Returns
    -------
    np.ndarray
        4x4 camera transform (rigid pose + ``T[3, 3]`` view scale).
    """
    if override is not None:
        if callable(override):
            return override(frames, x, u, t)
        return np.asarray(override, dtype=float)

    target = np.asarray(
        getattr(source, "camera_target", np.zeros(3)), dtype=float
    ).reshape(3)
    plot_axes = getattr(source, "camera_plot_axes", _DEFAULT_PLOT_AXES)
    scale = getattr(source, "camera_scale", _DEFAULT_SCALE)

    follow = getattr(source, "camera_follow_frame", None)
    if follow is not None and follow in frames:
        target = np.asarray(frames[follow], dtype=float)[:3, 3] + target
        return camera_matrix(target=target, plot_axes=plot_axes, scale=scale)

    return camera_matrix(target=target, plot_axes=plot_axes, scale=scale)


def follow_frame_camera(
    frame_key,
    *,
    scale=_DEFAULT_SCALE,
    plot_axes=_DEFAULT_PLOT_AXES,
    target_offset=(0.0, 0.0, 0.0),
):
    """Return a callable camera that re-centers on ``frames[frame_key]`` each frame.

    The callable is the whole Layer-3 contract — pass it to ``animate(camera=…)``.
    *target_offset* shifts the look-at relative to the followed frame's origin.
    """
    offset = np.asarray(target_offset, dtype=float).reshape(3)

    def camera(frames, x, u, t):
        target = np.asarray(frames[frame_key], dtype=float)[:3, 3] + offset
        return camera_matrix(target=target, plot_axes=plot_axes, scale=scale)

    return camera


def fixed_camera(target, *, scale=_DEFAULT_SCALE, plot_axes=_DEFAULT_PLOT_AXES):
    """Return a callable camera fixed at *target* (a constant look-at).

    Like :func:`follow_frame_camera` but frame-independent; pass it to
    ``animate(camera=…)``.
    """
    target = np.asarray(target, dtype=float).reshape(3)

    def camera(frames, x, u, t):
        return camera_matrix(target=target, plot_axes=plot_axes, scale=scale)

    return camera
