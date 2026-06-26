"""
Camera transform helpers and animate-time camera factories.

The 4x4 camera matrix is built at the animation boundary from
:class:`~minilink.core.system.System` camera hint attributes or from an
explicit ``camera=`` override passed to :meth:`~minilink.core.facades.SystemFacades.animate`.
"""

import numpy as np


def camera_matrix(target=(0.0, 0.0, 0.0), plot_axes=(0, 1), scale=10.0):
    """
  Standard 4x4 camera transform.

  ``T[:3, 3]`` is the look-at target; columns of ``T[:3, :3]`` are plot
  horizontal, plot vertical, and view-out; ``T[3, 3]`` is view scale.
  """
    T = np.eye(4)
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


def world_to_camera(camera):
    """World-to-camera (view) 4x4; ``T[3, 3]`` reset to 1."""
    R = camera[:3, :3]
    target = camera[:3, 3]
    W = np.eye(4)
    W[:3, :3] = R.T
    W[:3, 3] = -R.T @ target
    return W


def follow_xy_camera(x, y, scale):
    """Top-down camera centered on *(x, y)* with view half-extent *scale*."""
    return camera_matrix(target=(x, y, 0.0), plot_axes=(0, 1), scale=scale)


def resolve_camera_from_hints(hints, frames, t=0.0):
    """
    Build a camera 4x4 from drawable hint attributes and the current frame dict.

    Parameters
    ----------
    hints : object
        Object with ``camera_scale``, ``camera_target``, ``camera_plot_axes``,
        ``camera_follow_frame``.
    frames : dict
        World transforms keyed by frame name (must include follow target if used).
    """
    target = np.asarray(hints.camera_target, dtype=float).reshape(3).copy()
    follow = hints.camera_follow_frame
    if follow is not None:
        if follow not in frames:
            raise KeyError(
                f"camera_follow_frame={follow!r} not in tf() frames {list(frames)}"
            )
        target = target + np.asarray(frames[follow][:3, 3], dtype=float).reshape(3)
    return camera_matrix(
        target=target,
        plot_axes=hints.camera_plot_axes,
        scale=hints.camera_scale,
    )


def fixed_camera(target=(0.0, 0.0, 0.0), scale=10.0, plot_axes=(0, 1)):
    """Return a callable ``camera(frames, x, u, t) -> 4x4`` for a fixed view."""

    def camera(frames, x, u, t):
        return camera_matrix(target=target, plot_axes=plot_axes, scale=scale)

    return camera


def follow_frame_camera(frame, scale=10.0, offset=(0.0, 0.0, 0.0), plot_axes=(0, 1)):
    """Return a callable that follows a named frame each frame."""

    offset = np.asarray(offset, dtype=float).reshape(3)

    def camera(frames, x, u, t):
        if frame not in frames:
            raise KeyError(f"camera follow frame {frame!r} not in tf()")
        target = np.asarray(frames[frame][:3, 3], dtype=float).reshape(3) + offset
        return camera_matrix(target=target, plot_axes=plot_axes, scale=scale)

    return camera
