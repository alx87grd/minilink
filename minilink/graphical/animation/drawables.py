"""
Drawable overlays for animation composition.

TODO: User Architectural Review — full Scene / SceneHistory / Replay (Phase 3).
"""

from __future__ import annotations

import numpy as np

from minilink.core.kinematics import identity_matrix
from minilink.core.trajectory import Trajectory


class SceneHistory:
    """
    Time-indexed overlay (MPC horizons, executed trails).

    Uses only playback time *t*; frames are ``{"world": I}``.
    """

    def __init__(self, *, horizon=None, trail=None):
        self._horizon = horizon
        self._trail = trail
        self.camera_scale = 10.0
        self.camera_target = np.zeros(3, dtype=float)
        self.camera_plot_axes = (0, 1)
        self.camera_follow_frame = None
        self.camera_priority = 0.0

    def get_kinematic_geometry(self):
        return {}

    def tf(self, x, u, t=0, params=None):
        return {"world": identity_matrix()}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        from minilink.graphical.animation.primitives import CustomLine

        lines = []
        if self._trail is not None:
            pts = self._trail.points_at(t)
            lines.append(
                CustomLine(pts, color=self._trail.color, linewidth=self._trail.linewidth, style=self._trail.style)
            )
        if self._horizon is not None:
            pts = self._horizon.points_at(t)
            lines.append(
                CustomLine(
                    pts,
                    color=self._horizon.color,
                    linewidth=self._horizon.linewidth,
                    style=self._horizon.style,
                )
            )
        if not lines:
            return {}
        return {"world": lines}


class Replay:
    """Render another drawable's skin at ``x(t)`` from a stored trajectory."""

    def __init__(self, drawable, trajectory: Trajectory):
        self._drawable = drawable
        self._trajectory = trajectory
        self.camera_scale = getattr(drawable, "camera_scale", 10.0)
        self.camera_target = getattr(drawable, "camera_target", np.zeros(3))
        self.camera_plot_axes = getattr(drawable, "camera_plot_axes", (0, 1))
        self.camera_follow_frame = getattr(drawable, "camera_follow_frame", None)
        self.camera_priority = getattr(drawable, "camera_priority", 0.0)

    def get_kinematic_geometry(self):
        return self._drawable.get_kinematic_geometry()

    def tf(self, x, u, t=0, params=None):
        x_at_t = self._state_at(t)
        u_at_t = self._input_at(t)
        return self._drawable.tf(x_at_t, u_at_t, t, params)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        x_at_t = self._state_at(t)
        u_at_t = self._input_at(t)
        return self._drawable.get_dynamic_geometry(x_at_t, u_at_t, t, params)

    def _state_at(self, t):
        traj = self._trajectory
        idx = int(np.searchsorted(traj.t, t, side="right") - 1)
        idx = int(np.clip(idx, 0, traj.n_samples - 1))
        return traj.x[:, idx]

    def _input_at(self, t):
        traj = self._trajectory
        if traj.u.size == 0:
            return np.array([])
        idx = int(np.searchsorted(traj.t, t, side="right") - 1)
        idx = int(np.clip(idx, 0, traj.n_samples - 1))
        return traj.u[:, idx]
