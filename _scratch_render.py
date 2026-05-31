"""Scratch helper to render a single frame of a catalog system to PNG.

Temporary tool for the catalog migration pass; deleted when the pass is done.
"""

import matplotlib

matplotlib.use("Agg")

import numpy as np

from minilink.graphical.animation import Animator
from minilink.graphical.animation.renderers.matplotlib_renderer import (
    MatplotlibRenderer,
)


def render_frame(sys, x, u, t, path, is_3d=False, title=None):
    animator = Animator(sys)
    backend = MatplotlibRenderer(animator)
    prims = sys.get_kinematic_geometry()
    transforms = sys.get_kinematic_transforms(np.asarray(x, float), np.asarray(u, float), t)
    camera = sys.get_camera_transform(np.asarray(x, float), np.asarray(u, float), t)
    backend.open_scene(is_3d=is_3d, show=False, camera=camera, title=title or sys.name)
    backend.draw_frame(prims, transforms, t, camera)
    backend.fig.savefig(path, dpi=95)
    print(f"saved {path}  (prims={len(prims)} transforms={len(transforms)})")


def render_last(sys, path, is_3d=False):
    traj = sys.traj
    x = traj.x[:, -1]
    u = traj.u[:, -1] if len(traj.u) > 0 else np.array([])
    render_frame(sys, x, u, traj.t[-1], path, is_3d=is_3d)
