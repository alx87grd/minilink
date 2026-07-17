"""Path-line overlay block for project animations."""

import numpy as np

from minilink.core.system import System
from minilink.graphical.animation.primitives import CustomLine


class PathLines(System):
    """Static path polyline drawn in the world frame."""

    def __init__(
        self,
        pts,
        name: str,
        color: str = "seagreen",
        linewidth: int = 2,
        style: str = "--",
    ):
        super().__init__(0)
        self.name = name
        pts = np.asarray(pts, dtype=float)
        if pts.shape[1] == 2:
            pts = np.column_stack([pts, np.zeros(pts.shape[0])])
        self.pts = pts
        self.color = color
        self.linewidth = linewidth
        self.style = style

    def get_kinematic_geometry(self):
        return {
            "world": [
                CustomLine(
                    self.pts,
                    color=self.color,
                    linewidth=self.linewidth,
                    style=self.style,
                )
            ]
        }

    def tf(self, x, u, t=0.0, params=None):
        return {"world": np.eye(4)}


Lines = PathLines
