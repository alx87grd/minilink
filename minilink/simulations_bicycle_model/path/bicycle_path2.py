import matplotlib.pyplot as plt
import numpy as np

from minilink.core.system import System
from minilink.graphical.animation.primitives import (
    CustomLine,
)

_PATH_X0 = 0.0
_PATH_X1 = 100.0


class RightAngle(System):
    def __init__(
        self,
        x_corner: float = 1.0,
        y_path_const: float = 0.0,
    ):
        super().__init__(0)
        self.name = "Horizontal line path"
        self.x_corner = x_corner
        self.y_path_const = y_path_const

        self.add_output_port("path", dim=2, function=self.path, dependencies=())

    def path(self, x, u, t=0.0, params=None):

        return np.array([self.x_corner, self.y_path_const], dtype=float)

    def get_kinematic_geometry(self):

        pts = np.array(
            [
                [0.0, self.y_path_const, 0.0],
                [self.x_corner, self.y_path_const, 0.0],
                [self.x_corner, -_PATH_X1, 0.0],
            ]
        )

        return [CustomLine(pts, color="seagreen", linewidth=2, style="--")]

    def get_kinematic_transforms(self, x, u, t):
        return [np.eye(4)]


def main():
    path = RightAngle(x_corner=4.0, y_path_const=3.0)

    geometry = path.get_kinematic_geometry()

    color = geometry[0].color
    linewidth = geometry[0].linewidth
    linestyle = geometry[0].style

    path_pts = geometry[0].pts

    x = path_pts[:, 0]
    y = path_pts[:, 1]

    plt.figure()
    plt.plot(
        x,
        y,
        label="path",
        color=color,
        linewidth=linewidth,
        linestyle=linestyle,
    )
    plt.xlabel("X pos [m]")
    plt.ylabel("Y pos [m]")
    plt.title("Path")
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
