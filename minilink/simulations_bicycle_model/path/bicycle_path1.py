import matplotlib.pyplot as plt
import numpy as np

from minilink.core.system import System
from minilink.graphical.animation.primitives import (
    CustomLine,
)


class LinePath(System):
    def __init__(
        self,
        y_path_const: float = 0.0,
    ):
        super().__init__(0)
        self.name = "Horizontal line path"
        self.y_path_const = y_path_const

        self.add_output_port("path", dim=2, function=self.path, dependencies=())

    def path(self, x, u, t=0.0, params=None):
        # return np.array([0.0, self.y_path_const], dtype=float)
        return np.array([0.0, 0.0], dtype=float)

    def get_kinematic_geometry(self):
        PATH_X0 = 0.0
        PATH_X1 = 100.0
        xs = np.linspace(PATH_X0, PATH_X1, 320)
        ys = self.y_path_const * np.ones_like(xs)
        pts = np.column_stack([xs, ys, np.zeros_like(xs)])
        return [CustomLine(pts, color="seagreen", linewidth=2, style="--")]

    def get_kinematic_transforms(self, x, u, t):
        return [np.eye(4)]


def main():
    path = LinePath(y_path_const=3.0)

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
