import matplotlib.pyplot as plt
import numpy as np

from minilink.core.system import System
from minilink.graphical.animation.primitives import (
    CustomLine,
)


class Ellipse(System):
    def __init__(
        self,
        a: float = 1.0,
        b: float = 1.0,
        x0: float = 0.0,
        y0: float = 0.0,
    ):
        super().__init__(0)
        self.name = "Ellipse"

        self.a = a
        self.b = b
        self.x0 = x0
        self.y0 = y0

        self.add_output_port("path", dim=2, function=self.path, dependencies=())

    def path(self, x, u, t=0.0, params=None):
        # return np.array([self.a, self.b], dtype=float)
        return np.array([0.0, 0.0], dtype=float)

    def get_kinematic_geometry(self):

        x_local_up = np.linspace(-self.a, self.a, 320)
        inside_up = np.clip(1 - (x_local_up / self.a) ** 2, 0.0, 1.0)

        xs_up = x_local_up + self.x0
        ys_up = self.b * np.sqrt(inside_up) + self.y0

        x_local_down = np.linspace(self.a, -self.a, 320)
        inside_down = np.clip(1 - (x_local_down / self.a) ** 2, 0.0, 1.0)

        xs_down = x_local_down + self.x0
        ys_down = -self.b * np.sqrt(inside_down) + self.y0

        xs = np.concatenate([xs_up, xs_down])
        ys = np.concatenate([ys_up, ys_down])

        pts = np.column_stack([xs, ys, np.zeros_like(xs)])

        return [CustomLine(pts, color="seagreen", linewidth=2, style="--")]

    def get_kinematic_transforms(self, x, u, t):
        return [np.eye(4)]


def main():
    a = 1.0
    b = 0.1
    path = Ellipse(a=a, b=b, x0=0.0, y0=b)

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
