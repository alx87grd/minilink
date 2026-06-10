import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from minilink.core.system import System
from minilink.graphical.animation.primitives import CustomLine
from minilink.simulations_bicycle_model.path.bicycle_path3 import Ellipse


class PointOnEllipse(System):
    def __init__(
        self,
        ellipse: Ellipse,
        speed: float = 1.0,
        theta0: float = 0.0,
    ):
        super().__init__(0)

        self.name = "Point moving on ellipse"

        self.ellipse = ellipse
        self.speed = speed

        self.theta = theta0
        self.t_previous = None

        self.add_output_port("position", dim=2, function=self.position, dependencies=())

    def path_direction(self, theta):
        """
        Returns the local unit tangent direction of the ellipse.

        Ellipse:
            x = x0 + a cos(theta)
            y = y0 + b sin(theta)

        Tangent:
            dx/dtheta = -a sin(theta)
            dy/dtheta =  b cos(theta)
        """

        a = self.ellipse.a
        b = self.ellipse.b

        tangent = np.array(
            [
                -a * np.sin(theta),
                b * np.cos(theta),
            ],
            dtype=float,
        )

        tangent_norm = np.linalg.norm(tangent)

        if tangent_norm <= 0.0:
            raise ValueError("Ellipse tangent norm is zero.")

        return tangent / tangent_norm

    def position_from_theta(self, theta):
        """
        Returns the point position on the ellipse for a given theta.
        """

        a = self.ellipse.a
        b = self.ellipse.b
        x0 = self.ellipse.x0
        y0 = self.ellipse.y0

        return np.array(
            [
                x0 + a * np.cos(theta),
                y0 + b * np.sin(theta),
            ],
            dtype=float,
        )

    def position(self, x, u, t=0.0, params=None):
        """
        Returns the moving point position at time t.

        Constant physical speed condition:

            ||dp/dt|| = speed

        Since:

            dp/dt = dp/dtheta * dtheta/dt

        then:

            dtheta/dt = speed / ||dp/dtheta||
        """

        if self.t_previous is None:
            self.t_previous = t

        dt = t - self.t_previous
        self.t_previous = t

        if dt < 0.0:
            dt = 0.0

        a = self.ellipse.a
        b = self.ellipse.b

        dx_dtheta = -a * np.sin(self.theta)
        dy_dtheta = b * np.cos(self.theta)

        tangent_norm = np.sqrt(dx_dtheta**2 + dy_dtheta**2)

        if tangent_norm <= 0.0:
            raise ValueError("Ellipse tangent norm is zero.")

        dtheta_dt = self.speed / tangent_norm

        self.theta += dtheta_dt * dt
        self.theta = self.theta % (2.0 * np.pi)

        return self.position_from_theta(self.theta)

    def get_kinematic_transforms(self, x, u, t):
        pos = self.position(x, u, t)

        T = np.eye(4)
        T[0, 3] = pos[0]
        T[1, 3] = pos[1]

        return [T]


def main():
    a = 1.0
    b = 0.1

    path = Ellipse(a=a, b=b, x0=0.0, y0=b)

    moving_point = PointOnEllipse(
        ellipse=path,
        speed=0.2,
        theta0=0.0,
    )

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

    # Example: plot point positions over time
    times = np.linspace(0.0, 20.0, 300)
    point_positions = []

    for t in times:
        p = moving_point.position(None, None, t)
        point_positions.append(p)

    point_positions = np.array(point_positions)

    plt.plot(
        point_positions[:, 0],
        point_positions[:, 1],
        "r.",
        markersize=3,
        label="moving point",
    )

    plt.xlabel("X pos [m]")
    plt.ylabel("Y pos [m]")
    plt.title("Constant-Speed Point on Ellipse")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()
