import numpy as np

from minilink.control.lqr import lqr_at_operating_point
from minilink.dynamics.catalog.pendulum.cartpole import CartPole

plant = CartPole()
x_bar = np.array([0.0, np.pi, 0.0, 0.0])  # pole inverted, cart at origin

lqr_ctl = lqr_at_operating_point(
    plant,
    x_bar,
    Q=np.diag([1.0, 10.0, 1.0, 1.0]),
    R=np.array([[0.1]]),
)


diagram = lqr_ctl @ plant

diagram.plot_diagram()


plant.x0 = np.array([-1.0, np.pi + 0.3, 0.0, 0.0])

diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory()
diagram.animate()
