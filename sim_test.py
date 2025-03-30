import numpy as np
from framework import DynamicSystem, GrapheSystem, Step, StaticSystem, WhiteNoise
from analysis import Simulator, plot_trajectory


# Defining a test system
sys1 = DynamicSystem(2, 1, 1)

sys1.add_input_port("v", 1, default_value=np.array([0.6]))
sys1.add_input_port("w", 1, default_value=np.array([-10.0]))


def f(x, u, t):
    a = u[0]
    v = u[1]
    w = u[2]
    dx = np.zeros(2)
    dx[0] = x[1]
    dx[1] = -x[0] - x[1] + a + v
    return dx


def h(x, u, t):
    a = u[0]
    v = u[1]
    w = u[2]
    return x[0] + w


sys1.f = f
sys1.h = h
sys1.x0 = np.array([1.0, 0.0])

# Running the simulation

sim = Simulator(sys1, t0=0, tf=25)

x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)

plot_trajectory(sys1, t_traj, x_traj)
# plot_trajectory(sys1, t_traj, u_traj=u_traj)
# plot_trajectory(sys1, t_traj, x_traj, u_traj)
plot_trajectory(sys1, t_traj, y_traj=y_traj)


noise = WhiteNoise()
