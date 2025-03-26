import numpy as np
from scipy.integrate import solve_ivp
import graphical
import matplotlib.pyplot as plt
from framework import DynamicSystem, StaticSystem, Step, GrapheSystem


class Simulator:
    def __init__(self, sys, t0=0, tf=10, n_steps=None, dt=None, solver="scipy"):

        self.sys = sys

        if n_steps is None and dt is None:
            time_vector = np.linspace(t0, tf, 10000)
            # TODO: add automatic dt calculation from sys properties
        elif dt is None:
            time_vector = np.linspace(t0, tf, n_steps)
        elif n_steps is None:
            time_vector = np.arange(t0, tf + dt, dt)
        else:
            print("You must choose between n_steps and dt: using the specified n_steps")
            time_vector = np.linspace(t0, tf, n_steps)

        self.t = time_vector

        self.solver = solver

    ############################################################
    def solve(self, show=False, **solver_args):

        # Local variables names
        sys = self.sys
        times = self.t
        n_pts = len(times)

        # Check if x0 has the right dimension
        assert sys.x0.shape[0] == sys.n, "x0 has the wrong dimension"

        # Compute the input trajectory signal
        u_traj = np.zeros((sys.m, n_pts))
        for i, t in enumerate(times):
            u_traj[:, i] = sys.get_u_from_input_ports(t)

        # Note: the input trajectory is computed twice, once here and once in the solver
        # If the get_u method is not deterministic, the u_traj computed here may differ
        # from the one computed in the solver

        if self.solver == "scipy":

            # Compute the state trajectory

            # Define the ODE
            def f(t, x):
                return sys.fsim(t, x)

            # Solve the ODE
            sol = solve_ivp(
                f, [times[0], times[-1]], sys.x0, t_eval=times, **solver_args
            )

            # For debugging purposes
            self.sicpy_last_solution = sol

            # Extract the state trajectory
            t_traj = sol.t
            x_traj = sol.y

        if self.solver == "euler":

            t_traj = times
            x_traj = np.zeros((sys.n, n_pts))

            x_traj[:, 0] = sys.x0

            for i, t in enumerate(times):

                u = sys.get_u_from_input_ports(t)
                x = x_traj[:, i]
                dx = sys.f(x, u, t)

                if i < n_pts - 1:
                    dt = times[i + 1] - times[i]
                    x_next = x + dx * dt  # Euler integration
                    x_traj[:, i + 1] = x_next

                u_traj[:, i] = u

        if show:
            n_plots = sys.n + sys.m
            fig, ax = plt.subplots(n_plots, 1, figsize=(10, 2 * (sys.n + sys.m)))
            fig.canvas.manager.set_window_title("Trajectory for " + sys.name)
            if n_plots == 1:
                ax = [ax]
            for i in range(sys.n):
                ax[i].plot(t_traj, x_traj[i, :], label=f"{sys.state_label[i]}")
                ax[i].set_ylabel(f"{sys.state_label[i]}")
                ax[i].grid()
            for i in range(sys.m):
                ax[sys.n + i].plot(t_traj, u_traj[i, :], label=f"{sys.input_label[i]}")
                ax[sys.n + i].set_ylabel(f"{sys.input_label[i]}")
                ax[sys.n + i].grid()
            plt.show(block=graphical.figure_blocking)

        return x_traj, u_traj, t_traj


######################################################################
if __name__ == "__main__":

    # Defining a test system
    sys1 = DynamicSystem(1, 1, 1)

    sys1.add_input_port("v", 1, default_value=np.array([0.6]))
    sys1.add_input_port("w", 1)

    def f(x, u, t):
        # a = u[0]
        # v = u[1]
        # w = u[2]
        return -x  # + a + v + w

    sys1.f = f
    sys1.x0 = np.array([1.0])

    # Running the simulation

    sim = Simulator(sys1, t0=0, tf=10, n_steps=200)
    sim.solver = "euler"
    x_traj, u_traj, t_traj = sim.solve(show=True)

    np.set_printoptions(precision=2, suppress=True)
    print(f"Time vector:\n {t_traj}")
    print(f"Input trajectory:\n {u_traj}")
    print(f"State trajectory:\n {x_traj}")
