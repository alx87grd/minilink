import numpy as np
from scipy.integrate import solve_ivp
import logging
import graphical
import matplotlib.pyplot as plt
from framework import DynamicSystem, StaticSystem, Step, GrapheSystem


############################################################
def plot_trajectory(sys, t_traj=None, x_traj=None, u_traj=None, y_traj=None):

    # Extract the system dimensions
    n = sys.n
    m = sys.m
    p = sys.p
    name = sys.name
    state_label = sys.state_label
    input_label = sys.input_label
    # output_label = sys.output_label

    # Check if the trajectories are provided
    if x_traj is None:
        n = 0
    if u_traj is None:
        m = 0
    if y_traj is None:
        p = 0

    # Compute the number of plots
    n_plots = n + m + p

    if n_plots == 0:
        logging.warning("No signals to plot")
        return

    # Create the figure
    fig, ax = plt.subplots(n_plots, 1, figsize=(10, 2 * n_plots))
    fig.canvas.manager.set_window_title("Trajectory for " + name)
    if n_plots == 1:
        ax = [ax]

    # Plot the signals
    idx = 0
    for i in range(n):
        ax[idx].plot(t_traj, x_traj[i, :], label=f"{state_label[i]}")
        ax[idx].set_ylabel(f"{state_label[i]}")
        ax[idx].grid()
        idx += 1
    for i in range(m):
        ax[idx].plot(t_traj, u_traj[i, :], label=f"{input_label[i]}")
        ax[idx].set_ylabel(f"{input_label[i]}")
        ax[idx].grid()
        idx += 1
    for i in range(p):
        ax[idx].plot(t_traj, y_traj[i, :], label=f"y{[i]}")
        ax[idx].set_ylabel(f"y{[i]}")
        ax[idx].grid()
        idx += 1

    # Show the figure
    plt.show(block=graphical.figure_blocking)

    return fig, ax


######################################################################
class Trajectory:

    def __init__(
        self,
        time: np.ndarray,
        states: np.ndarray,
        inputs: np.ndarray,
        outputs: np.ndarray,
    ):
        """
        Initialize a trajectory object.

        Parameters:
        - time: np.ndarray, time vector
        - states: np.ndarray, state trajectory (n x len(time))
        - inputs: np.ndarray, input trajectory (m x len(time))
        """
        self.time = time
        self.states = states
        self.inputs = inputs

    def get_state_at(self, t: float) -> np.ndarray:
        """Retrieve the state at a specific time."""
        idx = (np.abs(self.time - t)).argmin()
        return self.states[:, idx]

    def get_input_at(self, t: float) -> np.ndarray:
        """Retrieve the input at a specific time."""
        idx = (np.abs(self.time - t)).argmin()
        return self.inputs[:, idx]

    def slice(self, start_time: float, end_time: float) -> "Trajectory":
        """Extract a portion of the trajectory between start_time and end_time."""
        mask = (self.time >= start_time) & (self.time <= end_time)
        return Trajectory(
            time=self.time[mask],
            states=self.states[:, mask],
            inputs=self.inputs[:, mask],
        )


class Simulator:
    def __init__(
        self, sys, t0=0, tf=10, n_steps=None, dt=None, solver="scipy", verbose=True
    ):
        self.verbose = True
        self.sys = sys

        if self.verbose:
            print(
                f"Simulator:\n"
                "--------------\n"
                f"Simulating system {sys.name} from t={t0} to t={tf}"
            )

        self.solver = self.select_solver(sys, solver)
        self.t, dt, n_steps = self.select_time_vector(t0, tf, n_steps, dt, sys)

        if self.verbose:
            print(f"Time steps = {n_steps}, dt={dt} and solver= {self.solver}")

    ############################################################
    def select_solver(self, sys, user_solver=None):

        # Check if the user has specified a solver
        if user_solver is not None:
            return user_solver

        # Check if the system if an ODE or discrete time system
        if not sys.solver_info["continuous_time_equation"]:
            return "discrete"

        # Check if the system has discontinuous behavior
        if sys.solver_info["discontinuous_behavior"]:
            return "euler"

        # Else, use the default scipy ODE solver
        return "scipy"

    ############################################################
    def select_time_vector(self, t0, tf, n_steps, dt, sys):

        if not sys.solver_info["continuous_time_equation"]:
            # Discrete time system
            # Time vector is an index vector
            time_vector = np.arange(int(t0), int(tf) + 1)
            dt = 1
            n_steps = len(time_vector)
            return time_vector, dt, n_steps

        if n_steps is None and dt is None:
            # Automatic dt calculation from sys properties
            dt = sys.solver_info["smallest_time_constant"] * 0.1
            time_vector = np.arange(t0, tf + dt, dt)

            if self.verbose:
                print(f"Automatic dt based on the smallest time constant of the system")
        elif dt is None:
            time_vector = np.linspace(t0, tf, n_steps)
        elif n_steps is None:
            time_vector = np.arange(t0, tf + dt, dt)
        else:
            logging.warning(
                "You must choose between n_steps and dt: using the specified n_steps"
            )
            time_vector = np.linspace(t0, tf, n_steps)

        # Compute the time step and number of steps
        dt = time_vector[1] - time_vector[0]
        n_steps = len(time_vector)

        return time_vector, dt, n_steps

    ############################################################
    def solve(self, show=False, **solver_args):

        # Local variables names
        sys = self.sys
        times = self.t
        n_pts = len(times)
        solver = self.solver

        # Check if x0 has the right dimension
        assert sys.x0.shape[0] == sys.n, "x0 has the wrong dimension"

        # Regular ODE system
        if solver == "scipy":

            # Define the ODE
            def f(t, x):
                return sys.fsim(t, x)

            # Solve the ODE
            sol = solve_ivp(
                f, [times[0], times[-1]], sys.x0, t_eval=times, **solver_args
            )

            # For debugging purposes
            self.scipy_last_solution = sol

            # Extract the state trajectory
            t_traj = sol.t
            x_traj = sol.y

            # Compute the input trajectory signal
            u_traj = np.zeros((sys.m, n_pts))
            for i, t in enumerate(times):
                u_traj[:, i] = sys.get_u_from_input_ports(t)

            # Note: the input trajectory is computed twice, once here and once in the solver
            # If the get_u method is not deterministic, the u_traj computed here may differ
            # from the one computed in the solver

        # Special need for Euler integration
        elif self.solver == "euler":

            t_traj = times
            u_traj = np.zeros((sys.m, n_pts))
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

        elif self.solver == "discrete":

            t_traj = times
            u_traj = np.zeros((sys.m, n_pts))
            x_traj = np.zeros((sys.n, n_pts))
            x_traj[:, 0] = sys.x0

            for i, t in enumerate(times):

                u = sys.get_u_from_input_ports(t)
                x = x_traj[:, t]
                x_next = sys.f(x, u, t)
                x_traj[:, i + 1] = x_next

                u_traj[:, i] = u

        # Compute the default output trajectory
        y_traj = np.zeros((sys.p, n_pts))
        for i in range(n_pts):
            y_traj[:, i] = sys.h(x_traj[:, i], u_traj[:, i], times[i])

        # Plot the trajectory
        if show:
            plot_trajectory(sys, t_traj, x_traj, u_traj, y_traj)

        return x_traj, u_traj, t_traj, y_traj


######################################################################
if __name__ == "__main__":

    pass
