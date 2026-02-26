import numpy as np
from scipy.integrate import solve_ivp
import logging


######################################################################
class Trajectory:

    def __init__(self, x, u, t):
        """
        x:  array of dim = ( time-steps , sys.n )
        u:  array of dim = ( time-steps , sys.m )
        t:  array of dim = ( time-steps , 1 )
        """

        self.x = x
        self.u = u
        self.t = t

        self.port = {}  # dictionary of port signals trajectories

        self._compute_size()

    ############################
    def _compute_size(self):

        # print(self.t)

        self.time_final = self.t.max()
        self.time_steps = self.t.size

        self.n = self.x.shape[0]
        self.m = self.u.shape[0]

        assert self.x.shape[1] == self.time_steps, "x has the wrong dimension"
        assert self.u.shape[1] == self.time_steps, "u has the wrong dimension"


######################################################################
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

        #

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
                if hasattr(sys, "f_fast"):
                    return sys.f_fast(x, np.array([]), t)
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

                if hasattr(sys, "f_fast"):
                    dx = sys.f_fast(x, u, t)
                else:
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
                x = x_traj[:, i]

                if hasattr(sys, "f_fast"):
                    x_next = sys.f_fast(x, u, t)
                else:
                    x_next = sys.f(x, u, t)

                if i < n_pts - 1:
                    x_traj[:, i + 1] = x_next

                u_traj[:, i] = u

        traj = Trajectory(x_traj, u_traj, t_traj)

        # Plot the trajectory
        if show:
            from minilink.graphical.plotting import plot_trajectory

            plot_trajectory(sys, traj)

        return traj


######################################################################
if __name__ == "__main__":

    pass
