import numpy as np

from minilink.core.framework import System, VectorSignal
from minilink.core.trajectory import Trajectory


######################################################################
class DiagramSystem(System):
    def __init__(self):

        self.subsystems = {}  # Nodes
        self.connections = {}  # Edges

        System.__init__(self, 0, 0, 0)

        self.name = "Diagram"

        self.inputs = {}
        self.outputs = {}

        self.recompute_input_properties()

        self.debug_print = False
        self.graphe_building_verbose = True

        self.refresh()

    ######################################################################
    def add_subsystem(self, sys, sys_id):

        self.subsystems[sys_id] = sys
        self.connections[sys_id] = {}

        for port_id, port in sys.inputs.items():
            self.connections[sys_id][port_id] = None  # No connection by default

        self.compute_state_properties()

    ######################################################################
    def compute_state_properties(self):

        self.n_sys = len(self.subsystems)

        # Compute total number of states
        n = 0
        state_labels = []
        state_units = []
        state_upper_bound = np.array([])
        state_lower_bound = np.array([])
        xbar = np.array([])
        x0 = np.array([])
        state_index = {}

        idx = 0
        for i, (key, sys) in enumerate(self.subsystems.items()):
            state_index[key] = (idx, idx + sys.n)

            # Update state properties
            n += sys.n

            for i in range(sys.n):
                if sys.state.labels[i] in state_labels:
                    state_labels.append(
                        key + ":" + sys.state.labels[i]
                    )  # Add subsystem id to the label
                else:
                    state_labels.append(sys.state.labels[i])
            state_units += sys.state.units
            state_upper_bound = np.concatenate(
                [state_upper_bound, sys.state.upper_bound]
            )
            state_lower_bound = np.concatenate(
                [state_lower_bound, sys.state.lower_bound]
            )
            xbar = np.concatenate([xbar, sys.state.nominal_value])
            x0 = np.concatenate([x0, sys.x0])

            idx += sys.n

        self.n = n
        self.state = VectorSignal(n, "x")
        self.state.labels = state_labels
        self.state.units = state_units
        self.state.upper_bound = state_upper_bound
        self.state.lower_bound = state_lower_bound
        self.state.nominal_value = xbar
        self.x0 = x0
        self.state_index = state_index

    ######################################################################
    def connect(self, source_sys_id, source_port_id, target_sys_id, target_port_id):

        self.connections[target_sys_id][target_port_id] = (
            source_sys_id,
            source_port_id,
        )

        if self.graphe_building_verbose:
            print(
                "Connected "
                + source_sys_id
                + ":"
                + source_port_id
                + " to "
                + target_sys_id
                + ":"
                + target_port_id
            )

    ######################################################################
    def connect_new_output_port(
        self, source_sys_id, source_port_id, output_port_id, dependencies="all"
    ):

        port = self.subsystems[source_sys_id].outputs[source_port_id]

        # Define the compute function for the output port
        def compute(x, u, t):
            return self.compute_subsys_output_port(
                x, u, t, source_sys_id, source_port_id
            )

        self.add_output_port(port.dim, output_port_id, compute, dependencies)

        if "output" not in self.connections:
            self.connections["output"] = {}  # Create the output dictionary

        self.connect(source_sys_id, source_port_id, "output", output_port_id)

    ######################################################################
    def get_graphe(self):
        from minilink.graphical.graphe import get_diagram_graphe

        return get_diagram_graphe(self)

    ######################################################################
    def get_local_state(self, x, sys_id):

        idx = self.state_index[sys_id]
        return x[idx[0] : idx[1]]

    ######################################################################
    def compute_subsys_output_port(self, x, u, t, sys_id, port_id):

        # Get the subsystem output port
        port = self.subsystems[sys_id].outputs[port_id]

        # Collect signals needed to compute the output
        local_x = self.get_local_state(x, sys_id)
        local_u = self.get_local_input(x, u, t, sys_id, port.dependencies)

        # Compute the output signal of the port
        port_y = port.compute(local_x, local_u, t)

        return port_y

    ######################################################################
    def get_subsys_input_port(self, x, u, t, sys_id, port_id):

        # Get the source of the signal
        source = self.connections[sys_id][port_id]

        # Check if the port is not connected
        if source is None:
            # Return the nominal value of the port
            return self.subsystems[sys_id].inputs[port_id].get_signal(t)

        # Source is connected to a system and port
        source_sys_id, source_port_id = source

        # Check if the source is an external input of the diagram
        if source_sys_id == "input":
            # Get the value from the diagram global input vector
            port_u = self.get_port_values_from_u(u)[source_port_id]

        else:
            # Else, the source is an output port of another subsystem
            source_y = self.compute_subsys_output_port(
                x, u, t, source_sys_id, source_port_id
            )

            # The signal at the input port is the output signal of the source port
            port_u = source_y

        if self.debug_print:
            print(
                f"getting u={port_u} on edge from {source_sys_id}:{source_port_id} to {sys_id}:{port_id}"
            )

        return port_u

    ######################################################################
    def check_algebraic_loops(self):
        """
        Detects algebraic loops and returns the topological port execution order.

        Raises
        ------
        RuntimeError
            If an algebraic loop is found (with full cycle path in the message).

        Returns
        -------
        list of (sys_id, port_id)
            Topologically sorted output-port schedule.
        """
        from minilink.compile import check_algebraic_loops

        return check_algebraic_loops(self)  # RuntimeError propagates if loop found

    ######################################################################
    def compile(self, backend="numpy", bind_params=False, verbose=False):
        """
        Compiles the diagram into a stateless Evaluator for high-performance simulation.

        Runs algebraic-loop detection internally; raises RuntimeError if a loop is found.

        Parameters
        ----------
        backend : str
            ``'numpy'`` (default) or ``'jax'``.
        bind_params : bool, optional
            If ``True``, subsystem ``params`` are deep-copied into the plan at compile
            time (see :func:`minilink.compile.compile_diagram`). This snapshots only the
            ``params`` dict, not other subsystem state; see :class:`minilink.core.framework.System`.
        verbose : bool
            If ``True``, print timed compilation steps.

        Returns
        -------
        NumpyDiagramEvaluator or JaxDiagramEvaluator
        """
        from minilink.compile import compile_diagram

        return compile_diagram(
            self, backend=backend, bind_params=bind_params, verbose=verbose
        )

    ######################################################################
    def refresh(self):
        """
        Refresh all subsystems and rebuild the compiled execution plan.
        """
        for _, sys in self.subsystems.items():
            sys.refresh()

    ######################################################################
    def reconstruct_internal_signals(self, traj: Trajectory) -> Trajectory:
        """
        Reconstruct all subsystem output-port trajectories for this diagram.

        Parameters
        ----------
        traj : Trajectory
            State-input trajectory sampled on a time grid.

        Returns
        -------
        Trajectory
            New trajectory enriched with one sampled signal per subsystem
            output port, keyed as ``"sys_id:port_id"``.
        """
        evaluator = self.compile(backend="numpy")
        internal_signals = {}
        for sys_id, sys in self.subsystems.items():
            for port_id, port in sys.outputs.items():
                internal_signals[f"{sys_id}:{port_id}"] = np.zeros(
                    (port.dim, traj.n_samples)
                )

        for i, t in enumerate(traj.t):
            step_signals = evaluator.compute_internal_signals_dict(
                traj.x[:, i], traj.u[:, i], t
            )
            for key, value in step_signals.items():
                internal_signals[key][:, i] = value

        return traj.with_signals(internal_signals)

    ######################################################################
    def compute_internal_signals(self, traj: Trajectory) -> Trajectory:
        """
        Compatibility alias for :meth:`reconstruct_internal_signals`.
        """
        return self.reconstruct_internal_signals(traj)

    ######################################################################
    def get_local_input(self, x, u, t, sys_id, dependencies="all"):
        """
        Get the input signal for a given subsystem

        Parameters
        ----------
        x : np.ndarray
        State vector of the whole diagram
        u : np.ndarray
        Input vector of the whole diagram
        t : float
        Time
        sys_id : str
        Subsystem id
        dependencies : list
        List of input ports that are required to compute the output signal
        """

        sys = self.subsystems[sys_id]

        local_u_list = []

        # For all input ports of the subsystem
        for port_id, port in sys.inputs.items():
            # Check if the output port requires getting the signal from the input ports
            # If not required, the u vector is filled with nominal values
            if dependencies != "all" and port_id not in dependencies:
                port_u = port.get_signal(t)  # Nominal value

            else:
                # Recursively get the input signal
                port_u = self.get_subsys_input_port(x, u, t, sys_id, port_id)

            local_u_list.append(port_u)

        if len(local_u_list) == 0:
            return np.array([])

        return np.concatenate(local_u_list)

    ######################################################################
    def f(self, x, u, t=0, params=None) -> np.ndarray:

        dx = np.zeros(self.n)

        idx = 0

        # For all subsystems
        for sys_id, sys in self.subsystems.items():
            # If the subsystem has states
            if sys.n > 0:
                # Get local input signals of the subsystem
                sys_u = self.get_local_input(x, u, t, sys_id)

                # Get local state of the subsystem
                sys_x = self.get_local_state(x, sys_id)

                # Compute local state derivative
                if self.debug_print:
                    print(f"Computing {sys_id} dynamic: dx=f({sys_x},{sys_u},{t})")

                sys_dx = sys.f(sys_x, sys_u, t)

                dx[idx : idx + sys.n] = sys_dx
                idx += sys.n

        return dx

    ######################################################################
    # Graphical Animation Engine Defaults for Diagram
    ######################################################################
    def get_kinematic_geometry(self):
        primitives = []
        for sys_id, sys in self.subsystems.items():
            primitives.extend(sys.get_kinematic_geometry())
        return primitives

    def get_kinematic_transforms(self, x, u, t):
        transforms = []
        for sys_id, sys in self.subsystems.items():
            # Get the input values for this specific subsystem at this time
            if sys.n > 0:
                local_u = self.get_local_input(x, u, t, sys_id)
                local_x = self.get_local_state(x, sys_id)
            else:
                # If static, it may only need u evaluated.
                local_u = self.get_local_input(x, u, t, sys_id)
                local_x = np.array([])

            transforms.extend(sys.get_kinematic_transforms(local_x, local_u, t))
        return transforms


######################################################################
if __name__ == "__main__":
    diagram = DiagramSystem()
