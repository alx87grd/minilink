import numpy as np
from framework import System, VectorSignal


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
        state_label = []
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
                if sys.state.labels[i] in state_label:
                    state_label.append(
                        key + ":" + sys.state.labels[i]
                    )  # Add subsystem id to the label
                else:
                    state_label.append(sys.state.labels[i])
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
        self.state.labels = state_label
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
    def get_graphe(self):

        try:
            import graphviz
        except ImportError:
            print("graphviz is not available")
            return None

        g = graphviz.Digraph(self.name, engine="dot")
        g.attr(rankdir="LR")

        # Add nodes
        for i, (sys_id, sys) in enumerate(self.subsystems.items()):

            label = f"<{sys.get_block_html(sys_id)}>"

            g.node(
                sys_id,
                shape="none",
                label=label,
            )

        # Add edges
        for sys_id, sys in self.subsystems.items():
            for port_id in sys.inputs:

                edge = self.connections[sys_id][port_id]

                if edge is not None:
                    g.edge(
                        edge[0] + ":" + edge[1] + ":e",
                        sys_id + ":" + port_id + ":w",
                    )

        return g

    ######################################################################
    def get_local_state(self, x, sys_id):
        idx = self.state_index[sys_id]
        return x[idx[0] : idx[1]]

    ######################################################################
    def get_local_input(self, x, u, t, sys_id, dependencies=None):
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

        u = np.array([])

        # For all input ports of the subsystem
        for port_id, port in sys.inputs.items():

            # Get the source of the signal
            source = self.connections[sys_id][port_id]

            # Check if the output port requires getting the signal from the input ports
            # If not required, the u vector is filled with nominal values
            if dependencies != None and port_id not in dependencies:
                port_u = port.get_signal(t)

            # Check if the port is not connected
            # If not connected, the u vector is filled with nominal values
            elif source is None:

                # Default unconnected port signal
                port_u = port.get_signal(t)

            else:

                # TODO: Add the option to connect to external input ports of the diagram
                # This assume the source is another subsystem ouput port

                source_sys_id, source_port_id = source
                source_sys = self.subsystems[source_sys_id]
                source_port = source_sys.outputs[source_port_id]

                # Collect signals needed to compute the source output
                source_x = self.get_local_state(x, source_sys_id)
                source_u = self.get_local_input(
                    x, u, t, source_sys_id, source_port.dependencies
                )  # Recursive call, TODO check for algebraic loops

                port_u = source_port.compute(source_x, source_u, t)

            u = np.concatenate([u, port_u])

        return u

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
                sys_dx = sys.f(sys_x, sys_u, t)

                dx[idx : idx + sys.n] = sys_dx
                idx += sys.n

        return dx


######################################################################
if __name__ == "__main__":

    diagram = DiagramSystem()
