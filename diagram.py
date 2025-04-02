import numpy as np
from framework import System, VectorSignal
from sources import Source


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
            print("graphviz is not available, cannot plot the diagram")
            return None

        g = graphviz.Digraph(self.name, engine="dot")
        g.attr(rankdir="LR")

        # If diagram has external inputs
        if not len(self.inputs) == 0:

            # Add input node block
            # This is a hack to use the get_block_html method
            input_block = System(0, 0, 0)
            input_block.name = ""
            input_block.inputs = {}
            input_block.outputs = self.inputs
            g.node(
                "input",
                shape="none",
                label="<" + input_block.get_block_html("Inputs") + ">",
            )

        # Add subsystems nodes
        for i, (sys_id, sys) in enumerate(self.subsystems.items()):

            label = f"<{sys.get_block_html(sys_id)}>"

            g.node(
                sys_id,
                shape="none",
                label=label,
            )

        # If diagram has external outputs
        if not len(self.outputs) == 0:

            # Add input node block
            # This is a hack to use the get_block_html method
            output_block = System(0, 0, 0)
            output_block.name = ""
            output_block.inputs = self.outputs
            output_block.outputs = {}
            g.node(
                "output",
                shape="none",
                label="<" + output_block.get_block_html("Outputs") + ">",
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
    def get_subsys_input_port(self, x, u, t, sys_id, port_id):

        # Get the source of the signal
        source = self.connections[sys_id][port_id]

        # Check if the port is not connected
        if source is None:
            # Return the nominal value of the port
            return self.subsystems[sys_id].inputs[port_id].get_signal(t)

        # Source is connected to a system and port
        source_sys_id, source_port_id = source

        # Check if the source is an input port of the diagram itself
        if source_sys_id == "input":
            # Get the value from the diagram gloabl input vector
            port_u = self.get_port_values_from_u(u)[source_port_id]

        else:
            # Else, the source is an output port of another subsystem
            source_port = self.subsystems[source_sys_id].outputs[source_port_id]

            # Collect signals needed to compute the source output
            source_x = self.get_local_state(x, source_sys_id)
            source_u = self.get_local_input(
                x, u, t, source_sys_id, source_port.dependencies
            )

            port_u = source_port.compute(source_x, source_u, t)

        print(
            f"getting u={port_u} on edge from {source_sys_id}:{source_port_id} to {sys_id}:{port_id}"
        )

        return port_u

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

        local_u = np.array([])

        # For all input ports of the subsystem
        for port_id, port in sys.inputs.items():

            # Check if the output port requires getting the signal from the input ports
            # If not required, the u vector is filled with nominal values
            if dependencies != None and port_id not in dependencies:
                port_u = port.get_signal(t)  # Nominal value

            else:
                # Recursively get the input signal
                port_u = self.get_subsys_input_port(x, u, t, sys_id, port_id)

            local_u = np.concatenate([local_u, port_u])

        return local_u

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
                print(f"Comuting {sys_id} dynamic: dx=f({sys_x},{sys_u},{t})")
                sys_dx = sys.f(sys_x, sys_u, t)

                dx[idx : idx + sys.n] = sys_dx
                idx += sys.n

        return dx


######################################################################
if __name__ == "__main__":

    diagram = DiagramSystem()
