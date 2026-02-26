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

        self.debug_print = False
        self.compiled = False
        self.port_execution_order = []

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
    def connect_new_output_port(self, source_sys_id, source_port_id, output_port_id):

        port = self.subsystems[source_sys_id].outputs[source_port_id]

        # Define the compute function for the output port
        def compute(x, u, t):
            return self.compute_subsys_output_port(
                x, u, t, source_sys_id, source_port_id
            )

        self.add_output_port(port.dim, output_port_id, compute)

        if "output" not in self.connections:
            self.connections["output"] = {}  # Create the output dictionary

        self.connect(source_sys_id, source_port_id, "output", output_port_id)

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

        # Add edges to the output node
        if "output" in self.connections:
            for port_id, edge in self.connections["output"].items():
                if edge is not None:
                    g.edge(
                        edge[0] + ":" + edge[1] + ":e",
                        "output" + ":" + port_id + ":w",
                    )

        return g

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
            # Get the value from the diagram gloabl input vector
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
    def compile(self):
        """
        Compiles the diagram by determining the valid execution order of all block output ports.
        
        This method performs a Depth-First Search (DFS) traverse over the dependencies of each out port.
        It achieves two main goals:
        1. Topological Sorting: It builds a `port_execution_order` list which guarantees that blocks 
           are evaluated in order of their dependencies. This allows for fast sequential evaluation 
           instead of recursive evaluation.
        2. Algebraic Loop Detection: It detects cycles that contain only direct feedthrough paths 
           (where an output port depends directly on an input port, which traces back to itself). 
           If a cycle is detected, an exception is raised because the system cannot be evaluated.
        """
        
        # 'visited' keeps track of all ports that have been fully processed and added to the execution order.
        visited = set()
        # 'stack' keeps track of ports currently in the recursion stack to detect cycles.
        stack = set()
        # The list that will store ports in the order they should be executed (dependencies first).
        self.port_execution_order = []
        
        def dfs(sys_id, port_id):
            """
            Recursive helper function to traverse the diagram backwards, from outputs to their inputs.
            
            Parameters
            ----------
            sys_id : str
                The ID of the subsystem being visited.
            port_id : str
                The ID of the output port within the subsystem being visited.
            """
            node = (sys_id, port_id)
            
            # If the current node is already in the recursion stack, we've found a cycle!
            # Since DFS only checks direct input dependencies of the output, this confirms an algebraic loop.
            if node in stack:
                raise RuntimeError(f"Algebraic loop detected involving direct feedthrough at {sys_id}:{port_id}")
            
            # If the node was already processed completely (all its dependencies are resolved), skip it.
            if node in visited:
                return
            
            # Mark the current node as being processed (add to the recursion stack)
            stack.add(node)
            
            # Retrieve the output port to figure out what it depends on internally
            port = self.subsystems[sys_id].outputs.get(port_id)
            if port is None:
                # If the port does not exist for some reason, finish processing it
                stack.remove(node)
                visited.add(node)
                return
                
            # Determine which inputs of the subsystem directly affect this specific output
            deps = port.dependencies
            sys_inputs = self.subsystems[sys_id].inputs
            # If explicit dependencies are not set for the port, assume it depends on ALL system inputs
            input_deps = deps if deps is not None else sys_inputs.keys()
            
            # Subsystem output depends on specific inputs. We trace those inputs backwards to their sources.
            for in_port_id in input_deps:
                # Find where this input is connected from
                source = self.connections[sys_id].get(in_port_id)
                if source is not None:
                    src_sys_id, src_port_id = source
                    # If the source is another block in the diagram (not a global input), recursively visit it
                    if src_sys_id != "input":
                        dfs(src_sys_id, src_port_id)

            # Once all dependencies (sources) of this node are resolved, we are done with it
            stack.remove(node)
            visited.add(node)
            # Add to the execution order. Since we use post-order traversal, 
            # dependencies are appended first, making it a valid topological order.
            self.port_execution_order.append(node)
            
        # Try to visit every output port of every subsystem to ensure the entire graph is compiled
        for sys_id, sys in self.subsystems.items():
            for port_id in sys.outputs:
                dfs(sys_id, port_id)
        
        self.compiled = True

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

        local_u_list = []

        # For all input ports of the subsystem
        for port_id, port in sys.inputs.items():

            # Check if the output port requires getting the signal from the input ports
            # If not required, the u vector is filled with nominal values
            if dependencies != None and port_id not in dependencies:
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
                    print(f"Comuting {sys_id} dynamic: dx=f({sys_x},{sys_u},{t})")

                sys_dx = sys.f(sys_x, sys_u, t)

                dx[idx : idx + sys.n] = sys_dx
                idx += sys.n

        return dx

    ######################################################################
    def f_fast(self, x, u, t=0, params=None) -> np.ndarray:
        """
        Evaluate the diagram state derivative without recursive traversal,
        by sequentially evaluating blocks based on the topological port execution order.
        """
        if not self.compiled:
            self.compile()

        # 1. Compute all output port signals in topological order
        signal_bus = {}
        for sys_id, port_id in self.port_execution_order:
            sys = self.subsystems[sys_id]
            port = sys.outputs[port_id]
            
            local_x = self.get_local_state(x, sys_id)
            
            # Gather inputs for THIS port based on its dependencies
            deps = port.dependencies
            sys_inputs = sys.inputs
            input_deps = deps if deps is not None else sys_inputs.keys()
            
            local_u_list = []
            for in_port_id in sys_inputs.keys():
                if deps is not None and in_port_id not in deps:
                    # Input not needed by this output port
                    port_u = sys_inputs[in_port_id].get_signal(t)
                else:
                    # Look up from signal bus or external inputs
                    source = self.connections[sys_id].get(in_port_id)
                    if source is None:
                        port_u = sys_inputs[in_port_id].get_signal(t)
                    else:
                        src_sys_id, src_port_id = source
                        if src_sys_id == "input":
                            port_u = self.get_port_values_from_u(u)[src_port_id]
                        else:
                            port_u = signal_bus[(src_sys_id, src_port_id)]
                
                local_u_list.append(port_u)
            
            if len(local_u_list) == 0:
                local_u = np.array([])
            else:
                local_u = np.concatenate(local_u_list)
                
            port_y = port.compute(local_x, local_u, t)
            signal_bus[(sys_id, port_id)] = port_y

        # 2. Compute state derivatives for all subsystems
        dx = np.zeros(self.n)
        idx = 0
        
        for sys_id, sys in self.subsystems.items():
            if sys.n > 0:
                local_x = self.get_local_state(x, sys_id)
                
                # Gather ALL inputs for the subsystem's f()
                local_u_list = []
                for in_port_id in sys.inputs.keys():
                    source = self.connections[sys_id].get(in_port_id)
                    if source is None:
                        port_u = sys.inputs[in_port_id].get_signal(t)
                    else:
                        src_sys_id, src_port_id = source
                        if src_sys_id == "input":
                            port_u = self.get_port_values_from_u(u)[src_port_id]
                        else:
                            # All outputs in the diagram have been evaluated
                            port_u = signal_bus[(src_sys_id, src_port_id)]
                    local_u_list.append(port_u)
                
                if len(local_u_list) == 0:
                    local_u = np.array([])
                else:
                    local_u = np.concatenate(local_u_list)
                    
                if self.debug_print:
                    print(f"Topological Computing {sys_id} dynamic: dx=f({local_x},{local_u},{t})")

                sys_dx = sys.f(local_x, local_u, t)
                dx[idx : idx + sys.n] = sys_dx
                idx += sys.n

        return dx

    ######################################################################
    def compile_super_fast(self):
        """
        Builds a flattened execution plan that avoids string Dictionary lookups
        and object traversal during the simulation loop. It uses a single global signal array.
        """
        if not self.compiled:
            self.compile()

        # 1. Map all outputs to a global signal flat array and allocate it
        self.output_slices = {} # (sys_id, port_id) -> slice
        current_idx = 0
        for sys_id, sys in self.subsystems.items():
            for port_id, port in sys.outputs.items():
                dim = port.dim
                self.output_slices[(sys_id, port_id)] = slice(current_idx, current_idx + dim)
                current_idx += dim
                
        self.global_signals = np.zeros(current_idx)
        
        self.port_execution_plan = []
        
        for sys_id, port_id in self.port_execution_order:
            sys = self.subsystems[sys_id]
            port = sys.outputs[port_id]
            
            deps = port.dependencies
            sys_inputs = sys.inputs
            
            port_gather_sources = []
            port_u_dim = 0
            for in_port_id, in_port in sys_inputs.items():
                if deps is not None and in_port_id not in deps:
                    source_type = "nominal"
                    source_val = None
                else:
                    source = self.connections[sys_id].get(in_port_id)
                    if source is None:
                        source_type = "nominal"
                        source_val = None
                    else:
                        src_sys_id, src_port_id = source
                        if src_sys_id == "input":
                            source_type = "external_u"
                            source_val = src_port_id
                        else:
                            source_type = "global_signals"
                            source_val = self.output_slices[(src_sys_id, src_port_id)]
                
                dim = in_port.dim
                port_gather_sources.append((source_type, source_val, in_port, dim))
                port_u_dim += dim
                
            out_slice = self.output_slices[(sys_id, port_id)]
            local_x_slice = slice(self.state_index[sys_id][0], self.state_index[sys_id][1]) if sys.n > 0 else slice(0, 0)
            
            self.port_execution_plan.append((
                port.compute,
                local_x_slice,
                port_gather_sources,
                out_slice,
                port_u_dim
            ))
            
        self.state_execution_plan = []
        
        for sys_id, sys in self.subsystems.items():
            if sys.n > 0:
                sys_gather_sources = []
                sys_u_dim = 0
                for in_port_id, in_port in sys.inputs.items():
                    source = self.connections[sys_id].get(in_port_id)
                    if source is None:
                        source_type = "nominal"
                        source_val = None
                    else:
                        src_sys_id, src_port_id = source
                        if src_sys_id == "input":
                            source_type = "external_u"
                            source_val = src_port_id
                        else:
                            source_type = "global_signals"
                            source_val = self.output_slices[(src_sys_id, src_port_id)]
                            
                    dim = in_port.dim
                    sys_gather_sources.append((source_type, source_val, in_port, dim))
                    sys_u_dim += dim
                    
                local_x_slice = slice(self.state_index[sys_id][0], self.state_index[sys_id][1])
                self.state_execution_plan.append((
                    sys.f,
                    local_x_slice,
                    sys_gather_sources,
                    sys_u_dim
                ))
                
        self.compiled_super_fast = True

    ######################################################################
    def f_super_fast(self, x, u, t=0, params=None) -> np.ndarray:
        """
        Hyper-optimized simulation step. No strings, no dicts, just arrays!
        """
        if not getattr(self, "compiled_super_fast", False):
            self.compile_super_fast()
            
        global_signals = self.global_signals
        
        external_u_dict = self.get_port_values_from_u(u) if len(u) > 0 else {}
        
        # 1. Compute all output port signals in topological order
        for compute_func, local_x_slice, gather_sources, out_slice, u_dim in self.port_execution_plan:
            local_x = x[local_x_slice]
            
            if u_dim == 0:
                local_u = np.array([])
            else:
                local_u = np.empty(u_dim)
                idx = 0
                for src_type, src_val, in_port, dim in gather_sources:
                    if src_type == "global_signals":
                        local_u[idx:idx+dim] = global_signals[src_val]
                    elif src_type == "nominal":
                        local_u[idx:idx+dim] = in_port.get_signal(t)
                    elif src_type == "external_u":
                        local_u[idx:idx+dim] = external_u_dict.get(src_val, 0)
                    idx += dim
                    
            global_signals[out_slice] = compute_func(local_x, local_u, t)
            
        # 2. Compute state derivatives
        dx = np.zeros(self.n)
        for f_func, local_x_slice, gather_sources, u_dim in self.state_execution_plan:
            local_x = x[local_x_slice]
            
            if u_dim == 0:
                local_u = np.array([])
            else:
                local_u = np.empty(u_dim)
                idx = 0
                for src_type, src_val, in_port, dim in gather_sources:
                    if src_type == "global_signals":
                        local_u[idx:idx+dim] = global_signals[src_val]
                    elif src_type == "nominal":
                        local_u[idx:idx+dim] = in_port.get_signal(t)
                    elif src_type == "external_u":
                        local_u[idx:idx+dim] = external_u_dict.get(src_val, 0)
                    idx += dim
                    
            dx[local_x_slice] = f_func(local_x, local_u, t)
            
        return dx


######################################################################
if __name__ == "__main__":

    diagram = DiagramSystem()
