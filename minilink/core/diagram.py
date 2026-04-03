import numpy as np

from minilink.core.framework import System, VectorSignal


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
        self.graphe_building_verbose = True
        self.port_execution_order = []

        self.refresh()

    ######################################################################
    def add_subsystem(self, sys, sys_id):

        self.subsystems[sys_id] = sys
        self.connections[sys_id] = {}

        for port_id, port in sys.inputs.items():
            self.connections[sys_id][port_id] = None  # No connection by default

        self.compute_state_properties()
        self.compiled = False

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

        self.compiled = False

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
        Detects cycles that contain only direct feedthrough paths and computes
        the topological execution order of ports.
        """
        visited = set()
        stack = []
        stack_set = set()
        self.port_execution_order = []

        # Depth-First Search to detect cycles and build topological order
        def dfs(sys_id, port_id):
            node = (sys_id, port_id)

            # Check if this port is already in the recursion stack (indicating a cycle)
            if node in stack_set:
                # Found a cycle! Track back to the start of the cycle for an informative error message
                cycle_start_idx = stack.index(node)
                cycle_path = stack[cycle_start_idx:] + [node]
                cycle_str = " -> ".join([f"{s}:{p}" for s, p in cycle_path])
                raise RuntimeError(f"Algebraic loop detected: {cycle_str}")

            # If we've already fully explored this port, skip it
            if node in visited:
                return

            # Mark as currently visiting
            stack.append(node)
            stack_set.add(node)

            port = self.subsystems[sys_id].outputs.get(port_id)
            # If the output port exists, explore its feedthrough dependencies
            if port is not None:
                deps = port.dependencies
                sys_inputs = self.subsystems[sys_id].inputs
                # If dependencies is "all", it depends on all input ports
                if deps == "all":
                    input_deps = sys_inputs.keys()
                else:
                    input_deps = deps

                # Traverse upstream for each direct feedthrough dependency
                for in_port_id in input_deps:
                    source = self.connections[sys_id].get(in_port_id)
                    # If the input is connected to another subsystem's output, explore it
                    if source is not None:
                        src_sys_id, src_port_id = source
                        # External diagram inputs are terminal nodes in this search
                        if src_sys_id != "input":
                            dfs(src_sys_id, src_port_id)

            # Cleanup recursion stack and mark as visited
            stack.pop()
            stack_set.remove(node)
            visited.add(node)
            # Port is ready for computation once all its dependencies are visited
            self.port_execution_order.append(node)

        # Iterate through all subsystems and their output ports to start DFS traversals
        for sys_id, sys in self.subsystems.items():
            for port_id in sys.outputs:
                dfs(sys_id, port_id)

    ######################################################################
    def build_execution_plan(self):
        """
        Builds a flattened execution plan that avoids string Dictionary lookups
        and object traversal during the simulation loop.
        """
        # 1. Map all outputs to a global signal flat array
        self.output_slices = {}  # (sys_id, port_id) -> slice
        current_idx = 0
        # Iterate through all subsystems to assign a slice of the global signal array to each output port
        for sys_id, sys in self.subsystems.items():
            for port_id, port in sys.outputs.items():
                dim = port.dim
                self.output_slices[(sys_id, port_id)] = slice(
                    current_idx, current_idx + dim
                )
                current_idx += dim

        self.global_signals = np.zeros(current_idx)

        # 2. Pre-allocate input buffers for each compute/f call
        # We also map external inputs (u) indices
        self.port_execution_plan = []
        # Build the execution plan for output ports in the topological order determined by DFS
        for sys_id, port_id in self.port_execution_order:
            sys = self.subsystems[sys_id]
            port = sys.outputs[port_id]

            deps = port.dependencies
            sys_inputs = sys.inputs

            port_gather_sources = []
            port_u_dim = 0
            # For each input port needed by this output port, determine where its signal comes from
            for in_port_id, in_port in sys_inputs.items():
                # If dependencies are defined and this input port isn't one of them, use the nominal value
                if deps != "all" and in_port_id not in deps:
                    source_type = 0  # nominal
                    source_val = in_port.nominal_value
                else:
                    source = self.connections[sys_id].get(in_port_id)
                    # If the port is not connected, use its nominal value
                    if source is None:
                        source_type = 0  # nominal
                        source_val = in_port.nominal_value
                    else:
                        src_sys_id, src_port_id = source
                        # If the source is an external input to the diagram, map it to the 'u' vector
                        if src_sys_id == "input":
                            source_type = 1  # external_u
                            u_idx = 0
                            # Search for the external input port index in the global input dictionary
                            for pid, p in self.inputs.items():
                                if pid == src_port_id:
                                    source_val = slice(u_idx, u_idx + p.dim)
                                    break
                                u_idx += p.dim
                        else:
                            # If the source is another subsystem's output, map it to the global signal array
                            source_type = 2  # global_signals
                            source_val = self.output_slices[(src_sys_id, src_port_id)]

                dim = in_port.dim
                port_gather_sources.append((source_type, source_val, dim))
                port_u_dim += dim

            out_slice = self.output_slices[(sys_id, port_id)]
            # If the subsystem has state, identify its slice in the global state vector
            local_x_slice = (
                slice(self.state_index[sys_id][0], self.state_index[sys_id][1])
                if sys.n > 0
                else slice(0, 0)
            )

            # Pack all information needed to compute this output port during simulation
            self.port_execution_plan.append(
                (
                    port.compute,
                    local_x_slice,
                    port_gather_sources,
                    out_slice,
                    port_u_dim,
                )
            )

        self.state_execution_plan = []
        # Build the execution plan for state derivatives (f) for each subsystem
        for sys_id, sys in self.subsystems.items():
            # Only subsystems with state need a derivative calculation
            if sys.n > 0:
                sys_gather_sources = []
                sys_u_dim = 0
                # Determine signal sources for all inputs of the subsystem needed for its 'f' call
                for in_port_id, in_port in sys.inputs.items():
                    source = self.connections[sys_id].get(in_port_id)
                    # If not connected, use the nominal value
                    if source is None:
                        source_type = 0  # nominal
                        source_val = in_port.nominal_value
                    else:
                        src_sys_id, src_port_id = source
                        # Map to external diagram inputs
                        if src_sys_id == "input":
                            source_type = 1  # external_u
                            u_idx = 0
                            # Search for the external input port index
                            for pid, p in self.inputs.items():
                                if pid == src_port_id:
                                    source_val = slice(u_idx, u_idx + p.dim)
                                    break
                                u_idx += p.dim
                        else:
                            # Map to the global signal array for internal connections
                            source_type = 2  # global_signals
                            source_val = self.output_slices[(src_sys_id, src_port_id)]

                    dim = in_port.dim
                    sys_gather_sources.append((source_type, source_val, dim))
                    sys_u_dim += dim

                # Identify the subsystem's slice in the global state vector
                local_x_slice = slice(
                    self.state_index[sys_id][0], self.state_index[sys_id][1]
                )
                # Pack all information needed to compute the state derivative during simulation
                self.state_execution_plan.append(
                    (sys.f, local_x_slice, sys_gather_sources, sys_u_dim)
                )

    ######################################################################
    def compile(self):
        """
        Compiles the diagram for fast execution.
        """
        self.check_algebraic_loops()
        self.build_execution_plan()
        self.compiled = True

    ######################################################################
    def refresh(self):
        """
        Refresh all subsystems and rebuild the compiled execution plan.
        """
        for _, sys in self.subsystems.items():
            sys.refresh()
        self.compile()

    ######################################################################
    def compile_numpy_pipeline(self):
        """
        Prototype external compiler entrypoint (Option 1 architecture).

        This keeps the existing in-class compilation/execution methods intact
        while returning a separate compiled artifact object.
        """
        from minilink.compile.compiler import compile_diagram

        return compile_diagram(self)

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
    ######################################################################
    def f_fast(self, x, u, t=0, params=None) -> np.ndarray:
        """
        Hyper-optimized simulation step. No strings, no dicts, just arrays!
        """
        if not self.compiled:
            self.compile()

        global_signals = self.global_signals

        # 1. Compute all output port signals in topological order
        # This pass ensures all signals are ready before computing state derivatives
        for (
            compute_func,
            local_x_slice,
            gather_sources,
            out_slice,
            u_dim,
        ) in self.port_execution_plan:
            local_x = x[local_x_slice]

            # Prepare the local input vector for the subsystem's output calculation
            if u_dim == 0:
                local_u = np.array([])
            else:
                local_u = np.empty(u_dim)
                idx = 0
                # Efficiently gather signal values from their pre-mapped sources
                for src_type, src_val, dim in gather_sources:
                    # Case 2: Source is an output from another subsystem (internal connection)
                    if src_type == 2:  # global_signals
                        local_u[idx : idx + dim] = global_signals[src_val]
                    # Case 0: Source is not connected or uses a constant value
                    elif src_type == 0:  # nominal
                        local_u[idx : idx + dim] = src_val
                    # Case 1: Source is an external input to the whole diagram
                    elif src_type == 1:  # external_u
                        local_u[idx : idx + dim] = u[src_val]
                    idx += dim

            # Execute the port's compute function and store the result in the global signals array
            global_signals[out_slice] = compute_func(local_x, local_u, t)

        # 2. Compute state derivatives
        # This pass updates the dx vector for all subsystems with state
        dx = np.zeros(self.n)
        for f_func, local_x_slice, gather_sources, u_dim in self.state_execution_plan:
            local_x = x[local_x_slice]

            # Prepare the local input vector for the subsystem's f (dynamic) calculation
            if u_dim == 0:
                local_u = np.array([])
            else:
                local_u = np.empty(u_dim)
                idx = 0
                # Gather signals similarly to the output pass
                for src_type, src_val, dim in gather_sources:
                    if src_type == 2:  # global_signals
                        local_u[idx : idx + dim] = global_signals[src_val]
                    elif src_type == 0:  # nominal
                        local_u[idx : idx + dim] = src_val
                    elif src_type == 1:  # external_u
                        local_u[idx : idx + dim] = u[src_val]
                    idx += dim

            # Execute the subsystem's dynamic function and store results in the global derivative vector
            dx[local_x_slice] = f_func(local_x, local_u, t)

        return dx

    ######################################################################
    # JAX prototype backend (Variant 2)
    ######################################################################
    def compile_jax(self, output_ports, jit=True):
        """
        Compile the diagram evaluation into a JAX-jittable function.

        This is a *prototype* backend intended for deployment / differentiable
        optimization when the participating subsystems are JAX-compatible.

        Notes / limitations:
        - Only intended for evaluating subsystem *output ports* (not diagram
          `diagram.outputs`).
        - Blocks must use JAX-traceable operations in their `f`/`h`/port
          compute functions. Python-side branching on traced values will fail.
        - This backend does not attempt to differentiate through SciPy/solve_ivp
          (trajectory rollout lives outside this function).

        Parameters
        ----------
        output_ports : list[tuple[str, str]]
            List of `(sys_id, port_id)` to return.
        jit : bool
            If True, wrap the returned callable in `jax.jit`.

        Returns
        -------
        callable
            Function with signature: `fn(x, u, t) -> y_concat`, where:
            - `x` is the diagram state vector (shape `(self.n,)`)
            - `u` is the diagram external input vector (shape `(self.m,)`)
            - `t` is a scalar time
            - return is concatenated outputs for `output_ports` in order.
        """

        # Ensure compilation artifacts exist (execution_plan + slices).
        if not self.compiled:
            self.compile()

        try:
            import jax
            import jax.numpy as jnp
        except ImportError as e:
            raise ImportError(
                "JAX is required for compile_jax(). Install `jax` and `jaxlib`."
            ) from e

        output_ports = list(output_ports)
        out_slices = [
            self.output_slices[(sys_id, port_id)] for sys_id, port_id in output_ports
        ]

        # Cache the plan locally to reduce Python attribute lookups in the hot path.
        port_plan = self.port_execution_plan
        output_dim_total = sum(s.stop - s.start for s in out_slices)

        def eval_outputs(x, u, t=0.0):
            # NOTE: dtype inference is best-effort; if x and u are both empty,
            # default to float32.
            sample = x if getattr(x, "size", 0) else u
            dtype = getattr(sample, "dtype", None)
            if dtype is None:
                dtype = jnp.float32

            # Evaluate all subsystem outputs into a global flat buffer.
            global_signals = jnp.zeros(self.global_signals.shape[0], dtype=dtype)

            for (
                compute_func,
                local_x_slice,
                gather_sources,
                out_slice,
                u_dim,
            ) in port_plan:
                local_x = x[local_x_slice]

                if u_dim == 0:
                    local_u = jnp.array([], dtype=dtype)
                else:
                    # Build local_u without in-place mutation (JAX-friendly).
                    pieces = []
                    for src_type, src_val, dim in gather_sources:
                        if src_type == 2:  # global_signals
                            pieces.append(global_signals[src_val])
                        elif src_type == 0:  # nominal (constant)
                            pieces.append(jnp.asarray(src_val, dtype=dtype))
                        elif src_type == 1:  # external diagram input vector
                            pieces.append(u[src_val])
                        else:
                            raise RuntimeError(f"Unknown source_type={src_type}")

                    local_u = (
                        jnp.concatenate(pieces, axis=0)
                        if pieces
                        else jnp.array([], dtype=dtype)
                    )

                y_out = compute_func(local_x, local_u, t)
                global_signals = global_signals.at[out_slice].set(y_out)

            # Return selected outputs in the requested order.
            out_pieces = [global_signals[s] for s in out_slices]
            y_concat = (
                jnp.concatenate(out_pieces, axis=0)
                if out_pieces
                else jnp.zeros((output_dim_total,), dtype=dtype)
            )
            return y_concat

        if jit:
            return jax.jit(eval_outputs)
        return eval_outputs

    def f_fast_jax(self, x, u, t=0.0):
        """
        JAX-compatible variant of `f_fast`.

        Returns the diagram state derivative `dx` computed from subsystem `f`
        functions and input/output port wiring.

        Prototype limitations:
        - Subsystems must be JAX-traceable.
        - External inputs `u` are treated as differentiable; gradients flow
          through the diagram evaluation (but not through solve_ivp).
        """
        if not self.compiled:
            self.compile()

        try:
            import jax.numpy as jnp
        except ImportError as e:
            raise ImportError(
                "JAX is required for f_fast_jax(). Install `jax` and `jaxlib`."
            ) from e

        sample = x if getattr(x, "size", 0) else u
        dtype = getattr(sample, "dtype", None)
        if dtype is None:
            dtype = jnp.float32

        global_signals = jnp.zeros(self.global_signals.shape[0], dtype=dtype)

        # 1) Compute all output ports (same logic as compile_jax).
        for (
            compute_func,
            local_x_slice,
            gather_sources,
            out_slice,
            u_dim,
        ) in self.port_execution_plan:
            local_x = x[local_x_slice]
            if u_dim == 0:
                local_u = jnp.array([], dtype=dtype)
            else:
                pieces = []
                for src_type, src_val, dim in gather_sources:
                    if src_type == 2:  # global_signals
                        pieces.append(global_signals[src_val])
                    elif src_type == 0:  # nominal
                        pieces.append(jnp.asarray(src_val, dtype=dtype))
                    elif src_type == 1:  # external diagram input vector
                        pieces.append(u[src_val])
                    else:
                        raise RuntimeError(f"Unknown source_type={src_type}")
                local_u = jnp.concatenate(pieces, axis=0)

            y_out = compute_func(local_x, local_u, t)
            global_signals = global_signals.at[out_slice].set(y_out)

        # 2) Compute state derivatives via subsystem f calls.
        dx = jnp.zeros(self.n, dtype=dtype)
        for f_func, local_x_slice, gather_sources, u_dim in self.state_execution_plan:
            local_x = x[local_x_slice]
            if u_dim == 0:
                local_u = jnp.array([], dtype=dtype)
            else:
                pieces = []
                for src_type, src_val, dim in gather_sources:
                    if src_type == 2:  # global_signals
                        pieces.append(global_signals[src_val])
                    elif src_type == 0:  # nominal
                        pieces.append(jnp.asarray(src_val, dtype=dtype))
                    elif src_type == 1:  # external diagram input vector
                        pieces.append(u[src_val])
                    else:
                        raise RuntimeError(f"Unknown source_type={src_type}")
                local_u = jnp.concatenate(pieces, axis=0)

            dx_piece = f_func(local_x, local_u, t)
            dx = dx.at[local_x_slice].set(dx_piece)

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
