from collections.abc import Mapping

import numpy as np

from minilink.compile.jax_utils import array_module
from minilink.core.system import System, VectorSignal
from minilink.core.trajectory import Trajectory


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

    def add_subsystem(self, sys, sys_id):
        self.subsystems[sys_id] = sys
        self.connections[sys_id] = {}

        for port_id in sys.inputs:
            self.connections[sys_id][port_id] = None

        self.compute_state_properties()

    def compute_state_properties(self):

        self.n_sys = len(self.subsystems)

        n = 0
        state_labels = []
        state_units = []
        state_upper_bound = np.array([])
        state_lower_bound = np.array([])
        xbar = np.array([])
        x0 = np.array([])
        state_index = {}

        idx = 0
        for sys_id, subsystem in self.subsystems.items():
            state_index[sys_id] = (idx, idx + subsystem.n)

            n += subsystem.n

            for i in range(subsystem.n):
                label = subsystem.state.labels[i]
                if label in state_labels:
                    state_labels.append(f"{sys_id}:{label}")
                else:
                    state_labels.append(label)
            state_units += subsystem.state.units
            state_upper_bound = np.concatenate(
                [state_upper_bound, subsystem.state.upper_bound]
            )
            state_lower_bound = np.concatenate(
                [state_lower_bound, subsystem.state.lower_bound]
            )
            xbar = np.concatenate([xbar, subsystem.state.nominal_value])
            x0 = np.concatenate([x0, subsystem.x0])

            idx += subsystem.n

        self.n = n
        self.state = VectorSignal(n, "x")
        self.state.labels = state_labels
        self.state.units = state_units
        self.state.upper_bound = state_upper_bound
        self.state.lower_bound = state_lower_bound
        self.state.nominal_value = xbar
        self.x0 = x0
        self.state_index = state_index

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

    def connect_new_output_port(
        self, source_sys_id, source_port_id, output_port_id, dependencies="all"
    ):
        port = self.subsystems[source_sys_id].outputs[source_port_id]

        # Define the compute function for the output port
        def compute(x, u, t, params=None):
            return self.compute_subsys_output_port(
                x, u, t, source_sys_id, source_port_id, params=params
            )

        self.add_output_port(port.dim, output_port_id, compute, dependencies)

        if "output" not in self.connections:
            self.connections["output"] = {}

        self.connect(source_sys_id, source_port_id, "output", output_port_id)

    def get_graphe(self):
        from minilink.graphical.graphe import get_diagram_graphe

        return get_diagram_graphe(self)

    def get_local_state(self, x, sys_id):

        idx = self.state_index[sys_id]
        return x[idx[0] : idx[1]]

    def _subsystem_params(self, params, sys_id):
        if params is None:
            return None
        if isinstance(params, Mapping) and sys_id in params:
            return params[sys_id]
        return params

    def _array_module_for(self, *values):
        for value in values:
            xp = array_module(value)
            if xp is not np:
                return xp
        return np

    def compute_subsys_output_port(self, x, u, t, sys_id, port_id, params=None):
        port = self.subsystems[sys_id].outputs[port_id]
        local_x = self.get_local_state(x, sys_id)
        local_u = self.get_local_input(
            x, u, t, sys_id, port.dependencies, params=params
        )
        return port.compute(
            local_x,
            local_u,
            t,
            self._subsystem_params(params, sys_id),
        )

    def get_subsys_input_port(self, x, u, t, sys_id, port_id, params=None):
        source = self.connections[sys_id][port_id]

        if source is None:
            return self.subsystems[sys_id].inputs[port_id].get_default_value()

        source_sys_id, source_port_id = source

        if source_sys_id == "input":
            port_u = self.get_port_values_from_u(u)[source_port_id]
        else:
            port_u = self.compute_subsys_output_port(
                x, u, t, source_sys_id, source_port_id, params=params
            )

        if self.debug_print:
            print(
                f"getting u={port_u} on edge from {source_sys_id}:{source_port_id} to {sys_id}:{port_id}"
            )

        return port_u

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
        from minilink.compile.compiler import check_algebraic_loops

        return check_algebraic_loops(self)  # RuntimeError propagates if loop found

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
            ``params`` dict, not other subsystem state; see :class:`minilink.core.system.System`.
        verbose : bool
            If ``True``, print timed compilation steps.

        Returns
        -------
        NumpyDiagramEvaluator or JaxDiagramEvaluator
        """
        from minilink.compile.compiler import compile_diagram

        return compile_diagram(
            self, backend=backend, bind_params=bind_params, verbose=verbose
        )

    def refresh(self):
        """
        Refresh all subsystems and rebuild the compiled execution plan.
        """
        for _, subsystem in self.subsystems.items():
            subsystem.refresh()

    def autowire(
        self,
        *,
        strict: bool = False,
        validate: bool = True,
    ) -> "DiagramSystem":
        """
        Conservatively connect unconnected inputs when one safe source matches.

        This is an optional diagram-building shortcut. It does not overwrite
        existing connections and returns ``self`` so it can be used fluently.
        """
        from minilink.core.composition import autowire

        return autowire(self, strict=strict, validate=validate)

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
        for sys_id, subsystem in self.subsystems.items():
            for port_id, port in subsystem.outputs.items():
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

    def compute_internal_signals(self, traj: Trajectory) -> Trajectory:
        """
        Compatibility alias for :meth:`reconstruct_internal_signals`.
        """
        return self.reconstruct_internal_signals(traj)

    def get_local_input(self, x, u, t, sys_id, dependencies="all", params=None):
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

        subsystem = self.subsystems[sys_id]
        local_u_list = []

        for port_id, port in subsystem.inputs.items():
            if dependencies != "all" and port_id not in dependencies:
                port_u = port.get_default_value()
            else:
                port_u = self.get_subsys_input_port(
                    x, u, t, sys_id, port_id, params=params
                )
            local_u_list.append(port_u)

        if len(local_u_list) == 0:
            xp = self._array_module_for(x, u)
            return xp.array([])

        xp = self._array_module_for(x, u, *local_u_list)
        pieces = []
        for port_u in local_u_list:
            pieces.append(xp.asarray(port_u).reshape(-1))
        return xp.concatenate(pieces)

    def f(self, x, u, t=0, params=None) -> np.ndarray:

        dx_pieces = []

        for sys_id, subsystem in self.subsystems.items():
            if subsystem.n > 0:
                sys_u = self.get_local_input(x, u, t, sys_id, params=params)
                sys_x = self.get_local_state(x, sys_id)

                if self.debug_print:
                    print(f"Computing {sys_id} dynamic: dx=f({sys_x},{sys_u},{t})")

                sys_dx = subsystem.f(
                    sys_x,
                    sys_u,
                    t,
                    self._subsystem_params(params, sys_id),
                )

                dx_pieces.append(sys_dx)

        xp = self._array_module_for(x, u, *dx_pieces)
        if not dx_pieces:
            return xp.array([])
        return xp.concatenate([xp.asarray(dx).reshape(-1) for dx in dx_pieces])

    # Graphical Animation Engine Defaults for Diagram
    def get_kinematic_geometry(self):
        primitives = []
        for _, subsystem in self.subsystems.items():
            primitives.extend(subsystem.get_kinematic_geometry())
        return primitives

    def get_kinematic_transforms(self, x, u, t):
        transforms = []
        for sys_id, subsystem in self.subsystems.items():
            if subsystem.n > 0:
                local_u = self.get_local_input(x, u, t, sys_id)
                local_x = self.get_local_state(x, sys_id)
            else:
                local_u = self.get_local_input(x, u, t, sys_id)
                local_x = np.array([])

            transforms.extend(subsystem.get_kinematic_transforms(local_x, local_u, t))
        return transforms


if __name__ == "__main__":
    diagram = DiagramSystem()
