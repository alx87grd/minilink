"""
Block diagrams: systems built by connecting other systems.

A :class:`DiagramSystem` is itself a :class:`~minilink.core.system.System`,
so diagrams nest, simulate, and compile like any other system.
"""

from collections.abc import Mapping

import numpy as np

from minilink.core.backends import array_module
from minilink.core.signals import VectorSignal
from minilink.core.system import System
from minilink.core.trajectory import Trajectory


def validate_diagram_params(params, subsystem_ids):
    """Validate the nested diagram params contract.

    ``params`` must be ``None`` or a mapping keyed by subsystem id whose values
    are per-subsystem params dicts. Missing subsystem ids are allowed (those
    subsystems fall back to their live ``self.params``); unknown ids raise so
    sys-id typos fail loudly instead of silently using defaults.

    Called once at public entry points (``DiagramSystem.f``, the evaluators'
    parametric tier, the ``params`` setter), not inside the recursion.

    Parameters
    ----------
    params : None or Mapping
        Candidate diagram-level params, e.g. ``{"plant": {"m": 1.0}}``.
    subsystem_ids : container of str
        Valid subsystem ids (e.g. ``diagram.subsystems``).
    """
    if params is None:
        return
    if not isinstance(params, Mapping):
        raise TypeError(
            "diagram params must be a mapping keyed by subsystem id "
            f"(e.g. {{'plant': {{...}}}}), got {type(params).__name__}"
        )
    unknown = [key for key in params if key not in subsystem_ids]
    if unknown:
        names = ", ".join(repr(key) for key in unknown)
        available = ", ".join(repr(key) for key in subsystem_ids)
        raise ValueError(
            f"Unknown subsystem ids in diagram params: {names}; available: {available}"
        )


class DiagramSystem(System):
    """
    A system composed of subsystems connected through their named ports.

    The diagram state stacks the subsystem states in insertion order,

        x = [x_1; x_2; ...; x_k],        dx_i = f_i(x_i, u_i, t; p_i),

    where each local input ``u_i`` is gathered from the output ports connected
    to subsystem ``i``, from the diagram boundary inputs, or from the port's
    constant nominal value when unconnected. Diagram-level params are nested
    by subsystem id: ``{"plant": {...}, "ctl": {...}}``.

    Build diagrams with :meth:`add_subsystem` + :meth:`connect` (canonical
    wiring for any topology) or with the shortcuts ``+``, ``>>``, ``@``, and
    :meth:`autowire` from :mod:`minilink.core.composition`.

    This class implements the wiring contract and a simple recursive
    evaluation of ``f`` used as the reference; :meth:`compile` produces the
    fast :class:`~minilink.core.compile.execution_plan.ExecutionPlan`-based
    evaluator used by simulation and optimization.
    """

    # Construction And Wiring

    def __init__(self):
        # Nodes: sys_id -> System
        self.subsystems = {}
        # Edges: target sys_id -> {input port id -> (source sys_id, port id) or None}
        self.connections = {}

        System.__init__(self, 0)

        self.name = "Diagram"

        # Print one line per new connection when True.
        self.connection_verbose = False

        # Default entry/output ports remembered by the composition shortcuts
        # (`+`, `>>`, `@`); see minilink.core.composition.
        self._composition_entry = None
        self._composition_output = None

        self.compute_state_properties()

    def add_subsystem(self, sys, sys_id):
        """Add a subsystem under a unique id, with all its inputs unconnected."""
        self.subsystems[sys_id] = sys
        self.connections[sys_id] = {port_id: None for port_id in sys.inputs}

        self.compute_state_properties()

    def connect(self, source_sys_id, source_port_id, target_sys_id, target_port_id):
        """
        Connect a source output port to a target input port.

        ``source_sys_id="input"`` connects from a diagram boundary input;
        ``target_sys_id="output"`` connects to a diagram boundary output
        (used by :meth:`connect_new_output_port`). Existence and dimension
        are validated immediately so wiring mistakes fail at connect time,
        not at compile or simulation time.
        """
        if source_sys_id == "input":
            if source_port_id not in self.inputs:
                raise ValueError(
                    f"Unknown diagram input port '{source_port_id}'; "
                    f"available: {', '.join(self.inputs) or '(none)'}"
                )
            source_port = self.inputs[source_port_id]
        else:
            if source_sys_id not in self.subsystems:
                raise ValueError(
                    f"Unknown source subsystem '{source_sys_id}'; "
                    f"available: {', '.join(self.subsystems) or '(none)'}"
                )
            source_outputs = self.subsystems[source_sys_id].outputs
            if source_port_id not in source_outputs:
                raise ValueError(
                    f"Unknown output port '{source_sys_id}:{source_port_id}'; "
                    f"available: {', '.join(source_outputs) or '(none)'}"
                )
            source_port = source_outputs[source_port_id]

        if target_sys_id == "output":
            if target_port_id not in self.outputs:
                raise ValueError(
                    f"Unknown diagram output port '{target_port_id}'; "
                    f"available: {', '.join(self.outputs) or '(none)'}"
                )
            target_port = self.outputs[target_port_id]
        else:
            if target_sys_id not in self.subsystems:
                raise ValueError(
                    f"Unknown target subsystem '{target_sys_id}'; "
                    f"available: {', '.join(self.subsystems) or '(none)'}"
                )
            target_inputs = self.subsystems[target_sys_id].inputs
            if target_port_id not in target_inputs:
                raise ValueError(
                    f"Unknown input port '{target_sys_id}:{target_port_id}'; "
                    f"available: {', '.join(target_inputs) or '(none)'}"
                )
            target_port = target_inputs[target_port_id]

        if source_port.dim != target_port.dim:
            raise ValueError(
                f"Port dimension mismatch: {source_sys_id}:{source_port_id} "
                f"has dim {source_port.dim}, {target_sys_id}:{target_port_id} "
                f"has dim {target_port.dim}"
            )

        self.connections.setdefault(target_sys_id, {})[target_port_id] = (
            source_sys_id,
            source_port_id,
        )

        if self.connection_verbose:
            print(
                f"Connected {source_sys_id}:{source_port_id} "
                f"to {target_sys_id}:{target_port_id}"
            )

    def connect_new_output_port(
        self, source_sys_id, source_port_id, output_port_id, dependencies="all"
    ):
        """Expose a subsystem output port as a new diagram boundary output."""
        port = self.subsystems[source_sys_id].outputs[source_port_id]

        def compute(x, u, t, params=None):
            return self.compute_subsys_output_port(
                x, u, t, source_sys_id, source_port_id, params=params
            )

        self.add_output_port(
            output_port_id,
            dim=port.dim,
            function=compute,
            dependencies=dependencies,
        )

        self.connect(source_sys_id, source_port_id, "output", output_port_id)

    def compute_state_properties(self):
        """
        Rebuild the flattened state metadata from the subsystems.

        The stacked state vector is ``x = [x_1; x_2; ...]`` in subsystem
        insertion order; :attr:`state_index` maps each subsystem id to its
        ``(start, end)`` indices in the stacked vector. State labels are
        prefixed with the subsystem id whenever the same label appears in
        several subsystems.
        """
        state_index = {}
        idx = 0
        for sys_id, subsystem in self.subsystems.items():
            state_index[sys_id] = (idx, idx + subsystem.n)
            idx += subsystem.n

        owned_labels = [
            (sys_id, label)
            for sys_id, subsystem in self.subsystems.items()
            for label in subsystem.state.labels
        ]
        label_counts = {}
        for _, label in owned_labels:
            label_counts[label] = label_counts.get(label, 0) + 1
        labels = [
            f"{sys_id}:{label}" if label_counts[label] > 1 else label
            for sys_id, label in owned_labels
        ]

        self.n = idx
        self.state_index = state_index
        self.state = VectorSignal("x", dim=self.n)
        self.x0 = np.zeros(self.n)

        if self.subsystems:
            substates = [sub.state for sub in self.subsystems.values()]
            self.state.labels = labels
            self.state.units = [unit for s in substates for unit in s.units]
            self.state.upper_bound = np.concatenate([s.upper_bound for s in substates])
            self.state.lower_bound = np.concatenate([s.lower_bound for s in substates])
            self.state.nominal_value = np.concatenate(
                [s.nominal_value for s in substates]
            )
            self.x0 = np.concatenate([sub.x0 for sub in self.subsystems.values()])

    def refresh(self):
        """Refresh all subsystems and rebuild the flattened state metadata.

        Compiled evaluators are snapshots: recompile after structural changes.
        """
        for subsystem in self.subsystems.values():
            subsystem.refresh()
        self.compute_state_properties()

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

    # Parameters

    @property
    def params(self):
        """Nested live view ``{sys_id: subsystem.params}``.

        Subsystems remain the single source of truth: the dict is assembled
        fresh on each access from live references, so mutating
        ``diagram.params["plant"]["m"]`` mutates that subsystem's params —
        the same semantics as mutating ``sys.params`` on a leaf system.
        """
        return {
            sys_id: subsystem.params for sys_id, subsystem in self.subsystems.items()
        }

    @params.setter
    def params(self, value):
        validate_diagram_params(value, self.subsystems)
        if value is None:
            return
        for sys_id, subsystem_params in value.items():
            self.subsystems[sys_id].params = subsystem_params

    def _subsystem_params(self, params, sys_id):
        """Route nested diagram params to one subsystem (strict contract).

        ``None`` → ``None`` (subsystem uses its live ``self.params``);
        a mapping → ``params.get(sys_id)`` (missing id → ``None`` → defaults).
        Anything else is a contract violation.
        """
        if params is None:
            return None
        if isinstance(params, Mapping):
            return params.get(sys_id)
        raise TypeError(
            "Diagram params must be a mapping keyed by subsystem id "
            f"(e.g. {{'plant': {{...}}}}), got {type(params).__name__}"
        )

    # Dynamics (reference recursive path)

    def f(self, x, u, t=0, params=None):
        """
        Stacked state derivative ``dx = [f_1(x_1, u_1, t); f_2(x_2, u_2, t); ...]``.

        This is the interpreted reference implementation; :meth:`compile`
        produces the fast equivalent used by simulation and optimization.
        """
        validate_diagram_params(params, self.subsystems)

        dx_pieces = []
        for sys_id, subsystem in self.subsystems.items():
            if subsystem.n == 0:
                continue
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id, params=params)
            local_params = self._subsystem_params(params, sys_id)

            dx_pieces.append(subsystem.f(local_x, local_u, t, local_params))

        xp = array_module(x, u, *dx_pieces)
        if not dx_pieces:
            return xp.array([])
        return xp.concatenate([xp.asarray(dx).reshape(-1) for dx in dx_pieces])

    def get_local_state(self, x, sys_id):
        """Extract one subsystem's state from the stacked state vector."""
        start, end = self.state_index[sys_id]
        return x[start:end]

    def get_local_input(self, x, u, t, sys_id, dependencies="all", params=None):
        """
        Assemble one subsystem's local input vector from its connected sources.

        Input ports outside ``dependencies`` contribute their constant nominal
        value (used to break false feedthrough when evaluating output ports).
        """
        subsystem = self.subsystems[sys_id]

        pieces = []
        for port_id, port in subsystem.inputs.items():
            if dependencies != "all" and port_id not in dependencies:
                port_u = port.get_default_value()
            else:
                port_u = self.get_subsys_input_port(
                    x, u, t, sys_id, port_id, params=params
                )
            pieces.append(port_u)

        xp = array_module(x, u, *pieces)
        if not pieces:
            return xp.array([])
        return xp.concatenate([xp.asarray(piece).reshape(-1) for piece in pieces])

    def get_subsys_input_port(self, x, u, t, sys_id, port_id, params=None):
        """Value of one subsystem input port: connected source or nominal fallback."""
        source = self.connections[sys_id][port_id]

        if source is None:
            return self.subsystems[sys_id].inputs[port_id].get_default_value()

        source_sys_id, source_port_id = source

        if source_sys_id == "input":
            return self.get_port_values_from_u(u)[source_port_id]
        return self.compute_subsys_output_port(
            x, u, t, source_sys_id, source_port_id, params=params
        )

    def compute_subsys_output_port(self, x, u, t, sys_id, port_id, params=None):
        """Evaluate one subsystem output port, recursing through its sources."""
        port = self.subsystems[sys_id].outputs[port_id]
        local_x = self.get_local_state(x, sys_id)
        local_u = self.get_local_input(
            x, u, t, sys_id, port.dependencies, params=params
        )
        local_params = self._subsystem_params(params, sys_id)

        return port.compute(local_x, local_u, t, local_params)

    # Compile And Analysis

    def compile(self, backend="numpy", bind_params=False, verbose=False):
        """
        Compile the diagram into a stateless evaluator for fast simulation.

        Runs algebraic-loop detection internally; raises RuntimeError if a
        loop is found.

        Parameters
        ----------
        backend : str
            ``'numpy'`` (default) or ``'jax'``.
        bind_params : bool, optional
            If ``True``, subsystem ``params`` are deep-copied into the plan at
            compile time (see :func:`minilink.core.compile.compiler.compile_diagram`).
        verbose : bool
            If ``True``, print timed compilation steps.

        Returns
        -------
        NumpyDiagramEvaluator or JaxDiagramEvaluator
        """
        from minilink.core.compile.compiler import compile_diagram

        return compile_diagram(
            self, backend=backend, bind_params=bind_params, verbose=verbose
        )

    def check_algebraic_loops(self):
        """
        Detect algebraic loops and return the topological port execution order.

        Raises
        ------
        RuntimeError
            If an algebraic loop is found (with full cycle path in the message).

        Returns
        -------
        list of (sys_id, port_id)
            Topologically sorted output-port schedule.
        """
        from minilink.core.compile.compiler import check_algebraic_loops

        return check_algebraic_loops(self)

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

    # Visualization / Kinematic Contract

    # Subsystem frames/geometry are namespaced with ``prefix_keys`` ("sys:key")
    # so keys stay unique across subsystems. The full ordered draw list is built
    # in the dynamic hook — per subsystem its kinematic-then-dynamic groups, in
    # subsystem order — so each subsystem reproduces its own draw order, matching
    # a concatenation of the subsystems' draw lists.

    def get_kinematic_geometry(self):
        return {}

    def tf(self, x, u, t=0, params=None):
        from minilink.graphical.animation.visualization import prefix_keys

        frames = {}
        for sys_id, subsystem in self.subsystems.items():
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id)
            sub_frames = dict(subsystem.tf(local_x, local_u, t))
            sub_frames.setdefault("world", np.eye(4))
            frames.update(prefix_keys(sub_frames, sys_id))
        return frames

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        from minilink.graphical.animation.visualization import (
            merge_geometry,
            prefix_keys,
        )

        merged: dict[str, list] = {}
        for sys_id, subsystem in self.subsystems.items():
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id)
            sub_kin = prefix_keys(subsystem.get_kinematic_geometry(), sys_id)
            sub_dyn = prefix_keys(
                subsystem.get_dynamic_geometry(local_x, local_u, t), sys_id
            )
            merged = merge_geometry(merged, sub_kin, sub_dyn)
        return merged


if __name__ == "__main__":
    # Hello world: unity-feedback loop  dx = Kp (r - x)
    from minilink.blocks.basic import Integrator
    from minilink.control.linear import ProportionalController

    diagram = DiagramSystem()
    diagram.add_subsystem(ProportionalController(), "ctl")
    diagram.add_subsystem(Integrator(), "plant")
    diagram.add_input_port("r")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("plant", "y", "ctl", "y")
    diagram.connect("ctl", "u", "plant", "u")

    evaluator = diagram.compile()
    print(evaluator.f(x=np.array([0.5]), u=np.array([1.0]), t=0.0))
