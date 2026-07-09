"""
Shared diagram wiring: mixin, params validation, and topology checks.

Evolution-agnostic machinery shared by :class:`~minilink.core.diagram.DiagramSystem`
and :class:`~minilink.core.diagram.StepDiagramSystem`. Flow diagrams pass simulation
time ``t`` (float) as the third gather slot; step diagrams pass step index ``k`` (int)
in the same parameter position.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING

import numpy as np

from minilink.core.backends import array_module
from minilink.core.signals import VectorSignal

if TYPE_CHECKING:
    from minilink.core.diagram import DiagramSystem


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


def check_algebraic_loops(
    diagram: DiagramSystem,
) -> list[tuple[str, str]]:
    """Detect algebraic loops and return the topological port execution order.

    Uses depth-first search over output-port dependency edges.  An algebraic
    loop exists when a cycle contains only direct-feedthrough paths (i.e.,
    output ports whose ``dependencies`` include the input ports that close
    the cycle).

    This function is **standalone** and can be called independently of
    :func:`~minilink.core.compile.compiler.build_execution_plan` — for example,
    right after wiring a diagram to validate its topology before simulation.

    Parameters
    ----------
    diagram : DiagramSystem
        The wired diagram to analyse.

    Returns
    -------
    list[tuple[str, str]]
        Output ports ``(sys_id, port_id)`` in valid topological order
        (dependencies before dependents).

    Raises
    ------
    RuntimeError
        If an algebraic loop is detected, with the full cycle path in the
        error message.
    """
    visited: set[tuple[str, str]] = set()
    stack: list[tuple[str, str]] = []
    stack_set: set[tuple[str, str]] = set()
    order: list[tuple[str, str]] = []

    def visit_port(sys_id: str, port_id: str) -> None:
        node = (sys_id, port_id)

        if node in stack_set:
            cycle_start_idx = stack.index(node)
            cycle_path = stack[cycle_start_idx:] + [node]
            cycle_str = " -> ".join(f"{s}:{p}" for s, p in cycle_path)
            raise RuntimeError(f"Algebraic loop detected: {cycle_str}")

        if node in visited:
            return

        stack.append(node)
        stack_set.add(node)

        port = diagram.subsystems[sys_id].outputs.get(port_id)
        if port is not None:
            deps = port.dependencies
            sys_inputs = diagram.subsystems[sys_id].inputs
            input_deps = sys_inputs.keys() if deps == "all" else deps

            for in_port_id in input_deps:
                source = diagram.connections[sys_id].get(in_port_id)
                if source is not None:
                    src_sys_id, src_port_id = source
                    if src_sys_id != "input":
                        visit_port(src_sys_id, src_port_id)

        stack.pop()
        stack_set.remove(node)
        visited.add(node)
        order.append(node)

    for sys_id, subsystem in diagram.subsystems.items():
        for port_id in subsystem.outputs:
            visit_port(sys_id, port_id)

    return order


class WiredDiagramMixin:
    """Evolution-agnostic diagram wiring, gather, and visualization helpers."""

    def _init_wiring(self, *, name: str = "Diagram") -> None:
        if not hasattr(self, "subsystems"):
            self.subsystems = {}
        if not hasattr(self, "connections"):
            self.connections = {}
        self.connection_verbose = False
        self._composition_entry = None
        self._composition_output = None
        self.name = name
        self.compute_state_properties()

    def add_subsystem(self, sys, sys_id):
        """Add a subsystem under a unique id, with all its inputs unconnected."""
        self.subsystems[sys_id] = sys
        self.connections[sys_id] = {port_id: None for port_id in sys.inputs}

        self.compute_state_properties()
        self._refresh_solver_info()

    def _refresh_solver_info(self):
        """Bubble subsystem solver hints to the diagram root."""
        if not hasattr(self, "solver_info"):
            return
        self.solver_info["discontinuous_behavior"] = any(
            subsystem.solver_info.get("discontinuous_behavior", False)
            for subsystem in self.subsystems.values()
        )

    def subsystem_id(self, subsystem):
        """Return the diagram id for a subsystem instance added to this diagram."""
        matches = [
            sys_id for sys_id, sub in self.subsystems.items() if sub is subsystem
        ]
        if len(matches) == 1:
            return matches[0]
        if not matches:
            raise ValueError(
                "Subsystem is not part of this diagram. "
                f"Available ids: {', '.join(self.subsystems) or '(none)'}"
            )
        rendered = ", ".join(matches)
        raise ValueError(
            "Subsystem appears more than once in this diagram "
            f"({rendered}); use an explicit 'sys_id:port' name instead."
        )

    def subsystem_signal(self, subsystem, port_id: str) -> str:
        """Return ``'{sys_id}:{port_id}'`` for a subsystem output port."""
        sys_id = self.subsystem_id(subsystem)
        outputs = self.subsystems[sys_id].outputs
        if port_id not in outputs:
            raise ValueError(
                f"Unknown output port {port_id!r} on subsystem {sys_id!r}; "
                f"available: {', '.join(outputs) or '(none)'}"
            )
        return f"{sys_id}:{port_id}"

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

    def get_local_state(self, x, sys_id):
        """Extract one subsystem's state from the stacked state vector."""
        start, end = self.state_index[sys_id]
        return x[start:end]

    def get_local_input(self, x, u, t, sys_id, dependencies="all", params=None):
        """
        Assemble one subsystem's local input vector from its connected sources.

        On flow diagrams the third slot is simulation time ``t`` (float).
        On step diagrams the third slot is step index ``k`` (int).

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
        return check_algebraic_loops(self)

    def get_kinematic_geometry(self):
        return {}

    def tf(self, x, u, t=0, params=None):
        from minilink.graphical.animation.visualization import (
            namespace_subsystem_frames,
        )

        frames = {}
        for sys_id, subsystem in self.subsystems.items():
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id)
            sub_frames = dict(subsystem.tf(local_x, local_u, t))
            frames.update(namespace_subsystem_frames(sub_frames, sys_id))
        return frames

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        from minilink.graphical.animation.visualization import (
            merge_geometry,
            merge_subsystem_geometry,
        )

        merged: dict[str, list] = {}
        for sys_id, subsystem in self.subsystems.items():
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id)
            sub_kin: dict[str, list] = {}
            sub_dyn: dict[str, list] = {}
            merge_subsystem_geometry(
                sub_kin, subsystem.get_kinematic_geometry(), sys_id
            )
            merge_subsystem_geometry(
                sub_dyn,
                subsystem.get_dynamic_geometry(local_x, local_u, t),
                sys_id,
            )
            merged = merge_geometry(merged, sub_kin, sub_dyn)
        return merged
