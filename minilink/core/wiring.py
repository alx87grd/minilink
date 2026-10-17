"""Diagram wiring: blocks joined output to input, shared by flow and step diagrams."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING

import numpy as np

from minilink.core.backends import array_module
from minilink.core.distributions import child_seed
from minilink.core.feedback import check_reserved_id
from minilink.core.signals import OutputPort, VectorSignal
from minilink.core.system import DEFAULT_SMALLEST_TIME_CONSTANT

if TYPE_CHECKING:
    from minilink.core.diagram import DiagramSystem


###############################################################################
# Public API
###############################################################################


class WiredDiagramMixin:
    """
    Evolution-agnostic diagram wiring, gather, and visualization helpers.

    Shared by :class:`~minilink.core.diagram.DiagramSystem` and
    :class:`~minilink.core.diagram.StepDiagramSystem`. Flow diagrams pass
    simulation time ``t`` (float) as the third gather slot; step diagrams pass
    step index ``k`` (int) in the same parameter position.
    """

    def init_wiring(self, *, name: str = "Diagram") -> None:
        """The wiring memory and the stacked state of a new diagram, set up by its ``__init__``.

        The diagram sets its subsystem registry and wiring table first, before
        ``System.__init__``, whose params setter reads them.
        """
        # Wiring memory: a quiet connect, and the diagram's name
        self.connection_verbose = False
        self.name = name

        # The stacked state of the (still empty) subsystems
        self.n, self.state_index, self.state, self.x0, self.derived_x0 = state_layout(
            self.subsystems
        )

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

    @property
    def is_random(self):
        """Whether any subsystem draws random numbers from a seed in its ``params``."""
        return any(subsystem.is_random for subsystem in self.subsystems.values())

    # Building the diagram

    def add_subsystem(self, sys, sys_id):
        """Add a subsystem under a unique id, with all its inputs unconnected.

        The ids ``plant``, ``controller`` and ``observer`` name the roles every
        tool reads (:func:`minilink.core.feedback.roles`); one of each per level.
        """
        check_reserved_id(sys_id, self.subsystems)

        self.subsystems[sys_id] = sys
        self.connections[sys_id] = {port_id: None for port_id in sys.inputs}

        # The stacked state and the solver hints, with the new block
        self.n, self.state_index, self.state, self.x0, self.derived_x0 = state_layout(
            self.subsystems
        )
        self.refresh_solver_info()

    def connect(self, source_sys_id, source_port_id, target_sys_id, target_port_id):
        """
        Connect a source output port to a target input port.

        ``source_sys_id="input"`` connects from a diagram boundary input;
        ``target_sys_id="output"`` connects to a diagram boundary output
        (used by :meth:`connect_new_output_port`). Existence and dimension
        are validated immediately so wiring mistakes fail at connect time,
        not at compile or simulation time.
        """
        self.require_connectable(
            source_sys_id, source_port_id, target_sys_id, target_port_id
        )

        # The target input reads the source output
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
        self,
        source_sys_id,
        source_port_id,
        output_port_id,
        dependencies=None,
        *,
        nominal_value=None,
        labels=None,
        units=None,
        lower_bound=None,
        upper_bound=None,
    ):
        """
        Expose a subsystem output port as a new diagram boundary output.

        ``dependencies=None`` derives the port's direct feedthrough from the
        wiring (see :class:`DiagramOutputPort`); a sequence of input ids or
        ``"all"`` declares it instead. The remaining keywords are the port's
        metadata, as in :meth:`add_output_port`.
        """
        port = self.subsystems[source_sys_id].outputs[source_port_id]
        if dependencies is not None:
            self.validate_output_dependencies(output_port_id, dependencies)

        # The boundary output, evaluated through the diagram
        self.outputs[output_port_id] = DiagramOutputPort(
            self,
            source_sys_id,
            source_port_id,
            output_port_id,
            dim=port.dim,
            dependencies=dependencies,
            nominal_value=nominal_value,
            labels=labels,
            units=units,
            lower_bound=lower_bound,
            upper_bound=upper_bound,
        )

        # Wired from the subsystem port it exposes
        self.connect(source_sys_id, source_port_id, "output", output_port_id)

    def expose(self, *names):
        """
        Put inside ports on the diagram's edge, by name: ``box.expose("r", "y", "u")``.

        Each name is resolved in this order: the open inputs of that name inside
        the diagram become one edge input that feeds all of them; the one inside
        output of that name becomes an edge output; ``"x"`` with no such port is
        the box's own state, the stack of its blocks' states, as an edge output.
        A name that is none of these, or an output several blocks carry, is
        refused. Call it once the box is wired (after ``autowire`` or
        ``connect``: an input already connected is not open) and complete.
        Returns the diagram, for chaining.
        """
        for name in names:
            open_inputs = [
                (sys_id, port_id)
                for sys_id, sys in self.subsystems.items()
                for port_id in sys.inputs
                if port_id == name and self.connections[sys_id][port_id] is None
            ]
            outputs = [
                sys_id for sys_id, sys in self.subsystems.items() if name in sys.outputs
            ]

            # The open inputs of that name: one edge input that feeds them all
            if open_inputs:
                self.expose_input(name, open_inputs)
                continue

            # Otherwise the one inside output of that name, or the box's own state
            self.require_exposable(name, outputs)

            if len(outputs) == 1:
                port = self.subsystems[outputs[0]].outputs[name]
                self.connect_new_output_port(
                    outputs[0],
                    name,
                    name,
                    nominal_value=port.nominal_value,
                    labels=port.labels,
                    units=port.units,
                    lower_bound=port.lower_bound,
                    upper_bound=port.upper_bound,
                )
            else:
                self.add_output_port(
                    "x",
                    dim=self.n,
                    function=self.compute_state,
                    dependencies=(),
                    labels=list(self.state.labels),
                    units=list(self.state.units),
                )

        return self

    def autowire(self, *, strict: bool = False, validate: bool = True) -> DiagramSystem:
        """
        Connect unconnected inputs by name: each takes the one output with its name and size.

        This is an optional diagram-building shortcut. It does not overwrite
        existing connections and returns ``self`` so it can be used fluently.
        """
        # A real cycle: composition imports DiagramSystem, built on this mixin
        from minilink.core.composition import autowire

        return autowire(self, strict=strict, validate=validate)

    # Naming a block and its signals

    def subsystem_id(self, subsystem):
        """Return the diagram id for a subsystem instance added to this diagram."""
        matches = [
            sys_id for sys_id, sub in self.subsystems.items() if sub is subsystem
        ]
        require_added_once(matches, self.subsystems)

        return matches[0]

    def subsystem_signal(self, subsystem, port_id: str) -> str:
        """Return ``'{sys_id}:{port_id}'`` for a subsystem output port."""
        sys_id = self.subsystem_id(subsystem)
        outputs = self.subsystems[sys_id].outputs
        require_output_port(outputs, sys_id, port_id)

        return f"{sys_id}:{port_id}"

    # Refresh, random draws and topology

    def refresh(self):
        """Refresh all subsystems and rebuild the flattened state metadata.

        The solver hints are then bubbled again from the refreshed subsystems
        (``refresh_solver_info``). Compiled evaluators are snapshots:
        recompile after structural changes. An ``x0`` set on the diagram itself
        is kept; otherwise the subsystems' current ``x0`` are stacked again.
        """
        # An x0 set on the diagram differs from the one stacked from its subsystems
        x0 = np.copy(self.x0)
        derived_x0 = self.derived_x0
        user_set_x0 = x0.shape == derived_x0.shape and not np.array_equal(
            x0, derived_x0
        )

        # The subsystems first, then the stack of their metadata
        for subsystem in self.subsystems.values():
            subsystem.refresh()
        self.n, self.state_index, self.state, self.x0, self.derived_x0 = state_layout(
            self.subsystems
        )

        # The diagram's own x0 stays while its size still fits
        if user_set_x0 and x0.shape == self.x0.shape:
            self.x0 = x0

        self.refresh_solver_info()

    def realize(self, key):
        """Nested params with every random block realized under its own stream, named by its id."""
        return {
            sys_id: subsystem.realize(None if key is None else child_seed(key, sys_id))
            for sys_id, subsystem in self.subsystems.items()
        }

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

    # Gather: one block's state, input and params from the diagram's

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

        # Each input port's value: its source's, or its nominal one outside dependencies
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

        # The local input stacks the port values, one block per port
        if pieces:
            local_u = xp.concatenate(
                [xp.asarray(piece).reshape(-1) for piece in pieces]
            )
        else:
            local_u = xp.array([])

        return local_u

    def subsystem_params(self, params, sys_id):
        """Route nested diagram params to one subsystem (strict contract).

        ``None`` → ``None`` (subsystem uses its live ``self.params``);
        a mapping → ``params.get(sys_id)`` (missing id → ``None`` → defaults).
        Anything else is a contract violation.
        """
        if params is None:
            return None
        require_params_mapping(params)

        return params.get(sys_id)

    def compute_subsys_output_port(self, x, u, t, sys_id, port_id, params=None):
        """Evaluate one subsystem output port, recursing through its sources."""
        port = self.subsystems[sys_id].outputs[port_id]
        local_x = self.get_local_state(x, sys_id)
        local_u = self.get_local_input(
            x, u, t, sys_id, port.dependencies, params=params
        )
        local_params = self.subsystem_params(params, sys_id)

        return port.compute(local_x, local_u, t, local_params)

    # Graphics: every block's frames and geometry, under its id

    def get_kinematic_geometry(self):
        """No geometry of its own: :meth:`get_dynamic_geometry` draws every block's."""
        return {}

    def tf(self, x, u, t=0, params=None):
        """Every block's frames ``T = tf(x, u, t)``, keyed under its id."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation.visualization import (
            namespace_subsystem_frames,
        )

        validate_diagram_params(params, self.subsystems)

        frames = {}
        for sys_id, subsystem in self.subsystems.items():
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id, params=params)
            local_params = self.subsystem_params(params, sys_id)
            sub_frames = dict(subsystem.tf(local_x, local_u, t, local_params))
            frames.update(namespace_subsystem_frames(sub_frames, sys_id))

        return frames

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """Every block's geometry, static and moving, keyed under its id."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation.visualization import (
            merge_geometry,
            merge_subsystem_geometry,
        )

        validate_diagram_params(params, self.subsystems)

        merged: dict[str, list] = {}
        for sys_id, subsystem in self.subsystems.items():
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id, params=params)
            local_params = self.subsystem_params(params, sys_id)
            sub_kin: dict[str, list] = {}
            sub_dyn: dict[str, list] = {}
            merge_subsystem_geometry(
                sub_kin, subsystem.get_kinematic_geometry(), sys_id
            )
            merge_subsystem_geometry(
                sub_dyn,
                subsystem.get_dynamic_geometry(local_x, local_u, t, local_params),
                sys_id,
            )
            merged = merge_geometry(merged, sub_kin, sub_dyn)

        return merged

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def refresh_solver_info(self):
        """Bubble subsystem solver hints to the diagram root.

        The diagram is discontinuous when any subsystem is. Its smallest time
        constant is the smallest one among the subsystems that carry one: every
        stateful subsystem, and a stateless block that sets its own. A static
        block left at the default has no time constant and does not pin the
        diagram to the default. Its sample period is the shortest one among the
        sampled subsystems, and it has none when no subsystem is sampled.

        The hints are derived from the subsystems at every :meth:`add_subsystem`
        and :meth:`refresh` (a ``Simulator`` refreshes before it solves), so a
        hint changed on a block after composition reaches the diagram. Set a
        hint on the block it describes: one set by hand on the diagram is
        replaced at the next refresh.
        """
        if not hasattr(self, "solver_info"):
            return

        # Discontinuous when any subsystem is
        self.solver_info["discontinuous_behavior"] = any(
            subsystem.solver_info.get("discontinuous_behavior", False)
            for subsystem in self.subsystems.values()
        )

        # The smallest time constant among the subsystems that carry one
        time_constants = []
        for subsystem in self.subsystems.values():
            tau = subsystem.solver_info.get(
                "smallest_time_constant", DEFAULT_SMALLEST_TIME_CONSTANT
            )
            if subsystem.n > 0 or tau != DEFAULT_SMALLEST_TIME_CONSTANT:
                time_constants.append(tau)
        self.solver_info["smallest_time_constant"] = min(
            time_constants, default=DEFAULT_SMALLEST_TIME_CONSTANT
        )

        # The shortest sample period, when any subsystem is sampled
        periods = [
            subsystem.solver_info["sample_period"]
            for subsystem in self.subsystems.values()
            if "sample_period" in subsystem.solver_info
        ]
        if periods:
            self.solver_info["sample_period"] = min(periods)
        else:
            self.solver_info.pop("sample_period", None)

    def require_connectable(
        self, source_sys_id, source_port_id, target_sys_id, target_port_id
    ):
        """Refuse a connection whose ports do not exist or differ in size."""
        source_port = self.source_port_of(source_sys_id, source_port_id)
        target_port = self.target_port_of(target_sys_id, target_port_id)

        if source_port.dim != target_port.dim:
            raise ValueError(
                f"Port dimension mismatch: {source_sys_id}:{source_port_id} "
                f"has dim {source_port.dim}, {target_sys_id}:{target_port_id} "
                f"has dim {target_port.dim}"
            )

    def source_port_of(self, source_sys_id, source_port_id):
        """The port a connection reads: a boundary input, or a subsystem output."""
        if source_sys_id == "input":
            if source_port_id not in self.inputs:
                raise ValueError(
                    f"Unknown diagram input port '{source_port_id}'; "
                    f"available: {', '.join(self.inputs) or '(none)'}"
                )
            return self.inputs[source_port_id]

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
        return source_outputs[source_port_id]

    def target_port_of(self, target_sys_id, target_port_id):
        """The port a connection feeds: a boundary output, or a subsystem input."""
        if target_sys_id == "output":
            if target_port_id not in self.outputs:
                raise ValueError(
                    f"Unknown diagram output port '{target_port_id}'; "
                    f"available: {', '.join(self.outputs) or '(none)'}"
                )
            return self.outputs[target_port_id]

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
        return target_inputs[target_port_id]

    def expose_input(self, name, targets):
        """One edge input ``name`` feeding the open inputs ``targets``."""
        self.require_new_input(name)
        ports = [self.subsystems[sys_id].inputs[port_id] for sys_id, port_id in targets]
        dims = {port.dim for port in ports}
        require_one_size(name, dims)

        # The edge input takes the first open input's size and metadata
        first = ports[0]
        self.add_input_port(
            name,
            dim=first.dim,
            nominal_value=first.nominal_value,
            labels=first.labels,
            units=first.units,
            lower_bound=first.lower_bound,
            upper_bound=first.upper_bound,
        )

        # It feeds every open input of that name
        for sys_id, port_id in targets:
            self.connect("input", name, sys_id, port_id)

    def require_new_input(self, name):
        """Refuse an edge input name the diagram already carries."""
        if name in self.inputs:
            raise ValueError(
                f"{name!r} is already an input on the edge of {self.name!r}"
            )

    def require_exposable(self, name, outputs):
        """Refuse an output several blocks carry, and a name with nothing to expose."""
        if len(outputs) > 1:
            raise ValueError(
                f"{name!r} is an output of several blocks ({', '.join(outputs)}); "
                "expose one of them with connect_new_output_port"
            )
        if not outputs and name != "x":
            raise ValueError(
                f"{name!r} is neither an open input nor an output of a block in "
                f"{self.name!r}; blocks: {', '.join(self.subsystems) or '(none)'}"
            )

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


class DiagramOutputPort(OutputPort):
    """
    A diagram boundary output: one subsystem output port exposed on the diagram.

    Its direct feedthrough is derived from the current wiring
    (``feedthrough_inputs``) unless declared explicitly, so a nested
    diagram reports exactly the dependencies a leaf would, and wiring done
    after the port was exposed is still accounted for.
    """

    def __init__(
        self,
        diagram,
        source_sys_id,
        source_port_id,
        id,
        *,
        dim=None,
        dependencies=None,
        nominal_value=None,
        labels=None,
        units=None,
        lower_bound=None,
        upper_bound=None,
    ):
        # The diagram and the subsystem port it exposes; set before
        # OutputPort.__init__, whose dependencies go through the property below
        self.diagram = diagram
        self.source = (source_sys_id, source_port_id)
        self.declared_dependencies = None

        # The port, evaluated through the diagram
        OutputPort.__init__(
            self,
            id,
            dim=dim,
            function=self.source_output,
            dependencies=dependencies,
            nominal_value=nominal_value,
            labels=labels,
            units=units,
            lower_bound=lower_bound,
            upper_bound=upper_bound,
        )

    @property
    def dependencies(self):
        """Declared dependencies, else the boundary inputs the source feeds through from."""
        if self.declared_dependencies is not None:
            return self.declared_dependencies
        return feedthrough_inputs(self.diagram, *self.source)

    @dependencies.setter
    def dependencies(self, value):
        self.declared_dependencies = value

    def source_output(self, x, u, t=0, params=None):
        """The exposed subsystem port, evaluated through the diagram."""
        source_sys_id, source_port_id = self.source
        return self.diagram.compute_subsys_output_port(
            x, u, t, source_sys_id, source_port_id, params=params
        )


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

    require_params_mapping(params)

    unknown = [key for key in params if key not in subsystem_ids]
    if unknown:
        names = ", ".join(repr(key) for key in unknown)
        available = ", ".join(repr(key) for key in subsystem_ids)
        raise ValueError(
            f"Unknown subsystem ids in diagram params: {names}; available: {available}"
        )


def check_algebraic_loops(diagram: DiagramSystem) -> list[tuple[str, str]]:
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

        # A port already on the search path closes an algebraic loop
        require_no_algebraic_loop(node, stack, stack_set)

        # A port already scheduled
        if node in visited:
            return

        # The sources the port feeds through from come first
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

        # Its sources scheduled, the port takes its place in the order
        stack.pop()
        stack_set.remove(node)
        visited.add(node)
        order.append(node)

    # Every output port of every subsystem, in insertion order
    for sys_id, subsystem in diagram.subsystems.items():
        for port_id in subsystem.outputs:
            visit_port(sys_id, port_id)

    return order


def iter_subsystems(diagram, prefix=()):
    """Yield ``(id_path, subsystem)`` for every block of a diagram, nested diagrams included."""
    for sys_id, subsystem in diagram.subsystems.items():
        path = (*prefix, sys_id)
        yield path, subsystem
        if hasattr(subsystem, "subsystems"):
            yield from iter_subsystems(subsystem, path)


###############################################################################
# Internal machinery
###############################################################################


def state_layout(subsystems):
    """
    The stacked state of a diagram's blocks: ``(n, state_index, state, x0, derived_x0)``.

    The stacked state vector is ``x = [x_1; x_2; ...]`` in subsystem insertion
    order; ``state_index`` maps each subsystem id to its ``(start, end)``
    indices in the stacked vector. State labels are prefixed with the
    subsystem id whenever the same label appears in several subsystems.
    ``derived_x0`` is a copy of the stacked ``x0``, so a refresh can tell an
    ``x0`` set on the diagram from a stale stacked one.
    """
    # x = [x_1; x_2; ...]: each subsystem's (start, end) in the stack
    state_index = {}
    idx = 0
    for sys_id, subsystem in subsystems.items():
        state_index[sys_id] = (idx, idx + subsystem.n)
        idx += subsystem.n

    # A label several subsystems share takes its subsystem id as a prefix
    owned_labels = [
        (sys_id, label)
        for sys_id, subsystem in subsystems.items()
        for label in subsystem.state.labels
    ]
    label_counts = {}
    for _, label in owned_labels:
        label_counts[label] = label_counts.get(label, 0) + 1
    labels = [
        f"{sys_id}:{label}" if label_counts[label] > 1 else label
        for sys_id, label in owned_labels
    ]

    # The stacked state signal, and its default initial condition
    n = idx
    state = VectorSignal("x", dim=n)
    x0 = np.zeros(n)

    # The subsystems' metadata and x0, stacked in insertion order
    if subsystems:
        substates = [sub.state for sub in subsystems.values()]
        state.labels = labels
        state.units = [unit for s in substates for unit in s.units]
        state.upper_bound = np.concatenate([s.upper_bound for s in substates])
        state.lower_bound = np.concatenate([s.lower_bound for s in substates])
        state.nominal_value = np.concatenate([s.nominal_value for s in substates])
        x0 = np.concatenate([sub.x0 for sub in subsystems.values()])

    # The x0 stacked from the subsystems
    derived_x0 = np.copy(x0)

    return n, state_index, state, x0, derived_x0


def require_params_mapping(params):
    """Refuse diagram params that are not a mapping keyed by subsystem id."""
    if not isinstance(params, Mapping):
        raise TypeError(
            "Diagram params must be a mapping keyed by subsystem id "
            f"(e.g. {{'plant': {{...}}}}), got {type(params).__name__}"
        )


def require_one_size(name, dims):
    """Refuse open inputs of one name that differ in size."""
    if len(dims) != 1:
        raise ValueError(
            f"the open inputs named {name!r} have different sizes ({sorted(dims)}); "
            "wire them with connect"
        )


def require_added_once(matches, subsystem_ids):
    """Refuse a block the diagram does not hold, or holds under several ids."""
    if not matches:
        raise ValueError(
            "Subsystem is not part of this diagram. "
            f"Available ids: {', '.join(subsystem_ids) or '(none)'}"
        )
    if len(matches) > 1:
        rendered = ", ".join(matches)
        raise ValueError(
            "Subsystem appears more than once in this diagram "
            f"({rendered}); use an explicit 'sys_id:port' name instead."
        )


def require_output_port(outputs, sys_id, port_id):
    """Refuse a port id the subsystem does not carry as an output."""
    if port_id not in outputs:
        raise ValueError(
            f"Unknown output port {port_id!r} on subsystem {sys_id!r}; "
            f"available: {', '.join(outputs) or '(none)'}"
        )


def feedthrough_inputs(diagram, sys_id, port_id) -> tuple[str, ...]:
    """
    Diagram boundary inputs that reach one subsystem output by direct feedthrough.

    Walks the ``dependencies`` edges back from ``sys_id:port_id``; a state
    breaks the path, a boundary input ends it. The result is what a leaf would
    declare for the same port, in diagram input order. An algebraic loop inside
    the diagram ends the walk here and is reported by :func:`check_algebraic_loops`.
    """
    reached = set()
    visited = set()
    stack = [(sys_id, port_id)]

    # Back along the feedthrough edges: a boundary input ends a path, an output
    # continues it
    while stack:
        node = stack.pop()
        if node in visited:
            continue
        visited.add(node)

        source_sys_id, source_port_id = node
        subsystem = diagram.subsystems[source_sys_id]
        deps = subsystem.outputs[source_port_id].dependencies
        for in_port_id in subsystem.inputs if deps == "all" else deps:
            source = diagram.connections[source_sys_id].get(in_port_id)
            if source is None:
                continue
            if source[0] == "input":
                reached.add(source[1])
            else:
                stack.append(source)

    return tuple(input_id for input_id in diagram.inputs if input_id in reached)


def require_no_algebraic_loop(node, stack, stack_set):
    """Refuse a port already on the search path, which closes an algebraic loop."""
    if node in stack_set:
        cycle_start_idx = stack.index(node)
        cycle_path = stack[cycle_start_idx:] + [node]
        cycle_str = " -> ".join(f"{s}:{p}" for s, p in cycle_path)
        raise RuntimeError(f"Algebraic loop detected: {cycle_str}")
