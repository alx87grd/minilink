"""Composition operators: ``+`` and ``>>`` build a diagram in one line, ``autowire`` wires one by port names."""

from __future__ import annotations

import warnings

from minilink.core.assembly import (
    add_input_port_like,
    add_system_to_diagram,
    algebra_of,
    carry_left_operand,
    default_output_port,
    inline_diagram,
    is_box,
    propagate_animation_camera,
)
from minilink.core.diagram import DiagramSystem
from minilink.core.system import System

#: Inputs that read their nominal value when left open, without a word from
#: autowire: a plant's disturbance and measurement noise.
_NOMINAL_WHEN_OPEN = frozenset({"d", "n"})


###############################################################################
# Public API
###############################################################################

# No operator modifies its operands: each returns a new diagram that shares the
# operands' blocks. ``+`` and ``>>`` lay a diagram operand out flat, with its ids,
# wiring, boundary ports, ``x0`` and camera settings, and keep the left operand's
# name. ``a >> b`` connects one output of ``a`` to one input of ``b`` by a fixed
# rule on their edges, with no memory of how either was built. Only ``@``
# (:mod:`minilink.core.closed_loop`) nests: a box on either side sits under its
# role name as one block. Explicit ``add_subsystem`` / ``connect`` remains the
# canonical way to build any topology.


def add_systems(*systems: System | DiagramSystem) -> DiagramSystem:
    """Return a new flat diagram containing ``systems`` as subsystems.

    Parameters
    ----------
    *systems : System or DiagramSystem
        Systems or diagrams to add, side by side. A diagram operand is laid
        out flat in the result, with its ids, wiring and boundary ports; it is
        not modified. The first operand's name, ``x0`` and camera settings
        are carried into the result when it is a diagram.

    Returns
    -------
    DiagramSystem
        A diagram containing all provided systems as direct subsystems.
        Existing diagram-internal connections are preserved, but no new
        cross-connections are inferred.
    """
    diagram = algebra_of(*systems)()

    # Each operand side by side: a diagram laid out flat, a leaf as one block
    for sys in systems:
        if is_box(sys):
            inline_diagram(diagram, sys, output_collision="unique")
        else:
            add_system_to_diagram(diagram, sys)

    # The first operand's name, x0 and camera, when it is a diagram; else the
    # sum as written
    if systems:
        carry_left_operand(diagram, systems[0])
        if not is_box(systems[0]):
            diagram.name = " + ".join(sys.name for sys in systems)

    return diagram


def series(
    left: System | DiagramSystem,
    right: System | DiagramSystem,
    *rest: System | DiagramSystem,
) -> DiagramSystem:
    """Connect systems in series: one output of each stage to one input of the next.

    ``A >> B`` wires one output of A to one input of B, the first rule that
    applies: A has one output and B one input; exactly one output of A and
    one input of B share a name; A has ``y`` and B has ``u``; else it refuses.
    A diagram counts its edge (its boundary outputs and its free boundary
    inputs), a leaf its ports. Sizes are a check, never a choice. One guard: a
    source into a block that reads both ``r`` and ``y`` is refused, since the
    name match would feed the measurement. The result keeps A's free inputs as
    its inputs and B's ``y`` (else its only output) as its output ``y``.

    Parameters
    ----------
    left : System or DiagramSystem
        The first system or diagram; a diagram is laid out flat in the result
        and left unchanged, its ``x0`` and camera settings carried.
    right : System or DiagramSystem
        System or diagram to connect after ``left``.
    *rest : System or DiagramSystem
        Additional systems or diagrams to append to the series chain.

    Returns
    -------
    DiagramSystem
        A new diagram with each stage connected output-to-input, named after
        every operand (``"Step >> PID >> Pendulum"``).
    """
    # The chain so far: a diagram operand copied, a leaf with its inputs on the edge,
    # in the algebra of every stage
    diagram = as_composition_diagram(left, algebra_of(left, right, *rest))

    # Each next stage on the chain's output, by the ``>>`` rule
    series_pair(diagram, right)
    for next_sys in rest:
        series_pair(diagram, next_sys)

    # The left operand's x0 and camera, when it is a diagram; the chain's name
    # carries every operand's
    carry_left_operand(diagram, left)
    diagram.name = " >> ".join(sys.name for sys in (left, right, *rest))

    return diagram


def autowire(
    diagram: DiagramSystem,
    *,
    strict: bool = False,
    validate: bool = True,
) -> DiagramSystem:
    """Connect unconnected inputs by name: each takes the one output with its name and size.

    Parameters
    ----------
    diagram : DiagramSystem
        Diagram to modify in place.
    strict : bool, optional
        If ``True``, raise when an input stays open (no match, or several). No
        new connections are applied before that error is raised. If ``False``,
        the open inputs are left unconnected and a warning lists them.
    validate : bool, optional
        If ``True``, run algebraic-loop detection after wiring.

    Returns
    -------
    DiagramSystem
        The same diagram, for fluent use.

    Notes
    -----
    Names only, no guessing: an output may feed several inputs, existing
    connections are never overwritten, and a disturbance ``d`` or a noise
    ``n`` left open reads its nominal value without a word. Name a source's
    signal after the input it drives (``Step(port="r")``) and a block's
    ports at creation (``rename_ports``).
    """
    require_diagram(diagram)

    # Each open input takes the one output of another block with its name and size;
    # none, or several, leave it open (a d or an n quietly reads its nominal value)
    decisions = []
    open_inputs = []
    for target_id, target in diagram.subsystems.items():
        for input_id, input_port in target.inputs.items():
            if diagram.connections[target_id][input_id] is not None:
                continue

            candidates = [
                (source_id, port_id)
                for source_id, source in diagram.subsystems.items()
                if source_id != target_id
                for port_id, output in source.outputs.items()
                if port_id == input_id and output.dim == input_port.dim
            ]
            if len(candidates) == 1:
                source_id, source_port = candidates[0]
                decisions.append((source_id, source_port, target_id, input_id))
            elif len(candidates) > 1:
                rendered = ", ".join(f"{s}:{p}" for s, p in candidates)
                open_inputs.append(
                    f"Ambiguous autowire target {target_id}:{input_id}; "
                    f"candidates: {rendered}"
                )
            elif input_id not in _NOMINAL_WHEN_OPEN:
                open_inputs.append(
                    f"no output named {input_id!r} for {target_id}:{input_id}"
                )

    # Strict: an open input is refused before any connection; else a warning
    # lists them
    if strict:
        require_wired(open_inputs)
    if open_inputs:
        warnings.warn(
            "autowire left these inputs unconnected (wire them with connect): "
            + "; ".join(open_inputs),
            stacklevel=3,
        )

    # The connections, once every input is decided
    for source_id, source_port, target_id, input_id in decisions:
        diagram.connect(source_id, source_port, target_id, input_id)

    if validate:
        diagram.check_algebraic_loops()

    return diagram


###############################################################################
# Internal machinery
###############################################################################


def as_composition_diagram(sys, algebra) -> DiagramSystem:
    """A diagram operand as a copy, a leaf as a diagram with its inputs on the edge, in ``algebra``."""
    diagram = algebra()
    if is_box(sys):
        inline_diagram(diagram, sys, output_collision="replace")
        carry_left_operand(diagram, sys)
        return diagram

    sys_id = add_system_to_diagram(diagram, sys)
    expose_free_inputs(diagram, sys_id)

    return diagram


def expose_free_inputs(diagram, sys_id: str) -> None:
    """Put every input of a leaf on the diagram's edge, under the same name."""
    sys = diagram.subsystems[sys_id]
    for port_id, port in sys.inputs.items():
        add_input_port_like(diagram, port_id, port)
        diagram.connect("input", port_id, sys_id, port_id)


# The series rule


def series_pair(diagram, right) -> None:
    """Append ``right`` to ``diagram`` (the chain so far) by the ``>>`` rule."""
    # The chain's output and the right side's input the rule connects
    outputs = edge_outputs(diagram)
    source_name, target_name = series_connection(diagram, outputs, right.inputs)
    source_id, source_port = outputs[source_name]

    # The chain's outputs are consumed; the result's output is the right side's
    for output_id in list(diagram.outputs):
        del diagram.outputs[output_id]
        diagram.connections["output"].pop(output_id, None)

    if is_box(right):
        # A box laid out flat, its input bound to the chain's output
        inline_diagram(
            diagram,
            right,
            input_bindings={target_name: (source_id, source_port)},
            output_collision="replace",
        )
    else:
        # A leaf as one block, its default output as the result's y
        right_id = add_system_to_diagram(diagram, right)
        diagram.connect(source_id, source_port, right_id, target_name)
        diagram.connect_new_output_port(right_id, default_output_port(right), "y")
        propagate_animation_camera(diagram, right)


def edge_outputs(diagram) -> dict:
    """The chain's outputs by name: its boundary outputs, or the ports of its one block."""
    if diagram.outputs:
        sources = diagram.connections.get("output", {})
        # A box's own state (``expose("x")``) has no source block; it is not a chain output.
        return {
            output_id: tuple(sources[output_id])
            for output_id in diagram.outputs
            if sources.get(output_id) is not None
        }

    if len(diagram.subsystems) == 1:
        ((sys_id, sys),) = diagram.subsystems.items()
        return {port_id: (sys_id, port_id) for port_id in sys.outputs}

    raise ValueError("Left side of '>>' has no output port to connect")


def series_connection(diagram, outputs, inputs) -> tuple[str, str]:
    """The ``(output, input)`` names ``>>`` connects, by the first rule that applies."""
    if not outputs:
        raise ValueError("Left side of '>>' has no output port to connect")
    if not inputs:
        raise ValueError("Right side of '>>' has no free input port to connect")

    # One output and one input
    if len(outputs) == 1 and len(inputs) == 1:
        return next(iter(outputs)), next(iter(inputs))

    # One shared name; a source (a chain with no inputs) into a block that reads r
    # and y would feed the measurement, and is refused
    shared = [name for name in outputs if name in inputs]
    if shared == ["y"] and "r" in inputs and not diagram.inputs:
        raise ValueError(
            "a source into a block that reads 'r' and 'y' would drive the measurement "
            "'y'; name the source's signal after the input it drives (port='r') for "
            "the block alone, or feed the loop: source >> (controller @ plant)"
        )
    if len(shared) == 1:
        return shared[0], shared[0]

    # The standard pair: y into u
    if "y" in outputs and "u" in inputs:
        return "y", "u"

    # Else refused, with a hint for a loop's reference
    hint = (
        "; a source for a loop's reference goes in closed_loop(..., reference=source)"
        if "r" in inputs
        else ""
    )
    raise ValueError(
        "'>>' connects one output to one input; wire this one with connect "
        f"(outputs: {', '.join(outputs)}; free inputs: {', '.join(inputs)}){hint}"
    )


# The refusals, in the order the public verbs meet them


def require_diagram(diagram) -> None:
    """Refuse anything but a diagram to wire."""
    if not is_box(diagram):
        raise TypeError("autowire() expects a DiagramSystem")


def require_wired(open_inputs) -> None:
    """Refuse the first input left open: no output of its name and size, or several."""
    if open_inputs:
        raise ValueError(open_inputs[0])
