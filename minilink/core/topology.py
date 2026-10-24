"""Diagram topology: the order a diagram evaluates its output ports in, and its algebraic loops."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from minilink.core.diagram import DiagramSystem


###############################################################################
# Public API
###############################################################################


def check_algebraic_loops(diagram: DiagramSystem) -> None:
    """Check a wired diagram for algebraic loops.

    An algebraic loop exists when a cycle contains only direct-feedthrough
    paths (i.e., output ports whose ``dependencies`` include the input ports
    that close the cycle). This check is **standalone**: call it right after
    wiring a diagram to validate its topology before simulation; the order
    the compiler evaluates the ports in is :func:`evaluation_order_of`.

    Parameters
    ----------
    diagram : DiagramSystem
        The wired diagram to analyse.

    Raises
    ------
    RuntimeError
        If an algebraic loop is detected, with the full cycle path in the
        error message.
    """
    evaluation_order_of(diagram)


def evaluation_order_of(diagram: DiagramSystem) -> list[tuple[str, str]]:
    """The topological evaluation order of a diagram's output ports.

    Uses depth-first search over output-port dependency edges; a cycle of
    direct-feedthrough paths (an algebraic loop) is refused.

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


###############################################################################
# Internal machinery
###############################################################################


def require_no_algebraic_loop(node, stack, stack_set):
    """Refuse a port already on the search path, which closes an algebraic loop."""
    if node in stack_set:
        cycle_start_idx = stack.index(node)
        cycle_path = stack[cycle_start_idx:] + [node]
        cycle_str = " -> ".join(f"{s}:{p}" for s, p in cycle_path)
        raise RuntimeError(f"Algebraic loop detected: {cycle_str}")
