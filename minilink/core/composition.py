"""Composition shortcuts: the ``+``, ``>>`` and ``@`` operators that build a diagram in one line.

No operator modifies its operands: each returns a new diagram that shares the
operands' blocks. ``+`` and ``>>`` lay a diagram operand out flat, with its ids,
wiring, boundary ports, ``x0`` and camera settings, and keep the left operand's
name. ``a >> b`` connects one output of ``a`` to one input of ``b`` by a fixed
rule on their edges, with no memory of how either was built.
Explicit ``add_subsystem`` / ``connect`` remains the canonical way to build any topology.
"""

from __future__ import annotations

import re
import warnings
from dataclasses import dataclass

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.feedback import block_roles, check_reserved_id, command_port
from minilink.core.system import System

_VALID_FEEDBACK = frozenset({"auto", "y", "qdq"})

#: Inputs that read their nominal value when left open, without a word from
#: autowire: a plant's disturbance and measurement noise.
_NOMINAL_WHEN_OPEN = frozenset({"d", "n", "w", "v"})


def _animated_geometry_source(obj):
    """First drawable in *obj* (or its subsystems) with static kinematic geometry."""
    if isinstance(obj, DiagramSystem):
        for sub in obj.subsystems.values():
            if sub.get_kinematic_geometry():
                return sub
        return None
    if obj.get_kinematic_geometry():
        return obj
    return None


def _subsystem_id_for_leaf(diagram: DiagramSystem, leaf: System) -> str | None:
    """Return ``sys_id`` when *leaf* is a direct subsystem of *diagram*."""
    for sys_id, subsystem in diagram.subsystems.items():
        if subsystem is leaf:
            return sys_id
    return None


def _namespaced_camera_follow_frame(
    diagram: DiagramSystem, animated: System, follow: str | None
) -> str | None:
    if follow is None:
        return None
    if ":" in follow:
        return follow
    sys_id = _subsystem_id_for_leaf(diagram, animated)
    if sys_id is None:
        return follow
    return f"{sys_id}:{follow}"


def _propagate_animation_camera(diagram, *sources):
    """Copy camera hints from a plant-like *source* onto *diagram*.

    Prefer a source (or nested leaf) that already carries kinematic geometry —
    e.g. a catalog plant — over a bare ``Step`` / controller that would keep the
    generic auto-fit default (``camera_scale=None``).
    """
    fallback = None
    for source in sources:
        animated = _animated_geometry_source(source)
        if animated is not None:
            _copy_animation_camera_hints(diagram, animated)
            return
        if (
            fallback is None
            and isinstance(source, System)
            and not isinstance(source, DiagramSystem)
        ):
            fallback = source
    if fallback is not None:
        _copy_animation_camera_hints(diagram, fallback)


def _copy_animation_camera_hints(diagram, animated):
    diagram.camera_scale = animated.camera_scale
    diagram.camera_target = np.array(animated.camera_target, dtype=float).copy()
    diagram.camera_plot_axes = getattr(animated, "camera_plot_axes", (0, 1))
    diagram.camera_follow_frame = _namespaced_camera_follow_frame(
        diagram,
        animated,
        getattr(animated, "camera_follow_frame", None),
    )


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
    diagram = DiagramSystem()
    for sys in systems:
        if isinstance(sys, DiagramSystem):
            _inline_diagram(diagram, sys, output_collision="unique")
        else:
            _add_system_to_diagram(diagram, sys)
    if systems:
        _carry_left_operand(diagram, systems[0])
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
        and left unchanged, its name, ``x0`` and camera settings carried.
    right : System or DiagramSystem
        System or diagram to connect after ``left``.
    *rest : System or DiagramSystem
        Additional systems or diagrams to append to the series chain.

    Returns
    -------
    DiagramSystem
        A new diagram with each stage connected output-to-input.
    """
    diagram = _as_composition_diagram(left, expose_entry=True)
    _series_pair(diagram, right)
    for next_sys in rest:
        _series_pair(diagram, next_sys)
    _carry_left_operand(diagram, left)
    return diagram


def feedback(sys, filter=1, *, of=None, sign=-1.0, validate=True) -> DiagramSystem:
    """Close ``sys`` on itself with an :class:`~minilink.blocks.routing.Error`
    block: ``r -> e = r - y -> sys -> y``, the textbook ``T = L / (1 + L H)``.

    Negative feedback (the default) inserts ``Error``, so the diagram shows
    ports ``+``, ``-``, and ``e``. Positive feedback still uses a signed
    :class:`~minilink.blocks.routing.Sum`. The junction drives the command
    input of ``sys`` (:func:`~minilink.core.feedback.command_port`: ``u``,
    else its one free input) — a plant, a transfer function, a compensator,
    or a series diagram such as ``C >> G``; a block that reads its own
    measurement (``y`` or ``x``) is closed on its plant with ``@`` instead.
    What comes back is ``of``, else ``y``, else the only output. ``sys`` is
    not modified: the result is a new diagram sharing its blocks, with
    boundary input ``r`` and that output as boundary output ``y``. ``@``
    always means controller at plant; this is the one way to close an open
    loop on itself.

    Parameters
    ----------
    filter : 1, scalar, array, or System
        The return path ``H``: ``1`` is a direct wire, a scalar or matrix
        inserts a ``Gain`` block, a system (a sensor, a low-pass) is inserted
        as a block; the block is named ``filter`` either way.
    of : str or (str, int), optional
        Which output comes back: a boundary output port id, or
        ``(port, index)`` for one component. Default: the whole output when
        its dimension matches the input; a scalar input against a vector
        output takes component 0 through a visible ``Demux``; any other
        mismatch is an error.
    sign : float
        ``-1.0`` for negative feedback (default), ``+1.0`` for positive.
    validate : bool
        Run algebraic-loop detection before returning.
    """
    diagram, _junction = _close_with_junction(
        sys, filter, of=of, sign=sign, validate=validate
    )
    return diagram


def _close_with_junction(sys, filter=1, *, of=None, sign=-1.0, validate=True):
    """The loop of :func:`feedback` and the id of its junction block."""
    from minilink.blocks.routing import Demux, Error, Gain, Sum

    refuse_two_port_entry(sys)
    entry = _command_entry(sys)
    diagram = DiagramSystem()
    if isinstance(sys, DiagramSystem):
        _inline_diagram(diagram, sys, output_collision="replace")
        targets = _boundary_input_targets(diagram, entry)
        # The free boundary input that fed the entry goes away: the junction feeds it.
        del diagram.inputs[entry]
        for target_sys, target_port in targets:
            diagram.connections[target_sys][target_port] = None
    else:
        entry_sys = _add_system_to_diagram(diagram, sys)
        targets = [(entry_sys, entry)]
        diagram.connect_new_output_port(entry_sys, _default_output_port(sys), "y")
    entry_dim = diagram.subsystems[targets[0][0]].inputs[targets[0][1]].dim

    signal_sys, signal_port = _return_signal(diagram, of, entry_dim, Demux)

    if isinstance(filter, System):
        filter_id = _add_system_to_diagram(diagram, filter, role="filter")
        diagram.connect(signal_sys, signal_port, filter_id, _default_input_port(filter))
        signal_sys, signal_port = filter_id, _default_output_port(filter)
    elif not (np.isscalar(filter) and float(filter) == 1.0):
        filter_id = _add_system_to_diagram(
            diagram, Gain(filter, dim=entry_dim), role="filter"
        )
        diagram.connect(signal_sys, signal_port, filter_id, "u")
        signal_sys, signal_port = filter_id, "y"

    if float(sign) == -1.0:
        junction_id = _add_system_to_diagram(
            diagram, Error(dim=entry_dim), role="error"
        )
        plus_port, minus_port, out_port = "+", "-", "e"
    else:
        junction_id = _add_system_to_diagram(
            diagram, Sum(signs=(1.0, float(sign)), dim=entry_dim), role="sum"
        )
        plus_port, minus_port, out_port = "in0", "in1", "y"
    diagram.add_input_port("r", dim=entry_dim)
    diagram.connect("input", "r", junction_id, plus_port)
    diagram.connect(signal_sys, signal_port, junction_id, minus_port)
    for target_sys, target_port in targets:
        diagram.connect(junction_id, out_port, target_sys, target_port)
    diagram.name = f"Closed loop of {sys.name}"
    if validate:
        diagram.check_algebraic_loops()
    return diagram, junction_id


def _command_entry(sys) -> str:
    """The input the junction drives: the command port of a leaf or of a diagram's edge."""
    try:
        return command_port(sys)
    except ValueError as exc:
        raise ValueError(
            f"feedback() closes {sys.name!r} on its command input: 'u', else its one "
            f"free input; it has {list(sys.inputs)}. Close a controller on its plant "
            "with controller @ plant, or wire the loop with DiagramSystem.connect."
        ) from exc


def refuse_unused_keywords(port_keywords):
    """Refuse port and feedback keywords that an Error-junction loop does not read.

    ``port_keywords`` maps each keyword to ``(value, default)``.
    """
    changed = [
        name for name, (value, default) in port_keywords.items() if value != default
    ]
    if changed:
        raise ValueError(
            f"{', '.join(changed)} not honoured by the Error-block loop: an "
            "error-driven block closes on its one input and returns y. Drop them, "
            "or wire the loop with DiagramSystem.connect."
        )


def refuse_two_port_entry(block):
    """Refuse to drive ``block`` from an Error junction when it reads its own measurement.

    A block with a reference and a measurement port (``r`` and ``y``, or ``r``
    and ``x``) is not driven by an error: a junction on its first input would
    leave the measurement unconnected and feed ``e = r - y`` into ``r``.
    """
    roles = block_roles(block)
    if not roles.measurements:
        return
    raise ValueError(
        f"{block.name} reads its measurement on {roles.measurement!r}"
        + (" (on its edge)" if isinstance(block, DiagramSystem) else "")
        + (f" and its reference on {roles.reference!r}" if roles.reference else "")
        + ": an Error block cannot drive it. Close it on the plant directly "
        "(controller @ plant), use its error layout (ports='error') if it has one, "
        "or wire the loop with DiagramSystem.connect."
    )


def _return_signal(diagram, of, entry_dim, Demux):
    """Source ``(sys_id, port_id)`` of what comes back to the junction."""
    port_id, index = of if isinstance(of, tuple) else (of, None)
    if port_id is None:
        if "y" in diagram.outputs or len(diagram.outputs) == 1:
            port_id = "y" if "y" in diagram.outputs else next(iter(diagram.outputs))
        else:
            raise ValueError(
                "Cannot choose what comes back: no output 'y' among "
                f"{', '.join(diagram.outputs)}; pass of=(port, index)"
            )
    if port_id not in diagram.outputs:
        raise ValueError(
            f"Unknown output {port_id!r}; boundary outputs: {', '.join(diagram.outputs)}"
        )
    out_sys, out_port = diagram.connections["output"][port_id]
    dim = diagram.subsystems[out_sys].outputs[out_port].dim
    if index is None:
        if dim == entry_dim:
            return out_sys, out_port
        if entry_dim == 1 and dim > 1:
            index = 0  # the standard choice: the first component (the position)
        else:
            raise ValueError(
                f"Cannot close the loop: the input takes {entry_dim} values and "
                f"the output {port_id!r} has {dim}; pass of=({port_id!r}, index) "
                "or wire a Demux / Mux explicitly."
            )
    index = int(index)
    if not 0 <= index < dim:
        raise ValueError(f"of index must be in [0, {dim - 1}] for {port_id!r}")
    if entry_dim != 1:
        raise ValueError("of=(port, index) selects one component; the input takes more")
    parts = [d for d in (index, 1, dim - index - 1) if d > 0]
    demux_id = _add_system_to_diagram(
        diagram, Demux(dims=parts, port=out_port), role="demux"
    )
    diagram.connect(out_sys, out_port, demux_id, out_port)
    return demux_id, f"{out_port}[{index}]"


def closed_loop(
    controller: System,
    plant: System,
    *,
    r: bool = True,
    w: bool = False,
    v: bool = False,
    ref_port: str = "r",
    measurement_port: str = "y",
    control_port: str = "u",
    plant_input_port: str = "u",
    plant_output_port: str = "y",
    output_port: str = "y",
    feedback: str = "auto",
    validate: bool = True,
) -> DiagramSystem:
    """Build a controller/plant feedback diagram — the ``controller @ plant`` operator.

    Two layouts, one rule "A drives B and B's output returns to A":

    - a controller with a measurement port (``r``, ``y`` → ``u``) is wired
      the standard way, the measurement returning to ``y``;
    - an error-driven block (:func:`~minilink.core.feedback.block_roles`: a
      compensator, a transfer function, a plant, a series diagram) gets an
      :class:`~minilink.blocks.routing.Error` block ``e = r - y`` inserted by
      :func:`feedback`.

    ``@`` needs a plant on the right: :func:`feedback` closes an open loop on
    itself (``feedback(L)``, ``feedback(L, K)``).

    Neither operand is modified: the result is a new diagram sharing their
    blocks, also when ``controller`` is a diagram such as ``C >> S``.

    Parameters
    ----------
    controller : System
        Controller subsystem with reference, measurement, and control ports.
    plant : System
        Plant subsystem with control input and measurement output ports.
    r, w, v : bool, optional
        The loop's boundary inputs: the reference ``r`` (default on), the load
        disturbance ``w`` summed into the plant input, and the measurement
        noise ``v`` summed into the measurement before it returns (both off by
        default). With ``r=False`` the reference reads its nominal value.
    ref_port, measurement_port, control_port, plant_input_port, plant_output_port : str
        Port names used for the standard feedback wiring.
    feedback : str, optional
        How to wire the controller measurement input: ``"auto"`` (default) tries
        ``plant.y``, then Mux ``(q, dq)``, then ``plant.x``; ``"y"`` forces
        ``plant.y``; ``"qdq"`` inserts ``Mux(q, dq) → controller.y``.
    output_port : str, optional
        Diagram boundary output name.
    validate : bool, optional
        If ``True``, run algebraic-loop detection before returning.

    Returns
    -------
    DiagramSystem
        A diagram exposing the controller reference as input and plant output as
        output.
    """
    if feedback not in _VALID_FEEDBACK:
        raise ValueError(
            f"feedback must be one of {sorted(_VALID_FEEDBACK)!r}, got {feedback!r}"
        )
    port_keywords = {
        "ref_port": (ref_port, "r"),
        "measurement_port": (measurement_port, "y"),
        "control_port": (control_port, "u"),
        "plant_input_port": (plant_input_port, "u"),
        "plant_output_port": (plant_output_port, "y"),
        "output_port": (output_port, "y"),
        "feedback": (feedback, "auto"),
    }
    if not isinstance(plant, System):
        raise TypeError(
            "@ needs a plant on the right; to close an open loop on itself use "
            "feedback(L) or feedback(L, K)"
        )
    if block_roles(controller).error_driven:
        refuse_unused_keywords(port_keywords)
        loop_gain = series(controller, plant)
        diagram, junction_id = _close_with_junction(loop_gain, validate=False)
        add_loop_inputs(diagram, plant, None, (junction_id, "-"), r=r, w=w, v=v)
        if validate:
            diagram.check_algebraic_loops()
        return diagram
    if isinstance(controller, DiagramSystem):
        roles = block_roles(controller)
        raise ValueError(
            f"{controller.name} is a diagram that reads its measurement on "
            f"{roles.measurement!r} on its edge: close one block on the plant "
            "(controller @ plant), or wire the loop with DiagramSystem.connect."
        )

    diagram = DiagramSystem()
    controller_id = _add_system_to_diagram(diagram, controller, role="ctl")
    plant_id = _add_system_to_diagram(diagram, plant, role="sys")

    using_defaults = (
        measurement_port == "y"
        and plant_output_port == "y"
        and control_port == "u"
        and plant_input_port == "u"
    )
    if feedback in ("auto", "qdq", "y") and using_defaults:
        wiring = resolve_standard_feedback(
            controller,
            plant,
            diagram,
            feedback=feedback,
            controller_id=controller_id,
            plant_id=plant_id,
        )
    else:
        wiring = resolve_standard_feedback(
            controller,
            plant,
            diagram,
            feedback=feedback,
            measurement_in=measurement_port,
            plant_out=plant_output_port,
            control_out=control_port,
            plant_in=plant_input_port,
            controller_id=controller_id,
            plant_id=plant_id,
        )
    measurement_port = wiring.measurement_in
    plant_output_port = wiring.plant_out
    control_port = wiring.control_out
    plant_input_port = wiring.plant_in

    _require_input(
        controller,
        measurement_port,
        f"controller measurement port {measurement_port!r}",
    )
    _require_output(
        controller, control_port, f"controller output port {control_port!r}"
    )
    _require_input(plant, plant_input_port, f"plant input port {plant_input_port!r}")
    _require_output(
        plant, plant_output_port, f"plant output port {plant_output_port!r}"
    )

    _require_same_dim(
        controller.outputs[control_port].dim,
        plant.inputs[plant_input_port].dim,
        f"{controller_id}:{control_port}",
        f"{plant_id}:{plant_input_port}",
    )
    if wiring.mux_id is None:
        _require_same_dim(
            plant.outputs[plant_output_port].dim,
            controller.inputs[measurement_port].dim,
            f"{plant_id}:{plant_output_port}",
            f"{controller_id}:{measurement_port}",
        )

    if r and ref_port in controller.inputs:
        ref = controller.inputs[ref_port]
        _add_input_port_like(diagram, ref_port, ref)
        diagram.connect("input", ref_port, controller_id, ref_port)

    if wiring.mux_id is not None:
        diagram.connect(plant_id, "q", wiring.mux_id, "in0")
        diagram.connect(plant_id, "dq", wiring.mux_id, "in1")
        diagram.connect(wiring.mux_id, "y", controller_id, measurement_port)
    else:
        diagram.connect(plant_id, plant_output_port, controller_id, measurement_port)
    diagram.connect(controller_id, control_port, plant_id, plant_input_port)
    diagram.connect_new_output_port(plant_id, plant_output_port, output_port)

    diagram.name = f"Closed loop {plant.name} with {controller.name}"

    _propagate_animation_camera(diagram, plant)

    add_loop_inputs(
        diagram,
        plant,
        plant_input_port,
        (controller_id, measurement_port),
        r=r,
        w=w,
        v=v,
    )
    if validate:
        diagram.check_algebraic_loops()
    return diagram


def closed_loop_qdq(
    controller: System,
    plant: System,
    **kwargs,
) -> DiagramSystem:
    """Build a closed loop with ``Mux(plant.q, plant.dq) → controller.y``."""
    return closed_loop(controller, plant, feedback="qdq", **kwargs)


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
    if not isinstance(diagram, DiagramSystem):
        raise TypeError("autowire() expects a DiagramSystem")

    decisions = []
    open_inputs = []
    for target_id, target in diagram.subsystems.items():
        for input_id, input_port in target.inputs.items():
            if diagram.connections[target_id][input_id] is not None:
                continue
            candidates = [
                (source_id, port_id)
                for source_id, _source, port_id, output in _iter_output_ports(
                    diagram, target_id
                )
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

    if open_inputs and strict:
        raise ValueError(open_inputs[0])
    if open_inputs:
        warnings.warn(
            "autowire left these inputs unconnected (wire them with connect): "
            + "; ".join(open_inputs),
            stacklevel=3,
        )

    for source_id, source_port, target_id, input_id in decisions:
        diagram.connect(source_id, source_port, target_id, input_id)

    if validate:
        diagram.check_algebraic_loops()
    return diagram


# Internal machinery
#
# Everything below implements the shortcut bookkeeping: choosing default
# ports, remembering entry/output, flattening diagram operands, and the
# conservative autowire matching rules.


def _as_composition_diagram(sys, *, expose_entry: bool) -> DiagramSystem:
    """A diagram operand as a copy; a leaf as a diagram with its inputs on the edge."""
    if isinstance(sys, DiagramSystem):
        return _copy_diagram(sys)

    diagram = DiagramSystem()
    sys_id = _add_system_to_diagram(diagram, sys)
    if expose_entry:
        _expose_free_inputs(diagram, sys_id)
    return diagram


def _copy_diagram(diagram):
    """A new diagram with the same blocks, ids, wiring, boundary ports, name, ``x0`` and camera.

    The blocks are shared, not copied; the operators build on the copy and leave
    the user's diagram untouched.
    """
    copy = DiagramSystem()
    _inline_diagram(copy, diagram, output_collision="replace")
    _carry_left_operand(copy, diagram)
    return copy


def _new_diagram_like(sys):
    """A diagram operand as a new diagram with the same ids and wiring; a leaf as is."""
    return _copy_diagram(sys) if isinstance(sys, DiagramSystem) else sys


def _carry_left_operand(diagram, left) -> None:
    """Carry a diagram left operand's name, ``x0`` and camera settings into ``diagram``.

    The left operand's blocks come first in the result, so its ``x0`` is the
    head of the stacked state; a leaf left operand carries nothing.
    """
    if not isinstance(left, DiagramSystem):
        return
    diagram.name = left.name
    diagram.x0 = np.concatenate(
        [np.asarray(left.x0, dtype=float), diagram.x0[left.n :]]
    )
    diagram.camera_scale = left.camera_scale
    diagram.camera_target = np.array(left.camera_target, dtype=float).copy()
    diagram.camera_plot_axes = left.camera_plot_axes
    diagram.camera_follow_frame = left.camera_follow_frame
    diagram.camera_priority = left.camera_priority
    diagram.scene_grid = left.scene_grid


def _series_pair(diagram, right):
    """Append ``right`` to ``diagram`` (the chain so far) by the ``>>`` rule."""
    outputs = _edge_outputs(diagram)
    inputs = right.inputs if not isinstance(right, DiagramSystem) else right.inputs
    source_name, target_name = _series_connection(diagram, outputs, inputs)
    source_id, source_port = outputs[source_name]
    _require_same_dim(
        diagram.subsystems[source_id].outputs[source_port].dim,
        inputs[target_name].dim,
        f"{source_id}:{source_port}",
        f"{right.name}:{target_name}",
    )
    # The chain's outputs are consumed; the result's output is the right side's.
    for output_id in list(diagram.outputs):
        del diagram.outputs[output_id]
        diagram.connections["output"].pop(output_id, None)

    if isinstance(right, DiagramSystem):
        _inline_diagram(
            diagram,
            right,
            input_bindings={target_name: (source_id, source_port)},
            output_collision="replace",
        )
        return diagram

    right_id = _add_system_to_diagram(diagram, right)
    diagram.connect(source_id, source_port, right_id, target_name)
    diagram.connect_new_output_port(right_id, _default_output_port(right), "y")
    _propagate_animation_camera(diagram, right)
    return diagram


def _edge_outputs(diagram) -> dict:
    """The chain's outputs by name: its boundary outputs, or the ports of its one block."""
    if diagram.outputs:
        # A box's own state (``expose("x")``) has no source block; it is not a chain output.
        return {
            output_id: tuple(source)
            for output_id in diagram.outputs
            if (source := diagram.connections.get("output", {}).get(output_id))
            is not None
        }
    if len(diagram.subsystems) == 1:
        ((sys_id, sys),) = diagram.subsystems.items()
        return {port_id: (sys_id, port_id) for port_id in sys.outputs}
    raise ValueError("Left side of '>>' has no output port to connect")


def _series_connection(diagram, outputs, inputs) -> tuple[str, str]:
    """The ``(output, input)`` names ``>>`` connects, by the first rule that applies."""
    if not outputs:
        raise ValueError("Left side of '>>' has no output port to connect")
    if not inputs:
        raise ValueError("Right side of '>>' has no free input port to connect")
    if len(outputs) == 1 and len(inputs) == 1:
        return next(iter(outputs)), next(iter(inputs))
    shared = [name for name in outputs if name in inputs]
    if shared == ["y"] and "r" in inputs and not diagram.inputs:
        raise ValueError(
            "a source into a block that reads 'r' and 'y' would drive the measurement "
            "'y'; name the source's signal after the input it drives (port='r') for "
            "the block alone, or feed the loop: source >> (controller @ plant)"
        )
    if len(shared) == 1:
        return shared[0], shared[0]
    if "y" in outputs and "u" in inputs:
        return "y", "u"
    raise ValueError(
        "'>>' connects one output to one input; wire this one with connect "
        f"(outputs: {', '.join(outputs)}; free inputs: {', '.join(inputs)})"
    )


def _inline_diagram(
    diagram,
    source,
    *,
    input_bindings=None,
    output_collision: str,
) -> dict[str, str]:
    """Flatten ``source`` into ``diagram``, remapping ids; return the id map."""
    input_bindings = {} if input_bindings is None else dict(input_bindings)
    source_subsystems = list(source.subsystems.items())
    source_inputs = list(source.inputs.items())
    source_connections = {
        source_id: dict(source.connections[source_id])
        for source_id, _subsystem in source_subsystems
    }
    source_output_connections = dict(source.connections.get("output", {}))
    source_outputs = dict(source.outputs)

    id_map = {}
    for source_id, subsystem in source_subsystems:
        check_reserved_id(source_id, diagram.subsystems)
        target_id = _unique_id(source_id, diagram.subsystems)
        diagram.add_subsystem(subsystem, target_id)
        id_map[source_id] = target_id

    input_port_map = {}
    for port_id, port in source_inputs:
        if port_id in input_bindings:
            continue
        target_port_id = _unique_id(port_id, diagram.inputs)
        _add_input_port_like(diagram, target_port_id, port)
        input_port_map[port_id] = target_port_id

    for source_target_id, subsystem in source_subsystems:
        target_id = id_map[source_target_id]
        for input_id in subsystem.inputs:
            edge = source_connections[source_target_id][input_id]
            if edge is None:
                continue
            source_sys_id, source_port_id = edge
            if source_sys_id == "input":
                if source_port_id in input_bindings:
                    bound_sys_id, bound_port_id = input_bindings[source_port_id]
                    diagram.connect(bound_sys_id, bound_port_id, target_id, input_id)
                else:
                    diagram.connect(
                        "input",
                        input_port_map[source_port_id],
                        target_id,
                        input_id,
                    )
            else:
                diagram.connect(
                    id_map[source_sys_id],
                    source_port_id,
                    target_id,
                    input_id,
                )

    for output_id, edge in source_output_connections.items():
        if edge is None:
            continue
        source_sys_id, source_port_id = edge
        if source_sys_id == "input":
            raise TypeError(
                "Flattening diagram outputs sourced directly from external "
                "inputs is not supported by composition shortcuts."
            )
        target_output_id = _output_port_id(diagram, output_id, output_collision)
        _connect_new_output_port_like(
            diagram,
            id_map[source_sys_id],
            source_port_id,
            target_output_id,
            source_outputs[output_id],
        )

    _propagate_animation_camera(diagram, source)

    return id_map


def _add_system_to_diagram(diagram, sys, *, role=None) -> str:
    if isinstance(sys, DiagramSystem):
        raise TypeError(
            "Nested DiagramSystem operands are not supported by this shortcut. "
            "Use explicit DiagramSystem wiring for nested diagrams."
        )
    if not isinstance(sys, System):
        raise TypeError(f"Expected a System, got {type(sys).__name__}")

    sys_id = _default_subsystem_id(sys, role=role)
    check_reserved_id(sys_id, diagram.subsystems)
    sys_id = _unique_id(sys_id, diagram.subsystems)
    diagram.add_subsystem(sys, sys_id)
    return sys_id


def add_loop_inputs(diagram, plant, plant_port, measurement, *, r, w, v):
    """Drop the boundary ``r`` unless ``r``; sum ``w`` into the plant input and ``v`` into the measurement."""
    from minilink.blocks.routing import Sum

    if not r and "r" in diagram.inputs:
        for sys_id, ports in diagram.connections.items():
            for port_id, source in ports.items():
                if source == ("input", "r"):
                    ports[port_id] = None
        del diagram.inputs["r"]
    if w:
        plant_id = loop_block_id(diagram, plant, "plant")
        port = _default_input_port(plant) if plant_port is None else plant_port
        sum_into(diagram, (plant_id, port), "w", "disturbance", Sum)
    if v:
        sum_into(diagram, measurement, "v", "noise", Sum)
    return diagram


def sum_into(diagram, target, port_id, role, Sum):
    """Rewire ``target`` to read its source plus a new boundary input ``port_id``."""
    sys_id, in_port = target
    source = diagram.connections[sys_id][in_port]
    dim = diagram.subsystems[sys_id].inputs[in_port].dim
    sum_id = _add_system_to_diagram(diagram, Sum(signs=(1.0, 1.0), dim=dim), role=role)
    diagram.add_input_port(port_id, dim=dim)
    diagram.connect(*source, sum_id, "in0")
    diagram.connect("input", port_id, sum_id, "in1")
    diagram.connect(sum_id, "y", sys_id, in_port)


def loop_block_id(diagram, sys, role):
    """The id of ``sys`` among the diagram's blocks; a nested diagram is refused."""
    for sys_id, block in diagram.subsystems.items():
        if block is sys:
            return sys_id
    raise ValueError(
        f"w= needs the {role} as one block; {sys.name!r} is a diagram. Wire the "
        "disturbance with DiagramSystem.connect and a Sum block instead."
    )


def _add_input_port_like(diagram, port_id: str, port) -> None:
    diagram.add_input_port(
        port_id,
        dim=port.dim,
        nominal_value=port.nominal_value,
        labels=port.labels,
        units=port.units,
        lower_bound=port.lower_bound,
        upper_bound=port.upper_bound,
    )


def _connect_new_output_port_like(
    diagram,
    source_sys_id: str,
    source_port_id: str,
    output_port_id: str,
    output_port,
) -> None:
    diagram.connect_new_output_port(
        source_sys_id,
        source_port_id,
        output_port_id,
        nominal_value=output_port.nominal_value,
        labels=output_port.labels,
        units=output_port.units,
        lower_bound=output_port.lower_bound,
        upper_bound=output_port.upper_bound,
    )


def _output_port_id(diagram, output_id: str, collision: str) -> str:
    if collision == "replace":
        return output_id
    if collision == "unique":
        return _unique_id(output_id, diagram.outputs)
    raise ValueError(f"Unknown output collision policy {collision!r}")


def _boundary_input_targets(diagram, boundary_port_id: str) -> list[tuple[str, str]]:
    targets = []
    for sys_id, subsystem in diagram.subsystems.items():
        for port_id in subsystem.inputs:
            if diagram.connections[sys_id][port_id] == ("input", boundary_port_id):
                targets.append((sys_id, port_id))
    return targets


def _expose_free_inputs(diagram, sys_id: str) -> None:
    """Put every input of a leaf on the diagram's edge, under the same name."""
    sys = diagram.subsystems[sys_id]
    for port_id, port in sys.inputs.items():
        _add_input_port_like(diagram, port_id, port)
        diagram.connect("input", port_id, sys_id, port_id)


def _default_output_port(sys) -> str:
    if "y" in sys.outputs:
        return "y"
    if len(sys.outputs) == 1:
        return next(iter(sys.outputs))
    ports = ", ".join(sys.outputs)
    raise ValueError(
        f"Cannot choose a default output port for {sys.name!r}; candidates: {ports}"
    )


def _default_input_port(sys) -> str:
    for preferred in ("u", "r"):
        if preferred in sys.inputs:
            return preferred
    if len(sys.inputs) == 1:
        return next(iter(sys.inputs))
    ports = ", ".join(sys.inputs)
    raise ValueError(
        f"Cannot choose a default input port for {sys.name!r}; candidates: {ports}"
    )


def _require_input(sys, port_id: str, label: str) -> None:
    if port_id not in sys.inputs:
        ports = ", ".join(sys.inputs)
        raise ValueError(f"Missing {label}; available input ports: {ports}")


def _require_output(sys, port_id: str, label: str) -> None:
    if port_id not in sys.outputs:
        ports = ", ".join(sys.outputs)
        raise ValueError(f"Missing {label}; available output ports: {ports}")


def _require_same_dim(left_dim: int, right_dim: int, left_label: str, right_label: str):
    if left_dim != right_dim:
        raise ValueError(
            f"Port dimension mismatch: {left_label} has dim {left_dim}, "
            f"{right_label} has dim {right_dim}"
        )


@dataclass(frozen=True)
class StandardFeedbackWiring:
    """Resolved port names for controller/computer ↔ plant feedback."""

    control_out: str
    measurement_in: str
    plant_in: str
    plant_out: str
    mux_id: str | None = None


def resolve_standard_feedback(
    controller,
    plant,
    diagram: DiagramSystem | None = None,
    *,
    feedback: str = "auto",
    measurement_in: str | None = None,
    plant_out: str | None = None,
    control_out: str | None = None,
    plant_in: str | None = None,
    controller_id: str = "ctl",
    plant_id: str = "plant",
) -> StandardFeedbackWiring:
    """
    Choose standard feedback port names shared by ``closed_loop`` and hybrid wiring.

    The controller's port names decide (:func:`minilink.core.feedback.block_roles`):
    a block that reads ``y`` closes on the plant's ``y``, one that reads ``x`` on
    its ``x``; the dimensions are a check, with a ``Mux(q, dq)`` for a mechanical
    plant whose measurement is ``[q; dq]``.

    Unset ``control_out`` / ``plant_in`` resolve to ``u`` (or ``u_ff`` / single port).
    Unset ``measurement_in`` / ``plant_out`` use ``feedback`` auto rules (``y``, ``qdq``, ``x``).
    """
    if feedback not in _VALID_FEEDBACK:
        raise ValueError(
            f"feedback must be one of {sorted(_VALID_FEEDBACK)!r}, got {feedback!r}"
        )

    resolved_control = control_out or _default_control_out(controller)
    resolved_plant_in = plant_in or _default_plant_in(plant)

    if measurement_in is not None and plant_out is not None:
        return StandardFeedbackWiring(
            resolved_control,
            measurement_in,
            resolved_plant_in,
            plant_out,
        )

    path = _resolve_feedback_path(
        controller,
        plant,
        diagram,
        feedback=feedback,
        plant_output_port=plant_out or "y",
        controller_id=controller_id,
        plant_id=plant_id,
    )
    return StandardFeedbackWiring(
        resolved_control,
        path.measurement_in,
        resolved_plant_in,
        path.plant_out,
        path.mux_id,
    )


@dataclass(frozen=True)
class _FeedbackPath:
    measurement_in: str
    plant_out: str
    mux_id: str | None = None


def _resolve_feedback_path(
    controller,
    plant,
    diagram: DiagramSystem | None,
    *,
    feedback: str,
    plant_output_port: str,
    controller_id: str,
    plant_id: str,
) -> _FeedbackPath:
    if feedback == "qdq":
        return _feedback_path_qdq(
            controller, plant, diagram, plant_output_port=plant_output_port
        )

    if feedback == "y":
        if not _y_feedback_available(controller, plant):
            raise ValueError(
                _feedback_mismatch_message(
                    controller,
                    plant,
                    controller_id,
                    plant_id,
                    attempted="y",
                )
            )
        return _FeedbackPath("y", "y")

    # auto: the controller's port names decide; a block that reads both ``y``
    # and ``x`` closes on the first the plant can serve.
    roles = block_roles(controller)
    if roles.measurements:
        return _feedback_path_from_declaration(
            controller,
            plant,
            diagram,
            measurements=roles.measurements,
            plant_output_port=plant_output_port,
            controller_id=controller_id,
            plant_id=plant_id,
        )

    raise ValueError(
        _feedback_mismatch_message(
            controller,
            plant,
            controller_id,
            plant_id,
            attempted="auto",
        )
    )


def _feedback_path_from_declaration(
    controller,
    plant,
    diagram,
    *,
    measurements: tuple,
    plant_output_port: str,
    controller_id: str,
    plant_id: str,
) -> _FeedbackPath:
    """Wire the first measurement the plant can serve to the matching plant output.

    A measurement ``x`` reads the plant state output; ``y`` reads the plant
    ``y``, inserting a ``Mux(q, dq)`` when its dimension is twice the plant
    dof. With several measurements the first wireable one wins, and the
    first one's mismatch is reported when none is.
    """
    first_error = None
    for measurement in measurements:
        try:
            return _feedback_path_for_measurement(
                controller,
                plant,
                diagram,
                measurement=measurement,
                plant_output_port=plant_output_port,
                controller_id=controller_id,
                plant_id=plant_id,
            )
        except ValueError as exc:
            first_error = first_error or exc
    raise first_error


def _feedback_path_for_measurement(
    controller,
    plant,
    diagram,
    *,
    measurement: str,
    plant_output_port: str,
    controller_id: str,
    plant_id: str,
) -> _FeedbackPath:
    dim = controller.inputs[measurement].dim

    if measurement == "x":
        if "x" in plant.outputs and plant.outputs["x"].dim == dim:
            return _FeedbackPath("x", "x")
        plant_x = plant.outputs["x"].dim if "x" in plant.outputs else None
        raise ValueError(
            f"Cannot wire closed-loop feedback: {controller_id} declares "
            f"measurement 'x' dim {dim}, but {plant_id} 'x' output has dim "
            f"{plant_x}"
        )

    if "y" in plant.outputs and plant.outputs["y"].dim == dim:
        return _FeedbackPath(measurement, "y")

    qdq_n = _plant_qdq_dof(plant)
    if diagram is not None and qdq_n is not None and dim == 2 * qdq_n:
        mux_id = _add_qdq_mux(diagram, qdq_n)
        return _FeedbackPath(measurement, plant_output_port, mux_id)

    if "y" not in plant.outputs:
        raise ValueError(
            f"Cannot wire closed-loop feedback: plant '{plant.name}' ({plant_id}) "
            "has no 'y' output port; declare it with output_dim=... (the standard "
            "port, y = x by default) or wire ports explicitly with "
            "add_subsystem/connect"
        )
    plant_y = plant.outputs["y"].dim
    hint = ""
    if qdq_n is not None:
        hint = (
            f"; plant exposes q/dq dof {qdq_n} — the measurement must be "
            f"dim {2 * qdq_n} for Mux(q, dq) wiring (or pass feedback='qdq')"
        )
    raise ValueError(
        f"Cannot wire closed-loop feedback: {controller_id} declares "
        f"measurement {measurement!r} dim {dim}, but {plant_id} 'y' output "
        f"has dim {plant_y}{hint}"
    )


def _feedback_path_qdq(controller, plant, diagram, *, plant_output_port: str):
    if "y" not in controller.inputs:
        raise ValueError(
            "feedback='qdq' requires controller measurement port 'y'; "
            f"available inputs: {', '.join(controller.inputs)}"
        )
    qdq_n = _plant_qdq_dof(plant)
    if qdq_n is None:
        raise ValueError(
            "feedback='qdq' requires plant outputs 'q' and 'dq' with equal dim"
        )
    if controller.inputs["y"].dim != 2 * qdq_n:
        raise ValueError(
            f"feedback='qdq' expects controller.y dim {2 * qdq_n} "
            f"(2 * plant dof), got {controller.inputs['y'].dim}"
        )
    if diagram is None:
        raise ValueError("feedback='qdq' requires a diagram for Mux insertion")
    mux_id = _add_qdq_mux(diagram, qdq_n)
    return _FeedbackPath("y", plant_output_port, mux_id)


def _default_control_out(sys) -> str:
    return block_roles(sys).command or _default_output_port(sys)


def _default_plant_in(sys) -> str:
    return _default_input_port(sys)


def default_computer_boundary_ports(controller) -> tuple[str, str]:
    """Return ``(measurement_in, control_out)`` for step-diagram boundary expose."""
    roles = block_roles(controller)
    measurement_in = roles.measurement or roles.entry or _default_input_port(controller)
    return measurement_in, _default_control_out(controller)


def _add_qdq_mux(diagram: DiagramSystem, dof: int) -> str:
    from minilink.blocks.routing import Mux

    return _add_system_to_diagram(diagram, Mux((dof, dof)))


def _plant_qdq_dof(plant) -> int | None:
    if "q" not in plant.outputs or "dq" not in plant.outputs:
        return None
    n = plant.outputs["q"].dim
    if plant.outputs["dq"].dim != n:
        return None
    return n


def _y_feedback_available(controller, plant) -> bool:
    return (
        "y" in controller.inputs
        and "y" in plant.outputs
        and controller.inputs["y"].dim == plant.outputs["y"].dim
    )


def _feedback_mismatch_message(
    controller,
    plant,
    controller_id: str,
    plant_id: str,
    *,
    attempted: str,
) -> str:
    if "y" not in plant.outputs and "y" in controller.inputs:
        return (
            f"Cannot wire closed-loop feedback: plant '{plant.name}' ({plant_id}) "
            "has no 'y' output port; declare it with output_dim=... (the standard "
            "port, y = x by default) or wire ports explicitly with "
            "add_subsystem/connect"
        )
    ctl_y = controller.inputs["y"].dim if "y" in controller.inputs else None
    plant_y = plant.outputs["y"].dim if "y" in plant.outputs else None
    qdq_n = _plant_qdq_dof(plant)
    profile = getattr(controller, "feedback_profile", None)
    hint = ""
    if attempted == "auto" and qdq_n is not None and ctl_y == 2 * qdq_n:
        hint = "; try feedback='qdq'"
    elif profile == "impedance" and qdq_n is not None:
        hint = "; impedance controller may need feedback='qdq'"
    elif profile is None:
        hint = (
            "; declare feedback_profile (or measurement_port / control_port) "
            "on the controller for declaration-driven wiring"
        )
    return (
        f"Cannot wire closed-loop feedback ({attempted!r}): "
        f"{controller_id}.y dim {ctl_y}, {plant_id}.y dim {plant_y}"
        f"{hint}"
    )


def _iter_output_ports(diagram, target_id: str):
    for source_id, source in diagram.subsystems.items():
        if source_id == target_id:
            continue
        for port_id, output in source.outputs.items():
            yield source_id, source, port_id, output


def _is_source_like(sys) -> bool:
    return sys.n == 0 and len(sys.inputs) == 0


def _is_controller_like(sys) -> bool:
    roles = block_roles(sys)
    return roles.command == "u" and (
        roles.reference is not None or bool(roles.measurements) or roles.entry == "e"
    )


def _default_subsystem_id(sys, *, role=None) -> str:
    if sys.id:
        return _normalize_identifier(str(sys.id))
    if role is not None:
        return _normalize_identifier(str(role))
    if _is_source_like(sys):
        return "ref"
    if _is_controller_like(sys):
        return "ctl"
    if sys.n > 0:
        return "sys"
    return _base_system_id(sys)


def _base_system_id(sys) -> str:
    raw = getattr(sys, "name", "") or sys.__class__.__name__
    if raw in {"System", "DynamicSystem"}:
        raw = sys.__class__.__name__
    return _normalize_identifier(raw)


def _normalize_identifier(value: str) -> str:
    value = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", str(value))
    value = re.sub(r"[^0-9A-Za-z]+", "_", value)
    value = value.strip("_").lower()
    if not value:
        value = "system"
    if value[0].isdigit():
        value = f"sys_{value}"
    return value


def _unique_id(base: str, existing) -> str:
    if base not in existing:
        return base
    i = 2
    while f"{base}{i}" in existing:
        i += 1
    return f"{base}{i}"
