"""Hybrid composition: a step computer and a continuous plant closed into one feedback loop."""

from __future__ import annotations

from typing import TYPE_CHECKING

from minilink.core.composition import (
    default_computer_boundary_ports,
    new_diagram_like,
    propagate_animation_camera,
    resolve_standard_feedback,
)
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.hybrid_diagram import BoundaryConnection, HybridDiagram
from minilink.core.system import StepSystem, System

if TYPE_CHECKING:
    from minilink.simulation.computer import Computer, StepSchedule

###############################################################################
# Public API
###############################################################################


def hybrid_closed_loop(
    computer_side: System | StepDiagramSystem,
    plant: System | DiagramSystem,
    *,
    schedule: StepSchedule | float,
    computer: Computer | None = None,
    computer_out: str = "u",
    plant_in: str = "u",
    plant_out: str = "y",
    computer_in: str = "y",
    ref_port: str = "r",
    output_port: str = "y",
    extra_boundaries: list[BoundaryConnection] | None = None,
) -> HybridDiagram:
    """
    Build a canonical computer ↔ plant feedback :class:`HybridDiagram`.

    ``Computer @ plant`` builds the same loop on the port names
    :func:`resolve_hybrid_feedback_ports` picks. The schedule attaches to the
    computer side before composition. The plant is not modified: when a plant
    diagram lacks a boundary port the loop needs, the loop gets a new diagram
    with the same blocks, ids, wiring and ``x0``, and the port is added there.

    Parameters
    ----------
    computer_side : System or StepDiagramSystem
        Step diagram or leaf controller wrapped in a step diagram.
    plant : System or DiagramSystem
        Continuous plant wrapped in a flow diagram when needed.
    schedule : StepSchedule or float
        Base tick schedule for the computer runtime. A leaf whose class defines
        its own ``%`` builds the runtime there (an MPC block checks ``schedule``
        against its ``dt_mpc``).
    computer : Computer, optional
        Reuse an existing compiled runtime (must match ``computer_side``).
    computer_out, plant_in, plant_out, computer_in : str
        Default SISO boundary port names.
    ref_port : str
        Computer diagram boundary input for the world reference.
    output_port : str
        Plant diagram boundary output exposed for logging (not wired by default).
    extra_boundaries : list of BoundaryConnection, optional
        Additional multi-channel boundary edges.
    """
    # Tools import core (DESIGN §3): the simulation band loads on first use
    from minilink.simulation.computer import Computer, StepSchedule, as_computer

    # A leaf whose class defines its own ``%`` builds its runtime there
    if computer is None and builds_own_computer(computer_side):
        # The block's own ``%`` may reset block state (an MPC block's dual-rate
        # hooks), so it runs once the plant and ports have passed on a plain wrap.
        hybrid_closed_loop(
            as_step_diagram(computer_side),
            plant,
            schedule=schedule,
            computer_out=computer_out,
            plant_in=plant_in,
            plant_out=plant_out,
            computer_in=computer_in,
            ref_port=ref_port,
            output_port=output_port,
            extra_boundaries=extra_boundaries,
        )
        computer = as_computer(computer_side, schedule)
        computer_side = computer.diagram
        schedule = computer.schedule

    # The two sides as diagrams, each with the boundary ports the loop wires
    step_diagram = as_step_diagram(computer_side)
    plant_diagram = as_plant_diagram(plant)
    ensure_computer_boundary_ports(
        step_diagram,
        sys_id=leaf_sys_id(computer_side, default="controller"),
        ref_port=ref_port,
        computer_in=computer_in,
        computer_out=computer_out,
    )
    plant_diagram = ensure_plant_boundary_ports(
        plant_diagram,
        sys_id=leaf_sys_id(plant, default="plant"),
        plant_in=plant_in,
        plant_out=plant_out,
        output_port=output_port,
        in_place=plant_diagram is not plant,
    )

    # The base tick schedule
    if isinstance(schedule, StepSchedule):
        step_schedule = schedule
    else:
        step_schedule = StepSchedule(dt_base=float(schedule))

    # The runtime: a new one, or the caller's, built on this diagram and schedule
    if computer is None:
        runtime = Computer(step_diagram, step_schedule)
    else:
        require_matching_computer(computer, step_diagram, step_schedule)
        runtime = computer

    # The loop: the command held on the plant (ZOH), the measurement sampled at
    # the ticks
    hybrid = HybridDiagram(computer=runtime, plant=plant_diagram)
    hybrid.connect_boundary(
        direction="computer_to_plant",
        computer_port=computer_out,
        plant_port=plant_in,
    )
    hybrid.connect_boundary(
        direction="plant_to_computer",
        computer_port=computer_in,
        plant_port=plant_out,
    )

    # The additional multi-channel edges
    if extra_boundaries:
        for conn in extra_boundaries:
            hybrid.connect_boundary(
                direction=conn.direction,
                computer_port=conn.computer_port,
                plant_port=conn.plant_port,
            )

    return hybrid


def refresh_plant_animation_camera(plant_diagram: DiagramSystem) -> None:
    """Sync plant diagram camera hints from its plant subsystem(s)."""
    if plant_diagram.subsystems:
        propagate_animation_camera(plant_diagram, *plant_diagram.subsystems.values())


def resolve_hybrid_feedback_ports(
    computer_side: System | StepDiagramSystem,
    plant: System | DiagramSystem,
) -> tuple[str, str, str, str]:
    """
    Return ``(computer_out, computer_in, plant_in, plant_out)`` for standard wiring.

    Uses the same rules as :func:`~minilink.core.composition.resolve_standard_feedback`.
    Multi-leaf step diagrams (e.g. dual-rate MPC) use boundary ports when present.
    """
    # A multi-leaf step diagram is wired through its own boundary ports
    if (
        isinstance(computer_side, StepDiagramSystem)
        and len(computer_side.subsystems) != 1
    ):
        computer_in = boundary_measurement_of(computer_side)
        computer_out = boundary_command_of(computer_side)
        plant_block = plant_leaf(plant)
        plant_in = "u" if "u" in plant_block.inputs else next(iter(plant_block.inputs))
        plant_out = (
            "y" if "y" in plant_block.outputs else next(iter(plant_block.outputs))
        )
        return computer_out, computer_in, plant_in, plant_out

    # A single leaf follows the rules of the continuous ``ctl @ plant``
    controller = computer_leaf(computer_side)
    plant_block = plant_leaf(plant)
    wiring = resolve_standard_feedback(controller, plant_block)

    return (
        wiring.control_out,
        wiring.measurement_in,
        wiring.plant_in,
        wiring.plant_out,
    )


def expose_computer_boundary_ports(diagram: StepDiagramSystem) -> None:
    """Expose standard measurement and control ports on a step diagram boundary."""
    if len(diagram.subsystems) != 1:
        return

    sys_id = next(iter(diagram.subsystems))
    controller = diagram.subsystems[sys_id]
    measurement_in, control_out = default_computer_boundary_ports(controller)

    # The reference, the measurement and the command
    ensure_computer_boundary_ports(
        diagram,
        sys_id=sys_id,
        ref_port="r",
        computer_in=measurement_in,
        computer_out=control_out,
    )

    # The reference, when the call above left it unexposed
    if "r" in controller.inputs and "r" not in diagram.inputs:
        add_boundary_input(diagram, sys_id, controller, "r")

    # The extra outputs of an MPC leaf, when it carries them
    for extra_out in ("x_ff", "z"):
        if extra_out in controller.outputs and extra_out not in diagram.outputs:
            diagram.connect_new_output_port(sys_id, extra_out, extra_out)


def as_step_diagram(computer_side: System | StepDiagramSystem) -> StepDiagramSystem:
    """Wrap *computer_side* in a :class:`StepDiagramSystem` when needed."""
    if isinstance(computer_side, StepDiagramSystem):
        return computer_side

    if isinstance(computer_side, (StepSystem, System)):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(computer_side, "controller")
        return diagram

    raise TypeError(
        f"computer_side must be StepDiagramSystem or System, "
        f"got {type(computer_side).__name__}"
    )


###############################################################################
# Internal machinery
###############################################################################


def builds_own_computer(computer_side: System | StepDiagramSystem) -> bool:
    """True when the block's class defines its own ``%`` (e.g. an MPC block)."""
    return (
        isinstance(computer_side, System)
        and type(computer_side).__mod__ is not System.__mod__
    )


def as_plant_diagram(plant: System | DiagramSystem) -> DiagramSystem:
    """Wrap a leaf plant in a flow diagram, its camera hints carried over."""
    if isinstance(plant, DiagramSystem):
        return plant

    if isinstance(plant, System):
        diagram = DiagramSystem()
        diagram.add_subsystem(plant, "plant")
        propagate_animation_camera(diagram, plant)
        return diagram

    raise TypeError(
        f"plant must be DiagramSystem or System, got {type(plant).__name__}"
    )


def leaf_sys_id(
    obj: System | StepDiagramSystem | DiagramSystem, *, default: str
) -> str | None:
    """A diagram's one subsystem id (``None`` for several), or ``default`` for a leaf."""
    if isinstance(obj, (StepDiagramSystem, DiagramSystem)):
        if len(obj.subsystems) == 1:
            return next(iter(obj.subsystems))
        return None

    return default


def ensure_computer_boundary_ports(
    diagram: StepDiagramSystem,
    *,
    sys_id: str | None,
    ref_port: str,
    computer_in: str,
    computer_out: str,
) -> None:
    """Expose the leaf's reference, measurement and command ports the diagram lacks."""
    if sys_id is None:
        return

    subsystem = diagram.subsystems[sys_id]
    if ref_port in subsystem.inputs and ref_port not in diagram.inputs:
        add_boundary_input(diagram, sys_id, subsystem, ref_port)
    if computer_in in subsystem.inputs and computer_in not in diagram.inputs:
        add_boundary_input(diagram, sys_id, subsystem, computer_in)
    if computer_out in subsystem.outputs and computer_out not in diagram.outputs:
        diagram.connect_new_output_port(sys_id, computer_out, computer_out)


def ensure_plant_boundary_ports(
    diagram: DiagramSystem,
    *,
    sys_id: str | None,
    plant_in: str,
    plant_out: str,
    output_port: str,
    in_place: bool,
) -> DiagramSystem:
    """Return *diagram* with the loop's plant boundary ports.

    Missing ports are added to *diagram* itself only when ``in_place`` (a
    diagram built here around a leaf plant); a user's plant diagram is left
    unchanged and the ports go on a new diagram with its ids, wiring and ``x0``.
    """
    if sys_id is None:
        return diagram

    subsystem = diagram.subsystems[sys_id]
    add_input = plant_in in subsystem.inputs and plant_in not in diagram.inputs

    # The loop reads `plant_out`; `output_port` is also exposed for logging.
    add_outputs = [
        port
        for port in dict.fromkeys((plant_out, output_port))
        if port in subsystem.outputs and port not in diagram.outputs
    ]

    # A user's diagram is copied before any port is added
    if not in_place and (add_input or add_outputs):
        diagram = new_diagram_like(diagram)

    if add_input:
        add_boundary_input(diagram, sys_id, subsystem, plant_in)
    for port in add_outputs:
        diagram.connect_new_output_port(sys_id, port, port)

    return diagram


def add_boundary_input(diagram, sys_id, subsystem, port_id) -> None:
    """Add a boundary input shaped like the subsystem's port ``port_id``, and wire it in."""
    port = subsystem.inputs[port_id]
    diagram.add_input_port(port_id, dim=port.dim, nominal_value=port.nominal_value)
    diagram.connect("input", port_id, sys_id, port_id)


def require_matching_computer(computer, step_diagram, step_schedule) -> None:
    """Refuse a runtime built on another diagram or another schedule than the loop's."""
    if computer.diagram is not step_diagram:
        raise ValueError("computer diagram does not match computer_side")
    if computer.schedule is not step_schedule:
        raise ValueError("computer schedule does not match schedule argument")


def boundary_measurement_of(diagram: StepDiagramSystem) -> str:
    """The measurement input of a multi-leaf step diagram: ``y``, else ``x``."""
    if "y" in diagram.inputs:
        return "y"
    if "x" in diagram.inputs:
        return "x"

    raise ValueError("multi-leaf computer diagram needs boundary input 'y' (or 'x')")


def boundary_command_of(diagram: StepDiagramSystem) -> str:
    """The command output of a multi-leaf step diagram: ``u_nom``, else ``u``, else ``u_ff``."""
    if "u_nom" in diagram.outputs:
        return "u_nom"
    if "u" in diagram.outputs:
        return "u"
    if "u_ff" in diagram.outputs:
        return "u_ff"

    raise ValueError(
        "multi-leaf computer diagram needs boundary output 'u_nom', 'u', or 'u_ff'"
    )


def plant_leaf(plant: System | DiagramSystem) -> System:
    """The plant's one block: the leaf itself, or a diagram's single subsystem."""
    if isinstance(plant, DiagramSystem):
        if len(plant.subsystems) != 1:
            raise ValueError(
                "auto wiring requires a single-subsystem plant diagram or leaf plant"
            )
        return next(iter(plant.subsystems.values()))

    return plant


def computer_leaf(computer_side: System | StepDiagramSystem) -> System:
    """The computer's one block: the leaf itself, or a step diagram's single subsystem."""
    if isinstance(computer_side, StepDiagramSystem):
        if len(computer_side.subsystems) != 1:
            raise ValueError(
                "auto wiring requires a single-subsystem step diagram or leaf block"
            )
        return next(iter(computer_side.subsystems.values()))

    return computer_side
