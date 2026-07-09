"""
Hybrid composition shortcuts.

``Computer @ plant`` and :func:`hybrid_closed_loop` build a :class:`HybridDiagram`
with default feedback wiring. Schedule attaches to the discrete runtime before
composition.
"""

from __future__ import annotations

from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.hybrid_diagram import BoundaryConnection, HybridDiagram
from minilink.core.system import StepSystem, System
from minilink.simulation.computer import Computer, StepSchedule

# Public API


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

    Parameters
    ----------
    computer_side : System or StepDiagramSystem
        Step diagram or leaf controller wrapped in a step diagram.
    plant : System or DiagramSystem
        Continuous plant wrapped in a flow diagram when needed.
    schedule : StepSchedule or float
        Base tick schedule for the computer runtime.
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
    step_diagram = _as_step_diagram(computer_side)
    plant_diagram = _as_plant_diagram(plant)
    _ensure_computer_boundary_ports(
        step_diagram,
        sys_id=_leaf_sys_id(computer_side, default="ctl"),
        ref_port=ref_port,
        computer_in=computer_in,
        computer_out=computer_out,
    )
    _ensure_plant_boundary_ports(
        plant_diagram,
        sys_id=_leaf_sys_id(plant, default="plant"),
        plant_in=plant_in,
        plant_out=plant_out,
        output_port=output_port,
    )

    if isinstance(schedule, StepSchedule):
        step_schedule = schedule
    else:
        step_schedule = StepSchedule(dt_base=float(schedule))

    if computer is None:
        runtime = Computer(step_diagram, step_schedule)
    else:
        if computer.diagram is not step_diagram:
            raise ValueError("computer diagram does not match computer_side")
        if computer.schedule is not step_schedule:
            raise ValueError("computer schedule does not match schedule argument")
        runtime = computer

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

    if extra_boundaries:
        for conn in extra_boundaries:
            hybrid.connect_boundary(
                direction=conn.direction,
                computer_port=conn.computer_port,
                plant_port=conn.plant_port,
            )

    return hybrid


# Internal machinery


def _as_step_diagram(computer_side: System | StepDiagramSystem) -> StepDiagramSystem:
    if isinstance(computer_side, StepDiagramSystem):
        return computer_side
    if isinstance(computer_side, (StepSystem, System)):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(computer_side, "ctl")
        return diagram
    raise TypeError(
        f"computer_side must be StepDiagramSystem or System, "
        f"got {type(computer_side).__name__}"
    )


def _as_plant_diagram(plant: System | DiagramSystem) -> DiagramSystem:
    if isinstance(plant, DiagramSystem):
        return plant
    if isinstance(plant, System):
        diagram = DiagramSystem()
        diagram.add_subsystem(plant, "plant")
        return diagram
    raise TypeError(
        f"plant must be DiagramSystem or System, got {type(plant).__name__}"
    )


def _leaf_sys_id(
    obj: System | StepDiagramSystem | DiagramSystem, *, default: str
) -> str | None:
    if isinstance(obj, (StepDiagramSystem, DiagramSystem)):
        if len(obj.subsystems) == 1:
            return next(iter(obj.subsystems))
        return None
    return default


def _ensure_computer_boundary_ports(
    diagram: StepDiagramSystem,
    *,
    sys_id: str | None,
    ref_port: str,
    computer_in: str,
    computer_out: str,
) -> None:
    if sys_id is None:
        return
    subsystem = diagram.subsystems[sys_id]
    if ref_port in subsystem.inputs and ref_port not in diagram.inputs:
        port = subsystem.inputs[ref_port]
        diagram.add_input_port(
            ref_port,
            dim=port.dim,
            nominal_value=port.nominal_value,
        )
        diagram.connect("input", ref_port, sys_id, ref_port)
    if computer_in in subsystem.inputs and computer_in not in diagram.inputs:
        port = subsystem.inputs[computer_in]
        diagram.add_input_port(
            computer_in,
            dim=port.dim,
            nominal_value=port.nominal_value,
        )
        diagram.connect("input", computer_in, sys_id, computer_in)
    if computer_out in subsystem.outputs and computer_out not in diagram.outputs:
        diagram.connect_new_output_port(sys_id, computer_out, computer_out)


def _ensure_plant_boundary_ports(
    diagram: DiagramSystem,
    *,
    sys_id: str | None,
    plant_in: str,
    plant_out: str,
    output_port: str,
) -> None:
    if sys_id is None:
        return
    subsystem = diagram.subsystems[sys_id]
    if plant_in in subsystem.inputs and plant_in not in diagram.inputs:
        port = subsystem.inputs[plant_in]
        diagram.add_input_port(
            plant_in,
            dim=port.dim,
            nominal_value=port.nominal_value,
        )
        diagram.connect("input", plant_in, sys_id, plant_in)
    if plant_out in subsystem.outputs and output_port not in diagram.outputs:
        diagram.connect_new_output_port(sys_id, plant_out, output_port)
