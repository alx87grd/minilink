"""Shared validation for MPC diagram blocks."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from minilink.simulation.computer import Computer, StepSchedule


def validate_mpc_planner(planner) -> tuple[int, int, int]:
    """Validate a plan-producing planner and return ``(n, m, n_z)``.

    Accepts a duck-typed
    :class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`
    (or compatible) with ``compile_parametric_program``,
    ``has_parametric_program``, and ``solve_trajectory_from``. Auto-compiles
    when needed so MPC ticks never re-transcribe.
    """
    missing = [
        name
        for name in (
            "compile_parametric_program",
            "has_parametric_program",
            "solve_trajectory_from",
        )
        if not hasattr(planner, name)
    ]
    if missing:
        raise TypeError(
            "MPC block requires a TrajectoryOptimizationPlanner-compatible "
            f"planner with {missing}; got {type(planner).__name__}."
        )

    n_steps = int(planner.transcription.options.n_steps)
    if n_steps < 2:
        raise ValueError(
            f"MPC block requires transcription n_steps >= 2 for x_ff, got {n_steps}"
        )

    if not planner.has_parametric_program:
        planner.compile_parametric_program()

    if not planner.has_parametric_program:
        raise RuntimeError(
            "planner.compile_parametric_program() did not produce a parametric "
            "program (check compile_backend='jax' and transcription support)."
        )

    sys = planner.problem.sys
    n = int(sys.n)
    m = int(sys.m)
    n_z = int(planner.transcription.decision_dimension(planner.problem))
    return n, m, n_z


def export_mpc_to_computer(
    block,
    schedule: "StepSchedule | float | None" = None,
    *,
    dt_mpc: float | None = None,
) -> "Computer":
    """
    Build a :class:`~minilink.simulation.computer.Computer` from an MPC block.

    Warm-start blocks (``ModelPredictiveController(..., warm_start=True)``)
    default ``schedule`` from ``dt_mpc``. Algebraic (``warm_start=False``)
    blocks require an explicit ``schedule``.
    """
    from minilink.simulation.computer import StepSchedule, as_computer

    # Single-rate path: undo any dual-rate hooks left on the block.
    if hasattr(block, "_replan_divisor"):
        block._replan_divisor = 1
    latch = getattr(block, "_latch", None)
    if latch is not None and hasattr(latch, "set_after_solve"):
        latch.set_after_solve(None)

    block_dt = dt_mpc if dt_mpc is not None else getattr(block, "_dt_mpc", None)
    if schedule is None:
        if block_dt is None:
            raise ValueError(
                "schedule is required for algebraic ModelPredictiveController "
                "(warm_start=False); pass export_to_computer(dt_mpc) or use mpc % dt"
            )
        schedule = StepSchedule(dt_base=float(block_dt))
    elif block_dt is not None:
        dt_sched = (
            schedule.dt_base if isinstance(schedule, StepSchedule) else float(schedule)
        )
        if abs(dt_sched - float(block_dt)) > 1e-12:
            raise ValueError(
                f"schedule dt_base={dt_sched} does not match block dt_mpc={block_dt}"
            )
    return as_computer(block, schedule)


def export_mpc_dual_rate_computer(block, *, dt_broadcast: float) -> "Computer":
    """
    Multi-rate Computer: replan at ``block.dt_mpc``, broadcast at ``dt_broadcast``.

    Boundary ports: ``y`` → replan, ``u_nom`` from broadcast (primary control).

    Coupling between ``replan`` and ``broadcast`` is a **shared latch** on
    ``block`` (``generate_nominal_interpolator`` after NLP, then
    ``get_nominal_*``) — not a diagram port. Graphviz correctly shows no edge.
    """
    from minilink.core.diagram import StepDiagramSystem
    from minilink.planning.mpc.broadcast_block import MPCBroadcastController
    from minilink.simulation.computer import Computer, StepSchedule

    dt_mpc = float(block.dt_mpc)
    dt_b = float(dt_broadcast)
    if dt_b <= 0.0:
        raise ValueError(f"dt_broadcast must be positive, got {dt_b}")
    ratio = dt_mpc / dt_b
    d = int(round(ratio))
    if d < 1 or abs(ratio - d) > 1e-9:
        raise ValueError(
            f"dt_mpc / dt_broadcast must be a positive integer "
            f"(got dt_mpc={dt_mpc}, dt_broadcast={dt_b}, ratio={ratio})"
        )

    block._replan_divisor = d
    block._latch.set_after_solve(
        lambda: block.generate_nominal_interpolator(derivatives=True)
    )

    broadcast = MPCBroadcastController(block, dt_broadcast=dt_b, t0=float(block.t0))
    diagram = StepDiagramSystem()
    diagram.name = "MPC Dual-Rate"
    diagram.add_subsystem(block, "replan")
    diagram.add_subsystem(broadcast, "broadcast")

    # Boundary y → replan; u_nom / extras on the diagram.
    y_port = block.inputs["y"]
    diagram.add_input_port("y", dim=y_port.dim, nominal_value=y_port.nominal_value)
    diagram.connect("input", "y", "replan", "y")
    diagram.connect_new_output_port("broadcast", "u_nom", "u_nom")
    diagram.connect_new_output_port("broadcast", "x_nom", "x_nom")
    for extra in ("u_ff", "x_ff", "z"):
        if extra in block.outputs:
            diagram.connect_new_output_port("replan", extra, extra)

    schedule = StepSchedule(
        dt_base=dt_b,
        fire={"replan": d, "broadcast": 1},
    )
    return Computer(diagram, schedule)
