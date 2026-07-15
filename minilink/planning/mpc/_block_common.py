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

    Warm-start :class:`~minilink.planning.mpc.step_block.MPCStatefulController` defaults
    ``schedule`` from ``dt_mpc`` on the block. Stateless
    :class:`~minilink.planning.mpc.controller.MPCStatelessController` requires ``schedule``.
    """
    from minilink.simulation.computer import StepSchedule, as_computer

    block_dt = dt_mpc if dt_mpc is not None else getattr(block, "_dt_mpc", None)
    if schedule is None:
        if block_dt is None:
            raise ValueError(
                "schedule is required for stateless MPCStatelessController; "
                "pass export_to_computer(MPC_DT) or use mpc % MPC_DT"
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
