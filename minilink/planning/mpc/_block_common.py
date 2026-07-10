"""Shared validation for MPC diagram blocks."""

from __future__ import annotations

from typing import TYPE_CHECKING

from minilink.planning.mpc.planner import MPCPlanner

if TYPE_CHECKING:
    from minilink.simulation.computer import Computer, StepSchedule


def validate_mpc_planner(planner: MPCPlanner) -> tuple[int, int, int]:
    """Validate a prepared planner and return ``(n, m, n_z)``."""
    if not isinstance(planner, MPCPlanner):
        raise TypeError(f"MPC block requires MPCPlanner, got {type(planner)}")
    if planner.program_evaluator is None:
        raise RuntimeError("MPCPlanner.prepare() must run before MPC block")

    n_steps = int(planner.transcription.options.n_steps)
    if n_steps < 2:
        raise ValueError(
            f"MPC block requires transcription n_steps >= 2 for x_ff, got {n_steps}"
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

    Warm-start :class:`~minilink.planning.mpc.step_block.MPCStepBlock` defaults
    ``schedule`` from ``dt_mpc`` on the block. Stateless
    :class:`~minilink.planning.mpc.controller.MPCController` requires ``schedule``.
    """
    from minilink.simulation.computer import StepSchedule, as_computer

    block_dt = dt_mpc if dt_mpc is not None else getattr(block, "_dt_mpc", None)
    if schedule is None:
        if block_dt is None:
            raise ValueError(
                "schedule is required for stateless MPCController; "
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
