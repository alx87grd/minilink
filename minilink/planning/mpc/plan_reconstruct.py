"""Reconstruct MPC horizon trajectories from a computer StepRollout."""

from __future__ import annotations

import numpy as np

from minilink.core.step_rollout import StepRollout
from minilink.core.trajectory import Trajectory
from minilink.planning.mpc.transcription import MPCDirectCollocationTranscription
from minilink.planning.problems import PlanningProblem


def mpc_plans_from_rollout(
    computer: StepRollout,
    transcription: MPCDirectCollocationTranscription,
    problem: PlanningProblem,
    *,
    z_source: str = "signals",
    signal_name: str = "z",
    t0: float = 0.0,
    dt_mpc: float,
) -> list[tuple[float, Trajectory]]:
    """
    Build absolute-time MPC plans from per-tick packed decision vectors.

    Parameters
    ----------
    computer : StepRollout
        Tick-indexed computer rollout from :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`.
    transcription : MPCDirectCollocationTranscription
        Collocation transcription used by the MPC block.
    problem : PlanningProblem
        Template planning problem (system / cost layout).
    z_source : str, optional
        ``"signals"`` reads ``computer.signals[signal_name]``; ``"x"`` reads
        ``computer.x`` (warm-start step block state).
    signal_name : str, optional
        Signal key when ``z_source="signals"``.
    t0 : float, optional
        Simulation start time.
    dt_mpc : float
        MPC sample period (``Computer.schedule.dt_base``).

    Returns
    -------
    list of (float, Trajectory)
        ``(t_solve, plan)`` pairs suitable for :class:`~minilink.graphical.animation.primitives.HorizonPolyline`.
    """
    if z_source == "signals":
        if signal_name not in computer.signals:
            raise KeyError(
                f"z_source='signals' requires computer.signals[{signal_name!r}]"
            )
        z_hist = computer.signals[signal_name]
    elif z_source == "x":
        z_hist = computer.x
    else:
        raise ValueError("z_source must be 'signals' or 'x'")

    t_grid = np.asarray(transcription.options.t, dtype=float).reshape(-1)
    plans: list[tuple[float, Trajectory]] = []

    for col in range(computer.n_samples):
        k = int(computer.k[col])
        t_solve = float(t0 + k * dt_mpc)
        z_k = np.asarray(z_hist[:, col], dtype=float).reshape(-1)
        x_plan, u_plan = transcription.unpack(z_k, problem)
        plans.append(
            (
                t_solve,
                Trajectory(
                    t=t_grid + t_solve,
                    x=x_plan,
                    u=u_plan,
                ),
            )
        )

    return plans
