"""MPC post-sim visualization: horizon unpack and animation overlays."""

from __future__ import annotations

import numpy as np

from minilink.core.step_rollout import StepRollout
from minilink.core.trajectory import Trajectory
from minilink.graphical.animation.drawables import SceneHistory
from minilink.graphical.animation.primitives import (
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationTranscription,
)
from minilink.simulation.hybrid_simulator import HybridSimResult


def mpc_plans_from_rollout(
    computer: StepRollout,
    transcription: DirectCollocationTranscription,
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
    transcription : DirectCollocationTranscription
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

    t_grid = np.asarray(transcription.options.t(problem), dtype=float).reshape(-1)
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


def mpc_animation_overlays(
    result: HybridSimResult,
    planner,
    *,
    scene=None,
    track=None,
    reference_pad: float | None = None,
    t0: float = 0.0,
    dt_mpc: float | None = None,
    scene_color: str = "tab:red",
    scene_opacity: float = 0.45,
) -> list:
    """
    Build ``animate(overlays=[...])`` layers for a hybrid MPC rollout.

    Reconstructs horizon polylines from the computer tick history, adds an
    executed-path trail, and optionally a straight reference line, obstacle
    scene skin, or track corridor.

    Parameters
    ----------
    result : HybridSimResult
        Output of :meth:`~minilink.core.hybrid_diagram.HybridDiagram.compute_trajectory`.
    planner :
        Trajopt planner (transcription and template problem).
    scene : Scene, optional
        Collision scene drawn via :meth:`~minilink.planning.spatial.scene.Scene.as_visualizer`.
    track : Track, optional
        2-D track corridor edges and centerline.
    reference_pad : float, optional
        When set, draw a dashed world ``y=0`` reference spanning the executed
        path in ``x`` by this padding on each end.
    t0 : float, optional
        Simulation start time for horizon alignment.
    dt_mpc : float, optional
        MPC sample period. Defaults to ``result.hybrid.computer.schedule.dt_base``.
    scene_color, scene_opacity
        Passed to :meth:`~minilink.planning.spatial.scene.Scene.as_visualizer`.

    Returns
    -------
    list
        Overlay drawables ready for :meth:`~minilink.core.hybrid_diagram.HybridDiagram.animate`.
    """
    if dt_mpc is None:
        hybrid = result.hybrid
        if hybrid is None:
            raise ValueError(
                "dt_mpc is required when result.hybrid is unset; "
                "pass dt_mpc=... or attach hybrid to HybridSimResult."
            )
        dt_mpc = float(hybrid.computer.schedule.dt_base)

    traj = result.plant
    mpc_plans = mpc_plans_from_rollout(
        result.computer,
        planner.transcription,
        planner.problem,
        t0=t0,
        dt_mpc=dt_mpc,
    )

    history_layers = {
        "trail": TrajectoryPolyline(
            traj,
            window="prefix",
            color="#1565c0",
            style="--",
            linewidth=1.0,
        ),
        "horizon": HorizonPolyline(
            mpc_plans,
            color="#ef6c00",
            linewidth=2.0,
            style="--",
        ),
    }
    if reference_pad is not None:
        pad = float(reference_pad)
        x0_ref = float(traj.x[0, 0]) - pad
        x1_ref = float(traj.x[0, -1]) + pad
        history_layers["reference"] = CustomLine(
            np.array([[x0_ref, 0.0, 0.0], [x1_ref, 0.0, 0.0]]),
            color="k",
            linewidth=1.0,
            style="--",
        )

    overlays = []
    if track is not None:
        from minilink.planning.spatial.overlays import TrackCorridorOverlay

        overlays.append(TrackCorridorOverlay(track))
    if scene is not None:
        overlays.append(scene.as_visualizer(color=scene_color, opacity=scene_opacity))
    overlays.append(SceneHistory(**history_layers))
    return overlays
