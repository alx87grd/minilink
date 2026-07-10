"""Default MPC hybrid animation overlays from rollout results."""

from __future__ import annotations

import numpy as np

from minilink.graphical.animation.drawables import SceneHistory
from minilink.graphical.animation.primitives import (
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)
from minilink.planning.mpc.plan_reconstruct import mpc_plans_from_rollout
from minilink.planning.mpc.planner import MPCPlanner
from minilink.simulation.hybrid_simulator import HybridSimResult


def mpc_animation_overlays(
    result: HybridSimResult,
    planner: MPCPlanner,
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
    planner : MPCPlanner
        MPC block planner (transcription and template problem).
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
