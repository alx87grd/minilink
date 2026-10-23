"""MPC visualization: the receding horizons of a hybrid rollout, as animation overlays."""

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

###############################################################################
# Public API
###############################################################################


def mpc_animation_overlays(
    result: HybridSimResult,
    planner,
    *,
    scene=None,
    track=None,
    traj=None,
    trail: bool = True,
    reference_pad: float | None = None,
    t0: float = 0.0,
    dt_mpc: float | None = None,
    scene_color: str = "tab:red",
    scene_opacity: float = 0.45,
) -> list:
    """
    Build ``animate(overlays=[...])`` layers for a hybrid MPC rollout.

    Reconstructs horizon polylines from the computer tick history, optionally
    an executed-path trail, and optionally a straight reference line, obstacle
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
    traj : Trajectory, optional
        Executed path for the trail. Defaults to ``result.plant``. Pass the
        leaf-car trajectory when the hybrid plant is a nested diagram, so the
        trail is pose ``(x, y)`` and not the first two diagram states.
    trail : bool, optional
        Draw the executed-path polyline. ``False`` keeps the receding horizon
        and any track/scene layers without the growing trail.
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
    dt_mpc = mpc_period_of(result, dt_mpc)
    traj = result.plant if traj is None else traj

    # The receding horizons: one plan per computer tick, in absolute time
    mpc_plans = mpc_plans_from_rollout(
        result.computer,
        planner.transcription,
        planner.problem,
        t0=t0,
        dt_mpc=dt_mpc,
    )

    # The time-indexed layers: the horizon, then the executed trail
    history_layers = {
        "horizon": HorizonPolyline(
            mpc_plans,
            color="#ef6c00",
            linewidth=2.0,
            style="--",
        ),
    }
    if trail:
        history_layers["trail"] = TrajectoryPolyline(
            traj,
            window="prefix",
            color="#1565c0",
            style="--",
            linewidth=1.0,
        )

    # A dashed y = 0 reference spanning the executed path in x, padded at each end
    if reference_pad is not None:
        pad = float(reference_pad)
        x0_ref = float(traj.x[0, 0]) - pad
        x1_ref = float(traj.x[0, -1]) + pad
        points = np.array([[x0_ref, 0.0, 0.0], [x1_ref, 0.0, 0.0]])
        history_layers["reference"] = CustomLine(
            points,
            color="k",
            linewidth=1.0,
            style="--",
        )

    # The static layers first, the track corridor and the obstacle scene, then the
    # time-indexed ones on top
    overlays = []
    if track is not None:
        # Lazy import: the spatial package loads when a track is drawn, not with MPC
        from minilink.planning.spatial.overlays import TrackCorridorOverlay

        overlays.append(TrackCorridorOverlay(track))
    if scene is not None:
        overlays.append(scene.as_visualizer(color=scene_color, opacity=scene_opacity))
    overlays.append(SceneHistory(**history_layers))

    return overlays


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
    z_hist = decision_history_of(computer, z_source, signal_name)

    t_grid = np.asarray(transcription.options.t(problem), dtype=float).reshape(-1)
    plans: list[tuple[float, Trajectory]] = []

    for col in range(computer.n_samples):
        # The tick's solve time t_solve = t₀ + k Δt
        k = int(computer.k[col])
        t_solve = float(t0 + k * dt_mpc)

        # Its decision vector unpacked as a plan, moved to absolute time
        z_k = np.asarray(z_hist[:, col], dtype=float).reshape(-1)
        x_plan, u_plan = transcription.unpack(z_k, problem)
        plan = Trajectory(t=t_grid + t_solve, x=x_plan, u=u_plan)
        plans.append((t_solve, plan))

    return plans


###############################################################################
# Internal machinery
###############################################################################


def mpc_period_of(result, dt_mpc):
    """The MPC sample period: ``dt_mpc`` as given, or the hybrid computer's base period."""
    if dt_mpc is not None:
        return dt_mpc

    hybrid = result.hybrid
    if hybrid is None:
        raise ValueError(
            "dt_mpc is required when result.hybrid is unset; "
            "pass dt_mpc=... or attach hybrid to HybridSimResult."
        )

    return float(hybrid.computer.schedule.dt_base)


def decision_history_of(computer, z_source, signal_name):
    """The per-tick decision vectors, as columns: ``computer.signals[signal_name]`` or ``computer.x``."""
    if z_source == "signals":
        if signal_name not in computer.signals:
            raise KeyError(
                f"z_source='signals' requires computer.signals[{signal_name!r}]"
            )
        return computer.signals[signal_name]

    if z_source == "x":
        return computer.x

    raise ValueError("z_source must be 'signals' or 'x'")
