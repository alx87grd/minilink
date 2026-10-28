"""MPC visualization: the receding horizons of a sampled MPC loop's run, as animation overlays."""

from __future__ import annotations

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.graphical.animation.drawables import SceneHistory
from minilink.graphical.animation.primitives import (
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)

###############################################################################
# Public API
###############################################################################


def mpc_animation_overlays(
    rollout: Trajectory,
    mpc,
    *,
    loop=None,
    block: str = "controller",
    scene=None,
    track=None,
    traj=None,
    trail: bool = True,
    reference_pad: float | None = None,
    scene_color: str = "tab:red",
    scene_opacity: float = 0.45,
) -> list:
    """
    Build ``animate(overlays=[...])`` layers for a sampled MPC loop's run.

    The receding horizons from the plan ``z`` the MPC block records at each
    tick, optionally an executed-path trail, a straight reference line, an
    obstacle scene skin, or a track corridor.

    Parameters
    ----------
    rollout : Trajectory
        A run of the loop ``mpc % dt @ plant`` (``compute_rollout``, or the
        evaluator's ``trajectory``), the MPC block's ``z`` among its signals.
    mpc : DelayedPredictiveController
        The MPC block (its compiled face reads the plans).
    loop : SampledDiagramSystem, optional
        The loop the run is of, for a run that records its state only
        (``loop.compute_trajectory``): the plan and the plant's path are read
        from their state slices.
    block : str, optional
        The MPC block's id in the loop (``"controller"`` for ``mpc % dt @ plant``).
    scene : Scene, optional
        Collision scene drawn via :meth:`~minilink.planning.spatial.scene.Scene.as_visualizer`.
    track : Track, optional
        2-D track corridor edges and centerline.
    traj : Trajectory, optional
        Executed path for the trail; default the run's plant.
    trail : bool, optional
        Draw the executed-path polyline.
    reference_pad : float, optional
        When set, draw a dashed world ``y=0`` reference spanning the executed
        path in ``x`` by this padding on each end.
    scene_color, scene_opacity
        Passed to :meth:`~minilink.planning.spatial.scene.Scene.as_visualizer`.

    Returns
    -------
    list
        Overlay drawables for ``animate(overlays=...)``.
    """
    traj = plant_path_of(rollout, loop) if traj is None else traj

    # The receding horizons: one plan per tick, in absolute time
    mpc_plans = mpc_plans_from_rollout(rollout, mpc, loop=loop, block=block)

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
    rollout: Trajectory, mpc, *, loop=None, block: str = "controller"
) -> list[tuple[float, Trajectory]]:
    """
    The plan of every tick of a sampled MPC loop's run, in absolute time: ``(t_k, plan)`` pairs.

    Each plan is the MPC block's ``z`` at that tick read on its knots: its
    origin ``z[-1]`` plus the knots' offsets, the state and input rows of a
    collocation plan, the pairs :class:`~minilink.graphical.animation.primitives.HorizonPolyline`
    draws. The plan is the run's ``"<block>:z"`` signal, else the block's
    state slice of ``loop``; a dense run repeats it inside each period, and
    each plan is kept once, at the first sample that holds it.
    """
    pv = mpc.pv
    z_history = plan_history_of(rollout, loop, block)
    n_steps = pv.offsets.size

    plans: list[tuple[float, Trajectory]] = []
    for col in range(z_history.shape[1]):
        z_k = np.asarray(z_history[:, col], dtype=float)
        if col > 0 and np.array_equal(z_k, z_history[:, col - 1]):
            continue

        # The tick's plan: its knots from its own origin, its state and input rows
        rows = z_k[:-1].reshape(-1, n_steps)
        t_plan = z_k[-1] + pv.offsets
        plan = Trajectory(t=t_plan, x=rows[: pv.n], u=rows[-pv.m :])
        plans.append((float(rollout.t[col]), plan))

    return plans


###############################################################################
# Internal machinery
###############################################################################


###############################################################################
# Internal machinery
###############################################################################


def plan_history_of(rollout, loop, block):
    """The MPC block's plan at every sample, as columns: its ``z`` signal, else its state slice of ``loop``."""
    name = f"{block}:z"
    if rollout.has_signal(name):
        return rollout.get_signal(name)
    require_loop(loop, name)

    # Lazy import: the sampled plan's layout loads with the first plan read
    from minilink.core.compile.plan import nested_layout

    # The block's state slice in the loop's nested layout, by its path ("a/b")
    x_slice, _ = nested_layout(loop)[tuple(block.split("/"))]

    return rollout.x[x_slice]


def plant_path_of(rollout, loop):
    """The plant's path in the run: its role, else its state slice of ``loop``."""
    if loop is None:
        return rollout.of("plant")

    start, end = loop.state_index["plant"]

    return Trajectory(t=rollout.t, x=rollout.x[start:end], u=rollout.u)


def require_loop(loop, name):
    """Refuse a run with no plan signal when no loop names the plan's state slice."""
    if loop is None:
        raise ValueError(
            f"the run records no {name!r} signal: pass loop= (the loop it is a run "
            "of) to read the plan from its state"
        )
