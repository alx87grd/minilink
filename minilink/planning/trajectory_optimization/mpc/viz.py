"""MPC visualization: the receding horizons of a sampled MPC loop's run, as animation overlays."""

from __future__ import annotations

import numpy as np

from minilink.core.discrete.sampled_diagram import plant_run_of
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
    block: str | None = None,
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
        The MPC block's path in the loop (``"controller"`` for ``mpc % dt @ plant``,
        ``"controller/DelayedPredictiveController"`` inside a dual-rate computer);
        default: the block of ``loop`` that holds ``mpc``'s face, else the run's one
        plan signal.
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
    traj = plant_trail_of(rollout, loop) if traj is None else traj

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
    rollout: Trajectory, mpc, *, loop=None, block: str | None = None
) -> list[tuple[float, Trajectory]]:
    """
    The plans a sampled MPC loop's run solved, in absolute time: ``(origin, plan)`` pairs.

    Each plan is the MPC block's ``z`` read on its knots: its origin ``z[-1]``
    plus the knots' offsets, the state and input rows of a collocation plan,
    the pairs :class:`~minilink.graphical.animation.primitives.HorizonPolyline`
    draws. The plan is the run's ``"<block>:z"`` signal, else the block's
    state slice of ``loop``; a dense run repeats it inside each period, and
    each solved plan is kept once (the default guess of tick 0 is not a
    solve). A plan without state knots (single shooting) is refused.
    """
    pv = mpc.pv
    require_state_knots(pv)
    z_history = plan_history_of(rollout, mpc, loop, block)
    guess = np.asarray(mpc.x0, dtype=float)
    n_steps = pv.offsets.size

    plans: list[tuple[float, Trajectory]] = []
    for col in range(z_history.shape[1]):
        z_k = np.asarray(z_history[:, col], dtype=float)
        repeated = col > 0 and np.array_equal(z_k, z_history[:, col - 1])
        if repeated or np.array_equal(z_k, guess):
            continue

        # The plan: its knots from its own origin, its state and input rows
        rows = z_k[:-1].reshape(-1, n_steps)
        origin = float(z_k[-1])
        plan = Trajectory(t=origin + pv.offsets, x=rows[: pv.n], u=rows[-pv.m :])
        plans.append((origin, plan))

    return plans


def plant_run(rollout: Trajectory, loop=None, params=None) -> Trajectory:
    """
    The plant's run inside a loop's run: its role, else its slice of ``loop`` (a sampled loop's ``compute_trajectory``).

    With ``loop``, ``x`` is the plant's state slice and ``u`` the input the
    loop fed it, as its compiled plan gathers it at each tick (an open port at
    its nominal value) and held over the period, so the run can be handed on as
    the plant's own (to a diagram's ``trajectory_of`` when the plant is itself
    a diagram). ``params`` is the tree the run was made with (``None``: the
    loop's own). Reading ``u`` evaluates every block once per tick: a pure MPC
    solves again.
    """
    if loop is None:
        return rollout.of("plant")

    _, run = plant_run_of(loop, rollout, params)

    return run


###############################################################################
# Internal machinery
###############################################################################


def plant_trail_of(rollout, loop):
    """The plant's states for the executed trail, with no block evaluated again (its role with no ``loop``)."""
    if loop is None:
        return rollout.of("plant")

    return Trajectory(t=rollout.t, x=plant_states_of(rollout, loop), u=rollout.u)


def plant_states_of(rollout, loop):
    """The plant's state slice of a sampled loop's run."""
    start, end = loop.state_index["plant"]

    return rollout.x[start:end]


def plan_history_of(rollout, mpc, loop, block):
    """The MPC block's plan at every sample, as columns: its ``z`` signal, else its state slice of ``loop``."""
    # Lazy import: the sampled plan's layout loads with the first plan read
    from minilink.core.compile.plan import nested_layout

    path = block_path_of(rollout, mpc, loop, block)
    name = f"{'/'.join(path)}:z"
    if rollout.has_signal(name):
        return rollout.get_signal(name)
    require_loop(loop, name)

    # The block's state slice in the loop's nested layout, by its path ("a/b")
    x_slice, _ = nested_layout(loop)[path]

    return rollout.x[x_slice]


def block_path_of(rollout, mpc, loop, block):
    """The MPC block's path: as given, else the block of ``loop`` holding ``mpc``'s face, else the run's one plan signal."""
    if block is not None:
        return tuple(block.split("/"))
    if loop is not None:
        paths = [path for path, sys in blocks_of(loop) if holds_plan_of(sys, mpc.pv)]
        require_one_block(paths)
        return paths[0]

    names = [name[: -len(":z")] for name in rollout.signals if name.endswith(":z")]
    if not names:
        require_loop(loop, "<block>:z")
    require_one_block(names)

    return tuple(names[0].split("/"))


def blocks_of(diagram, prefix=()):
    """Every leaf block of a diagram with its path, its nested boxes walked."""
    for sys_id, sys in diagram.subsystems.items():
        path = (*prefix, sys_id)
        if hasattr(sys, "subsystems"):
            yield from blocks_of(sys, path)
        else:
            yield path, sys


def holds_plan_of(block, pv) -> bool:
    """Whether ``block`` (or the block a clocked view wraps) is the one whose state is the plan of ``pv``: the delayed MPC, or the zero-delay MPC's memory."""
    own = getattr(block, "source", None) or block

    return getattr(own, "pv", None) is pv and int(block.n) == pv.n_z


def require_state_knots(pv):
    """Refuse a plan without state knots: single shooting rolls its states out."""
    if pv.n_z - 1 != (pv.n + pv.m) * pv.offsets.size:
        raise NotImplementedError(
            "the plan carries no state knots (single shooting): read "
            "pv.solution(z).trajectory for its planned states"
        )


def require_one_block(found):
    """Refuse a search for the MPC block that finds none, or several."""
    if len(found) != 1:
        raise ValueError(
            f"found {len(found)} MPC plan(s) in the run ({found}): pass block= "
            "(its path in the loop, as 'controller' or 'controller/<id>')"
        )


def require_loop(loop, name):
    """Refuse a run with no plan signal when no loop names the plan's state slice."""
    if loop is None:
        raise ValueError(
            f"the run records no {name!r} signal: pass loop= (the loop it is a run "
            "of) to read the plan from its state"
        )
