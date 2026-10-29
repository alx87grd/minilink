"""MPC visualization: the receding horizons of a sampled MPC loop's run, as animation overlays."""

from __future__ import annotations

import numpy as np

from minilink.core.discrete.sampled_diagram import plant_run_of
from minilink.core.trajectory import Trajectory
from minilink.graphical.animation.drawables import Overlay, SceneHistory
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
    samples: int | None = None,
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
    samples : int, optional
        For a sampling planner (MPPI): draw each tick's ``samples`` heaviest
        sampled futures, shaded by weight, and the plan they moved to
        (:func:`mpc_sampled_futures`, rebuilt from the run, needs ``loop``).

    Returns
    -------
    list
        Overlay drawables for ``animate(overlays=...)``.
    """
    traj = plant_trail_of(rollout, loop) if traj is None else traj

    # The receding horizons: one plan per tick, in absolute time
    mpc_plans = mpc_plans_from_rollout(rollout, mpc, loop=loop, block=block)

    # The sampled futures of each tick, rebuilt from the run (a sampling planner)
    futures = None
    if samples is not None:
        futures = mpc_sampled_futures(rollout, mpc, loop=loop, n_shown=samples)

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
    if futures is not None:
        overlays.append(SampledFutures(futures))
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

    # A plan without its states (MPPI's inputs alone): the plans of the rebuilt solves
    if not pv.predicts and hasattr(pv, "samples"):
        futures = mpc_sampled_futures(rollout, mpc, loop=loop, n_shown=1)
        return [(fan["t"], fan_plan(pv, fan)) for fan in futures]

    require_state_knots(pv)
    z_history = plan_history_of(rollout, mpc, loop, block)
    guess = np.asarray(mpc.x0, dtype=float)

    plans: list[tuple[float, Trajectory]] = []
    for col in range(z_history.shape[1]):
        z_k = np.asarray(z_history[:, col], dtype=float)
        repeated = col > 0 and np.array_equal(z_k, z_history[:, col - 1])
        if repeated or np.array_equal(z_k, guess):
            continue

        # The plan: its knots from its own origin, its state and input rows
        plans.append((float(z_k[-1]), pv.planned_trajectory(z_k)))

    return plans


def mpc_sampled_futures(rollout: Trajectory, mpc, *, loop, n_shown: int = 32) -> list:
    """
    The fan of every solve of a sampled MPC loop's run on a sampling planner (MPPI): one dict per solve.

    Each tick's solve inputs are read back through the loop (the block's
    inputs from the blocks it reads, the solve not run) and its samples drawn
    again on the solve's own key, the seed and the tick, so each fan is the
    solve's: ``{"t", "futures" (n_shown, N + 1, n), "weights", "plan" (N + 1, n),
    "inputs" (N, m)}``, the heaviest futures first (``pv.samples``).
    """
    pv = mpc.pv
    require_samples(pv)

    return [
        pv.samples(P, z0, n_shown) for _, P, z0 in tick_solves_of(rollout, mpc, loop)
    ]


def plot_sampled_futures(
    rollout: Trajectory,
    mpc,
    *,
    loop,
    ticks,
    axes=(0, 1),
    n_shown: int = 32,
    ax=None,
    show: bool = True,
):
    """
    The fans of a sampling MPC at the solves of ``ticks``: the heaviest futures shaded by weight, the plan in black, the closed loop.

    ``axes`` names what is drawn against what: two state indices (``(0, 1)``,
    the car's plane), or ``("t", i)`` for the state ``i`` against time (the
    pendulum's angle). ``ticks`` index the solves of the run. Returns
    ``(fig, ax)``.
    """
    # Lazy import: matplotlib loads with the first figure
    import matplotlib.pyplot as plt

    futures = mpc_sampled_futures(rollout, mpc, loop=loop, n_shown=n_shown)
    plant = plant_trail_of(rollout, loop)
    pv = mpc.pv
    if ax is None:
        fig, ax = plt.subplots()
    else:
        fig = ax.figure

    def coordinates(t, x_path):
        """The drawn pair of a path: time or a state on each axis."""
        return [t if axis == "t" else x_path[:, axis] for axis in axes]

    # Each fan: its futures shaded by weight, the plan they moved to
    for tick in ticks:
        fan = futures[tick]
        t_fan = fan["t"] + pv.dt * np.arange(fan["plan"].shape[0])
        weights = fan["weights"] / fan["weights"].max()
        for x_path, weight in zip(fan["futures"], weights):
            ax.plot(
                *coordinates(t_fan, x_path),
                color="tab:blue",
                alpha=0.08 + 0.5 * weight,
                lw=0.7,
            )
        ax.plot(*coordinates(t_fan, fan["plan"]), color="black", lw=1.5)

    # The closed loop
    ax.plot(
        *coordinates(plant.t, plant.x.T), color="tab:grey", lw=2, label="closed loop"
    )
    ax.set_xlabel("t [s]" if axes[0] == "t" else f"x[{axes[0]}]")
    ax.set_ylabel("t [s]" if axes[1] == "t" else f"x[{axes[1]}]")
    ax.set_title(f"MPPI: {n_shown} of {pv.K} sampled futures, shaded by weight")
    ax.legend(loc="best", fontsize=8)
    if show:
        plt.show()

    return fig, ax


class SampledFutures(Overlay):
    """
    The latest solve's sampled futures and its plan, in the world frame: an animation overlay.

    A future's weight sets its hue, its alpha (matplotlib) and its ribbon
    width (meshcat, which keeps the hue and drops the alpha). ``futures`` is
    :func:`mpc_sampled_futures`; ``axes`` the two states drawn in the plane.
    """

    def __init__(self, futures, *, axes=(0, 1), cmap="Blues"):
        # Lazy import: matplotlib's colormaps load with the first overlay
        from matplotlib import colormaps

        self.futures = futures
        self.axes = list(axes)
        self.cmap = colormaps[cmap]

    def get_dynamic_geometry(self, t=0.0, params=None):
        # Lazy import: the world frame's name loads with the animation
        from minilink.graphical.animation.visualization import WORLD

        fan = latest_fan(self.futures, t)
        weights = fan["weights"] / fan["weights"].max()
        lines = [
            CustomLine(
                x_path[:, self.axes],
                color=(*self.cmap(0.35 + 0.65 * weight)[:3], 0.1 + 0.6 * weight),
                linewidth=0.6 + 1.2 * weight,
            )
            for x_path, weight in zip(fan["futures"], weights)
        ]
        lines.append(
            CustomLine(fan["plan"][:, self.axes], color="black", linewidth=2.0)
        )

        return {WORLD: lines}


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


def tick_solves_of(rollout, mpc, loop):
    """``[(t_k, P_k, z0_k)]`` of every solve of a sampled MPC loop's run: each tick's solve inputs read back through the loop, the solve not run."""
    # Lazy import: the sampled plan's layout loads with the first read
    from minilink.core.compile.plan import nested_layout

    require_loop(loop, "<the MPC's solve inputs>")
    ev = loop.compile()
    path, block = solving_block_of(loop, mpc.pv)
    divisor = ev.operation_of(path).divisor
    x_slice, _ = nested_layout(loop)[path]
    S = ev.plan.dense_points
    base = loop.sample_period
    t_start = float(rollout.t[0])
    n_ticks = (rollout.t.size - 1) // S

    solves = []
    for k in range(0, n_ticks, divisor):
        x = rollout.x[:, k * S]
        u = rollout.u[:, k * S]
        u_block = ev.block_input(path, x, u, k, t_start=t_start)
        t_k = t_start + k * base
        P, z0 = block.solve_inputs(x[x_slice], u_block, t_k)
        solves.append((t_k, P, z0))

    return solves


def solving_block_of(loop, pv):
    """``(path, block)`` of the block of ``loop`` that solves on ``pv``: the delayed MPC, the zero-delay MPC's solve, the pure MPC."""
    found = []
    for path, sys in blocks_of(loop):
        own = getattr(sys, "source", None) or sys
        if hasattr(own, "solve_inputs") and getattr(own, "pv", None) is pv:
            found.append((path, own))
    require_one_block([path for path, _ in found])

    return found[0]


def fan_plan(pv, fan) -> Trajectory:
    """A fan's plan as a trajectory: its rolled-out states and its held inputs, from the solve's origin."""
    t = fan["t"] + pv.dt * np.arange(fan["plan"].shape[0])
    u = np.concatenate([fan["inputs"], fan["inputs"][-1:]], axis=0)

    return Trajectory(t=t, x=fan["plan"].T, u=u.T)


def latest_fan(futures, t):
    """The fan of the last solve at or before ``t`` (the first one before it)."""
    solved = [fan for fan in futures if fan["t"] <= t + 1e-9]

    return solved[-1] if solved else futures[0]


def require_samples(pv):
    """Refuse a planner that draws no samples: the fan is a sampling planner's (MPPI)."""
    if not hasattr(pv, "samples"):
        raise TypeError(
            f"{type(pv).__name__} draws no samples: sampled futures are a sampling "
            "planner's (MPPIPlanner)"
        )


def plant_trail_of(rollout, loop):
    """The plant's states for the executed trail, with no block evaluated again (its role with no ``loop``)."""
    if loop is None:
        return rollout.of("plant")

    return Trajectory(t=rollout.t, x=plant_states_of(rollout, loop), u=rollout.u)


def plant_states_of(rollout, loop):
    """The plant's state slice of a sampled loop's run: the plan's plant, at any depth."""
    plant_op = loop.compile().plan.plant_op

    return rollout.x[plant_op.local_x_slice]


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
