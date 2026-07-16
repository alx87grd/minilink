"""
ModelPredictiveController — product factory and shared deploy / debug API.

Returns a Minilink ``System`` (algebraic) or ``StepSystem`` (warm-start) that
owns ``dt_mpc`` for warm-start shift, Computer schedule, and tick clock.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from minilink.planning.mpc.command import Command
from minilink.planning.results import SolveMetadata
from minilink.planning.trajectory_optimization.planner import (
    reject_unknown_online_params,
)

if TYPE_CHECKING:
    from minilink.planning.trajectory_optimization.planner import (
        TrajectoryOptimizationPlanner,
    )


class ModelPredictiveControllerMixin:
    """
    Shared deploy / export / debug surface for MPC System family blocks.

    Concrete classes provide ``_planner``, ``_latch``, ``_dt_mpc``, ``_t0``,
    ``_debug``, and optional warm-start via ``_z_warm_for_command``.
    """

    _planner: TrajectoryOptimizationPlanner
    _latch: object
    _dt_mpc: float
    _t0: float
    _debug: bool
    _deploy_k: int
    _last_command: Command | None
    _debug_handles: object | None
    _debug_sys: object | None

    @property
    def dt_mpc(self) -> float:
        """Replan period (warm-start shift and Computer schedule)."""
        return float(self._dt_mpc)

    @property
    def t0(self) -> float:
        """Absolute-time epoch: ``t_solve = t0 + k * dt_mpc``."""
        return float(self._t0)

    @property
    def last_command(self) -> Command | None:
        """Most recent :meth:`compute_command` result, if any."""
        return self._last_command

    def get_solve_metadata(self) -> SolveMetadata | None:
        """
        Last NLP solve extras for deploy / logging (ROS-agnostic).

        Prefers :attr:`last_command` metadata; otherwise the planner's
        :attr:`~minilink.planning.planner.Planner.last_trajectory_plan`
        (e.g. after a hybrid tick with no deploy call). ``None`` if nothing
        has been solved yet.
        """
        if self._last_command is not None:
            return self._last_command.metadata
        plan = self._planner.last_trajectory_plan
        if plan is not None:
            return plan.metadata
        return None

    def reset(self) -> None:
        """Clear deploy counter, last command, and tick latch memo."""
        self._deploy_k = 0
        self._last_command = None
        self._latch.reset_latch()

    def _z_warm_for_command(self):
        """Override on StepSystem: previous packed ``z`` for warm-start."""
        return None

    def compute_command(
        self,
        y,
        *,
        params=None,
        k: int | None = None,
        t: float | None = None,
    ) -> Command:
        """
        Deploy replan tick: measure ``y`` → NLP → :class:`Command`.

        Parameters
        ----------
        y : array_like
            Measured plant state (diagram input ``y``).
        params : mapping, optional
            Passed to
            :meth:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner.solve_trajectory_from`
            (rejected until E7 if non-empty keys).
        k : int, optional
            Tick index. Default: increment internal deploy counter.
        t : float, optional
            Absolute solve time. Default: ``t0 + k * dt_mpc``.
        """
        if k is None:
            k_int = int(self._deploy_k)
            self._deploy_k = k_int + 1
        else:
            k_int = int(k)

        if t is None:
            t_solve = float(self._t0 + k_int * self._dt_mpc)
        else:
            t_solve = float(t)

        reject_unknown_online_params(params)
        y_arr = np.asarray(y, dtype=float).reshape(-1)
        z_warm = self._z_warm_for_command()
        tick = self._latch.solve_for_tick(
            k_int,
            y_arr,
            z_warm=z_warm,
            dt_mpc=self._dt_mpc if z_warm is not None else None,
        )
        plan = self._planner.require_trajectory_plan()
        cmd = Command(
            plan=plan,
            k=k_int,
            t_solve=t_solve,
            u_ff=np.asarray(tick.u_ff, dtype=float).copy(),
            x_ff=np.asarray(tick.x_ff, dtype=float).copy(),
            z=np.asarray(tick.z, dtype=float).copy(),
            success=bool(plan.metadata.success),
        )
        self._last_command = cmd
        if self._debug:
            self.update_debug_figure(cmd)
        return cmd

    def export_to_computer(self, schedule=None):
        """Build a single-rate :class:`~minilink.simulation.computer.Computer`."""
        from minilink.planning.mpc._block_common import export_mpc_to_computer

        return export_mpc_to_computer(self, schedule, dt_mpc=self._dt_mpc)

    def __matmul__(self, plant):
        """``mpc @ plant`` → hybrid via ``export_to_computer() @ plant``."""
        return self.export_to_computer() @ plant

    def init_debug_figure(self, sys, **kwargs):
        """
        Create a matplotlib figure for live plan visualization.

        Under a non-interactive backend (e.g. ``Agg``), still builds axes so
        tests can smoke the path; ``update_debug_figure`` redraws plan knots.
        """
        del kwargs
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1, 1, figsize=(6, 4))
        ax.set_xlabel("plan-local time τ")
        ax.set_ylabel("state / input")
        ax.set_title("MPC debug: latched plan")
        self._debug_handles = {"fig": fig, "ax": ax, "lines": []}
        self._debug_sys = sys
        return fig

    def update_debug_figure(self, cmd: Command | None = None):
        """Redraw debug axes from ``cmd`` or :attr:`last_command`."""
        if self._debug_handles is None:
            return None
        use = cmd if cmd is not None else self._last_command
        if use is None:
            return self._debug_handles["fig"]

        ax = self._debug_handles["ax"]
        ax.cla()
        ax.set_xlabel("plan-local time τ")
        ax.set_ylabel("state / input")
        ax.set_title(f"MPC debug k={use.k} t_solve={use.t_solve:.3g}")
        traj = use.plan.trajectory
        for i in range(int(traj.x.shape[0])):
            ax.plot(traj.t, traj.x[i], label=f"x[{i}]")
        for i in range(int(traj.u.shape[0])):
            ax.plot(traj.t, traj.u[i], "--", label=f"u[{i}]")
        ax.legend(loc="best", fontsize=8)
        fig = self._debug_handles["fig"]
        fig.canvas.draw_idle()
        return fig


def ModelPredictiveController(
    planner: TrajectoryOptimizationPlanner,
    *,
    dt_mpc: float,
    warm_start: bool = True,
    step_disp: bool = False,
    t0: float = 0.0,
    debug: bool = False,
):
    """
    Build the product MPC System family block.

    Parameters
    ----------
    planner : TrajectoryOptimizationPlanner
        Trajopt planner (auto-compiles parametric NLP when needed).
    dt_mpc : float
        Replan period (required for both warm-start modes).
    warm_start : bool, optional
        If True, return a :class:`~minilink.core.system.StepSystem` with packed
        ``z`` on ``Computer.x``. If False, algebraic :class:`~minilink.core.system.System`.
    step_disp, t0, debug
        Latch printouts, absolute-time epoch, and live debug figure auto-update.
    """
    if warm_start:
        from minilink.planning.mpc.step_block import MPCStatefulController

        return MPCStatefulController(
            planner,
            dt_mpc=dt_mpc,
            step_disp=step_disp,
            t0=t0,
            debug=debug,
        )
    from minilink.planning.mpc.controller import MPCStatelessController

    return MPCStatelessController(
        planner,
        dt_mpc=dt_mpc,
        step_disp=step_disp,
        t0=t0,
        debug=debug,
    )
