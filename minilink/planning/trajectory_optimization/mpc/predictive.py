"""Model predictive control: the receding-horizon block a planner becomes."""

from __future__ import annotations

from minilink.planning.trajectory_optimization.mpc.controller import (
    MPCStatefulController,
    MPCStatelessController,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

###############################################################################
# Public API
###############################################################################


def ModelPredictiveController(
    planner: TrajectoryOptimizationPlanner,
    *,
    dt_mpc: float,
    warm_start: bool = True,
    verbose: bool = False,
    t0: float = 0.0,
    debug: bool = False,
):
    """
    Model predictive control: re-solve the planner's problem from the measured state every
    ``dt_mpc`` seconds and apply the first input of the plan.

    Close the loop as with any controller, ``mpc @ plant``.

    Parameters
    ----------
    planner : TrajectoryOptimizationPlanner
        The planner solved at each tick (its problem's start is the measured state).
    dt_mpc : float
        The replan period, in seconds.
    warm_start : bool, optional
        True: each solve starts from the previous plan, kept as the block's state (a
        :class:`~minilink.core.system.StepSystem`). False: every solve starts cold (a
        static :class:`~minilink.core.system.System`).
    verbose : bool, optional
        Print one line per solve.
    t0 : float, optional
        The time of the first tick.
    debug : bool, optional
        Redraw the plan in a live figure at each solve.
    """
    # Warm start: the last solve's packed z is the block's state
    if warm_start:
        return MPCStatefulController(
            planner,
            dt_mpc=dt_mpc,
            verbose=verbose,
            t0=t0,
            debug=debug,
        )

    # Cold start: an algebraic block, no solver state kept between ticks
    return MPCStatelessController(
        planner,
        dt_mpc=dt_mpc,
        verbose=verbose,
        t0=t0,
        debug=debug,
    )
