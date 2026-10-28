"""Dual-rate MPC computer with lookahead on the broadcast ``x_nom``.

The slow MPC replans; the fast follower reads the received plan's state at
``t + dt_project`` for the servo references (logging only in bicycle_los_v3).
"""

from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class PlanAhead(System):
    """The received plan's state read ahead: ``x_nom = x_plan(t + dt_project)``."""

    def __init__(self, planner, *, dt_project=0.2):
        super().__init__(0)
        self.name = "Plan ahead"
        self.pv = planner.compile()
        self.dt_project = float(dt_project)

        self.add_input_port("z", dim=self.pv.n_z)
        self.add_output_port(
            "x_nom", dim=self.pv.n, function=self.compute_x_nom, dependencies=("z",)
        )

    def compute_x_nom(self, x, u, t=0, params=None):
        """``x_nom(t) = x_plan(t + dt_project)``, the plan's state knots read ahead."""
        x_nom, _ = self.pv.prediction(u, t + self.dt_project)
        return x_nom


def dual_rate_computer_ahead(mpc, planner, *, dt_mpc, dt_broadcast, dt_project=0.2):
    """The MPC replanning every ``dt_mpc``, its plan read ``dt_project`` ahead every ``dt_broadcast``."""
    follower = PlanAhead(planner, dt_project=dt_project)

    return (mpc % dt_mpc) >> (follower % dt_broadcast)
