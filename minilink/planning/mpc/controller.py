"""Algebraic (no warm-start state) MPC controller — ModelPredictiveController family."""

from __future__ import annotations

import numpy as np

from minilink.core.system import System
from minilink.planning.mpc._block_common import validate_mpc_planner
from minilink.planning.mpc.model_predictive_controller import (
    ModelPredictiveControllerMixin,
)
from minilink.planning.mpc.planner import MPCPlanner
from minilink.planning.mpc.tick_latch import MPCTickLatch


class MPCStatelessController(ModelPredictiveControllerMixin, System):
    """
    Algebraic MPC feedforward block (``warm_start=False`` product sibling).

    Owns ``dt_mpc`` for Computer schedule / ``@``. Outputs ``u_ff``, ``x_ff``,
    and ``z`` come from one NLP per integer tick ``k`` (latch-memoized).
    """

    def __init__(
        self,
        planner: MPCPlanner,
        *,
        dt_mpc: float,
        step_disp: bool = False,
        t0: float = 0.0,
        debug: bool = False,
    ) -> None:
        n, m, n_z = validate_mpc_planner(planner)

        super().__init__()
        self.name = "MPC Stateless Controller"
        self._planner = planner
        self._dt_mpc = float(dt_mpc)
        self._t0 = float(t0)
        self._debug = bool(debug)
        self._deploy_k = 0
        self._last_command = None
        self._debug_handles = None
        self._debug_sys = None
        self._latch = MPCTickLatch(
            planner,
            step_disp=step_disp,
            dt_mpc=self._dt_mpc,
            t0=self._t0,
        )

        self.add_input_port("y", dim=n)
        self.add_output_port(
            "u_ff",
            dim=m,
            function=self._compute_u_ff,
            dependencies=("y",),
        )
        self.add_output_port(
            "x_ff",
            dim=n,
            function=self._compute_x_ff,
            dependencies=("y",),
        )
        self.add_output_port(
            "z",
            dim=n_z,
            function=self._compute_z,
            dependencies=("y",),
        )

    @property
    def planner(self) -> MPCPlanner:
        """Prepared compile-once planner."""
        return self._planner

    def _measurement(self, u) -> np.ndarray:
        return np.asarray(u, dtype=float).reshape(self.inputs["y"].dim)

    def _compute_u_ff(self, x, u, t=0, params=None):
        del x, params
        return self._latch.solve_for_tick(t, self._measurement(u)).u_ff

    def _compute_x_ff(self, x, u, t=0, params=None):
        del x, params
        return self._latch.solve_for_tick(t, self._measurement(u)).x_ff

    def _compute_z(self, x, u, t=0, params=None):
        del x, params
        return self._latch.solve_for_tick(t, self._measurement(u)).z


def mpc_stateless_controller(
    planner: MPCPlanner,
    *,
    dt_mpc: float,
    step_disp: bool = False,
    t0: float = 0.0,
    debug: bool = False,
) -> MPCStatelessController:
    """Build algebraic MPC block (alias of ``ModelPredictiveController(..., warm_start=False)``)."""
    return MPCStatelessController(
        planner, dt_mpc=dt_mpc, step_disp=step_disp, t0=t0, debug=debug
    )
