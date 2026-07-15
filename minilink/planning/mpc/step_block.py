"""Warm-start StepSystem MPC controller — ModelPredictiveController family."""

from __future__ import annotations

import numpy as np

from minilink.core.system import StepSystem
from minilink.planning.mpc._block_common import validate_mpc_planner
from minilink.planning.mpc.model_predictive_controller import (
    ModelPredictiveControllerMixin,
)
from minilink.planning.mpc.tick_latch import MPCTickLatch
from minilink.planning.mpc.warm_start import mpc_default_computer_x0
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)


class MPCStatefulController(ModelPredictiveControllerMixin, StepSystem):
    """
    Warm-start MPC block with packed optimizer state on ``Computer.x``.

    ``warm_start=True`` product sibling. State ``x`` is packed ``z`` from the
    previous tick. Ports and :meth:`step` share one NLP per tick via the latch.
    """

    def __init__(
        self,
        planner: TrajectoryOptimizationPlanner,
        *,
        dt_mpc: float,
        step_disp: bool = False,
        t0: float = 0.0,
        debug: bool = False,
    ) -> None:
        n, m, n_z = validate_mpc_planner(planner)
        super().__init__(n_z, expose_state=False)
        self.name = "MPC Stateful Controller"
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
        self.x0 = mpc_default_computer_x0(planner)

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
    def planner(self) -> TrajectoryOptimizationPlanner:
        """Compiled trajopt planner (parametric NLP)."""
        return self._planner

    @property
    def latch(self) -> MPCTickLatch:
        """Per-tick solve memo (reset via :meth:`MPCTickLatch.reset_latch`)."""
        return self._latch

    def _z_warm_for_command(self):
        if self._last_command is not None:
            return self._last_command.z
        return None

    def _measurement(self, u) -> np.ndarray:
        return np.asarray(u, dtype=float).reshape(self.inputs["y"].dim)

    def _compute_u_ff(self, x, u, t=0, params=None):
        del params
        return self._latch.solve_for_tick(
            t,
            self._measurement(u),
            z_warm=x,
            dt_mpc=self._dt_mpc,
        ).u_ff

    def _compute_x_ff(self, x, u, t=0, params=None):
        del params
        return self._latch.solve_for_tick(
            t,
            self._measurement(u),
            z_warm=x,
            dt_mpc=self._dt_mpc,
        ).x_ff

    def _compute_z(self, x, u, t=0, params=None):
        del params
        return self._latch.solve_for_tick(
            t,
            self._measurement(u),
            z_warm=x,
            dt_mpc=self._dt_mpc,
        ).z

    def step(self, x, u, k=0, params=None):
        del params
        y = self._measurement(u)
        z_new = self._latch.solve_for_tick(
            k,
            y,
            z_warm=x,
            dt_mpc=self._dt_mpc,
        ).z
        return np.asarray(z_new, dtype=float).reshape(self.n).copy()
