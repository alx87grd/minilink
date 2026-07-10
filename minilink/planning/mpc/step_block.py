"""Warm-start MPC step block for hybrid simulation (Phase 6b)."""

from __future__ import annotations

import numpy as np

from minilink.core.system import StepSystem
from minilink.planning.mpc._block_common import (
    export_mpc_to_computer,
    validate_mpc_planner,
)
from minilink.planning.mpc.planner import MPCPlanner
from minilink.planning.mpc.tick_latch import MPCTickLatch
from minilink.planning.mpc.warm_start import mpc_default_computer_x0


class MPCStepBlock(StepSystem):
    """
    Warm-start MPC block with packed optimizer state on ``Computer.x``.

    Diagram input ``y`` is the plant measurement. State ``x`` is the packed
    decision vector ``z`` from the previous tick. Outputs ``u_ff``, ``x_ff``,
    and ``z`` come from one :meth:`~MPCPlanner.step` per tick (memoized on the
    block); :meth:`step` commits the latched ``z`` without a second NLP.
    """

    def __init__(
        self,
        planner: MPCPlanner,
        *,
        dt_mpc: float,
        step_disp: bool = False,
        t0: float = 0.0,
    ) -> None:
        n, m, n_z = validate_mpc_planner(planner)
        super().__init__(n_z, expose_state=False)
        self.name = "MPC Step Block"
        self._planner = planner
        self._latch = MPCTickLatch(
            planner,
            step_disp=step_disp,
            dt_mpc=dt_mpc,
            t0=t0,
        )
        self._dt_mpc = float(dt_mpc)
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
    def planner(self) -> MPCPlanner:
        """Prepared compile-once planner."""
        return self._planner

    @property
    def latch(self) -> MPCTickLatch:
        """Per-tick solve memo (reset via :meth:`MPCTickLatch.reset_latch`)."""
        return self._latch

    def export_to_computer(self, schedule=None):
        """Return a :class:`~minilink.simulation.computer.Computer` using block ``dt_mpc``."""
        return export_mpc_to_computer(self, schedule, dt_mpc=self._dt_mpc)

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


def mpc_step_block(
    planner: MPCPlanner,
    *,
    dt_mpc: float,
    step_disp: bool = False,
    t0: float = 0.0,
) -> MPCStepBlock:
    """Build a warm-start :class:`MPCStepBlock` from a prepared :class:`MPCPlanner`."""
    return MPCStepBlock(planner, dt_mpc=dt_mpc, step_disp=step_disp, t0=t0)
