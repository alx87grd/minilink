"""Stateless algebraic MPC controller for step / hybrid simulation (Phase 6a)."""

from __future__ import annotations

import numpy as np

from minilink.core.system import System
from minilink.planning.mpc._block_common import (
    export_mpc_to_computer,
    validate_mpc_planner,
)
from minilink.planning.mpc.planner import MPCPlanner
from minilink.planning.mpc.tick_latch import MPCTickLatch


class MPCStatelessController(System):
    """
    Stateless MPC feedforward block for step diagrams and hybrid simulation.

    Diagram input ``y`` is the plant measurement (typically full state). Outputs
    ``u_ff``, ``x_ff``, and ``z`` come from one :meth:`~MPCPlanner.step` per
    integer tick ``k`` (Computer fire index), memoized on the block.
    """

    def __init__(self, planner: MPCPlanner, *, step_disp: bool = False) -> None:
        n, m, n_z = validate_mpc_planner(planner)

        super().__init__()
        self.name = "MPC Stateless Controller"
        self._planner = planner
        self._latch = MPCTickLatch(planner, step_disp=step_disp)

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

    def export_to_computer(self, schedule=None):
        """Return a :class:`~minilink.simulation.computer.Computer` scheduled at ``schedule``."""
        return export_mpc_to_computer(self, schedule)

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
    planner: MPCPlanner, *, step_disp: bool = False
) -> MPCStatelessController:
    """Build a stateless :class:`MPCStatelessController` from a prepared :class:`MPCPlanner`."""
    return MPCStatelessController(planner, step_disp=step_disp)
