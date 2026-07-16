"""Algebraic broadcast leaf: high-rate ``u_nom`` / ``x_nom`` via get_nominal_*."""

from __future__ import annotations

from minilink.core.system import System


class MPCBroadcastController(System):
    """
    Sample latched nominals at Computer base ticks (no NLP).

    Absolute time is ``t0 + k * dt_broadcast`` where ``k`` is the Computer tick
    passed as the port ``t`` argument.

    Holds a reference to the replan MPC (shared ``NominalCache``) — not a
    port input. See dual-rate packaging option A in DESIGN / phase-E8.
    """

    def __init__(self, mpc, *, dt_broadcast: float, t0: float = 0.0) -> None:
        super().__init__()
        self.name = "MPC Broadcast"
        self._mpc = mpc
        self._dt_broadcast = float(dt_broadcast)
        self._t0 = float(t0)
        n = int(mpc._planner.problem.sys.n)
        m = int(mpc._planner.problem.sys.m)
        self.add_output_port("u_nom", dim=m, function=self._compute_u_nom)
        self.add_output_port("x_nom", dim=n, function=self._compute_x_nom)

    def _abs_t(self, k_tick) -> float:
        return float(self._t0 + float(k_tick) * self._dt_broadcast)

    def _compute_u_nom(self, x, u, t=0, params=None):
        del x, u, params
        return self._mpc.get_nominal_u(self._abs_t(t))

    def _compute_x_nom(self, x, u, t=0, params=None):
        del x, u, params
        return self._mpc.get_nominal_x(self._abs_t(t))
