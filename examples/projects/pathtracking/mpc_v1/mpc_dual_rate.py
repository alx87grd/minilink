"""Dual-rate MPC computer with lookahead on broadcast ``x_nom``.

Samples the latched plan at ``t + dt_project`` for servo references while
keeping ``u_nom`` at time ``t`` (logging only in bicycle_los_v3).
"""

from minilink.control.mpc.controller import MPCBroadcastController
from minilink.core.diagram import StepDiagramSystem
from minilink.simulation.computer import Computer, StepSchedule


class MpcBroadcastAhead(MPCBroadcastController):
    """Broadcast leaf that projects ``x_nom`` ahead by ``dt_project``."""

    def __init__(self, mpc, *, dt_broadcast, dt_project=0.25, t0=0.0):
        super().__init__(mpc, dt_broadcast=dt_broadcast, t0=t0)
        self.name = "MPC Broadcast Ahead"
        self._dt_project = float(dt_project)

    def _compute_x_nom(self, x, u, t=0, params=None):
        del x, u, params
        return self._mpc.get_nominal_x(self._abs_t(t) + self._dt_project)


def dual_rate_computer_ahead(block, *, dt_broadcast, dt_project=0.2):
    """Like ``mpc.dual_rate_computer``, but ``x_nom`` is sampled ahead.

    Same replan/broadcast schedule and latch wiring as core
    ``export_mpc_dual_rate_computer``; only the broadcast leaf differs.
    """
    dt_mpc = float(block.dt_mpc)
    dt_b = float(dt_broadcast)
    if dt_b <= 0.0:
        raise ValueError(f"dt_broadcast must be positive, got {dt_b}")
    ratio = dt_mpc / dt_b
    d = int(round(ratio))
    if d < 1 or abs(ratio - d) > 1e-9:
        raise ValueError(
            f"dt_mpc / dt_broadcast must be a positive integer "
            f"(got dt_mpc={dt_mpc}, dt_broadcast={dt_b}, ratio={ratio})"
        )

    block._replan_divisor = d
    block._latch.set_after_solve(
        lambda: block.generate_nominal_interpolator(derivatives=True)
    )

    broadcast = MpcBroadcastAhead(
        block,
        dt_broadcast=dt_b,
        dt_project=float(dt_project),
        t0=float(block.t0),
    )
    diagram = StepDiagramSystem()
    diagram.name = "MPC Dual-Rate Ahead"
    diagram.add_subsystem(block, "replan")
    diagram.add_subsystem(broadcast, "broadcast")

    y_port = block.inputs["y"]
    diagram.add_input_port("y", dim=y_port.dim, nominal_value=y_port.nominal_value)
    diagram.connect("input", "y", "replan", "y")
    diagram.connect_new_output_port("broadcast", "u_nom", "u_nom")
    diagram.connect_new_output_port("broadcast", "x_nom", "x_nom")
    for extra in ("u_ff", "x_ff", "z"):
        if extra in block.outputs:
            diagram.connect_new_output_port("replan", extra, extra)

    schedule = StepSchedule(
        dt_base=dt_b,
        fire={"replan": d, "broadcast": 1},
    )
    return Computer(diagram, schedule)
