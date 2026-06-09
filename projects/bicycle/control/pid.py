"""Project-local PID variant with diagnostic output ports.

The core :class:`minilink.control.pid.PID` exposes only the command output.
The bicycle simulations also plot controller internals, so this subclass adds
two diagnostic ports on top of the core block:

- ``logs`` -> ``[ref, meas]``
- ``pid_int_value`` -> ``[error, d_meas, int_e]``

This is project instrumentation, not a core feature.
"""

import numpy as np

from minilink.control.pid import PID


class InstrumentedPID(PID):
    """Core :class:`PID` plus ``logs`` and ``pid_int_value`` diagnostic ports."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.add_output_port(
            "logs",
            dim=2,
            function=self.h_logs,
            dependencies=["ref", "meas"],
            labels=["ref", "meas"],
        )
        self.add_output_port(
            "pid_int_value",
            dim=3,
            function=self.h_internals,
            dependencies=[],
            labels=["error", "d_meas", "int_e"],
        )

    def h_logs(self, x, u, t=0.0, params=None):
        ref, meas = self.get_port_values_from_u(u, "ref", "meas")
        return np.array([ref[0], meas[0]])

    def h_internals(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = x[0], x[1]
        ref, meas = self.get_port_values_from_u(u, "ref", "meas")

        e, d_meas_filt, _ = self.control_law(int_e, meas_filt, ref[0], meas[0], p)
        return np.array([e, d_meas_filt, int_e])
