"""Scalar measurement (index selector) block for the bicycle project.

This is a project-local block, not a core minilink feature. It selects a single
component out of a system's monolithic output vector ``y`` and exposes it as a
scalar ``meas`` port (used to feed scalar PID loops here).

TODO: User Architectural Review. If this kind of vector tap recurs across
projects, promote a generic, label-aware ``Selector`` block to core (selecting
by label and carrying through the source port's labels/units) rather than this
fixed-index helper. For models you control, prefer declaring an explicit scalar
output port on the system instead of slicing it downstream.
"""

import numpy as np

from minilink.core.system import System


class Measurement(System):
    """Expose component ``index`` of the input vector ``y`` as a scalar.

    Parameters
    ----------
    name : str
        Display name of the block.
    y_size : int
        Dimension of the incoming signal vector.
    index : int
        Component of the input vector to expose as the measurement.
    show : bool
        When True, print the measured value at each evaluation (debug aid).
    """

    def __init__(self, name: str, y_size: int = 12, index: int = 5, show: bool = False):
        super().__init__(0)

        self.name = name
        self.show = show

        self.y_size = int(y_size)
        self.index = int(index)

        self.inputs = {}
        self.add_input_port("y", nominal_value=np.zeros(self.y_size))

        self.outputs = {}
        self.add_output_port(
            "meas",
            dim=1,
            function=self.h_meas,
            dependencies=["y"],
            labels=["meas"],
        )

    def h_meas(self, x, u, t=0.0, params=None):
        y = self.get_port_values_from_u(u, "y")
        if self.show:
            print(f"Meas: {y[self.index]}")
        return y[self.index : self.index + 1]
