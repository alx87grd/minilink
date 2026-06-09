"""Generic measurement (sensor) blocks."""

import numpy as np

from minilink.core.system import System


class Measurement(System):
    """Select a single component of an input vector as a scalar measurement.

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
        )

    def h_meas(self, x, u, t=0.0, params=None):
        if self.show:
            print(f"Meas: {u[self.index]}")
        return np.array([u[self.index]], dtype=float)
