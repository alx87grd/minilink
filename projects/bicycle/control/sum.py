"""Project-local two-input summing junction (actuator mixing).

Used by the bicycle sims to add a feedforward steering map to the yaw-rate PID
output. Kept in the project since core's PID already provides the only generic
difference junction needed for control.
"""

import numpy as np

from minilink.core.system import System


class Sum(System):
    """Add two scalar inputs, with optional output saturation."""

    def __init__(self, max=None, min=None, name: str = "Sum"):
        super().__init__(0)

        self.name = name
        self.max = max
        self.min = min

        self.inputs = {}
        self.add_input_port("1", nominal_value=np.array([0.0]))
        self.add_input_port("2", nominal_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port(
            "result",
            dim=1,
            function=self.h_sum,
            dependencies=["1", "2"],
            labels=["result"],
        )

    def h_sum(self, x, u, t=0.0, params=None):
        a, b = self.get_port_values_from_u(u, "1", "2")
        result = a[0] + b[0]

        if self.max is not None and self.min is not None:
            result = np.clip(result, self.min, self.max)

        return np.array([result])
