"""Map EngineBicycle plant output to MPC bicycle state."""

import numpy as np

from minilink.core.system import System


def plant_y_to_mpc_x(y):
    """Pack plant ``y`` (10) into MPC ``x`` (8).

    Uses ``delta_act`` as the MPC steer state (index 7).
    """
    y = np.asarray(y, dtype=float).reshape(10)
    return np.array(
        [y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[9]],
        dtype=float,
    )


class PlantToMpcState(System):
    """``y`` (10-D plant output) → ``y_mpc`` (8-D MPC measurement)."""

    def __init__(self, name: str = "Plant to MPC state"):
        super().__init__(0)
        self.name = name
        self.add_input_port("y", nominal_value=np.zeros(10))
        self.add_output_port(
            "y_mpc",
            dim=8,
            function=self.h_y_mpc,
            dependencies=("y",),
        )

    def h_y_mpc(self, x, u, t=0.0, params=None):
        return plant_y_to_mpc_x(u)
