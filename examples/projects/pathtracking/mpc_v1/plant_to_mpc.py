"""Plant to MPC state: the EngineBicycle output as the MPC bicycle's state."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


def plant_y_to_mpc_x(y):
    """Pack plant ``y`` (10) into MPC ``x`` (8).

    Uses ``delta_act`` as the MPC steer state (index 7).
    """
    xp = array_module(y)

    y = xp.reshape(y, (10,))

    # x_mpc = [X, Y, θ, vx, vy, ψ̇, ω_rear, δ_act]: y without ω_front and τ_engine
    x_mpc = xp.concatenate([y[0:7], y[9:10]])

    return x_mpc


class PlantToMpcState(System):
    """``y`` (10-D plant output) → ``y_mpc`` (8-D MPC measurement)."""

    def __init__(self, name: str = "Plant to MPC state"):
        super().__init__(0)
        self.name = name

        # Ports: the plant output in, the MPC's measured state out
        self.add_input_port("y", nominal_value=np.zeros(10))
        self.add_output_port(
            "y_mpc",
            dim=8,
            function=self.h_y_mpc,
            dependencies=("y",),
        )

    def h_y_mpc(self, x, u, t=0.0, params=None):
        return plant_y_to_mpc_x(u)
