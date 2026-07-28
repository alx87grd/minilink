"""Slice MPC nominal state into cascade references."""

import numpy as np

from minilink.core.system import System


class NominalRefs(System):
    """``x_nom`` (8-D MPC state) → ``heading_ref``, ``vx_ref``, ``delta_ff``."""

    def __init__(self, name: str = "Nominal refs"):
        super().__init__(0)
        self.name = name
        self.add_input_port("x_nom", nominal_value=np.zeros(8))
        self.add_output_port(
            "heading_ref",
            dim=1,
            function=self.h_heading_ref,
            dependencies=("x_nom",),
        )
        self.add_output_port(
            "vx_ref",
            dim=1,
            function=self.h_vx_ref,
            dependencies=("x_nom",),
        )
        self.add_output_port(
            "delta_ff",
            dim=1,
            function=self.h_delta_ff,
            dependencies=("x_nom",),
        )

    def h_heading_ref(self, x, u, t=0.0, params=None):
        return np.array([u[2]], dtype=float)

    def h_vx_ref(self, x, u, t=0.0, params=None):
        return np.array([u[3]], dtype=float)

    def h_delta_ff(self, x, u, t=0.0, params=None):
        return np.array([u[7]], dtype=float)
