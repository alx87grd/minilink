"""Reference-signal source blocks."""

import numpy as np

from minilink.core.system import System


class ConstantReference(System):
    """Emit a constant scalar reference signal.

    Parameters
    ----------
    ref : float
        Constant value emitted on the ``ref`` output port.
    name : str, optional
        Display name of the block.
    """

    def __init__(self, ref: float = 1.0, name: str | None = None):
        super().__init__(0)

        self.name = name if name is not None else "ConstantReference"

        self.inputs = {}
        self.outputs = {}
        self.recompute_input_properties()

        self.ref = float(ref)

        self.add_output_port(
            "ref",
            dim=1,
            function=self.h_ref,
            dependencies=[],
        )

    def h_ref(self, x, u, t=0.0, params=None):
        return np.array([self.ref], dtype=float)
