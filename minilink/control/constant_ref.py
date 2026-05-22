import numpy as np

from minilink.core.system import System


class ConstantReference(System):
    """Constant scalar reference signal."""

    def __init__(self, ref: float = 1.0, name: str | None = None):
        super().__init__(0, 0, 1)

        self.name = name if name is not None else "Not named :("

        self.inputs = {}
        self.outputs = {}
        self.recompute_input_properties()

        self.ref = float(ref)

        self.add_output_port(
            1,
            "ref",
            function=self.h_ref,
            dependencies=[],
        )

    def h_ref(self, x, u, t=0.0, params=None):
        return np.array([self.ref], dtype=float)
