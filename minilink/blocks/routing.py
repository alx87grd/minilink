"""Signal routing and combination blocks.

Plant-agnostic plumbing for multi-signal diagrams. Each block is a
:class:`~minilink.core.system.System` with explicit named ports whose
dimensions are fixed at construction, so diagram wiring is validated at connect
time.

- :class:`Error` — tracking-error comparison ``e = r - y`` on ports ``+``, ``-``, ``e``.
- :class:`Sum` — signed junction ``y = Σ sign_i · in_i``.
- :class:`Gain` — constant gain ``y = K · u``.
- :class:`Mux` — stack several input signals into one vector output.
- :class:`Demux` — split one vector into NumPy-style slices (``y[0]``, ``y[1]``, …).
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import System


class Error(System):
    """Tracking-error comparison ``e = r - y`` on ports ``+``, ``-``, ``e``.

    The automatic feedback pattern (``C @ plant``, ``sys @ 1``) inserts this
    block so the diagram reads as a comparison, not a generic summer. All
    three ports share the same dimension ``dim``.
    """

    def __init__(self, dim=1):
        super().__init__()
        self.name = "Error"
        self.dim = int(dim)

        self.add_input_port("+", dim=self.dim)
        self.add_input_port("-", dim=self.dim)
        self.add_output_port(
            "e", dim=self.dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        r, y = u.reshape(2, self.dim)
        return r - y


class Sum(System):
    """Signed summing junction ``y = Σ sign_i · in_i`` on ports ``in0, in1, …``.

    The default ``signs=(1.0, -1.0)`` is a two-input difference
    ``y = in0 - in1``. Use :class:`Error` for the named tracking-error
    comparison that ``@`` inserts. All input ports share the same dimension
    ``dim``.

    ``signs`` is a structural attribute fixed at construction (it sets the
    number of input ports), deliberately not a tunable ``params`` entry.
    """

    def __init__(self, signs=(1.0, -1.0), dim=1):
        super().__init__()
        self.name = "Sum"
        self.signs = np.asarray(signs, dtype=float).reshape(-1)
        self.dim = int(dim)

        for i in range(self.signs.size):
            self.add_input_port(f"in{i}", dim=self.dim)
        self.add_output_port(
            "y", dim=self.dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        xp = array_module(u)
        signs = xp.asarray(self.signs)

        # weighted sum over the stacked input ports
        stacked = u.reshape(self.signs.size, self.dim)
        return signs @ stacked


class Gain(System):
    """Constant gain ``y = K · u``.

    ``K`` may be a full ``(p, m)`` matrix, a 1-D vector (diagonal gain), or a
    scalar (needs ``dim`` to set the port size). The gain lives in
    ``params["K"]`` so it can be tuned or differentiated.
    """

    def __init__(self, K, dim=None):
        super().__init__()
        self.name = "Gain"

        K = np.asarray(K, dtype=float)
        if K.ndim == 0:
            if dim is None:
                raise ValueError("scalar gain needs an explicit dim")
            K = float(K) * np.eye(int(dim))
        elif K.ndim == 1:
            K = np.diag(K)
        elif K.ndim != 2:
            raise ValueError("K must be scalar, 1-D, or 2-D")

        self.params = {"K": K}
        p, m = K.shape
        self.add_input_port("u", dim=m)
        self.add_output_port("y", dim=p, function=self.compute, dependencies="all")

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        K = params["K"]

        return K @ u


class Mux(System):
    """Stack input signals ``in0, in1, …`` into one vector output ``y``.

    ``dims`` lists each input-port dimension; the output dimension is their sum.
    The combined signal is just the concatenation in port order, so the compute
    is the identity on the stacked input vector.
    """

    def __init__(self, dims=(1, 1)):
        super().__init__()
        self.name = "Mux"
        self.dims = [int(d) for d in dims]

        for i, d in enumerate(self.dims):
            self.add_input_port(f"in{i}", dim=d)
        self.add_output_port(
            "y", dim=sum(self.dims), function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        return u


class Demux(System):
    """Split one vector into NumPy-style slices of a named signal.

    ``dims`` lists each output-port dimension; the input dimension is their
    sum. The input port is ``port`` (default ``u``); each output is the
    corresponding slice, ``u[0]``, ``u[1]``, or ``u[0:2]`` for a wider
    block. Auto-wired feedback passes the plant port name so a ``y``
    measurement becomes ``y`` in and ``y[0]``, ``y[1]``, … out.
    """

    def __init__(self, dims=(1, 1), *, port="u"):
        super().__init__()
        self.name = "Demux"
        self.dims = [int(d) for d in dims]
        self.port = str(port)

        self.add_input_port(self.port, dim=sum(self.dims))
        start = 0
        for d in self.dims:
            port_id = (
                f"{self.port}[{start}]"
                if d == 1
                else f"{self.port}[{start}:{start + d}]"
            )
            self.add_output_port(
                port_id,
                dim=d,
                function=self._slice(slice(start, start + d)),
                dependencies=(self.port,),
            )
            start += d

    def _slice(self, port_slice):
        """Build the compute for one output port: return that slice of ``u``."""

        def compute(x, u, t=0, params=None):
            return u[port_slice]

        return compute


if __name__ == "__main__":
    # Hello world: error comparison, gain, and a Mux/Demux round trip.
    error = Error()
    print("Error:", error.compute(None, np.array([5.0, 2.0])))  # 3.0

    gain = Gain([2.0, 3.0])
    print("Gain:", gain.compute(None, np.array([1.0, 1.0])))  # [2, 3]

    split = Demux(dims=(2, 1))
    print("Demux u[2]:", split.outputs["u[2]"].compute(None, np.array([1.0, 2.0, 9.0])))
