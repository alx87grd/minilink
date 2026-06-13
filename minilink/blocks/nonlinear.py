"""Static nonlinearity blocks.

Memoryless input-output maps used for actuator and sensor realism. Each is a
:class:`~minilink.core.system.StaticSystem` with the standard ``u``/``y`` ports
and JAX-traceable equations (``xp.clip`` / ``xp.where``), applied elementwise
across a signal of dimension ``dim``.

- :class:`Saturation` — clip to ``[lower, upper]``.
- :class:`DeadZone` — zero inside ``[-width, width]``, shifted outside.
- :class:`Relay` — bang-bang ``±amplitude`` on the sign of the input.

Stateful nonlinearities (rate limiter, hysteresis) are planned as small
``DynamicSystem`` blocks and live here too once added.
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import StaticSystem


class Saturation(StaticSystem):
    """Symmetric or asymmetric clip ``y = clip(u, lower, upper)``."""

    def __init__(self, lower=-1.0, upper=1.0, dim=1):
        super().__init__()
        self.name = "Saturation"
        self.dim = int(dim)
        self.params = {"lower": float(lower), "upper": float(upper)}

        self.add_input_port("u", dim=self.dim)
        self.add_output_port(
            "y", dim=self.dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        lower = params["lower"]
        upper = params["upper"]

        xp = array_module(u)
        return xp.clip(u, lower, upper)


class DeadZone(StaticSystem):
    """Dead zone of half-width ``width``: zero inside, shifted outside.

    ``y = u - width`` for ``u > width``, ``y = u + width`` for ``u < -width``,
    and ``y = 0`` in between — the standard backlash/stiction model.
    """

    def __init__(self, width=1.0, dim=1):
        super().__init__()
        self.name = "DeadZone"
        self.dim = int(dim)
        self.params = {"width": float(width)}

        self.add_input_port("u", dim=self.dim)
        self.add_output_port(
            "y", dim=self.dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        width = params["width"]

        xp = array_module(u)
        above = xp.where(u > width, u - width, 0.0)
        below = xp.where(u < -width, u + width, 0.0)
        return above + below


class Relay(StaticSystem):
    """Bang-bang relay ``y = amplitude · sign(u)`` (sign(0) = 0)."""

    def __init__(self, amplitude=1.0, dim=1):
        super().__init__()
        self.name = "Relay"
        self.dim = int(dim)
        self.params = {"amplitude": float(amplitude)}

        self.add_input_port("u", dim=self.dim)
        self.add_output_port(
            "y", dim=self.dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        amplitude = params["amplitude"]

        xp = array_module(u)
        return amplitude * xp.sign(u)


if __name__ == "__main__":
    grid = np.linspace(-2.0, 2.0, 9)
    print("input:     ", grid)
    print(
        "saturation:",
        np.array([Saturation(-1, 1).compute(None, np.array([v]))[0] for v in grid]),
    )
    print(
        "dead zone: ",
        np.array([DeadZone(0.5).compute(None, np.array([v]))[0] for v in grid]),
    )
    print(
        "relay:     ",
        np.array([Relay(2.0).compute(None, np.array([v]))[0] for v in grid]),
    )
