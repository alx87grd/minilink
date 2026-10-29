"""Cost shaping: the penalty of a scalar field value, before a field's cost weights it."""

from abc import ABC, abstractmethod

from minilink.core.backends import array_module
from minilink.core.params import own_params

# The field stays a signed physical quantity (clearance in length units); shaping
# at the edge, before :meth:`~minilink.core.fields.Field.as_cost` weights it, turns
# it into a barrier, a one-sided hinge, or a bounded occupancy score, so an
# obstacle term balances cleanly against other costs such as quadratic tracking.
#
# Each factory returns a native-array :class:`Shaping`, ``shape(v, params) -> penalty``,
# for ``field.as_cost(shaping=...)`` and ``set_.as_cost(shaping=...)``; its number
# (a threshold, a scale) is its ``params``, a leaf of the cost's tree under
# ``"shaping"``. The shapings stay NumPy under NumPy input and trace under JAX.


###############################################################################
# Public API
###############################################################################


class Shaping(ABC):
    """A penalty of a scalar field value, ``shape(v, params)``; ``params`` its numbers (``params=None`` reads them)."""

    params = own_params()

    @abstractmethod
    def __call__(self, v, params=None):
        """The penalty of the field value ``v``."""


def quadratic_hinge(threshold: float = 0.0):
    """
    One-sided quadratic penalty below ``threshold``: ``max(threshold - v, 0)**2``.

    Zero in free space and quadratic once the value drops past ``threshold`` —
    the same scale as a quadratic tracking cost, so their weights are directly
    comparable, and the gradient keeps growing with penetration.
    """
    return QuadraticHinge(threshold)


def quadratic_excess(threshold: float = 0.0):
    """
    One-sided quadratic penalty above ``threshold``: ``max(v - threshold, 0)**2``.

    Use with :class:`~minilink.planning.spatial.state_fields.PathDistanceField`
    to penalize deviation from a reference path (mirror of :func:`quadratic_hinge`
    used for clearance margins).
    """
    return QuadraticExcess(threshold)


def inverse_barrier(epsilon: float = 1e-2):
    """
    Repulsive barrier ``1 / max(v, epsilon)**2`` that blows up at contact.

    Strong, unbounded repulsion near the boundary; ``epsilon`` caps the value
    once the clearance is exhausted so the penalty stays finite.
    """
    return InverseBarrier(epsilon)


def occupancy(scale: float = 1.0):
    """
    Bounded occupancy score in ``[0, 1]``: ``sigmoid(-v / scale)``.

    ``~0`` well outside, ``0.5`` on the boundary, ``~1`` deep inside. Bounded,
    so weighting an obstacle term against an unbounded cost is interpretable —
    at the price of a gradient that saturates far from the boundary.
    """
    return Occupancy(scale)


class QuadraticHinge(Shaping):
    """``max(threshold − v, 0)²``; ``params`` is ``{"threshold"}``."""

    def __init__(self, threshold: float = 0.0):
        self.params = {"threshold": float(threshold)}

    def __call__(self, v, params=None):
        params = self.params if params is None else params
        xp = array_module(v)

        threshold = params["threshold"]

        penalty = xp.maximum(threshold - v, 0.0) ** 2

        return penalty


class QuadraticExcess(Shaping):
    """``max(v − threshold, 0)²``; ``params`` is ``{"threshold"}``."""

    def __init__(self, threshold: float = 0.0):
        self.params = {"threshold": float(threshold)}

    def __call__(self, v, params=None):
        params = self.params if params is None else params
        xp = array_module(v)

        threshold = params["threshold"]

        penalty = xp.maximum(v - threshold, 0.0) ** 2

        return penalty


class InverseBarrier(Shaping):
    """``1 / max(v, ε)²``; ``params`` is ``{"epsilon"}``."""

    def __init__(self, epsilon: float = 1e-2):
        self.params = {"epsilon": float(epsilon)}

    def __call__(self, v, params=None):
        params = self.params if params is None else params
        xp = array_module(v)

        epsilon = params["epsilon"]

        barrier = 1.0 / xp.maximum(v, epsilon) ** 2

        return barrier


class Occupancy(Shaping):
    """``σ(−v / s)``; ``params`` is ``{"scale"}``."""

    def __init__(self, scale: float = 1.0):
        self.params = {"scale": float(scale)}

    def __call__(self, v, params=None):
        params = self.params if params is None else params
        xp = array_module(v)

        scale = params["scale"]

        # σ(−v / s) = 1 / (1 + e^{v/s})
        occupancy = 1.0 / (1.0 + xp.exp(v / scale))

        return occupancy


class CallableShaping(Shaping):
    """A plain function ``shape(v)`` as a shaping with no numbers of its own."""

    def __init__(self, function):
        self.function = function

    def __call__(self, v, params=None):
        function = self.function

        penalty = function(v)

        return penalty


###############################################################################
# Internal machinery
###############################################################################


def shaping_of(shaping):
    """The shaping a cost holds: ``None``, a :class:`Shaping`, or a plain function wrapped as one."""
    if shaping is None or isinstance(shaping, Shaping):
        return shaping

    return CallableShaping(shaping)
