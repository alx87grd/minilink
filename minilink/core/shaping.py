"""Cost shaping: the penalty of a scalar field value, before a field's cost weights it."""

from minilink.core.backends import array_module

# The field stays a signed physical quantity (clearance in length units); shaping
# at the edge, before :meth:`~minilink.core.fields.Field.as_cost` weights it, turns
# it into a barrier, a one-sided hinge, or a bounded occupancy score, so an
# obstacle term balances cleanly against other costs such as quadratic tracking.
#
# Each factory returns a native-array callable ``shape(v) -> penalty`` for
# ``field.as_cost(shaping=...)`` and ``set_.as_cost(shaping=...)``; the callables
# stay NumPy under NumPy input and trace under JAX.


###############################################################################
# Public API
###############################################################################


def quadratic_hinge(threshold: float = 0.0):
    """
    One-sided quadratic penalty below ``threshold``: ``max(threshold - v, 0)**2``.

    Zero in free space and quadratic once the value drops past ``threshold`` —
    the same scale as a quadratic tracking cost, so their weights are directly
    comparable, and the gradient keeps growing with penetration.
    """
    threshold = float(threshold)

    def shape(v):
        xp = array_module(v)

        penalty = xp.maximum(threshold - v, 0.0) ** 2

        return penalty

    return shape


def quadratic_excess(threshold: float = 0.0):
    """
    One-sided quadratic penalty above ``threshold``: ``max(v - threshold, 0)**2``.

    Use with :class:`~minilink.planning.spatial.state_fields.PathDistanceField`
    to penalize deviation from a reference path (mirror of :func:`quadratic_hinge`
    used for clearance margins).
    """
    threshold = float(threshold)

    def shape(v):
        xp = array_module(v)

        penalty = xp.maximum(v - threshold, 0.0) ** 2

        return penalty

    return shape


def inverse_barrier(epsilon: float = 1e-2):
    """
    Repulsive barrier ``1 / max(v, epsilon)**2`` that blows up at contact.

    Strong, unbounded repulsion near the boundary; ``epsilon`` caps the value
    once the clearance is exhausted so the penalty stays finite.
    """
    epsilon = float(epsilon)

    def shape(v):
        xp = array_module(v)

        barrier = 1.0 / xp.maximum(v, epsilon) ** 2

        return barrier

    return shape


def occupancy(scale: float = 1.0):
    """
    Bounded occupancy score in ``[0, 1]``: ``sigmoid(-v / scale)``.

    ``~0`` well outside, ``0.5`` on the boundary, ``~1`` deep inside. Bounded,
    so weighting an obstacle term against an unbounded cost is interpretable —
    at the price of a gradient that saturates far from the boundary.
    """
    scale = float(scale)

    def shape(v):
        xp = array_module(v)

        # σ(−v / s) = 1 / (1 + e^{v/s})
        occupancy = 1.0 / (1.0 + xp.exp(v / scale))

        return occupancy

    return shape
