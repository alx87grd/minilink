"""
Cost-shaping functions for spatial state fields.

A shaping maps a scalar field value to a penalty before
:meth:`~minilink.planning.spatial.state_fields.StateField.as_cost` weights it.
The core field stays a signed physical quantity (clearance in length units);
shaping at the edge turns it into a barrier, a one-sided hinge, or a bounded
occupancy score so an obstacle term balances cleanly against other costs such
as quadratic tracking.

Each factory returns a native-array callable ``shape(v) -> penalty`` suitable
for ``field.as_cost(shaping=...)``; the callables stay NumPy under NumPy input
and trace under JAX.
"""

from minilink.core.backends import array_module

# Public API


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
        return xp.maximum(threshold - v, 0.0) ** 2

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
        return xp.maximum(v - threshold, 0.0) ** 2

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
        return 1.0 / xp.maximum(v, epsilon) ** 2

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
        return 1.0 / (1.0 + xp.exp(v / scale))

    return shape
