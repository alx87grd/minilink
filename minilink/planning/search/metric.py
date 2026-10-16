"""Nearest-neighbour metrics: the distance between two states that a search tree ranks by."""

from collections.abc import Callable

import numpy as np

###############################################################################
# Public API
###############################################################################

# A metric is a plain callable ``metric(a, b) -> float``, a pairwise state
# pseudometric, not a CostFunction: it integrates no running cost along a
# trajectory. The planner uses it to pick the node to grow from and the best
# candidate edge; a SteeringFunction exposes a matching ``distance``, so a
# steering RRT can pass ``metric=steering.distance``.


def euclidean(a, b) -> float:
    """Euclidean distance between two states."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)

    distance = float(np.linalg.norm(a - b))

    return distance


def weighted(weights) -> Callable:
    """Return the per-axis weighted Euclidean distance: each squared difference scaled by its weight."""
    w = np.asarray(weights, dtype=float).reshape(-1).copy()

    def distance(a, b) -> float:
        d = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)

        # ‖a − b‖_w = √(Σᵢ wᵢ (aᵢ − bᵢ)²)
        weighted_distance = float(np.sqrt(np.sum(w * d * d)))

        return weighted_distance

    return distance
