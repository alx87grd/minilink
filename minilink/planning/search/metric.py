"""
Nearest-neighbour distances for the tree.

A metric is a plain callable ``metric(a, b) -> float`` (a pairwise state
pseudometric) — distinct from a :class:`~minilink.core.costs.CostFunction`,
which integrates a running cost along a trajectory. The orchestrator uses it to
pick the node to grow from and the best candidate edge; a `SteeringFunction`
exposes a matching ``distance`` so a steering RRT can pass ``metric=steering.distance``.
"""

from collections.abc import Callable

import numpy as np


def euclidean(a, b) -> float:
    """Euclidean distance between two states."""
    return float(
        np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float))
    )


def weighted(weights) -> Callable:
    """Return a per-axis weighted Euclidean distance ``sqrt(Σ w_i (a_i - b_i)^2)``."""
    w = np.asarray(weights, dtype=float).reshape(-1).copy()

    def distance(a, b) -> float:
        d = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
        return float(np.sqrt(np.sum(w * d * d)))

    return distance
