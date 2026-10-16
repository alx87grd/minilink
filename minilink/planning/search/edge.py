"""Tree edges: the trajectory segment every RRT connector produces."""

from dataclasses import dataclass

import numpy as np

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class Edge:
    """
    A trajectory segment grown from a tree node.

    Both connector styles, forward-integrated controls (kinodynamic) and exact
    two-point steering, produce one. Every minilink system is an ODE
    ``ẋ = f(x, u, t)``, so an edge always carries real states, inputs and
    times; path extraction concatenates edges into a single
    :class:`~minilink.core.trajectory.Trajectory`.

    Parameters
    ----------
    states : np.ndarray
        State samples with shape ``(k+1, n)`` (knot ``0`` is the parent state).
    inputs : np.ndarray
        Control samples with shape ``(k, m)`` (one per interval).
    times : np.ndarray
        Relative times with shape ``(k+1,)`` starting at ``0``.
    cost : float
        Edge cost-to-come (integrated running cost / arc length).
    """

    states: np.ndarray
    inputs: np.ndarray
    times: np.ndarray
    cost: float

    @property
    def x_end(self) -> np.ndarray:
        """Final state of the segment."""
        return self.states[-1]
