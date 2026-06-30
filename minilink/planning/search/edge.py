"""
The shared currency of the RRT family: a trajectory segment.

Both connector styles — forward-integrating controls (kinodynamic) and exact
two-point steering — produce an :class:`Edge`. Because every minilink system is
an ODE ``dx = f(x,u,t)``, an edge always carries real states, inputs, and times;
path extraction concatenates edges into a single :class:`~minilink.core.trajectory.Trajectory`.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Edge:
    """
    A trajectory segment grown from a tree node.

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
