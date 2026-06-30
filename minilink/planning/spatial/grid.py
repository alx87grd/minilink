"""
Backend-neutral rasters of a scalar field on a 2-D grid.

:func:`sample_grid` evaluates a scalar query ``fn(p, t, params)`` over an
axis-aligned grid and returns a :class:`FieldGrid` of plain NumPy arrays — no
matplotlib. Scenes and state fields use it to rasterize clearance and
cost-density queries for heatmaps and grid planners; rendering lives in
:mod:`minilink.planning.spatial.plotting`.
"""

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

# Public API


@dataclass(frozen=True)
class FieldGrid:
    """
    A scalar field sampled on a 2-D axis-aligned grid.

    Parameters
    ----------
    xs : np.ndarray
        Grid coordinates along the first axis, shape ``(nx,)``.
    ys : np.ndarray
        Grid coordinates along the second axis, shape ``(ny,)``.
    Z : np.ndarray
        Sampled values, shape ``(ny, nx)`` (row ``j`` holds ``ys[j]``).
    """

    xs: np.ndarray
    ys: np.ndarray
    Z: np.ndarray

    @property
    def extent(self) -> tuple:
        """``(x_min, x_max, y_min, y_max)`` for image-style plotting."""
        return (
            float(self.xs[0]),
            float(self.xs[-1]),
            float(self.ys[0]),
            float(self.ys[-1]),
        )


def sample_grid(
    fn: Callable,
    bounds,
    *,
    grid: tuple = (120, 120),
    t: float = 0.0,
    params=None,
) -> FieldGrid:
    """
    Sample a scalar query over a 2-D axis-aligned grid.

    Parameters
    ----------
    fn : callable
        Scalar query ``fn(p, t, params) -> float`` over a workspace point ``p``.
    bounds : ((x_min, x_max), (y_min, y_max))
        Grid extent along each axis.
    grid : (nx, ny)
        Number of samples along each axis.
    """
    (x_lo, x_hi), (y_lo, y_hi) = bounds
    nx, ny = grid
    xs = np.linspace(x_lo, x_hi, nx)
    ys = np.linspace(y_lo, y_hi, ny)

    Z = np.empty((ny, nx), dtype=float)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            Z[j, i] = float(fn(np.array([x, y]), t, params))
    return FieldGrid(xs, ys, Z)
