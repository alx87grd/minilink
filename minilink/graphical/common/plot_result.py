"""Common plotting result container."""

from dataclasses import dataclass
from typing import Any


@dataclass
class PlotResult:
    """Result returned by a plotting backend."""

    backend: str
    payload: Any
    figure: Any = None
    axes: Any = None

    def _ipython_display_(self):
        """Display nothing in a notebook: the plot call has already drawn the figure.

        A bare ``plant.plot_trajectory()`` as a cell's last line then shows the
        figure alone, without this record's text under it. The figure stays on
        ``result.figure``.
        """
