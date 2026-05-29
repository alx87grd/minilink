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
