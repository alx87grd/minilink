"""
Discretization placeholders for future deterministic policy planners.

This module is intentionally small in the architecture pass. Grid
construction for dynamic programming will live here once the planning API
has been reviewed.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GridSpec:
    """
    Lightweight grid specification for future DP-style solvers.
    """

    shape: tuple[int, ...]

    def __post_init__(self) -> None:
        shape = tuple(int(v) for v in self.shape)
        if any(v < 1 for v in shape):
            raise ValueError("Grid dimensions must be positive")
        object.__setattr__(self, "shape", shape)
