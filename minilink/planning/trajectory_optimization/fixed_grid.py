"""Shared fixed-time-grid options for trajectory transcriptions."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FixedGridOptions:
    """Uniform fixed-time-grid options."""

    tf: float
    n_steps: int

    def __post_init__(self) -> None:
        tf = float(self.tf)
        if not np.isfinite(tf) or tf <= 0.0:
            raise ValueError("tf must be a positive finite scalar")
        if isinstance(self.n_steps, bool) or int(self.n_steps) < 2:
            raise ValueError("n_steps must be an integer greater than or equal to 2")
        object.__setattr__(self, "tf", tf)
        object.__setattr__(self, "n_steps", int(self.n_steps))

    @property
    def t(self) -> np.ndarray:
        """Uniform transcription time grid."""
        return np.linspace(0.0, self.tf, self.n_steps)

    @property
    def dt(self) -> float:
        """Uniform time step between grid knots."""
        return self.tf / float(self.n_steps - 1)
