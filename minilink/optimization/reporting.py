"""Shared formatting for the human-readable ``disp`` solve panels.

Used by :class:`~minilink.optimization.optimizer.Optimizer` and
:class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`
so both panels share one frame width and vector preview style.
"""

import numpy as np

DISP_LINE_WIDTH = 60
DISP_RULE_MAIN = "=" * DISP_LINE_WIDTH
DISP_RULE_DIV = "-" * DISP_LINE_WIDTH


def preview_vector(value) -> str | None:
    """Compact one-line preview of a vector (head + length for long ones)."""
    if value is None:
        return None
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size <= 8:
        return np.array2string(arr, precision=6, max_line_width=96)
    head = np.array2string(arr[:4], precision=6, max_line_width=96)
    return f"{head} ... ({arr.size} values)"
