"""Solve panels: the frame and the vector preview of the human-readable reports."""

import numpy as np

# One frame width and one preview style for every solve panel: the
# Optimizer's and the TrajectoryOptimizationPlanner's
DISP_LINE_WIDTH = 60
DISP_RULE_MAIN = "=" * DISP_LINE_WIDTH
DISP_RULE_DIV = "-" * DISP_LINE_WIDTH


###############################################################################
# Public API
###############################################################################


def preview_vector(value) -> str | None:
    """Compact one-line preview of a vector (head + length for long ones)."""
    if value is None:
        return None

    arr = np.asarray(value, dtype=float).reshape(-1)

    # A short vector in full
    if arr.size <= 8:
        return np.array2string(arr, precision=6, max_line_width=96)

    # A long one by its first four entries and its length
    head = np.array2string(arr[:4], precision=6, max_line_width=96)

    return f"{head} ... ({arr.size} values)"
