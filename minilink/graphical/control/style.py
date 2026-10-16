"""Control-analysis figure style: the titles, labels, colors and axes both backends share."""

from __future__ import annotations

import numpy as np

# Titles, axis labels and colours follow the MATLAB Control System Toolbox, so
# the figures read like the ones in the course notes.
SYSTEM_COLOR = "#0072BD"  # MATLAB blue
SECOND_COLOR = "#D95319"  # MATLAB orange
CRITICAL_COLOR = "#A2142F"  # MATLAB red, the -1 point
AXIS_COLOR = "#7f7f7f"

BODE_TITLE = "Bode Diagram"
PZMAP_TITLE = "Pole-Zero Map"
ROOT_LOCUS_TITLE = "Root Locus"
NYQUIST_TITLE = "Nyquist Diagram"
LINEAR_STEP_TITLE = "Linear Step Response"  # The system is a linear model
LINEARIZED_STEP_TITLE = "Linearized Step Response"  # It was linearized first

MAGNITUDE_LABEL = "Magnitude (dB)"
PHASE_LABEL = "Phase (deg)"
FREQUENCY_LABEL = "Frequency (rad/s)"
REAL_LABEL = "Real Axis (seconds⁻¹)"
IMAG_LABEL = "Imaginary Axis (seconds⁻¹)"
TIME_LABEL = "Time (seconds)"
AMPLITUDE_LABEL = "Amplitude"


###############################################################################
# Public API
###############################################################################


def channel_subtitle(of_label: str, wrt_label: str) -> str:
    """``"From: u[0]  To: y[1]"``, or the components' declared labels."""
    return f"From: {wrt_label}  To: {of_label}"


def phase_ticks(phase_deg) -> tuple[float, ...]:
    """Ticks at multiples of 45 or 90 degrees covering the phase range."""
    lo = float(np.min(phase_deg))
    hi = float(np.max(phase_deg))

    # A 45° step over a range of at most 270°, a 90° step beyond
    step = 45.0 if hi - lo <= 270.0 else 90.0

    # The multiples of the step from the one below lo to the one above hi
    first = np.floor(lo / step) * step
    last = np.ceil(hi / step) * step
    ticks = np.arange(first, last + step, step)

    return tuple(ticks)


def padded_limits(
    values, fraction: float = 0.1, minimum: float = 1.0
) -> tuple[float, float]:
    """Axis limits around ``values`` with ``fraction`` of the span (at least ``minimum``) on each side."""
    values = np.asarray(values, dtype=float).reshape(-1)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return -minimum, minimum

    lo = float(values.min())
    hi = float(values.max())

    # A fraction of the span on each side, never less than half the minimum width
    pad = max(fraction * (hi - lo), 0.5 * minimum)
    lower = lo - pad
    upper = hi + pad

    return lower, upper
