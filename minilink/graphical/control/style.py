"""The look of the control-analysis figures, shared by both backends.

Titles, axis labels and colours follow the MATLAB Control System Toolbox so
the figures read like the ones in the course notes.
"""

from __future__ import annotations

import numpy as np

SYSTEM_COLOR = "#0072BD"  # MATLAB blue
SECOND_COLOR = "#D95319"  # MATLAB orange
CRITICAL_COLOR = "#A2142F"  # MATLAB red, the -1 point
AXIS_COLOR = "#7f7f7f"

BODE_TITLE = "Bode Diagram"
PZMAP_TITLE = "Pole-Zero Map"
ROOT_LOCUS_TITLE = "Root Locus"
NYQUIST_TITLE = "Nyquist Diagram"
STEP_TITLE = "Step Response"

MAGNITUDE_LABEL = "Magnitude (dB)"
PHASE_LABEL = "Phase (deg)"
FREQUENCY_LABEL = "Frequency (rad/s)"
REAL_LABEL = "Real Axis (seconds⁻¹)"
IMAG_LABEL = "Imaginary Axis (seconds⁻¹)"
TIME_LABEL = "Time (seconds)"
AMPLITUDE_LABEL = "Amplitude"


def channel_subtitle(of_name: str, of_index: int, wrt_name: str, wrt_index: int) -> str:
    """``"From: u[0]  To: y[1]"``."""
    return f"From: {wrt_name}[{wrt_index}]  To: {of_name}[{of_index}]"


def phase_ticks(phase_deg) -> tuple[float, ...]:
    """Ticks at multiples of 45 or 90 degrees covering the phase range."""
    lo, hi = float(np.min(phase_deg)), float(np.max(phase_deg))
    step = 45.0 if hi - lo <= 270.0 else 90.0
    return tuple(
        np.arange(np.floor(lo / step) * step, np.ceil(hi / step) * step + step, step)
    )


def padded_limits(
    values, fraction: float = 0.1, minimum: float = 1.0
) -> tuple[float, float]:
    """Axis limits around ``values`` with ``fraction`` of the span (at least ``minimum``) on each side."""
    values = np.asarray(values, dtype=float).reshape(-1)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return -minimum, minimum
    lo, hi = float(values.min()), float(values.max())
    pad = max(fraction * (hi - lo), 0.5 * minimum)
    return lo - pad, hi + pad
