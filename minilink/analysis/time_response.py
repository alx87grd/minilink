"""Time response of one input–output channel of a linearized model.

The ODE march lives in :func:`minilink.analysis.linear.step_response`. This
module picks the channel and the horizon, then reads textbook step-response
figures off the samples.

``step_response`` keeps the same channel selection as
:mod:`minilink.analysis.frequency`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.analysis import linear
from minilink.analysis.frequency import channel_subtitle, siso_matrices
from minilink.graphical.common import PlotResult
from minilink.graphical.control import (
    ControlFigure,
    Note,
    Panel,
    RefLine,
    Trace,
    render_control_figure,
    style,
)


@dataclass(frozen=True)
class StepInfo:
    """Textbook step-response figures (MATLAB ``stepinfo`` conventions)."""

    rise_time: float  # 10 % to 90 % of the final value
    settling_time: float  # last time the response leaves the 2 % band
    overshoot: float  # percent of the final value
    peak: float
    peak_time: float
    steady_state: float  # the last sample; nan when the response has not settled


def step_response(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    tf=None,
    n: int = 500,
    method: str = "auto",
    eps: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(time, y)``: the unit-step response of the selected channel from rest.

    Same channel arguments as :func:`~minilink.analysis.frequency.bode`;
    ``tf`` defaults to five times the slowest stable time constant.
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    if tf is None:
        tf = linear.settling_horizon(A)
    time = np.linspace(0.0, float(tf), int(n))
    return time, linear.step_response(A, B, C, D, time)


def step_info(time, y) -> StepInfo:
    """Rise time, settling time, overshoot, peak and steady state of a sampled step response."""
    time = np.asarray(time, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)

    # y_∞ = last sample if the tail stays in the 2 % band, else nan
    # rise:      t(90%) − t(10%) of |y_∞|
    # settling:  last time |y − y_∞| leaves the 2 % band
    # overshoot: 100 · (|peak| − |y_∞|) / |y_∞|
    final = y[-1]
    peak_index = int(np.argmax(np.abs(y)))
    peak = y[peak_index]
    return StepInfo(
        _rise_time(time, y, final),
        _settling_time(time, y, final),
        _overshoot(peak, final),
        float(peak),
        float(time[peak_index]),
        _steady_state(time, y, final),
    )


def plot_step_response(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    tf=None,
    n: int = 500,
    method: str = "auto",
    eps: float = 1e-6,
    backend="matplotlib",
    show: bool = True,
) -> PlotResult:
    """Step response of the selected channel with rise time, settling time and overshoot."""
    time, y = step_response(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, tf=tf, n=n, method=method, eps=eps
    )
    info = step_info(time, y)

    return render_control_figure(
        _step_figure(time, y, info, sys, of, wrt), backend=backend, show=show
    )


# =============================================================================
# Internal machinery
# =============================================================================


def _steady_state(time, y, final):
    tail = y[time >= 0.8 * time[-1]]
    settled = np.all(np.abs(tail - final) <= 0.02 * max(abs(final), 1e-12))
    return float(final) if settled else float(np.nan)


def _overshoot(peak, final):
    if final == 0.0:
        return float(np.nan)
    return float(100.0 * max((abs(peak) - abs(final)) / abs(final), 0.0))


def _rise_time(time, y, final):
    if final == 0.0:
        return float(np.nan)
    crossed_10 = np.flatnonzero(np.abs(y) >= 0.1 * abs(final))
    crossed_90 = np.flatnonzero(np.abs(y) >= 0.9 * abs(final))
    if crossed_10.size and crossed_90.size:
        return float(time[crossed_90[0]] - time[crossed_10[0]])
    return float(np.nan)


def _settling_time(time, y, final):
    tail = y[time >= 0.8 * time[-1]]
    settled = np.all(np.abs(tail - final) <= 0.02 * max(abs(final), 1e-12))
    if not settled:
        return float(np.nan)
    outside = np.flatnonzero(np.abs(y - final) > 0.02 * abs(final))
    if outside.size and outside[-1] + 1 < time.size:
        return float(time[outside[-1] + 1])
    return float(time[0])


def _step_figure(time, y, info, sys, of, wrt):
    lines = () if np.isnan(info.steady_state) else (RefLine("y", info.steady_state),)
    note = "\n".join(
        f"{name} = {value:.3g} {unit}"
        for name, value, unit in (
            ("Rise time", info.rise_time, "s"),
            ("Settling time", info.settling_time, "s"),
            ("Overshoot", info.overshoot, "%"),
        )
        if np.isfinite(value)
    )
    span = float(np.ptp(y)) or 1.0
    return ControlFigure(
        title=style.STEP_TITLE,
        subtitle=channel_subtitle(sys, of, wrt),
        panels=(
            Panel(
                traces=(
                    Trace(
                        time,
                        y,
                        style.SYSTEM_COLOR,
                        hover=tuple(
                            f"t = {tk:.3g} s<br>y = {yk:.3g}" for tk, yk in zip(time, y)
                        ),
                    ),
                    Trace(
                        np.array([info.peak_time]),
                        np.array([info.peak]),
                        style.SYSTEM_COLOR,
                        mode="markers",
                        marker="o",
                    ),
                ),
                lines=lines,
                notes=(Note(note, x=0.55, y=0.04),) if note else (),
                x_label=style.TIME_LABEL,
                y_label=style.AMPLITUDE_LABEL,
                y_lim=(
                    min(0.0, float(y.min())) - 0.35 * span,
                    float(y.max()) + 0.1 * span,
                ),
            ),
        ),
    )
