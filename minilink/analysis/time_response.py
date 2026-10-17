"""Step response of one input–output channel of the linear model, and its textbook figures."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.analysis import linear
from minilink.analysis.linearization import channel_subtitle, siso_matrices
from minilink.dynamics.abstraction.state_space import LTISystem
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

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class StepInfo:
    """Textbook step-response figures (MATLAB ``stepinfo`` conventions)."""

    rise_time: float  # 10 % to 90 % of the final value; nan when there is none
    settling_time: float  # Last exit from the band of 2 % of the peak deviation
    overshoot: float  # Percent of the final value; nan when there is none
    peak: float
    peak_time: float
    steady_state: float  # The last sample; nan when the response has not settled


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

    The response of the linear model, in deviation variables: ``sys`` as is
    when it is an ``LTISystem``, else linearized at ``(x_bar, u_bar)``. Same
    channel arguments as :func:`~minilink.analysis.frequency.bode`; ``tf``
    defaults to eight times the slowest stable time constant, and ``n``
    samples span ``[0, tf]``. The step of the nonlinear loop itself is a
    simulation, ``closed_loop(C, plant, reference=Step(...))`` then
    ``compute_trajectory``, and :func:`step_info` reads its sampled output the
    same way.
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    # The horizon: eight slowest time constants unless given
    if tf is None:
        tf = linear.settling_horizon(A)
    time = np.linspace(0.0, float(tf), int(n))

    # y(t) from rest under a unit step
    y = linear.step_response(A, B, C, D, time)

    return time, y


def step_info(time, y) -> StepInfo:
    """Rise time, settling time, overshoot, peak and steady state of a sampled step response.

    Reads any sampled response: the linear one from :func:`step_response`, or
    a simulated trajectory's time and output arrays. The settling band is 2 %
    of the response's peak deviation from its final value: 2 % of the final
    value for a step from rest that neither dips below its start nor overshoots
    by 100 %, and still defined for a response that returns to zero.
    """
    time = np.asarray(time, dtype=float).reshape(-1)
    y = np.asarray(y, dtype=float).reshape(-1)

    # Final value y_f: the last sample
    y_final = y[-1]

    # The settling band: 2 % of the peak deviation max |y − y_f|
    band = 0.02 * np.max(np.abs(y - y_final))

    # Settled when the last fifth of the record stays within the band
    tail = y[time >= 0.8 * time[-1]]
    settled = np.all(np.abs(tail - y_final) <= band)

    # Steady state y_ss = y_f, once settled
    y_ss = float(y_final) if settled else float(np.nan)

    # Peak: the largest |y| and its time
    k_p = int(np.argmax(np.abs(y)))
    y_p = float(y[k_p])
    t_p = float(time[k_p])

    # Settling time t_s: the first sample after the last exit from the band
    outside = np.flatnonzero(np.abs(y - y_final) > band)
    t_s = first_sample_after(time, outside) if settled else float(np.nan)

    # Rise time and overshoot are relative to a final value: none when the response
    # has not settled, or settles at zero (its final value inside the band)
    if not settled or abs(y_final) <= band:
        return StepInfo(
            rise_time=float(np.nan),
            settling_time=t_s,
            overshoot=float(np.nan),
            peak=y_p,
            peak_time=t_p,
            steady_state=y_ss,
        )

    # Progress toward y_f, signed so an undershoot (a zero in the right half plane) never counts
    progress = np.sign(y_final) * y

    # Rise time t_r = t(90 % of y_f) − t(10 % of y_f)
    t_10 = first_time_reaching(time, progress, 0.1 * abs(y_final))
    t_90 = first_time_reaching(time, progress, 0.9 * abs(y_final))
    t_r = float(t_90 - t_10)

    # Overshoot M_p = 100 (max progress − |y_f|) / |y_f| in percent, floored at 0
    M_p = float(100.0 * max((np.max(progress) - abs(y_final)) / abs(y_final), 0.0))

    return StepInfo(
        rise_time=t_r,
        settling_time=t_s,
        overshoot=M_p,
        peak=y_p,
        peak_time=t_p,
        steady_state=y_ss,
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
    """Step response of the selected channel with rise time, settling time and overshoot.

    The linear model's response, as :func:`step_response`; the title reads
    "Linear Step Response" for an ``LTISystem`` and "Linearized Step
    Response" for a system linearized here.
    """
    time, y = step_response(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, tf=tf, n=n, method=method, eps=eps
    )
    info = step_info(time, y)

    figure = step_figure(time, y, info, sys, of, wrt)

    return render_control_figure(figure, backend=backend, show=show)


###############################################################################
# Internal machinery
###############################################################################


def first_sample_after(time, outside):
    """Time of the sample after the last index in ``outside``; the start when none is."""
    if outside.size and outside[-1] + 1 < time.size:
        return float(time[outside[-1] + 1])
    return float(time[0])


def first_time_reaching(time, progress, level):
    """The first time ``progress`` reaches ``level``; ``nan`` when it never does."""
    reached = np.flatnonzero(progress >= level)
    return time[reached[0]] if reached.size else np.nan


def step_figure(time, y, info, sys, of, wrt):
    """The step-response figure: the response and its peak, the steady state and the figures' note."""
    # The steady-state line once settled, and the note of the finite figures
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

    # The title says whether the model is linear or was linearized here
    linear_model = isinstance(sys, LTISystem)
    title = style.LINEAR_STEP_TITLE if linear_model else style.LINEARIZED_STEP_TITLE
    subtitle = channel_subtitle(sys, of, wrt)

    # The response, hovered sample by sample, and its peak
    hover = tuple(f"t = {tk:.3g} s<br>y = {yk:.3g}" for tk, yk in zip(time, y))
    response = Trace(time, y, style.SYSTEM_COLOR, hover=hover)
    t_peak = np.array([info.peak_time])
    y_peak = np.array([info.peak])
    peak = Trace(t_peak, y_peak, style.SYSTEM_COLOR, mode="markers", marker="o")

    # One panel, with room under the response for the note
    notes = (Note(note, x=0.55, y=0.04),) if note else ()
    y_lim = (min(0.0, float(y.min())) - 0.35 * span, float(y.max()) + 0.1 * span)
    panel = Panel(
        traces=(response, peak),
        lines=lines,
        notes=notes,
        x_label=style.TIME_LABEL,
        y_label=style.AMPLITUDE_LABEL,
        y_lim=y_lim,
    )

    figure = ControlFigure(title=title, subtitle=subtitle, panels=(panel,))

    return figure
