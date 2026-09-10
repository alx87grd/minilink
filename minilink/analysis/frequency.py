"""Frequency-domain analysis of one input–output channel of a linearized model.

Every number comes from :mod:`minilink.analysis.linear`. This module picks
one SISO channel ``(A, b, c, d)`` and plots the result.

``bode``, ``pzmap``, ``nyquist``, ``margins``, ``root_locus`` and
``transfer_function`` linearize ``sys`` about ``(x_bar, u_bar)`` with the
same arguments as :func:`~minilink.analysis.linearize.linearize`. ``of``
names the output and ``wrt`` the input (a port id means component 0,
``(port, index)`` one component, a diagram wire ``"block:port"`` an internal
signal). The ``plot_`` tools build one
:class:`~minilink.graphical.control.ControlFigure` and render it with
matplotlib or plotly.
"""

from __future__ import annotations

import numpy as np

from minilink.analysis import linear
from minilink.analysis.linear import Margins
from minilink.analysis.linearize import linearize_matrices, output_selectors
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


def frequency_response(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    w=None,
    n: int = 200,
    method: str = "auto",
    eps: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(w, G)`` with ``G(jw)`` complex on the selected channel.

    Parameters
    ----------
    sys : System
        System or diagram to linearize before computing the frequency response.
    x_bar, u_bar : array-like, optional
        Operating point; default ``sys.x0`` and the nominal port values.
    t : float, optional
        Time at which the Jacobians are evaluated.
    params : dict, optional
        Parameter set; default the live ``sys.params``.
    of : str or (str, int), optional
        Output of the channel: a port id, a diagram wire ``"block:port"``, or
        ``(selector, index)`` for one component (a bare id means component 0).
        Default: component 0 of the primary output.
    wrt : str or (str, int), optional
        Input of the channel, same forms. Default: component 0 of the first
        input port.
    w : array, optional
        Frequencies in rad/s. When omitted, a logarithmic grid runs one
        decade below the slowest pole or zero to one decade above the fastest.
    n : int, optional
        Number of frequencies for the automatic grid.
    method : {"auto", "fd", "jax"}, optional
        Differentiation backend, see :func:`~minilink.analysis.derivatives.jacobian`.
    eps : float, optional
        Central-difference step.
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )
    w = frequency_grid(A, B, C, D, w, n)

    return w, linear.frequency_response(A, B, C, D, w)


def bode(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    w=None,
    n: int = 200,
    method: str = "auto",
    eps: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the SISO Bode response ``(w, magnitude_db, phase_deg)``.

    Same arguments as :func:`frequency_response`; the phase is unwrapped.
    """
    w, G = frequency_response(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, w=w, n=n, method=method, eps=eps
    )

    magnitude_db, phase_deg = _bode_coordinates(G)
    return w, magnitude_db, phase_deg


def margins(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    w=None,
    n: int = 2000,
    method: str = "auto",
    eps: float = 1e-6,
) -> Margins:
    """Gain and phase margins of the selected channel taken as a loop gain.

    Same arguments as :func:`frequency_response`; returns
    :class:`~minilink.analysis.linear.Margins` (``inf`` where a crossover
    does not exist).
    """
    w, G = frequency_response(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, w=w, n=n, method=method, eps=eps
    )

    return linear.margins(w, G)


def nyquist(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    w=None,
    n: int = 500,
    method: str = "auto",
    eps: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(w, G)`` for the Nyquist contour, positive frequencies only.

    Same arguments as :func:`frequency_response`. The plot mirrors the
    conjugate branch; poles on the imaginary axis make ``|G|`` blow up at
    those frequencies (the indented contour is not drawn).
    """
    return frequency_response(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, w=w, n=n, method=method, eps=eps
    )


def pzmap(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    method: str = "auto",
    eps: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Return the selected SISO channel as ``(zeros, poles, gain)``.

    Same arguments as :func:`frequency_response` without the frequency grid:
    poles are the eigenvalues of ``A``, zeros the transmission zeros of the
    channel, ``gain`` the leading coefficient ``k`` of
    ``G(s) = k prod(s - z) / prod(s - p)``.
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    return linear.zeros(A, B, C, D), linear.poles(A), linear.gain(A, B, C, D)


def root_locus(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    gains=None,
    method: str = "auto",
    eps: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray]:
    """Closed-loop poles of ``u = -K y`` on the selected channel over a gain sweep.

    Same arguments as :func:`frequency_response` without the frequency grid;
    ``gains`` defaults to an adaptive sweep from ``K = 0`` to the gain where
    the far branches leave the picture. Returns ``(gains, roots)`` with
    ``roots`` of shape ``(len(gains), n)``, one continuous branch per column.
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    return linear.root_locus(A, B, C, D, gains)


def transfer_function(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    method: str = "auto",
    eps: float = 1e-6,
):
    """Return the selected SISO channel as a ``TransferFunction`` block.

    Same arguments as :func:`frequency_response` without the frequency grid.
    ``num(s) = k prod(s - z)`` and ``den(s) = prod(s - p)`` come from the
    zeros, poles and gain of :func:`pzmap`; the block is their state-space
    realization and carries ``numerator``, ``denominator``, ``poles`` and
    ``zeros``, so it can be plotted, simulated, or wired like any other block.
    """
    from minilink.blocks.transfer_function import TransferFunction

    z, p, k = pzmap(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    # num(s) = k ∏(s − z),   den(s) = ∏(s − p)
    num = np.real_if_close(k * np.poly(z)).astype(float)
    den = np.real_if_close(np.poly(p)).astype(float)
    return TransferFunction(num, den, name=f"{sys.name} {channel_label(sys, of, wrt)}")


# =============================================================================
# Public API — plots
# =============================================================================


def plot_bode(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    w=None,
    n: int = 200,
    margins: bool = True,
    method: str = "auto",
    eps: float = 1e-6,
    backend="matplotlib",
    show: bool = True,
    title: str | None = None,
) -> PlotResult:
    """Bode diagram of the selected channel; ``margins=True`` marks the crossovers.

    ``title`` overrides the figure heading (default ``"Bode Diagram"``).
    """
    w, G = frequency_response(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, w=w, n=n, method=method, eps=eps
    )

    return render_control_figure(
        _bode_figure(w, G, sys, of, wrt, margins, title=title),
        backend=backend,
        show=show,
    )


def plot_pzmap(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    method: str = "auto",
    eps: float = 1e-6,
    backend="matplotlib",
    show: bool = True,
) -> PlotResult:
    """Pole-zero map of the selected channel: ``x`` poles, ``o`` zeros."""
    z, p, gain = pzmap(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    return render_control_figure(
        _pzmap_figure(z, p, sys, of, wrt), backend=backend, show=show
    )


def plot_root_locus(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    gains=None,
    method: str = "auto",
    eps: float = 1e-6,
    backend="matplotlib",
    show: bool = True,
) -> PlotResult:
    """Root locus of the selected channel closed with ``u = -K y``."""
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )
    K, roots = linear.root_locus(A, B, C, D, gains)

    return render_control_figure(
        _root_locus_figure(A, B, C, D, K, roots, sys, of, wrt),
        backend=backend,
        show=show,
    )


def plot_nyquist(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    of=None,
    wrt=None,
    w=None,
    n: int = 500,
    method: str = "auto",
    eps: float = 1e-6,
    backend="matplotlib",
    show: bool = True,
) -> PlotResult:
    """Nyquist diagram of the selected channel with the critical point ``-1``."""
    w, G = nyquist(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, w=w, n=n, method=method, eps=eps
    )

    return render_control_figure(
        _nyquist_figure(w, G, sys, of, wrt), backend=backend, show=show
    )


# =============================================================================
# Channel helpers
# =============================================================================


def siso_channel(sys, of, wrt):
    """Normalize the channel to ``((of_name, index), (wrt_name, index))``.

    ``of_name`` is ``None`` when the output is the state itself (no ``y``
    port): the row is then taken from ``C = I``.
    """
    if wrt is None:
        if not sys.inputs:
            raise ValueError("Frequency analysis requires at least one input port.")
        wrt = (next(iter(sys.inputs)), 0)
    if of is None:
        default = output_selectors(sys, None)
        of = (None, 0) if default is None else (default[0][0], 0)
    return _component(of, "of"), _component(wrt, "wrt")


def channel_label(sys, of, wrt):
    """``"y[1] / u[0]"`` for the selected channel."""
    (of_name, i), (wrt_name, j) = siso_channel(sys, of, wrt)
    return f"{'x' if of_name is None else of_name}[{i}] / {wrt_name}[{j}]"


def channel_subtitle(sys, of, wrt):
    """``"From: u[0]  To: y[1]"`` for the selected channel."""
    (of_name, i), (wrt_name, j) = siso_channel(sys, of, wrt)
    return style.channel_subtitle("x" if of_name is None else of_name, i, wrt_name, j)


def siso_matrices(sys, x_bar, u_bar, t, params, *, of, wrt, method, eps):
    """``A, b, c, d`` of the selected channel (``b`` a column, ``c`` a row)."""
    (of_name, i), channel_in = siso_channel(sys, of, wrt)
    A, B, C, D = linearize_matrices(
        sys,
        x_bar,
        u_bar,
        t,
        params,
        of=None if of_name is None else [(of_name, i)],
        wrt=[channel_in],
        method=method,
        eps=eps,
    )
    if of_name is None:  # state output: pick the component of C = I
        if i < 0 or i >= C.shape[0]:
            raise ValueError(
                f"of index must be in [0, {C.shape[0] - 1}] for the state."
            )
        C, D = C[[i], :], D[[i], :]
    return A, B, C, D


def frequency_grid(A, B, C, D, w, n):
    """The frequencies to evaluate: the user's, or a log grid from the poles and zeros."""
    if w is None:
        w_min, w_max = linear.frequency_range(A, B, C, D)
        w = np.logspace(np.log10(w_min), np.log10(w_max), int(n))
    w = np.asarray(w, dtype=float).reshape(-1)
    if w.size == 0 or np.any(w <= 0.0):
        raise ValueError("Frequencies must be positive.")
    return w


# =============================================================================
# Internal machinery
# =============================================================================


def _bode_coordinates(G):
    with np.errstate(divide="ignore"):
        magnitude_db = 20.0 * np.log10(np.abs(G))
    return magnitude_db, np.degrees(np.unwrap(np.angle(G)))


def _component(selector, name):
    """One component ``(name, index)`` from a port id or a ``(port, index)`` pair."""
    if isinstance(selector, str):
        return (selector, 0)
    if (
        isinstance(selector, tuple)
        and len(selector) == 2
        and (selector[0] is None or isinstance(selector[0], str))
        and isinstance(selector[1], (int, np.integer))
        and not isinstance(selector[1], bool)
    ):
        return (selector[0], int(selector[1]))
    raise TypeError(
        f"{name} names one channel: a port id, a diagram wire 'block:port', or "
        f"(selector, index); got {selector!r}"
    )


def _margins_text(m: Margins) -> str:
    gm = (
        "inf"
        if not np.isfinite(m.gain_margin_db)
        else f"{m.gain_margin_db:.1f} dB (at {m.w_phase_crossover:.3g} rad/s)"
    )
    pm = (
        "inf"
        if not np.isfinite(m.phase_margin_deg)
        else f"{m.phase_margin_deg:.1f} deg (at {m.w_gain_crossover:.3g} rad/s)"
    )
    return f"Gm = {gm}\nPm = {pm}"


def _root_text(s) -> str:
    damping = float(-s.real / abs(s)) if abs(s) > 0.0 else 1.0
    return (
        f"s = {s.real:.3g} {s.imag:+.3g}j<br>ζ = {damping:.3f}, ωn = {abs(s):.3g} rad/s"
    )


def _root_markers(p, z):
    """Poles as ``x`` and zeros as ``o`` in the system colour."""
    return (
        Trace(
            p.real,
            p.imag,
            style.SYSTEM_COLOR,
            mode="markers",
            marker="x",
            hover=tuple(_root_text(s) for s in p),
        ),
        Trace(
            z.real,
            z.imag,
            style.SYSTEM_COLOR,
            mode="markers",
            marker="o",
            hover=tuple(_root_text(s) for s in z),
        ),
    )


def _bode_figure(w, G, sys, of, wrt, margins, title=None):
    magnitude_db, phase_deg = _bode_coordinates(G)
    hover = tuple(
        f"ω = {wk:.3g} rad/s<br>|G| = {mk:.1f} dB<br>∠G = {pk:.1f}°"
        for wk, mk, pk in zip(w, magnitude_db, phase_deg)
    )
    crossovers, notes, references = (), (), ()
    if margins:
        m = linear.margins(w, G)
        crossovers = tuple(
            RefLine("x", w_c)
            for w_c in (m.w_gain_crossover, m.w_phase_crossover)
            if np.isfinite(w_c)
        )
        notes = (Note(_margins_text(m)),)
        references = (RefLine("y", 0.0),), (RefLine("y", -180.0),)
    else:
        references = (), ()
    return ControlFigure(
        title=style.BODE_TITLE if title is None else title,
        subtitle=channel_subtitle(sys, of, wrt),
        share_x=True,
        panels=(
            Panel(
                traces=(Trace(w, magnitude_db, style.SYSTEM_COLOR, hover=hover),),
                lines=crossovers + references[0],
                notes=notes,
                x_label="",
                y_label=style.MAGNITUDE_LABEL,
                x_log=True,
            ),
            Panel(
                traces=(Trace(w, phase_deg, style.SYSTEM_COLOR, hover=hover),),
                lines=crossovers + references[1],
                x_label=style.FREQUENCY_LABEL,
                y_label=style.PHASE_LABEL,
                x_log=True,
                y_ticks=style.phase_ticks(phase_deg),
            ),
        ),
    )


def _pzmap_figure(z, p, sys, of, wrt):
    points = np.concatenate([z, p])
    return ControlFigure(
        title=style.PZMAP_TITLE,
        subtitle=channel_subtitle(sys, of, wrt),
        panels=(
            Panel(
                traces=_root_markers(p, z),
                x_label=style.REAL_LABEL,
                y_label=style.IMAG_LABEL,
                zero_lines=True,
                x_lim=style.padded_limits(np.concatenate([points.real, [0.0]])),
                y_lim=style.padded_limits(np.concatenate([points.imag, -points.imag])),
            ),
        ),
    )


def _root_locus_figure(A, B, C, D, K, roots, sys, of, wrt):
    # The view keeps the poles, zeros and the branches near them; the far tails
    # of the asymptotes leave the frame as they do in MATLAB.
    reach = 3.0 * max(
        np.max(
            np.abs(np.concatenate([linear.poles(A), linear.zeros(A, B, C, D)])),
            initial=0.0,
        ),
        1.0,
    )
    near = roots[np.abs(roots) <= reach]
    branches = tuple(
        Trace(
            roots[:, j].real,
            roots[:, j].imag,
            style.SYSTEM_COLOR,
            hover=tuple(
                f"K = {k:.3g}<br>" + _root_text(s) for k, s in zip(K, roots[:, j])
            ),
        )
        for j in range(roots.shape[1])
    )
    return ControlFigure(
        title=style.ROOT_LOCUS_TITLE,
        subtitle=channel_subtitle(sys, of, wrt),
        panels=(
            Panel(
                traces=branches
                + _root_markers(linear.poles(A), linear.zeros(A, B, C, D)),
                x_label=style.REAL_LABEL,
                y_label=style.IMAG_LABEL,
                zero_lines=True,
                x_lim=style.padded_limits(np.concatenate([near.real, [0.0]])),
                y_lim=style.padded_limits(np.concatenate([near.imag, -near.imag])),
            ),
        ),
    )


def _nyquist_figure(w, G, sys, of, wrt):
    hover = tuple(
        f"ω = {wk:.3g} rad/s<br>G = {g.real:.3g} {g.imag:+.3g}j" for wk, g in zip(w, G)
    )
    # Zoom on the part of the contour that matters: poles at the origin send
    # |G| to infinity at low frequency, so the limits ignore the far points.
    visible = G[np.abs(G) <= 10.0 * np.median(np.abs(G))]
    points = np.concatenate([visible, np.conj(visible), [-1.0 + 0j]])
    critical = Trace(
        np.array([-1.0]),
        np.array([0.0]),
        style.CRITICAL_COLOR,
        mode="markers",
        marker="+",
        size=10.0,
    )
    return ControlFigure(
        title=style.NYQUIST_TITLE,
        subtitle=channel_subtitle(sys, of, wrt),
        panels=(
            Panel(
                traces=(
                    Trace(G.real, G.imag, style.SYSTEM_COLOR, hover=hover, arrows=True),
                    Trace(G.real, -G.imag, style.SYSTEM_COLOR, dash="dash"),
                    critical,
                ),
                x_label="Real Axis",
                y_label="Imaginary Axis",
                zero_lines=True,
                x_lim=style.padded_limits(points.real),
                y_lim=style.padded_limits(points.imag),
            ),
        ),
    )
