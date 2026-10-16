"""Frequency-domain analysis of one input–output channel of a linearized model."""

from __future__ import annotations

import inspect
import os
import warnings

import numpy as np

from minilink.analysis import linear
from minilink.analysis.linear import Margins
from minilink.analysis.linearization import (
    channel_label,
    channel_subtitle,
    siso_matrices,
)
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

# Frequencies of the grid on which margins() reads the crossovers
_MARGIN_GRID_POINTS = 2000


###############################################################################
# Public API
###############################################################################


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
        Operating point; default the nominal state and the nominal port values.
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
        decade below the slowest pole or zero to one decade above the fastest,
        widened until it brackets the 0 dB crossing.
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

    # G(jω) on the grid
    G = linear.frequency_response(A, B, C, D, w)

    return w, G


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

    # |G(jω)| in dB and the unwrapped ∠G(jω) in degrees
    with np.errstate(divide="ignore"):
        magnitude_db = 20.0 * np.log10(np.abs(G))
    phase_deg = np.degrees(np.unwrap(np.angle(G)))

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
    n: int = _MARGIN_GRID_POINTS,
    method: str = "auto",
    eps: float = 1e-6,
) -> Margins:
    """Gain and phase margins of the selected channel taken as a loop gain.

    Same arguments as :func:`frequency_response`; returns
    :class:`~minilink.analysis.linear.Margins` (``inf`` where a crossover
    does not exist).
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )
    w = frequency_grid(A, B, C, D, w, n)

    # L(jω) on the grid brackets the crossovers; each is then solved on the exact L(jω)
    G = linear.frequency_response(A, B, C, D, w)
    loop_margins = linear.margins(w, G, loop_response(A, B, C, D))

    return loop_margins


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
    w, G = frequency_response(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, w=w, n=n, method=method, eps=eps
    )

    return w, G


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
    minimal: bool | None = None,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Return the selected SISO channel as ``(zeros, poles, gain)``.

    Same arguments as :func:`frequency_response` without the frequency grid:
    poles are the eigenvalues of ``A``, zeros the transmission zeros of the
    channel, ``gain`` the leading coefficient of its factored transfer
    function. ``minimal`` (default: yes, with a
    one-time notice when a pair cancels) first reduces the channel to its
    minimal realization, so the uncontrollable and unobservable modes cancel;
    ``minimal=False`` keeps every mode of the realization.
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    # Minimal realization: the uncontrollable and unobservable modes cancel
    A, B, C, D = minimal_channel(A, B, C, D, minimal)

    # G(s) = k ∏(s − z) / ∏(s − p)
    z = linear.zeros(A, B, C, D)
    p = linear.poles(A)
    k = linear.gain(A, B, C, D)

    return z, p, k


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
    minimal: bool | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Closed-loop poles of ``u = -K y`` on the selected channel over a gain sweep.

    Same arguments as :func:`frequency_response` without the frequency grid;
    ``gains`` defaults to an adaptive sweep from ``K = 0`` to the gain where
    the far branches leave the picture. Returns ``(gains, roots)`` with
    ``roots`` of shape ``(len(gains), n)``, one continuous branch per column.
    ``minimal`` as in :func:`pzmap`.
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    # Minimal realization: the uncontrollable and unobservable modes cancel
    A, B, C, D = minimal_channel(A, B, C, D, minimal)

    # p(K) = eig(A − B K (1 + K d)⁻¹ C) over the gain sweep
    gains, roots = linear.root_locus(A, B, C, D, gains)

    return gains, roots


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
    minimal: bool | None = None,
):
    """Return the selected SISO channel as a ``TransferFunction`` block.

    Same arguments as :func:`frequency_response` without the frequency grid;
    ``minimal`` as in :func:`pzmap`.
    The numerator and denominator come from the zeros, poles and gain of
    :func:`pzmap`; the block is their state-space realization and carries
    ``numerator``, ``denominator``, ``poles`` and ``zeros``, so it can be
    plotted, simulated, or wired like any other block.
    """
    # Lazy import: the blocks and their drawing primitives load on first use, not
    # with the analysis
    from minilink.blocks.transfer_function import TransferFunction

    z, p, k = pzmap(
        sys,
        x_bar,
        u_bar,
        t,
        params,
        of=of,
        wrt=wrt,
        method=method,
        eps=eps,
        minimal=minimal,
    )

    # num(s) = k ∏(s − z),   den(s) = ∏(s − p)
    num = np.real_if_close(k * np.poly(z)).astype(float)
    den = np.real_if_close(np.poly(p)).astype(float)

    return TransferFunction(num, den, name=f"{sys.name} {channel_label(sys, of, wrt)}")


# Plots


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

    The crossovers and the printed margins are those of :func:`margins`, read
    on its grid (or on ``w``), not on the ``n`` plotted frequencies.
    ``title`` overrides the figure heading (default ``"Bode Diagram"``).
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )
    w_plot = frequency_grid(A, B, C, D, w, n)

    # |G(jω)| in dB and the unwrapped ∠G(jω) in degrees
    G = linear.frequency_response(A, B, C, D, w_plot)
    with np.errstate(divide="ignore"):
        magnitude_db = 20.0 * np.log10(np.abs(G))
    phase_deg = np.degrees(np.unwrap(np.angle(G)))

    # The margins, bracketed on the grid of margins() and solved on the exact L(jω)
    loop_margins = None
    if margins:
        w_margins = frequency_grid(A, B, C, D, w, _MARGIN_GRID_POINTS)
        G_margins = linear.frequency_response(A, B, C, D, w_margins)
        loop_margins = linear.margins(w_margins, G_margins, loop_response(A, B, C, D))

    figure = bode_figure(
        w_plot, magnitude_db, phase_deg, sys, of, wrt, loop_margins, title=title
    )

    return render_control_figure(figure, backend=backend, show=show)


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
    minimal: bool | None = None,
    radius: float | None = None,
    backend="matplotlib",
    show: bool = True,
) -> PlotResult:
    """Pole-zero map of the selected channel: ``x`` poles, ``o`` zeros.

    ``minimal`` as in :func:`pzmap`. ``radius`` frames the roots within that
    distance of the origin, the window a stiff model needs to show its slow
    roots; the default frames every root.
    """
    z, p, gain = pzmap(
        sys,
        x_bar,
        u_bar,
        t,
        params,
        of=of,
        wrt=wrt,
        method=method,
        eps=eps,
        minimal=minimal,
    )

    figure = pzmap_figure(z, p, sys, of, wrt, radius)

    return render_control_figure(figure, backend=backend, show=show)


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
    minimal: bool | None = None,
    radius: float | None = None,
    backend="matplotlib",
    show: bool = True,
) -> PlotResult:
    """Root locus of the selected channel closed with ``u = -K y``.

    ``minimal`` as in :func:`pzmap`. ``radius`` frames the branches within that
    distance of the origin; the default frames three times the radius of the
    open-loop roots, so the far tails of the asymptotes leave the picture.
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    # Minimal realization: the uncontrollable and unobservable modes cancel
    A, B, C, D = minimal_channel(A, B, C, D, minimal)

    # p(K) = eig(A − B K (1 + K d)⁻¹ C) over the gain sweep
    gains, roots = linear.root_locus(A, B, C, D, gains)

    figure = root_locus_figure(A, B, C, D, gains, roots, sys, of, wrt, radius)

    return render_control_figure(figure, backend=backend, show=show)


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

    figure = nyquist_figure(w, G, sys, of, wrt)

    return render_control_figure(figure, backend=backend, show=show)


###############################################################################
# Internal machinery
###############################################################################


def frequency_grid(A, B, C, D, w, n):
    """The frequencies to evaluate: the user's, or a log grid from the poles and zeros."""
    # By default, n frequencies evenly spaced in log ω across the poles and zeros
    if w is None:
        w_min, w_max = linear.frequency_range(A, B, C, D)
        w = np.logspace(np.log10(w_min), np.log10(w_max), int(n))

    w = np.asarray(w, dtype=float).reshape(-1)
    if w.size == 0 or np.any(w <= 0.0):
        raise ValueError("Frequencies must be positive.")

    return w


def loop_response(A, B, C, D):
    """``w -> L(jw)``, one complex value, for the root solve of the crossovers."""

    def response(w):
        return linear.frequency_response(A, B, C, D, np.atleast_1d(w))[0]

    return response


def minimal_channel(A, B, C, D, minimal):
    """The channel's minimal realization unless ``minimal=False``; the default reports what it drops once."""
    if minimal is False:
        return A, B, C, D

    A_min, B_min, C_min, D_min = linear.minreal(A, B, C, D)
    dropped = A.shape[0] - A_min.shape[0]

    # The default cancels and says how many pairs; minimal=True cancels silently
    if minimal is None and dropped > 0:
        warnings.warn(
            f"Cancelled {dropped} pole/zero pair(s): modes that are uncontrollable or "
            "unobservable on this channel. Pass minimal=False to keep them.",
            stacklevel=caller_stacklevel(),
        )

    return A_min, B_min, C_min, D_min


def caller_stacklevel():
    """``stacklevel`` of the first frame outside minilink, so a warning names the user's line."""
    package = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # One level per frame inside minilink, from this function's caller outward
    frame, level = inspect.currentframe().f_back, 1
    while frame is not None and os.path.abspath(frame.f_code.co_filename).startswith(
        package
    ):
        frame, level = frame.f_back, level + 1

    return level


# Figures


def bode_figure(w, magnitude_db, phase_deg, sys, of, wrt, m, title=None):
    """The Bode figure: magnitude over phase on one log-frequency axis, the margins ``m`` marked when given."""
    hover = tuple(
        f"ω = {wk:.3g} rad/s<br>|G| = {mk:.1f} dB<br>∠G = {pk:.1f}°"
        for wk, mk, pk in zip(w, magnitude_db, phase_deg)
    )

    # With margins: the crossover lines, the margins note, the 0 dB and −180° references
    crossovers, notes, references = (), (), ()
    if m is not None:
        crossovers = tuple(
            RefLine("x", w_c)
            for w_c in (m.w_gain_crossover, m.w_phase_crossover)
            if np.isfinite(w_c)
        )
        notes = (Note(margins_text(m)),)
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


def margins_text(m: Margins) -> str:
    """The margins note: ``Gm`` and ``Pm`` at their crossover frequencies, ``inf`` where none exists."""
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


def pzmap_figure(z, p, sys, of, wrt, radius=None):
    """The pole-zero map figure: poles as ``x``, zeros as ``o``."""
    # The view frames every root, or those within radius of the origin
    points = np.concatenate([z, p])
    if radius is not None:
        points = points[np.abs(points) <= radius]

    return ControlFigure(
        title=style.PZMAP_TITLE,
        subtitle=channel_subtitle(sys, of, wrt),
        panels=(
            Panel(
                traces=root_markers(p, z),
                x_label=style.REAL_LABEL,
                y_label=style.IMAG_LABEL,
                zero_lines=True,
                x_lim=style.padded_limits(np.concatenate([points.real, [0.0]])),
                y_lim=style.padded_limits(np.concatenate([points.imag, -points.imag])),
            ),
        ),
    )


def root_markers(p, z):
    """Poles as ``x`` and zeros as ``o`` in the system colour."""
    return (
        Trace(
            p.real,
            p.imag,
            style.SYSTEM_COLOR,
            mode="markers",
            marker="x",
            hover=tuple(root_text(s) for s in p),
        ),
        Trace(
            z.real,
            z.imag,
            style.SYSTEM_COLOR,
            mode="markers",
            marker="o",
            hover=tuple(root_text(s) for s in z),
        ),
    )


def root_text(s) -> str:
    """The hover text of a root ``s``: its value, damping ratio and natural frequency."""
    # ζ = −Re s / |s| (1 at the origin by convention), ω_n = |s|
    zeta = float(-s.real / abs(s)) if abs(s) > 0.0 else 1.0
    wn = abs(s)

    return f"s = {s.real:.3g} {s.imag:+.3g}j<br>ζ = {zeta:.3f}, ωn = {wn:.3g} rad/s"


def root_locus_figure(A, B, C, D, gains, roots, sys, of, wrt, radius=None):
    """The root-locus figure: one trace per branch, with the open-loop poles and zeros."""
    # The view keeps the poles, zeros and the branches near them; the far tails
    # of the asymptotes leave the frame as they do in MATLAB, or the window
    # within radius of the origin when one is given.
    if radius is None:
        radius = 3.0 * linear.open_loop_radius(
            linear.poles(A), linear.zeros(A, B, C, D)
        )
    reach = float(radius)
    near = roots[np.abs(roots) <= reach]

    # One trace per branch, its gain and root in the hover
    branches = tuple(
        Trace(
            roots[:, j].real,
            roots[:, j].imag,
            style.SYSTEM_COLOR,
            hover=tuple(
                f"K = {k:.3g}<br>" + root_text(s) for k, s in zip(gains, roots[:, j])
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
                + root_markers(linear.poles(A), linear.zeros(A, B, C, D)),
                x_label=style.REAL_LABEL,
                y_label=style.IMAG_LABEL,
                zero_lines=True,
                x_lim=style.padded_limits(np.concatenate([near.real, [0.0]])),
                y_lim=style.padded_limits(np.concatenate([near.imag, -near.imag])),
            ),
        ),
    )


def nyquist_figure(w, G, sys, of, wrt):
    """The Nyquist figure: the contour, its mirror for negative frequencies, and the critical point ``-1``."""
    hover = tuple(
        f"ω = {wk:.3g} rad/s<br>G = {g.real:.3g} {g.imag:+.3g}j" for wk, g in zip(w, G)
    )

    # Zoom on the part of the contour that matters: poles at the origin send
    # |G| to infinity at low frequency, so the limits ignore the far points.
    visible = G[np.abs(G) <= 10.0 * np.median(np.abs(G))]
    points = np.concatenate([visible, np.conj(visible), [-1.0 + 0j]])

    # The critical point −1
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
