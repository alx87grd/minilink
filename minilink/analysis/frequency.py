"""Frequency-domain analysis of one input–output channel of a linearized model.

``bode``, ``pzmap`` and ``transfer_function`` linearize ``sys`` about
``(x_bar, u_bar)`` with the same arguments as
:func:`~minilink.analysis.linearize.linearize` and then look at one SISO
channel: ``of`` names the output (a port id, a diagram wire ``"block:port"``,
or ``(selector, index)`` for one component; a bare id means component 0) and
``wrt`` the input the same way. The defaults are component 0 of the primary
output and of the first input port.
"""

from __future__ import annotations

import numpy as np

from minilink.analysis.linearize import linearize_matrices, output_selectors
from minilink.graphical.common import PlotResult


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
        Frequencies in rad/s. When omitted, a logarithmic grid is chosen from
        the linearized poles.
    n : int, optional
        Number of frequencies for the automatic grid.
    method : {"auto", "fd", "jax"}, optional
        Differentiation backend, see :func:`~minilink.analysis.derivatives.jacobian`.
    eps : float, optional
        Central-difference step.

    Returns
    -------
    w, magnitude_db, phase_deg : tuple of ndarray
        Frequency grid, magnitude in dB, and unwrapped phase in degrees.
    """
    A, B, C, D = siso_matrices(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )

    # --- frequency grid omega [rad/s] ---
    if w is None:
        poles = np.linalg.eigvals(A) if A.size else np.array([])
        rates = np.abs(poles[np.abs(poles) > 0.0])
        if rates.size:
            wmin = 10.0 ** np.floor(np.log10(np.min(rates)) - 2.0)
            wmax = 10.0 ** np.ceil(np.log10(np.max(rates)) + 2.0)
        else:
            wmin, wmax = 1e-2, 1e2
        w = np.logspace(np.log10(wmin), np.log10(wmax), int(n))
    else:
        w = np.asarray(w, dtype=float).reshape(-1)

    if w.size == 0 or np.any(w <= 0.0):
        raise ValueError("Bode frequencies must be positive.")

    # --- transfer function G(j omega) = C (j omega I - A)^-1 B + D ---
    if A.size:
        I = np.eye(A.shape[0])
        G = np.empty(w.size, dtype=complex)
        for k, omega in enumerate(w):
            G[k] = (C @ np.linalg.solve(1j * omega * I - A, B) + D)[0, 0]
    else:
        G = np.full(w.shape, D[0, 0], dtype=complex)  # static: G = D

    # --- Bode coordinates: |G| in dB, arg(G) in degrees ---
    with np.errstate(divide="ignore"):
        magnitude_db = 20.0 * np.log10(np.abs(G))
    phase_deg = np.unwrap(np.angle(G)) * 180.0 / np.pi

    return w, magnitude_db, phase_deg


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

    Same arguments as :func:`bode` without the frequency grid.
    """
    num, den = siso_numden(
        *siso_matrices(
            sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
        )
    )
    tol = np.finfo(float).eps * max(num.size, den.size)
    tol *= max(np.max(np.abs(num)), np.max(np.abs(den)), 1.0)
    if den.size == 1:  # static channel: pure gain
        return np.array([], dtype=complex), np.array([], dtype=complex), num[0] / den[0]
    if np.all(np.abs(num) <= tol):
        return np.array([], dtype=complex), np.roots(den), 0.0

    from scipy import signal

    zeros, poles, gain = signal.tf2zpk(num, den)
    return np.asarray(zeros), np.asarray(poles), float(np.real_if_close(gain))


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

    Same arguments as :func:`bode` without the frequency grid. The block is
    the state-space realization of ``num(s) / den(s)`` and carries
    ``numerator``, ``denominator``, ``poles`` and ``zeros``; it can be plotted,
    simulated, or wired like any other block.
    """
    from minilink.blocks.transfer_function import TransferFunction

    num, den = siso_numden(
        *siso_matrices(
            sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
        )
    )
    label = channel_label(sys, of, wrt)
    return TransferFunction(num, den, name=f"{sys.name} {label}")


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
    method: str = "auto",
    eps: float = 1e-6,
    backend="matplotlib",
    show: bool = True,
) -> PlotResult:
    """Plot the selected SISO Bode response (arguments as :func:`bode`)."""
    if not isinstance(backend, str) or backend.strip().lower() != "matplotlib":
        raise ValueError("Bode plotting currently supports backend='matplotlib'.")

    w, magnitude_db, phase_deg = bode(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, w=w, n=n, method=method, eps=eps
    )
    channel = channel_label(sys, of, wrt)

    import matplotlib
    import matplotlib.pyplot as plt

    from minilink.graphical.common.environment import is_blocking_needed
    from minilink.graphical.common.matplotlib_style import (
        DPI_FIGURE,
        FIGSIZE_BASE,
        FONT_SIZE,
        style_trajectory_subplot,
    )

    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["ps.fonttype"] = 42

    fig, axes = plt.subplots(
        2,
        1,
        figsize=FIGSIZE_BASE,
        sharex=True,
        frameon=True,
        dpi=DPI_FIGURE,
    )
    axes = list(axes)
    manager = getattr(fig.canvas, "manager", None)
    set_window_title = getattr(manager, "set_window_title", None)
    if callable(set_window_title):
        set_window_title(f"Bode plot of {sys.name}")

    axes[0].semilogx(w, magnitude_db, linewidth=1.5)
    axes[1].semilogx(w, phase_deg, linewidth=1.5)
    axes[0].set_ylabel(f"{channel}\n[dB]", fontsize=FONT_SIZE)
    axes[1].set_ylabel("Phase [deg]", fontsize=FONT_SIZE)
    axes[1].set_xlabel("Frequency [rad/s]", fontsize=FONT_SIZE)
    for ax in axes:
        style_trajectory_subplot(ax)

    fig.tight_layout()

    if show and plt.get_backend().lower() != "agg":
        plt.show(block=is_blocking_needed())

    return PlotResult(
        backend="matplotlib",
        payload=(fig, axes),
        figure=fig,
        axes=axes,
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
    """Plot poles and zeros of the selected SISO channel (arguments as :func:`pzmap`)."""
    if not isinstance(backend, str) or backend.strip().lower() != "matplotlib":
        raise ValueError("Pole-zero plotting currently supports backend='matplotlib'.")

    zeros, poles, gain = pzmap(
        sys, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
    )
    channel = channel_label(sys, of, wrt)

    import matplotlib
    import matplotlib.pyplot as plt

    from minilink.graphical.common.environment import is_blocking_needed
    from minilink.graphical.common.matplotlib_style import (
        DPI_FIGURE,
        FIGSIZE_BASE,
        FONT_SIZE,
        style_trajectory_subplot,
    )

    matplotlib.rcParams["pdf.fonttype"] = 42
    matplotlib.rcParams["ps.fonttype"] = 42

    fig, ax = plt.subplots(figsize=FIGSIZE_BASE, frameon=True, dpi=DPI_FIGURE)
    manager = getattr(fig.canvas, "manager", None)
    set_window_title = getattr(manager, "set_window_title", None)
    if callable(set_window_title):
        set_window_title(f"Pole-zero map of {sys.name}")

    if zeros.size:
        ax.plot(
            zeros.real,
            zeros.imag,
            marker="o",
            linestyle="none",
            fillstyle="none",
            label="zeros",
        )
    if poles.size:
        ax.plot(
            poles.real,
            poles.imag,
            marker="x",
            linestyle="none",
            label="poles",
        )
    ax.axhline(0.0, color="0.7", linewidth=0.8)
    ax.axvline(0.0, color="0.7", linewidth=0.8)
    ax.set_xlabel("Real", fontsize=FONT_SIZE)
    ax.set_ylabel("Imaginary", fontsize=FONT_SIZE)
    ax.set_title(f"{channel}   gain = {gain:.4g}", fontsize=FONT_SIZE)
    if zeros.size or poles.size:
        ax.legend(fontsize=FONT_SIZE)
    style_trajectory_subplot(ax)
    fig.tight_layout()

    if show and plt.get_backend().lower() != "agg":
        plt.show(block=is_blocking_needed())

    return PlotResult(backend="matplotlib", payload=(fig, ax), figure=fig, axes=ax)


# Channel helpers


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


def channel_label(sys, of, wrt):
    """``"y[1] / u[0]"`` for the selected channel."""
    (of_name, i), (wrt_name, j) = siso_channel(sys, of, wrt)
    return f"{'x' if of_name is None else of_name}[{i}] / {wrt_name}[{j}]"


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


def siso_numden(A, B, C, D):
    """Polynomial numerator and denominator of the SISO channel ``(A, b, c, d)``."""
    if not A.size:
        return np.array([float(D[0, 0])]), np.array([1.0])

    from scipy import signal

    num, den = signal.ss2tf(A, B, C, D)
    num = np.asarray(num[0], dtype=float)
    den = np.asarray(den, dtype=float)

    tol = np.finfo(float).eps * max(num.size, den.size)
    tol *= max(np.max(np.abs(num)), np.max(np.abs(den)), 1.0)
    while num.size > 1 and abs(num[0]) <= tol:
        num = num[1:]
    return num, den
