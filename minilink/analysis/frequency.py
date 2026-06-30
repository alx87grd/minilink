"""Frequency-response tools for linearized systems."""

from __future__ import annotations

import numpy as np

from minilink.analysis.linearize import linearize_matrices
from minilink.graphical.common import PlotResult


def bode(
    sys,
    x_bar=None,
    u_bar=None,
    *,
    input_port=None,
    input_index: int = 0,
    output_port=None,
    output_index: int = 0,
    w=None,
    n: int = 200,
    method: str = "fd",
    t: float = 0.0,
    params=None,
    epsilon: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the SISO Bode response ``(w, magnitude_db, phase_deg)``.

    Parameters
    ----------
    sys : System
        System or diagram to linearize before computing the frequency response.
    x_bar : array of shape (n,), optional
        Operating-point state. Defaults to ``sys.x0``.
    u_bar : array of shape (m,), optional
        Full operating-point input. Defaults to the system's nominal port values.
    input_port : str, optional
        Boundary input port selected for the SISO channel. Defaults to the first
        boundary input port.
    input_index : int, optional
        Component inside ``input_port``.
    output_port : str or (str, str), optional
        Boundary output port, or internal diagram ``(sys_id, port_id)`` output.
        Defaults to the standard linearization output.
    output_index : int, optional
        Component inside the selected output.
    w : array, optional
        Frequencies in rad/s. When omitted, a logarithmic grid is chosen from
        the linearized poles.
    n : int, optional
        Number of frequencies for the automatic grid.
    method : {"fd", "jax"}, optional
        Linearization method passed to ``linearize_matrices``.

    Returns
    -------
    w, magnitude_db, phase_deg : tuple of ndarray
        Frequency grid, magnitude in dB, and unwrapped phase in degrees.
    """
    # --- operating point ---
    if x_bar is None:
        x_bar = sys.x0

    # --- SISO channel: one input port/component, one output port/component ---
    if input_port is None:
        if not sys.inputs:
            raise ValueError("Bode analysis requires at least one input port.")
        input_port = next(iter(sys.inputs))

    # Linearize about (x_bar, u_bar):
    #   dDelta x = A Delta x + B Delta u
    #   Delta y  = C Delta x + D Delta u
    A, B, C, D = linearize_matrices(
        sys,
        x_bar,
        u_bar,
        inputs=[input_port],
        outputs=None if output_port is None else [output_port],
        method=method,
        t=t,
        params=params,
        epsilon=epsilon,
    )

    input_index = int(input_index)
    output_index = int(output_index)
    if input_index < 0 or input_index >= B.shape[1]:
        raise ValueError(
            f"input_index must be in [0, {B.shape[1] - 1}] for port {input_port!r}."
        )
    if output_index < 0 or output_index >= C.shape[0]:
        raise ValueError(
            f"output_index must be in [0, {C.shape[0] - 1}] for selected output."
        )

    # Keep matrix shapes: B is (n, 1), C is (1, n), D is (1, 1).
    B = B[:, [input_index]]
    C = C[[output_index], :]
    D = D[[output_index], [input_index]]

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
    *,
    input_port=None,
    input_index: int = 0,
    output_port=None,
    output_index: int = 0,
    method: str = "fd",
    t: float = 0.0,
    params=None,
    epsilon: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, float]:
    """Return the selected SISO channel as ``(zeros, poles, gain)``."""
    # --- operating point ---
    if x_bar is None:
        x_bar = sys.x0

    # --- SISO channel: one input port/component, one output port/component ---
    if input_port is None:
        if not sys.inputs:
            raise ValueError("Pole-zero analysis requires at least one input port.")
        input_port = next(iter(sys.inputs))

    # Linearize about (x_bar, u_bar):
    #   dDelta x = A Delta x + B Delta u
    #   Delta y  = C Delta x + D Delta u
    A, B, C, D = linearize_matrices(
        sys,
        x_bar,
        u_bar,
        inputs=[input_port],
        outputs=None if output_port is None else [output_port],
        method=method,
        t=t,
        params=params,
        epsilon=epsilon,
    )

    input_index = int(input_index)
    output_index = int(output_index)
    if input_index < 0 or input_index >= B.shape[1]:
        raise ValueError(
            f"input_index must be in [0, {B.shape[1] - 1}] for port {input_port!r}."
        )
    if output_index < 0 or output_index >= C.shape[0]:
        raise ValueError(
            f"output_index must be in [0, {C.shape[0] - 1}] for selected output."
        )

    # Keep matrix shapes: B is (n, 1), C is (1, n), D is (1, 1).
    B = B[:, [input_index]]
    C = C[[output_index], :]
    D = D[[output_index], [input_index]]

    if A.size:
        from scipy import signal

        num, den = signal.ss2tf(A, B, C, D)
        num = np.asarray(num[0], dtype=float)
        den = np.asarray(den, dtype=float)

        tol = np.finfo(float).eps * max(num.size, den.size)
        tol *= max(np.max(np.abs(num)), np.max(np.abs(den)), 1.0)
        while num.size > 1 and abs(num[0]) <= tol:
            num = num[1:]

        if np.all(np.abs(num) <= tol):
            zeros = np.array([], dtype=complex)
            poles = np.roots(den)
            gain = 0.0
        else:
            zeros, poles, gain = signal.tf2zpk(num, den)
    else:
        zeros = np.array([], dtype=complex)
        poles = np.array([], dtype=complex)
        gain = D[0, 0]

    return np.asarray(zeros), np.asarray(poles), float(np.real_if_close(gain))


def plot_bode(
    sys,
    x_bar=None,
    u_bar=None,
    *,
    input_port=None,
    input_index: int = 0,
    output_port=None,
    output_index: int = 0,
    w=None,
    n: int = 200,
    method: str = "fd",
    t: float = 0.0,
    params=None,
    epsilon: float = 1e-6,
    backend="matplotlib",
    show: bool = True,
) -> PlotResult:
    """Plot the selected SISO Bode response."""
    if not isinstance(backend, str) or backend.strip().lower() != "matplotlib":
        raise ValueError("Bode plotting currently supports backend='matplotlib'.")

    w, magnitude_db, phase_deg = bode(
        sys,
        x_bar,
        u_bar,
        input_port=input_port,
        input_index=input_index,
        output_port=output_port,
        output_index=output_index,
        w=w,
        n=n,
        method=method,
        t=t,
        params=params,
        epsilon=epsilon,
    )

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

    output_name = output_port
    if output_name is None:
        if "y" in sys.outputs:
            output_name = "y"
        elif sys.outputs:
            output_name = next(iter(sys.outputs))
        else:
            output_name = "x"
    if isinstance(output_name, tuple):
        output_name = f"{output_name[0]}:{output_name[1]}"
    input_name = input_port
    if input_name is None:
        input_name = next(iter(sys.inputs), "input")
    channel = f"{output_name}[{output_index}] / {input_name}[{input_index}]"

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
    *,
    input_port=None,
    input_index: int = 0,
    output_port=None,
    output_index: int = 0,
    method: str = "fd",
    t: float = 0.0,
    params=None,
    epsilon: float = 1e-6,
    backend="matplotlib",
    show: bool = True,
) -> PlotResult:
    """Plot poles and zeros for the selected SISO channel."""
    if not isinstance(backend, str) or backend.strip().lower() != "matplotlib":
        raise ValueError("Pole-zero plotting currently supports backend='matplotlib'.")

    zeros, poles, gain = pzmap(
        sys,
        x_bar,
        u_bar,
        input_port=input_port,
        input_index=input_index,
        output_port=output_port,
        output_index=output_index,
        method=method,
        t=t,
        params=params,
        epsilon=epsilon,
    )

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

    output_name = output_port
    if output_name is None:
        if "y" in sys.outputs:
            output_name = "y"
        elif sys.outputs:
            output_name = next(iter(sys.outputs))
        else:
            output_name = "x"
    if isinstance(output_name, tuple):
        output_name = f"{output_name[0]}:{output_name[1]}"
    input_name = input_port
    if input_name is None:
        input_name = next(iter(sys.inputs), "input")
    channel = f"{output_name}[{output_index}] / {input_name}[{input_index}]"

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
    ax.axhline(0.0, color="black", linewidth=0.8, alpha=0.4)
    ax.axvline(0.0, color="black", linewidth=0.8, alpha=0.4)
    ax.set_xlabel("Real", fontsize=FONT_SIZE)
    ax.set_ylabel("Imaginary", fontsize=FONT_SIZE)
    ax.set_title(f"{channel}\ngain = {gain:.4g}", fontsize=FONT_SIZE)
    style_trajectory_subplot(ax)
    if zeros.size or poles.size:
        ax.legend(loc="best")
    fig.tight_layout()

    if show and plt.get_backend().lower() != "agg":
        plt.show(block=is_blocking_needed())

    return PlotResult(
        backend="matplotlib",
        payload=(fig, ax),
        figure=fig,
        axes=ax,
    )
