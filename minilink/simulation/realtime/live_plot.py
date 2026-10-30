"""Live time-signal plotting output for real-time simulation."""

from __future__ import annotations

from collections import deque

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.trajectory import Trajectory
from minilink.simulation.realtime.io import RealtimeOutput

###############################################################################
# Public API
###############################################################################


class LiveSignalPlotOutput(RealtimeOutput):
    """
    Live time-signal plotter for :class:`~minilink.simulation.realtime.simulator.RealtimeSimulator`.

    Wraps :func:`~minilink.graphical.signals.open_time_signal_plot` with a
    rolling time window and frame-stride throttling so real-time sessions redraw
    smoothly without exceeding the frame budget or stealing keyboard focus.

    Parameters
    ----------
    signals : sequence of str or (System, str), optional
        Signal names to plot; defaults to
        :func:`~minilink.graphical.signals.resolve_plot_signals`.
    window : float, optional
        Rolling time window in seconds (default ``10.0``). Pass ``None`` to
        keep the full trajectory from ``t0``.
    every : int
        Redraw every ``every`` frames (default ``3``, ~10 Hz at 30 FPS).
    backend : str
        Signal plotting backend (``"matplotlib"`` or ``"plotly"``).
    show : bool
        Whether to display the plot window (default ``True``; ``False`` for
        headless testing).
    close_on_stop : bool
        Close the plot window when the real-time run ends (default ``False`` so
        the final window stays visible when the user quits).
    """

    needs_boundary_outputs = False

    def __init__(
        self,
        *,
        signals=None,
        window: float | None = 10.0,
        every: int = 3,
        backend: str = "matplotlib",
        show: bool = True,
        close_on_stop: bool = False,
    ):
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.signals.time_signals import require_signal_backend

        if window is not None and float(window) <= 0.0:
            raise ValueError(f"window must be positive or None, got {window!r}")
        if int(every) < 1:
            raise ValueError(f"every must be >= 1, got {every!r}")

        self.signals = signals
        self.window = None if window is None else float(window)
        self.every = int(every)
        self.backend = require_signal_backend(backend)
        self.show = bool(show)
        self.close_on_stop = bool(close_on_stop)

        # Runtime state initialized in open()
        self.sys = None
        self.resolved_signals: tuple[str, ...] = ()
        self.handle = None
        self.diagram_evaluator = None
        self.internal_keys: tuple[str, ...] = ()
        self.channel_keys: tuple[str, ...] = ()
        self.ts: deque[float] = deque()
        self.xs: deque[np.ndarray] = deque()
        self.us: deque[np.ndarray] = deque()
        self.signal_buffers: dict[str, deque[np.ndarray]] = {}
        self.step_count = 0

    @property
    def fig(self):
        """Underlying figure when the active handle exposes one."""
        return getattr(self.handle, "fig", None)

    @property
    def lines(self):
        """Matplotlib line artists when the active handle exposes them."""
        return getattr(self.handle, "lines", [])

    def open(self, sys, *, x0=None, u0=None, t0: float = 0.0) -> None:
        """Open the live plot window at the initial sample before the loop starts."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.signals import (
            open_time_signal_plot,
            resolve_plot_signals,
        )
        from minilink.graphical.signals.time_signals import (
            normalize_plot_signals,
            system_channel_keys_of,
        )

        self.sys = sys
        raw_signals = (
            resolve_plot_signals(sys) if self.signals is None else self.signals
        )
        self.resolved_signals = normalize_plot_signals(sys, raw_signals)

        # Compile a NumPy diagram evaluator once when top-level internal ports are requested
        self.diagram_evaluator = None
        self.internal_keys = internal_signal_keys_of(sys, self.resolved_signals)
        self.channel_keys = system_channel_keys_of(sys, self.resolved_signals)
        if self.internal_keys:
            self.diagram_evaluator = sys.compile(backend="numpy")

        x_init = np.asarray(
            sys.x0 if x0 is None else x0,
            dtype=float,
        ).reshape(sys.n)
        u_init = np.asarray(
            sys.get_u_from_input_ports() if u0 is None else u0,
            dtype=float,
        ).reshape(sys.m)
        t_init = float(t0)

        self.ts = deque([t_init])
        self.xs = deque([x_init.copy()])
        self.us = deque([u_init.copy()])
        self.signal_buffers = {
            key: deque() for key in (*self.internal_keys, *self.channel_keys)
        }
        self.record_sample(x_init, u_init, t_init)
        self.step_count = 0

        init_traj = self.window_trajectory()
        self.handle = open_time_signal_plot(
            sys,
            init_traj,
            signals=self.resolved_signals,
            backend=self.backend,
            show=self.show,
            pause=0.001,
        )

    def publish(self, t, x, u, outputs=None) -> None:
        """Record one frame sample and redraw the rolling window on every k-th frame."""
        t_val = float(t)
        x_arr = np.asarray(x, dtype=float).reshape(self.sys.n)
        u_arr = np.asarray(u, dtype=float).reshape(self.sys.m)

        self.ts.append(t_val)
        self.xs.append(x_arr.copy())
        self.us.append(u_arr.copy())
        self.record_sample(x_arr, u_arr, t_val)
        self.trim_window(t_val)

        self.step_count += 1
        if self.handle is not None and self.step_count % self.every == 0:
            traj_window = self.window_trajectory()
            title = f"Live signals for {self.sys.name} — t = {t_val:.2f} s"
            self.handle.update(traj_window, title=title)
            flush_live_canvas(self.handle, self.show)

    def close(self) -> None:
        """Release live plot resources when the session stops."""
        if self.handle is None:
            return

        if self.close_on_stop or self.backend == "plotly":
            self.handle.close()

    def window_trajectory(self) -> Trajectory:
        """Return the current rolling-window samples as a :class:`Trajectory`."""
        signals = {
            key: np.column_stack(values) for key, values in self.signal_buffers.items()
        }
        return Trajectory(
            t=np.asarray(self.ts, dtype=float),
            x=np.column_stack(self.xs),
            u=np.column_stack(self.us),
            signals=signals,
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def record_sample(self, x: np.ndarray, u: np.ndarray, t: float) -> None:
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.signals.time_signals import evaluate_system_channel

        if self.diagram_evaluator is not None:
            step_signals = self.diagram_evaluator.compute_internal_signals_dict(x, u, t)
            for key in self.internal_keys:
                self.signal_buffers[key].append(
                    np.asarray(step_signals[key], dtype=float).reshape(-1).copy()
                )
        for key in self.channel_keys:
            self.signal_buffers[key].append(
                evaluate_system_channel(self.sys, key, x, u, t)
            )

    def trim_window(self, t_now: float) -> None:
        if self.window is None:
            return
        cutoff = t_now - self.window
        # Keep at least two samples when available so line plots have a span
        while len(self.ts) > 2 and self.ts[0] < cutoff - 1e-12:
            self.ts.popleft()
            self.xs.popleft()
            self.us.popleft()
            for buf in self.signal_buffers.values():
                buf.popleft()


###############################################################################
# Internal machinery
###############################################################################


def internal_signal_keys_of(sys, resolved_signals: tuple[str, ...]) -> tuple[str, ...]:
    """Top-level ``'sys_id:port_id'`` diagram signals among ``resolved_signals``."""
    if not isinstance(sys, DiagramSystem):
        return ()

    keys = []
    for name in resolved_signals:
        if ":" not in name or "/" in name.split(":", 1)[0]:
            continue
        sys_id, port_id = name.split(":", 1)
        subsystem = sys.subsystems.get(sys_id)
        if subsystem is not None and port_id in subsystem.outputs and name not in keys:
            keys.append(name)

    return tuple(keys)


def flush_live_canvas(handle, show: bool) -> None:
    """Flush pending GUI events without raising the figure or stealing OS focus."""
    if not show:
        return
    fig = getattr(handle, "fig", None)
    canvas = getattr(fig, "canvas", None)
    if canvas is not None:
        canvas.flush_events()
