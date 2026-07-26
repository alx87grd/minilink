"""
Live input and output contracts for real-time simulation.

:class:`RealtimeInput` supplies the boundary input vector ``u`` once per
frame (keyboard, joystick, network peer). :class:`RealtimeOutput` publishes
each frame sample to an external consumer (logger, socket, cosimulation
peer). Rendering is **not** a :class:`RealtimeOutput` — live drawing stays on
the :class:`~minilink.graphical.animation.renderers.renderer.AnimationRenderer`
pipeline driven by :class:`~minilink.simulation.realtime.simulator.RealtimeSimulator`.

TODO: User Architectural Review — contracts are a first slice; the first
cosimulation adapter should validate them before they are considered stable.
"""

from abc import ABC, abstractmethod

import numpy as np

# =============================================================================
# Public API — contracts
# =============================================================================


class RealtimeInput(ABC):
    """
    Live source of the boundary input vector ``u`` during a real-time run.

    Lifecycle: :meth:`open` once before the loop starts, :meth:`poll` once
    per frame, :meth:`close` once when the run stops. Only :meth:`poll` is
    abstract; devices without setup/teardown keep the no-op defaults.
    """

    def open(self, sys):
        """Acquire devices or connections before the loop starts (no-op default)."""

    @abstractmethod
    def poll(self, t, x):
        """
        Return ``(u, should_stop)`` for the frame starting at time ``t``.

        Parameters
        ----------
        t : float
            Current simulation time.
        x : np.ndarray
            Current state with shape ``(n,)``.

        Returns
        -------
        u : np.ndarray
            Input vector held over the coming frame (ZOH), shape ``(m,)``.
        should_stop : bool
            True to end the run (device quit, connection closed).
        """

    def close(self):
        """Release devices or connections after the loop stops (no-op default)."""


class RealtimeOutput(ABC):
    """
    External consumer of per-frame samples during a real-time run.

    Lifecycle mirrors :class:`RealtimeInput`: :meth:`open` before the loop,
    :meth:`publish` once per frame, :meth:`close` after the loop.
    """

    def open(self, sys):
        """Acquire connections or files before the loop starts (no-op default)."""

    @abstractmethod
    def publish(self, t, x, u, outputs):
        """
        Publish one frame sample.

        Parameters
        ----------
        t : float
            Simulation time of the sample.
        x : np.ndarray
            State with shape ``(n,)``.
        u : np.ndarray
            Input held over the frame that produced ``x``, shape ``(m,)``.
        outputs : dict of str to np.ndarray
            Named boundary outputs from ``evaluator.outputs(x, u, t)``.
        """

    def close(self):
        """Release connections or files after the loop stops (no-op default)."""


# =============================================================================
# Public API — callable adapters
# =============================================================================


class CallbackInput(RealtimeInput):
    """
    Adapt a plain input law ``u(t, x)`` to the :class:`RealtimeInput` contract.

    The callable never requests a stop; combine with ``max_steps`` or ``tf``
    on the simulator to end the run.
    """

    def __init__(self, fn):
        self.fn = fn

    def poll(self, t, x):
        u = np.asarray(self.fn(t, x), dtype=float)
        return u, False


class CallbackOutput(RealtimeOutput):
    """Adapt a plain callable ``fn(t, x, u, outputs)`` to :class:`RealtimeOutput`."""

    def __init__(self, fn):
        self.fn = fn

    def publish(self, t, x, u, outputs):
        self.fn(t, x, u, outputs)
