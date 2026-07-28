"""
Pygame keyboard input backend for real-time simulation.

:class:`PygameInput` maps held keys onto the system's flat boundary input
vector using each input port's ``lower_bound`` / ``upper_bound``: the
positive key of an axis commands the component's upper bound, the negative
key its lower bound. Components with non-finite bounds fall back to a
configurable ``gain``. The first connected joystick (if any) overrides the
matching key axis with an analog deflection.
"""

from __future__ import annotations

import time

import numpy as np

from minilink.simulation.realtime.io import RealtimeInput

# (positive, negative) pygame key names assigned to input components in flat
# ``u`` order: arrows first, then WASD-style pairs for higher dimensions.
DEFAULT_KEY_AXES = (
    ("up", "down"),
    ("right", "left"),
    ("w", "s"),
    ("d", "a"),
    ("i", "k"),
    ("l", "j"),
)

# Wall-clock integration step cap for ``mode="rate"`` so a stalled frame
# cannot slew the command arbitrarily far.
MAX_RATE_ELAPSED_S = 0.1


def _import_pygame():
    try:
        import pygame
    except ImportError as e:
        raise ImportError(
            "pygame is required for PygameInput. Install with: "
            "pip install 'minilink[visualization]'"
        ) from e
    return pygame


class PygameInput(RealtimeInput):
    """
    Keyboard → ``u`` through the input-port bounds.

    Parameters
    ----------
    mode : str
        ``"hold"`` (default): a held key commands the port bound, release
        returns the component to 0 — natural for forces and torques.
        ``"rate"``: a held key slews the commanded value at ``rate`` units
        per wall-clock second, clipped to the port bounds; release holds the
        last value — natural for setpoints.
    gain : float
        Command magnitude for components whose port bound is non-finite
        (default ``1.0``).
    rate : float
        Slew rate in units per second for ``mode="rate"`` (default ``1.0``).
    key_axes : sequence of (str, str), optional
        ``(positive_key, negative_key)`` pygame key names per input
        component, in flat ``u`` order. Defaults to :data:`DEFAULT_KEY_AXES`
        (UP/DOWN drives ``u[0]``, RIGHT/LEFT drives ``u[1]``, then WASD).
    joystick : bool
        Use the first connected joystick when available (default ``True``;
        silently falls back to keyboard-only when none is connected). Stick
        deflection scales the command analogically between 0 and the port
        bound, and overrides the key axis of the same component.
    joystick_axes : sequence of (int, float), optional
        ``(axis_index, sign)`` per input component, in flat ``u`` order.
        Defaults to axis ``i`` with sign ``+1`` for component ``i``; pass
        e.g. ``[(1, -1.0), (0, 1.0)]`` to remap or invert sticks.
    joystick_deadzone : float
        Deflections below this magnitude are ignored (default ``0.1``).

    Notes
    -----
    Display ownership: the renderer opens its window first. If no pygame
    display exists when :meth:`open` runs (e.g. meshcat or matplotlib view),
    this input opens a small focus window so SDL can deliver key events, and
    draws a ``u`` overlay on it. ESC or closing the window stops the run.
    """

    def __init__(
        self,
        *,
        mode="hold",
        gain=1.0,
        rate=1.0,
        key_axes=None,
        joystick=True,
        joystick_axes=None,
        joystick_deadzone=0.1,
    ):
        if mode not in ("hold", "rate"):
            raise ValueError(f"mode must be 'hold' or 'rate', got {mode!r}")
        self.mode = mode
        self.gain = float(gain)
        self.rate = float(rate)
        self.key_axes = DEFAULT_KEY_AXES if key_axes is None else tuple(key_axes)
        self.joystick = joystick
        self.joystick_axes = None if joystick_axes is None else tuple(joystick_axes)
        self.joystick_deadzone = float(joystick_deadzone)

        self.pygame = None
        self._screen = None  # focus/overlay window, only when we own the display
        self._key_codes = ()
        self._joy = None
        self._joy_axes = ()
        self._hold_lower = None
        self._hold_upper = None
        self._clip_lower = None
        self._clip_upper = None
        self._u_cmd = None
        self._last_poll_s = None

    def open(self, sys):
        pygame = _import_pygame()
        pygame.init()
        self.pygame = pygame

        # A 1x1 hidden surface is not focusable on many platforms, so use a
        # small visible window when the renderer did not open a display.
        if pygame.display.get_surface() is None:
            self._screen = pygame.display.set_mode((640, 240))
            pygame.display.set_caption(
                f"minilink realtime — keyboard (focus here) — {sys.name}"
            )

        lower, upper = _input_port_bounds(sys)
        # Hold commands need finite targets, so non-finite bounds fall back to
        # ±gain; rate commands only clip, so raw (possibly infinite) bounds apply.
        self._hold_lower = np.where(np.isfinite(lower), lower, -self.gain)
        self._hold_upper = np.where(np.isfinite(upper), upper, +self.gain)
        self._clip_lower = lower
        self._clip_upper = upper

        m = lower.size
        if m > len(self.key_axes):
            raise ValueError(
                f"{m} input components but only {len(self.key_axes)} key axes; "
                "pass key_axes=[(positive, negative), ...] covering every component."
            )
        self._key_codes = tuple(
            (pygame.key.key_code(pos), pygame.key.key_code(neg))
            for pos, neg in self.key_axes[:m]
        )

        self._joy = None
        if self.joystick:
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self._joy = pygame.joystick.Joystick(0)
                self._joy.init()
        if self._joy is not None:
            if self.joystick_axes is None:
                self._joy_axes = tuple(
                    (i, 1.0) for i in range(min(m, self._joy.get_numaxes()))
                )
            else:
                self._joy_axes = tuple(self.joystick_axes[:m])

        u0 = np.asarray(sys.get_u_from_input_ports(), dtype=float)
        self._u_cmd = np.clip(u0, self._clip_lower, self._clip_upper)
        self._last_poll_s = None

    def poll(self, t, x):
        pygame = self.pygame
        should_stop = False
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                should_stop = True
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                should_stop = True

        # Keys give a digital ±1 direction; a joystick overrides the same
        # component with an analog deflection in [-1, 1].
        keys = pygame.key.get_pressed()
        direction = np.zeros(self._hold_lower.size)
        for i, (pos, neg) in enumerate(self._key_codes):
            direction[i] = float(bool(keys[pos])) - float(bool(keys[neg]))
        for i, (axis, sign) in enumerate(self._joy_axes):
            value = sign * self._joy.get_axis(axis)
            if abs(value) > self.joystick_deadzone:
                direction[i] = value

        if self.mode == "hold":
            # ±1 commands the bound exactly; analog deflection scales toward it.
            u = np.where(
                direction > 0.0,
                direction * self._hold_upper,
                -direction * self._hold_lower,
            )
        else:
            now = time.perf_counter()
            if self._last_poll_s is None:
                elapsed = 0.0
            else:
                elapsed = min(now - self._last_poll_s, MAX_RATE_ELAPSED_S)
            self._last_poll_s = now
            self._u_cmd = np.clip(
                self._u_cmd + direction * self.rate * elapsed,
                self._clip_lower,
                self._clip_upper,
            )
            u = self._u_cmd.copy()

        self._draw_overlay(u)
        return u, should_stop

    def close(self):
        # Quit pygame only when this input opened the display; a pygame
        # renderer shuts SDL down itself in close_scene().
        if self.pygame is not None and self._screen is not None:
            self.pygame.quit()
        self.pygame = None
        self._screen = None

    # Internal machinery

    def _draw_overlay(self, u):
        """Show focus hint, key axes, and current ``u`` on the focus window."""
        if self._screen is None:
            return
        pygame = self.pygame
        self._screen.fill((45, 45, 50))
        try:
            pygame.font.init()
            font = pygame.font.SysFont(None, 28)
            lines = ["Focus this window for keyboard input.  ESC: quit"]
            for i, (pos, neg) in enumerate(self.key_axes[: u.size]):
                lines.append(f"{pos.upper()}/{neg.upper()} -> u[{i}] = {u[i]:+.3g}")
            y = 12
            for line in lines:
                surf = font.render(line, True, (230, 230, 230))
                self._screen.blit(surf, (12, y))
                y += 32
        except Exception:
            pass  # font backends vary; the overlay is purely informative
        pygame.display.flip()


def _input_port_bounds(sys):
    """Concatenated ``(lower, upper)`` bounds across input ports, flat ``u`` order."""
    if sys.m == 0:
        return np.zeros(0), np.zeros(0)
    lower = np.concatenate(
        [np.asarray(port.lower_bound, dtype=float) for port in sys.inputs.values()]
    )
    upper = np.concatenate(
        [np.asarray(port.upper_bound, dtype=float) for port in sys.inputs.values()]
    )
    return lower, upper
