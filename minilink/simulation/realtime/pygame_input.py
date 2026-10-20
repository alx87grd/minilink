"""Pygame keyboard and gamepad input backend for real-time simulation."""

from __future__ import annotations

import os
import time
import warnings

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

# Right stick (azimuth, elevation) on 6-axis gamepads (Xbox / DualSense).
DEFAULT_CAMERA_AXES = (
    (3, -1.0),
    (4, -1.0),
)

# Wall-clock integration step cap for ``mode="rate"`` so a stalled frame
# cannot slew the command arbitrarily far.
MAX_RATE_ELAPSED_S = 0.1


###############################################################################
# Public API
###############################################################################


class PygameInput(RealtimeInput):
    """
    Keyboard and joystick → ``u`` through the input-port bounds.

    Maps held keys onto the system's flat boundary input vector using each
    input port's ``lower_bound`` / ``upper_bound``: the positive key of an
    axis commands the component's upper bound, the negative key its lower
    bound. Components with non-finite bounds fall back to ``gain``. The first
    connected joystick (if any) overrides the matching key axis with an
    analog deflection.

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
        e.g. ``[(1, 1.0), (0, -1.0)]`` to remap or invert sticks.
    joystick_deadzone : float
        Deflections below this magnitude are ignored (default ``0.1``).
    camera_axes : bool or sequence of (int, float), optional
        Joystick ``[(azimuth_axis, sign), (elevation_axis, sign)]`` for 3-D
        camera orbit (e.g. MeshCat). Defaults to :data:`DEFAULT_CAMERA_AXES`
        (the right stick, axes ``(3, 4)``) when neither axis is used by
        ``joystick_axes``. Pass ``None`` or ``False`` to disable joystick
        camera orbit.
    camera_rate : float
        Camera orbit angular rate at full stick deflection [rad/s]
        (default ``2.0``).
    window : bool or str
        Whether to open a ``640x240`` focus window when no pygame display
        surface exists yet (e.g. with ``renderer="meshcat"`` or
        ``"matplotlib"``). ``"auto"`` (default) skips the window when a
        joystick is connected (since SDL reads the gamepad in the background)
        and opens it only for keyboard-only sessions. Pass ``False`` to never
        open a window, or ``True`` to always open it when no display surface
        exists.

    Notes
    -----
    Display ownership: the renderer opens its window first. When a keyboard
    focus window is needed and no pygame display exists when :meth:`open`
    runs, this input opens a small window so SDL can deliver key events, and
    draws a ``u`` overlay on it. ESC, ``Ctrl+C``, or closing the window stops
    the run.
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
        camera_axes=DEFAULT_CAMERA_AXES,
        camera_rate=2.0,
        window: bool | str = "auto",
    ):
        if mode not in ("hold", "rate"):
            raise ValueError(f"mode must be 'hold' or 'rate', got {mode!r}")
        if not isinstance(window, bool) and window != "auto":
            raise ValueError(f"window must be True, False, or 'auto', got {window!r}")
        self.mode = mode
        self.gain = float(gain)
        self.rate = float(rate)
        self.key_axes = DEFAULT_KEY_AXES if key_axes is None else tuple(key_axes)
        self.joystick = joystick
        self.joystick_axes = None if joystick_axes is None else tuple(joystick_axes)
        self.joystick_deadzone = float(joystick_deadzone)
        if camera_axes is True:
            camera_axes = DEFAULT_CAMERA_AXES
        self.camera_axes = (
            ()
            if not camera_axes
            else tuple((int(ax), float(sign)) for ax, sign in camera_axes[:2])
        )
        self.camera_rate = float(camera_rate)
        self.window = window

        self.pygame = None
        self._screen = None  # focus/overlay window, only when we own the display
        self._owns_pygame = False
        self._font = None
        self._last_drawn_u = None
        self._key_codes = ()
        self._joy = None
        self._num_axes = 0
        self._joy_axes = ()
        self._joy_camera_axes = ()
        self._hold_lower = None
        self._hold_upper = None
        self._clip_lower = None
        self._clip_upper = None
        self._u_cmd = None
        self._last_poll_s = None

    def open(self, sys):
        # Allow SDL2 to read joystick/gamepad axes even when a browser or
        # external renderer window has OS focus.
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        pygame = _import_pygame()
        has_external_surface = (
            pygame.display.get_init() and pygame.display.get_surface() is not None
        )
        self._owns_pygame = not has_external_surface
        pygame.init()
        self.pygame = pygame
        self._screen = None
        self._font = None
        self._last_drawn_u = None

        # On macOS, SDL2's IOHIDManager / GameController driver only enumerates
        # controllers after Cocoa creates a window and pumps the NSRunLoop once.
        # Creating a 1x1 HIDDEN window first lets SDL discover Bluetooth/USB
        # gamepads without showing a window or stealing OS focus.
        opened_hidden_probe = False
        if not has_external_surface and (
            self.joystick or self.joystick_axes is not None
        ):
            hidden_flag = getattr(pygame, "HIDDEN", 0)
            if isinstance(hidden_flag, int) and hidden_flag != 0:
                try:
                    pygame.display.set_mode((1, 1), flags=hidden_flag)
                    opened_hidden_probe = True
                except Exception:
                    pass
            pygame.event.pump()

        lower, upper = input_port_bounds(sys)
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
        self._num_axes = 0
        self._joy_axes = ()
        self._joy_camera_axes = ()
        if self.joystick or self.joystick_axes is not None:
            pygame.joystick.init()
            pygame.event.pump()
            # Give macOS GameController / IOHIDManager runloop a brief window
            # to deliver async controller-connect notifications.
            if (
                opened_hidden_probe or self.joystick_axes is not None
            ) and pygame.joystick.get_count() == 0:
                deadline = time.perf_counter() + 0.15
                while (
                    pygame.joystick.get_count() == 0 and time.perf_counter() < deadline
                ):
                    time.sleep(0.01)
                    pygame.event.pump()
            self.bind_joystick(m)
            if self._joy is None and self.joystick_axes is not None:
                warnings.warn(
                    "PygameInput: joystick_axes was specified, but no joystick "
                    "was detected by pygame; falling back to keyboard.",
                    UserWarning,
                    stacklevel=2,
                )

        # Open a visible 640x240 focus window only when no joystick is
        # connected in "auto" mode (SDL reads gamepads in the background).
        need_window = (
            (self._joy is None) if self.window == "auto" else bool(self.window)
        )
        if need_window and not has_external_surface:
            if opened_hidden_probe:
                pygame.display.quit()
                pygame.display.init()
            self._screen = pygame.display.set_mode((640, 240))
            pygame.display.set_caption(
                f"minilink realtime — keyboard (focus here) — {sys.name}"
            )
            try:
                pygame.font.init()
                self._font = pygame.font.SysFont(None, 28)
            except Exception:
                self._font = None

        u0 = np.asarray(sys.get_u_from_input_ports(), dtype=float)
        self._u_cmd = np.clip(u0, self._clip_lower, self._clip_upper)
        self._last_poll_s = None
        self.draw_overlay(self._u_cmd, force=True)

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
        if self._joy is None and (self.joystick or self.joystick_axes is not None):
            self.bind_joystick(self._hold_lower.size)
        keys = pygame.key.get_pressed()
        direction = np.zeros(self._hold_lower.size)
        for i, (pos, neg) in enumerate(self._key_codes):
            direction[i] = float(bool(keys[pos])) - float(bool(keys[neg]))
        for i, (axis, sign) in enumerate(self._joy_axes):
            if 0 <= axis < self._num_axes:
                value = float(sign) * float(self._joy.get_axis(axis))
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

        self.draw_overlay(u)
        return u, should_stop

    def poll_camera(self, dt: float) -> tuple[float, float]:
        """Return ``(d_azimuth, d_elevation)`` [rad] from the camera joystick axes."""
        if self._joy is None or len(self._joy_camera_axes) < 2:
            return 0.0, 0.0
        (ax_az, sign_az), (ax_el, sign_el) = self._joy_camera_axes[:2]
        v_az = (
            float(sign_az) * float(self._joy.get_axis(ax_az))
            if 0 <= ax_az < self._num_axes
            else 0.0
        )
        v_el = (
            float(sign_el) * float(self._joy.get_axis(ax_el))
            if 0 <= ax_el < self._num_axes
            else 0.0
        )
        d_az = (
            v_az * self.camera_rate * float(dt)
            if abs(v_az) > self.joystick_deadzone
            else 0.0
        )
        d_el = (
            v_el * self.camera_rate * float(dt)
            if abs(v_el) > self.joystick_deadzone
            else 0.0
        )
        return float(d_az), float(d_el)

    def close(self):
        # Quit pygame only when this input owns the SDL session; a pygame
        # renderer shuts SDL down itself in close_scene().
        if self.pygame is not None and self._owns_pygame:
            self.pygame.quit()
        self.pygame = None
        self._screen = None
        self._owns_pygame = False
        self._font = None
        self._last_drawn_u = None
        self._joy = None
        self._num_axes = 0
        self._joy_axes = ()
        self._joy_camera_axes = ()

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def bind_joystick(self, m: int) -> None:
        """Attach the first available SDL joystick and resolve its axis mapping."""
        if self.pygame.joystick.get_count() <= 0:
            return
        self._joy = self.pygame.joystick.Joystick(0)
        self._joy.init()
        num_axes = int(self._joy.get_numaxes())
        self._num_axes = num_axes
        if self.joystick_axes is None:
            self._joy_axes = tuple((i, 1.0) for i in range(min(m, num_axes)))
        else:
            self._joy_axes = tuple(
                (int(ax), float(sign)) for ax, sign in self.joystick_axes[:m]
            )

        used = {ax for ax, _ in self._joy_axes}
        if len(self.camera_axes) == 2 and all(
            0 <= ax < num_axes and ax not in used for ax, _ in self.camera_axes
        ):
            self._joy_camera_axes = self.camera_axes
        else:
            self._joy_camera_axes = ()

    def draw_overlay(self, u, *, force: bool = False):
        """Show focus hint, key axes, and current ``u`` on the focus window."""
        if self._screen is None:
            return
        if (
            not force
            and self._last_drawn_u is not None
            and np.allclose(u, self._last_drawn_u, atol=1e-6, rtol=0.0)
        ):
            return
        self._last_drawn_u = np.asarray(u, dtype=float).copy()
        pygame = self.pygame
        self._screen.fill((45, 45, 50))
        if self._font is not None:
            try:
                lines = ["Focus this window for keyboard input.  ESC: quit"]
                for i, (pos, neg) in enumerate(self.key_axes[: u.size]):
                    lines.append(f"{pos.upper()}/{neg.upper()} -> u[{i}] = {u[i]:+.3g}")
                y = 12
                for line in lines:
                    surf = self._font.render(line, True, (230, 230, 230))
                    self._screen.blit(surf, (12, y))
                    y += 32
            except Exception:
                pass  # font backends vary; the overlay is purely informative
        pygame.display.flip()


###############################################################################
# Internal machinery
###############################################################################


def _import_pygame():
    try:
        import pygame
    except ImportError as e:
        raise ImportError(
            "pygame is required for PygameInput. Install with: pip install pygame"
        ) from e
    return pygame


def input_port_bounds(sys):
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
