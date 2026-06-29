"""Shared helpers for the real-time interactive ``game()`` loop.

The keyboard-input mapping and the pygame focus/overlay window are pipeline
agnostic — they take an input dimension and a pygame handle, never a system or
a renderer — so :class:`~minilink.graphical.animation.animator.Animator` can
reuse them for the interactive ``game()`` mode.

Roadmap (``ROADMAP.md`` §7): pluggable **input** backends (keyboard vs TCP
cosimulation) and optional **live output push** should sit beside this keyboard
path as swappable sources/sinks.
"""

from __future__ import annotations

import numpy as np


def u_from_keyboard(keys, *, m: int, pygame) -> np.ndarray:
    """Map arrow-key state to a signed input vector ``u``.

    This is the **pygame-keyboard** live-input path only.

    MVP mapping:
    - UP / DOWN control ``u[0]`` as +50 / -10 (opposites cancel to 0).
    - LEFT / RIGHT control ``u[1]`` as -10 / +10 (opposites cancel to 0).
    - For ``m < 2``, only ``u[0]`` is used.
    """
    u = np.zeros(m, dtype=float)

    if m >= 1:
        up = bool(keys[pygame.K_UP])
        down = bool(keys[pygame.K_DOWN])
        if up and not down:
            u[0] = 50.0
        elif down and not up:
            u[0] = -10.0

    if m >= 2:
        right = bool(keys[pygame.K_RIGHT])
        left = bool(keys[pygame.K_LEFT])
        if right and not left:
            u[1] = -10.0
        elif left and not right:
            u[1] = +10.0

    return u


def draw_keyboard_input_overlay(screen, pygame, u: np.ndarray) -> None:
    """Show focus hint and current ``u`` on the dedicated pygame input window."""
    if screen is None:
        return
    screen.fill((45, 45, 50))
    try:
        pygame.font.init()
        font = pygame.font.SysFont(None, 28)
        lines = [
            "Focus this window for arrow keys.",
            "UP/DOWN -> u[0] (+1 / -1), LEFT/RIGHT -> u[1] (-1 / +1)",
            "ESC: quit",
        ]
        if u.size >= 1:
            lines.append(f"u[0] = {u[0]:.3g}")
        if u.size >= 2:
            lines.append(f"u[1] = {u[1]:.3g}")
        y = 12
        for line in lines:
            surf = font.render(line, True, (230, 230, 230))
            screen.blit(surf, (12, y))
            y += 32
    except Exception:
        pass
    pygame.display.flip()
