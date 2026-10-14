"""State observers: blocks that reconstruct the plant state from ``(u, y)``, and their designs.

Teaching imports::

    from minilink.estimation import StateObserver, kalman, luenberger

Placement rule: online, in-the-loop inference lives here; *offline* fitting over
logged data is a verb and lives in ``identification/``. The designs take a linear model
or a plant in (``kalman(plant, Q_n, R_n)``) and return this package's blocks out; their
gains take arrays (``kalman_gain(A, C, Q_n, R_n)``). The LQG box that pairs an observer
with an LQR gain is ``minilink.control.lqg``.
"""

from __future__ import annotations

from minilink.core.facade import lazy_facade

_EXPORTS: dict[str, tuple[str, str]] = {
    "StateObserver": ("minilink.estimation.observer", "StateObserver"),
    "kalman": ("minilink.estimation.design", "kalman"),
    "kalman_gain": ("minilink.estimation.design", "kalman_gain"),
    "luenberger": ("minilink.estimation.design", "luenberger"),
    "luenberger_gain": ("minilink.estimation.design", "luenberger_gain"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
