"""Bridges to external ecosystems.

Wrappers that let minilink systems talk to other frameworks, and external
models enter minilink as plants::

    from minilink.interfaces import Sys2Gym, SB3Controller

:class:`~minilink.interfaces.gymnasium.Sys2Gym` exposes a system + cost as an
RL environment (``reward = -g * dt``); trained policies come back as
:class:`~minilink.interfaces.gymnasium.SB3Controller` feedback blocks. Names
resolve on first use, so the optional ``gymnasium`` dependency (``pip install
minilink[rl]``) is imported only then.

Placement rule: anything whose job is "talk to another ecosystem" lives
here; homegrown plants — whatever their implementation technology — live in
``dynamics/``.
"""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# name -> (module path, attribute)
_EXPORTS: dict[str, tuple[str, str]] = {
    "Sys2Gym": ("minilink.interfaces.gymnasium", "Sys2Gym"),
    "SB3Controller": ("minilink.interfaces.gymnasium", "SB3Controller"),
    "ProblemEnv": ("minilink.interfaces.gymnasium", "ProblemEnv"),
    "to_gymnasium": ("minilink.interfaces.gymnasium", "to_gymnasium"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
