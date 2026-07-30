"""System analysis tools.

Verbs that *characterize* a system; they return data, ``LTISystem`` models,
or plots — never user-facing system classes (factories are fine).

Teaching imports::

    from minilink.analysis import bode, modal_analysis
    from minilink.analysis.linearize import linearize
    from minilink.analysis.discretize import discretize

Symbols whose **name matches a submodule** (``linearize``, ``discretize``) are
imported from that submodule — not re-exported on the package attribute.

Implemented modules:

- ``linearize.py`` — equilibrium linearization → matrices or ``LTISystem``
- ``structural.py`` — controllability / observability
- ``equilibria.py`` — trim points and root-finding on ``f``
- ``modal.py`` — ``modal_analysis`` (poles, modes) and ``animate_modal``
- ``frequency.py`` — selected-channel Bode / pole-zero response and plots
- ``discretize.py`` — continuous→step plant wrappers

Planned additions (see ROADMAP.md teaching-release priorities):

- ``frequency.py`` — Nyquist, gain/phase margins (pole-zero / Bode landed)
- ``time_response.py`` — step/impulse sugar over the simulator

Placement rule: if it *characterizes* an existing system, it belongs here;
if it *is* a block you wire into a diagram, it belongs in a library package.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

# Only names that do not collide with submodule filenames.
_EXPORTS: dict[str, tuple[str, str]] = {
    "StructuralResult": ("minilink.analysis.structural", "StructuralResult"),
    "animate_modal": ("minilink.analysis.modal", "animate_modal"),
    "bode": ("minilink.analysis.frequency", "bode"),
    "controllability": ("minilink.analysis.structural", "controllability"),
    "find_equilibrium": ("minilink.analysis.equilibria", "find_equilibrium"),
    "modal_analysis": ("minilink.analysis.modal", "modal_analysis"),
    "observability": ("minilink.analysis.structural", "observability"),
    "plot_bode": ("minilink.analysis.frequency", "plot_bode"),
    "plot_pzmap": ("minilink.analysis.frequency", "plot_pzmap"),
    "pzmap": ("minilink.analysis.frequency", "pzmap"),
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str) -> Any:
    try:
        module_path, attr = _EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from exc
    value = getattr(import_module(module_path), attr)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
