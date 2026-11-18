"""Time-signal colors: the state in blue, the input in red, every other signal from one palette."""

from __future__ import annotations

from dataclasses import dataclass

from minilink.graphical.figure import TAB10

# The state in blue, the input in red, from the one palette
STATE_COLOR = TAB10[0]
INPUT_COLOR = TAB10[3]

# Every other signal: green, orange, purple, olive, brown, pink, cyan, grey
INTERNAL_SIGNAL_COLORS = tuple(TAB10[k] for k in (2, 1, 4, 8, 5, 6, 9, 7))

CORE_INPUT_NAMES = frozenset({"u", "u_cmd"})
CORE_STATE_NAMES = frozenset({"x"})


###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class SignalStyle:
    """Resolved color and alpha for one trace row."""

    color: str
    alpha: float = 1.0
    linewidth: float = 1.5


def color_for_signal(
    name: str,
    *,
    component: int = 0,
    n_components: int = 1,
    internal_index: int | None = None,
) -> SignalStyle:
    """
    Resolve plot color for a signal channel.

    State ``x`` → blue; boundary input ``u`` / ``u_cmd`` → red; all other
    names cycle through :data:`INTERNAL_SIGNAL_COLORS` by ``internal_index``.
    """
    # The hue and the width of the signal's tier
    if is_core_state(name):
        base = STATE_COLOR
        linewidth = 2.0
    elif is_core_input(name):
        base = INPUT_COLOR
        linewidth = 2.0
    else:
        if internal_index is None:
            internal_index = 0
        base = INTERNAL_SIGNAL_COLORS[internal_index % len(INTERNAL_SIGNAL_COLORS)]
        linewidth = 1.5

    # The components of one signal share its hue, told apart by their opacity
    alpha = component_alpha(component, n_components)

    return SignalStyle(color=base, alpha=alpha, linewidth=linewidth)


def is_core_input(name: str) -> bool:
    """Whether ``name`` is the boundary input, ``u`` or ``u_cmd``."""
    return name in CORE_INPUT_NAMES


def is_core_state(name: str) -> bool:
    """Whether ``name`` is the state ``x``."""
    return name in CORE_STATE_NAMES


def is_internal_signal(name: str) -> bool:
    """Whether ``name`` is neither the state nor the boundary input."""
    return not is_core_state(name) and not is_core_input(name)


###############################################################################
# Internal machinery
###############################################################################


def component_alpha(component: int, n_components: int) -> float:
    """The opacity of one component: 1 when it is alone, else from 0.55 for the first to 1 for the last."""
    if n_components <= 1:
        return 1.0

    span = max(n_components - 1, 1)
    alpha = 0.55 + 0.45 * (component / span)

    return alpha
