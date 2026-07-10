"""Three-tier color policy for time-signal plots."""

from __future__ import annotations

from dataclasses import dataclass

STATE_COLOR = "tab:blue"
INPUT_COLOR = "tab:red"

INTERNAL_SIGNAL_COLORS = (
    "tab:green",
    "tab:orange",
    "tab:purple",
    "tab:olive",
    "tab:brown",
    "tab:pink",
    "tab:cyan",
    "tab:gray",
)

# Matplotlib tab: names → hex for Plotly parity
SIGNAL_COLOR_HEX: dict[str, str] = {
    "blue": "#0000ff",
    "tab:blue": "#1f77b4",
    "tab:red": "#d62728",
    "tab:green": "#2ca02c",
    "tab:orange": "#ff7f0e",
    "tab:purple": "#9467bd",
    "tab:olive": "#bcbd22",
    "tab:brown": "#8c564b",
    "tab:pink": "#e377c2",
    "tab:cyan": "#17becf",
    "tab:gray": "#7f7f7f",
}

CORE_INPUT_NAMES = frozenset({"u", "u_cmd"})
CORE_STATE_NAMES = frozenset({"x"})


@dataclass(frozen=True)
class SignalStyle:
    """Resolved color and alpha for one trace row."""

    color: str
    alpha: float = 1.0
    linewidth: float = 1.5


def is_core_input(name: str) -> bool:
    return name in CORE_INPUT_NAMES


def is_core_state(name: str) -> bool:
    return name in CORE_STATE_NAMES


def is_internal_signal(name: str) -> bool:
    return not is_core_state(name) and not is_core_input(name)


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

    alpha = _component_alpha(component, n_components)
    return SignalStyle(color=base, alpha=alpha, linewidth=linewidth)


def plotly_color(color: str) -> str:
    """Map a matplotlib color name to hex for Plotly."""
    return SIGNAL_COLOR_HEX.get(color, color)


def _component_alpha(component: int, n_components: int) -> float:
    if n_components <= 1:
        return 1.0
    span = max(n_components - 1, 1)
    return 0.55 + 0.45 * (component / span)
