"""Option checks: an option's value among the choices its owner declares, and a warning that points at the caller's line."""

import sys

###############################################################################
# Public API
###############################################################################


def require_one_of(name, value, choices):
    """Refuse an option outside its known choices, naming them."""
    if value not in choices:
        raise ValueError(f"{name} must be one of {choices}, got {value!r}")


def caller_stacklevel() -> int:
    """The ``stacklevel`` of the first caller outside the library, so a warning points at the line that can act on it."""
    level = 1
    frame = sys._getframe(1)
    while frame is not None and frame.f_globals.get("__name__", "").startswith(
        "minilink."
    ):
        level += 1
        frame = frame.f_back

    return level
