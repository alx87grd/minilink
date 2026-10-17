"""Option checks: an option's value among the choices its owner declares."""

###############################################################################
# Public API
###############################################################################


def require_one_of(name, value, choices):
    """Refuse an option outside its known choices, naming them."""
    if value not in choices:
        raise ValueError(f"{name} must be one of {choices}, got {value!r}")
