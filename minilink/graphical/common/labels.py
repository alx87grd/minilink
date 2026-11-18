"""Axis labels: a label with its unit, and a state coordinate's label."""

###############################################################################
# Public API
###############################################################################


def label_with_unit(label, unit, separator=" ") -> str:
    """``label``, ``separator``, then its unit in brackets; ``[m]`` stays ``[m]``, no unit adds nothing."""
    if not unit:
        return label
    unit = str(unit).strip()
    if unit.startswith("[") and unit.endswith("]"):
        return f"{label}{separator}{unit}"

    return f"{label}{separator}[{unit}]"


def state_axis_label(sys, i) -> str:
    """State ``i``'s label with its unit, ``x[i]`` where the state names none."""
    labels = list(sys.state.labels)
    units = list(sys.state.units)
    label = str(labels[i]) if i < len(labels) else f"x[{i}]"
    unit = units[i] if i < len(units) else ""

    return label_with_unit(label, unit)
