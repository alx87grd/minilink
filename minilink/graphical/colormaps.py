"""Colour maps read in matplotlib: one value's colour, for drawings that colour their own parts."""

###############################################################################
# Public API
###############################################################################


def colormap_rgb(name, value):
    """The ``(r, g, b)`` of the matplotlib colour map ``name`` at ``value`` in ``[0, 1]``."""
    # Lazy import: matplotlib's colour maps load with the first colour read
    from matplotlib import colormaps

    r, g, b, _ = colormaps[name](value)

    return (r, g, b)
