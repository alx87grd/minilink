"""Matplotlib's show, shared by the planning views."""


###############################################################################
# Public API
###############################################################################


def maybe_show(plt, show, *, block_when_needed=False):
    """
    Show the figures, unless asked not to or on the non-interactive Agg backend.

    ``block_when_needed`` blocks on the show when the process needs it to stay
    alive (a bare script), as the search views do.
    """
    if show and plt.get_backend().lower() != "agg":
        if block_when_needed:
            # Lazy import: the graphics load on first use, not with the core
            from minilink.graphical.common.environment import is_blocking_needed

            plt.show(block=is_blocking_needed())
        else:
            plt.show()
