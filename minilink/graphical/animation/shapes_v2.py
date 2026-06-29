"""Deprecated transitional shim — honest arrows now live in ``primitives``.

``ArrowV2`` / ``TorqueArrowV2`` were folded into
:class:`~minilink.graphical.animation.primitives.Arrow` /
:class:`~minilink.graphical.animation.primitives.TorqueArrow` at the Phase 5
cutover. This module re-exports them under the old names only to keep any
straggling imports working; new code should import from ``primitives`` directly.
"""

from minilink.graphical.animation.primitives import (
    Arrow as ArrowV2,
    TorqueArrow as TorqueArrowV2,
    arrow_pts,
    torque_arc_pts,
)

__all__ = ["ArrowV2", "TorqueArrowV2", "arrow_pts", "torque_arc_pts"]
