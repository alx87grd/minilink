"""Deprecated façade for parametric direct-collocation transcription.

Prefer
:class:`~minilink.planning.trajectory_optimization.direct_collocation.DirectCollocationTranscription`
directly; it now owns :meth:`~DirectCollocationTranscription.transcribe_parametric`.
"""

from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationTranscription,
)


class MPCDirectCollocationTranscription(DirectCollocationTranscription):
    """
    Compatibility alias for
    :class:`~minilink.planning.trajectory_optimization.direct_collocation.DirectCollocationTranscription`.

    Parametric ``x0`` support lives on the base class via
    :meth:`~DirectCollocationTranscription.transcribe_parametric`.
    """
