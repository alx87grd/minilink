"""Signal routing and combination blocks (placeholder — shelf decided, content planned).

Plant-agnostic plumbing for multi-signal diagrams.

Planned blocks: ``Sum`` (signed junction), ``Mux`` / ``Demux`` (stack and
split vector signals), ``Switch`` (select a source), ``Mixer`` (matrix gain
mapping, e.g. quadrotor motor mixing).

All are :class:`~minilink.core.system.StaticSystem` subclasses with explicit
named ports; dimensions are fixed at construction so diagram wiring stays
validated at connect time.
"""
