"""Signal-conditioning filter blocks (placeholder — shelf decided, content planned).

Plant-agnostic filters used anywhere in a diagram: on references,
measurements, or derivatives. Dynamically these are LTI systems, but their
*role* is wiring — which is why they live in ``blocks/`` and build on
:class:`~minilink.dynamics.abstraction.state_space.LTISystem` (the dependency
law allows libraries to use ``dynamics/abstraction`` interfaces).

Planned blocks: ``LowPassFilter``, ``NotchFilter``, ``Washout`` — likely as
thin :class:`~minilink.blocks.transfer_function.TransferFunction` subclasses
or factories.
"""
