"""Computational engines that generate plants from scene descriptions.

Distinct from the other two children of ``dynamics/``: ``abstraction/``
holds mathematical bases you subclass, ``catalog/`` holds named plants you
pick, and ``engines/`` holds kernels that *build* a
:class:`~minilink.core.system.System` from a description
(``make_world_model(...)`` -> :class:`~minilink.dynamics.engines.world.PhysicsWorldSystem`).

The boundary test: engine code **executes inside ``f`` during
simulation** (``world_ode`` runs every step). Offline authoring that derives
a plant and then exits — symbolic EoM derivation, data fitting — is a verb
and lives in the tools band (``symbolic/``, ``identification/``). Bridges to
*external* engines (MuJoCo, FMI) belong in ``interfaces/``. Status:
experimental (TRL 1) — math not yet QA-validated.
"""
