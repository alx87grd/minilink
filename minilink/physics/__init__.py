"""
JAX-first physics engine blocks.

MVP provides:
- sphere rigid bodies (6-DoF state),
- fixed plane contact (penalty spring-damper),
- a minilink DynamicSystem wrapper.
"""

from minilink.physics.engine_jax import (
    PlaneModel,
    SphereModel,
    WorldModel,
    make_world_model,
    pack_state,
    unpack_state,
    world_ode,
)
from minilink.physics.system import PhysicsWorldSystem

__all__ = [
    "PlaneModel",
    "PhysicsWorldSystem",
    "SphereModel",
    "WorldModel",
    "make_world_model",
    "pack_state",
    "unpack_state",
    "world_ode",
]
