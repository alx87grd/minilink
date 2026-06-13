"""
Demo: differentiable JAX ANCF tire-ring falling onto a plane.

The tire is a prototype ANCF ring/cable model: nodes carry absolute positions
and slopes, elastic forces come from JAX-differentiated energy, and contact is
a smooth penalty force against a fixed ground plane.
"""

import jax
import jax.numpy as jnp

from minilink.dynamics.engines.ancf_tire_jax import (
    ANCFTireSystem,
    make_ancf_tire_model,
)

model = make_ancf_tire_model(
    n_nodes=32,
    radius=0.5,
    mass=14.0,
    k_stretch=2.0e4,
    k_bend=25.0,
    k_area=8.0e3,
    k_slope=1.5e3,
    k_contact=7.0e4,
    c_contact=450.0,
    mu_static=0.95,
    mu_dynamic=0.8,
)

sys = ANCFTireSystem(
    model,
    center=(0.0, 0.0, 1.15),
    linear_velocity=(0.0, 0.0, -0.2),
    angular_velocity=(0.0, 18.0, 0.0),
    contact_force_scale=0.001,
    name="ANCFTireFallDemo",
)

traj = sys.compute_trajectory(
    tf=1.0,
    dt=0.0005,
    solver="rk4_fixedsteps",
    compile_backend="jax",
)

# Differentiability smoke check: tangent through one ODE call.
evaluator = sys.compile(backend="jax")
x0 = jnp.asarray(sys.x0)
u0 = jnp.zeros(sys.m)
_, dx_tangent = jax.jvp(
    lambda x: evaluator.f(x, u0, 0.0),
    (x0,),
    (jnp.ones_like(x0) * 1.0e-4,),
)
print(f"JAX JVP norm: {float(jnp.linalg.norm(dx_tangent)):.6g}")

sys.animate(traj, renderer="meshcat", is_3d=True)
