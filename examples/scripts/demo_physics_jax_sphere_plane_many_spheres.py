"""
Demo: JAX physics world with many spheres at varied initial heights.

This is a larger scene variant of demo_physics_jax_sphere_plane.py.
"""

from __future__ import annotations

import time

import numpy as np

from minilink.compile import compile
from minilink.physics import PhysicsWorldSystem
from minilink.physics.engine_jax import PlaneModel, SphereModel, make_world_model

# Mix of radii/masses for visual and dynamic variety.
specs = [
    (0.25, 1.0),
    (0.30, 1.4),
    (0.22, 0.8),
    (0.35, 1.8),
    (0.28, 1.2),
    (0.18, 0.6),
    (0.32, 1.6),
    (0.24, 0.9),
    (0.27, 1.1),
    (0.20, 0.7),
    (0.34, 1.7),
    (0.26, 1.0),
]
spheres = [SphereModel(mass=m, radius=r) for (r, m) in specs]

# 4x3 grid in XY, varied Z heights.
xy_grid = [
    (-2.0, -1.2),
    (-0.7, -1.3),
    (0.7, -1.1),
    (2.0, -1.4),
    (-2.1, 0.0),
    (-0.6, 0.2),
    (0.8, -0.1),
    (2.2, 0.1),
    (-1.9, 1.3),
    (-0.8, 1.4),
    (0.9, 1.2),
    (2.1, 1.5),
]
z_heights = [1.2, 1.8, 2.4, 3.0, 1.5, 2.1, 2.8, 3.3, 1.9, 2.6, 3.4, 4.0]

world = make_world_model(
    spheres,
    PlaneModel(normal=(0.0, 0.0, 1.0), offset=0.0),
    gravity=(0.0, 0.0, -9.81),
    k_contact=1000.0,
    c_contact=1.0,
)

sys = PhysicsWorldSystem(world, name="PhysicsManySpheres")

# State layout per body: [p(3), q(4), v(3), w(3)].
x0 = np.zeros(sys.n)
for i in range(sys.world.n_bodies):
    base = 13 * i
    x, y = xy_grid[i]
    z = z_heights[i]
    x0[base : base + 3] = [x, y, z]
    x0[base + 3 : base + 7] = [1.0, 0.0, 0.0, 0.0]  # unit quaternion
sys.x0 = x0


sys.compute_trajectory(tf=10.0, show=False, solver="scipy")
sys.animate(renderer="meshcat")


x = np.asarray(sys.x0)
u = np.asarray(sys.get_u_from_input_ports(0.0))

# evaluator = sys.compile(backend="jax", verbose=True)
evaluator = compile(sys, backend="jax", verbose=False)

n_warm = 10
n_iter = 500
for _ in range(n_warm):
    sys.f(x, u, 0.0)
    evaluator.f(x, u, 0.0)

t0 = time.perf_counter()
for _ in range(n_iter):
    sys.f(x, u, 0.0)
t_pure = time.perf_counter() - t0

t0 = time.perf_counter()
for _ in range(n_iter):
    evaluator.f(x, u, 0.0)
t_compiled = time.perf_counter() - t0

print(
    f"Speed ({n_iter} calls): \n"
    f"sys.f (native python):   {1e6 * t_pure / n_iter:.1f} us/call,\n"
    f"evaluator.f (jax jit):   {1e6 * t_compiled / n_iter:.1f} us/call, "
)
