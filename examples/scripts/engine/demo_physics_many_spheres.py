"""
Demo: JAX physics world with many spheres at varied initial heights.

This is a larger scene variant of demo_physics_jax_sphere_plane.py.
"""

import time

import jax
import numpy as np

from minilink.dynamics.engines.contact_jax import PlaneModel, SphereModel, make_world_model
from minilink.dynamics.engines.world import PhysicsWorldSystem

# Demo controls.
PRINT_COMPILE_REPORT = True  # Print JAX compile timing diagnostics.

# 10x larger scene than the 12-sphere MVP: 12x10 grid = 120 spheres.
nx, ny = 12, 10
n_spheres = nx * ny

# Smooth radius/mass variation for visual diversity.
radii = np.linspace(0.18, 0.35, n_spheres)
masses = np.linspace(0.6, 1.8, n_spheres)
specs = list(zip(radii, masses))
spheres = [SphereModel(mass=m, radius=r) for (r, m) in specs]

# XY layout and varied Z heights.
x_vals = np.linspace(-6.0, 6.0, nx)
y_vals = np.linspace(-4.5, 4.5, ny)
xy_grid = []
for y in y_vals:
    for x in x_vals:
        xy_grid.append((x, y))
z_heights = np.linspace(1.0, 6.0, n_spheres)

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
u = np.asarray(sys.get_u_from_input_ports())

evaluator = sys.compile(backend="jax", verbose=PRINT_COMPILE_REPORT)

n_warm = 10
n_iter = 1000
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


# auto-diff
u = np.zeros(sys.m)
t = 0.0


def fx(x):
    return evaluator.f(x, u, t)


df_dx_func = jax.jacfwd(fx)

x = np.random.randn(sys.n)
df_dx = df_dx_func(x)

df_dx
