"""Engine-heavy synthetic systems for benchmark workloads."""

from __future__ import annotations

import numpy as np


def make_physics_many_spheres(nx=6, ny=4):
    """Build a reduced many-spheres world for speed/precision sweeps."""
    from minilink.physics.engine_jax import PlaneModel, SphereModel, make_world_model
    from minilink.physics.system import PhysicsWorldSystem

    n_spheres = nx * ny
    radii = np.linspace(0.18, 0.35, n_spheres)
    masses = np.linspace(0.6, 1.8, n_spheres)
    specs = list(zip(radii, masses))
    spheres = [SphereModel(mass=m, radius=r) for (r, m) in specs]

    x_vals = np.linspace(-6.0, 6.0, nx)
    y_vals = np.linspace(-4.5, 4.5, ny)
    xy_grid = [(x, y) for y in y_vals for x in x_vals]
    z_heights = np.linspace(1.0, 6.0, n_spheres)

    world = make_world_model(
        spheres,
        PlaneModel(normal=(0.0, 0.0, 1.0), offset=0.0),
        gravity=(0.0, 0.0, -9.81),
        k_contact=1000.0,
        c_contact=1.0,
    )

    sys = PhysicsWorldSystem(world, name="PhysicsManySpheres")
    x0 = np.zeros(sys.n)
    for i in range(sys.world.n_bodies):
        base = 13 * i
        x, y = xy_grid[i]
        z = z_heights[i]
        x0[base : base + 3] = [x, y, z]
        x0[base + 3 : base + 7] = [1.0, 0.0, 0.0, 0.0]
    sys.x0 = x0
    return sys
