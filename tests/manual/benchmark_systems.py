"""Reusable system builders for manual performance benchmarks."""

from __future__ import annotations

import numpy as np
import jax.numpy as jnp

from minilink.core.diagram import DiagramSystem
from minilink.core.framework import DynamicSystem, System


class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, m=1, p=2)
        self.name = "Pendulum"

    def f(self, x, u, t=0, params=None):
        gravity = 9.81
        length = 1.0
        damping = 0.0
        q, dq = x[0], x[1]
        ddq = -(gravity / length) * jnp.sin(q) - damping * dq + u[0]
        return jnp.array([dq, ddq])


class SimpleGain(System):
    def __init__(self, id_str, gain=2.0):
        super().__init__(0, 1, 1)
        self.name = id_str
        self.gain = gain
        self.add_input_port(1, "u")
        self.add_output_port(1, "y", function=self.h, dependencies=["u"])

    def h(self, x, u, t=0, params=None):
        return u * self.gain


class SimpleIntegrator(System):
    def __init__(self, id_str):
        super().__init__(1, 1, 1)
        self.name = id_str
        self.add_input_port(1, "u")
        self.add_output_port(1, "x", function=self.compute_state, dependencies="")

    def compute_state(self, x, u, t=0, params=None):
        return x

    def f(self, x, u, t=0, params=None):
        return u


class MultiInputNode(System):
    def __init__(self, id_str, in_ports):
        super().__init__(1, in_ports, 1)
        self.name = id_str
        self.in_ports = in_ports
        for p in range(in_ports):
            self.add_input_port(1, f"u{p}")
        self.add_output_port(1, "x", function=self.compute_state, dependencies="all")

    def compute_state(self, x, u, t=0, params=None):
        return x

    def f(self, x, u, t=0, params=None):
        return u.sum()


def make_pendulum():
    sys = Pendulum()
    sys.x0[0] = 2.0
    return sys


def make_dense_network(num_nodes=50, connections_per_node=5, seed=42):
    """Build a dense feed-forward diagram benchmark."""
    diag = DiagramSystem()
    diag.graphe_building_verbose = False
    for i in range(num_nodes):
        diag.add_subsystem(SimpleIntegrator(f"Node{i}"), f"Node{i}")

    np.random.seed(seed)
    for i in range(1, num_nodes):
        num_conn = min(i, connections_per_node)
        sources = np.random.choice(range(i), size=num_conn, replace=False)

        sys_id = f"MultiNode{i}"
        diag.add_subsystem(MultiInputNode(sys_id, num_conn), sys_id)

        for p_idx, src_i in enumerate(sources):
            diag.connect(f"Node{src_i}", "x", sys_id, f"u{p_idx}")

        diag.connect(sys_id, "x", f"Node{i}", "u")

    diag.add_subsystem(SimpleGain("SourceNode"), "SourceNode")
    diag.connect("SourceNode", "y", "Node0", "u")
    diag.name = "DenseNetwork"
    return diag


def make_physics_many_spheres(nx=6, ny=4):
    """Build a reduced many-spheres world for speed/precision sweeps."""
    from minilink.physics import PhysicsWorldSystem
    from minilink.physics.engine_jax import PlaneModel, SphereModel, make_world_model

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
