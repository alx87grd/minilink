"""DynamicSystem wrapper for the JAX physics engine world."""

from __future__ import annotations

import numpy as np

from minilink.core.system import DynamicSystem
from minilink.graphical.primitives import Plane, Sphere, translation_matrix
from minilink.physics.engine_jax import WorldModel, unpack_state, world_ode


def _is_jax_array(a) -> bool:
    return type(a).__module__.startswith("jax")


class PhysicsWorldSystem(DynamicSystem):
    """
    Multi-body rigid sphere world wrapped as one DynamicSystem.

    State per body: [p(3), q(4), v(3), w(3)].
    Input per body: [Fx, Fy, Fz, Tx, Ty, Tz].
    """

    def __init__(self, world: WorldModel, *, name="PhysicsWorldSystem"):
        self.world = world
        n_b = world.n_bodies
        n = 13 * n_b
        m = 6 * n_b
        p = n
        super().__init__(n=n, m=m, p=p)
        self.name = name

        self.params["n_bodies"] = n_b
        self.params["k_contact"] = world.k_contact
        self.params["c_contact"] = world.c_contact

        # Defaults: identity quaternions and zero velocities.
        self.x0 = np.zeros(n, dtype=float)
        for i in range(n_b):
            self.x0[13 * i + 3] = 1.0

        uport = self.inputs["u"]
        labels = []
        units = []
        for i in range(n_b):
            labels += [
                f"F{i}_x",
                f"F{i}_y",
                f"F{i}_z",
                f"T{i}_x",
                f"T{i}_y",
                f"T{i}_z",
            ]
            units += ["N", "N", "N", "Nm", "Nm", "Nm"]
        uport.labels = labels
        uport.units = units

        self.outputs["y"].labels = [f"x[{i}]" for i in range(n)]
        self.outputs["y"].units = [""] * n

    def f(self, x, u, t=0, params=None):
        dx = world_ode(self.world, x, u)
        if _is_jax_array(x) or _is_jax_array(u):
            return dx
        return np.asarray(dx, dtype=float)

    def h(self, x, u, t=0, params=None):
        return x

    def get_kinematic_geometry(self):
        prim = []
        for i in range(self.world.n_bodies):
            r = float(np.asarray(self.world.radii)[i])
            prim.append(
                Sphere(radius=r, center=[0.0, 0.0, 0.0], color="red", opacity=1.0)
            )
        prim.append(
            Plane(
                normal=np.asarray(self.world.plane_normal, dtype=float),
                offset=float(self.world.plane_offset),
                size=100.0,
                thickness=0.1,
                color="blue",
                opacity=0.0,
            )
        )
        return prim

    def get_kinematic_transforms(self, x, u, t):
        pos, _, _, _ = unpack_state(x, self.world.n_bodies)
        pos_np = np.asarray(pos, dtype=float)
        T = [translation_matrix(p[0], p[1], p[2]) for p in pos_np]
        T.append(np.eye(4, dtype=float))  # plane guide is already in world coords
        return T
