import numpy as np
import jax
import jax.numpy as jnp


# from minilink.blocks.examples import Pendulum
from minilink.core.analysis import Simulator as OldSimulator
from minilink.simulation import Simulator as NewSimulator
from minilink.core.framework import DynamicSystem


class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, m=1, p=2)
        self.name = "Pendulum"

    def f(self, x, u, t=0, params=None):
        # Pure JAX implementation
        gravity = 9.81
        length = 1.0
        damping = 0.5
        q, dq = x[0], x[1]
        ddq = -(gravity / length) * jnp.sin(q) - damping * dq + u[0]
        return jnp.array([dq, ddq])


Pendulum = Pendulum()
Pendulum.params["m"] = 1.0
Pendulum.params["l"] = 1.0
Pendulum.x0[0] = 2.0

solver = "euler"
# solver = "scipy"
# compile_backend = "numpy"
compile_backend = "jax"

t0 = 0
tf = 10
dt = 0.0001

new_sim = NewSimulator(
    Pendulum, t0=t0, tf=tf, dt=dt, solver=solver, compile_backend=compile_backend
)
new_traj = new_sim.solve()
print(new_traj.x[:, -1])

old_sim = OldSimulator(Pendulum, t0=t0, tf=tf, dt=dt, solver=solver)
old_traj = old_sim.solve()
print(old_traj.x[:, -1])
