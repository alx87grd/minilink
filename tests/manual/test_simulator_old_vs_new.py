import jax.numpy as jnp

# from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.core.system import DynamicSystem
from minilink.simulation.simulator import Simulator


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

sim_numpy = Simulator(
    Pendulum, t0=t0, tf=tf, dt=dt, solver=solver, compile_backend="numpy"
)
traj_numpy = sim_numpy.solve()
print(traj_numpy.x[:, -1])

sim_jax = Simulator(
    Pendulum, t0=t0, tf=tf, dt=dt, solver=solver, compile_backend=compile_backend
)
traj_jax = sim_jax.solve()
print(traj_jax.x[:, -1])
