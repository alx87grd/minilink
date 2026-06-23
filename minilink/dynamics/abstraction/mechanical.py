"""
Second-order mechanical systems in generalized coordinates.

Numeric template; see :mod:`minilink.dynamics.abstraction`.

Equation of motion::

    H(q) a + C(q, v) v + d(q, v, u, t) + g(q)
        = generalized_force(q, v, u, t)

* :class:`MechanicalSystem` — native-array default template. Concrete subclasses
  remain JAX-traceable only if their hooks use JAX-compatible math.
* :class:`JaxMechanicalSystem` — convenience base for explicit JAX plant
  variants. JAX is loaded lazily when you call methods on that class.
"""

import numpy as np

from minilink.core.backends import array_module, require_jax_numpy
from minilink.core.system import DynamicSystem


class MechanicalSystem(DynamicSystem):
    """
    Mechanical system with equation of motion

        H(q) a + C(q, v) v + d(q, v, u, t) + g(q)
            = generalized_force(q, v, u, t)

    State is stacked as ``x = [q; v]`` with ``n = 2 * dof`` and default output
    ``y = x``. For ordinary generalized coordinates, ``v = qdot``.

    Subclasses override ``H``, ``C``, ``B``, ``g``, ``d``, and/or
    ``generalized_force``. The default hooks follow Minilink's native-array
    rule; a concrete subclass is JAX-traceable only if its overridden hooks do
    the same.
    """

    def __init__(self, dof=1, actuators=None):
        self.dof = dof
        if actuators is None:
            actuators = dof

        n = dof * 2
        m = actuators
        p = dof * 2

        super().__init__(
            n=n,
            input_dim=m,
            output_dim=p,
            expose_state=True,
            y_dependencies=(),
        )

        self.name = f"{dof}DoF Mechanical System"

        lim = 2 * np.pi
        for i in range(dof):
            self.state.labels[i] = f"Angle {i}"
            self.state.units[i] = "[rad]"
            self.state.upper_bound[i] = lim
            self.state.lower_bound[i] = -lim
            j = i + dof
            self.state.labels[j] = f"Velocity {i}"
            self.state.units[j] = "[rad/sec]"
            self.state.upper_bound[j] = lim
            self.state.lower_bound[j] = -lim

        uport = self.inputs["u"]
        for i in range(actuators):
            uport.labels[i] = f"Torque {i}"
            uport.units[i] = "[Nm]"
            uport.upper_bound[i] = 5.0
            uport.lower_bound[i] = -5.0

        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        self.add_output_port("q", dim=dof, function=self.h_q, dependencies=())
        self.add_output_port("dq", dim=dof, function=self.h_dq, dependencies=())
        self.outputs["q"].labels = self.state.labels[:dof]
        self.outputs["q"].units = self.state.units[:dof]
        self.outputs["dq"].labels = self.state.labels[dof:]
        self.outputs["dq"].units = self.state.units[dof:]

    def h_q(self, x, u, t=0, params=None):
        q, _ = self.x2q(x)
        return q

    def h_dq(self, x, u, t=0, params=None):
        _, dq = self.x2q(x)
        return dq

    def H(self, q, params=None):
        """Inertia matrix, shape (dof, dof). Kinetic energy = 0.5 * dq^T H(q) dq."""
        xp = array_module(q)
        return xp.eye(self.dof)

    def C(self, q, v, params=None):
        """Coriolis and centrifugal matrix, shape (dof, dof)."""
        xp = array_module(q)
        return xp.zeros((self.dof, self.dof))

    def B(self, q, params=None):
        """Actuator matrix, shape (dof, m)."""
        xp = array_module(q)
        return xp.eye(self.dof, self.m)

    def g(self, q, params=None):
        """Gravitational / conservative forces, shape (dof,)."""
        xp = array_module(q)
        return xp.zeros(self.dof)

    def d(self, q, v, u=None, t=0.0, params=None):
        """Left-side load/dissipation vector, shape ``(dof,)``."""
        xp = array_module(q)
        return xp.zeros(self.dof)

    def generalized_force(self, q, v, u, t=0.0, params=None):
        """Right-side generalized force vector.

        The default is the textbook actuator map ``B(q) @ u``. Override this
        method when inputs mix force commands, geometry commands, environment
        variables, or other non-actuator semantics.
        """
        return self.B(q, params) @ u

    def x2q(self, x):
        """Split state ``x`` into ``q`` and ``v``."""
        q = x[0 : self.dof]
        v = x[self.dof : self.n]
        return [q, v]

    def q2x(self, q, v):
        """Stack ``q`` and ``v`` into state ``x``."""
        xp = array_module(q)
        return xp.concatenate([q, v])

    def inverse_dynamics(self, q, v, acceleration, u=None, t=0.0, params=None):
        """Generalized RHS force required by a trajectory."""
        params = self.params if params is None else params
        H = self.H(q, params)
        C = self.C(q, v, params)
        g = self.g(q, params)
        d = self.d(q, v, u, t, params)
        return H @ acceleration + C @ v + g + d

    def forward_dynamics(self, q, v, u, t=0.0, params=None):
        """Forward dynamics: generalized acceleration given ``u``."""
        params = self.params if params is None else params
        H = self.H(q, params)
        C = self.C(q, v, params)
        g = self.g(q, params)
        d = self.d(q, v, u, t, params)
        tau = self.generalized_force(q, v, u, t, params)
        rhs = tau - C @ v - g - d
        xp = array_module(rhs)
        return xp.linalg.solve(H, rhs)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        q, v = self.x2q(x)
        acceleration = self.forward_dynamics(q, v, u, t, params)
        return self.q2x(v, acceleration)

    def h(self, x, u, t=0.0, params=None):
        return x

    def kinetic_energy(self, q, v, params=None):
        params = self.params if params is None else params
        return 0.5 * (v @ (self.H(q, params) @ v))


class JaxMechanicalSystem(MechanicalSystem):
    """
    JAX-traceable counterpart of :class:`MechanicalSystem`.

    Inherits port labels, bounds, ``__init__``, ``x2q`` / ``h``, and the
    ``inverse_dynamics`` formula from :class:`MechanicalSystem`. Overrides
    only the matrix builders (:meth:`H`, :meth:`C`, :meth:`B`, :meth:`g`,
    :meth:`d`), the state-stacking helper :meth:`q2x`, and methods that need
    JAX array creation or ``jax.numpy.linalg.solve``.

    JAX is loaded lazily inside each method via
    :func:`~minilink.core.backends.require_jax_numpy`, so importing this
    module stays free without the ``minilink[jax]`` extra.
    """

    def H(self, q, params=None):
        jnp = require_jax_numpy()
        dt = getattr(q, "dtype", None) or jnp.float32
        return jnp.diag(jnp.ones(self.dof, dtype=dt))

    def C(self, q, v, params=None):
        jnp = require_jax_numpy()
        dt = getattr(q, "dtype", None) or jnp.float32
        return jnp.zeros((self.dof, self.dof), dtype=dt)

    def B(self, q, params=None):
        jnp = require_jax_numpy()
        dt = getattr(q, "dtype", None) or jnp.float32
        B = jnp.zeros((self.dof, self.m), dtype=dt)
        for i in range(min(self.m, self.dof)):
            B = B.at[i, i].set(jnp.asarray(1.0, dtype=dt))
        return B

    def g(self, q, params=None):
        jnp = require_jax_numpy()
        dt = getattr(q, "dtype", None) or jnp.float32
        return jnp.zeros(self.dof, dtype=dt)

    def d(self, q, v, u=None, t=0.0, params=None):
        jnp = require_jax_numpy()
        dt = getattr(q, "dtype", None) or jnp.float32
        return jnp.zeros(self.dof, dtype=dt)

    def generalized_force(self, q, v, u, t=0.0, params=None):
        return self.B(q, params) @ u

    def q2x(self, q, v):
        jnp = require_jax_numpy()
        dt = getattr(q, "dtype", None) or jnp.float32
        return jnp.concatenate(
            [jnp.asarray(q, dtype=dt).ravel(), jnp.asarray(v, dtype=dt).ravel()]
        )

    def forward_dynamics(self, q, v, u, t=0.0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        u = jnp.asarray(u)
        H = self.H(q, params)
        C = self.C(q, v, params)
        g = self.g(q, params)
        d = self.d(q, v, u, t, params)
        tau = self.generalized_force(q, v, u, t, params)
        rhs = tau - C @ v - g - d
        return jnp.linalg.solve(H, rhs)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        u = jnp.asarray(u)
        q, v = self.x2q(x)
        acceleration = self.forward_dynamics(q, v, u, t, params)
        return self.q2x(v, acceleration)

    def kinetic_energy(self, q, v, params=None):
        params = self.params if params is None else params
        return 0.5 * (v @ (self.H(q, params) @ v))
