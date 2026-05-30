"""
Second-order mechanical systems in generalized coordinates.

Numeric template; see :mod:`minilink.dynamics.abstraction`.

Equation of motion::

    H(q) ddq + C(q, dq) dq + d(q, dq) + g(q) = B(q) u

* :class:`MechanicalSystem` — native-array default template. Concrete subclasses
  remain JAX-traceable only if their hooks use JAX-compatible math.
* :class:`JaxMechanicalSystem` — compatibility/convenience base for explicit JAX
  plant variants. JAX is loaded lazily when you call methods on that class.
"""

import numpy as np

from minilink.compile.jax_utils import array_module, require_jax_numpy
from minilink.core.system import DynamicSystem


class MechanicalSystem(DynamicSystem):
    """
    Mechanical system with equation of motion

        H(q) ddq + C(q, dq) dq + d(q, dq) + g(q) = B(q) u

    State is stacked as ``x = [q; dq]`` with ``n = 2 * dof`` and default output ``y = x``.

    Subclasses override ``H``, ``C``, ``B``, ``g``, and/or ``d``. The default
    hooks follow Minilink's native-array rule; a concrete subclass is
    JAX-traceable only if its overridden hooks do the same.
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

    def H(self, q, params=None):
        """Inertia matrix, shape (dof, dof). Kinetic energy = 0.5 * dq^T H(q) dq."""
        xp = array_module(q)
        return xp.eye(self.dof)

    def C(self, q, dq, params=None):
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

    def d(self, q, dq, params=None):
        """Dissipative forces, shape (dof,)."""
        xp = array_module(q)
        return xp.zeros(self.dof)

    def x2q(self, x):
        """Split state ``x`` into ``q`` and ``dq``."""
        q = x[0 : self.dof]
        dq = x[self.dof : self.n]
        return [q, dq]

    def q2x(self, q, dq):
        """Stack ``q`` and ``dq`` into state ``x``."""
        xp = array_module(q)
        return xp.concatenate([q, dq])

    def generalized_forces(self, q, dq, ddq, t=0, params=None):
        """Generalized forces for a given trajectory ``q, dq, ddq``."""
        params = self.params if params is None else params
        H = self.H(q, params)
        C = self.C(q, dq, params)
        g = self.g(q, params)
        d = self.d(q, dq, params)
        return H @ ddq + C @ dq + g + d

    def actuator_forces(self, q, dq, ddq, t=0, params=None):
        """Inverse dynamics: actuator forces given ``q, dq, ddq`` (square ``B`` only)."""
        if self.dof != self.m:
            raise NotImplementedError
        params = self.params if params is None else params
        B = self.B(q, params)
        forces = self.generalized_forces(q, dq, ddq, t, params)
        xp = array_module(forces)
        return xp.linalg.solve(B, forces)

    def ddq(self, q, dq, u, t=0, params=None):
        """Forward dynamics: generalized accelerations given ``u``."""
        params = self.params if params is None else params
        H = self.H(q, params)
        C = self.C(q, dq, params)
        g = self.g(q, params)
        d = self.d(q, dq, params)
        B = self.B(q, params)
        rhs = B @ u - C @ dq - g - d
        xp = array_module(rhs)
        return xp.linalg.solve(H, rhs)

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        q, dq = self.x2q(x)
        ddq = self.ddq(q, dq, u, t, params)
        return self.q2x(dq, ddq)

    def h(self, x, u, t=0, params=None):
        return x

    def kinetic_energy(self, q, dq, params=None):
        params = self.params if params is None else params
        return 0.5 * (dq @ (self.H(q, params) @ dq))


class JaxMechanicalSystem(MechanicalSystem):
    """
    JAX-traceable counterpart of :class:`MechanicalSystem`.

    Inherits port labels, bounds, ``__init__``, ``x2q`` / ``h``, and the
    ``generalized_forces`` formula from :class:`MechanicalSystem`. Overrides
    only the matrix builders (:meth:`H`, :meth:`C`, :meth:`B`, :meth:`g`,
    :meth:`d`), the state-stacking helper :meth:`q2x`, and the methods that
    call ``np.linalg.solve`` (:meth:`actuator_forces`, :meth:`ddq`,
    :meth:`f`) so the dynamics trace through ``jax.numpy``.

    JAX is loaded lazily inside each method via
    :func:`~minilink.compile.jax_utils.require_jax_numpy`, so importing this
    module stays free without the ``minilink[jax]`` extra.
    """

    def H(self, q, params=None):
        jnp = require_jax_numpy()
        dt = getattr(q, "dtype", None) or jnp.float32
        return jnp.diag(jnp.ones(self.dof, dtype=dt))

    def C(self, q, dq, params=None):
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

    def d(self, q, dq, params=None):
        jnp = require_jax_numpy()
        dt = getattr(q, "dtype", None) or jnp.float32
        return jnp.zeros(self.dof, dtype=dt)

    def q2x(self, q, dq):
        jnp = require_jax_numpy()
        dt = getattr(q, "dtype", None) or jnp.float32
        return jnp.concatenate(
            [jnp.asarray(q, dtype=dt).ravel(), jnp.asarray(dq, dtype=dt).ravel()]
        )

    def actuator_forces(self, q, dq, ddq, t=0, params=None):
        if self.dof != self.m:
            raise NotImplementedError
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        B = self.B(q, params)
        forces = self.generalized_forces(q, dq, ddq, t, params)
        return jnp.linalg.solve(B, forces)

    def ddq(self, q, dq, u, t=0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        u = jnp.asarray(u)
        H = self.H(q, params)
        C = self.C(q, dq, params)
        g = self.g(q, params)
        d = self.d(q, dq, params)
        B = self.B(q, params)
        rhs = B @ u - C @ dq - g - d
        return jnp.linalg.solve(H, rhs)

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        u = jnp.asarray(u)
        q, dq = self.x2q(x)
        ddq = self.ddq(q, dq, u, t, params)
        return self.q2x(dq, ddq)

    def kinetic_energy(self, q, dq, params=None):
        params = self.params if params is None else params
        return 0.5 * (dq @ (self.H(q, params) @ dq))
