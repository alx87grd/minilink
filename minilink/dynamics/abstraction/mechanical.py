"""Second-order mechanical systems in generalized coordinates: ``H(q) v̇ + C(q, v) v + d + g(q) = τ``."""

import numpy as np

from minilink.core.backends import array_module, require_jax_numpy
from minilink.core.system import DynamicSystem

###############################################################################
# Public API
###############################################################################


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

        # Dimensions: the state x = [q; v] and the output y = x
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

        # Signal labels, units and bounds: an angle and a velocity per coordinate
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

        # A torque per actuator, and the output named after the state
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

        dof = self.dof

        # H = I: a unit inertia on every coordinate until a subclass writes its own
        H = xp.eye(dof)

        return H

    def C(self, q, v, params=None):
        """Coriolis and centrifugal matrix, shape (dof, dof)."""
        xp = array_module(q)

        dof = self.dof

        # C = 0: no velocity coupling until a subclass writes its own
        C = xp.zeros((dof, dof))

        return C

    def B(self, q, params=None):
        """Actuator matrix, shape (dof, m)."""
        xp = array_module(q)

        dof = self.dof
        m = self.m

        # B = I: actuator i drives coordinate i
        B = xp.eye(dof, m)

        return B

    def g(self, q, params=None):
        """Gravitational / conservative forces, shape (dof,)."""
        xp = array_module(q)

        dof = self.dof

        # g = 0: no conservative force until a subclass writes its own
        g = xp.zeros(dof)

        return g

    def d(self, q, v, u=None, t=0.0, params=None):
        """Left-side load/dissipation vector, shape ``(dof,)``."""
        xp = array_module(q)

        dof = self.dof

        # d = 0: no load or dissipation until a subclass writes its own
        d = xp.zeros(dof)

        return d

    def generalized_force(self, q, v, u, t=0.0, params=None):
        """Right-side generalized force vector.

        The default is the textbook actuator map ``B(q) @ u``. Override this
        method when inputs mix force commands, geometry commands, environment
        variables, or other non-actuator semantics.
        """
        B = self.B(q, params)

        tau = B @ u

        return tau

    def x2q(self, x):
        """Split state ``x`` into ``q`` and ``v``."""
        dof = self.dof
        n = self.n

        q = x[0:dof]
        v = x[dof:n]

        return [q, v]

    def q2x(self, q, v):
        """Stack ``q`` and ``v`` into state ``x``."""
        xp = array_module(q)

        # x = [q; v]
        x = xp.concatenate([q, v])

        return x

    def inverse_dynamics(self, q, v, acceleration, u=None, t=0.0, params=None):
        """Generalized RHS force required by a trajectory."""
        params = self.params if params is None else params

        H = self.H(q, params)
        C = self.C(q, v, params)
        g = self.g(q, params)
        d = self.d(q, v, u, t, params)

        tau = H @ acceleration + C @ v + g + d

        return tau

    def forward_dynamics(self, q, v, u, t=0.0, params=None):
        """Forward dynamics: generalized acceleration given ``u``."""
        params = self.params if params is None else params
        xp = array_module(q, v)

        H = self.H(q, params)
        C = self.C(q, v, params)
        g = self.g(q, params)
        d = self.d(q, v, u, t, params)
        tau = self.generalized_force(q, v, u, t, params)

        # H v̇ = τ − C v − g − d
        vdot = xp.linalg.solve(H, tau - C @ v - g - d)

        return vdot

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        q, v = self.x2q(x)
        vdot = self.forward_dynamics(q, v, u, t, params)

        # dx = [v; v̇]
        dx = self.q2x(v, vdot)

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def kinetic_energy(self, q, v, params=None):
        params = self.params if params is None else params

        H = self.H(q, params)

        # T = ½ vᵀ H(q) v
        T = 0.5 * (v @ (H @ v))

        return T


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

        dof = self.dof

        dtype = dtype_of(q, jnp)

        # H = I, in the dtype of q
        H = jnp.diag(jnp.ones(dof, dtype=dtype))

        return H

    def C(self, q, v, params=None):
        jnp = require_jax_numpy()

        dof = self.dof

        dtype = dtype_of(q, jnp)

        # C = 0, in the dtype of q
        C = jnp.zeros((dof, dof), dtype=dtype)

        return C

    def B(self, q, params=None):
        jnp = require_jax_numpy()

        dof = self.dof
        m = self.m

        dtype = dtype_of(q, jnp)

        # B = I, set entry by entry on the diagonal of a traced array
        B = jnp.zeros((dof, m), dtype=dtype)
        for i in range(min(m, dof)):
            B = B.at[i, i].set(jnp.asarray(1.0, dtype=dtype))

        return B

    def g(self, q, params=None):
        jnp = require_jax_numpy()

        dof = self.dof

        dtype = dtype_of(q, jnp)

        # g = 0, in the dtype of q
        g = jnp.zeros(dof, dtype=dtype)

        return g

    def d(self, q, v, u=None, t=0.0, params=None):
        jnp = require_jax_numpy()

        dof = self.dof

        dtype = dtype_of(q, jnp)

        # d = 0, in the dtype of q
        d = jnp.zeros(dof, dtype=dtype)

        return d

    def q2x(self, q, v):
        jnp = require_jax_numpy()

        dtype = dtype_of(q, jnp)
        q_flat = jnp.asarray(q, dtype=dtype).ravel()
        v_flat = jnp.asarray(v, dtype=dtype).ravel()

        # x = [q; v], both in the dtype of q
        x = jnp.concatenate([q_flat, v_flat])

        return x

    def forward_dynamics(self, q, v, u, t=0.0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()

        u = jnp.asarray(u)
        H = self.H(q, params)
        C = self.C(q, v, params)
        g = self.g(q, params)
        d = self.d(q, v, u, t, params)
        tau = self.generalized_force(q, v, u, t, params)

        # H v̇ = τ − C v − g − d
        vdot = jnp.linalg.solve(H, tau - C @ v - g - d)

        return vdot

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()

        u = jnp.asarray(u)
        q, v = self.x2q(x)
        vdot = self.forward_dynamics(q, v, u, t, params)

        # dx = [v; v̇]
        dx = self.q2x(v, vdot)

        return dx


###############################################################################
# Internal machinery
###############################################################################


def dtype_of(q, jnp):
    """The dtype of ``q``, or float32 when ``q`` carries none (a list, a scalar)."""
    return getattr(q, "dtype", None) or jnp.float32
