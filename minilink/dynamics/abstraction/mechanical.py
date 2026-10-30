"""Second-order mechanical systems in generalized coordinates: ``H(q) v̇ + C(q, v) v + d + g(q) = τ``."""

from minilink.core.backends import array_module
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

        # Signal labels and units: an angle and a velocity per coordinate; no bounds,
        # each plant states its own range
        for i in range(dof):
            self.state.labels[i] = f"Angle {i}"
            self.state.units[i] = "[rad]"
            j = i + dof
            self.state.labels[j] = f"Velocity {i}"
            self.state.units[j] = "[rad/sec]"

        # A torque per actuator, and the output named after the state
        uport = self.inputs["u"]
        for i in range(actuators):
            uport.labels[i] = f"Torque {i}"
            uport.units[i] = "[Nm]"

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
