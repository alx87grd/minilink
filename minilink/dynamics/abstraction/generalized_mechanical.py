"""Second-order mechanical systems with generalized velocities."""

import numpy as np

from minilink.compile.jax_utils import array_module
from minilink.core.system import DynamicSystem


class GeneralizedMechanicalSystem(DynamicSystem):
    """Mechanical system whose velocities need not equal ``qdot``.

    The state is stacked as ``x = [q; v]``. In the course-note notation, ``v``
    corresponds to a generalized or body-frame velocity, often written
    ``nu``. Dynamics use the structure::

        M(q) dv + C(q, v) v + d(q, v, u) + g(q) = tau(q, v, u)
        qdot = N(q) v

    The default generalized input force is ``B(q) @ u`` through
    :meth:`input_forces`. Subclasses with mixed force and geometry inputs
    should override :meth:`input_forces` rather than introducing a separate
    ``WithPositionInputs`` inheritance branch.
    """

    def __init__(self, dof=1, pos=None, actuators=None):
        self.dof = int(dof)
        self.pos = self.dof if pos is None else int(pos)
        if actuators is None:
            actuators = self.dof

        n = self.pos + self.dof
        m = int(actuators)
        p = n
        super().__init__(n=n, m=m, p=p)

        self.name = f"{self.dof}DoF Generalized Mechanical System"

        for i in range(self.pos):
            self.state.labels[i] = f"Position {i}"
            self.state.units[i] = "[m]"
            self.state.upper_bound[i] = 10.0
            self.state.lower_bound[i] = -10.0

        for i in range(self.dof):
            j = self.pos + i
            self.state.labels[j] = f"Velocity {i}"
            self.state.units[j] = "[m/s]"
            self.state.upper_bound[j] = 10.0
            self.state.lower_bound[j] = -10.0

        uport = self.inputs["u"]
        for i in range(self.m):
            uport.labels[i] = f"Force input {i}"
            uport.units[i] = "[N]"
            uport.upper_bound[i] = 5.0
            uport.lower_bound[i] = -5.0

        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def M(self, q, params=None):
        """Inertia matrix, shape ``(dof, dof)``."""
        xp = array_module(q)
        return xp.eye(self.dof)

    def C(self, q, v, params=None):
        """Coriolis and centrifugal matrix, shape ``(dof, dof)``."""
        xp = array_module(q)
        return xp.zeros((self.dof, self.dof))

    def N(self, q, params=None):
        """Kinematic map from generalized velocities to configuration rates."""
        xp = array_module(q)
        return xp.eye(self.pos, self.dof)

    def B(self, q, params=None):
        """Actuator matrix used by the default :meth:`input_forces` hook."""
        xp = array_module(q)
        return xp.eye(self.dof, self.m)

    def g(self, q, params=None):
        """Conservative generalized forces, shape ``(dof,)``."""
        xp = array_module(q)
        return xp.zeros(self.dof)

    def d(self, q, v, u=None, t=0.0, params=None):
        """Dissipative generalized forces, shape ``(dof,)``."""
        xp = array_module(q)
        return xp.zeros(self.dof)

    def input_forces(self, q, v, u, t=0.0, params=None):
        """Generalized input forces ``tau(q, v, u)``.

        The default is ``B(q) @ u``. Override this method for systems where
        inputs include steering angles, control-surface positions, tire laws,
        aerodynamic forces, propulsor maps, or other mixed input semantics.
        """
        return self.B(q, params) @ u

    def x2qv(self, x):
        """Split state ``x`` into configuration ``q`` and velocity ``v``."""
        q = x[0 : self.pos]
        v = x[self.pos : self.n]
        return [q, v]

    def qv2x(self, q, v):
        """Stack configuration ``q`` and velocity ``v`` into state ``x``."""
        xp = array_module(q)
        return xp.concatenate([q, v])

    def qdot(self, q, v, params=None):
        """Configuration derivative ``qdot = N(q) @ v``."""
        params = self.params if params is None else params
        return self.N(q, params) @ v

    def dv(self, q, v, u, t=0.0, params=None):
        """Velocity derivative from forward dynamics."""
        params = self.params if params is None else params
        M = self.M(q, params)
        C = self.C(q, v, params)
        g = self.g(q, params)
        d = self.d(q, v, u, t, params)
        tau = self.input_forces(q, v, u, t, params)
        rhs = tau - C @ v - g - d
        xp = array_module(rhs)
        return xp.linalg.solve(M, rhs)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        q, v = self.x2qv(x)
        return self.qv2x(self.qdot(q, v, params), self.dv(q, v, u, t, params))

    def h(self, x, u, t=0.0, params=None):
        return x

    def kinetic_energy(self, q, v, params=None):
        params = self.params if params is None else params
        return 0.5 * (v @ (self.M(q, params) @ v))
