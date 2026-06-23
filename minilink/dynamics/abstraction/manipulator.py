"""
Serial manipulators: mechanical dynamics plus task-space kinematics.

Equation of motion (from :class:`~minilink.dynamics.abstraction.mechanical.MechanicalSystem`)::

    H(q) q̈ + C(q, q̇) q̇ + g(q) = τ

Kinematics (this module)::

    p = f(q)
    ṗ = J(q) q̇

Joint-space outputs ``q``, ``dq`` live on :class:`MechanicalSystem`.
Task-space outputs ``p``, ``pdot`` are added here.

See ``docs/plans/manipulator-abstraction.md``.
"""

from minilink.core.backends import array_module
from minilink.dynamics.abstraction.mechanical import MechanicalSystem


class Manipulator(MechanicalSystem):
    """
    Serial manipulator with task-space kinematic outputs.

    Dynamics follow :class:`MechanicalSystem`. Subclasses override
    :meth:`forward_kinematics` and :meth:`J`.

    Output ports
    ------------
    q, dq :
        Joint position and velocity (from :class:`MechanicalSystem`).
    p, pdot :
        Task position ``p = f(q)`` and velocity ``ṗ = J(q) q̇``.
    y, x :
        Full state ``[q; q̇]`` (backward compatible).

    Input port ``u`` carries joint torques ``τ``.
    """

    def __init__(self, dof=1, actuators=None, task_dim=2):
        super().__init__(dof=dof, actuators=actuators)
        self.task_dim = int(task_dim)
        if self.task_dim <= 0:
            raise ValueError("task_dim must be positive")

        self.name = f"{dof}DoF Manipulator"

        p_labels = [f"p{i}" for i in range(self.task_dim)]
        pdot_labels = [f"pdot{i}" for i in range(self.task_dim)]
        self.add_output_port(
            "p",
            dim=self.task_dim,
            function=self.h_p,
            dependencies=(),
            labels=p_labels,
            units=["[m]"] * self.task_dim,
        )
        self.add_output_port(
            "pdot",
            dim=self.task_dim,
            function=self.h_pdot,
            dependencies=(),
            labels=pdot_labels,
            units=["[m/s]"] * self.task_dim,
        )

    def forward_kinematics(self, q, params=None):
        """Task-space position ``p = f(q)``, shape ``(task_dim,)``."""
        xp = array_module(q)
        return xp.zeros(self.task_dim)

    def J(self, q, params=None):
        """Manipulator Jacobian ``∂p/∂q``, shape ``(task_dim, dof)``."""
        xp = array_module(q)
        return xp.zeros((self.task_dim, self.dof))

    def h_p(self, x, u, t=0, params=None):
        q, dq = self.x2q(x)
        return self.forward_kinematics(q, params)

    def h_pdot(self, x, u, t=0, params=None):
        q, dq = self.x2q(x)
        J = self.J(q, params)

        # task-space velocity: ṗ = J(q) q̇
        return J @ dq
