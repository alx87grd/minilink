"""
Serial manipulators: mechanical dynamics plus task-space kinematics.

Equation of motion (from :class:`~minilink.dynamics.abstraction.mechanical.MechanicalSystem`)::

    H(q) q̈ + C(q, q̇) q̇ + g(q) = τ

Kinematics (this module)::

    p = f(q)
    ṗ = J(q) q̇

Joint-space outputs ``q``, ``dq`` live on :class:`MechanicalSystem`.
Task-space outputs ``p``, ``pdot`` are added here.

See ``DESIGN.md`` (dynamics + control feedback profiles).
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

    def inverse_kinematics(
        self,
        p_d,
        q_guess=None,
        *,
        params=None,
        tol=1e-9,
        bounds=None,
        method="auto",
    ):
        """Return ``q`` with ``forward_kinematics(q) ≈ p_d``.

        Uses ``scipy.optimize.fsolve`` when ``task_dim == dof`` and no joint
        bounds apply; otherwise ``scipy.optimize.least_squares``. Returns one
        solution near ``q_guess`` (default: nominal ``q`` output).
        """
        import numpy as np
        from scipy.optimize import fsolve, least_squares

        p_d = np.asarray(p_d, dtype=float).reshape(-1)
        if q_guess is None:
            q_guess = np.asarray(self.outputs["q"].nominal_value, dtype=float)
        else:
            q_guess = np.asarray(q_guess, dtype=float)
        q_guess = q_guess.reshape(-1)
        if p_d.shape != (self.task_dim,):
            raise ValueError(
                f"p_d must have shape ({self.task_dim},), got {p_d.shape}"
            )
        if q_guess.shape != (self.dof,):
            raise ValueError(
                f"q_guess must have shape ({self.dof},), got {q_guess.shape}"
            )

        def residual(q):
            p = np.asarray(self.forward_kinematics(q, params), dtype=float).reshape(-1)
            return p - p_d

        bound_pair = None
        if bounds is not None:
            bound_pair = (
                np.asarray(bounds[0], dtype=float).reshape(-1),
                np.asarray(bounds[1], dtype=float).reshape(-1),
            )
        else:
            lower = np.asarray(self.state.lower_bound[: self.dof], dtype=float)
            upper = np.asarray(self.state.upper_bound[: self.dof], dtype=float)
            if np.all(np.isfinite(lower)) and np.all(np.isfinite(upper)):
                bound_pair = (lower, upper)

        if method == "auto":
            method = (
                "fsolve"
                if self.task_dim == self.dof and bound_pair is None
                else "least_squares"
            )
        if method not in {"fsolve", "least_squares"}:
            raise ValueError(f"method must be 'auto', 'fsolve', or 'least_squares', got {method!r}")

        if method == "fsolve":
            q_sol = np.asarray(fsolve(residual, q_guess, xtol=tol), dtype=float).reshape(-1)
            if np.linalg.norm(residual(q_sol)) > tol * 1e3:
                raise RuntimeError("inverse_kinematics did not converge")
            return q_sol

        if bound_pair is None:
            lo = np.full(self.dof, -np.inf)
            hi = np.full(self.dof, np.inf)
        else:
            lo, hi = bound_pair
        result = least_squares(
            residual,
            q_guess,
            bounds=(lo, hi),
            ftol=tol,
            xtol=tol,
        )
        if not result.success or np.linalg.norm(result.fun) > tol * 1e3:
            raise RuntimeError("inverse_kinematics did not converge")
        return np.asarray(result.x, dtype=float).reshape(-1)
