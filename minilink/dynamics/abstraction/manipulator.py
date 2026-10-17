"""Serial manipulators: the mechanical dynamics plus the task-space kinematics ``p = f(q)``, ``ṗ = J(q) q̇``."""

import numpy as np

from minilink.core.backends import array_module
from minilink.dynamics.abstraction.mechanical import MechanicalSystem

###############################################################################
# Public API
###############################################################################


class Manipulator(MechanicalSystem):
    """
    Serial manipulator with task-space kinematic outputs.

    Dynamics follow :class:`MechanicalSystem`. Subclasses override
    :meth:`forward_kinematics` and :meth:`J`.

    Output ports
    ------------
    p, pdot :
        Task position ``p = f(q)`` and velocity ``ṗ = J(q) q̇``.
    y, x :
        Full state ``[q; q̇]``.

    Input port ``u`` carries joint torques ``τ``. An optional world-frame
    tool-force port ``f`` (added by a subclass) is drawn as a crimson arrow
    at the tool when :meth:`get_dynamic_geometry` runs.
    """

    def __init__(self, dof=1, actuators=None, task_dim=2):
        super().__init__(dof=dof, actuators=actuators)
        self.task_dim = int(task_dim)
        require_positive_task_dim(self.task_dim)
        self.name = f"{dof}DoF Manipulator"

        # Graphics: the scale of the optional tool-force arrow, in m/N
        self.force_arrow_scale = 0.02

        # Ports: the task position p and the task velocity ṗ
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

        task_dim = self.task_dim

        # p = 0: the tool sits at the origin until a subclass writes its kinematics
        p = xp.zeros(task_dim)

        return p

    def J(self, q, params=None):
        """Manipulator Jacobian ``∂p/∂q``, shape ``(task_dim, dof)``."""
        xp = array_module(q)

        task_dim = self.task_dim
        dof = self.dof

        # J = 0: no joint moves the tool until a subclass writes its kinematics
        J = xp.zeros((task_dim, dof))

        return J

    def h_p(self, x, u, t=0, params=None):
        q, _ = self.x2q(x)

        p = self.forward_kinematics(q, params)

        return p

    def h_pdot(self, x, u, t=0, params=None):
        q, dq = self.x2q(x)
        J = self.J(q, params)

        # ṗ = J(q) q̇
        pdot = J @ dq

        return pdot

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """Crimson tool-force arrow when input port ``f`` is present and nonzero."""
        if "f" not in self.inputs:
            return {}

        f = np.asarray(u[self.get_input_port_slice("f")], dtype=float).reshape(-1)

        # Only a planar or spatial force above 10⁻¹² N draws an arrow
        if f.size not in (2, 3) or float(np.linalg.norm(f)) < 1e-12:
            return {}

        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation.primitives import Arrow

        return {
            "force": [
                Arrow(
                    base=np.zeros(f.size),
                    vector=f,
                    scale=self.force_arrow_scale,
                    color="crimson",
                    linewidth=2.5,
                )
            ]
        }

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
        solution near ``q_guess`` (default: the nominal state's ``q``).
        """
        # Lazy import: SciPy's solvers load on first use, not with the model
        from scipy.optimize import fsolve, least_squares

        dof = self.dof
        task_dim = self.task_dim

        # Input checks: the target and the initial guess, as flat float vectors
        p_d = np.asarray(p_d, dtype=float).reshape(-1)
        q_guess = initial_guess_of(self, q_guess)
        check_ik_shapes(p_d, q_guess, task_dim, dof)

        # The residual r(q) = f(q) − p_d, zero at a solution
        def residual(q):
            p = np.asarray(self.forward_kinematics(q, params), dtype=float).reshape(-1)
            r = p - p_d
            return r

        # Setup: the joint bounds, then the solver they allow
        bound_pair = joint_bounds_of(self, bounds)
        method = ik_method_of(method, task_dim, dof, bound_pair)

        if method == "fsolve":
            # Square and unbounded: a root of r(q) = 0, refused when ‖r‖ > 10³ tol
            q_sol = np.asarray(
                fsolve(residual, q_guess, xtol=tol), dtype=float
            ).reshape(-1)
            require_ik_converged(residual(q_sol), tol)
        else:
            # Otherwise: min ‖r(q)‖² within the bounds (none when there are none),
            # refused when the solver fails or ‖r‖ > 10³ tol
            if bound_pair is None:
                lo = np.full(dof, -np.inf)
                hi = np.full(dof, np.inf)
            else:
                lo, hi = bound_pair
            result = least_squares(
                residual,
                q_guess,
                bounds=(lo, hi),
                ftol=tol,
                xtol=tol,
            )
            require_ik_converged(result.fun, tol, result.success)
            q_sol = np.asarray(result.x, dtype=float).reshape(-1)

        return q_sol


###############################################################################
# Internal machinery
###############################################################################


def require_positive_task_dim(task_dim):
    """Refuse a task space without a dimension."""
    if task_dim <= 0:
        raise ValueError("task_dim must be positive")


def initial_guess_of(sys, q_guess):
    """The initial guess as a flat float vector: the one given, else the nominal state's ``q``."""
    if q_guess is None:
        q_guess = np.asarray(sys.state.nominal_value[: sys.dof], dtype=float)
    else:
        q_guess = np.asarray(q_guess, dtype=float)

    return q_guess.reshape(-1)


def check_ik_shapes(p_d, q_guess, task_dim, dof):
    """Refuse a target that is not a task-space point, or a guess that is not a joint vector."""
    if p_d.shape != (task_dim,):
        raise ValueError(f"p_d must have shape ({task_dim},), got {p_d.shape}")
    if q_guess.shape != (dof,):
        raise ValueError(f"q_guess must have shape ({dof},), got {q_guess.shape}")


def joint_bounds_of(sys, bounds):
    """The joint bounds ``(lower, upper)``: the pair given, else the state's when all are finite, else ``None``."""
    if bounds is not None:
        return (
            np.asarray(bounds[0], dtype=float).reshape(-1),
            np.asarray(bounds[1], dtype=float).reshape(-1),
        )

    lower = np.asarray(sys.state.lower_bound[: sys.dof], dtype=float)
    upper = np.asarray(sys.state.upper_bound[: sys.dof], dtype=float)
    if np.all(np.isfinite(lower)) and np.all(np.isfinite(upper)):
        return (lower, upper)

    return None


def ik_method_of(method, task_dim, dof, bound_pair):
    """The solver: ``"auto"`` is fsolve on a square unbounded problem, least squares otherwise."""
    if method == "auto":
        method = "fsolve" if task_dim == dof and bound_pair is None else "least_squares"
    if method not in {"fsolve", "least_squares"}:
        raise ValueError(
            f"method must be 'auto', 'fsolve', or 'least_squares', got {method!r}"
        )

    return method


def require_ik_converged(r, tol, success=True):
    """Refuse a solve that failed, or whose residual ``‖r‖`` exceeds ``10³ tol``."""
    if not success or np.linalg.norm(r) > tol * 1e3:
        raise RuntimeError("inverse_kinematics did not converge")
