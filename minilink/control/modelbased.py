"""Model-based joint control: computed torque and sliding mode on the plant's own model."""

from __future__ import annotations

import copy

import numpy as np

from minilink.control.siso import as_dof_vector
from minilink.core.backends import array_module
from minilink.core.system import System
from minilink.dynamics.abstraction.mechanical import MechanicalSystem

###############################################################################
# Public API
###############################################################################


class ComputedTorque(System):
    """Computed torque with built-in outer PD on joint space (Pyro ``nonlinear.py``).

    Desired acceleration ``qdd = Kp (q_d - q) + Kd (dq_d - dq)`` (regulation
    uses ``dq_d = 0``), then ``τ = inverse_dynamics(q, dq, qdd)``.

    ``y`` is the plant's state ``[q; dq]`` (``ctl @ plant``); reference
    ``r`` is ``q_d`` (dim ``dof``) or stacked ``[q_d; dq_d]`` when
    ``tracking_ref=True``.

    The embedded model (``inverse_dynamics``) reads the controller's own
    ``params["model"]``, a copy of the plant's numbers taken at construction:
    a model mismatch is one edit of that subtree (DESIGN §4, *Embedded-model
    params rule*).
    """

    plot_space = "absolute_qdq"  # plot_control_law sweeps the (q, dq) plane

    def __init__(
        self,
        plant: MechanicalSystem,
        *,
        tracking_ref: bool = True,
        Kp=None,
        Kd=None,
    ):
        require_mechanical_plant(plant)

        super().__init__()

        # The embedded model: the plant's equations, on the controller's own numbers
        self.plant = plant
        self.tracking_ref = tracking_ref
        n = plant.dof
        ref_dim = 2 * n if tracking_ref else n

        # Parameters: the outer PD gains, per joint, and the model's numbers
        self.params = {
            "Kp": as_dof_vector(25.0 if Kp is None else Kp, n),
            "Kd": as_dof_vector(8.0 if Kd is None else Kd, n),
            "model": copy.deepcopy(plant.params),
        }

        # Ports: the reference r (q_d, or [q_d; dq_d] when tracking), the state
        # y = [q; dq], and the torque u from the bound law
        self.add_input_port("r", dim=ref_dim, nominal_value=np.zeros(ref_dim))
        self.add_input_port(
            "y",
            dim=2 * n,
            nominal_value=np.zeros(2 * n),
            labels=[f"q{i}" for i in range(n)] + [f"dq{i}" for i in range(n)],
        )
        ctl_fn = self.ctl_tracking if tracking_ref else self.ctl_regulation
        self.add_output_port(
            "u",
            dim=n,
            function=ctl_fn,
            dependencies=("r", "y"),
        )

    def ctl(self, x, u, t=0, params=None):
        """Evaluate the bound regulation or tracking law."""
        return self.outputs["u"].compute(x, u, t, params)

    def ctl_regulation(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        plant = self.plant  # The embedded model, on the controller's own numbers
        model = params["model"]
        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])

        n = plant.dof
        q_d = u[:n]
        q = u[n : 2 * n]
        dq = u[2 * n : 3 * n]

        # q̈_des = Kp (q_d − q) − Kd q̇: the outer PD on the joints, toward a reference at rest
        qdd_des = Kp * (q_d - q) - Kd * dq

        # τ = inverse_dynamics(q, q̇, q̈_des): the torque that produces that acceleration
        tau = plant.inverse_dynamics(q, dq, qdd_des, params=model)
        tau = xp.asarray(tau).reshape(-1)

        return tau

    def ctl_tracking(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        plant = self.plant  # The embedded model, on the controller's own numbers
        model = params["model"]
        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])

        n = plant.dof
        r = u[: 2 * n]
        q = u[2 * n : 3 * n]
        dq = u[3 * n : 4 * n]
        q_d = r[:n]
        dq_d = r[n : 2 * n]

        # q̈_des = Kp (q_d − q) + Kd (q̇_d − q̇): the outer PD on the joints
        qdd_des = Kp * (q_d - q) + Kd * (dq_d - dq)

        # τ = inverse_dynamics(q, q̇, q̈_des): the torque that produces that acceleration
        tau = plant.inverse_dynamics(q, dq, qdd_des, params=model)
        tau = xp.asarray(tau).reshape(-1)

        return tau


class SlidingMode(ComputedTorque):
    """Joint-space sliding-mode law on a mechanical plant (Pyro ``nonlinear.py``).

    Inherits computed-torque port layout (``r``, ``y = [q; dq]``). Fixed-goal
    regulation uses ``dq_d = ddq_d = 0``; stacked ``r = [q_d; dq_d]`` when
    ``tracking_ref=True``.

    The law drives the tracking error onto the sliding surface ``s = ė + λ e``
    with a computed-torque reaching term and a discontinuous gain (the
    equations are written in :meth:`ctl_regulation` and :meth:`ctl_tracking`).
    """

    plot_space = "absolute_qdq"  # plot_control_law sweeps the (q, dq) plane

    def __init__(
        self,
        plant: MechanicalSystem,
        *,
        tracking_ref: bool = True,
        lam=None,
        gain=None,
        nab=None,
    ):
        super().__init__(plant, tracking_ref=tracking_ref, Kp=0.0, Kd=0.0)
        n = plant.dof

        # Parameters: the surface slope λ, the switching gain, and the switching gain
        # scaled by the inertia, per joint, and the model's numbers
        self.params = {
            "lam": as_dof_vector(1.0 if lam is None else lam, n),
            "gain": as_dof_vector(1.0 if gain is None else gain, n),
            "nab": as_dof_vector(0.1 if nab is None else nab, n),
            "model": copy.deepcopy(plant.params),
        }

        # Ports: the parent's layout, its torque output bound to this law
        ctl_fn = self.ctl_tracking if tracking_ref else self.ctl_regulation
        self.outputs["u"].compute = ctl_fn

        # Solver hints: the sign(s) term switches
        self.solver_info["discontinuous_behavior"] = True

    def ctl_regulation(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        plant = self.plant  # The embedded model, on the controller's own numbers
        model = params["model"]
        lam = xp.asarray(params["lam"])
        gain = xp.asarray(params["gain"])
        nab = xp.asarray(params["nab"])

        # A fixed goal: the reference at rest, q̇_d = q̈_d = 0
        n = plant.dof
        q_d = u[:n]
        q = u[n : 2 * n]
        dq = u[2 * n : 3 * n]
        dq_d = xp.zeros(n)
        ddq_d = xp.zeros(n)

        # s = ė + λ e on the tracking error e = q − q_d, and the reaching acceleration
        # q̈_r = q̈_d − λ ė
        q_e = q - q_d
        dq_e = dq - dq_d
        s = dq_e + lam * q_e
        ddq_r = ddq_d - lam * dq_e

        # τ = inverse_dynamics(q, q̇, q̈_r) − K(q) sign(s),  K(q) = diag(gain) + H(q) diag(nab)
        H = plant.H(q, model)
        K = xp.diag(gain) + H @ xp.diag(nab)
        u_computed = plant.inverse_dynamics(q, dq, ddq_r, params=model)
        u_discontinuous = K @ xp.sign(s)
        tau = u_computed - u_discontinuous
        tau = xp.asarray(tau).reshape(-1)

        return tau

    def ctl_tracking(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        plant = self.plant  # The embedded model, on the controller's own numbers
        model = params["model"]
        lam = xp.asarray(params["lam"])
        gain = xp.asarray(params["gain"])
        nab = xp.asarray(params["nab"])

        # The reference [q_d; q̇_d], at rest in acceleration: q̈_d = 0
        n = plant.dof
        r = u[: 2 * n]
        q = u[2 * n : 3 * n]
        dq = u[3 * n : 4 * n]
        q_d = r[:n]
        dq_d = r[n : 2 * n]
        ddq_d = xp.zeros(n)

        # s = ė + λ e on the tracking error e = q − q_d, and the reaching acceleration
        # q̈_r = q̈_d − λ ė
        q_e = q - q_d
        dq_e = dq - dq_d
        s = dq_e + lam * q_e
        ddq_r = ddq_d - lam * dq_e

        # τ = inverse_dynamics(q, q̇, q̈_r) − K(q) sign(s),  K(q) = diag(gain) + H(q) diag(nab)
        H = plant.H(q, model)
        K = xp.diag(gain) + H @ xp.diag(nab)
        u_computed = plant.inverse_dynamics(q, dq, ddq_r, params=model)
        u_discontinuous = K @ xp.sign(s)
        tau = u_computed - u_discontinuous
        tau = xp.asarray(tau).reshape(-1)

        return tau


###############################################################################
# Internal machinery
###############################################################################


def require_mechanical_plant(plant):
    """Refuse a plant that is not a ``MechanicalSystem``: the law needs its model."""
    if not isinstance(plant, MechanicalSystem):
        raise TypeError("ComputedTorque requires a MechanicalSystem plant")
