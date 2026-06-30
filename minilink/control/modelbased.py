"""Model-based mechanical control (Pyro ``nonlinear.py``)."""

from __future__ import annotations

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import StaticSystem
from minilink.dynamics.abstraction.mechanical import MechanicalSystem

from minilink.control.impedance import _as_dof_vector


class ComputedTorqueController(StaticSystem):
    """Computed torque with built-in outer PD on joint space.

    Desired acceleration ``qdd = Kp (q_d - q) + Kd (dq_d - dq)`` (regulation
    uses ``dq_d = 0``), then ``τ = inverse_dynamics(q, dq, qdd)``.

    Wire ``y`` from ``[q; dq]`` (``closed_loop`` / ``closed_loop_qdq``); reference
    ``r`` is ``q_d`` (dim ``dof``) or stacked ``[q_d; dq_d]`` when
    ``tracking_ref=True``.
    """

    feedback_profile = "modelbased"

    def __init__(
        self,
        plant: MechanicalSystem,
        *,
        tracking_ref: bool = True,
        Kp=None,
        Kd=None,
    ):
        if not isinstance(plant, MechanicalSystem):
            raise TypeError("ComputedTorqueController requires a MechanicalSystem plant")
        super().__init__()
        self.plant = plant
        n = plant.dof
        ref_dim = 2 * n if tracking_ref else n
        self.name = "Computed Torque Controller"
        self.params = {
            "Kp": _as_dof_vector(25.0 if Kp is None else Kp, n),
            "Kd": _as_dof_vector(8.0 if Kd is None else Kd, n),
        }

        self.add_input_port("r", dim=ref_dim, nominal_value=np.zeros(ref_dim))
        self.add_input_port("y", dim=2 * n, nominal_value=np.zeros(2 * n))
        self.add_output_port(
            "u",
            dim=n,
            function=self.ctl,
            dependencies=("r", "y"),
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)
        n = self.plant.dof
        ref_dim = self.inputs["r"].dim

        r = u[:ref_dim]
        q = u[ref_dim : ref_dim + n]
        dq = u[ref_dim + n : ref_dim + 2 * n]

        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])

        if ref_dim == n:
            qdd_des = Kp * (r - q) - Kd * dq
        else:
            q_d = r[:n]
            dq_d = r[n:]
            qdd_des = Kp * (q_d - q) + Kd * (dq_d - dq)

        tau = self.plant.inverse_dynamics(q, dq, qdd_des)
        return xp.asarray(tau).reshape(-1)


class SlidingModeController(StaticSystem):
    """Joint-space sliding-mode law on a mechanical plant.

    Reference ``r`` is stacked ``[q_d; dq_d]`` (dim ``2·dof``). The reaching
    law drives the sliding surface ``s = (dq_d - dq) + Λ (q_d - q)`` toward zero:

        a = a_d - η sign(s),   τ = inverse_dynamics(q, dq, a)
    """

    feedback_profile = "modelbased"

    def __init__(self, plant: MechanicalSystem, *, Lambda=None, eta=None):
        if not isinstance(plant, MechanicalSystem):
            raise TypeError("SlidingModeController requires a MechanicalSystem plant")
        super().__init__()
        self.plant = plant
        n = plant.dof
        self.name = "Sliding Mode Controller"
        self.params = {
            "Lambda": _as_dof_vector(1.0 if Lambda is None else Lambda, n),
            "eta": _as_dof_vector(1.0 if eta is None else eta, n),
        }

        self.add_input_port("q", dim=n, nominal_value=np.zeros(n))
        self.add_input_port("dq", dim=n, nominal_value=np.zeros(n))
        self.add_input_port("r", dim=2 * n, nominal_value=np.zeros(2 * n))
        self.add_output_port(
            "u",
            dim=n,
            function=self.ctl,
            dependencies=("q", "dq", "r"),
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)
        n = self.plant.dof

        q = u[:n]
        dq = u[n : 2 * n]
        q_d = u[2 * n : 3 * n]
        dq_d = u[3 * n : 4 * n]

        Lambda = xp.asarray(params["Lambda"])
        eta = xp.asarray(params["eta"])

        e = q_d - q
        ed = dq_d - dq
        s = ed + Lambda * e
        qdd_des = -Lambda * ed - eta * xp.sign(s)
        tau = self.plant.inverse_dynamics(q, dq, qdd_des)
        return xp.asarray(tau).reshape(-1)
