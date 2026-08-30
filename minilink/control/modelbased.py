"""Model-based mechanical control (Pyro ``nonlinear.py``)."""

from __future__ import annotations

import numpy as np

from minilink.control.impedance import _as_dof_vector
from minilink.core.backends import array_module
from minilink.core.feedback import Controller
from minilink.dynamics.abstraction.mechanical import MechanicalSystem


class ComputedTorqueController(Controller):
    """Computed torque with built-in outer PD on joint space.

    Desired acceleration ``qdd = Kp (q_d - q) + Kd (dq_d - dq)`` (regulation
    uses ``dq_d = 0``), then ``τ = inverse_dynamics(q, dq, qdd)``.

    Wire ``y`` from ``[q; dq]`` (``closed_loop`` / ``closed_loop_qdq``); reference
    ``r`` is ``q_d`` (dim ``dof``) or stacked ``[q_d; dq_d]`` when
    ``tracking_ref=True``.

    The embedded model (``inverse_dynamics``) reads the referenced plant's
    **live** ``self.params``; the controller's own params carry gains only.
    See DESIGN §4 (*Embedded-model params rule*).
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
            raise TypeError(
                "ComputedTorqueController requires a MechanicalSystem plant"
            )
        super().__init__()
        self.plant = plant
        n = plant.dof
        ref_dim = 2 * n if tracking_ref else n
        self.tracking_ref = tracking_ref
        self.name = "Computed Torque Controller"
        self.params = {
            "Kp": _as_dof_vector(25.0 if Kp is None else Kp, n),
            "Kd": _as_dof_vector(8.0 if Kd is None else Kd, n),
        }

        self.add_input_port("r", dim=ref_dim, nominal_value=np.zeros(ref_dim))
        self.add_input_port(
            "y",
            dim=2 * n,
            nominal_value=np.zeros(2 * n),
            labels=[f"q{i}" for i in range(n)] + [f"dq{i}" for i in range(n)],
        )
        ctl_fn = self._ctl_tracking if tracking_ref else self._ctl_regulation
        self.add_output_port(
            "u",
            dim=n,
            function=ctl_fn,
            dependencies=("r", "y"),
        )

    def ctl(self, x, u, t=0, params=None):
        """Evaluate the bound regulation or tracking law."""
        return self.outputs["u"].compute(x, u, t, params)

    def _ctl_regulation(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)
        n = self.plant.dof

        q_d = u[:n]
        q = u[n : 2 * n]
        dq = u[2 * n : 3 * n]

        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])

        qdd_des = Kp * (q_d - q) - Kd * dq
        # Embedded model: plant live self.params (DESIGN §4 embedded-model rule).
        tau = self.plant.inverse_dynamics(q, dq, qdd_des)
        return xp.asarray(tau).reshape(-1)

    def _ctl_tracking(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)
        n = self.plant.dof

        r = u[: 2 * n]
        q = u[2 * n : 3 * n]
        dq = u[3 * n : 4 * n]

        q_d = r[:n]
        dq_d = r[n : 2 * n]
        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])

        qdd_des = Kp * (q_d - q) + Kd * (dq_d - dq)
        # Embedded model: plant live self.params (DESIGN §4 embedded-model rule).
        tau = self.plant.inverse_dynamics(q, dq, qdd_des)
        return xp.asarray(tau).reshape(-1)


class SlidingModeController(ComputedTorqueController):
    """Joint-space sliding-mode law on a mechanical plant (Pyro ``nonlinear.py``).

    Inherits computed-torque port layout (``r``, ``y = [q; dq]``). Fixed-goal
    regulation uses ``dq_d = ddq_d = 0``; stacked ``r = [q_d; dq_d]`` when
    ``tracking_ref=True``.

    Sliding surface ``s = dq_e + lam * q_e`` with ``q_e = q - q_d``,
    ``dq_e = dq - dq_d``. Reaching law:

        ddq_r = ddq_d - lam * dq_e
        K(q) = diag(gain) + H(q) @ diag(nab)
        τ = inverse_dynamics(q, dq, ddq_r) - K(q) @ sign(s)
    """

    feedback_profile = "modelbased"

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
        self.name = "Sliding Mode Controller"
        self.params = {
            "lam": _as_dof_vector(1.0 if lam is None else lam, n),
            "gain": _as_dof_vector(1.0 if gain is None else gain, n),
            "nab": _as_dof_vector(0.1 if nab is None else nab, n),
        }
        ctl_fn = self._ctl_tracking if tracking_ref else self._ctl_regulation
        self.outputs["u"].compute = ctl_fn
        self.solver_info["discontinuous_behavior"] = True

    def _ctl_regulation(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)
        n = self.plant.dof

        q_d = u[:n]
        q = u[n : 2 * n]
        dq = u[2 * n : 3 * n]
        dq_d = xp.zeros(n)

        return self._sliding_torque(q, dq, q_d, dq_d, params)

    def _ctl_tracking(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        n = self.plant.dof

        r = u[: 2 * n]
        q = u[2 * n : 3 * n]
        dq = u[3 * n : 4 * n]

        q_d = r[:n]
        dq_d = r[n : 2 * n]

        return self._sliding_torque(q, dq, q_d, dq_d, params)

    def _sliding_torque(self, q, dq, q_d, dq_d, params):
        xp = array_module(q, dq, q_d, dq_d)

        ddq_d = xp.zeros(self.plant.dof)
        lam = xp.asarray(params["lam"])
        gain = xp.asarray(params["gain"])
        nab = xp.asarray(params["nab"])

        q_e = q - q_d
        dq_e = dq - dq_d
        s = dq_e + lam * q_e
        ddq_r = ddq_d - lam * dq_e

        # Embedded model: plant live self.params (DESIGN §4 embedded-model rule).
        H = self.plant.H(q)
        K = xp.diag(gain) + H @ xp.diag(nab)
        u_computed = self.plant.inverse_dynamics(q, dq, ddq_r)
        u_discontinuous = K @ xp.sign(s)
        return xp.asarray(u_computed - u_discontinuous).reshape(-1)
