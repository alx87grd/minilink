"""Manipulator-specific impedance wrappers (Pyro ``robotcontrollers.py``)."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from minilink.control.impedance import ImpedanceController, _as_dof_vector
from minilink.core.backends import array_module
from minilink.core.system import StaticSystem
from minilink.dynamics.abstraction.manipulator import Manipulator
from minilink.dynamics.abstraction.mechanical import MechanicalSystem

GravityHook = Callable[..., np.ndarray]


def _gravity_feedforward(plant, gravity, q, params=None):
    xp = array_module(q)
    if gravity is not None:
        try:
            g = gravity(q, params)
        except TypeError:
            g = gravity(q)
    else:
        if plant is None:
            raise ValueError("gravity_comp requires plant or gravity hook")
        g = plant.g(q, params)
    return xp.asarray(g, dtype=float).reshape(-1)


def _impedance_joint_torque(ref_dim, n, r, q, dq, Kp, Kd, xp):
    if ref_dim == n:
        return Kp * (r - q) - Kd * dq
    pos_d = r[:n]
    vel_d = r[n:]
    return Kp * (pos_d - q) + Kd * (vel_d - dq)


class ModelJointImpedance(StaticSystem):
    """Joint-space impedance on a mechanical plant with optional gravity feedforward.

    Law: ``τ = Kp e + Kd ė + g(q)`` when ``gravity_comp`` is enabled.
    """

    feedback_profile = "impedance"

    def __init__(
        self,
        plant: MechanicalSystem,
        *,
        tracking_ref: bool = False,
        gravity_comp: bool = False,
        gravity: GravityHook | None = None,
        Kp=None,
        Kd=None,
    ):
        if not isinstance(plant, MechanicalSystem):
            raise TypeError("ModelJointImpedance requires a MechanicalSystem plant")
        if gravity_comp and gravity is None and not hasattr(plant, "g"):
            raise TypeError("plant must implement g(q) when gravity_comp=True")
        super().__init__()
        self.plant = plant
        self.gravity = gravity
        self.dof = plant.dof
        n = self.dof
        ref_dim = 2 * n if tracking_ref else n

        self.params = {
            "Kp": _as_dof_vector(10.0 if Kp is None else Kp, n),
            "Kd": _as_dof_vector(1.0 if Kd is None else Kd, n),
            "gravity_comp": bool(gravity_comp),
        }
        self.name = "Joint Impedance"

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
        n = self.dof
        ref_dim = self.inputs["r"].dim

        r = u[:ref_dim]
        q = u[ref_dim : ref_dim + n]
        dq = u[ref_dim + n : ref_dim + 2 * n]

        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])
        tau = _impedance_joint_torque(ref_dim, n, r, q, dq, Kp, Kd, xp)

        if params.get("gravity_comp", False):
            tau = tau + _gravity_feedforward(self.plant, self.gravity, q)

        return tau.reshape(-1)


def JointImpedance(
    plant_or_dof=1,
    *,
    dof=None,
    tracking_ref: bool = False,
    gravity_comp: bool = False,
    gravity: GravityHook | None = None,
    Kp=None,
    Kd=None,
    **impedance_kwargs,
):
    """Joint-space impedance: generic (``dof`` only) or plant-aware.

    ``JointImpedance(dof=n)`` returns a domain-neutral
    :class:`~minilink.control.impedance.ImpedanceController`.

    ``JointImpedance(plant, gravity_comp=True)`` returns
    :class:`ModelJointImpedance` with optional ``τ += g(q)`` (or a custom
    ``gravity(q)`` hook).
    """
    if isinstance(plant_or_dof, MechanicalSystem):
        if dof is not None:
            raise ValueError("pass either a plant or dof=, not both")
        if impedance_kwargs:
            raise ValueError("plant-based JointImpedance does not accept ImpedanceController kwargs")
        return ModelJointImpedance(
            plant_or_dof,
            tracking_ref=tracking_ref,
            gravity_comp=gravity_comp,
            gravity=gravity,
            Kp=Kp,
            Kd=Kd,
        )

    if gravity_comp or gravity is not None:
        raise ValueError("gravity_comp and gravity require a MechanicalSystem plant")

    n = int(dof if dof is not None else plant_or_dof)
    return ImpedanceController(
        dof=n,
        tracking_ref=tracking_ref,
        **impedance_kwargs,
    )


class TaskImpedance(StaticSystem):
    """Task-space impedance using an internal kinematic model.

    Joint measurements ``y = [q; dq]`` are mapped to task space via the
    plant's ``forward_kinematics`` and ``J(q)``; no task-space plant ports are
    required for control (``p`` / ``pdot`` remain for visualization).

        τ = J(q)^T (Kp e_p + Kd e_v) [+ g(q)]

    Reference ``r`` has dim ``task_dim`` (regulation) or ``2·task_dim`` for
    stacked ``[p_d; pdot_d]`` tracking.
    """

    feedback_profile = "task"

    def __init__(
        self,
        plant: Manipulator,
        *,
        tracking_ref: bool = False,
        gravity_comp: bool = False,
        gravity: GravityHook | None = None,
        Kp=None,
        Kd=None,
    ):
        super().__init__()
        self.plant = plant
        self.gravity = gravity
        self.dof = plant.dof
        self.task_dim = plant.task_dim
        n = self.task_dim
        ref_dim = 2 * n if tracking_ref else n

        Kp = np.full(n, 10.0) if Kp is None else _as_dof_vector(Kp, n)
        Kd = np.full(n, 1.0) if Kd is None else _as_dof_vector(Kd, n)
        self.params = {
            "Kp": Kp,
            "Kd": Kd,
            "gravity_comp": bool(gravity_comp),
        }
        self.name = "Task Impedance"

        self.add_input_port("r", dim=ref_dim, nominal_value=np.zeros(ref_dim))
        self.add_input_port("y", dim=2 * self.dof, nominal_value=np.zeros(2 * self.dof))
        self.add_output_port(
            "u",
            dim=self.dof,
            function=self.ctl,
            dependencies=("r", "y"),
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        task_dim = self.task_dim
        dof = self.dof
        ref_dim = self.inputs["r"].dim

        r = u[:ref_dim]
        q = u[ref_dim : ref_dim + dof]
        dq = u[ref_dim + dof : ref_dim + 2 * dof]

        p = xp.asarray(self.plant.forward_kinematics(q))
        J = xp.asarray(self.plant.J(q))
        pdot = J @ dq

        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])

        if ref_dim == task_dim:
            e_p = r - p
            u_task = Kp * e_p - Kd * pdot
        else:
            p_d = r[:task_dim]
            pdot_d = r[task_dim:]
            u_task = Kp * (p_d - p) + Kd * (pdot_d - pdot)

        tau = J.T @ u_task
        if params.get("gravity_comp", False):
            tau = tau + _gravity_feedforward(self.plant, self.gravity, q)

        return tau.reshape(-1)


class TaskKinematic(StaticSystem):
    """Task-space kinematic controller for velocity-controlled manipulators.

    Joint measurements ``y = q``; output ``u = dq`` drives a
    :class:`~minilink.dynamics.catalog.manipulators.arms.SpeedControlledManipulator`.

        v_task = Kp (p_d - p)
        dq = J(q)^# v_task

    Reference ``r`` is the desired end-effector position ``p_d`` (dim ``task_dim``).
    """

    feedback_profile = "kinematic"

    def __init__(self, plant, *, Kp=None):
        if not hasattr(plant, "forward_kinematics") or not hasattr(plant, "J"):
            raise TypeError("TaskKinematic requires a plant with forward_kinematics and J")
        dof = plant.dof
        task_dim = getattr(plant, "task_dim", plant.effector_dim)
        if dof < task_dim:
            raise ValueError(
                f"TaskKinematic requires dof >= task_dim, got {dof} and {task_dim}"
            )
        super().__init__()
        self.plant = plant
        self.dof = dof
        self.task_dim = task_dim
        self.params = {
            "Kp": _as_dof_vector(1.0 if Kp is None else Kp, task_dim),
        }
        self.name = "Task Kinematic"

        self.add_input_port("r", dim=task_dim, nominal_value=np.zeros(task_dim))
        self.add_input_port("y", dim=dof, nominal_value=np.zeros(dof))
        self.add_output_port(
            "u",
            dim=dof,
            function=self.ctl,
            dependencies=("r", "y"),
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        n = self.task_dim
        dof = self.dof

        p_d = u[:n]
        q = u[n : n + dof]

        p = self.plant.forward_kinematics(q)
        J = self.plant.J(q)
        Kp = xp.asarray(params["Kp"])

        v_task = Kp * (p_d - p)
        if dof == n:
            return xp.linalg.solve(J, v_task)
        return xp.linalg.pinv(J) @ v_task


class TaskKinematicNullspace(TaskKinematic):
    """Kinematic controller with a secondary joint posture in the nullspace.

        dq = J^# v_task + (I - J^# J) K_null (q_null - q)

    Primary reference ``r`` is ``p_d``; secondary reference ``r_null`` is
    ``q_null``. Gain ``K_null`` lives in ``params``.
    """

    def __init__(self, plant, *, Kp=None, K_null=None):
        if not hasattr(plant, "forward_kinematics") or not hasattr(plant, "J"):
            raise TypeError("TaskKinematic requires a plant with forward_kinematics and J")
        dof = plant.dof
        task_dim = getattr(plant, "task_dim", plant.effector_dim)
        if dof < task_dim:
            raise ValueError(
                f"TaskKinematic requires dof >= task_dim, got {dof} and {task_dim}"
            )
        super(TaskKinematic, self).__init__()
        self.plant = plant
        self.dof = dof
        self.task_dim = task_dim
        self.params = {
            "Kp": _as_dof_vector(1.0 if Kp is None else Kp, task_dim),
            "K_null": _as_dof_vector(1.0 if K_null is None else K_null, dof),
        }
        self.name = "Task Kinematic Nullspace"

        self.add_input_port("r", dim=task_dim, nominal_value=np.zeros(task_dim))
        self.add_input_port("r_null", dim=dof, nominal_value=np.zeros(dof))
        self.add_input_port("y", dim=dof, nominal_value=np.zeros(dof))
        self.add_output_port(
            "u",
            dim=dof,
            function=self.ctl,
            dependencies=("r", "r_null", "y"),
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        n = self.task_dim
        dof = self.dof

        p_d = u[:n]
        q_null = u[n : n + dof]
        q = u[n + dof : n + 2 * dof]

        p = self.plant.forward_kinematics(q)
        J = self.plant.J(q)
        Kp = xp.asarray(params["Kp"])
        K_null = xp.asarray(params["K_null"])

        v_task = Kp * (p_d - p)
        J_pinv = xp.linalg.pinv(J)
        null_proj = xp.eye(dof) - J_pinv @ J
        return J_pinv @ v_task + null_proj @ (K_null * (q_null - q))
