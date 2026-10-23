"""Manipulator controllers: joint and task impedance, task-space kinematics (Pyro ``robotcontrollers.py``)."""

from __future__ import annotations

import inspect
from collections.abc import Callable

import numpy as np

from minilink.control.siso import as_dof_vector
from minilink.core.backends import array_module
from minilink.core.system import System
from minilink.dynamics.abstraction.manipulator import Manipulator
from minilink.dynamics.abstraction.mechanical import MechanicalSystem

GravityHook = Callable[..., np.ndarray]


###############################################################################
# Public API
###############################################################################


class JointImpedance(System):
    """Joint-space impedance on a mechanical plant with optional gravity feedforward.

    Law: ``τ = Kp e + Kd ė + g(q)`` when ``gravity_comp`` is enabled.
    """

    plot_space = "error_qdq"  # plot_control_law sweeps the (e, de) plane

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
        require_mechanical_plant(plant)
        require_gravity_model(plant, gravity_comp, gravity)

        super().__init__()

        # The embedded plant model, and its custom gravity hook
        self.plant = plant
        self.gravity = gravity
        self.dof = plant.dof

        n = self.dof
        ref_dim = 2 * n if tracking_ref else n

        # Parameters: the joint stiffness and damping, and the feedforward switch
        self.params = {
            "Kp": as_dof_vector(10.0 if Kp is None else Kp, n),
            "Kd": as_dof_vector(1.0 if Kd is None else Kd, n),
            "gravity_comp": bool(gravity_comp),
        }

        # Ports: the reference and the measurement [q; dq] in, the joint torque out
        self.add_input_port("r", dim=ref_dim, nominal_value=np.zeros(ref_dim))
        self.add_input_port(
            "y",
            dim=2 * n,
            nominal_value=np.zeros(2 * n),
            labels=[f"q{i}" for i in range(n)] + [f"dq{i}" for i in range(n)],
        )
        self.add_output_port(
            "u",
            dim=n,
            function=self.ctl,
            dependencies=("r", "y"),
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        plant = self.plant
        gravity = self.gravity
        n = self.dof
        ref_dim = self.inputs["r"].dim
        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])

        r = u[:ref_dim]
        q = u[ref_dim : ref_dim + n]
        dq = u[ref_dim + n : ref_dim + 2 * n]

        # τ = Kp e + Kd ė: regulation holds the rate reference at zero, tracking
        # reads the stacked reference [q_d; dq_d]
        if ref_dim == n:
            tau = Kp * (r - q) - Kd * dq
        else:
            q_d = r[:n]
            dq_d = r[n:]
            tau = Kp * (q_d - q) + Kd * (dq_d - dq)

        # τ ← τ + g(q), when the gravity feedforward is on
        if params.get("gravity_comp", False):
            tau = tau + gravity_feedforward(plant, gravity, q)

        return tau.reshape(-1)


class TaskImpedance(System):
    """Task-space impedance using an internal kinematic model.

    Joint measurements ``y = [q; dq]`` are mapped to task space via the
    plant's ``forward_kinematics`` and ``J(q)``; no task-space plant ports are
    required for control (``p`` / ``pdot`` remain for visualization).

        τ = J(q)ᵀ (Kp e_p + Kd e_v) [+ g(q)]

    Reference ``r`` has dim ``task_dim`` (regulation) or ``2·task_dim`` for
    stacked ``[p_d; pdot_d]`` tracking.

    Optional visualization (``show_task_force=True``) draws:

    * a ball at the desired task point ``p_d`` (frame ``task_target``);
    * the commanded spring-damper task force ``f_task = Kp e + Kd ė`` at the
      current end-effector (frame ``task_force``) — not gravity feedforward and
      not the mapped joint torque.

    In a diagram those frames namespace to ``controller:task_target`` /
    ``controller:task_force``.
    """

    plot_space = "absolute_qdq"  # plot_control_law sweeps the (q, dq) plane

    def __init__(
        self,
        plant: Manipulator,
        *,
        tracking_ref: bool = False,
        gravity_comp: bool = False,
        gravity: GravityHook | None = None,
        Kp=None,
        Kd=None,
        show_task_force: bool = False,
        task_force_scale: float = 0.01,
        task_target_radius: float = 0.04,
    ):
        super().__init__()

        # The embedded kinematic model, and its custom gravity hook
        self.plant = plant
        self.gravity = gravity
        self.dof = plant.dof
        self.task_dim = plant.task_dim

        # Graphics: the target ball and the task-force arrow
        self.show_task_force = bool(show_task_force)
        self.task_force_scale = float(task_force_scale)
        self.task_target_radius = float(task_target_radius)

        n = self.task_dim
        ref_dim = 2 * n if tracking_ref else n

        # Parameters: the task stiffness and damping, and the feedforward switch
        Kp = np.full(n, 10.0) if Kp is None else as_dof_vector(Kp, n)
        Kd = np.full(n, 1.0) if Kd is None else as_dof_vector(Kd, n)
        self.params = {
            "Kp": Kp,
            "Kd": Kd,
            "gravity_comp": bool(gravity_comp),
        }

        # Ports: the task reference and the joint measurement [q; dq] in, the joint
        # torque out
        self.add_input_port("r", dim=ref_dim, nominal_value=np.zeros(ref_dim))
        self.add_input_port(
            "y",
            dim=2 * self.dof,
            nominal_value=np.zeros(2 * self.dof),
            labels=[f"q{i}" for i in range(self.dof)]
            + [f"dq{i}" for i in range(self.dof)],
        )
        self.add_output_port(
            "u",
            dim=self.dof,
            function=self.ctl,
            dependencies=("r", "y"),
        )

    def task_quantities(self, u, params=None):
        """Return ``(q, p, p_d, J, f_task)`` for the current reference and measurement."""
        params = self.params if params is None else params
        xp = array_module(u)

        plant = self.plant
        task_dim = self.task_dim
        dof = self.dof
        ref_dim = self.inputs["r"].dim
        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])

        r = u[:ref_dim]
        q = u[ref_dim : ref_dim + dof]
        dq = u[ref_dim + dof : ref_dim + 2 * dof]

        # Task position and velocity: p = f(q), ṗ = J(q) q̇
        p = xp.asarray(plant.forward_kinematics(q))
        J = xp.asarray(plant.J(q))
        pdot = J @ dq

        # Task spring-damper force f = Kp e_p + Kd e_v (regulation: ṗ_d = 0)
        if ref_dim == task_dim:
            p_d = r
            f_task = Kp * (r - p) - Kd * pdot
        else:
            p_d = r[:task_dim]
            pdot_d = r[task_dim:]
            f_task = Kp * (p_d - p) + Kd * (pdot_d - pdot)

        return q, p, p_d, J, f_task

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        plant = self.plant
        gravity = self.gravity

        q, _, _, J, f_task = self.task_quantities(u, params)

        # τ = Jᵀ f_task
        tau = J.T @ f_task

        # τ ← τ + g(q), when the gravity feedforward is on
        if params.get("gravity_comp", False):
            tau = tau + gravity_feedforward(plant, gravity, q)

        return tau.reshape(-1)

    def tf(self, x, u, t=0, params=None):
        if not self.show_task_force:
            return {}

        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.catalog.shapes import point_pose

        _, p, p_d, _, _ = self.task_quantities(u, params)

        # The target ball at p_d, the force arrow at the end effector p
        return {
            "task_target": point_pose(p_d),
            "task_force": point_pose(p),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        if not self.show_task_force:
            return {}

        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation.primitives import Arrow, Sphere

        _, _, _, _, f_task = self.task_quantities(u, params)
        f_task = np.asarray(f_task, dtype=float).reshape(-1)
        base = np.zeros(f_task.size)

        # A green ball at the target, a crimson arrow for the spring-damper force
        return {
            "task_target": [
                Sphere(
                    radius=self.task_target_radius,
                    center=(0.0, 0.0, 0.0),
                    color="limegreen",
                    opacity=0.85,
                )
            ],
            "task_force": [
                Arrow(
                    base=base,
                    vector=f_task,
                    scale=self.task_force_scale,
                    color="crimson",
                    linewidth=2.5,
                )
            ],
        }


class TaskKinematic(System):
    """Task-space kinematic controller for velocity-controlled manipulators.

    Joint measurements ``y = q``; output ``u = dq`` drives a
    :class:`~minilink.dynamics.catalog.manipulators.arms.KinematicManipulator`.

        v_task = Kp (p_d − p)
        dq = J(q)⁺ v_task

    Reference ``r`` is the desired end-effector position ``p_d`` (dim ``task_dim``).
    """

    plot_space = "measurement"  # the task reference is not a joint error

    def __init__(self, plant, *, Kp=None):
        dof, task_dim = kinematic_dims_of(plant)

        super().__init__()

        # The embedded kinematic model
        self.plant = plant
        self.dof = dof
        self.task_dim = task_dim

        # Parameters: the task gain
        self.params = {
            "Kp": as_dof_vector(1.0 if Kp is None else Kp, task_dim),
        }

        # Ports: the task reference and the joint measurement q in, the joint rate out
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

        plant = self.plant
        n = self.task_dim
        dof = self.dof
        Kp = xp.asarray(params["Kp"])

        p_d = u[:n]
        q = u[n : n + dof]
        p = plant.forward_kinematics(q)
        J = plant.J(q)

        # v = Kp (p_d − p): the task velocity command
        v_task = Kp * (p_d - p)

        # J q̇ = v, solved when J is square, in least squares (q̇ = J⁺ v) when redundant
        if dof == n:
            dq = xp.linalg.solve(J, v_task)
        else:
            dq = xp.linalg.pinv(J) @ v_task

        return dq


class TaskKinematicNullspace(TaskKinematic):
    """Kinematic controller with a secondary joint posture in the nullspace.

        dq = J⁺ v_task + (I − J⁺ J) K_null (q_null − q)

    Primary reference ``r`` is ``p_d``; secondary reference ``r_null`` is
    ``q_null``. Gain ``K_null`` lives in ``params``.
    """

    def __init__(self, plant, *, Kp=None, K_null=None):
        dof, task_dim = kinematic_dims_of(plant)

        # System's constructor, not TaskKinematic's: this block declares its own ports
        super(TaskKinematic, self).__init__()

        # The embedded kinematic model
        self.plant = plant
        self.dof = dof
        self.task_dim = task_dim

        # Parameters: the task gain and the posture gain
        self.params = {
            "Kp": as_dof_vector(1.0 if Kp is None else Kp, task_dim),
            "K_null": as_dof_vector(1.0 if K_null is None else K_null, dof),
        }

        # Ports: the task reference, the posture reference and the joint measurement q
        # in, the joint rate out
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

        plant = self.plant
        n = self.task_dim
        dof = self.dof
        Kp = xp.asarray(params["Kp"])
        K_null = xp.asarray(params["K_null"])

        p_d = u[:n]
        q_null = u[n : n + dof]
        q = u[n + dof : n + 2 * dof]
        p = plant.forward_kinematics(q)
        J = plant.J(q)

        # v = Kp (p_d − p): the task velocity command
        v_task = Kp * (p_d - p)

        # q̇ = J⁺ v + (I − J⁺ J) K_null (q_null − q): the posture acts in the nullspace of J
        J_pinv = xp.linalg.pinv(J)
        null_proj = xp.eye(dof) - J_pinv @ J
        dq = J_pinv @ v_task + null_proj @ (K_null * (q_null - q))

        return dq


###############################################################################
# Internal machinery
###############################################################################


def require_mechanical_plant(plant):
    """Refuse a plant without the mechanical structure the joint law reads."""
    if not isinstance(plant, MechanicalSystem):
        raise TypeError("JointImpedance requires a MechanicalSystem plant")


def require_gravity_model(plant, gravity_comp, gravity):
    """Refuse the gravity feedforward when neither a hook nor the plant's ``g(q)`` supplies it."""
    if gravity_comp and gravity is None and not hasattr(plant, "g"):
        raise TypeError("plant must implement g(q) when gravity_comp=True")


def gravity_feedforward(plant, gravity, q, model_params=None):
    """Gravity feedforward ``g(q)`` from the controller's embedded model.

    ``model_params=None`` (the default) means the embedded model reads the
    referenced plant's **live** ``self.params`` — the controller's own params
    dict (gains) is never forwarded. Diagram-level overrides of the plant
    subsystem do not reach this embedded copy; see DESIGN §4
    (*Embedded-model params rule*).

    A custom hook is ``gravity(q)``, or ``gravity(q, params)`` when its
    signature requires the second argument. Only the signature is probed, so a
    ``TypeError`` raised inside the hook reaches the caller.
    """
    xp = array_module(q)

    # The custom hook, called with the parameters only when its signature needs them
    if gravity is not None:
        try:
            inspect.signature(gravity).bind(q)
            takes_params = False
        except TypeError:
            # The signature requires a second argument
            takes_params = True
        except ValueError:
            # No signature to read: the documented gravity(q)
            takes_params = False
        g = gravity(q, model_params) if takes_params else gravity(q)

    # Otherwise the embedded plant's own g(q)
    else:
        if plant is None:
            raise ValueError("gravity_comp requires plant or gravity hook")
        g = plant.g(q, model_params)

    g = xp.asarray(g, dtype=float).reshape(-1)

    return g


def kinematic_dims_of(plant):
    """``(dof, task_dim)`` of a plant a task kinematic controller can drive."""
    if not hasattr(plant, "forward_kinematics") or not hasattr(plant, "J"):
        raise TypeError("TaskKinematic requires a plant with forward_kinematics and J")

    dof = plant.dof
    task_dim = plant.task_dim if hasattr(plant, "task_dim") else plant.effector_dim
    if dof < task_dim:
        raise ValueError(
            f"TaskKinematic requires dof >= task_dim, got {dof} and {task_dim}"
        )

    return dof, task_dim
