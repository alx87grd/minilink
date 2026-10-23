"""Control: the controller and static law blocks, from reference and sensor to actuation."""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# The band facade for short teaching imports, e.g.
# ``from minilink.control import PID, Impedance, lqr, place`` and
# ``from minilink.planning.trajectory_optimization.mpc import ModelPredictiveController``.
#
# The classical family is ``P``, ``PI``, ``PD``, ``PID`` (one measurement per axis, the
# derivative filtered) and ``Impedance``, ``IntegralImpedance`` (a
# measured ``[q; dq]``, the rate read), six blocks in one grammar: per-axis gains, ``dof``
# independent loops, actuator limits that are infinite by default.
#
# The design verbs (``lqr``, ``place``, ``lqg``, ``lqr_finite_horizon``,
# ``trajectory_lqr``) take a linear model or a plant; their gains (``lqr_gain``,
# ``place_gain``, ``lqr_gain_schedule``) take arrays. Both live in ``control.design``.
#
# Each exported name maps to its (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    # Model predictive control: the block lives with its trajectory optimization planner
    "ModelPredictiveController": (
        "minilink.planning.trajectory_optimization.mpc.controller",
        "ModelPredictiveController",
    ),
    "ComputedTorque": (
        "minilink.control.modelbased",
        "ComputedTorque",
    ),
    "NeuralPolicyController": ("minilink.control.neural", "NeuralPolicyController"),
    "angle_features": ("minilink.control.neural", "angle_features"),
    "PID": ("minilink.control.siso", "PID"),
    "PurePursuit": ("minilink.control.geometric", "PurePursuit"),
    "PI": ("minilink.control.siso", "PI"),
    "PD": ("minilink.control.siso", "PD"),
    "Impedance": ("minilink.control.impedance", "Impedance"),
    "lqr": ("minilink.control.design", "lqr"),
    "lqr_gain": ("minilink.control.design", "lqr_gain"),
    "lqr_gain_schedule": ("minilink.control.design", "lqr_gain_schedule"),
    "lqr_finite_horizon": ("minilink.control.design", "lqr_finite_horizon"),
    "trajectory_lqr": ("minilink.control.design", "trajectory_lqr"),
    "place": ("minilink.control.design", "place"),
    "place_gain": ("minilink.control.design", "place_gain"),
    "lqg": ("minilink.control.design", "lqg"),
    "IntegralImpedance": (
        "minilink.control.impedance",
        "IntegralImpedance",
    ),
    "JointImpedance": ("minilink.control.robotic", "JointImpedance"),
    "P": ("minilink.control.siso", "P"),
    "SlidingMode": (
        "minilink.control.modelbased",
        "SlidingMode",
    ),
    "StateFeedback": (
        "minilink.control.state",
        "StateFeedback",
    ),
    "OutputReference": (
        "minilink.control.state",
        "OutputReference",
    ),
    "TimeVaryingFeedback": (
        "minilink.control.state",
        "TimeVaryingFeedback",
    ),
    "TrajectoryFeedback": (
        "minilink.control.state",
        "TrajectoryFeedback",
    ),
    "TaskImpedance": ("minilink.control.robotic", "TaskImpedance"),
    "TaskKinematic": ("minilink.control.robotic", "TaskKinematic"),
    "TaskKinematicNullspace": (
        "minilink.control.robotic",
        "TaskKinematicNullspace",
    ),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
