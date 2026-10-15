"""Controller and static law blocks (reference/sensor to actuation).

Teaching imports::

    from minilink.control import PID, ImpedanceController, lqr, place
    from minilink.control.mpc import ModelPredictiveController

The classical family is ``P``, ``PI``, ``PD``, ``PID`` (one measurement per axis, the
derivative filtered) and ``ImpedanceController``, ``ImpedanceIntegralController`` (a
measured ``[q; dq]``, the rate read), six blocks in one grammar: per-axis gains, ``dof``
independent loops, actuator limits that are infinite by default.

The design verbs (``lqr``, ``place``, ``lqg``, ``lqr_finite_horizon``,
``trajectory_lqr``) take a linear model or a plant; their gains (``lqr_gain``,
``place_gain``, ``lqr_gain_schedule``) take arrays. Both live in ``control.design``.
"""

from __future__ import annotations

from minilink.core.facade import lazy_facade

_EXPORTS: dict[str, tuple[str, str]] = {
    "ComputedTorqueController": (
        "minilink.control.modelbased",
        "ComputedTorqueController",
    ),
    "NeuralPolicyController": ("minilink.control.neural", "NeuralPolicyController"),
    "angle_features": ("minilink.control.neural", "angle_features"),
    "PID": ("minilink.control.siso", "PID"),
    "PurePursuit": ("minilink.control.geometric", "PurePursuit"),
    "PI": ("minilink.control.siso", "PI"),
    "PD": ("minilink.control.siso", "PD"),
    "ImpedanceController": ("minilink.control.impedance", "ImpedanceController"),
    "lqr": ("minilink.control.design", "lqr"),
    "lqr_gain": ("minilink.control.design", "lqr_gain"),
    "lqr_gain_schedule": ("minilink.control.design", "lqr_gain_schedule"),
    "lqr_finite_horizon": ("minilink.control.design", "lqr_finite_horizon"),
    "trajectory_lqr": ("minilink.control.design", "trajectory_lqr"),
    "place": ("minilink.control.design", "place"),
    "place_gain": ("minilink.control.design", "place_gain"),
    "lqg": ("minilink.control.design", "lqg"),
    "ImpedanceIntegralController": (
        "minilink.control.impedance",
        "ImpedanceIntegralController",
    ),
    "JointImpedance": ("minilink.control.robotic", "JointImpedance"),
    "ModelJointImpedance": ("minilink.control.robotic", "ModelJointImpedance"),
    "P": ("minilink.control.siso", "P"),
    "SlidingModeController": (
        "minilink.control.modelbased",
        "SlidingModeController",
    ),
    "StateFeedbackController": (
        "minilink.control.state",
        "StateFeedbackController",
    ),
    "OutputReferenceController": (
        "minilink.control.state",
        "OutputReferenceController",
    ),
    "TimeVaryingStateFeedbackController": (
        "minilink.control.state",
        "TimeVaryingStateFeedbackController",
    ),
    "TrajectoryFeedbackController": (
        "minilink.control.state",
        "TrajectoryFeedbackController",
    ),
    "TaskImpedance": ("minilink.control.robotic", "TaskImpedance"),
    "TaskKinematic": ("minilink.control.robotic", "TaskKinematic"),
    "TaskKinematicNullspace": (
        "minilink.control.robotic",
        "TaskKinematicNullspace",
    ),
}

__all__, __getattr__, __dir__ = lazy_facade(
    globals(), _EXPORTS, modules={"mpc": "minilink.control.mpc"}
)
__all__ = [*__all__, "mpc"]
