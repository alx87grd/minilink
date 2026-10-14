"""Controller and static law blocks (reference/sensor to actuation).

Teaching imports::

    from minilink.control import ImpedanceController, lqr, place
    from minilink.control.mpc import ModelPredictiveController

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
    "ProportionalController": (
        "minilink.control.output",
        "ProportionalController",
    ),
    "SlidingModeController": (
        "minilink.control.modelbased",
        "SlidingModeController",
    ),
    "StateFeedbackController": (
        "minilink.control.state",
        "StateFeedbackController",
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
