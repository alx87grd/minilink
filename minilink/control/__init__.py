"""Controller and static law blocks (reference/sensor to actuation).

Teaching imports::

    from minilink.control import ImpedanceController
    from minilink.control.lqr import lqr
    from minilink.control.place import place
    from minilink.control.mpc import ModelPredictiveController

Note: ``lqr`` lives in the ``control.lqr`` module and ``place`` in ``control.place``
— neither is re-exported on the package attribute of the same name (that name
is the submodule itself).
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
