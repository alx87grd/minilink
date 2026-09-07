"""Controller and static law blocks (reference/sensor to actuation).

Teaching imports::

    from minilink.control import ImpedanceController
    from minilink.control.lqr import lqr
    from minilink.control.mpc import ModelPredictiveController

Note: ``lqr`` lives in the ``control.lqr`` module — it is not re-exported on
the package attribute ``control.lqr`` (that name is the submodule itself).
"""

from __future__ import annotations

from minilink.core.facade import lazy_facade

_EXPORTS: dict[str, tuple[str, str]] = {
    "ComputedTorqueController": (
        "minilink.control.modelbased",
        "ComputedTorqueController",
    ),
    "PID": ("minilink.control.siso", "PID"),
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
