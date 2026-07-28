"""Controller and static law blocks (reference/sensor to actuation).

Teaching imports::

    from minilink.control import ImpedanceController
    from minilink.control.lqr import lqr
    from minilink.control.mpc import ModelPredictiveController

Note: ``lqr`` lives in the ``control.lqr`` module — it is not re-exported on
the package attribute ``control.lqr`` (that name is the submodule itself).
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

_EXPORTS: dict[str, tuple[str, str]] = {
    "ComputedTorqueController": (
        "minilink.control.modelbased",
        "ComputedTorqueController",
    ),
    "FilteredController": ("minilink.control.siso", "FilteredController"),
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

__all__ = [*sorted(_EXPORTS), "mpc"]


def __getattr__(name: str) -> Any:
    if name == "mpc":
        mpc_pkg = import_module("minilink.control.mpc")
        globals()["mpc"] = mpc_pkg
        return mpc_pkg
    try:
        module_path, attr = _EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from exc
    value = getattr(import_module(module_path), attr)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
