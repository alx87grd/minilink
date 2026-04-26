"""
Result objects for policy-synthesis planners.

Policy-synthesis planners return feedback policies or value functions,
rather than a single nominal trajectory.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class PolicySynthesisResult:
    """
    Result returned by policy-synthesis planners.

    Parameters
    ----------
    policy : object, optional
        Feedback policy, lookup table, or callable control law.
    value : object, optional
        Value function or cost-to-go artifact.
    controller : object, optional
        Minilink controller adapter when available.
    cost : float, optional
        Start-state cost-to-go or aggregate objective value.
    """

    problem: Any | None = None
    policy: Any | None = None
    value: Any | None = None
    controller: Any | None = None
    cost: float | None = None
    success: bool = False
    message: str = ""
    stats: dict[str, Any] = field(default_factory=dict)

    @property
    def has_policy(self) -> bool:
        """Return ``True`` when a policy is available."""
        return self.policy is not None

    @property
    def has_controller(self) -> bool:
        """Return ``True`` when a controller adapter is available."""
        return self.controller is not None

    def require_policy(self) -> Any:
        """Return the policy or raise a clear error."""
        if self.policy is None:
            raise ValueError("This policy-synthesis result does not contain a policy")
        return self.policy

    def as_controller(self) -> Any:
        """Return the controller/control law or raise a clear error."""
        if self.controller is None:
            raise ValueError(
                "This policy-synthesis result does not contain a controller"
            )
        return self.controller
