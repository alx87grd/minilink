"""
Base class for deterministic policy-synthesis planners.

Policy-synthesis planners compute feedback policies or value functions
from optimal-control problems, rather than returning a single nominal
trajectory as the primary artifact.
"""

from __future__ import annotations

from minilink.planning.base import Planner
from minilink.planning.policy_synthesis.results import PolicySynthesisResult


class PolicySynthesisPlanner(Planner[PolicySynthesisResult]):
    """
    Base class for planners that compute policies or value functions.

    Dynamic programming is the first expected consumer. Policy planners
    typically require a cost, but concrete subclasses decide when to
    validate that requirement.
    """

    def require_policy_result(self) -> PolicySynthesisResult:
        """Return the latest result and ensure it contains a policy."""
        result = self.require_result()
        if result.policy is None:
            raise ValueError("The latest result does not contain a policy")
        return result
