"""Learning: policies learned from interaction, reinforcement learning on a planning problem."""

from __future__ import annotations

from minilink.core.exports import lazy_facade

# The band facade for short teaching imports, e.g.
#
#     from minilink.learning import ReinforcementLearningPlanner, TabularLearningPlanner
#
# The defining modules stay importable (``minilink.learning.reinforcement_learning``).
# Each exported name maps to its (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    # Reinforcement learning, neural and tabular
    "ReinforcementLearningPlanner": (
        "minilink.learning.reinforcement_learning.planner",
        "ReinforcementLearningPlanner",
    ),
    "TabularLearningPlanner": (
        "minilink.learning.reinforcement_learning.tabular",
        "TabularLearningPlanner",
    ),
    "EpsilonGreedy": (
        "minilink.learning.reinforcement_learning.tabular",
        "EpsilonGreedy",
    ),
    "UCB": ("minilink.learning.reinforcement_learning.tabular", "UCB"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
