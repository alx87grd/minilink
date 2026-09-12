"""
Reinforcement learning as a policy-family planner (pure JAX).

The pieces every algorithm shares — the rollout environment built from a
stochastic planning problem, the policy block and its exploration head, the
critics, the collectors and the optimizer — live in this package; each
algorithm is one file under ``algorithms/`` holding only its update rule.
:class:`ReinforcementLearningPlanner` runs the training loop and returns the
learned law as a controller block.
"""

from minilink.planning.reinforcement_learning.algorithms import PPO, SAC, Algorithm
from minilink.planning.reinforcement_learning.environment import RolloutEnvironment
from minilink.planning.reinforcement_learning.planner import (
    ReinforcementLearningPlanner,
)

__all__ = [
    "Algorithm",
    "PPO",
    "SAC",
    "ReinforcementLearningPlanner",
    "RolloutEnvironment",
]
