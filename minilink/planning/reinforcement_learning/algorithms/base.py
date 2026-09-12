"""
The algorithm contract: an update rule and its train state, nothing else.

A learning algorithm receives the *functions* the planner built from the
policy block, the exploration head and the critics, initializes its train
state (weights plus optimizer moments, target networks, temperatures) and
updates it from a batch of experience. Environment, collectors, heads,
critics, the optimizer and the result type are shared; adding a method means
adding one file that subclasses :class:`Algorithm`.
"""

from abc import ABC, abstractmethod

# Public API


class PolicyFunctions:
    """
    Callables an algorithm consumes, built once by the planner.

    ``mean(actor_params, x)`` normalized mean action; ``observe(x)`` the
    policy's features; ``m`` the action dimension; ``head`` the exploration
    distribution; ``value(critic_params, x)`` state value (on-policy) or
    ``q(q_params, x, a)`` action value (off-policy).
    """

    def __init__(self, *, mean, head, observe, m, value=None, q=None):
        self.mean = mean
        self.head = head
        self.observe = observe
        self.m = int(m)
        self.value = value
        self.q = q


class Algorithm(ABC):
    """
    Update rule of a reinforcement learning method.

    Class attributes declare which training loop the planner runs:
    ``on_policy`` (collect a rollout, update, repeat) or off-policy (step,
    store in the replay buffer, update from samples).
    """

    on_policy = True

    def bind(self, functions: PolicyFunctions):
        """Receive the policy / critic callables (called once by the planner)."""
        self.functions = functions
        return self

    @abstractmethod
    def init(self, key, params):
        """Train state from the initial ``params`` pytree ``{"actor", "head", "critic", ...}``."""
        ...

    @abstractmethod
    def update(self, train_state, batch, key):
        """One learning update; return ``(train_state, stats)`` with scalar stats."""
        ...

    @staticmethod
    def params(train_state):
        """Current weights pytree from a train state."""
        return train_state["params"]
