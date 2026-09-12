"""The algorithm contract: an update rule and its train state, nothing else."""

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

    An algorithm receives the callables the planner built from the policy
    block, the exploration head and the critics, initializes its train state
    (weights plus optimizer moments, target networks, temperatures) and
    updates it from a batch of experience. Everything else is shared, so a new
    method is one file subclassing this class. ``on_policy`` declares which
    training loop the planner runs: collect a rollout then update, or step,
    store in the replay buffer, and update from samples.
    """

    on_policy = True

    #: Discount per control period; ``None`` until the planner resolves it in :meth:`bind`.
    gamma = None

    #: The policy and critic callables, set by :meth:`bind`.
    functions = None

    def bind(self, functions: PolicyFunctions, gamma: float):
        """Receive the policy and critic callables and the one discount (called once by the planner)."""
        self.functions = functions
        self.gamma = float(gamma)
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
