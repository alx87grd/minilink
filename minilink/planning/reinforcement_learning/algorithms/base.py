"""The algorithm contract: an update rule and its train state, nothing else."""

from abc import ABC, abstractmethod

# Public API


class Algorithm(ABC):
    """
    Update rule of a reinforcement learning method, and its train state.

    The planner builds the stochastic policy and the critic the method's
    family needs and binds them with the one discount; ``init`` creates the
    train state (weights, optimizer moments, target networks, temperatures)
    and ``update`` moves it from a batch of experience. Everything else is
    shared, so a new method is one file subclassing this class.

    Three class attributes state the family. ``on_policy`` chooses the
    planner's loop: collect a batch then update, or step, store and update
    from replayed minibatches. ``head_kind`` names the exploration
    distribution, ``"gaussian"`` or ``"squashed"``. ``critic_kind`` names
    what the critic estimates, ``"V"``, ``"Q"`` or ``None``. An ``episodic``
    method learns from complete episodes: every plant restarts before a
    collection, which then lasts one episode.
    """

    on_policy = True
    head_kind = "gaussian"
    critic_kind = "V"
    episodic = False

    #: Discount per control period; ``None`` until the planner resolves it in :meth:`bind`.
    gamma = None

    #: The stochastic policy and its critic, bound by the planner.
    policy = None
    critic = None

    def bind(self, policy, critic, gamma):
        """Receive the policy, its critic (``None`` for a critic-free method) and the one discount."""
        self.policy = policy
        self.critic = critic
        self.gamma = float(gamma)
        return self

    @abstractmethod
    def init(self, key):
        """Train state from the policy's and the critic's initial weights."""
        ...

    @abstractmethod
    def update(self, train_state, batch, key):
        """One learning update; return ``(train_state, stats)`` with scalar stats."""
        ...

    @staticmethod
    def params(train_state):
        """Current weights pytree ``{"policy", "critic"}`` from a train state."""
        return train_state["params"]
