"""
Gymnasium bridge: expose a minilink dynamic system as an RL environment.

:class:`Sys2Gym` wraps a :class:`~minilink.core.system.DynamicSystem` (or a
:class:`~minilink.core.diagram.DiagramSystem`) plus a
:class:`~minilink.core.costs.CostFunction` as a ``gymnasium.Env``: the episode
reward is the negative running cost integrated over one time step
(``reward = -g(x, u, t) * dt``), so an RL agent maximizing reward solves the
same problem as a planner minimizing the cost.

Training happens outside the library (e.g. with stable-baselines3); the learned
policy comes back into a minilink diagram through :class:`SB3Controller`, a
state-feedback block whose output calls ``model.predict``.

Port of pyro's ``pyro/tools/sys2gym.py`` to the minilink system contract.
"""

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from minilink.core.system import System

# Public API


class Sys2Gym(gym.Env):
    """
    Create a Gymnasium environment from a minilink system and a cost function.

    Taken from the minilink system:

    - ``x0``: nominal initial state
    - ``f``: state evolution function ``dx = f(x, u, t)``
    - ``h``: observation function ``y = h(x, u, t)`` (state feedback for
      catalog plants, whose default output is the full state)
    - state / input port bounds: observation and action ``Box`` spaces

    Taken from the cost function:

    - ``g(x, u, t)``: running cost (``reward = -g * dt``)
    - ``h(x, t)``: terminal cost (negative terminal reward when the optional
      termination condition is reached)

    Additional parameters of the gym wrapper:

    Parameters
    ----------
    sys : DynamicSystem
        Plant (or closed-loop diagram) to expose.
    cost : CostFunction
        Running / terminal cost pair defining the reward.
    dt : float
        Time step for the forward-Euler integration of one env step.
    tf : float
        Maximum duration of an episode (truncation).
    t0 : float
        Initial time (only relevant if the system is time dependent).
    reset_mode : {"uniform", "gaussian", "determinist"}
        Distribution of the initial state on :meth:`reset`.
    render_mode : None or "human"
        ``"human"`` renders the system with the matplotlib animator.

    Attributes
    ----------
    clipping_inputs : bool
        Clip the actions to the input-port bounds (default ``True``).
    clipping_states : bool
        Clip the states to the state bounds after integration (default
        ``False``; some classic gym envs do this, but it is a very artificial
        behaviour — use with caution).
    x0_lb, x0_ub : arrays
        Initial-state bounds for ``reset_mode="uniform"``.
    x0_std : array
        Initial-state standard deviation for ``reset_mode="gaussian"``.
    """

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        sys,
        cost,
        dt=0.05,
        tf=10.0,
        t0=0.0,
        render_mode=None,
        reset_mode="uniform",
    ):
        self.sys = sys
        self.cost = cost
        self.dt = dt
        self.t0 = t0
        self.tf = tf  # for truncation of episodes

        # Boundaries
        self.x_lb, self.x_ub = _state_bounds(sys)
        self.u_lb, self.u_ub = _input_bounds(sys)
        self.observation_space = spaces.Box(self.x_lb, self.x_ub)  # state feedback
        self.action_space = spaces.Box(self.u_lb, self.u_ub)

        # Optional clipping behaviour
        self.clipping_inputs = True
        self.clipping_states = False

        # Rendering
        self.render_mode = render_mode
        self._animator = None
        self._render_backend = None

        # Reset parameters (stochasticity of the initial state)
        self.reset_mode = reset_mode
        self.x0_lb = sys.x0 + 0.1 * self.x_lb
        self.x0_ub = sys.x0 + 0.1 * self.x_ub
        self.x0_std = 0.1 * (self.x_ub - self.x_lb)

        # Memory
        self.x = np.asarray(sys.x0, dtype=float).copy()
        self.u = sys.get_u_from_input_ports()
        self.t = t0

        if self.render_mode == "human":
            self._init_render()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        if self.reset_mode == "uniform":
            self.x = self.np_random.uniform(self.x0_lb, self.x0_ub)
        elif self.reset_mode == "gaussian":
            self.x = self.np_random.normal(self.sys.x0, self.x0_std)
        else:  # deterministic
            self.x = np.asarray(self.sys.x0, dtype=float).copy()

        self.u = self.sys.get_u_from_input_ports()
        self.t = self.t0

        y = np.asarray(
            self.sys.h(self.x, self.u, self.t), dtype=self.observation_space.dtype
        )
        info = {"state": self.x, "action": self.u}
        return y, info

    def step(self, u):
        u = np.asarray(u, dtype=float).reshape(-1)
        if self.clipping_inputs:
            u = np.clip(u, self.u_lb, self.u_ub)

        # State and time at the beginning of the step
        x, t, dt = self.x, self.t, self.dt

        # Forward-Euler integration over one step
        dx = self.sys.f(x, u, t)
        x_new = x + dx * dt
        t_new = t + dt

        if self.clipping_states:
            x_new = np.clip(x_new, self.x_lb, self.x_ub)

        # Termination of episodes (task-defined; default is a continuous task)
        terminated = bool(self.termination_is_reached())

        # Reward = negative of the cost function
        if terminated:
            r = -float(self.cost.h(x, t))  # terminal cost
        else:
            r = -float(self.cost.g(x, u, t)) * dt  # running cost over the step

        # Truncation of episodes if out of space-time bounds
        truncated = bool(
            (t_new > self.tf) or np.any(x_new < self.x_lb) or np.any(x_new > self.x_ub)
        )

        # Memory update
        self.x = x_new
        self.t = t_new
        self.u = u  # control-input memory is only used for rendering

        y = np.asarray(
            self.sys.h(self.x, self.u, self.t), dtype=self.observation_space.dtype
        )
        info = {"state": self.x, "action": self.u}

        if self.render_mode == "human":
            self.render()

        return y, r, terminated, truncated, info

    def render(self):
        if self.render_mode == "human":
            if self._animator is None:
                self._init_render()
            else:
                self._animator.update_live_frame(
                    self._render_backend, self.x, self.u, self.t
                )

    def termination_is_reached(self):
        """
        Return ``True`` when the task is completed.

        Override in a subclass to define a termination condition for a task
        that is not purely time-defined (the default represents a continuous
        task that only truncates on the time horizon or domain exit).
        """
        return False

    # Internal machinery

    def _init_render(self):
        from minilink.graphical.animation.animator import Animator, make_renderer

        self._animator = Animator(self.sys)
        self._render_backend = make_renderer("matplotlib", self._animator)
        self._animator.open_live_scene(self._render_backend, self.x, self.u, self.t)


class SB3Controller(System):
    """
    State-feedback controller wrapping a trained RL policy.

    Wraps any model exposing ``predict(obs, deterministic=True)`` (the
    stable-baselines3 API) as a static feedback block ``u = pi(x)``, so a
    learned policy composes with a plant exactly like the other minilink
    controllers::

        diagram = DiagramSystem()
        diagram.add_subsystem(ctl, "ctl")
        diagram.add_subsystem(plant, "plant")
        diagram.connect("plant", "y", "ctl", "x")
        diagram.connect("ctl", "u", "plant", "u")

    The model itself is duck-typed: stable-baselines3 is not imported.
    """

    def __init__(self, model, name="RL Policy Controller"):
        super().__init__()
        self.name = name
        self.model = model

        n = int(model.observation_space.shape[0])
        m = int(model.action_space.shape[0])

        self.add_input_port("x", dim=n)
        self.add_output_port("u", dim=m, function=self.ctl, dependencies=("x",))

    def action(self, x) -> np.ndarray:
        """Return the deterministic policy action ``u`` at a single state ``x``."""
        u, _ = self.model.predict(np.asarray(x, dtype=np.float32), deterministic=True)
        return np.asarray(u, dtype=float).reshape(-1)

    def ctl(self, x, u, t=0, params=None):
        """State feedback; the ``x`` input port carries the plant state in ``u``."""
        return self.action(u)


def to_gymnasium(sys, cost, **kwargs) -> Sys2Gym:
    """Convenience factory returning a :class:`Sys2Gym` environment."""
    return Sys2Gym(sys, cost, **kwargs)


# Internal machinery


def _state_bounds(sys):
    """Return the state bounds ``(lb, ub)`` as float32 arrays."""
    lb, ub = sys.state.lower_bound, sys.state.upper_bound
    if lb is None or ub is None:
        raise ValueError(
            "Sys2Gym requires state bounds; set sys.state.lower_bound and "
            "sys.state.upper_bound."
        )
    return (
        np.asarray(lb, dtype=np.float32),
        np.asarray(ub, dtype=np.float32),
    )


def _input_bounds(sys):
    """Return stacked input-port bounds ``(lb, ub)`` as float32 arrays."""
    lb = np.full(sys.m, -np.inf, dtype=np.float32)
    ub = np.full(sys.m, +np.inf, dtype=np.float32)
    i = 0
    for port in sys.inputs.values():
        if port.lower_bound is not None:
            lb[i : i + port.dim] = port.lower_bound
        if port.upper_bound is not None:
            ub[i : i + port.dim] = port.upper_bound
        i += port.dim
    return lb, ub
