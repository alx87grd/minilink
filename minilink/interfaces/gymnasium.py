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

from minilink.core.compile.evaluators.stepping import require_step_knobs
from minilink.core.sets import BoxInputSet
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
        Time step of one env step (zero-order hold on the action).
    tf : float
        Maximum duration of an episode (truncation).
    t_start : float
        Initial time (only relevant if the system is time dependent).
    x_start : Distribution, optional
        The law of the initial state on :meth:`reset`, drawn with the
        environment's ``np_random`` (``Uniform(lower, upper)``,
        ``Gaussian(mean, std)``, any :class:`~minilink.core.distributions.Distribution`);
        ``None`` starts every episode at ``sys.x0``.
    render_mode : None or "human"
        ``"human"`` renders the system with the matplotlib animator.
    solver : {"rk4", "euler"}
        Integration scheme of the step model on the compiled plant.
    substeps : int
        Integration steps per env step, the action held over them.
    backend : {None, "numpy", "jax"}
        Evaluator used for the step. ``None`` picks JAX when it is installed
        and the plant traces (one jitted step per env step), NumPy otherwise;
        :attr:`backend` records the choice.

    Attributes
    ----------
    backend : str
        Backend actually used for the step (``"numpy"`` or ``"jax"``).
    evaluator : DynamicEvaluator or DiagramEvaluator
        Compiled plant the step runs on.
    clipping_inputs : bool
        Clip the actions to the input-port bounds (default ``True``).
    clipping_states : bool
        Clip the states to the state bounds after integration (default
        ``False``; some classic gym envs do this, but it is a very artificial
        behaviour — use with caution).
    x_start : Distribution or None
        The start's law, as given (it may be replaced between episodes).
    """

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        sys,
        cost,
        dt=0.05,
        tf=10.0,
        t_start=0.0,
        render_mode=None,
        x_start=None,
        solver="rk4",
        substeps=1,
        backend=None,
    ):
        self.sys = sys
        self.cost = cost
        self.dt = dt
        self.t_start = t_start
        self.tf = tf  # for truncation of episodes

        # The step model of one env step, compiled once for the whole run
        require_step_knobs(solver, substeps)
        self.solver = solver
        self.substeps = int(substeps)
        self.backend, self.evaluator = _compiled_step(sys, backend)

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

        # The start's law (None: sys.x0 every episode)
        self.x_start = x_start

        # Memory
        self.x = np.asarray(sys.x0, dtype=float).copy()
        self.u = sys.get_u_from_input_ports()
        self.t = t_start

        if self.render_mode == "human":
            self._init_render()

    @classmethod
    def from_problem(cls, problem, dt=0.05, episode_length=None, **kwargs):
        """
        The Gymnasium view of a :class:`~minilink.planning.problems.PlanningProblem`.

        ``reset`` draws the initial state from the problem, the reward is the
        problem's running cost, a finite horizon ends the episode at ``tf``
        with the terminal cost ``h``, leaving the constraint set ``X`` (read on
        the problem's set parameters) is a failure (*terminated*, charged the
        price of infeasibility) and leaving the plant's state box, the training
        zone, is a *truncation*.
        Parameter laws and disturbance sources are not applied by this view.
        A problem with a point start starts every episode at its ``x_start``.
        """
        finite = problem.horizon_kind() == "finite"
        tf = float(problem.tf) if finite else float(episode_length or 10.0)

        env = ProblemEnv(problem.sys, problem.require_cost(), dt=dt, tf=tf, **kwargs)
        env.problem = problem
        env.finite_horizon = finite
        env.infeasible_cost = problem.infeasibility_price(tf, dt=dt)
        return env

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # x(0) ~ the start's law, drawn with the environment's generator, else sys.x0
        if self.x_start is None:
            self.x = np.asarray(self.sys.x0, dtype=float).copy()
        else:
            self.x = np.asarray(self.x_start.sample(self.np_random), dtype=float)

        self.u = self.sys.get_u_from_input_ports()
        self.t = self.t_start

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

        # One step of the plant's step model, the action held
        x_new = np.asarray(
            self.evaluator.step(
                x, u, t, dt=dt, solver=self.solver, substeps=self.substeps
            ),
            dtype=float,
        )
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

        # Truncation of episodes at the horizon tf (within half a step) or out of
        # the state bounds
        truncated = bool(
            (t_new >= self.tf - 0.5 * dt)
            or np.any(x_new < self.x_lb)
            or np.any(x_new > self.x_ub)
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
    controllers. Its input port is named ``x``, so ``plot_control_law()``
    draws the same textbook slice as LQR / VI::

        ctl = SB3Controller(model, sys=plant)
        ctl.plot_control_law(x_axis=2, y_axis=5)

    The model itself is duck-typed: stable-baselines3 is not imported.
    Pass ``sys`` to copy state/input labels, units, and bounds onto the
    ports (sweep domain and axis names for ``plot_control_law``).
    """

    def __init__(self, model, sys=None, name=None):
        super().__init__()
        if name is not None:
            self.name = name
        self.model = model

        n = int(model.observation_space.shape[0])
        m = int(model.action_space.shape[0])
        x_lb, x_ub = _box_bounds(model.observation_space)
        u_lb, u_ub = _box_bounds(model.action_space)
        x_labels = x_units = u_labels = u_units = None
        if sys is not None:
            x_labels = list(sys.state.labels)
            x_units = list(sys.state.units)
            x_lb, x_ub = sys.state.lower_bound, sys.state.upper_bound
            if "u" in sys.inputs:
                u_labels = list(sys.inputs["u"].labels)
                u_units = list(sys.inputs["u"].units)
                u_lb = sys.inputs["u"].lower_bound
                u_ub = sys.inputs["u"].upper_bound

        self.add_input_port(
            "x",
            dim=n,
            labels=x_labels,
            units=x_units,
            lower_bound=x_lb,
            upper_bound=x_ub,
        )
        self.add_output_port(
            "u",
            dim=m,
            function=self.ctl,
            dependencies=("x",),
            labels=u_labels,
            units=u_units,
            lower_bound=u_lb,
            upper_bound=u_ub,
        )

    def action(self, x) -> np.ndarray:
        """Return the deterministic policy action ``u`` at a single state ``x``."""
        u, _ = self.model.predict(np.asarray(x, dtype=np.float32), deterministic=True)
        return np.asarray(u, dtype=float).reshape(-1)

    def ctl(self, x, u, t=0, params=None):
        """State feedback; the ``x`` input port carries the plant state in ``u``."""
        return self.action(u)


# Internal machinery


class ProblemEnv(Sys2Gym):
    """:class:`Sys2Gym` whose starts and episode ends follow a planning problem."""

    problem = None
    finite_horizon = False
    infeasible_cost = None

    def reset(self, seed=None, options=None):
        gym.Env.reset(self, seed=seed)  # seeds np_random; the problem draws the start
        self.x = np.asarray(self.problem.sample_x0(self.np_random), dtype=float)
        self.u = self.sys.get_u_from_input_ports()
        self.t = self.t_start
        y = np.asarray(
            self.sys.h(self.x, self.u, self.t), dtype=self.observation_space.dtype
        )
        return y, {"state": self.x, "action": self.u}

    def step(self, u):
        problem = self.problem
        y, r, terminated, truncated, info = super().step(u)
        x, t = self.x, self.t

        # A failure is charged its price; the end of a finite horizon its h, a
        # failure on the last step both, as the evaluation scores the run
        failed = not problem.X.contains(x, t, None)
        ended = self.finite_horizon and t >= self.tf - 0.5 * self.dt
        if failed:
            r -= float(self.infeasible_cost)
        if ended:
            r -= float(problem.require_cost().h(x, t))
        if failed or ended:
            terminated, truncated = True, False

        return y, r, terminated, truncated, info


def _compiled_step(sys, backend):
    """Return ``(backend, evaluator)``; the env steps through ``evaluator.step``.

    ``backend=None`` tries JAX when it is installed and falls back to
    NumPy for plants that do not trace (catalog plants written with NumPy
    branches). Under JAX the whole step is one jitted call.
    """
    if backend is None:
        from minilink.core.compile.compiler import compile_auto

        backend, evaluator = compile_auto(sys)
    else:
        evaluator = sys.compile(backend=backend, verbose=False)

    return backend, evaluator


def _box_bounds(space):
    """Return ``(low, high)`` from a gym ``Box``, or ``(None, None)``."""
    low = getattr(space, "low", None)
    high = getattr(space, "high", None)
    if low is None or high is None:
        return None, None
    return np.asarray(low, dtype=float), np.asarray(high, dtype=float)


def _state_bounds(sys):
    """Return the state bounds ``(lb, ub)`` as float32 arrays."""
    box = sys.state.box
    return box.lower.astype(np.float32), box.upper.astype(np.float32)


def _input_bounds(sys):
    """Return stacked input-port bounds ``(lb, ub)`` as float32 arrays."""
    box = BoxInputSet.from_system_inputs(sys).box
    return box.lower.astype(np.float32), box.upper.astype(np.float32)
