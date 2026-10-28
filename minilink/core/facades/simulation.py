"""Simulation shortcuts: compile a system, simulate it, score the run."""

import numpy as np

###############################################################################
# Public API
###############################################################################


class SystemSimulation:
    """
    Simulation shortcuts every :class:`~minilink.core.system.System` has: compile, simulate, score.

    Relies on attributes defined by :class:`~minilink.core.system.System`
    (``n``, ``m``, ``x0``, ``traj``). The latest rollout is cached on
    :attr:`traj` as a convenience only; library code never reads it as an
    input. On a static ``System`` (``n=0``), :meth:`compute_trajectory` and
    :meth:`compute_forced` use
    :class:`~minilink.simulation.static_simulator.StaticSimulator`;
    :class:`DynamicSimulation` and :class:`StepSimulation` override them.
    """

    def compile(self, backend="numpy", verbose=False):
        """
        Convenience shortcut to compile the system into a backend evaluator.

        This delegates to :func:`minilink.core.compile.compile`.
        """
        # A real cycle through minilink.core.system
        from minilink.core.compile.compiler import compile as compile_system

        return compile_system(self, backend=backend, verbose=verbose)

    def compute_trajectory(
        self,
        t_start=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        backend="numpy",
        verbose=True,
    ):
        """
        Convenience shortcut to sample boundary IO on a time grid.

        On static ``System`` leaves (``n=0``), this uses
        :class:`~minilink.simulation.static_simulator.StaticSimulator` — not ODE
        integration. Continuous systems override via
        :class:`DynamicSimulation`.

        Parameters
        ----------
        backend : str
            Passed to the simulator (default ``"numpy"``).
            Use ``backend="auto"`` (see :data:`~minilink.simulation.COMPILE_BACKEND_AUTO`)
            to try JAX then fall back to NumPy.
        verbose : bool
            Print simulator setup (solver, ``dt``, compile backend). Default
            ``True`` for interactive ``compute_trajectory`` calls; library
            helpers pass ``verbose=False``.

        Returns
        -------
        Trajectory
            The sampled trajectory, also stored in :attr:`traj`.
        """
        # A real cycle through minilink.core.system
        from minilink.simulation.static_simulator import StaticSimulator

        sim = StaticSimulator(
            self,
            x0=x0,
            t_start=t_start,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            backend=backend,
            verbose=verbose,
        )
        traj = sim.solve()

        if show:
            # A real cycle through minilink.core.system
            from minilink.graphical.signals import plot_time_signals

            plot_time_signals(self, traj)

        self.traj = traj

        return traj

    def compute_forced(
        self,
        u,
        input_port_id=None,
        t_start=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        backend="numpy",
        verbose=True,
    ):
        """
        Convenience shortcut to sample boundary IO under a prescribed input.

        On static ``System`` leaves, uses
        :meth:`~minilink.simulation.static_simulator.StaticSimulator.solve_forced`.

        Parameters
        ----------
        u : np.ndarray or callable
            Forced input description:

            - If ``input_port_id is None``: either a full input trajectory with
              shape ``(m, n_pts)`` or a callable ``u(t)`` returning the full
              input vector.
            - If ``input_port_id`` is provided: either a trajectory for that
              port only with shape ``(port_dim, n_pts)`` or a callable
              returning that port signal. Other ports stay at their default
              values.
        input_port_id : str, optional
            Named input port to force while keeping the others at default
            values.
        verbose : bool
            Print simulator setup. Default ``True``; pass ``False`` from library
            helpers.

        Returns
        -------
        Trajectory
            Sampled state-input trajectory.
        """
        # A real cycle through minilink.core.system
        from minilink.simulation.static_simulator import StaticSimulator

        sim = StaticSimulator(
            self,
            x0=x0,
            t_start=t_start,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            backend=backend,
            verbose=verbose,
        )
        traj = sim.solve_forced(u, input_port_id=input_port_id)

        if show:
            # A real cycle through minilink.core.system
            from minilink.graphical.signals import plot_time_signals

            plot_time_signals(self, traj)

        self.traj = traj

        return traj

    def compute_cost(self, cost, *, of=None, traj=None):
        """
        Convenience shortcut returning the cost ``J`` of a trajectory, as a number.

        ``J`` is the running cost integrated along the trajectory (the trapezoid
        of :meth:`~minilink.core.costs.CostFunction.integrate`) plus the
        terminal cost at its end.

        Parameters
        ----------
        cost : CostFunction
            Cost written for the system that is scored.
        of : System, optional
            On a diagram, the block to score, as it ran inside the diagram
            (:meth:`~minilink.core.diagram.DiagramSystem.trajectory_of`).
            ``None`` scores this whole system in its own ``(x, u)``.
        traj : Trajectory, optional
            Trajectory of this system; defaults to the last one computed.

        Returns
        -------
        float
            The cost ``J``.
        """
        scored = scored_trajectory(self, of, traj)
        _, running = cost.integrate(scored)

        # J = ∫ g dt + h(x(t_N), t_N)
        J = float(running[-1]) + float(cost.h(scored.x[:, -1], float(scored.t[-1])))

        return J


class DynamicSimulation:
    """
    Continuous-time simulation for :class:`~minilink.core.system.DynamicSystem`, through the :class:`~minilink.simulation.simulator.Simulator`.
    """

    def compute_trajectory(
        self,
        t_start=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        backend="numpy",
        verbose=True,
        solver_warnings="warn",
    ):
        """
        Convenience shortcut to simulate the system and return a trajectory.

        This method is a façade over :class:`~minilink.simulation.simulator.Simulator`.
        It uses model defaults such as :attr:`x0` and stores the resulting
        trajectory in :attr:`traj` for later convenience.

        Parameters
        ----------
        backend : str
            Passed to :class:`~minilink.simulation.simulator.Simulator` (default ``"numpy"``).
            Use ``backend="auto"`` (see :data:`~minilink.simulation.COMPILE_BACKEND_AUTO`)
            to try JAX then fall back to NumPy.
        verbose : bool
            Print solver selection, time grid, and compile backend (default
            ``True`` for interactive use).
        solver_warnings : str
            ``"warn"`` (default), ``"error"``, or ``"ignore"`` for discontinuous-loop
            warnings (see :mod:`minilink.simulation.solver_warnings`).

        Returns
        -------
        Trajectory
            The simulated trajectory, also stored in :attr:`traj`.
        """
        # A real cycle through minilink.core.system
        from minilink.simulation.simulator import Simulator

        sim = Simulator(
            self,
            x0=x0,
            t_start=t_start,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            backend=backend,
            verbose=verbose,
            solver_warnings=solver_warnings,
        )
        traj = sim.solve()

        if show:
            # A real cycle through minilink.core.system
            from minilink.graphical.signals import plot_time_signals

            plot_time_signals(self, traj)

        self.traj = traj

        return traj

    def compute_forced(
        self,
        u,
        input_port_id=None,
        t_start=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        backend="numpy",
        verbose=True,
        solver_warnings="warn",
    ):
        """
        Convenience shortcut to simulate the system under a prescribed input.

        This method is a façade over
        :meth:`~minilink.simulation.simulator.Simulator.solve_forced`.

        Parameters
        ----------
        u : np.ndarray or callable
            Forced input description:

            - If ``input_port_id is None``: either a full input trajectory with
              shape ``(m, n_pts)`` or a callable ``u(t)`` returning the full
              input vector.
            - If ``input_port_id`` is provided: either a trajectory for that
              port only with shape ``(port_dim, n_pts)`` or a callable
              returning that port signal. Other ports stay at their default
              values.
        input_port_id : str, optional
            Named input port to force while keeping the others at default
            values.
        verbose : bool
            Print solver selection and time grid (default ``True``).
        solver_warnings : str
            ``"warn"`` (default), ``"error"``, or ``"ignore"`` for discontinuous-loop
            warnings (see :mod:`minilink.simulation.solver_warnings`).

        Returns
        -------
        Trajectory
            Simulated state-input trajectory.
        """
        # A real cycle through minilink.core.system
        from minilink.simulation.simulator import Simulator

        sim = Simulator(
            self,
            x0=x0,
            t_start=t_start,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            backend=backend,
            verbose=verbose,
            solver_warnings=solver_warnings,
        )
        traj = sim.solve_forced(u, input_port_id=input_port_id)

        if show:
            # A real cycle through minilink.core.system
            from minilink.graphical.signals import plot_time_signals

            plot_time_signals(self, traj)

        self.traj = traj

        return traj


class StepSimulation:
    """
    The run verbs of :class:`~minilink.core.discrete.step_system.StepSystem`: the ticks and, on a clock, the dense run.

    :meth:`compute_rollout` gives the ticks, what ``evaluate`` scores;
    :meth:`compute_trajectory` is the same scan with a continuous plant's
    substeps kept, wherever continuous time is underneath (a system on a clock
    with a continuous source, a sampled loop on a continuous plant). A tick law
    has no continuous time: it refuses it.
    """

    def compute_trajectory(
        self,
        tf=10.0,
        u=None,
        *,
        x0=None,
        t_start=0.0,
        solver=None,
        backend="numpy",
        show=False,
        verbose=False,
    ):
        """
        The run with the continuous plant's substeps kept: a :class:`~minilink.core.trajectory.Trajectory` on ``[t_start, tf]``.

        The same scan as :meth:`compute_rollout`, its value at each tick the tick
        value bit for bit; inside each period the plant shows its substep ends
        and every other block its state at the tick, the input held.
        ``solver="scipy"`` runs the host clock runner instead: the same blocks
        ticked on the host and the plant integrated adaptively between the
        ticks, the cross-check (NumPy).

        Parameters
        ----------
        tf : float
            End of the run in seconds; the run takes ``round((tf - t_start) / Δ)`` ticks.
        u : array, sequence, callable, or None, optional
            The input of each tick, as for :meth:`compute_rollout`.
        x0 : array, optional
            Initial state; defaults to :attr:`x0`.
        t_start : float
            The clock's origin.
        solver : None or "scipy"
            ``None``: the scan with its substeps; ``"scipy"``: the host clock runner.
        """
        # A real cycle through minilink.core.system
        from minilink.core.trajectory import Trajectory

        require_continuous_underneath(self)
        require_run_solver(solver)
        h = self.sample_period
        n_steps = int(round((float(tf) - float(t_start)) / h))
        ev = self.compile(backend="numpy" if solver else backend, verbose=verbose)
        x0 = x0 if x0 is not None else self.x0
        u_samples = ev.rollout_inputs_of(n_steps, u)
        u_steps = u_samples[:, :n_steps].T
        S = dense_points_of(self, ev)

        # The scan with its substeps, or the host runner on the same blocks
        if solver is None:
            xs = ev.rollout_dense(x0, u_steps, n_steps=n_steps, t_start=t_start)
        else:
            # Tools import core (DESIGN §3): the host runner loads on first use
            from minilink.simulation.discrete.clock_runner import run_on_the_clock

            xs = run_on_the_clock(self, u_steps, x0=x0, t_start=t_start, points=S)

        # t = t_start + (k + j/S) Δ, the input held over each period
        t = t_start + h * np.arange(n_steps * S + 1) / S
        u_dense = np.repeat(u_samples[:, :n_steps], S, axis=1)
        u_dense = np.concatenate([u_dense, u_samples[:, -1:]], axis=1)

        traj = Trajectory(t=t, x=np.asarray(xs, dtype=float).T, u=u_dense)
        self.traj = traj

        if show:
            self.plot_trajectory(traj)

        return traj

    def compute_forced(self, *args, **kwargs):
        """Refuse: a step system rolls out with :meth:`compute_rollout`."""
        raise TypeError(
            f"{type(self).__name__} is a StepSystem — use compute_rollout(...), "
            "not compute_forced. Continuous-time simulation belongs on "
            "DynamicSystem; hybrid closed loops use HybridSimulator."
        )

    def compute_rollout(
        self,
        n_steps,
        u=None,
        *,
        x0=None,
        t_start=0.0,
        backend="numpy",
        show=False,
        verbose=False,
    ):
        """
        Convenience shortcut to roll out a discrete-time step system: its ticks, a :class:`~minilink.core.trajectory.Trajectory`.

        The samples sit at ``t = k`` on a tick law, at ``t = t_start + k·Δ``
        seconds on a system on a clock of period ``Δ``.

        Parameters
        ----------
        n_steps : int
            Number of step transitions to apply.
        u : array, sequence, callable, or None, optional
            The input of each step: ``None`` (the nominal input), a constant
            vector, an ``(n_steps, m)`` sequence, or ``u(k)``.
        x0 : array, optional
            Initial state; defaults to :attr:`x0`.
        t_start : float
            The clock's origin, the time of the first tick (a system on a clock).
        backend : str
            Backend passed to :meth:`compile`.
        show : bool
            If ``True``, plot the rollout via :meth:`plot_rollout`.

        Returns
        -------
        Trajectory
            The ticks ``(t, x, u)``, also stored in :attr:`traj`.
        """
        # A real cycle through minilink.core.system
        from minilink.core.trajectory import Trajectory

        ev = self.compile(backend=backend, verbose=verbose)
        x0 = x0 if x0 is not None else self.x0
        u_samples = ev.rollout_inputs_of(n_steps, u)

        # x_{k+1} = step(x_k, u_k, k) for k = 0 … N − 1, at the ticks' times
        xs = ev.rollout(x0, u_samples[:, :n_steps].T, n_steps=n_steps, t_start=t_start)
        k = np.arange(n_steps + 1, dtype=float)
        t = k if self.sample_period is None else t_start + k * self.sample_period

        traj = Trajectory(t=t, x=np.asarray(xs, dtype=float).T, u=u_samples)
        self.traj = traj

        if show:
            self.plot_rollout(traj)

        return traj


###############################################################################
# Internal machinery
###############################################################################


def require_continuous_underneath(system):
    """Refuse the dense run of a system with no continuous time underneath: a tick law counts, it has no seconds."""
    # A real cycle through minilink.core.system
    from minilink.core.discrete.sampled_diagram import SampledDiagramSystem
    from minilink.core.discrete.sampled_system import SampledSystem

    on_a_clock = isinstance(system, (SampledSystem, SampledDiagramSystem))
    if (
        not on_a_clock
        or system.sample_period is None
        or isinstance(system, SampledSystem)
        and system.counts
    ):
        raise TypeError(
            f"{type(system).__name__} {system.name!r} counts its ticks: it has no continuous "
            "time underneath — use compute_rollout(n_steps)"
        )


def require_run_solver(solver):
    """Refuse a solver the dense run does not have: the scan (``None``) or the host runner (``"scipy"``)."""
    if solver not in (None, "scipy"):
        raise ValueError(
            f"solver must be None (the scan with its substeps) or 'scipy' (the host "
            f"clock runner with an adaptive plant), got {solver!r}"
        )


def dense_points_of(system, ev):
    """The points per period the dense run keeps: the plan's, or a lone block's substeps."""
    plan = getattr(ev, "plan", None)
    if plan is not None:
        return plan.dense_points

    return system.substeps if system.resolves else 1


def scored_trajectory(system, of, traj):
    """The trajectory a cost shortcut scores: the system's own, or one block's inside it."""
    # A real cycle through minilink.core.system
    from minilink.core.diagram import DiagramSystem

    traj = system.traj if traj is None else traj
    if traj is None:
        raise ValueError(
            "no trajectory to score: call compute_trajectory() first, or pass traj"
        )
    if of is None:
        return traj
    if not isinstance(system, DiagramSystem):
        raise ValueError(
            f"of= names a block inside a diagram; {system.name!r} is not a diagram"
        )
    return system.trajectory_of(of, traj)
