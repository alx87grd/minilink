"""Simulation shortcuts: compile a system, simulate it, score the run."""


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
        t0=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        compile_backend="numpy",
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
        compile_backend : str
            Passed to the simulator (default ``"numpy"``).
            Use ``compile_backend="auto"`` (see :data:`~minilink.simulation.COMPILE_BACKEND_AUTO`)
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
            t0=t0,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            compile_backend=compile_backend,
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
        t0=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        compile_backend="numpy",
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
            t0=t0,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            compile_backend=compile_backend,
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
        t0=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        compile_backend="numpy",
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
        compile_backend : str
            Passed to :class:`~minilink.simulation.simulator.Simulator` (default ``"numpy"``).
            Use ``compile_backend="auto"`` (see :data:`~minilink.simulation.COMPILE_BACKEND_AUTO`)
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
            t0=t0,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            compile_backend=compile_backend,
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
        t0=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        compile_backend="numpy",
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
            t0=t0,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            compile_backend=compile_backend,
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
    Discrete-time rollouts for :class:`~minilink.core.discrete.step_system.StepSystem`.

    A step system has no continuous-time simulation: :meth:`compute_trajectory`
    and :meth:`compute_forced` raise and point at :meth:`compute_rollout`
    (compiled ``step`` / ``rollout``).
    """

    def compute_trajectory(self, *args, **kwargs):
        """Refuse: a step system rolls out with :meth:`compute_rollout`."""
        raise TypeError(
            f"{type(self).__name__} is a StepSystem — use compute_rollout(...), "
            "not compute_trajectory. Continuous-time simulation belongs on "
            "DynamicSystem; hybrid closed loops use HybridSimulator."
        )

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
        compile_backend="numpy",
        show=False,
        verbose=False,
    ):
        """
        Convenience shortcut to roll out a discrete-time step system.

        Returns a state-only :class:`~minilink.core.discrete.step_rollout.StepRollout`.
        Boundary output logging belongs in :class:`~minilink.simulation.discrete.computer.Computer`
        or :class:`~minilink.simulation.discrete.hybrid_simulator.HybridSimulator` — not here.

        Parameters
        ----------
        n_steps : int
            Number of step transitions to apply.
        u : array, sequence, callable, or None, optional
            Input schedule passed to the compiled evaluator rollout.
        x0 : array, optional
            Initial state; defaults to :attr:`x0`.
        compile_backend : str
            Backend passed to :meth:`compile`.
        show : bool
            If ``True``, plot the rollout via :meth:`plot_rollout`.

        Returns
        -------
        StepRollout
            The rollout, also stored in :attr:`rollout`.
        """
        ev = self.compile(backend=compile_backend, verbose=verbose)
        x0 = x0 if x0 is not None else self.x0
        rollout = ev.rollout(x0, n_steps=n_steps, u=u)

        self.rollout = rollout

        if show:
            self.plot_rollout(rollout)

        return rollout


###############################################################################
# Internal machinery
###############################################################################


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
