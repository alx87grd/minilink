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
    input. On a static ``System`` (``n=0``), :meth:`compute_trajectory` uses
    :class:`~minilink.simulation.static_simulator.StaticSimulator`;
    :class:`DynamicSimulation` and :class:`StepSimulation` override it.
    """

    def compile(self, *, backend="numpy", verbose=False):
        """
        Convenience shortcut to compile the system into a backend evaluator.

        This delegates to :func:`minilink.core.compile.compile`.
        """
        # A real cycle through minilink.core.system
        from minilink.core.compile.compiler import compile as compile_system

        return compile_system(self, backend=backend, verbose=verbose)

    def compute_trajectory(
        self,
        tf=10,
        u=None,
        *,
        port=None,
        t0=0,
        n_samples=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        backend="numpy",
        verbose=False,
    ):
        """
        Convenience shortcut to sample boundary IO on a time grid, under the nominal input or ``u``.

        On static ``System`` leaves (``n=0``), this uses
        :class:`~minilink.simulation.static_simulator.StaticSimulator`, not ODE
        integration. Continuous systems override via :class:`DynamicSimulation`.

        Parameters
        ----------
        u : array_like or callable, optional
            The prescribed input: a sampled matrix ``(m, N)`` (a column per
            sample, the layout of ``traj.u``), a function ``u(t)`` of the full
            input, a constant vector, or a scalar on a one-input system; ``None``
            is the nominal input. With ``port`` it describes that one
            port, the others at their nominal values.
        n_samples : int, optional
            Number of time samples on ``[t0, tf]``, endpoints included.
        backend : str
            Passed to the simulator (default ``"numpy"``).
            Use ``backend="auto"`` (see :data:`~minilink.core.backends.BACKEND_AUTO`)
            to try JAX then fall back to NumPy.
        verbose : bool
            Print simulator setup (solver, ``dt``, compile backend). Default
            ``False``: the run is silent unless asked.

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
            n_samples=n_samples,
            dt=dt,
            solver=solver,
            backend=backend,
            verbose=verbose,
        )
        traj = sim.solve(u=u, port=port)

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
        of : System or str, optional
            On a diagram, the block to score, as it ran inside the diagram: the
            block or its id, ``of="plant"``: its recorded run, ``traj.of`` (a
            default run records the plant's input), else its input rebuilt from
            the diagram's states with the blocks' current numbers. ``None``
            scores this whole system in its own ``(x, u)``.
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
        tf=10,
        u=None,
        *,
        port=None,
        t0=0,
        n_samples=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        backend="numpy",
        verbose=False,
        solver_warnings="warn",
        record="plant",
    ):
        """
        Convenience shortcut to simulate the system, under its nominal input or ``u``, and return a trajectory.

        This method is a façade over :class:`~minilink.simulation.simulator.Simulator`.
        It uses model defaults such as :attr:`x0` and stores the resulting
        trajectory in :attr:`traj` for later convenience.

        Parameters
        ----------
        u : array_like or callable, optional
            The prescribed input: a sampled matrix ``(m, N)`` (a column per
            sample, the layout of ``traj.u``, so ``compute_trajectory(tf, u=run.u)``
            replays a run), a function ``u(t)`` of the full input, a constant
            vector, or a scalar on a one-input system; ``None`` is the nominal
            input. A sampled input is held over each sample period and a function
            of time is read where the solver evaluates ``f``, on every solver.
        port : str, optional
            The one input port ``u`` describes, the others kept at their nominal
            values.
        n_samples : int, optional
            Number of time samples on ``[t0, tf]``, endpoints included.
        backend : str
            Passed to :class:`~minilink.simulation.simulator.Simulator` (default ``"numpy"``).
            Use ``backend="auto"`` (see :data:`~minilink.core.backends.BACKEND_AUTO`)
            to try JAX then fall back to NumPy.
        verbose : bool
            Print solver selection, time grid, and compile backend (default
            ``False``).
        solver_warnings : str
            ``"warn"`` (default), ``"error"``, or ``"ignore"`` for discontinuous-loop
            warnings (see :mod:`minilink.simulation.solver_warnings`).
        record : "plant", None, sequence of str or "all"
            A diagram's signals kept in the trajectory's ``signals``, from the
            run itself on the run's numbers: ``"plant"`` (the default) the
            plant's whole input ``"plant:u"`` when the diagram has a ``plant``
            block, so ``traj.of("plant")`` is the plant's complete run; ``None``
            nothing (the fastest run); names, ``"block:port"``, a block's whole
            input ``"block:u"``, a nested block by its path; ``"all"`` every
            output port of every block. A fixed-step run reads them off each
            step's own first stage; an adaptive run computes them at the saved
            samples after the run. A lone block records nothing (its ``x`` and
            ``u`` are the trajectory) and refuses names.

        Returns
        -------
        Trajectory
            The simulated trajectory, also stored in :attr:`traj`.
        """
        # A real cycle through minilink.core.system
        from minilink.core.diagram import subsystem_roles
        from minilink.core.trajectory import Trajectory
        from minilink.simulation.simulator import Simulator

        sim = Simulator(
            self,
            x0=x0,
            t0=t0,
            tf=tf,
            n_samples=n_samples,
            dt=dt,
            solver=solver,
            backend=backend,
            verbose=verbose,
            solver_warnings=solver_warnings,
        )
        traj = sim.solve(u=u, port=port, record=recorded_names_of(self, record))
        if hasattr(self, "state_index"):
            # A diagram's run: every block's state slice, the inputs recorded
            traj = Trajectory(
                t=traj.t,
                x=traj.x,
                u=traj.u,
                signals=traj.signals,
                roles=subsystem_roles(self, traj.signal_names),
            )

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
        t0=0.0,
        dt=None,
        solver=None,
        backend="numpy",
        record="plant",
        show=False,
        verbose=False,
    ):
        """
        The run with the continuous plant's substeps kept: a :class:`~minilink.core.trajectory.Trajectory` on ``[t0, tf]``.

        The same scan as :meth:`compute_rollout`, its value at each tick the tick
        value bit for bit; inside each period the plant shows its substep ends
        and every other block its state at the tick, the input held.
        ``solver="scipy"`` runs the host clock runner instead: the same blocks
        ticked on the host and the plant integrated adaptively between the
        ticks, the cross-check (NumPy). A loop of step systems (no continuous
        plant) has only its ticks: it refuses, naming :meth:`compute_rollout`.

        Parameters
        ----------
        tf : float
            End of the run in seconds; the run takes ``round((tf - t0) / Δ)`` ticks.
        u : array, sequence, callable, or None, optional
            The input of each tick, as for :meth:`compute_rollout`.
        x0 : array, optional
            Initial state; defaults to :attr:`x0`.
        t0 : float
            The clock's origin.
        dt : float, optional
            The plant's integration step, ``⌈Δ / dt⌉`` steps per period (a loop
            with a continuous plant); ``None`` is the automatic step,
            ``AUTO_DT_SCALE`` times the plant's smallest time constant.
        solver : None, "rk4", "euler" or "scipy"
            ``None``: the scan with the plant's own formula (RK4); ``"rk4"`` or
            ``"euler"``: the scan with that formula; ``"scipy"``: the host clock
            runner.
        record : "plant", None, sequence of str or "all"
            Signals kept from the run's own signal pass, in the trajectory's
            ``signals``: ``"plant"`` (the default) the plant's input
            ``"plant:u"`` when the loop has a plant, so ``traj.of("plant")`` and
            the read-backs (plots, animation, overlays) compute nothing again;
            ``None`` nothing; names, ``"path/block:port"`` (a block's output) or
            ``"plant:u"``; ``"all"`` every block output and the plant's input.
            The host clock runner (``solver="scipy"``) records nothing.
        """
        # A real cycle through minilink.core.system
        from minilink.core.compile.plan import PLANT_INPUT
        from minilink.core.trajectory import Trajectory

        require_continuous_underneath(self)
        require_run_solver(solver)
        h = self.sample_period
        n_steps = int(round((float(tf) - float(t0)) / h))
        host = solver == "scipy"
        run = stepped_view(self, dt, None if host else solver)
        ev = run.compile(backend="numpy" if host else backend, verbose=verbose)
        x0 = x0 if x0 is not None else self.x0
        u_samples = ev.rollout_inputs_of(n_steps, u)
        u_steps = u_samples[:, :n_steps].T
        S = dense_points_of(self, ev)

        # What the run records: the named signals, and a loop's plant input
        plant_op = plant_op_of(ev)
        record = recorded_names(record, ev, host)
        signals = {}
        roles = {}

        # The scan with its substeps, or the host runner on the same blocks
        if not host and record:
            xs, signals = ev.rollout_dense(
                x0,
                u_steps,
                ev.call_params(),
                n_steps=n_steps,
                t0=t0,
                record=record,
            )
            signals = {
                name: np.asarray(v, dtype=float).T for name, v in signals.items()
            }
        elif not host:
            xs = ev.rollout_dense(x0, u_steps, ev.call_params(), n_steps=n_steps, t0=t0)
        else:
            # Tools import core (DESIGN §3): the host runner loads on first use
            from minilink.simulation.discrete.clock_runner import run_on_the_clock

            xs = run_on_the_clock(run, u_steps, x0=x0, t0=t0, points=S)

        # t = t0 + (k + j/S) Δ, the input held over each period
        t = t0 + h * np.arange(n_steps * S + 1) / S
        u_dense = np.repeat(u_samples[:, :n_steps], S, axis=1)
        u_dense = np.concatenate([u_dense, u_samples[:, -1:]], axis=1)

        # The plant role: its state slice and its recorded input, and the
        # blocks inside a plant that is itself a diagram
        if plant_op is not None:
            plant_slice = plant_op.local_x_slice
            roles["plant"] = {
                "x": (plant_slice.start, plant_slice.stop),
                "u": PLANT_INPUT if PLANT_INPUT in signals else None,
            }
            roles.update(plant_subsystem_roles(plant_op, plant_slice.start))

        traj = Trajectory(
            t=t,
            x=np.asarray(xs, dtype=float).T,
            u=u_dense,
            signals=signals,
            roles=roles,
        )
        self.traj = traj

        if show:
            self.plot_trajectory(traj)

        return traj

    def compute_rollout(
        self,
        n_steps,
        u=None,
        *,
        x0=None,
        t0=0.0,
        dt=None,
        record="plant",
        backend="numpy",
        show=False,
        verbose=False,
    ):
        """
        Convenience shortcut to roll out a discrete-time step system: its ticks, a :class:`~minilink.core.trajectory.Trajectory`.

        The samples sit at ``t = k`` on a tick law, at ``t = t0 + k·Δ``
        seconds on a system on a clock of period ``Δ``.

        Parameters
        ----------
        n_steps : int
            Number of step transitions to apply: ``n_steps + 1`` samples.
        u : array, sequence, callable, or None, optional
            The input of each step: ``None`` (the nominal input), a constant
            vector, an ``(m, n_steps)`` sequence (a column per step, the layout
            of ``traj.u``), or ``u(k)``.
        x0 : array, optional
            Initial state; defaults to :attr:`x0`.
        t0 : float
            The clock's origin, the time of the first tick (a system on a clock).
        dt : float, optional
            The continuous plant's integration step, as for :meth:`compute_trajectory`.
        record : "plant", None, sequence of str or "all"
            Signals kept at the ticks from the run's own signal pass, as for
            :meth:`compute_trajectory`: the plant's input by default.
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
        from minilink.core.compile.plan import PLANT_INPUT
        from minilink.core.trajectory import Trajectory

        run = stepped_view(self, dt)
        ev = run.compile(backend=backend, verbose=verbose)
        x0 = x0 if x0 is not None else self.x0
        u_samples = ev.rollout_inputs_of(n_steps, u)
        u_steps = u_samples[:, :n_steps].T

        # What the run records: the named signals, and a loop's plant input
        plant_op = plant_op_of(ev)
        if record is not None and record != "plant":
            require_clocked_signals_to_record(self)
        record = recorded_names(record, ev, False)
        signals = {}
        roles = {}

        # x_{k+1} = step(x_k, u_k, k) for k = 0 … N − 1, at the ticks' times; a
        # record keeps the ticks of the dense run, the same states bit for bit
        if record:
            S = ev.plan.dense_points
            xs, dense = ev.rollout_dense(
                x0,
                u_steps,
                ev.call_params(),
                n_steps=n_steps,
                t0=t0,
                record=record,
            )
            xs = np.asarray(xs)[::S]
            signals = {
                name: np.asarray(v, dtype=float)[::S].T for name, v in dense.items()
            }
        else:
            xs = ev.rollout(x0, u_steps, ev.call_params(), n_steps=n_steps, t0=t0)
        k = np.arange(n_steps + 1, dtype=float)
        t = k if self.sample_period is None else t0 + k * self.sample_period

        # The plant role: its state slice and its recorded input, and the
        # blocks inside a plant that is itself a diagram
        if plant_op is not None:
            plant_slice = plant_op.local_x_slice
            roles["plant"] = {
                "x": (plant_slice.start, plant_slice.stop),
                "u": PLANT_INPUT if PLANT_INPUT in signals else None,
            }
            roles.update(plant_subsystem_roles(plant_op, plant_slice.start))

        traj = Trajectory(
            t=t,
            x=np.asarray(xs, dtype=float).T,
            u=u_samples,
            signals=signals,
            roles=roles,
        )
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
    """Refuse a solver the dense run does not have: the scan (``None``, ``"rk4"``, ``"euler"``) or the host runner (``"scipy"``)."""
    if solver not in (None, "rk4", "euler", "scipy"):
        raise ValueError(
            f"solver must be None, 'rk4' or 'euler' (the scan with its substeps) or "
            f"'scipy' (the host clock runner with an adaptive plant), got {solver!r}"
        )


def stepped_view(system, dt=None, solver=None):
    """The system with its continuous plant integrated at the step ``dt`` by ``solver``; itself when both are ``None``."""
    # A real cycle through minilink.core.system
    from minilink.core.discrete.sampled_diagram import (
        SampledDiagramSystem,
        plant_steps_of,
        with_plant_steps,
    )
    from minilink.core.discrete.sampled_system import steps_per_period

    if dt is None and solver is None:
        return system
    if isinstance(system, SampledDiagramSystem):
        return with_plant_steps(system, **plant_steps_of(system, dt, solver))
    if system.resolves:
        substeps = None if dt is None else steps_per_period(system.sample_period, dt)
        return system.with_steps(substeps=substeps, solver=solver)

    raise ValueError(
        f"{system.name} integrates no continuous system inside its period: it has no "
        "integration step to choose (dt=, solver=)"
    )


def require_clocked_signals_to_record(system):
    """Refuse a ticks-run record outside a sampled diagram: the signal pass it keeps is a sampled loop's."""
    # A real cycle through minilink.core.system
    from minilink.core.discrete.sampled_diagram import SampledDiagramSystem

    if not isinstance(system, SampledDiagramSystem):
        raise TypeError(
            f"{system.name} is not a sampled diagram: record= keeps the signals of a "
            "sampled loop's own signal pass ((block % dt) @ plant)"
        )


def recorded_names_of(system, record):
    """The names a run records: ``"plant"`` the plant's input when there is one, ``None`` nothing, ``"all"`` every output port, else the names given."""
    diagram = hasattr(system, "state_index")
    if record is None:
        return ()
    if isinstance(record, str) and record == "plant":
        return ("plant:u",) if diagram and "plant" in system.subsystems else ()
    if isinstance(record, str) and record == "all":
        if not diagram:
            return ()
        return tuple(
            f"{sys_id}:{port_id}"
            for sys_id, block in system.subsystems.items()
            for port_id in block.outputs
        )
    require_record_names(record)
    require_signals_to_record(system)

    return tuple(record)


def require_record_names(record):
    """Refuse a record that is not ``"plant"``, ``"all"``, ``None`` or a sequence of names."""
    if isinstance(record, str):
        raise ValueError(
            f"record={record!r}: pass 'plant', 'all', None, or a sequence of names "
            f"such as ({record!r},)"
        )


def require_signals_to_record(system):
    """Refuse a record on a leaf: its signals are its state and input, already in the trajectory."""
    if not hasattr(system, "reconstruct_internal_signals"):
        raise TypeError(
            f"{system.name} is a leaf: its trajectory already holds its x and u; "
            "record= names the internal signals of a diagram"
        )


def recorded_names(record, ev, host) -> tuple:
    """
    The signals a sampled loop's dense run records.

    ``"plant"`` the plant's input when the loop has a plant; ``None`` nothing;
    ``"all"`` every block output and the plant's input; else the names given.
    The host clock runner records nothing, and refuses names.
    """
    # A real cycle: the plan imports the diagrams, which import these facades
    from minilink.core.compile.plan import PLANT_INPUT

    plant_op = plant_op_of(ev)
    if record is None or (host and record == "plant"):
        return ()
    if isinstance(record, str) and record == "plant":
        return (PLANT_INPUT,) if plant_op is not None else ()
    if isinstance(record, str) and record == "all":
        names = all_output_names(ev) + ((PLANT_INPUT,) if plant_op is not None else ())
    else:
        require_record_names(record)
        names = tuple(record)
    if host:
        require_no_record(names)

    return names


def all_output_names(ev) -> tuple:
    """Every block output of a compiled sampled loop, ``"path/block:port"``."""
    plan = getattr(ev, "plan", None)
    if plan is None or not hasattr(plan, "paths"):
        return ()
    paths = plan.paths

    return tuple(
        f"{'/'.join(paths[sys_id])}:{port_id}" for sys_id, port_id in plan.output_slices
    )


def require_no_record(names):
    """Refuse a record on the host clock runner: it ticks the blocks on the host, outside the scan."""
    if names:
        raise ValueError(
            "record= keeps signals from the scan's own signal pass; the host clock "
            "runner (solver='scipy') has none: run the scan (solver=None)"
        )


def plant_op_of(ev):
    """A loop's plant operation in the compiled plan (``None`` for a lone block or no plan)."""
    plan = getattr(ev, "plan", None)

    return getattr(plan, "plant_op", None)


def dense_points_of(system, ev):
    """The points per period the dense run keeps: the plan's, or a lone block's substeps."""
    plan = getattr(ev, "plan", None)
    if plan is not None:
        return plan.dense_points

    return system.substeps if system.resolves else 1


def scored_trajectory(system, of, traj):
    """The trajectory a cost shortcut scores: the system's own, or one block's inside it (its recorded run, else reconstructed)."""
    # A real cycle through minilink.core.system
    from minilink.core.diagram import DiagramSystem, subsystem_run

    traj = system.traj if traj is None else traj
    if traj is None:
        raise ValueError(
            "no trajectory to score: call compute_trajectory() first, or pass traj"
        )
    if of is None:
        return traj
    if not hasattr(system, "subsystem_id"):
        raise ValueError(
            f"of= names a block inside a diagram; {system.name!r} is not a diagram"
        )

    # The block's run as recorded (its input among the signals), else rebuilt
    # from the diagram's states on a continuous diagram
    path = of if isinstance(of, str) else system.subsystem_id(of)
    recorded = traj.roles.get(path)
    if recorded is not None and recorded["u"] is not None:
        return traj.of(path)
    if not isinstance(system, DiagramSystem):
        raise ValueError(
            f"this run records no input for {path!r}: record=({path + ':u'!r},)"
        )

    return subsystem_run(system, path, traj)


def plant_subsystem_roles(plant_op, offset):
    """The blocks inside a loop's plant when it is itself a diagram, by path ``"plant/<id>"``, their slices offset into the loop's state."""
    # A real cycle through minilink.core.system
    from minilink.core.diagram import subsystem_roles

    plant = getattr(plant_op.system, "source", plant_op.system)
    if not hasattr(plant, "state_index"):
        return {}

    return subsystem_roles(plant, (), offset, "plant/")
