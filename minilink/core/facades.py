"""
System convenience facades.

Evolution-aware mixin shortcuts for :class:`~minilink.core.system.System`
subclasses: :class:`SharedSystemFacades` (all kinds), :class:`DynamicSystemFacades`
(continuous evolution), and :class:`StepSystemFacades` (discrete rollout).

The mixins are shortcuts only: mathematical, structural, and visualization
contracts stay in :mod:`minilink.core.system`. Heavy dependencies
(simulation, graphics) are imported lazily inside each method.
"""


class SharedSystemFacades:
    """
    Mixin providing shortcuts shared by all :class:`~minilink.core.system.System` kinds.

    Relies on attributes defined by :class:`~minilink.core.system.System`
    (``n``, ``m``, ``x0``, ``traj``). The latest facade rollout is cached on
    :attr:`traj` as a convenience only; library code never reads it as an
    input.

    On static ``System`` leaves (``n=0``), :meth:`compute_trajectory` and
    :meth:`compute_forced` use :class:`~minilink.simulation.static_simulator.StaticSimulator`.
    :class:`DynamicSystem` subclasses override those methods via
    :class:`DynamicSystemFacades`. :class:`StepSystem` overrides them to raise
    and point at :meth:`~minilink.core.facades.StepSystemFacades.compute_rollout`.
    """

    # User Shortcut / Facade API

    def compile(self, backend="numpy", verbose=False):
        """
        Convenience shortcut to compile the system into a backend evaluator.

        This delegates to :func:`minilink.core.compile.compile`.
        """
        from minilink.core.compile.compiler import compile as compile_system

        return compile_system(self, backend=backend, verbose=verbose)

    def jacobian(
        self,
        of,
        wrt,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        method="auto",
        eps=1e-6,
    ):
        """Return ``d(of)/d(wrt)`` at an operating point, as a NumPy array.

        ``plant.jacobian("f", "x")`` is ∂f/∂x at the nominal point;
        ``plant.jacobian("f", "u", x_bar, u_bar)`` at ``(x̄, ū)``. ``of`` is
        ``"f"``, an output port id, or a diagram wire ``"block:port"``; ``wrt``
        is ``"x"``, ``"u"``, an input port id, ``"t"``, ``"params"`` (dict
        result), or a wire. See :func:`minilink.analysis.derivatives.jacobian`.
        """
        from minilink.analysis.derivatives import jacobian

        return jacobian(self, of, wrt, x_bar, u_bar, t, params, method=method, eps=eps)

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
        :class:`DynamicSystemFacades`.

        Parameters
        ----------
        compile_backend : str
            Passed to the simulator (default ``\"numpy\"``).
            Use ``compile_backend=\"auto\"`` (see :data:`~minilink.simulation.COMPILE_BACKEND_AUTO`)
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
            Forced input description.
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
            from minilink.graphical.signals import plot_time_signals

            plot_time_signals(self, traj)

        self.traj = traj

        return traj

    def plot_trajectory(
        self,
        traj=None,
        *,
        signals=None,
        backend="matplotlib",
        show=True,
    ):
        """
        Convenience shortcut to plot sampled time signals.

        If the trajectory is not computed yet, it is computed using :meth:`compute_trajectory`.
        If the trajectory is already computed, it is used directly.
        If the trajectory is provided, it is used directly.

        Parameters
        ----------
        signals : tuple of str, optional
            Signal names to plot; see
            :func:`minilink.graphical.signals.plot_time_signals`.
            For diagrams, each entry may be a boundary name (``"x"``),
            an internal port (``"plant:p"``), or a ``(subsystem, port)``
            pair resolved via :meth:`~minilink.core.diagram.DiagramSystem.subsystem_signal`.
            When ``None``, defaults are chosen via
            :func:`minilink.graphical.signals.resolve_plot_signals`.

        Returns
        -------
        PlotResult
            The plot result from
            :func:`minilink.graphical.signals.plot_time_signals`.
        """
        from minilink.graphical.signals import plot_time_signals, resolve_plot_signals

        if signals is None:
            signals = resolve_plot_signals(self)

        if traj is None:
            traj = self.traj
        if traj is None:
            traj = self.compute_trajectory(show=False, verbose=False)

        return plot_time_signals(
            self,
            traj,
            signals=signals,
            backend=backend,
            show=show,
        )

    def plot_input_output_map(self, **kwargs):
        """
        Convenience shortcut plotting one output component over swept inputs.

        Sweeps components of one input port (line, heatmap, or
        ``show_3d=True`` surface) with every other input pinned at its
        nominal value. See
        :func:`minilink.graphical.port_map.plot_input_output_map`.
        """
        from minilink.graphical.port_map import plot_input_output_map

        return plot_input_output_map(self, **kwargs)

    def get_diagram(self):
        """
        Convenience shortcut returning a renderable diagram representation.
        """
        from minilink.graphical.diagrams import get_diagram

        return get_diagram(self)

    def _repr_svg_(self):
        """
        Convenience notebook representation for the system diagram.
        """
        g = self.get_diagram()
        if g is None:
            return None
        try:
            return g._repr_image_svg_xml()
        except Exception:
            return None

    def plot_diagram(self, filename=None, show=True, show_inline=None, show_pdf=None):
        """
        Convenience shortcut to render the system diagram.

        Jupyter / Colab get inline SVG by default. Bare scripts and IPython
        REPLs open a Matplotlib window with the Graphviz PNG (same blocking
        policy as trajectory plots). Pass ``show_pdf=True`` for the legacy
        OS PDF viewer; pass ``filename`` to write Graphviz output to disk;
        pass ``show=False`` to build the Digraph only.
        """
        from minilink.graphical.diagrams import plot_diagram

        return plot_diagram(
            self,
            show=show,
            show_inline=show_inline,
            show_pdf=show_pdf,
            filename=filename,
        )

    def render(
        self,
        x,
        u,
        t,
        is_3d=False,
        renderer="matplotlib",
        camera=None,
        overlays=None,
    ):
        """
        Convenience shortcut rendering a single frame of the system.

        ``camera`` accepts an optional override: a constant 4x4 or a
        ``camera(frames, x, u, t)`` callable.
        """
        from minilink.graphical.animation import Animator

        animator = Animator(self)
        return animator.show(
            x,
            u,
            t,
            is_3d=is_3d,
            renderer=renderer,
            camera=camera,
            overlays=overlays,
        )

    def animate(
        self,
        traj=None,
        time_factor_video=1.0,
        is_3d=False,
        html: bool | None = None,
        renderer="matplotlib",
        native: bool = True,
        scene_title: str | None = None,
        show: bool = True,
        save: bool = False,
        file_name: str = "Animation",
        camera=None,
        overlays=None,
    ):
        """
        Convenience shortcut to animate a trajectory of this system.

        ``html=None`` auto-resolves via
        :func:`minilink.graphical.common.environment.prefers_inline_animation`:
        ``True`` in Colab and in local Jupyter with a non-interactive
        matplotlib backend (``inline`` / ``agg``); ``False`` for bare
        script, IPython REPL, and Jupyter with an interactive backend
        (``qt`` / ``widget`` / ``macosx`` / ``tk`` / ``nbagg``).
        ``native=True`` (default) drives each backend's own animation
        engine (matplotlib ``FuncAnimation`` / meshcat ``Animation``).
        Pass ``native=False`` to fall back to the per-frame Python-loop
        playback (useful for debugging or when the native path's limitations
        matter — e.g. meshcat freezes per-frame dynamic geometry such as an
        ``Arrow`` length/direction or ``TorqueArrow`` sweep; see ``DESIGN.md``
        §4.7). ``camera`` accepts an
        optional override (a constant 4x4 or a ``camera(frames, x, u, t)``
        callable). ``save=True`` with ``renderer="matplotlib"`` writes a GIF
        with the Pillow writer (``{file_name}.gif``).
        """
        from minilink.graphical.animation import Animator
        from minilink.graphical.common.environment import prefers_inline_animation

        if traj is None:
            if self.traj is not None:
                traj = self.traj
            else:
                traj = self.compute_trajectory(verbose=False)

        resolved_html = prefers_inline_animation() if html is None else html

        animator = Animator(self)
        show_plot = show and not resolved_html
        ani_obj = animator.animate_simulation(
            traj,
            time_factor_video=time_factor_video,
            is_3d=is_3d,
            html=resolved_html,
            show=show_plot,
            save=save,
            file_name=file_name,
            renderer=renderer,
            native=native,
            scene_title=scene_title,
            camera=camera,
            overlays=overlays,
        )

        # For html output, return the IPython.display.HTML object and let the
        # notebook auto-display it via the standard last-expression rule.
        # Calling display.display() *and* returning the object renders twice.
        return ani_obj


class DynamicSystemFacades:
    """
    Continuous-time simulation and analysis shortcuts for :class:`~minilink.core.system.DynamicSystem`.

    Overrides :meth:`compute_trajectory` and :meth:`compute_forced` to use
    :class:`~minilink.simulation.simulator.Simulator`. Inherited by
    :class:`~minilink.core.diagram.DiagramSystem`.
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
            Passed to :class:`~minilink.simulation.simulator.Simulator` (default ``\"numpy\"``).
            Use ``compile_backend=\"auto\"`` (see :data:`~minilink.simulation.COMPILE_BACKEND_AUTO`)
            to try JAX then fall back to NumPy.
        verbose : bool
            Print solver selection, time grid, and compile backend (default
            ``True`` for interactive use).
        solver_warnings : str
            ``\"warn\"`` (default), ``\"error\"``, or ``\"ignore\"`` for discontinuous-loop
            warnings (see :mod:`minilink.simulation.solver_warnings`).

        Returns
        -------
        Trajectory
            The simulated trajectory, also stored in :attr:`traj`.
        """
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
            Forced input description.
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
            ``\"warn\"`` (default), ``\"error\"``, or ``\"ignore\"`` for discontinuous-loop
            warnings (see :mod:`minilink.simulation.solver_warnings`).

        Returns
        -------
        Trajectory
            Simulated state-input trajectory.
        """
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
            from minilink.graphical.signals import plot_time_signals

            plot_time_signals(self, traj)

        self.traj = traj

        return traj

    def plot_phase_plane(
        self,
        traj=None,
        *,
        x_axis=0,
        y_axis=None,
        backend="matplotlib",
        show=True,
        **kwargs,
    ):
        """
        Convenience shortcut to plot a phase-plane vector field.

        If ``traj`` is provided, or if :attr:`traj` contains a previous
        simulation result, the sampled state path is overlaid on the vector
        field. Otherwise only the vector field is plotted.
        """
        from minilink.graphical.phase_plane import plot_phase_plane

        if traj is None:
            traj = self.traj
        return plot_phase_plane(
            self,
            traj,
            x_axis=x_axis,
            y_axis=y_axis,
            backend=backend,
            show=show,
            **kwargs,
        )

    def linearize(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        method="auto",
        eps=1e-6,
    ):
        """Linearize about ``(x_bar, u_bar)`` and return an ``LTISystem``.

        ``lin = plant.linearize(x_bar)`` gives ``lin.A()``, ``lin.B()``,
        ``lin.C()``, ``lin.D()``. See :func:`minilink.analysis.linearize.linearize`.
        """
        from minilink.analysis.linearize import linearize

        return linearize(
            self, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
        )

    def transfer_function(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        method="auto",
        eps=1e-6,
    ):
        """Return one SISO channel of the linearization as a ``TransferFunction``.

        See :func:`minilink.analysis.frequency.transfer_function`.
        """
        from minilink.analysis.frequency import transfer_function

        return transfer_function(
            self, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
        )

    def bode(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        w=None,
        n=200,
        method="auto",
        eps=1e-6,
    ):
        """Return ``(w, magnitude_db, phase_deg)`` of one SISO channel.

        See :func:`minilink.analysis.frequency.bode`.
        """
        from minilink.analysis.frequency import bode

        return bode(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            w=w,
            n=n,
            method=method,
            eps=eps,
        )

    def pzmap(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        method="auto",
        eps=1e-6,
    ):
        """Return ``(zeros, poles, gain)`` of one SISO channel.

        See :func:`minilink.analysis.frequency.pzmap`.
        """
        from minilink.analysis.frequency import pzmap

        return pzmap(
            self, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
        )

    def plot_bode(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        w=None,
        n=200,
        margins=True,
        method="auto",
        eps=1e-6,
        backend="matplotlib",
        show=True,
        title=None,
    ):
        """Bode diagram of one SISO channel, gain and phase margins marked.

        See :func:`minilink.analysis.frequency.plot_bode`.
        """
        from minilink.analysis.frequency import plot_bode

        return plot_bode(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            w=w,
            n=n,
            margins=margins,
            method=method,
            eps=eps,
            backend=backend,
            show=show,
            title=title,
        )

    def margins(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        w=None,
        n=2000,
        method="auto",
        eps=1e-6,
    ):
        """Gain and phase margins of one SISO channel taken as a loop gain.

        See :func:`minilink.analysis.frequency.margins`.
        """
        from minilink.analysis.frequency import margins

        return margins(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            w=w,
            n=n,
            method=method,
            eps=eps,
        )

    def nyquist(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        w=None,
        n=500,
        method="auto",
        eps=1e-6,
    ):
        """Return ``(w, G)`` of one SISO channel for the Nyquist contour.

        See :func:`minilink.analysis.frequency.nyquist`.
        """
        from minilink.analysis.frequency import nyquist

        return nyquist(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            w=w,
            n=n,
            method=method,
            eps=eps,
        )

    def root_locus(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        gains=None,
        method="auto",
        eps=1e-6,
    ):
        """Return ``(gains, roots)`` of one SISO channel closed with ``u = -K y``.

        See :func:`minilink.analysis.frequency.root_locus`.
        """
        from minilink.analysis.frequency import root_locus

        return root_locus(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            gains=gains,
            method=method,
            eps=eps,
        )

    def step_response(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        tf=None,
        n=500,
        method="auto",
        eps=1e-6,
    ):
        """Return ``(time, y)``, the unit-step response of one SISO channel.

        See :func:`minilink.analysis.time_response.step_response`.
        """
        from minilink.analysis.time_response import step_response

        return step_response(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            tf=tf,
            n=n,
            method=method,
            eps=eps,
        )

    def region_of_attraction(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        method="quadratic",
        Q=None,
        window=None,
        samples=None,
        search="auto",
    ):
        """Certify a region of attraction about an equilibrium of this loop.

        See :func:`minilink.analysis.lyapunov.region_of_attraction`.
        """
        from minilink.analysis.lyapunov import region_of_attraction

        return region_of_attraction(
            self,
            x_bar,
            u_bar,
            t,
            params,
            method=method,
            Q=Q,
            window=window,
            samples=samples,
            search=search,
        )

    def plot_region_of_attraction(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        x_axis=0,
        y_axis=1,
        basin=False,
        detail=True,
        verified=0,
        trajectories=(),
        n=201,
        limits=None,
        show=True,
        method="quadratic",
        Q=None,
        window=None,
        samples=None,
        search="auto",
    ):
        """Certify a region of attraction and draw a slice of it.

        See :func:`minilink.analysis.lyapunov.plot_region_of_attraction`.
        """
        certificate = self.region_of_attraction(
            x_bar,
            u_bar,
            t,
            params,
            method=method,
            Q=Q,
            window=window,
            samples=samples,
            search=search,
        )
        return certificate.plot(
            x_axis,
            y_axis,
            basin=basin,
            detail=detail,
            verified=verified,
            trajectories=trajectories,
            n=n,
            limits=limits,
            show=show,
        )

    def plot_root_locus(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        gains=None,
        method="auto",
        eps=1e-6,
        backend="matplotlib",
        show=True,
    ):
        """Root locus of one SISO channel closed with ``u = -K y``.

        See :func:`minilink.analysis.frequency.plot_root_locus`.
        """
        from minilink.analysis.frequency import plot_root_locus

        return plot_root_locus(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            gains=gains,
            method=method,
            eps=eps,
            backend=backend,
            show=show,
        )

    def plot_nyquist(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        w=None,
        n=500,
        method="auto",
        eps=1e-6,
        backend="matplotlib",
        show=True,
    ):
        """Nyquist diagram of one SISO channel with the critical point.

        See :func:`minilink.analysis.frequency.plot_nyquist`.
        """
        from minilink.analysis.frequency import plot_nyquist

        return plot_nyquist(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            w=w,
            n=n,
            method=method,
            eps=eps,
            backend=backend,
            show=show,
        )

    def plot_step_response(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        tf=None,
        n=500,
        method="auto",
        eps=1e-6,
        backend="matplotlib",
        show=True,
    ):
        """Step response of one SISO channel with rise time, settling time and overshoot.

        See :func:`minilink.analysis.time_response.plot_step_response`.
        """
        from minilink.analysis.time_response import plot_step_response

        return plot_step_response(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            tf=tf,
            n=n,
            method=method,
            eps=eps,
            backend=backend,
            show=show,
        )

    def plot_pzmap(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        method="auto",
        eps=1e-6,
        backend="matplotlib",
        show=True,
    ):
        """Plot poles and zeros of one SISO channel.

        See :func:`minilink.analysis.frequency.plot_pzmap`.
        """
        from minilink.analysis.frequency import plot_pzmap

        return plot_pzmap(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            method=method,
            eps=eps,
            backend=backend,
            show=show,
        )

    def modal_analysis(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        mode=None,
        method="auto",
        eps=1e-6,
        amplitude=1.0,
        tf=None,
        n_steps=2001,
        time_factor_video=3.0,
        renderer="matplotlib",
        is_3d=False,
        show=True,
        html=None,
        native=True,
    ):
        """Linearize and eigendecompose ``A``; returns ``(poles, modes)``.

        With ``mode=None`` analyze only. With ``mode=0`` or ``mode="all"``,
        also animate the mode shapes through
        :func:`~minilink.analysis.modal.animate_modal`.
        """
        from minilink.analysis.modal import animate_modal, modal_analysis

        if mode is not None:
            return animate_modal(
                self,
                x_bar,
                u_bar,
                t,
                params,
                mode=mode,
                method=method,
                eps=eps,
                amplitude=amplitude,
                tf=tf,
                n_steps=n_steps,
                time_factor_video=time_factor_video,
                renderer=renderer,
                is_3d=is_3d,
                show=show,
                html=html,
                native=native,
            )
        return modal_analysis(self, x_bar, u_bar, t, params, method=method, eps=eps)

    def find_equilibrium(self, x_guess, u_bar=None, t=0.0, params=None, *, tol=1e-9):
        """Return a state near ``x_guess`` where ``f`` vanishes.

        See :func:`minilink.analysis.equilibria.find_equilibrium`.
        """
        from minilink.analysis.equilibria import find_equilibrium

        return find_equilibrium(self, x_guess, u_bar, t, params, tol=tol)

    def game(
        self,
        *,
        frame_dt=1 / 30.0,
        sim_dt=None,
        renderer="pygame",
        is_3d=False,
        sync="locked",
        compile_backend=None,
        max_steps=None,
        tf=None,
        x0=None,
        t0=0.0,
    ):
        """
        Convenience shortcut for a live keyboard-driven real-time session.

        Façade over
        :class:`~minilink.simulation.realtime.simulator.RealtimeSimulator`
        with a :class:`~minilink.simulation.realtime.pygame_input.PygameInput`
        keyboard source: held keys command the input-port bounds, the plant
        advances in sync with the wall clock (``frame_dt`` per rendered frame,
        integrated internally at ``sim_dt``, auto-calibrated when omitted),
        and the session returns a
        :class:`~minilink.core.trajectory.Trajectory` when the user quits
        (ESC or window close), also stored in :attr:`traj` for later
        ``plot_trajectory`` / ``animate``.

        ``compile_backend=None`` (default) tries JAX when available and
        compatible, otherwise NumPy — preferred for live sessions where speed
        matters.
        """
        from minilink.simulation.realtime import PygameInput, RealtimeSimulator

        rt_sim = RealtimeSimulator(
            self,
            frame_dt=frame_dt,
            sim_dt=sim_dt,
            sync=sync,
            renderer=renderer,
            is_3d=is_3d,
            input=PygameInput(),
            compile_backend=compile_backend,
            max_steps=max_steps,
            tf=tf,
        )
        traj = rt_sim.run(x0=x0, t0=t0)
        self.traj = traj
        return traj


class StepSystemFacades:
    """Discrete-time rollout shortcuts for :class:`~minilink.core.system.StepSystem`.

    Step plants do **not** use :class:`~minilink.simulation.simulator.Simulator`
    or :class:`~minilink.simulation.static_simulator.StaticSimulator` — those
    belong to continuous / static leaves. Discrete evolution goes through
    :meth:`compute_rollout` (compiled ``step`` / ``rollout``). Continuous-time
    facades inherited from :class:`SharedSystemFacades`
    (:meth:`compute_trajectory`, :meth:`compute_forced`) raise here with a
    pointer to the rollout API.
    """

    def jacobian(
        self,
        of,
        wrt,
        x_bar=None,
        u_bar=None,
        k=0,
        params=None,
        *,
        method="auto",
        eps=1e-6,
    ):
        """Return ``d(of)/d(wrt)`` at ``(x_bar, u_bar, k)``; ``of="step"`` is the update map.

        See :func:`minilink.analysis.derivatives.jacobian`.
        """
        from minilink.analysis.derivatives import jacobian

        return jacobian(self, of, wrt, x_bar, u_bar, k, params, method=method, eps=eps)

    def compute_trajectory(self, *args, **kwargs):
        raise TypeError(
            f"{type(self).__name__} is a StepSystem — use compute_rollout(...), "
            "not compute_trajectory. Continuous-time simulation belongs on "
            "DynamicSystem; hybrid closed loops use HybridSimulator."
        )

    def compute_forced(self, *args, **kwargs):
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

        Returns a state-only :class:`~minilink.core.step_rollout.StepRollout`.
        Boundary output logging belongs in :class:`~minilink.simulation.computer.Computer`
        or :class:`~minilink.simulation.hybrid_simulator.HybridSimulator` — not here.

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
        rollout = ev.rollout(x0 if x0 is not None else self.x0, n_steps=n_steps, u=u)
        self.rollout = rollout
        if show:
            self.plot_rollout(rollout)
        return rollout

    def plot_rollout(
        self,
        rollout=None,
        *,
        signals=None,
        backend="matplotlib",
        show=True,
    ):
        """
        Convenience shortcut to plot sampled step signals.

        If the rollout is not computed yet, it must be provided or available on
        :attr:`rollout`.
        """
        from minilink.graphical.signals import (
            STEP_ABSCISSA_LABEL,
            plot_time_signals,
            resolve_plot_signals,
        )

        if signals is None:
            signals = resolve_plot_signals(self)

        if rollout is None:
            rollout = self.rollout
        if rollout is None:
            raise ValueError(
                "No rollout available; pass rollout=... or call compute_rollout first."
            )

        return plot_time_signals(
            self,
            rollout.as_trajectory(),
            signals=signals,
            abscissa_label=STEP_ABSCISSA_LABEL,
            backend=backend,
            show=show,
        )
