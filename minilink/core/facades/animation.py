"""Animation shortcuts: a system drawn, played back, or played live."""


###############################################################################
# Public API
###############################################################################


class SystemAnimation:
    """
    Animation shortcuts every :class:`~minilink.core.system.System` has: one frame, or a trajectory played back.
    """

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
        # Lazy import: the graphics load on first use, not with the core
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
        playback (useful for debugging; see ``DESIGN.md`` §4.7). ``camera``
        accepts an optional override (a constant 4x4 or a
        ``camera(frames, x, u, t)`` callable). ``save=True`` writes the backend's file: a Pillow GIF
        for ``renderer="matplotlib"`` (``{file_name}.gif`` when the name has
        no ``.gif`` suffix), or a self-contained HTML page of the native
        Meshcat animation for ``renderer="meshcat"`` (``{file_name}.html``
        when the name has no suffix).
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation import Animator
        from minilink.graphical.common.environment import prefers_inline_animation

        # The trajectory passed, else the last one computed, else a fresh one
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


class DynamicAnimation:
    """
    Animation shortcuts for :class:`~minilink.core.system.DynamicSystem`: its modes, and the system played live.
    """

    def animate_modal(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        mode="all",
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
        """Linearize, excite the selected mode(s), and animate them on this system.

        See :func:`minilink.analysis.modal.animate_modal`.
        """
        # A real cycle through minilink.core.system
        from minilink.analysis.modal import animate_modal

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

    def game(
        self,
        *,
        frame_dt=1 / 30.0,
        sim_dt=None,
        renderer="pygame",
        is_3d=False,
        input=None,
        output=None,
        joystick_axes=None,
        camera_axes=((3, -1.0), (4, -1.0)),
        plot=False,
        plot_backend="matplotlib",
        sync="locked",
        compile_backend=None,
        max_steps=None,
        tf=None,
        x0=None,
        t0=0.0,
        verbose=False,
    ):
        """
        Convenience shortcut for a live keyboard/joystick real-time session.

        Façade over
        :class:`~minilink.simulation.realtime.simulator.RealtimeSimulator`
        with a :class:`~minilink.simulation.realtime.pygame_input.PygameInput`
        input source: held keys or a connected joystick command the input-port
        bounds, the plant advances in sync with the wall clock (``frame_dt``
        per rendered frame, integrated internally at ``sim_dt``,
        auto-calibrated when omitted), and the session returns a
        :class:`~minilink.core.trajectory.Trajectory` when the user quits
        (ESC, ``Ctrl+C``, or window close), also stored in :attr:`traj` for
        later ``plot_trajectory`` / ``animate``.

        Pass ``joystick_axes=[(axis, sign), ...]`` to remap or invert gamepad
        sticks, and ``camera_axes=[(az_axis, sign), (el_axis, sign)]`` to
        control 3-D camera orbit (or pass a custom
        :class:`~minilink.simulation.realtime.RealtimeInput` via ``input=``;
        a :class:`~minilink.simulation.realtime.RealtimeOutput` via
        ``output=`` receives every frame). Pass ``plot=True``, or the signals
        to show (``plot=("slip", "grip")``), to open a live rolling-window
        time-signal plot alongside the scene, on ``plot_backend``.

        ``compile_backend=None`` (default) tries JAX when available and
        compatible, otherwise NumPy — preferred for live sessions where speed
        matters.
        """
        # A real cycle through minilink.core.system
        from minilink.simulation.realtime import PygameInput, RealtimeSimulator

        if input is None:
            input = PygameInput(
                joystick_axes=joystick_axes,
                camera_axes=camera_axes,
            )

        rt_sim = RealtimeSimulator(
            self,
            frame_dt=frame_dt,
            sim_dt=sim_dt,
            sync=sync,
            renderer=renderer,
            is_3d=is_3d,
            plot=plot,
            plot_backend=plot_backend,
            input=input,
            output=output,
            compile_backend=compile_backend,
            max_steps=max_steps,
            tf=tf,
            verbose=verbose,
        )
        traj = rt_sim.run(x0=x0, t0=t0)

        self.traj = traj

        return traj
