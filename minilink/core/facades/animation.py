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
        *,
        is_3d=False,
        camera=None,
        overlays=None,
        renderer="matplotlib",
        show=True,
    ):
        """
        Draw one frame of the system at ``(x, u, t)``: an :class:`AnimationResult` of one frame.

        ``camera`` accepts an optional override: a constant 4x4 or a
        ``camera(frames, x, u, t)`` callable.
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation import Animator

        return Animator(self).render_frame(
            x,
            u,
            t,
            is_3d=is_3d,
            renderer=renderer,
            show=show,
            camera=camera,
            overlays=overlays,
        )

    def animate(
        self,
        traj=None,
        *,
        speed=1.0,
        is_3d=False,
        scene_title: str | None = None,
        camera=None,
        overlays=None,
        renderer="matplotlib",
        show=True,
    ):
        """
        Animate a trajectory of this system: an :class:`AnimationResult`.

        ``speed`` is the playback rate, in simulated seconds per wall-clock
        second: ``2.0`` plays twice as fast as real time. ``show=True`` plays it
        where the session puts it: inline in a notebook, in a window or a
        browser viewer elsewhere; ``"inline"`` or ``"window"`` names the place,
        ``False`` builds it only. The record saves it: ``.save("swing.gif")``
        on matplotlib, ``.save("lap.html")`` on Meshcat and Plotly. ``camera``
        accepts an optional override (a constant 4x4 or a
        ``camera(frames, x, u, t)`` callable).
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation import Animator

        # The trajectory passed, else the last one computed, else a fresh one
        if traj is None:
            if self.traj is not None:
                traj = self.traj
            else:
                traj = self.compute_trajectory(verbose=False)

        return Animator(self).animate_simulation(
            traj,
            speed=speed,
            is_3d=is_3d,
            renderer=renderer,
            show=show,
            scene_title=scene_title,
            camera=camera,
            overlays=overlays,
        )


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
        n_samples=2001,
        speed=3.0,
        is_3d=False,
        renderer="matplotlib",
        show=True,
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
            n_samples=n_samples,
            speed=speed,
            renderer=renderer,
            is_3d=is_3d,
            show=show,
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
        plot_renderer="matplotlib",
        sync="locked",
        backend=None,
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
        time-signal plot alongside the scene, on ``plot_renderer``.

        ``backend=None`` (default) tries JAX when available and
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
            plot_renderer=plot_renderer,
            input=input,
            output=output,
            backend=backend,
            max_steps=max_steps,
            tf=tf,
            verbose=verbose,
        )
        traj = rt_sim.run(x0=x0, t0=t0)

        self.traj = traj

        return traj
