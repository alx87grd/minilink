"""
System convenience facades.

This module defines :class:`SystemFacades`, the mixin that gives every
:class:`~minilink.core.system.System` its user-shortcut methods
(:meth:`~SystemFacades.compile`, :meth:`~SystemFacades.compute_trajectory`,
:meth:`~SystemFacades.plot_trajectory`, :meth:`~SystemFacades.animate`,
:meth:`~SystemFacades.modal_analysis`, ...).

The mixin is shortcuts only: the mathematical, structural, and visualization
contracts stay in :mod:`minilink.core.system`. Heavy dependencies
(simulation, graphics) are imported lazily inside each method.
"""

import numpy as np


class SystemFacades:
    """
    Mixin providing user-shortcut facades.

    Relies on attributes defined by :class:`~minilink.core.system.System`
    (``n``, ``m``, ``x0``, ``traj``). The latest facade rollout is cached on
    :attr:`traj` as a convenience only; library code never reads it as an
    input.
    """

    # User Shortcut / Facade API

    def compile(self, backend="numpy", verbose=False):
        """
        Convenience shortcut to compile the system into a backend evaluator.

        This delegates to :func:`minilink.core.compile.compile`.
        """
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
        verbose=False,
    ):
        """
        Convenience shortcut to simulate the system and return a trajectory.

        This method is a façade over :class:`minilink.simulation.Simulator`.
        It uses model defaults such as :attr:`x0` and stores the resulting
        trajectory in :attr:`traj` for later convenience.

        Parameters
        ----------
        compile_backend : str
            Passed to :class:`~minilink.simulation.Simulator` (default ``\"numpy\"``).
            Use ``compile_backend=\"auto\"`` (see :data:`~minilink.simulation.COMPILE_BACKEND_AUTO`)
            to try JAX then fall back to NumPy.

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
        verbose=False,
    ):
        """
        Convenience shortcut to simulate the system under a prescribed input.

        This method is a façade over :class:`minilink.simulation.Simulator`
        and :meth:`minilink.simulation.Simulator.solve_forced`.

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

        if traj is not None:
            return plot_time_signals(
                self, traj, signals=signals, backend=backend, show=show
            )

        if self.traj is not None:
            return plot_time_signals(
                self, self.traj, signals=signals, backend=backend, show=show
            )

        traj = self.compute_trajectory(show=False)
        return plot_time_signals(
            self,
            traj,
            signals=signals,
            backend=backend,
            show=show,
        )

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

    def plot_bode(
        self,
        x_bar=None,
        u_bar=None,
        *,
        input_port=None,
        input_index=0,
        output_port=None,
        output_index=0,
        w=None,
        n=200,
        method="fd",
        t=0.0,
        params=None,
        epsilon=1e-6,
        backend="matplotlib",
        show=True,
    ):
        """
        Convenience shortcut to plot a selected SISO Bode response.

        ``input_port`` selects a boundary input port and ``input_index`` selects
        one component inside it. ``output_port`` selects a boundary output port,
        or an internal diagram output ``(sys_id, port_id)``; ``output_index``
        selects one component inside that output.
        """
        from minilink.analysis.frequency import plot_bode

        if x_bar is None:
            x_bar = self.x0
        return plot_bode(
            self,
            x_bar,
            u_bar,
            input_port=input_port,
            input_index=input_index,
            output_port=output_port,
            output_index=output_index,
            w=w,
            n=n,
            method=method,
            t=t,
            params=params,
            epsilon=epsilon,
            backend=backend,
            show=show,
        )

    def plot_pzmap(
        self,
        x_bar=None,
        u_bar=None,
        *,
        input_port=None,
        input_index=0,
        output_port=None,
        output_index=0,
        method="fd",
        t=0.0,
        params=None,
        epsilon=1e-6,
        backend="matplotlib",
        show=True,
    ):
        """
        Convenience shortcut to plot poles and zeros for a selected SISO channel.
        """
        from minilink.analysis.frequency import plot_pzmap

        if x_bar is None:
            x_bar = self.x0
        return plot_pzmap(
            self,
            x_bar,
            u_bar,
            input_port=input_port,
            input_index=input_index,
            output_port=output_port,
            output_index=output_index,
            method=method,
            t=t,
            params=params,
            epsilon=epsilon,
            backend=backend,
            show=show,
        )

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
        return g._repr_image_svg_xml()

    def plot_diagram(self, filename=None, show_inline=None, show_pdf=None):
        """
        Convenience shortcut to render the system diagram.

        ``show_inline`` and ``show_pdf`` default to ``None`` and auto-resolve
        via :func:`minilink.graphical.common.environment.is_inline_capable`:
        Jupyter / Colab get inline SVG only (no viewer pop-up, no disk write),
        while bare scripts and IPython REPLs get the legacy render-to-temp-file
        + open-in-OS-PDF-viewer behavior. Pass explicit booleans to override;
        pass ``filename`` to force a specific on-disk output.
        """
        from minilink.graphical.diagrams import plot_diagram

        return plot_diagram(
            self,
            show_inline=show_inline,
            show_pdf=show_pdf,
            filename=filename,
        )

    def render(self, x, u, t, is_3d=False, renderer="matplotlib"):
        """
        Convenience shortcut rendering a single frame of the system.
        """
        from minilink.graphical.animation import Animator

        animator = Animator(self)
        return animator.show(x, u, t, is_3d=is_3d, renderer=renderer)

    def render_v2(self, x, u, t, is_3d=False, renderer="matplotlib", camera=None):
        """
        Convenience shortcut rendering a single frame through the v2 pipeline.

        Parallel to :meth:`render`; uses ``Animator2`` and the frame-keyed
        ``tf_v2`` / ``get_kinematic_geometry_v2`` / ``get_dynamic_geometry_v2``
        hooks (empty until the Phase 3 catalog migration). ``camera`` accepts a
        constant 4x4 or a ``camera(frames, x, u, t)`` callable override.
        """
        from minilink.graphical.animation.animator2 import Animator2

        animator = Animator2(self)
        return animator.show(x, u, t, is_3d=is_3d, renderer=renderer, camera=camera)

    def animate_v2(
        self,
        traj=None,
        time_factor_video=1.0,
        is_3d=False,
        html: bool | None = None,
        renderer="matplotlib",
        native: bool = True,
        scene_title: str | None = None,
        show: bool = True,
        camera=None,
    ):
        """
        Convenience shortcut to animate a trajectory through the v2 pipeline.

        Parallel to :meth:`animate`; uses ``Animator2`` and the frame-keyed v2
        hooks. ``camera`` accepts the Layer-3 override (a constant 4x4 or a
        ``camera(frames, x, u, t)`` callable). The v2 hooks are empty until the
        Phase 3 catalog migration, so this draws nothing for unmigrated plants.
        """
        from minilink.graphical.animation.animator2 import Animator2
        from minilink.graphical.common.environment import prefers_inline_animation

        if traj is None:
            if self.traj is not None:
                traj = self.traj
            else:
                traj = self.compute_trajectory()

        resolved_html = prefers_inline_animation() if html is None else html

        animator = Animator2(self)
        show_plot = show and not resolved_html
        return animator.animate_simulation(
            traj,
            time_factor_video=time_factor_video,
            is_3d=is_3d,
            html=resolved_html,
            show=show_plot,
            renderer=renderer,
            native=native,
            scene_title=scene_title,
            camera=camera,
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
        Pass ``native=False`` to fall back to the legacy per-frame
        Python-loop playback (useful for debugging or when the native
        path's limitations matter — e.g. meshcat freezes dynamic-geometry
        primitives such as ``TorqueArrow``; see ``DESIGN.md`` §4.7).
        """
        from minilink.graphical.animation import Animator
        from minilink.graphical.common.environment import prefers_inline_animation

        if traj is None:
            if self.traj is not None:
                traj = self.traj
            else:
                traj = self.compute_trajectory()

        resolved_html = prefers_inline_animation() if html is None else html

        animator = Animator(self)
        show_plot = show and not resolved_html
        ani_obj = animator.animate_simulation(
            traj,
            time_factor_video=time_factor_video,
            is_3d=is_3d,
            html=resolved_html,
            show=show_plot,
            renderer=renderer,
            native=native,
            scene_title=scene_title,
        )

        # For html output, return the IPython.display.HTML object and let the
        # notebook auto-display it via the standard last-expression rule.
        # Calling display.display() *and* returning the object renders twice.
        return ani_obj

    def modal_analysis(
        self,
        x_bar=None,
        u_bar=None,
        *,
        mode=None,
        method="fd",
        amplitude=1.0,
        tf=None,
        n_steps=2001,
        time_factor_video=3.0,
        renderer="matplotlib",
        is_3d=False,
        show=True,
        html=None,
        native=True,
        t=0.0,
        params=None,
        epsilon=1e-6,
    ):
        """
        Linearize and eigendecompose ``A``.

        Returns ``(poles, modes)``. With ``mode=None``, analyze only.
        With ``mode=0`` or ``mode='all'``, delegates to
        :func:`~minilink.analysis.modal.animate_modal`.
        """
        from minilink.analysis.modal import animate_modal, modal_analysis

        if x_bar is None:
            x_bar = self.x0
        if mode is not None:
            return animate_modal(
                self,
                x_bar,
                mode,
                u_bar,
                t=t,
                params=params,
                method=method,
                epsilon=epsilon,
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
        return modal_analysis(
            self,
            x_bar,
            u_bar,
            t=t,
            params=params,
            method=method,
            epsilon=epsilon,
        )

    def game(
        self,
        *,
        dt=1 / 30.0,
        dynamics_substeps=1,
        renderer="pygame",
        is_3d=False,
        x0=None,
        u0=None,
        t0=0.0,
        max_steps=None,
    ):
        """
        Convenience shortcut for the prototype interactive game loop.

        See ``Animator.game`` and ``ROADMAP.md`` §7: integrator + live I/O are planned as
        pluggable backends (today: pygame keyboard + Euler in the animator loop).
        """
        from minilink.graphical.animation import Animator

        animator = Animator(self)
        return animator.game(
            dt=dt,
            dynamics_substeps=dynamics_substeps,
            renderer=renderer,
            is_3d=is_3d,
            x0=self.x0 if x0 is None else x0,
            u0=np.zeros(self.m) if u0 is None else u0,
            t0=t0,
            max_steps=max_steps,
        )

    def game_v2(
        self,
        *,
        dt=1 / 30.0,
        dynamics_substeps=1,
        renderer="pygame",
        is_3d=False,
        x0=None,
        u0=None,
        t0=0.0,
        max_steps=None,
    ):
        """
        Interactive game loop through the v2 pipeline.

        Parallel to :meth:`game`; uses ``Animator2`` and the frame-keyed v2 hooks
        (``tf_v2`` / ``get_kinematic_geometry_v2`` / ``get_dynamic_geometry_v2``),
        so live force/torque arrows are rebuilt each tick. See ``Animator2.game``
        and ``ROADMAP.md`` §7.
        """
        from minilink.graphical.animation.animator2 import Animator2

        animator = Animator2(self)
        return animator.game(
            dt=dt,
            dynamics_substeps=dynamics_substeps,
            renderer=renderer,
            is_3d=is_3d,
            x0=self.x0 if x0 is None else x0,
            u0=np.zeros(self.m) if u0 is None else u0,
            t0=t0,
            max_steps=max_steps,
        )
