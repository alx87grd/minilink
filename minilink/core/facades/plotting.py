"""Plotting shortcuts: a system's trajectory, cost, maps, diagram and linear-analysis plots."""

from minilink.core.facades.simulation import scored_trajectory
from minilink.core.inspect import inspect_text

###############################################################################
# Public API
###############################################################################


class SystemPlotting:
    """
    Plotting shortcuts every :class:`~minilink.core.system.System` has: its trajectory, its cost, its maps and its block diagram.
    """

    def plot_trajectory(
        self,
        traj=None,
        *,
        signals=None,
        renderer="matplotlib",
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
            pair resolved via :meth:`~minilink.core.diagram.DiagramSystem.subsystem_signal`;
            the pair may name a block nested in a sub-diagram (``"loop/plant:x"``).
            When ``None``, defaults are chosen via
            :func:`minilink.graphical.signals.resolve_plot_signals`.

        Returns
        -------
        PlotResult
            The plot result from
            :func:`minilink.graphical.signals.plot_time_signals`.
        """
        # A real cycle through minilink.core.system
        from minilink.graphical.signals import plot_time_signals, resolve_plot_signals

        if signals is None:
            signals = resolve_plot_signals(self)

        # The trajectory passed, else the last one computed, else a fresh one
        if traj is None:
            traj = self.traj
        if traj is None:
            traj = self.compute_trajectory(show=False, verbose=False)

        return plot_time_signals(
            self,
            traj,
            signals=signals,
            renderer=renderer,
            show=show,
        )

    def plot_cost(self, cost, *, of=None, traj=None, renderer="matplotlib", show=True):
        """
        Convenience shortcut plotting the running cost and the cumulative cost ``J(t)``.

        The signals are ``cost_rate`` (the discounted running cost) and
        ``cost`` (its integral from the start); :meth:`compute_cost` adds the
        terminal cost to the final value. ``of`` and ``traj`` select the scored
        trajectory as in :meth:`compute_cost`.

        Returns
        -------
        PlotResult
            The plot result from
            :func:`minilink.graphical.signals.plot_time_signals`.
        """
        scored_system = self if of is None else of
        scored = scored_trajectory(self, of, traj).with_cost(cost)

        return scored_system.plot_trajectory(
            scored, signals=("cost_rate", "cost"), renderer=renderer, show=show
        )

    def plot_input_output_map(
        self,
        *,
        in_port=None,
        out_port=None,
        x_axis=0,
        y_axis=None,
        out_axis=0,
        bounds=None,
        grid_shape=(101, 101),
        is_3d=False,
        params=None,
        t=0.0,
        title=None,
        cmap="bwr",
        vmin=None,
        vmax=None,
        ax=None,
        show=True,
    ):
        """
        Convenience shortcut plotting one output component over swept inputs.

        Sweeps components of one input port (line, heatmap, or
        ``is_3d=True`` surface) with every other input pinned at its
        nominal value. See
        :func:`minilink.graphical.port_map.plot_input_output_map`.
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.port_map import plot_input_output_map

        return plot_input_output_map(
            self,
            in_port=in_port,
            out_port=out_port,
            x_axis=x_axis,
            y_axis=y_axis,
            out_axis=out_axis,
            bounds=bounds,
            grid_shape=grid_shape,
            is_3d=is_3d,
            params=params,
            t=t,
            title=title,
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
            ax=ax,
            show=show,
        )

    def plot_control_law(
        self,
        *,
        x_axis=None,
        y_axis=None,
        u_axis=0,
        r=None,
        space=None,
        bounds=None,
        grid_shape=(101, 101),
        is_3d=False,
        params=None,
        t=0.0,
        title=None,
        cmap="bwr",
        vmin=None,
        vmax=None,
        ax=None,
        show=True,
    ):
        """
        Convenience shortcut plotting the textbook slice of this block's control law.

        The zero-argument call sweeps the coordinates the block's feedback
        declaration names (the error for an error-driven law, the measurement
        or the state otherwise) and pins the reference, every other coordinate
        and any internal state at nominal; ``x_axis`` / ``y_axis`` index the
        workspace ``z = [measurement-space; x_ctrl]`` for teaching slices such
        as windup. A block that reads no measurement has no control law to
        draw; :meth:`plot_input_output_map` sweeps any input of any block. See
        :func:`minilink.graphical.port_map.plot_control_law`.
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.port_map import plot_control_law

        return plot_control_law(
            self,
            x_axis=x_axis,
            y_axis=y_axis,
            u_axis=u_axis,
            r=r,
            space=space,
            bounds=bounds,
            grid_shape=grid_shape,
            is_3d=is_3d,
            params=params,
            t=t,
            title=title,
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
            ax=ax,
            show=show,
        )

    def get_diagram(self, *, expand=True):
        """
        Convenience shortcut returning a renderable diagram representation.

        ``expand=False`` draws a nested diagram as one block instead of a
        labelled box around its own blocks.
        """
        # A real cycle through minilink.core.system
        from minilink.graphical.diagrams import get_diagram

        return get_diagram(self, expand=expand)

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

    def _repr_mimebundle_(self, include=None, exclude=None):
        """Notebook repr: text inspect plus inline diagram SVG when available."""
        bundle = {"text/plain": inspect_text(self)}
        svg = self._repr_svg_()
        if svg is not None:
            bundle["image/svg+xml"] = svg

        return bundle

    def plot_diagram(self, file_name=None, show=True, *, expand=True):
        """
        Convenience shortcut to render the system diagram.

        A diagram that contains another diagram shows the inner blocks inside
        a labelled box, at every depth, with the wires going straight to the
        inner ports. Pass ``expand=False`` to draw each nested diagram as one
        block with its boundary ports.

        ``show`` is where the figure goes: ``True`` picks the place, inline
        SVG in Jupyter / Colab (the returned Graphviz object is the cell
        output; this method does not also call ``display()``, which would
        draw the figure twice) and a Matplotlib window with the Graphviz PNG
        in bare scripts and IPython REPLs (same blocking policy as trajectory
        plots); ``"inline"``, ``"window"`` or ``"pdf"`` (the OS PDF viewer)
        name it; ``False`` builds the Digraph only. Pass ``file_name`` to
        write Graphviz output to disk.
        Without the Graphviz binary the figure is skipped with a warning and
        ``None`` is returned, so a notebook cell does not fail.
        """
        # A real cycle through minilink.core.system
        from minilink.graphical.diagrams import plot_diagram

        return plot_diagram(self, file_name=file_name, show=show, expand=expand)


class LinearPlotting:
    """
    The linear-analysis plots of one SISO channel, in ``s`` on a :class:`~minilink.core.system.DynamicSystem` and in ``z`` on a :class:`~minilink.core.discrete.step_system.StepSystem`.
    """

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
        n_samples=200,
        margins=True,
        method="auto",
        eps=1e-6,
        renderer="matplotlib",
        show=True,
        title=None,
    ):
        """Bode diagram of one SISO channel, gain and phase margins marked.

        See :func:`minilink.analysis.frequency.plot_bode`.
        """
        # A real cycle through minilink.core.system
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
            n_samples=n_samples,
            margins=margins,
            method=method,
            eps=eps,
            renderer=renderer,
            show=show,
            title=title,
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
        minimal=None,
        radius=None,
        renderer="matplotlib",
        show=True,
    ):
        """Root locus of one SISO channel closed with ``u = -K y``.

        See :func:`minilink.analysis.frequency.plot_root_locus`.
        """
        # A real cycle through minilink.core.system
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
            minimal=minimal,
            radius=radius,
            renderer=renderer,
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
        minimal=None,
        radius=None,
        renderer="matplotlib",
        show=True,
    ):
        """Plot poles and zeros of one SISO channel.

        See :func:`minilink.analysis.frequency.plot_pzmap`.
        """
        # A real cycle through minilink.core.system
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
            minimal=minimal,
            radius=radius,
            renderer=renderer,
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
        n_samples=500,
        method="auto",
        eps=1e-6,
        renderer="matplotlib",
        show=True,
    ):
        """Nyquist diagram of one SISO channel, with the critical point ``-1``.

        See :func:`minilink.analysis.frequency.plot_nyquist`.
        """
        # A real cycle through minilink.core.system
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
            n_samples=n_samples,
            method=method,
            eps=eps,
            renderer=renderer,
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
        n_samples=500,
        method="auto",
        eps=1e-6,
        renderer="matplotlib",
        show=True,
    ):
        """Step response of one SISO channel of the linear model, with rise time, settling time and overshoot.

        See :func:`minilink.analysis.time_response.plot_step_response`.
        """
        # A real cycle through minilink.core.system
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
            n_samples=n_samples,
            method=method,
            eps=eps,
            renderer=renderer,
            show=show,
        )


class DynamicPlotting(LinearPlotting):
    """
    Plotting shortcuts for :class:`~minilink.core.system.DynamicSystem`: the phase plane and the linear-analysis plots.
    """

    def plot_phase_plane(
        self,
        traj=None,
        *,
        x_axis=0,
        y_axis=None,
        renderer="matplotlib",
        show=True,
        **kwargs,
    ):
        """
        Convenience shortcut to plot a phase-plane vector field.

        If ``traj`` is provided, or if :attr:`traj` contains a previous
        simulation result, the sampled state path is overlaid on the vector
        field. Otherwise only the vector field is plotted. See
        :func:`minilink.graphical.phase_plane.plot_phase_plane`.
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.phase_plane import plot_phase_plane

        if traj is None:
            traj = self.traj

        return plot_phase_plane(
            self,
            traj,
            x_axis=x_axis,
            y_axis=y_axis,
            renderer=renderer,
            show=show,
            **kwargs,
        )


class StepPlotting(LinearPlotting):
    """
    Plotting shortcuts for :class:`~minilink.core.discrete.step_system.StepSystem`: the rollout and the linear-analysis plots.
    """

    def plot_rollout(
        self,
        traj=None,
        *,
        signals=None,
        renderer="matplotlib",
        show=True,
    ):
        """
        Convenience shortcut to plot sampled step signals.

        ``traj`` is the rollout to plot; ``None`` reads :attr:`traj`, the last
        :meth:`compute_rollout`.
        """
        # A real cycle through minilink.core.system
        from minilink.graphical.signals import (
            STEP_ABSCISSA_LABEL,
            TIME_ABSCISSA_LABEL,
            plot_time_signals,
            resolve_plot_signals,
        )

        if signals is None:
            signals = resolve_plot_signals(self)

        # The rollout passed, else the last one computed
        if traj is None:
            traj = self.traj
        require_rollout(traj)

        return plot_time_signals(
            self,
            traj,
            signals=signals,
            abscissa_label=(
                STEP_ABSCISSA_LABEL
                if self.sample_period is None
                else TIME_ABSCISSA_LABEL
            ),
            renderer=renderer,
            show=show,
        )


###############################################################################
# Internal machinery
###############################################################################


def require_rollout(rollout):
    """Refuse a missing rollout: none was passed and none was computed."""
    if rollout is None:
        raise ValueError(
            "No rollout available; pass traj=... or call compute_rollout first."
        )
