"""Plotting shortcuts: a system's trajectory, cost, maps, diagram and linear-analysis plots."""

from minilink.core.facades.simulation import scored_trajectory

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
            backend=backend,
            show=show,
        )

    def plot_cost(self, cost, *, of=None, traj=None, backend="matplotlib", show=True):
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
            scored, signals=("cost_rate", "cost"), backend=backend, show=show
        )

    def plot_input_output_map(self, **kwargs):
        """
        Convenience shortcut plotting one output component over swept inputs.

        Sweeps components of one input port (line, heatmap, or
        ``show_3d=True`` surface) with every other input pinned at its
        nominal value. See
        :func:`minilink.graphical.port_map.plot_input_output_map`.
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.port_map import plot_input_output_map

        return plot_input_output_map(self, **kwargs)

    def plot_control_law(self, **kwargs):
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

        return plot_control_law(self, **kwargs)

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
        from minilink.core.inspect import inspect_text

        bundle = {"text/plain": inspect_text(self)}
        svg = self._repr_svg_()
        if svg is not None:
            bundle["image/svg+xml"] = svg

        return bundle

    def plot_diagram(
        self,
        filename=None,
        show=True,
        show_inline=None,
        show_pdf=None,
        *,
        expand=True,
    ):
        """
        Convenience shortcut to render the system diagram.

        A diagram that contains another diagram shows the inner blocks inside
        a labelled box, at every depth, with the wires going straight to the
        inner ports. Pass ``expand=False`` to draw each nested diagram as one
        block with its boundary ports.

        Jupyter / Colab get inline SVG by default (the returned Graphviz
        object is the cell output; this method does not also call
        ``display()``, which would draw the figure twice). Bare scripts
        and IPython REPLs open a Matplotlib window with the Graphviz PNG
        (same blocking policy as trajectory plots). Pass ``show_pdf=True``
        for the legacy OS PDF viewer; pass ``filename`` to write Graphviz
        output to disk; pass ``show=False`` to build the Digraph only.
        Without the Graphviz binary the figure is skipped with a warning and
        ``None`` is returned, so a notebook cell does not fail.
        """
        # A real cycle through minilink.core.system
        from minilink.graphical.diagrams import plot_diagram

        return plot_diagram(
            self,
            show=show,
            show_inline=show_inline,
            show_pdf=show_pdf,
            filename=filename,
            expand=expand,
        )


class DynamicPlotting:
    """
    Plotting shortcuts for :class:`~minilink.core.system.DynamicSystem`: the phase plane and the linear-analysis plots.
    """

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
            backend=backend,
            show=show,
            **kwargs,
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
            n=n,
            margins=margins,
            method=method,
            eps=eps,
            backend=backend,
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
        backend="matplotlib",
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
        minimal=None,
        radius=None,
        backend="matplotlib",
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
            n=n,
            method=method,
            eps=eps,
            backend=backend,
            show=show,
        )


class StepPlotting:
    """
    Plotting shortcuts for :class:`~minilink.core.discrete.step_system.StepSystem`.
    """

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
        # A real cycle through minilink.core.system
        from minilink.graphical.signals import (
            STEP_ABSCISSA_LABEL,
            plot_time_signals,
            resolve_plot_signals,
        )

        if signals is None:
            signals = resolve_plot_signals(self)

        # The rollout passed, else the last one computed
        if rollout is None:
            rollout = self.rollout
        require_rollout(rollout)

        return plot_time_signals(
            self,
            rollout.as_trajectory(),
            signals=signals,
            abscissa_label=STEP_ABSCISSA_LABEL,
            backend=backend,
            show=show,
        )


###############################################################################
# Internal machinery
###############################################################################


def require_rollout(rollout):
    """Refuse a missing rollout: none was passed and none was computed."""
    if rollout is None:
        raise ValueError(
            "No rollout available; pass rollout=... or call compute_rollout first."
        )
