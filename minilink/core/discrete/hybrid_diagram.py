"""Hybrid diagrams: a scheduled computer side and a continuous plant side."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal

from minilink.core.diagram import DiagramSystem
from minilink.core.discrete.step_diagram import StepDiagramSystem
from minilink.core.inspect import inspect_text

if TYPE_CHECKING:
    from minilink.simulation.discrete.computer import Computer

# The older spellings, which name the computer side after its step diagram
_DIRECTION_ALIASES = {
    "step_to_plant": "computer_to_plant",
    "plant_to_step": "plant_to_computer",
}

BoundaryDirection = Literal["computer_to_plant", "plant_to_computer"]


###############################################################################
# Public API
###############################################################################


@dataclass
class HybridDiagram:
    """
    Two-side hybrid topology: :class:`Computer` + continuous :class:`DiagramSystem`.

    Boundary ports between the sides use ZOH (computer → plant) or sample
    (plant → computer). The schedule lives on
    :class:`~minilink.simulation.discrete.computer.Computer`.

    Parameters
    ----------
    computer : Computer
        Scheduled step runtime (diagram + :class:`~minilink.simulation.discrete.computer.StepSchedule`).
    plant : DiagramSystem
        Continuous plant diagram.
    connections : list of BoundaryConnection, optional
        Multi-channel boundary wiring.
    """

    computer: Computer
    plant: DiagramSystem
    connections: list[BoundaryConnection] = field(default_factory=list)

    def __post_init__(self) -> None:
        # The two sides: a scheduled runtime and a continuous plant diagram
        require_sides(self.computer, self.plant)

        # Memory: the latest simulation, and its plant trajectory
        self.last_result = None
        self.traj = None

    @property
    def rollout(self):
        """Tick-indexed computer view from :attr:`last_result` (``None`` if unset)."""
        if self.last_result is None:
            return None

        return self.last_result.computer

    @classmethod
    def from_diagrams(
        cls,
        step_diagram: StepDiagramSystem,
        plant: DiagramSystem,
        schedule,
        *,
        connections: list[BoundaryConnection] | None = None,
    ) -> HybridDiagram:
        """Wrap ``step_diagram`` and ``schedule`` in a :class:`Computer`."""
        # Tools import core (DESIGN §3): the simulation band loads on first use
        from minilink.simulation.discrete.computer import Computer, as_schedule

        schedule = as_schedule(schedule)
        computer = Computer(step_diagram, schedule)

        return cls(computer=computer, plant=plant, connections=list(connections or []))

    def connect_boundary(
        self,
        *,
        direction: BoundaryDirection | str,
        computer_port: str,
        plant_port: str,
    ) -> None:
        """Append and validate one boundary channel."""
        direction = normalize_direction(direction)
        conn = BoundaryConnection(
            direction=direction,
            computer_port=computer_port,
            plant_port=plant_port,
        )
        require_boundary_ports(self, conn)
        self.connections.append(conn)

    def compute_trajectory(
        self,
        t0=0,
        tf=10,
        *,
        x0_plant=None,
        x0_computer=None,
        n_steps=None,
        plant_dt_inner=None,
        compile_backend="numpy",
        verbose=True,
        show=False,
    ):
        """Simulate with nominal boundary inputs on ``[t0, tf]``.

        Parameters
        ----------
        verbose : bool
            Print hybrid simulator setup (default ``True`` for interactive use).

        Returns
        -------
        HybridSimResult
            Stored on :attr:`last_result`; plant view on :attr:`traj`.
        """
        result = self.build_simulator(
            t0=t0,
            tf=tf,
            x0_plant=x0_plant,
            x0_computer=x0_computer,
            n_steps=n_steps,
            plant_dt_inner=plant_dt_inner,
            compile_backend=compile_backend,
            verbose=verbose,
        ).solve()
        self.cache_result(result)

        if show:
            result.plot()

        return result

    def compute_forced(
        self,
        u,
        *,
        input_port_id=None,
        t0=0,
        tf=10,
        x0_plant=None,
        x0_computer=None,
        n_steps=None,
        plant_dt_inner=None,
        compile_backend="numpy",
        verbose=True,
        show=False,
    ):
        """Simulate with prescribed computer-boundary input on ``[t0, tf]``.

        Parameters
        ----------
        verbose : bool
            Print hybrid simulator setup (default ``True`` for interactive use).

        Returns
        -------
        HybridSimResult
            Stored on :attr:`last_result`; plant view on :attr:`traj`.
        """
        result = self.build_simulator(
            t0=t0,
            tf=tf,
            x0_plant=x0_plant,
            x0_computer=x0_computer,
            n_steps=n_steps,
            plant_dt_inner=plant_dt_inner,
            compile_backend=compile_backend,
            verbose=verbose,
        ).solve_forced(u, input_port_id=input_port_id)
        self.cache_result(result)

        if show:
            result.plot()

        return result

    def plot_trajectory(
        self,
        traj=None,
        *,
        signals=None,
        abscissa="t",
        show=True,
        backend="matplotlib",
        **kwargs,
    ):
        """
        Plot plant channels (continuous-time view) from the last rollout or result,
        or the computer channels on the tick index with ``abscissa="k"``.

        If ``traj`` is omitted and :attr:`traj` is unset, runs :meth:`compute_trajectory`
        first (same convenience pattern as :class:`~minilink.core.facades.plotting.SystemPlotting`).

        ``abscissa`` picks the view: ``"t"`` plots the plant channels against time
        (:meth:`HybridSimResult.plot`); ``"k"`` plots the computer channels against
        the tick index (:meth:`HybridSimResult.plot_computer`). A plain
        :class:`~minilink.core.trajectory.Trajectory` passed with ``abscissa="k"``
        must be a computer rollout, ``traj=result.computer.as_trajectory()``, and is
        drawn with the same default channels; a plant trajectory raises
        ``ValueError``.
        """
        require_abscissa(abscissa)

        # The latest result, simulated first when there is none
        if traj is None:
            if self.last_result is None:
                self.compute_trajectory(show=False, verbose=False)
            traj = self.last_result

        # A real cycle: simulation.discrete.hybrid_simulator imports HybridDiagram from this module
        from minilink.simulation.discrete.hybrid_simulator import HybridSimResult

        # A hybrid result draws its own plant and computer views
        if isinstance(traj, HybridSimResult):
            plot_view = traj.plot if abscissa == "t" else traj.plot_computer
            return plot_view(
                signals=signals,
                show=show,
                backend=backend,
                **kwargs,
            )

        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.signals.time_signals import (
            STEP_ABSCISSA_LABEL,
            TIME_ABSCISSA_LABEL,
            plot_time_signals,
            resolve_plot_signals,
        )

        # A plain trajectory: the plant against time, or a computer rollout
        # against the tick index
        if abscissa == "t":
            sys = self.plant
            abscissa_label = TIME_ABSCISSA_LABEL
        else:
            sys = self.computer.diagram
            abscissa_label = STEP_ABSCISSA_LABEL
            require_computer_rollout(traj, sys)
            if signals is None and traj.signals:
                signals = tuple(traj.signals)

        # The default channels, with the input when the trajectory carries one
        if signals is None:
            signals = resolve_plot_signals(sys)
            if traj.u.shape[0]:
                signals = tuple(dict.fromkeys((*signals, "u")))

        return plot_time_signals(
            sys,
            traj,
            signals=signals,
            abscissa_label=abscissa_label,
            backend=backend,
            show=show,
            **kwargs,
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
        Animate the continuous plant using the fine :attr:`traj` cache.

        Same kwargs as :meth:`~minilink.core.facades.animation.SystemAnimation.animate`
        on a flow diagram. Uses :attr:`plant` geometry; ``traj`` defaults to the
        last plant trajectory from :meth:`compute_trajectory` / :meth:`compute_forced`.
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation import Animator
        from minilink.graphical.common.environment import prefers_inline_animation

        # The cached plant trajectory, else the latest result's, else a fresh run
        if traj is None:
            if self.traj is not None:
                traj = self.traj
            elif self.last_result is not None:
                traj = self.last_result.plant
            else:
                self.compute_trajectory(verbose=False)
                traj = self.traj

        resolved_html = prefers_inline_animation() if html is None else html

        # A real cycle: core.discrete.hybrid_composition imports HybridDiagram from this module
        from minilink.core.discrete.hybrid_composition import (
            refresh_plant_animation_camera,
        )

        # The plant's camera hints, refreshed from its blocks
        refresh_plant_animation_camera(self.plant)
        animator = Animator(self.plant)
        show_plot = show and not resolved_html

        return animator.animate_simulation(
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

    def plot_diagram(
        self,
        filename=None,
        show=True,
        show_inline=None,
        show_pdf=None,
        *,
        abstract_boundary=True,
    ):
        """Render Plant + Computer clusters with boundary ZOH/sample edges.

        Display policy matches :meth:`minilink.core.facades.plotting.SystemPlotting.plot_diagram`
        (notebook SVG as the returned cell output, without a second
        ``display()``; script Matplotlib window; ``show_pdf`` / ``filename`` /
        ``show=False`` overrides). ``abstract_boundary=True`` (default) omits
        external Inputs/Outputs routing nodes and anchors hybrid edges on the
        wired subsystem ports.
        """
        # A real cycle: graphical.diagrams.hybrid_dot imports HybridDiagram through
        # hybrid_topology
        from minilink.graphical.diagrams.hybrid_dot import plot_hybrid_diagram

        return plot_hybrid_diagram(
            self,
            filename=filename,
            show=show,
            show_inline=show_inline,
            show_pdf=show_pdf,
            abstract_boundary=abstract_boundary,
        )

    def __str__(self):
        return inspect_text(self)

    def _repr_svg_(self):
        """Notebook inline SVG for the hybrid topology (IPython / Jupyter)."""
        # A real cycle: graphical.diagrams.hybrid_dot imports HybridDiagram through
        # hybrid_topology
        from minilink.graphical.diagrams.hybrid_dot import export_hybrid_graphviz

        # A real cycle: hybrid_topology imports HybridDiagram from this module
        from minilink.graphical.diagrams.hybrid_topology import build_hybrid_topology

        graph = export_hybrid_graphviz(build_hybrid_topology(self))
        if graph is None:
            return None

        try:
            return graph._repr_image_svg_xml()
        except Exception:
            return None

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def build_simulator(
        self,
        *,
        t0=0,
        tf=10,
        x0_plant=None,
        x0_computer=None,
        n_steps=None,
        plant_dt_inner=None,
        compile_backend="numpy",
        verbose=False,
    ):
        """The :class:`~minilink.simulation.discrete.hybrid_simulator.HybridSimulator` of one run."""
        # A real cycle: simulation.discrete.hybrid_simulator imports HybridDiagram from this module
        from minilink.simulation.discrete.hybrid_simulator import HybridSimulator

        return HybridSimulator(
            self,
            x0_plant=x0_plant,
            x0_computer=x0_computer,
            t0=t0,
            tf=tf,
            n_steps=n_steps,
            plant_dt_inner=plant_dt_inner,
            compile_backend=compile_backend,
            verbose=verbose,
        )

    def cache_result(self, result) -> None:
        """Store full hybrid result and plant trajectory cache."""
        self.last_result = result
        self.traj = result.plant


@dataclass(frozen=True)
class BoundaryConnection:
    """One channel across the computer ↔ plant boundary."""

    direction: BoundaryDirection
    computer_port: str
    plant_port: str

    def __post_init__(self) -> None:
        # The canonical direction, set past the frozen dataclass
        direction = normalize_direction(self.direction)
        object.__setattr__(self, "direction", direction)


###############################################################################
# Internal machinery
###############################################################################


def require_sides(computer, plant) -> None:
    """Refuse a computer that is not a ``Computer``, or a plant not a ``DiagramSystem``."""
    # Tools import core (DESIGN §3): the simulation band loads on first use
    from minilink.simulation.discrete.computer import Computer

    if not isinstance(computer, Computer):
        raise TypeError(
            f"HybridDiagram requires Computer, got {type(computer).__name__}"
        )
    if not isinstance(plant, DiagramSystem):
        raise TypeError(
            f"HybridDiagram requires DiagramSystem plant, got {type(plant).__name__}"
        )


def normalize_direction(direction: str) -> BoundaryDirection:
    """The canonical boundary direction, the step-diagram aliases resolved."""
    key = _DIRECTION_ALIASES.get(direction, direction)
    if key not in ("computer_to_plant", "plant_to_computer"):
        raise ValueError(
            f"direction must be 'computer_to_plant' or 'plant_to_computer', "
            f"got {direction!r}"
        )

    return key  # type: ignore[return-value]


def require_boundary_ports(hybrid: HybridDiagram, conn: BoundaryConnection) -> None:
    """Refuse a channel whose ports are missing on either side or differ in size."""
    diagram = hybrid.computer.diagram
    plant = hybrid.plant

    # Computer output → plant input
    if conn.direction == "computer_to_plant":
        if conn.computer_port not in diagram.outputs:
            raise ValueError(
                f"Unknown computer output port {conn.computer_port!r}; "
                f"available: {', '.join(diagram.outputs) or '(none)'}"
            )
        if conn.plant_port not in plant.inputs:
            raise ValueError(
                f"Unknown plant input port {conn.plant_port!r}; "
                f"available: {', '.join(plant.inputs) or '(none)'}"
            )
        c_dim = diagram.outputs[conn.computer_port].dim
        p_dim = plant.inputs[conn.plant_port].dim

    # Plant output → computer input
    else:
        if conn.computer_port not in diagram.inputs:
            raise ValueError(
                f"Unknown computer input port {conn.computer_port!r}; "
                f"available: {', '.join(diagram.inputs) or '(none)'}"
            )
        if conn.plant_port not in plant.outputs:
            raise ValueError(
                f"Unknown plant output port {conn.plant_port!r}; "
                f"available: {', '.join(plant.outputs) or '(none)'}"
            )
        c_dim = diagram.inputs[conn.computer_port].dim
        p_dim = plant.outputs[conn.plant_port].dim

    if c_dim != p_dim:
        raise ValueError(
            f"Boundary dimension mismatch on {conn.computer_port!r} ↔ "
            f"{conn.plant_port!r}: {c_dim} vs {p_dim}"
        )


def require_abscissa(abscissa) -> None:
    """Refuse a view other than plant time ``"t"`` or computer ticks ``"k"``."""
    if abscissa not in ("t", "k"):
        raise ValueError(
            "abscissa must be 't' (plant time) or 'k' (computer ticks), "
            f"got {abscissa!r}"
        )


def require_computer_rollout(traj, sys) -> None:
    """Refuse a trajectory whose state and input sizes are not the computer diagram's."""
    if traj.x.shape[0] != sys.n or traj.u.shape[0] != sys.m:
        raise ValueError(
            "abscissa='k' plots a computer rollout: pass the HybridSimResult "
            "or result.computer.as_trajectory(). This trajectory has "
            f"n={traj.x.shape[0]}, m={traj.u.shape[0]}; the computer "
            f"diagram has n={sys.n}, m={sys.m}."
        )
