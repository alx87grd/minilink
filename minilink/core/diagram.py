"""Block diagrams: systems built by connecting other systems."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import DynamicSystem, System
from minilink.core.trajectory import Trajectory
from minilink.core.wiring import WiredDiagramMixin, check_diagram_params

__all__ = ["DiagramSystem", "check_diagram_params"]


###############################################################################
# Public API
###############################################################################


class DiagramSystem(WiredDiagramMixin, DynamicSystem):
    """
    A system composed of subsystems connected through their named ports.

    A diagram is a :class:`~minilink.core.system.DynamicSystem`, so diagrams
    nest, simulate, and compile like any other continuous system. The diagram
    state stacks the subsystem states in insertion order,

        x = [x_1; x_2; ...; x_k],        dx_i = f_i(x_i, u_i, t; p_i),

    where each local input ``u_i`` is gathered from the output ports connected
    to subsystem ``i``, from the diagram boundary inputs, or from the port's
    constant nominal value when unconnected. Diagram-level params are nested
    by subsystem id: ``{"plant": {...}, "controller": {...}}``. ``@`` names
    its blocks ``controller`` and ``plant``; ``+`` and ``>>`` keep the blocks'
    own names.

    Build diagrams with :meth:`add_subsystem` + :meth:`connect` (canonical
    wiring for any topology) or with the shortcuts ``+``, ``>>``, ``@``, and
    :meth:`autowire` from :mod:`minilink.core.composition`.

    This class implements the wiring contract and a simple recursive
    evaluation of ``f`` used as the reference; :meth:`compile` produces the
    fast :class:`~minilink.core.compile.plan.ExecutionPlan`-based
    evaluator used by simulation and optimization.
    """

    def __init__(self):
        # The registry first: the params setter reads it while System.__init__ runs
        self.subsystems = {}
        self.connections = {}

        # The System shell alone: a diagram's ports are the ones it exposes, not
        # DynamicSystem's standard u and y
        System.__init__(self, 0)

        # Wiring tables and the stacked-state layout
        self.init_wiring()

    def add_subsystem(self, sys, sys_id):
        """Add a continuous or static subsystem under a unique id.

        A :class:`StepSystem` has no ``f`` to stack, so it is refused here
        rather than left out of the state derivative.
        """
        require_flow_subsystem(sys, sys_id)

        super().add_subsystem(sys, sys_id)

    def f(self, x, u, t=0, params=None):
        """
        Stacked state derivative ``dx = [f_1(x_1, u_1, t); f_2(x_2, u_2, t); ...]``.

        This is the interpreted reference implementation; :meth:`compile`
        produces the fast equivalent used by simulation and optimization.
        """
        check_diagram_params(params, self.subsystems)

        dx_pieces = []
        for sys_id, subsystem in self.subsystems.items():
            # A static block has no f to stack
            if not isinstance(subsystem, DynamicSystem):
                continue

            # x_i, u_i and p_i of block i, gathered through the wiring
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id, params=params)
            local_params = self.subsystem_params(params, sys_id)

            # ẋ_i = f_i(x_i, u_i, t; p_i)
            dx_pieces.append(subsystem.f(local_x, local_u, t, local_params))

        xp = array_module(x, u, *dx_pieces)

        # ẋ = [f_1(x_1, u_1, t); f_2(x_2, u_2, t); …], one block per subsystem,
        # and empty when no block carries an f
        if not dx_pieces:
            dx = xp.array([])
        else:
            dx = xp.concatenate([xp.asarray(dx_i).reshape(-1) for dx_i in dx_pieces])

        return dx

    def compile(self, backend="numpy", bind_params=False, verbose=False):
        """
        Compile the diagram into a stateless evaluator for fast simulation.

        Runs algebraic-loop detection internally; raises RuntimeError if a
        loop is found.

        Parameters
        ----------
        backend : str
            ``'numpy'`` (default) or ``'jax'``.
        bind_params : bool, optional
            If ``True``, subsystem ``params`` are deep-copied into the plan at
            compile time (see :func:`minilink.core.compile.compiler.compile_diagram`).
        verbose : bool
            If ``True``, print timed compilation steps.

        Returns
        -------
        DiagramEvaluator
        """
        # Lazy import: a cycle, the compiler reads DiagramSystem from this module
        from minilink.core.compile.compiler import compile_diagram

        return compile_diagram(
            self, backend=backend, bind_params=bind_params, verbose=verbose
        )

    def reconstruct_internal_signals(self, traj: Trajectory) -> Trajectory:
        """
        Reconstruct all subsystem output-port trajectories for this diagram.

        Parameters
        ----------
        traj : Trajectory
            State-input trajectory sampled on a time grid.

        Returns
        -------
        Trajectory
            New trajectory enriched with one sampled signal per subsystem
            output port, keyed as ``"sys_id:port_id"``.
        """
        evaluator = self.compile(backend="numpy")

        # One table per subsystem output port, keyed "sys_id:port_id"
        internal_signals = {}
        for sys_id, subsystem in self.subsystems.items():
            for port_id, port in subsystem.outputs.items():
                internal_signals[f"{sys_id}:{port_id}"] = np.zeros(
                    (port.dim, traj.n_samples)
                )

        # Every port's value at each sample, from the compiled evaluator
        for i, t in enumerate(traj.t):
            step_signals = evaluator.compute_internal_signals_dict(
                traj.x[:, i], traj.u[:, i], t
            )
            for key, value in step_signals.items():
                internal_signals[key][:, i] = value

        return traj.with_signals(internal_signals)

    def trajectory_of(self, subsystem, traj=None) -> Trajectory:
        """
        The trajectory of one block of this diagram, as that block saw it.

        ``x`` is the block's own state and ``u`` the input it received from
        the diagram (connected signals, boundary inputs, or nominal values), on
        the time grid of the diagram trajectory. The input is recomputed from
        the stored states with the blocks' current ``params``, so change them
        only after scoring the trajectory they produced. Anything written for
        the block alone applies to it, such as the cost of the plant inside a
        closed loop: ``closed_loop.compute_cost(cost, of=plant)``.

        Parameters
        ----------
        subsystem : System
            A block added to this diagram (for a block inside a nested diagram,
            call ``trajectory_of`` once per level).
        traj : Trajectory, optional
            Trajectory of this diagram; defaults to the last one computed.

        Returns
        -------
        Trajectory
            ``t`` of ``traj``, ``x`` with shape ``(subsystem.n, N)`` and ``u``
            with shape ``(subsystem.m, N)``.
        """
        traj = self.traj if traj is None else traj
        require_trajectory(traj)
        sys_id = self.subsystem_id(subsystem)

        # The block's own state, and the input the diagram fed it, at each sample
        x = np.zeros((subsystem.n, traj.n_samples))
        u = np.zeros((subsystem.m, traj.n_samples))
        for k, t in enumerate(traj.t):
            x[:, k] = self.get_local_state(traj.x[:, k], sys_id)
            u[:, k] = self.get_local_input(traj.x[:, k], traj.u[:, k], t, sys_id)

        return Trajectory(t=traj.t, x=x, u=u)


###############################################################################
# Internal machinery
###############################################################################


def require_flow_subsystem(sys, sys_id):
    """Refuse a :class:`StepSystem` in a flow diagram: it has no ``f`` to stack."""
    if isinstance(sys, StepSystem):
        raise TypeError(
            f"{type(sys).__name__} {sys_id!r} is a StepSystem and cannot join "
            "a flow DiagramSystem: a StepSystem belongs in a StepDiagramSystem "
            "or a Computer: block % dt @ plant"
        )


def require_trajectory(traj):
    """Refuse a missing trajectory: none was passed and none was computed."""
    if traj is None:
        raise ValueError("no trajectory: pass traj or call compute_trajectory()")


if __name__ == "__main__":
    # Hello world: unity-feedback loop  dx = Kp (r - x)
    from minilink.blocks.basic import Integrator
    from minilink.control.siso import P

    diagram = DiagramSystem()
    diagram.add_subsystem(P(ports="reference"), "controller")
    diagram.add_subsystem(Integrator(), "plant")
    diagram.add_input_port("r")
    diagram.connect("input", "r", "controller", "r")
    diagram.connect("plant", "y", "controller", "y")
    diagram.connect("controller", "u", "plant", "u")

    diagram.plot_diagram()

    evaluator = diagram.compile()
    print(evaluator.f(x=np.array([0.5]), u=np.array([1.0]), t=0.0))
