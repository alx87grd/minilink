"""
Block diagrams: systems built by connecting other systems.

A :class:`DiagramSystem` is a :class:`~minilink.core.system.DynamicSystem`,
so diagrams nest, simulate, and compile like any other continuous system.
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem, StepSystem, System
from minilink.core.trajectory import Trajectory
from minilink.core.wiring import WiredDiagramMixin, validate_diagram_params

__all__ = ["DiagramSystem", "StepDiagramSystem", "validate_diagram_params"]


class DiagramSystem(WiredDiagramMixin, DynamicSystem):
    """
    A system composed of subsystems connected through their named ports.

    The diagram state stacks the subsystem states in insertion order,

        x = [x_1; x_2; ...; x_k],        dx_i = f_i(x_i, u_i, t; p_i),

    where each local input ``u_i`` is gathered from the output ports connected
    to subsystem ``i``, from the diagram boundary inputs, or from the port's
    constant nominal value when unconnected. Diagram-level params are nested
    by subsystem id: ``{"sys": {...}, "ctl": {...}}``. Shortcut-built
    diagrams default to role ids ``ref``, ``ctl``, and ``sys``.

    Build diagrams with :meth:`add_subsystem` + :meth:`connect` (canonical
    wiring for any topology) or with the shortcuts ``+``, ``>>``, ``@``, and
    :meth:`autowire` from :mod:`minilink.core.composition`.

    This class implements the wiring contract and a simple recursive
    evaluation of ``f`` used as the reference; :meth:`compile` produces the
    fast :class:`~minilink.core.compile.execution_plan.ExecutionPlan`-based
    evaluator used by simulation and optimization.
    """

    def __init__(self):
        # Registry before System.__init__: its params setter runs during shell init.
        self.subsystems = {}
        self.connections = {}
        System.__init__(self, 0)
        self.init_wiring(name="Diagram")

    def add_subsystem(self, sys, sys_id):
        """Add a continuous or static subsystem under a unique id.

        A :class:`StepSystem` has no ``f`` to stack, so it is refused here
        rather than left out of the state derivative.
        """
        if isinstance(sys, StepSystem):
            raise TypeError(
                f"{type(sys).__name__} {sys_id!r} is a StepSystem and cannot join "
                "a flow DiagramSystem: a StepSystem belongs in a StepDiagramSystem "
                "or a Computer: block % dt @ plant"
            )
        super().add_subsystem(sys, sys_id)

    def f(self, x, u, t=0, params=None):
        """
        Stacked state derivative ``dx = [f_1(x_1, u_1, t); f_2(x_2, u_2, t); ...]``.

        This is the interpreted reference implementation; :meth:`compile`
        produces the fast equivalent used by simulation and optimization.
        """
        validate_diagram_params(params, self.subsystems)

        dx_pieces = []
        for sys_id, subsystem in self.subsystems.items():
            if not isinstance(subsystem, DynamicSystem):
                continue
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id, params=params)
            local_params = self.subsystem_params(params, sys_id)

            dx_pieces.append(subsystem.f(local_x, local_u, t, local_params))

        xp = array_module(x, u, *dx_pieces)
        if not dx_pieces:
            return xp.array([])

        # dx = [f_1(x_1, u_1, t); f_2(x_2, u_2, t); ...], one block per subsystem
        dx = xp.concatenate([xp.asarray(dx).reshape(-1) for dx in dx_pieces])

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
        NumpyDiagramEvaluator or JaxDiagramEvaluator
        """
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
        internal_signals = {}
        for sys_id, subsystem in self.subsystems.items():
            for port_id, port in subsystem.outputs.items():
                internal_signals[f"{sys_id}:{port_id}"] = np.zeros(
                    (port.dim, traj.n_samples)
                )

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
        closed loop: ``cost.total_cost(closed_loop.trajectory_of(plant))``.

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
        if traj is None:
            raise ValueError("no trajectory: pass traj or call compute_trajectory()")
        sys_id = self.subsystem_id(subsystem)

        x = np.zeros((subsystem.n, traj.n_samples))
        u = np.zeros((subsystem.m, traj.n_samples))
        for k, t in enumerate(traj.t):
            x[:, k] = self.get_local_state(traj.x[:, k], sys_id)
            u[:, k] = self.get_local_input(traj.x[:, k], traj.u[:, k], t, sys_id)

        return Trajectory(t=traj.t, x=x, u=u)


class StepDiagramSystem(WiredDiagramMixin, StepSystem):
    """
    A discrete-time diagram composed of :class:`StepSystem` and static blocks.

    The diagram state stacks subsystem states in insertion order,

        x_{k+1} = step(x_k, u_k, k),

    where each local input is gathered from connected output ports, boundary
    inputs, or port nominal values. The third gather slot is step index ``k``
    (``int``), not simulation time.

    :meth:`compile` produces a :class:`~minilink.core.compile.evaluators.evaluators.StepEvaluator`
    for fast :meth:`~minilink.core.facades.StepSystemFacades.compute_rollout`.
    """

    def __init__(self):
        self.subsystems = {}
        self.connections = {}
        System.__init__(self, 0)
        self.rollout = None
        self.init_wiring(name="StepDiagram")

    def add_subsystem(self, sys, sys_id):
        """Add a step or static subsystem under a unique id.

        A :class:`DynamicSystem` with states has no ``step`` to stack, so it is
        refused here rather than left frozen by :meth:`step`.
        """
        if isinstance(sys, DynamicSystem) and sys.n > 0:
            raise TypeError(
                f"{type(sys).__name__} {sys_id!r} has continuous states and cannot "
                "join a StepDiagramSystem: keep it in a DiagramSystem and close the "
                "sampled loop with block % dt @ plant, or step it with "
                "discretize(plant, dt)"
            )
        super().add_subsystem(sys, sys_id)

    def step(self, x, u, k=0, params=None):
        """
        Stacked discrete update ``x_new = [step_1(...); step_2(...); ...]``.

        Interpreted reference; :meth:`compile` produces the fast equivalent.
        """
        validate_diagram_params(params, self.subsystems)

        xp = array_module(x, u)
        x_arr = xp.asarray(x, dtype=float).reshape(self.n)
        x_new = xp.array(x_arr, copy=True)
        for sys_id, subsystem in self.subsystems.items():
            if not isinstance(subsystem, StepSystem):
                continue
            local_x = self.get_local_state(x_arr, sys_id)
            local_u = self.get_local_input(x, u, k, sys_id, params=params)
            local_params = self.subsystem_params(params, sys_id)
            piece = subsystem.step(local_x, local_u, k, local_params)
            start, end = self.state_index[sys_id]
            x_new[start:end] = xp.asarray(piece, dtype=float).reshape(end - start)

        return x_new

    def compile(self, backend="numpy", bind_params=False, verbose=False):
        """
        Compile the step diagram into a stateless step evaluator.

        Parameters
        ----------
        backend : str
            ``'numpy'`` (default) or ``'jax'``.
        bind_params : bool, optional
            If ``True``, subsystem ``params`` are deep-copied into the plan.
        verbose : bool
            If ``True``, print timed compilation steps.

        Returns
        -------
        NumpyStepDiagramEvaluator or JaxStepDiagramEvaluator
        """
        from minilink.core.compile.step_compiler import compile_step_diagram

        return compile_step_diagram(
            self, backend=backend, bind_params=bind_params, verbose=verbose
        )


if __name__ == "__main__":
    # Hello world: unity-feedback loop  dx = Kp (r - x)
    from minilink.blocks.basic import Integrator
    from minilink.control.output import ProportionalController

    diagram = DiagramSystem()
    diagram.add_subsystem(ProportionalController(), "ctl")
    diagram.add_subsystem(Integrator(), "plant")
    diagram.add_input_port("r")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("plant", "y", "ctl", "y")
    diagram.connect("ctl", "u", "plant", "u")

    diagram.plot_diagram()

    evaluator = diagram.compile()
    print(evaluator.f(x=np.array([0.5]), u=np.array([1.0]), t=0.0))
