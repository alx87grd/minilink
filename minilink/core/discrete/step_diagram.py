"""Step diagrams: discrete-time systems built by connecting step and static blocks."""

from minilink.core.backends import array_module
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import DynamicSystem, System
from minilink.core.wiring import WiredDiagramMixin, check_diagram_params

###############################################################################
# Public API
###############################################################################


class StepDiagramSystem(WiredDiagramMixin, StepSystem):
    """
    A discrete-time diagram composed of :class:`StepSystem` and static blocks.

    The diagram state stacks subsystem states in insertion order,

        x_{k+1} = step(x_k, u_k, k),

    where each local input is gathered from connected output ports, boundary
    inputs, or port nominal values. The third gather slot is step index ``k``
    (``int``), not simulation time.

    :meth:`compile` produces a :class:`~minilink.core.compile.evaluators.discrete.StepDiagramEvaluator`
    for fast :meth:`~minilink.core.facades.simulation.StepSimulation.compute_rollout`.
    """

    def __init__(self):
        # The registry first: the params setter reads it while System.__init__ runs
        self.subsystems = {}
        self.connections = {}

        # The System shell alone: a step diagram may hold no state yet (StepSystem
        # requires n ≥ 1), and its ports are the ones it exposes
        System.__init__(self, 0)

        # Memory: the last rollout ``compute_rollout`` produced
        self.rollout = None

        # Wiring tables and the stacked-state layout
        self.init_wiring()

    def add_subsystem(self, sys, sys_id):
        """Add a step or static subsystem under a unique id.

        A :class:`DynamicSystem` with states has no ``step`` to stack, so it is
        refused here rather than left frozen by :meth:`step`.
        """
        require_step_subsystem(sys, sys_id)

        super().add_subsystem(sys, sys_id)

    def step(self, x, u, k=0, params=None):
        """
        Stacked discrete update ``x_{k+1} = [step_1(...); step_2(...); ...]``.

        Interpreted reference; :meth:`compile` produces the fast equivalent.
        """
        check_diagram_params(params, self.subsystems)

        xp = array_module(x, u)

        n = self.n

        x_k = xp.asarray(x, dtype=float).reshape(n)

        x_next_pieces = []
        for sys_id, subsystem in self.subsystems.items():
            # A static block has no step to stack, and no slot in x
            if not isinstance(subsystem, StepSystem):
                continue

            # x_i, u_i and p_i of block i, gathered through the wiring
            local_x = self.get_local_state(x_k, sys_id)
            local_u = self.get_local_input(x, u, k, sys_id, params=params)
            local_params = self.subsystem_params(params, sys_id)

            # x_{i,k+1} = step_i(x_i, u_i, k; p_i), on block i's slots
            start, end = self.state_index[sys_id]
            x_next_i = subsystem.step(local_x, local_u, k, local_params)
            x_next_pieces.append(xp.asarray(x_next_i, dtype=float).reshape(end - start))

        # x_{k+1} = [step_1(x_1, u_1, k); step_2(x_2, u_2, k); …], one block per step subsystem
        if not x_next_pieces:
            x_next = xp.zeros(0)
        else:
            x_next = xp.concatenate(x_next_pieces)

        return x_next

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
        StepDiagramEvaluator
        """
        # Lazy import: a cycle, the compiler reads StepDiagramSystem from here
        from minilink.core.compile.compiler import compile_step_diagram

        return compile_step_diagram(
            self, backend=backend, bind_params=bind_params, verbose=verbose
        )


###############################################################################
# Internal machinery
###############################################################################


def require_step_subsystem(sys, sys_id):
    """Refuse a :class:`DynamicSystem` with states: it has no ``step`` to stack."""
    if isinstance(sys, DynamicSystem) and sys.n > 0:
        raise TypeError(
            f"{type(sys).__name__} {sys_id!r} has continuous states and cannot "
            "join a StepDiagramSystem: keep it in a DiagramSystem and close the "
            "sampled loop with block % dt @ plant, or step it with "
            "discretize(plant, dt)"
        )
