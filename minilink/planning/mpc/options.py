"""Workflow options for the beta :class:`~minilink.planning.mpc.planner.MPCPlanner`."""

from dataclasses import dataclass, field

from minilink.core.backends import BACKEND_JAX


@dataclass
class MPCOptions:
    """Beta MPC workflow options.

    ``step_disp`` prints per-step timing. Compile time is reported once at
    :meth:`~minilink.planning.mpc.planner.MPCPlanner.prepare`.
    """

    compile_backend: str = BACKEND_JAX
    optimizer_method: str = "scipy_slsqp"
    optimizer_options: dict[str, object] = field(default_factory=dict)
    record_solve_time: bool = True
    step_disp: bool = False
