"""
Step execution plan data structures for compiled step diagrams.

Parallel to :mod:`minilink.core.compile.execution_plan` for flow diagrams — same
port topology, ``step_ops`` instead of ``state_ops``.
"""

from collections.abc import Callable
from dataclasses import dataclass

from minilink.core.compile.execution_plan import PortOperation


@dataclass(frozen=True)
class StepOperation:
    """One discrete state-advance step in a compiled step diagram.

      Calls ``step(x, u, k, params) -> x_new`` and writes the result into the
    global state vector at ``local_x_slice``.
    """

    step_func: Callable[..., object]
    local_x_slice: slice
    gather_sources: tuple[tuple[int, object, int], ...]
    u_dim: int
    bound_params: dict | None = None
    label: str = ""
    sys_id: str = ""


@dataclass(frozen=True)
class StepExecutionPlan:
    """Immutable flattened schedule for a compiled step diagram."""

    state_dim: int
    signal_dim: int
    port_ops: tuple[PortOperation, ...]
    step_ops: tuple[StepOperation, ...]
    output_slices: dict[tuple[str, str], slice]
    external_output_slices: dict[str, slice]
