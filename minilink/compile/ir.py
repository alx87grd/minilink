from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class PortOp:
    compute_func: Callable
    local_x_slice: slice
    gather_sources: tuple[tuple[int, Any, int], ...]
    out_slice: slice
    u_dim: int


@dataclass(frozen=True)
class StateOp:
    f_func: Callable
    local_x_slice: slice
    gather_sources: tuple[tuple[int, Any, int], ...]
    u_dim: int


@dataclass(frozen=True)
class DiagramIR:
    state_dim: int
    output_signal_dim: int
    port_execution_order: tuple[tuple[str, str], ...]
    output_slices: dict[tuple[str, str], slice]
    port_plan: tuple[PortOp, ...]
    state_plan: tuple[StateOp, ...]
