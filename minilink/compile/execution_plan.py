"""
Execution plan data structures for compiled diagrams.

An :class:`ExecutionPlan` is the output of the compilation step: it captures,
in an immutable and flattened form, all the information needed to evaluate a
:class:`~minilink.core.diagram.DiagramSystem` without any dictionary lookups
or recursive calls.

The plan is consumed by evaluator backends
(:class:`~minilink.compile.numpy_evaluator.NumpyDiagramEvaluator`,
:class:`~minilink.compile.jax_evaluator.JaxDiagramEvaluator`) which walk through the
operation lists in topological order to compute state derivatives and outputs.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


# ── Source-type constants ────────────────────────────────────────────
# Used in the ``gather_sources`` tuples of PortOperation / StateOperation.

NOMINAL = 0
"""Source is a constant value (port not connected, or not a dependency)."""

EXTERNAL_INPUT = 1
"""Source is an external input to the diagram (from the diagram's ``u`` vector)."""

INTERNAL_SIGNAL = 2
"""Source is an output from another subsystem (from the internal signal buffer)."""


# ── Operation dataclasses ────────────────────────────────────────────


@dataclass(frozen=True)
class PortOperation:
    """One output-port evaluation step in topological order.

    When a diagram is compiled, every output port becomes a PortOperation.
    During evaluation, operations execute sequentially in topological order:
    for each operation, gather the local input vector ``u`` from the signal
    buffer, call the compute function, and write the result into the buffer.

    Attributes
    ----------
    compute_func : Callable[..., np.ndarray]
        The port's compute function ``(x, u, t, params) → y``.
        ``params`` may be ``None`` (use live ``self.params`` in the block) or a
        bound snapshot when ``bind_params=True`` at compile time.
    local_x_slice : slice
        Index slice into the global state vector ``x`` that extracts this
        subsystem's local state.  For static systems (``n = 0``): ``slice(0, 0)``.
    gather_sources : tuple[tuple[int, Any, int], ...]
        Recipe for assembling the local input vector ``u`` from various sources.
        Each entry is ``(source_type, source_value, dim)`` where:

        - ``source_type = NOMINAL (0)``:
          Port not connected or not a dependency.
          ``source_value`` is the constant ``np.ndarray`` (nominal value).
        - ``source_type = EXTERNAL_INPUT (1)``:
          From the diagram's external input vector.
          ``source_value`` is a ``slice`` into the diagram's ``u``.
        - ``source_type = INTERNAL_SIGNAL (2)``:
          From another subsystem's output.
          ``source_value`` is a ``slice`` into the internal signal buffer.
    out_slice : slice
        Where to write this port's output in the internal signal buffer.
    u_dim : int
        Total dimension of the assembled local input vector.
        Equal to the sum of all ``dim`` values in ``gather_sources``.
    bound_params : dict | None
        If set, a deep copy of the subsystem ``params`` at compile time; passed
        as the fourth argument to ``compute_func``. If ``None``, evaluators pass
        ``None`` so blocks use ``params or self.params``. This does not freeze
        other instance attributes; purity of ``compute_func`` is not enforced.
    """

    compute_func: Callable[..., np.ndarray]
    local_x_slice: slice
    gather_sources: tuple[tuple[int, Any, int], ...]
    out_slice: slice
    u_dim: int
    bound_params: dict | None = None
    label: str = ""


@dataclass(frozen=True)
class StateOperation:
    """One state-derivative evaluation step.

    Similar to :class:`PortOperation`, but calls ``f(x, u, t) → dx`` instead
    of ``h(x, u, t) → y``.  The result is written into the global derivative
    vector ``dx`` at ``local_x_slice``.

    Attributes
    ----------
    f_func : Callable[..., np.ndarray]
        The subsystem's state-derivative function ``(x, u, t, params) → dx``.
        ``params`` may be ``None`` or a bound snapshot (see :attr:`bound_params`).
    local_x_slice : slice
        Index slice into the global state vector ``x`` and derivative vector
        ``dx`` for this subsystem's local state.
    gather_sources : tuple[tuple[int, Any, int], ...]
        Recipe for assembling the local input vector ``u``.
        Same format as :attr:`PortOperation.gather_sources` — the ``f``
        function always needs **all** inputs (no dependency filtering).
    u_dim : int
        Total dimension of the assembled local input vector.
    bound_params : dict | None
        Same semantics as :attr:`PortOperation.bound_params` for ``f_func``
        (``params`` dict snapshot only; see :class:`minilink.core.system.System`).
    """

    f_func: Callable[..., np.ndarray]
    local_x_slice: slice
    gather_sources: tuple[tuple[int, Any, int], ...]
    u_dim: int
    bound_params: dict | None = None
    label: str = ""


# ── Execution plan ───────────────────────────────────────────────────


@dataclass(frozen=True)
class ExecutionPlan:
    """Immutable flattened execution schedule for a compiled diagram.

    Built by :func:`~minilink.compile.compiler.build_execution_plan`.
    Consumed by evaluator backends to compute state derivatives and outputs
    without any dictionary lookups or recursive traversals.

    Attributes
    ----------
    state_dim : int
        Total state dimension of the diagram (sum of all subsystem ``n`` values).
    signal_dim : int
        Total size of the internal signal buffer (sum of all output port
        dimensions across all subsystems).
    port_ops : tuple[PortOperation, ...]
        Output-port operations in topological order.  Evaluating them
        sequentially fills the internal signal buffer.
    state_ops : tuple[StateOperation, ...]
        State-derivative operations.  Evaluating them sequentially (after
        the port operations) fills the derivative vector ``dx``.
    output_slices : dict[tuple[str, str], slice]
        Map from ``(subsystem_id, port_id)`` to the corresponding ``slice``
        in the internal signal buffer.  Used to extract specific port outputs
        after evaluation.
    external_output_slices : dict[str, slice]
        Diagram **boundary** outputs only: maps diagram output port id (keys of
        :attr:`~minilink.core.diagram.DiagramSystem.outputs` on the diagram) to
        the slice in the internal buffer that holds the connected source
        subsystem port's signal. Empty when the diagram has no external outputs.
    """

    state_dim: int
    signal_dim: int
    port_ops: tuple[PortOperation, ...]
    state_ops: tuple[StateOperation, ...]
    output_slices: dict[tuple[str, str], slice]
    external_output_slices: dict[str, slice]
