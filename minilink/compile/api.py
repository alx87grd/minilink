from minilink.compile.ir import DiagramIR, PortOp, StateOp
from minilink.compile.numpy_backend import CompiledNumpyDiagram


def lower_diagram_to_ir(diagram) -> DiagramIR:
    """
    Lower a DiagramSystem to backend-agnostic IR.

    Prototype implementation:
    - Reuses the current in-class compilation flow for correctness parity.
    - Extracts immutable plans so external backends can evaluate without using
      DiagramSystem's mutable buffers.
    """
    if not diagram.compiled:
        diagram.compile()

    port_plan = []
    for compute_func, local_x_slice, gather_sources, out_slice, u_dim in diagram.port_execution_plan:
        port_plan.append(
            PortOp(
                compute_func=compute_func,
                local_x_slice=local_x_slice,
                gather_sources=tuple(gather_sources),
                out_slice=out_slice,
                u_dim=u_dim,
            )
        )

    state_plan = []
    for f_func, local_x_slice, gather_sources, u_dim in diagram.state_execution_plan:
        state_plan.append(
            StateOp(
                f_func=f_func,
                local_x_slice=local_x_slice,
                gather_sources=tuple(gather_sources),
                u_dim=u_dim,
            )
        )

    return DiagramIR(
        state_dim=diagram.n,
        output_signal_dim=diagram.global_signals.shape[0],
        port_execution_order=tuple(diagram.port_execution_order),
        output_slices=dict(diagram.output_slices),
        port_plan=tuple(port_plan),
        state_plan=tuple(state_plan),
    )


def compile_numpy(diagram) -> CompiledNumpyDiagram:
    """
    Compile a DiagramSystem into an external NumPy compiled artifact.
    """
    ir = lower_diagram_to_ir(diagram)
    return CompiledNumpyDiagram(ir=ir)
