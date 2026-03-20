from dataclasses import dataclass

import numpy as np

from minilink.compile.ir import DiagramIR


@dataclass
class CompiledNumpyDiagram:
    ir: DiagramIR

    def eval_dx(self, x: np.ndarray, u: np.ndarray, t: float = 0.0) -> np.ndarray:
        """
        Evaluate the diagram state derivative using NumPy.
        """
        global_signals = np.zeros(self.ir.output_signal_dim)

        # 1) Evaluate all output-port ops in topological order.
        for op in self.ir.port_plan:
            local_x = x[op.local_x_slice]
            if op.u_dim == 0:
                local_u = np.array([])
            else:
                local_u = np.empty(op.u_dim)
                idx = 0
                for src_type, src_val, dim in op.gather_sources:
                    if src_type == 2:
                        local_u[idx : idx + dim] = global_signals[src_val]
                    elif src_type == 0:
                        local_u[idx : idx + dim] = src_val
                    elif src_type == 1:
                        local_u[idx : idx + dim] = u[src_val]
                    else:
                        raise RuntimeError(f"Unknown source_type={src_type}")
                    idx += dim
            global_signals[op.out_slice] = op.compute_func(local_x, local_u, t)

        # 2) Evaluate all state derivative ops.
        dx = np.zeros(self.ir.state_dim)
        for op in self.ir.state_plan:
            local_x = x[op.local_x_slice]
            if op.u_dim == 0:
                local_u = np.array([])
            else:
                local_u = np.empty(op.u_dim)
                idx = 0
                for src_type, src_val, dim in op.gather_sources:
                    if src_type == 2:
                        local_u[idx : idx + dim] = global_signals[src_val]
                    elif src_type == 0:
                        local_u[idx : idx + dim] = src_val
                    elif src_type == 1:
                        local_u[idx : idx + dim] = u[src_val]
                    else:
                        raise RuntimeError(f"Unknown source_type={src_type}")
                    idx += dim
            dx[op.local_x_slice] = op.f_func(local_x, local_u, t)

        return dx

    def eval_outputs(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0, output_ports=None
    ) -> np.ndarray:
        """
        Evaluate selected subsystem output ports and return their concatenation.

        If `output_ports` is None, all subsystem output ports are returned in the
        same order as they appear in `ir.output_slices`.
        """
        global_signals = np.zeros(self.ir.output_signal_dim)

        for op in self.ir.port_plan:
            local_x = x[op.local_x_slice]
            if op.u_dim == 0:
                local_u = np.array([])
            else:
                local_u = np.empty(op.u_dim)
                idx = 0
                for src_type, src_val, dim in op.gather_sources:
                    if src_type == 2:
                        local_u[idx : idx + dim] = global_signals[src_val]
                    elif src_type == 0:
                        local_u[idx : idx + dim] = src_val
                    elif src_type == 1:
                        local_u[idx : idx + dim] = u[src_val]
                    else:
                        raise RuntimeError(f"Unknown source_type={src_type}")
                    idx += dim
            global_signals[op.out_slice] = op.compute_func(local_x, local_u, t)

        if output_ports is None:
            slices = list(self.ir.output_slices.values())
        else:
            slices = [self.ir.output_slices[port_key] for port_key in output_ports]

        if not slices:
            return np.array([])
        return np.concatenate([global_signals[s] for s in slices])
