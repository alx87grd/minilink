# Refactoring Compilation into Dedicated Backends

This plan aims to consolidate the fragmented and redundant compilation MVPs (`f_fast`, `f_fast_jax`, `compile_jax`, etc.) into a robust, backend-agnostic IR system with dedicated NumPy and JAX evaluators.

## User Review Required

> [!IMPORTANT]
> This refactor will significantly change the internal state of `DiagramSystem`. While standard methods like `.compile()` will remain for backwards compatibility, the inner attributes like `self.port_execution_plan` will be moved to the compiled artifact.

> [!WARNING]
> This change will enforce thread-safety by eliminating the shared `global_signals` buffer on `DiagramSystem`. Each simulation call will allocate its own signal buffer via the backend.

## Proposed Changes

### [MODIFY] [ir.py](file:///Users/alex/Code/minilink/minilink/compile/ir.py)
- Support `params` in `PortOp` and `StateOp`.
- Add metadata needed for both NumPy and JAX evaluation (e.g., input port index maps).

### [MODIFY] [api.py](file:///Users/alex/Code/minilink/minilink/compile/api.py)
- Rename to `compiler.py` or update to handle the full "lowering" logic stolen from `DiagramSystem`.
- Move `check_algebraic_loops` (topological sort) here.
- Functional implementation of `build_execution_plan`.

### [MODIFY] [numpy_backend.py](file:///Users/alex/Code/minilink/minilink/compile/numpy_backend.py)
- Consolidate the signal gathering logic (`src_type == 0/1/2`) into an internal helper.
- Ensure `eval_dx` and `eval_outputs` handle `params` and are fully stateless.

### [NEW] [jax_backend.py](file:///Users/alex/Code/minilink/minilink/compile/jax_backend.py)
- Port JAX-compatible evaluation logic from `DiagramSystem` into a `CompiledJaxDiagram` class.
- Uses `jax.numpy` and `.at[].set()` for traceability.
- Provides `eval_dx` and `eval_outputs` (jittable).

### [MODIFY] [diagram.py](file:///Users/alex/Code/minilink/minilink/core/diagram.py)
- **REMOVE** `check_algebraic_loops`, `build_execution_plan`, `f_fast`, `compile_jax`, `f_fast_jax`.
- Update `compile()` to call `minilink.compile.api.compile_numpy(self)` (or JAX equivalent) and return a compiled artifact.
- `f()` remains the slow, recursive default.

### [MODIFY] [analysis.py](file:///Users/alex/Code/minilink/minilink/core/analysis.py)
- Update `compute_internal_signals` to use the backend's `eval_outputs()`.
- Update `Simulator.solve()` to use the backend's `eval_dx()`.

## Verification Plan

### Automated Tests
- Run existing compilation and JAX tests: `pytest tests/manual/test_compiling.py`, `pytest tests/manual/test_jax_compile.py`.
- Compare final `dx` values between standard `f()` and new backends.

### Manual Verification
- Benchmark simulation speed to ensure no regressions from extra artifact overhead.
