# Layer 2: Scheduled Orchestrator

Introduces multi-rate physical time to the pure discrete domain via orchestration, removing the need to mangle diagrams with hold blocks.

## `StepSchedule`

**File:** `minilink/simulation/scheduled_step.py`

```python
@dataclass
class StepSchedule:
    dt_base: float
    fire: dict[str, int]   # sys_id -> tick divisor; default 1 for every block
```

### Contract
- Maps system IDs to their firing frequency (divisor of the base tick).
- `None` implies all blocks fire every base tick (Phase A).

## `ScheduledStepOrchestrator`

Runs a **logical** compiled step diagram at `dt_base` using native buffers instead of diagram expansion.

```python
class ScheduledStepOrchestrator:
    def __init__(self, diagram, schedule: StepSchedule, *, compile_backend="numpy"): ...

    def tick(self, x_step, u_ext, t, k) -> tuple[x_step, u_plant, BufferSnapshot]: ...
    """Fire blocks per schedule.fire; update hold/latch buffers; return plant input."""

    def buffer_history(self) -> dict[str, np.ndarray]: ...  # debug / plot internal holds
```

### Contract
- **No Diagram Expansion:** Uses compiled port wiring and per-block `phi` callables on the pure `StepDiagramSystem`.
- **ZOH implicitly provided:** If a downstream block runs at a faster rate than its upstream source, the orchestrator supplies the last computed output from its internal buffer.
- **Public API:** `ScheduledStepOrchestrator` is the primary, public mechanism for multi-rate loops.

## Optional Lowering (Out of Scope for initial release)
- `expand_scheduled_step` (inserting explicit `RateGate` and `SampleLatch` subsystems) is deferred as an optional lowering feature for later phases if explicitly requested.

### Tests
- `test_scheduled_step_orchestrator.py`: Phase B fire mask, ZOH/sample buffers, and cascade performance.\n