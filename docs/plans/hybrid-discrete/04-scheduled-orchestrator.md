# Phase 4: Scheduled Step Orchestrator

**After [Phase 2](02-step-diagram.md).** Introduces **physical sample time** and multi-rate firing
for step diagrams. Pure `step` blocks stay clock-free; **`StepSchedule.dt_base`** is the grid.

**Before [Phase 5 hybrid](05-hybrid-simulation.md)** — hybrid sim **always** delegates the step
side to this orchestrator (single-rate = trivial schedule).

**File:** `minilink/simulation/scheduled_step.py`

## Split of concerns

| Component | Owns |
| --- | --- |
| **`StepSchedule` + `ScheduledStepOrchestrator`** | Sample time **`dt_base`**; which blocks fire each tick; **intra–step-diagram** hold buffers (cross-rate) |
| **`HybridSimulator`** ([Phase 5](05-hybrid-simulation.md)) | Step↔plant **boundary** ZOH/sample; continuous plant integration |
| **`StepSystem` leaf** | Pure `step(x, u, k)` only |

## `StepSchedule`

```python
@dataclass
class StepSchedule:
    dt_base: float
    fire: dict[str, int] = field(default_factory=dict)
    # sys_id -> divisor d; block fires when k % d == 0
    # empty fire => every block every tick (single-rate default)
```

- **`dt_base`** — authoritative sample period for orchestrated stepping (`t_k = t0 + k · dt_base`).
- **`fire`** omitted or all divisors `1` — **single-rate** (Phase 5a hybrid, simple discrete sim).
- Non-trivial **`fire`** — multi-rate cascade (Phase 5b hybrid demo, standalone discrete multi-rate).

## `ScheduledStepOrchestrator`

Runs a compiled **`StepDiagramSystem`** at `dt_base` with internal buffers — **no graph expansion**.

```python
class ScheduledStepOrchestrator:
    def __init__(self, diagram, schedule: StepSchedule, *, compile_backend="numpy"): ...

    def tick(self, x_step, u_step, t, k) -> tuple[x_step, step_outputs, BufferSnapshot]:
        """Fire blocks per schedule; update cross-rate buffers; return boundary outputs."""

    def buffer_history(self) -> dict[str, np.ndarray]: ...
```

### Contract

- **Single-rate mode:** all blocks fire every tick; no cross-rate buffers needed (degenerate case).
- **Multi-rate mode:** slow consumer ← fast producer **sample**; fast ← slow **ZOH** (orchestrator buffers).
- **Public API** for clocked step diagrams — standalone or embedded in `HybridSimulator`.
- Does **not** integrate the continuous plant or handle hybrid boundary ports.

## Standalone use (no hybrid)

```python
schedule = StepSchedule(dt_base=0.01, fire={"filter": 1, "mpc": 10})
orch = ScheduledStepOrchestrator(controller, schedule)
# closed-loop discrete sim @ dt_base
```

## Tests

- `test_scheduled_step_orchestrator.py`: trivial schedule (all fire); multi-rate fire mask;
  cross-rate buffers; cascade parity.

## Deferred

- `expand_scheduled_step()` — optional lowering to hold blocks in diagram state.
