# Phase 4: Scheduled Step Orchestrator

**After [Phase 2](02-step-diagram.md).** Introduces **`StepSchedule.dt_base`** for **when** to fire
step-diagram blocks and (in hybrid) the plant hold interval. **`StepSystem` leaves still see only
`k`** — no wall time injected at the leaf.

**Before [Phase 5 hybrid](05-hybrid-simulation.md)** — hybrid sim **always** delegates the step
side to this orchestrator (single-rate = trivial schedule).

**File:** `minilink/simulation/scheduled_step.py`

## Split of concerns

| Component | Owns |
| --- | --- |
| **`StepSchedule` + `ScheduledStepOrchestrator`** | **`dt_base`** (scheduling grid); which blocks fire each **`k`**; intra–step-diagram hold buffers |
| **`HybridSimulator`** ([Phase 5](05-hybrid-simulation.md)) | Step↔plant **boundary** ZOH/sample; continuous plant integration at **`t`** |
| **`StepSystem` leaf** | Pure **`step(x, u, k)`** — **`k` only**, no `dt` / `t` on class |

**`dt_base` is not time inside `StepSystem`.** It answers: “how often does the orchestrator
increment the fire index and (in hybrid) integrate the plant?” The diagram evaluator receives
integer **`k`**; `HybridSimulator` separately tracks **`t_k = t0 + k · dt_base`** for the
**continuous plant only**.

## `StepSchedule`

```python
@dataclass
class StepSchedule:
    dt_base: float
    fire: dict[str, int] = field(default_factory=dict)
    # sys_id -> divisor d; block fires when k % d == 0
    # empty fire => every block every tick (single-rate default)
```

- **`dt_base`** — grid for orchestrator firing and (hybrid) plant ZOH interval.
- **`fire`** omitted or all divisors `1` — **single-rate** (Phase 5a hybrid).
- Non-trivial **`fire`** — multi-rate cascade (Phase 5b).

## `ScheduledStepOrchestrator`

Runs a compiled **`StepDiagramSystem`** with internal buffers — **no graph expansion**.

```python
class ScheduledStepOrchestrator:
    def __init__(self, diagram, schedule: StepSchedule, *, compile_backend="numpy"): ...

    def tick(self, x_step, u_step, k) -> tuple[x_step, step_outputs, BufferSnapshot]:
        """Fire blocks per schedule; pass **k** into step/port eval; return boundary outputs."""
```

`tick` accepts **`k: int`** only on the step-diagram eval path. **`t` is not a parameter** —
orchestrator does not convert `k` to seconds for leaves.

When embedded in `HybridSimulator`, the simulator computes **`t_k`** for the plant and passes
**`k`** to `orchestrator.tick` ([Phase 5](05-hybrid-simulation.md)).

### Contract

- **Single-rate mode:** all blocks fire every tick; no cross-rate buffers needed (degenerate case).
- **Multi-rate mode:** slow consumer ← fast producer **sample**; fast ← slow **ZOH** (orchestrator buffers).
- **Partial firing:** each tick runs only `sys_id`s with `k % fire[sys_id] == 0`; skipped blocks
  keep prior outputs in orchestrator buffers. Uses per-block hooks from Phase 2 compile.
- **Step eval:** `StepEvaluator.step(x, u, k)` and port ops receive **`int` `k`**.
- Does **not** integrate the continuous plant or handle hybrid boundary ports.

## Standalone use (no hybrid)

```python
schedule = StepSchedule(dt_base=0.01, fire={"filter": 1, "mpc": 10})
orch = ScheduledStepOrchestrator(controller, schedule)
k = 0
while k < N:
    x_step, outs, _ = orch.tick(x_step, u_step, k)
    k += 1
```

`dt_base` may be used for **logging / real-time sleep** outside the library; it does not enter
`StepSystem.step`.

## Tests

- `test_scheduled_step_orchestrator.py`: trivial schedule; multi-rate `fire`; cross-rate buffers;
  verify eval called with **`k`** only.

## Deferred

- `expand_scheduled_step()` — optional lowering to hold blocks in diagram state.
