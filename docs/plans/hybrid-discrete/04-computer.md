# Phase 4: Computer (clocked step side)

**After [Phase 2](02-step-diagram.md).** Introduces the **`Computer`** — the public name for the
clocked discrete side: a **`StepDiagramSystem`** plus **`StepSchedule`**, with a **`.tick()`**
runtime that fires sub-blocks on integer **`k`**.

**`StepSystem` leaves still see only `k`** — no wall time injected at the leaf.

**Before [Phase 5 hybrid](05-hybrid-simulation.md)** — hybrid sim **always** runs the step side
through **`Computer.tick`** (single-rate = trivial schedule).

**Files:** `minilink/simulation/computer.py` (`Computer`, `StepSchedule`, tick buffers)

**Supersedes** the working name `ScheduledStepOrchestrator` in earlier plan drafts.

## Split of concerns

| Component | Owns |
| --- | --- |
| **`Computer`** (`diagram` + `schedule` + runtime buffers) | **`dt_base`** grid; which blocks fire each **`k`**; intra–step-diagram hold buffers; **`.tick()`** |
| **`HybridSimulator`** ([Phase 5](05-hybrid-simulation.md)) | **One boundary** between computer and plant (multi-channel `connect_boundary` list); ZOH/sample; plant integration at **`t`** |
| **`StepSystem` leaf** | Pure **`step(x, u, k)`** — **`k` only**, no `dt` / `t` on class |

**`dt_base` is not time inside `StepSystem`.** It answers: “how often does the computer
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

- **`dt_base`** — grid for computer firing and (hybrid) plant ZOH interval.
- **`fire`** omitted or all divisors `1` — **single-rate** (Phase 5a hybrid).
- Non-trivial **`fire`** — multi-rate cascade (Phase 5b).

## `Computer`

Configuration **and** runtime for the discrete side. Bundles diagram + schedule so hybrid code
does not thread them separately.

```python
@dataclass
class Computer:
    diagram: StepDiagramSystem
    schedule: StepSchedule

    def compile(self, backend="numpy", *, bind_params=False, verbose=False): ...

    def tick(
        self,
        x,
        u,
        k: int,
        *,
        params=None,
    ) -> ComputerTickResult:
        """
        Advance one base tick on integer fire index k.

        Fires subsystems per schedule.fire; passes k into step/port eval.
        Returns updated stacked state, boundary outputs dict, optional buffer snapshot.
        """
```

### `.tick()` contract (frozen for Phase 4)

| Input | Role |
| --- | --- |
| **`x`** | Stacked step-diagram state, shape `(n,)` — same layout as `diagram.x0` |
| **`u`** | Step-diagram **boundary** input vector for this tick (world refs + plant samples already assembled by caller) |
| **`k`** | Integer fire index — **only** time coordinate on the step path |

| Output (`ComputerTickResult`) | Role |
| --- | --- |
| **`x_new`** | Stacked state after this tick |
| **`outputs`** | Boundary output dict (`port_id` → vector) from `evaluator.outputs` |
| **`buffers`** | Optional snapshot of cross-rate hold buffers (debug / tests) |

**Rules:**

- **Single-rate mode:** all blocks fire every tick; no cross-rate buffers needed (degenerate case).
- **Multi-rate mode:** slow consumer ← fast producer **sample**; fast ← slow **ZOH** (computer hold buffers).
- **Partial firing:** each tick runs only `sys_id`s with `k % fire[sys_id] == 0`; skipped blocks
  keep prior outputs in buffers. Uses per-block hooks from Phase 2 compile (`step_block` preview).
- **Step eval:** compiled `StepEvaluator.step(x, u, k)` and port ops receive **`int` `k`**.
- **Does not** integrate the continuous plant or read/write hybrid boundary ZOH/sample buffers —
  that is **`HybridSimulator`** only.

**Lazy compile:** first `.tick()` (or explicit `.compile()`) builds and caches the diagram
evaluator on the `Computer` instance.

### Standalone use (no hybrid)

```python
schedule = StepSchedule(dt_base=0.01, fire={"filter": 1, "mpc": 10})
computer = Computer(controller_diagram, schedule)
computer.compile()
x = computer.diagram.x0.copy()
k = 0
while k < N:
    u_k = ...  # boundary inputs at tick k
    result = computer.tick(x, u_k, k)
    x = result.x_new
    k += 1
```

`dt_base` may be used for **logging / real-time sleep** outside the library; it does not enter
`StepSystem.step`.

## Tests

- `test_computer.py`: trivial schedule; multi-rate `fire`; cross-rate buffers; verify eval called
  with **`k`** only; lazy compile idempotence.

## Deferred

- `expand_scheduled_step()` — optional lowering to hold blocks in diagram state.
