# Phase 4: Computer (scheduled discrete simulation)

**Status: Done** (July 2026).

**After [Phase 2](02-step-diagram.md).** Introduces the **`Computer`** — a **simulation-only**
runtime for a **`StepDiagramSystem`** topology under a **firing schedule**.

**`StepSystem` leaves still see only integer `k`** (supplied internally by the Computer) — no wall
time on the leaf.

**Before [Phase 5 hybrid](05-hybrid-simulation.md)** — hybrid sim **always** runs the discrete side
through **`Computer.tick`**.

**Files:** `minilink/simulation/computer.py` (`Computer`, `StepSchedule`, tick buffers)

**Supersedes** the working name `ScheduledStepOrchestrator` in earlier plan drafts.

## StepDiagram: topology shared by two pipelines

`StepDiagramSystem` is **wiring + synchronous discrete semantics** — not the scheduled runtime.

| Pipeline | Runtime | Semantics | Primary use |
| --- | --- | --- | --- |
| **A — Synchronous** | `compile()` → `StepDiagramEvaluator` / **`compute_rollout`** | Every block fires every index; stateless `step(x, u, k)` | Teaching, logistic map, ZOH chains, parity tests |
| **B — Scheduled** | **`Computer`** (stateful `.tick(u)`) | Subset fire per base tick; internal signal buffers | Clocked discrete sim, hybrid step side |

Both pipelines compile the **same** [`StepExecutionPlan`](02-step-diagram.md) (topology, gather
slices, per-subsystem `step_func` / port ops). **Do not** duplicate wiring logic in Computer.

**Important:** when `schedule.fire` is non-trivial, **`compute_rollout` ≠ `Computer`** — rollout
assumes synchronous firing; Computer implements the scheduled dynamics. Document this at call sites;
do not expect multi-rate diagram rollouts to match hybrid sim.

Computer is **not** a core `System` type — no analysis path (linearize, etc.) in Phase 4. It lives
in `minilink/simulation/` beside `Simulator`.

## Split of concerns

| Component | Owns |
| --- | --- |
| **`Computer`** | Firing schedule; stacked **`x`**; **double signal buffers**; internal tick index **`k`**; **`.tick(u)`** |
| **`HybridSimulator`** ([Phase 5](05-hybrid-simulation.md)) | `t_k = t0 + k · dt_base`; hybrid boundary ZOH/sample; plant `integrate_zoh_rollout` |
| **`StepSystem` leaf** | Pure **`step(x, u, k)`** — **`k` only**, no `dt` / `t` on class |

**`dt_base` on `StepSchedule`** is metadata for hybrid time alignment and user-facing Hz helpers —
**not** used inside `Computer.tick()` math. Computer is fully discrete; HybridSimulator maps
**`computer.k` → seconds**.

## `StepSchedule`

Low-level contract (integer divisors on base tick index):

```python
@dataclass
class StepSchedule:
    dt_base: float
    fire: dict[str, int] = field(default_factory=dict)
    # sys_id -> divisor d; block fires when k % d == 0
    # empty fire => every block every tick (single-rate default)
```

- **`dt_base`** — base tick interval [s] for hybrid plant holds and logging (required for
  `HybridDiagram`; optional for step-only Computer tests).
- **`fire[sys_id] = d`** — block fires when internal **`k % d == 0`**.
- Empty **`fire`** or all divisors **`1`** — single-rate (Phase 5a).

### Hz helper (user-facing layer)

Avoid hand-computing divisors. Example API (exact names TBD):

```python
StepSchedule.from_rates(
    dt_base=0.01,           # 100 Hz base tick
    rates_hz={"filter": 100, "mpc": 10},   # filter every tick, mpc every 10th
)
# equivalent to fire={"filter": 1, "mpc": 10}
```

Rules: each rate must be an **integer divisor** of the base rate `1/dt_base` (v1 — same integer
multi-rate subset as master plan). Invalid combinations raise at construction.

Static blocks (`n=0`) use the same **`sys_id`** entry in **`fire`**; when fired, run port compute
only.

## `Computer`

Configuration **and** stateful runtime. Bundles diagram + schedule.

```python
class Computer:
    diagram: StepDiagramSystem
    schedule: StepSchedule

    def compile(self, backend="numpy", *, bind_params=False, verbose=False): ...

    def reset(self, x0=None) -> None:
        """k <- 0; copy x0; initialize signal buffers from diagram nominals."""

    def tick(self, u) -> dict[str, np.ndarray]:
        """
        Advance one base tick. Only argument: boundary input u (diagram boundary layout).

        Mutates internal x and buffers. Returns boundary output dict (port_id -> vector).
        """
```

Read-only **`computer.k`** — internal fire index used at the **start** of the current tick (passed
to leaf `step(..., k)`); incremented at end of each successful **`tick`**.

**No `k` argument on `tick`.** Replay of tick 17 requires **`reset()` + 17 ticks** or a full state
snapshot — not an external `k`.

### Double-buffer tick (parallel firing semantics)

Within one base tick, treat all subsystems that fire as **parallel** (order-agnostic):

1. **Read buffer** ← committed signal state at tick start.
2. Inject boundary **`u`** into read buffer at diagram input slots.
3. **Write buffer** ← copy of read buffer (so non-fired slots carry forward without explicit hold masks).
4. For each **`sys_id`** with **`k % fire[sys_id] == 0`**: gather inputs from **read buffer** only;
   run port ops / **`step_func`**; write outputs and updated state slices into **write buffer** and
   stacked **`x`**.
5. **Commit:** swap read ↔ write (or copy write → read).

Skipped blocks leave write-buffer slots unchanged from the copy in step 3 — natural **hold** for
multi-rate. No separate “hold mask” pass.

### `.tick(u)` contract (frozen for Phase 4)

| | |
| --- | --- |
| **Input `u`** | Step-diagram **boundary** input vector (world refs + plant samples assembled by caller) |
| **Returns** | `dict[str, np.ndarray]` boundary outputs |
| **Internal** | Updates **`x`**, signal buffers, then **`k += 1`** |
| **Leaf `k`** | **`self.k`** at tick start — not caller-supplied |
| **Plant / hybrid** | **Out of scope** — `HybridSimulator` injects **`u`** and reads outputs |

**Compile:** **`build_step_execution_plan(diagram)`** + per-block funcs from Phase 2 — **not**
monolithic **`StepDiagramEvaluator.step`** as the scheduled runtime loop.

**Backend:** NumPy v1; JAX deferred.

### Standalone use (no hybrid)

```python
computer = Computer(controller_diagram, StepSchedule.from_rates(dt_base=0.01, rates_hz={...}))
computer.compile()
computer.reset()
while computer.k < N:
    u_k = input_fn(computer.k)
    outs = computer.tick(u_k)
```

## Tests

- `test_computer.py`: single-rate; multi-rate **`fire`** / **`from_rates`**; double-buffer hold
  (fast/slow parity vs hand loop); internal **`k`** increments; leaf receives **`k`** not **`t`**;
  **`compute_rollout` ≠ Computer** smoke when **`fire`** non-trivial.

## Deferred

- `expand_scheduled_step()` — optional lowering to hold blocks in diagram state.
- Checkpoints / **`load_snapshot`** for mid-run replay.
- JAX Computer runtime.
