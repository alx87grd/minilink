# Phase 2: Step Diagram + Evaluator

**After [Phase 0](00-wiring-refactor.md) and [Phase 1](01-step-core.md).** Compose `StepSystem` +
`StaticSystem` blocks into `StepDiagramSystem`; compile to `StepEvaluator`. **No wall clock on the
diagram** — `StepRunner` advances **`k`** only; `dt_base` arrives in Phase 4 for orchestrator /
hybrid plant scheduling, not inside leaf `step`.

**Files:** `minilink/core/diagram.py` (add `StepDiagramSystem` beside `DiagramSystem`),
`minilink/core/compile/` (step path), `minilink/simulation/step_runner.py`

Wiring, gather, `tf`, and `check_algebraic_loops` come from [Phase 0](00-wiring-refactor.md)
**`WiredDiagramMixin`** — do not re-extract.

## `StepDiagramSystem`

- Same module as `DiagramSystem`: **`minilink/core/diagram.py`**.
- Composes **`WiredDiagramMixin`** (same `connect` / boundary ports / gather / viz as flow).
- Stacked **`step(x, u, k)`** loop (same topology machinery as flow `f`).
- `compile()` → **`compile_step_diagram`** → **`StepEvaluator.step(x, u, k)`**.
- Subsystems: `StepSystem`, `StaticSystem` only; reject `DynamicSystem` at compile.
- Reuse `PortOperation` gather recipes and topology — do not fork wiring.

Mark compile path **`TODO: User Architectural Review`** until closed-loop tests pass.

### Third coordinate slot on step diagrams

Shared mixin gather and `port.compute` use one Python third-argument position:

| Path | Passes | Type |
| --- | --- | --- |
| `StepDiagramSystem.step()` → `subsystem.step` | **`k`** | `int` |
| Port gather / `compute_subsys_output_port` | **`k`** | `int` |
| Mixin `tf` / `get_dynamic_geometry` | **`k`** | `int` |

**Never** pass float wall time into step diagram evaluation. **Never** convert `k` ↔ `t`.

Flow diagrams pass **`float` `t`** on the same code paths ([Phase 0](00-wiring-refactor.md)).

### `ExecutionPlan` and step advance

Flow `ExecutionPlan` has `StateOperation` with `f_func → dx`. Step compile **reuses**
`PortOperation` gather recipes and plan topology; state advance uses a parallel list
(e.g. `step_operations` with `step_func → x_new`) or a tagged evolution kind on state ops.
**Do not** overload `f` to mean `step` on `StepSystem` leaves.

`StepEvaluator.step`:

1. Walk **`port_ops`** — pass **`k`** into each `compute_func`.
2. Apply **`step_ops`** — pass **`k`** into each `step_func` → write `x_new`.

Order: port signals before state advance (same as flow: ports before `f`).

### Evaluator backends

| Backend | Public API | Third slot in JIT |
| --- | --- | --- |
| `NumpyStepEvaluator` | `.step(x, u, k)` | `int` `k` |
| `JaxStepEvaluator` | `.step(x, u, k)` | `int` `k` — **separate** JIT from `JaxDiagramEvaluator` |

**JAX:** do not share one `jax.jit` between flow (`t` float) and step (`k` int). Do not
`float(k)` for tracing. Blocks that branch on `k` follow the same traceability rules as `t` on
the flow side.

### Partial firing (Phase 4 preview)

`ScheduledStepOrchestrator` fires a **subset** of blocks per tick. Phase 2 compile should
expose per-`sys_id` step/port op ranges (or callable single-block step) so Phase 4 does not
re-compile per mask. Implementation completes in
[Phase 4](04-scheduled-orchestrator.md).

## Run (clock-free or orchestrated)

### `StepRunner`

- Advance **`N`** steps by incrementing **`k`**; **no `tf`, no `dt`**.
- Turn-based games, unit tests, algorithmic stepping.

### `TimedStepSimulator` (Phase 2 stopgap)

- Caller may supply `sync_dt` for **logging only**; inner loop calls **`StepEvaluator.step`**
  with **`k`**.
- **Replaced for clocked / multi-rate work** by `ScheduledStepOrchestrator` in
  [Phase 4](04-scheduled-orchestrator.md).
- **Do not** document in README until Phase 4 lands (or mark deprecated immediately).

## Composition shortcuts

`minilink/core/composition.py` is `DiagramSystem`-only today. Phase 2 tests use explicit
`connect` / `add_subsystem`. Extend `@` / `>>` for `StepDiagramSystem` in Phase 2 **only if**
low cost; otherwise defer — not a gate for Phase 4.

## Tests

- `test_step_diagram.py`: wiring; closed loop via explicit `connect`; gather passes **`k`**.
- `test_step_runner.py`: clock-free stepping with integer **`k`**.
- `test_timed_step_simulator.py`: uniform grid via direct `StepEvaluator.step`.
- `test_step_evaluator_jax.py` (if JAX step blocks): step JIT uses **`int` `k`** only.
