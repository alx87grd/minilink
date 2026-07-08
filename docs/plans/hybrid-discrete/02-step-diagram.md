# Phase 2: Step Diagram + Evaluator

**After [Phase 0](00-wiring-refactor.md) and [Phase 1](01-step-core.md).** Compose `StepSystem` +
`StaticSystem` blocks into `StepDiagramSystem`; compile to `StepEvaluator`. Still **no wall clock**
on the diagram — `StepRunner` advances `k` without `dt`.

**Files:** `minilink/core/step_diagram.py`, `minilink/core/compile/` (step path),
`minilink/simulation/step_runner.py`

Wiring comes from [Phase 0](00-wiring-refactor.md) **`WiredDiagramMixin`** — do not re-extract here.
## `StepDiagramSystem`

- Composes **`WiredDiagramMixin`** from Phase 0 (same `connect` / boundary ports as flow diagrams).
- Stacked **`step(x, u, k)`** loop (same topology machinery as flow `f`).
- `compile_step_diagram()` → **`StepEvaluator.step(x, u, k)`**.
- Subsystems: `StepSystem`, `StaticSystem` only; reject `DynamicSystem` at compile.
- Reuse port-gather / `ExecutionPlan` topology — do not fork wiring.

Mark compile path **`TODO: User Architectural Review`** until closed-loop tests pass.

### `ExecutionPlan` and step advance

Flow `ExecutionPlan` today has `StateOperation` with `f_func → dx`. Step compile **reuses**
`PortOperation` gather recipes and plan topology, but state advance uses a parallel list
(e.g. `step_operations` with `step_func → x_new`) or a tagged evolution kind on state ops.
**Do not** overload `f` to mean `step` on `StepSystem` leaves.

`StepEvaluator.step` walks port ops then applies step ops in plan order, writing the next
stacked state `x_new` (not a derivative buffer).

### Partial firing (Phase 4 preview)

`ScheduledStepOrchestrator` fires a **subset** of blocks per tick. Phase 2 compile should
expose per-`sys_id` step/port op ranges (or callable single-block step) so Phase 4 does not
re-compile per mask. Document the hook here; implementation completes in
[Phase 4](04-scheduled-orchestrator.md).

## Run (clock-free or uniform grid)

### `StepRunner`

- Advance **`N`** steps by incrementing `k`; **no `tf`, no `dt`**.
- Turn-based games, unit tests, algorithmic stepping.

### `TimedStepSimulator` (Phase 2 stopgap)

- Single-rate closed loop: caller supplies `sync_dt` for **logging only**; inner loop calls
  **`StepEvaluator.step`** directly.
- **Replaced for clocked / multi-rate work** by `ScheduledStepOrchestrator` in
  [Phase 4](04-scheduled-orchestrator.md) (hybrid sim **always** uses Phase 4).
- **Do not** document in README until Phase 4 lands (or mark deprecated immediately).

## Composition shortcuts

`minilink/core/composition.py` is `DiagramSystem`-only today. Phase 2 tests use explicit
`connect` / `add_subsystem`. Extend `@` / `>>` for `StepDiagramSystem` in Phase 2 **only if**
low cost; otherwise defer — not a gate for Phase 4.

## Tests

- `test_step_diagram.py`: wiring; closed loop via explicit `connect` (and `@` if extended).
- `test_step_runner.py`: clock-free stepping.
- `test_timed_step_simulator.py`: uniform grid via direct `StepEvaluator.step`.
