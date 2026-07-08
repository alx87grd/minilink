# Phase 2: Step Diagram + Evaluator

**After [Phase 1](01-step-core.md).** Compose `StepSystem` + `StaticSystem` blocks into
`StepDiagramSystem`; compile to `StepEvaluator`. Still **no wall clock** on the diagram —
`StepRunner` advances `k` without `dt`.

**Files:** `minilink/core/wiring.py`, `minilink/core/step_diagram.py`,
`minilink/core/compile/` (step path), `minilink/simulation/step_runner.py`

## Shared wiring mixin

Extract from `DiagramSystem`: `connect`, boundary ports, params nesting, `state_index` —
shared by flow and step diagrams.

## `StepDiagramSystem`

- Sibling of `DiagramSystem`.
- Stacked **`step(x, u, k)`** loop (same topology machinery as flow `f`).
- `compile_step_diagram()` → **`StepEvaluator.step(x, u, k)`**.
- Subsystems: `StepSystem`, `StaticSystem` only; reject `DynamicSystem` at compile.
- Reuse `ExecutionPlan`; do not fork wiring.

Mark compile path **`TODO: User Architectural Review`** until closed-loop tests pass.

## Run (clock-free or uniform grid)

### `StepRunner`

- Advance **`N`** steps by incrementing `k`; **no `tf`, no `dt`**.
- Turn-based games, unit tests, algorithmic stepping.

### `TimedStepSimulator` (Phase 2 stopgap)

- Single-rate closed loop: caller supplies `sync_dt` for **logging only**; inner loop calls
  **`StepEvaluator.step`** directly.
- **Replaced for clocked / multi-rate work** by `ScheduledStepOrchestrator` in
  [Phase 4](04-scheduled-orchestrator.md) (hybrid sim **always** uses Phase 4).

## Tests

- `test_step_diagram.py`: wiring; `@` closed loop.
- `test_step_runner.py`: clock-free stepping.
- `test_timed_step_simulator.py`: uniform grid via direct `StepEvaluator.step`.
