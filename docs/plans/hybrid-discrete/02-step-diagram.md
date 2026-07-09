# Phase 2: Step Diagram + Evaluator

**After [Phase 0](00-wiring-refactor.md), [Phase 1](01-step-core.md), and
[Phase 1b](01b-facade-mixin-split.md).** Compose `StepSystem` + static `System` (`n=0`)
blocks into `StepDiagramSystem`; compile to diagram **`StepEvaluator`**. **No wall
clock on the diagram** — reuse Phase 1 **`StepEvaluator.rollout`** and
**`compute_rollout`** (Phase 1b façades); `dt_base` arrives in Phase 4 for orchestrator /
hybrid plant scheduling, not inside leaf `step`.

**Files:** `minilink/core/diagram.py`, `minilink/core/compile/step_execution_plan.py` (new),
`minilink/core/compile/step_compiler.py` (new),
`minilink/core/compile/evaluators/step_diagram_evaluator.py` (new),
`minilink/simulation/timed_step_simulator.py` (optional test stopgap)

Wiring, gather, `tf`, and `check_algebraic_loops` come from [Phase 0](00-wiring-refactor.md)
**`WiredDiagramMixin`** — do not re-extract. Leaf compile + `rollout()` live in
[Phase 1](01-step-core.md) — do not duplicate.

## `StepDiagramSystem`

- Same module as `DiagramSystem`: **`minilink/core/diagram.py`**.
- Composes **`WiredDiagramMixin`** (same `connect` / boundary ports / gather / viz as flow).
- Inherits **`StepSystem`** — mirror **`DiagramSystem(DynamicSystem)`** from Phase 1b (IS-A for
  MRO / `compute_rollout`, not for leaf `__init__` boilerplate).
- Stacked **`step(x, u, k)`** reference loop (same topology machinery as flow `f`).
- `compile()` → **`compile_step_diagram`** → diagram **`StepEvaluator`** with `.step`, `.outputs`,
  `.rollout`.
- Subsystems: `StepSystem`, static `System` (`n=0`) only; reject `DynamicSystem` at compile.
- Static-only step diagrams (`n=0`, no `StepSystem` state) are allowed — signal-flow only, empty
  `step_ops` (same pattern as static-only flow diagrams).

Mark compile path **`TODO: User Architectural Review`** until closed-loop tests pass.

### Init pattern (mirror Phase 1b)

`StepSystem` requires `n >= 1` at construction; diagram `n` is rebuilt by
`compute_state_properties()` after wiring. **Skip leaf `StepSystem.__init__`:**

```python
class StepDiagramSystem(WiredDiagramMixin, StepSystem):
    def __init__(self):
        self.subsystems = {}
        self.connections = {}
        System.__init__(self, 0)
        self._init_wiring(name="StepDiagram")
```

### Interpreted reference `step(x, u, k)`

Mirror `DiagramSystem.f()`:

1. For each `StepSystem` subsystem: gather `local_x`, `local_u` with third slot **`k`**.
2. `x_new_local = subsystem.step(local_x, local_u, k, params)`.
3. Write into stacked `x_new` at `state_index[sys_id]` (step returns **next state**, not `dx`).
4. Static subsystems (`n=0`): port ops only; no entry in `step_ops`.

### Third coordinate slot on step diagrams

Shared mixin gather and `port.compute` use one Python third-argument position:

| Path | Passes | Type |
| --- | --- | --- |
| `StepDiagramSystem.step()` → `subsystem.step` | **`k`** | `int` |
| Port gather / `compute_subsys_output_port` | **`k`** | `int` |
| Mixin `tf` / `get_dynamic_geometry` | **`k`** | `int` |

**Never** pass float wall time into step diagram evaluation. **Never** convert `k` ↔ `t`.

Flow diagrams pass **`float` `t`** on the same code paths ([Phase 0](00-wiring-refactor.md)).

### User API (Phase 1b façades)

| Verb | On `StepDiagramSystem` | Engine |
| --- | --- | --- |
| **`compute_rollout(n_steps, u=...)`** | yes — via `StepSystemFacades` (MRO) | `compile()` → `StepEvaluator.rollout` |
| **`plot_rollout()`** | yes | `StepRollout.as_trajectory()` |
| `compute_trajectory` | not the step API — misuse hits `StaticSimulator` rejection | — |

Closed-loop demos and tests should prefer **`compute_rollout`** or **`compile().rollout(...)`** —
no new runner class.

## Execution plan — parallel pipeline (decision)

**Verdict: build a sibling step compile path; do not extend flow `ExecutionPlan` / `StateOperation`.**

| Approach | Continuous path | Step path | Tradeoff |
| --- | --- | --- | --- |
| **Parallel (adopt)** | `ExecutionPlan` + `state_ops` (`f_func → dx`) unchanged | **`StepExecutionPlan`** + `step_ops` (`step_func → x_new`) | Zero risk to flow sim / trajopt / MPC on `f`; clear `k` vs `t` |
| **Merged plan** | One `ExecutionPlan` with both `state_ops` and `step_ops` | Same type | Slightly less duplication; every consumer must know which list is live |
| **Tagged `StateOperation`** | Overload `f_func` / add kind flag | Same dataclass | Rejects frozen decision: **do not overload `f` to mean `step`** |

**Why parallel is enough:** port topology + gather recipes are the expensive shared part (~90% of
compile logic). Evolution walk (`f` vs `step`) is a small, separate loop. Phase 4 partial-fire
only needs **`sys_id` on ops** + optional per-block `step` entry points — same on both plan types.

**What to reuse (import/call, no edits to flow builders):**

| Shared machinery | Owner | Step path usage |
| --- | --- | --- |
| `check_algebraic_loops` | `wiring.py` | same call → port execution order |
| `_build_gather_sources`, `_state_slice` | `compiler.py` (private) | import in `step_compiler.py` — diagram-agnostic |
| `PortOperation` | `execution_plan.py` | **reuse dataclass** in `StepExecutionPlan.port_ops` |
| `NumpyDiagramEvaluator._compute_port_signals` pattern | `numpy_evaluator.py` | copy into step diagram evaluator with **`k`** third arg |

**What stays flow-only (do not touch in Phase 2):**

- `build_execution_plan`, `compile_diagram`, `ExecutionPlan.state_ops`
- `NumpyDiagramEvaluator.f`, `JaxDiagramEvaluator`, `Simulator`, flow `compile()` fork for
  `DiagramSystem`

### `StepExecutionPlan` (new)

**File:** `minilink/core/compile/step_execution_plan.py`

```text
StepOperation          # mirror StateOperation: step_func, local_x_slice, gather_sources, sys_id
StepExecutionPlan      # state_dim, signal_dim, port_ops, step_ops, output_slices, external_output_slices
```

- **`port_ops`**: tuple of existing **`PortOperation`** — identical gather recipes to flow.
- **`step_ops`**: tuple of **`StepOperation`** — `isinstance(subsystem, StepSystem)` only.
- **`build_step_execution_plan(diagram)`** in **`step_compiler.py`** — parallel to
  `_build_execution_plan_from_order`, calls shared `_build_gather_sources` / `_state_slice`.

`StepEvaluator.step` (diagram):

1. Walk **`port_ops`** — pass **`k`** into each `compute_func`.
2. Apply **`step_ops`** — pass **`k`** into each `step_func` → write `x_new` slices.

Order: port signals before state advance (same as flow: ports before evolution).

### `compile()` dispatch (extends Phase 1)

**Order is critical** — `StepDiagramSystem` IS-A `StepSystem`:

| Order | `system` type | Route |
| --- | --- | --- |
| 1 | `DiagramSystem` | `compile_diagram` → `NumpyDiagramEvaluator` / `JaxDiagramEvaluator` |
| 2 | `StepDiagramSystem` | `compile_step_diagram` → `NumpyStepDiagramEvaluator` / `JaxStepDiagramEvaluator` |
| 3 | `StepSystem` leaf | `NumpyStepEvaluator` / `JaxStepEvaluator` ([Phase 1](01-step-core.md)) |
| 4 | `DynamicSystem` leaf | `NumpyDynamicEvaluator` / `JaxDynamicEvaluator` |
| 5 | `System` `n==0` | static evaluator |

Compile-time guards (symmetric to flow):

- Flow `compile_diagram`: reject `StepSystem` subsystems (already implemented).
- Step `compile_step_diagram`: reject `DynamicSystem` subsystems.

### Evaluator backends

| Backend | Class | Public API | Third slot |
| --- | --- | --- | --- |
| NumPy | `NumpyStepDiagramEvaluator` | `.step`, `.step_p`, `.outputs`, `.outputs_p`, `.rollout` | `int` `k` |
| JAX | `JaxStepDiagramEvaluator` | same | `int` `k` — **separate** JIT from `JaxDiagramEvaluator` |

- Subclass **`StepEvaluator`** (same as leaf) — inherits **`StepRolloutMixin.rollout`**.
- Boundary outputs: **`outputs` dict** only — **no** evaluator `.h` ([Phase 1a](01a-evolution-map-refactor.md)).
- **JAX:** do not share one `jax.jit` between flow (`t` float) and step (`k` int). Do not
  `float(k)` for tracing.

### Partial firing (Phase 4 preview)

`ScheduledStepOrchestrator` fires a **subset** of blocks per tick. Phase 2 compile should:

- Keep **`sys_id`** on every `PortOperation` / `StepOperation` (already on flow ops).
- Expose optional **`step_block(sys_id, x, u, k)`** (or op index ranges) on diagram
  `StepEvaluator` so Phase 4 does not re-compile per mask.

Full orchestrator logic: [Phase 4](04-scheduled-orchestrator.md).

## Run

### `rollout()` / `compute_rollout` (Phase 1 — reuse)

Primary path — no new public runner:

```python
diagram.compute_rollout(n_steps=50)
# or
diagram.compile().rollout(x0, n_steps=50, u=u_seq)
```

### `TimedStepSimulator` (optional stopgap)

- **Optional** — only if a test needs `sync_dt` **logging** before Phase 4.
- Inner loop: `StepEvaluator.step` / `rollout` with integer **`k`**.
- **Replaced** by `ScheduledStepOrchestrator` in [Phase 4](04-scheduled-orchestrator.md).
- **Do not** document in README until Phase 4 lands.

If `compute_rollout` covers all Phase 2 tests, **skip** `TimedStepSimulator` entirely.

## Composition shortcuts

`minilink/core/composition.py` is `DiagramSystem`-only today. Phase 2 tests use explicit
`connect` / `add_subsystem`. Extend `@` / `>>` for `StepDiagramSystem` in Phase 2 **only if**
low cost; otherwise defer — not a gate for Phase 4.

## Tests

| File | Cases |
| --- | --- |
| `test_step_diagram.py` | wiring; `step()` reference; closed loop via `connect`; gather passes **`k`**; reject `DynamicSystem` at compile |
| `test_step_diagram_rollout.py` | `compile().rollout` and **`compute_rollout`** on diagram; parity vs hand-wired leaf chain |
| `test_step_diagram_jax.py` | (if needed) diagram step JIT uses **`int` `k`** only |
| `test_timed_step_simulator.py` | only if `TimedStepSimulator` is built |

**Suggested parity fixture:** discrete accumulator (or plant) + static `Gain` in unity feedback;
compare diagram rollout to manual Python loop.

## Implementation slices

| # | Slice | Touches flow path? |
| --- | --- | --- |
| 1 | `StepDiagramSystem` shell + interpreted `step()` | no |
| 2 | `StepOperation`, `StepExecutionPlan`, `step_compiler.py` | no — imports private gather helpers only |
| 3 | `NumpyStepDiagramEvaluator` | no |
| 4 | `compile()` fork + `compile_step_diagram` | yes — **one** ordered `isinstance` branch in `compile()` only |
| 5 | `test_step_diagram*.py`, optional `examples/scripts/step/demo_step_diagram_*.py` | no |
| 6 | `JaxStepDiagramEvaluator` | no |
| 7 | Partial-fire hooks (`step_block` / op ranges) | no |
| 8 | `TimedStepSimulator` | no — optional |

## Explicit non-goals (Phase 2)

- Edits to `build_execution_plan`, `StateOperation`, `NumpyDiagramEvaluator.f`, `Simulator`
- `StepRunner`, `run_steps`, `step_runner.py`
- Evaluator `.h` / `.h_p` tiers
- README hybrid call chain (Phase 13)
- `hybrid_closed_loop`, `HybridSimulator` (Phase 5)
