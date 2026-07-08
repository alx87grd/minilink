# Phase 0: Core wiring refactor (prep)

**Before [Phase 1](01-step-core.md).** Extract shared diagram **wiring** from `DiagramSystem`
into a reusable mixin — **no new simulation behavior**, no `StepSystem`, no hybrid types.

Goal: land a behavior-preserving refactor so Phase 2 `StepDiagramSystem` composes the same
`connect` / boundary-port / `state_index` machinery without duplicating `diagram.py`.

## Why a dedicated phase

Phase 2 otherwise mixes “split wiring” with “new step compile path” in one diff. Phase 0 keeps
the first hybrid-related change **small, reviewable, and test-gated** — only rearrangement.

## In scope

**File:** `minilink/core/wiring.py` (mixin or shared base — name TBD at implementation)

Extract from `DiagramSystem` (delegate or move, same public API on `DiagramSystem`):

| Concern | Examples today (`diagram.py`) |
| --- | --- |
| Subsystem registry | `subsystems`, `connections`, `add_subsystem`, `subsystem_id` |
| Port wiring | `connect`, `connect_new_output_port`, boundary `"input"` / `"output"` |
| Stacked state metadata | `compute_state_properties`, `state_index`, label prefixing |
| Params nesting | diagram `params` setter validation (reuse `validate_diagram_params`) |
| Composition bookkeeping | `_composition_entry`, `_composition_output` (used by shortcuts) |

`DiagramSystem` **inherits or composes** the mixin and keeps flow-specific pieces:

- `f()`, `compile()`, `compute_subsys_*` for continuous evaluation
- `DynamicsEvaluator` integration
- animation / `tf()` geometry merge (unchanged)

## Out of scope (later phases)

| Item | Phase |
| --- | --- |
| `StepSystem`, `StepDiagramSystem` | 1–2 |
| `build_diagram_topology` generalization | 0 optional; **5c** if deferred |
| `hybrid_closed_loop`, `plot_hybrid_diagram` | 5c |
| Evaluator / compile fork | 2 |

## Contract

- **Public `DiagramSystem` API unchanged** — same method names, same `connect` validation,
  same composition shortcuts (`+`, `>>`, `@`, `autowire`).
- **No semantic change** to compiled evaluators or simulation results.
- Mixin must not import from `core/compile/` (system-library law).

Optional: define a **`WiredDiagram`** protocol (structural typing) documenting the surface
`build_diagram_topology` needs: `subsystems`, `inputs`, `outputs`, `connections`, `name`.
Enables Phase 2 / 5c without `isinstance(DiagramSystem)` only.

## Validation gate (must pass before Phase 1)

Treat Phase 0 as done only when continuous diagrams are provably unchanged.

| Check | Command / scope |
| --- | --- |
| Lint | `ruff check .` · `ruff format --check .` |
| Composition + wiring | `pytest tests/unittest/test_composition.py` |
| Diagram topology / plot | `pytest tests/unittest/test_diagrams.py` |
| Diagram compile / eval | `pytest tests/unittest/test_diagram*.py` (any diagram compile tests) |
| Analysis on diagrams | `pytest tests/unittest/test_analysis_control.py` (boundary output cases) |
| Full suite (handoff) | `pytest` before merge to main hybrid branch |

**Smoke (manual or script):** from repo root, run unchanged:

- `examples/scripts/diagrams/demo_diagram_shortcuts.py`
- `examples/scripts/diagrams/demo_closed_loop.py`
- `diagram.plot_diagram()` on one existing closed-loop example — visual spot-check optional

**Acceptance:** bit-for-bit equivalence not required for floating sim, but **same topology
snapshots** (`build_diagram_topology`) and **same closed-loop trajectories** on a fixed seed
where tests already exist.

## Tests to add (Phase 0 only if gaps)

- `test_wiring_mixin.py`: mixin extracted; fresh `DiagramSystem` wiring parity vs pre-refactor
  golden topology JSON / edge lists (capture once, compare after refactor).
- Optional: param nesting + `state_index` after add/remove subsystem sequence.

## Implementation notes

- Prefer **mixin** (`class WiredDiagramMixin`) over subclassing `DiagramSystem` for step sibling.
- Move `validate_diagram_params` next to mixin if not already co-located.
- Keep diff mechanical: copy methods, delegate, delete duplicates — no drive-by renames.
- Mark **`TODO: User Architectural Review`** on mixin until validation gate passes.

## Downstream

Phase 2 `StepDiagramSystem` **uses this mixin** and only adds `step()` recursion + step compile.
Phase 5c topology export calls `build_diagram_topology(hybrid.step)` once step diagrams exist.
