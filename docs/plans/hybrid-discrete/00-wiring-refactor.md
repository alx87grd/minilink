# Phase 0: Core wiring refactor (prep)

**Before [Phase 1](01-step-core.md).** Extract **all evolution-agnostic diagram machinery**
from `DiagramSystem` into a reusable mixin — **no new simulation behavior**, no `StepSystem`,
no hybrid types, no compile fork.

Goal: land a behavior-preserving refactor so Phase 2 `StepDiagramSystem` (same file as
`DiagramSystem`) reuses one wiring + port-gather + viz implementation. The only fundamental
diagram fork is **`f()` vs `step()`**; everything else is shared or a parallel
compile/evaluator path.

## Design rationale

Discrete and continuous diagrams share the same wiring graph:

```text
for each subsystem with state:
    local_x, local_u = gather(x, u, coord, sys_id)   # shared — see third-slot rule
    piece = subsystem.f(...)       # DiagramSystem
    piece = subsystem.step(...)    # StepDiagramSystem  (Phase 2)
stack pieces → dx or x_new
```

We **split types and names** (`DynamicSystem.f` vs `StepSystem.step`) so the distinction stays
explicit — not hidden in a `time_domain` flag on `f`. Phase 0 **maximizes shared code**:
wiring, interpreted port gather, params, topology checks, diagram-level `tf` / geometry, and
(later) the port half of `ExecutionPlan` are identical. Evaluator rollout (`rk4_*`) differs only
on the flow path.

## Third coordinate slot (`t` vs `k`)

One Python parameter position on shared gather / `port.compute` / diagram `tf` paths; **call-site
semantics** differ by diagram kind:

| Context | Third slot | Type | Meaning |
| --- | --- | --- | --- |
| Flow diagram / `DynamicsEvaluator` | **`t`** | `float` | simulation time (seconds) |
| Step diagram / `StepEvaluator` | **`k`** | `int` | step / turn index (no wall clock) |
| `StepSystem.step` / `StepSystem.h` | **`k`** | `int` | evolution / output index |
| `DynamicSystem.f` / `h` | **`t`** | `float` | continuous time |

**Rules:**

- **`StepSystem` has no wall time** — chess / turn-based systems use `k = 0, 1, 2, …` only.
- **Do not** inject `t_k = k · dt_base` into `StepSystem` leaves (no artificial time).
- **Do not** convert at boundaries (`float(k)`, `int(t)`) — each path passes its native type.
- Flow time-varying blocks (`Step` source at `step_time` s, `WhiteNoise`, etc.) are **not**
  reused in step diagrams without discrete counterparts (v1).
- `OutputPort.compute(x, u, t, params)` keeps the parameter name **`t`** in the dataclass; on
  step diagrams callers pass **`k`** in that position (document in docstrings).

**JAX (Phase 2+):** separate `DynamicsEvaluator` and `StepEvaluator` JIT caches — flow JIT
threads **`float` `t`**; step JIT threads **`int` `k`**. Never one jitted graph mixing both.
Same rule as today for `if t < threshold`: blocks that branch on the third slot need
JAX-safe style on either path.

Phase 0 gather methods keep parameter name **`t`** on `DiagramSystem` for backward compatibility;
docstrings note step-side semantics. Phase 2 `StepDiagramSystem` gather passes **`k`**.

## Why a dedicated phase

Phase 2 otherwise mixes “split wiring” with “new step compile path” in one diff. Phase 0 keeps
the first hybrid-related change **small, reviewable, and test-gated** — only rearrangement.

## Target file layout

### Phase 0 (this phase)

```text
minilink/core/
├── wiring.py          # NEW — validate_diagram_params, WiredDiagram, WiredDiagramMixin
└── diagram.py         # DiagramSystem(WiredDiagramMixin, System) — flow tail only
```

### Phase 2 (not Phase 0)

```text
minilink/core/
├── wiring.py          # unchanged
└── diagram.py         # + StepDiagramSystem(WiredDiagramMixin, System) in the SAME file
```

`StepDiagramSystem` lives beside `DiagramSystem` in `diagram.py` — one module for both diagram
siblings. Hybrid types stay in `hybrid_diagram.py` (Phase 5).

## Class layout after Phase 0

```text
System                          # ports, h, params shell (unchanged)
    │
WiredDiagramMixin               # wiring.py — all shared diagram machinery
    │
DiagramSystem                   # diagram.py — f() + flow compile + flow-only post-process
    .f()
    .compile()  → compile_diagram → DynamicsEvaluator
```

### Phase 2 addition (same `diagram.py`)

```text
StepDiagramSystem(WiredDiagramMixin, System)
    .step()
    .compile()  → compile_step_diagram → StepEvaluator
```

Both diagram classes inherit **`WiredDiagramMixin` only**. `compile()` keeps the same method
name on both; return type and backend differ (Phase 2).

## Method move map — `diagram.py` today → Phase 0

### → `minilink/core/wiring.py` (`WiredDiagramMixin` + helpers)

| Method / attr | Role | Notes |
| --- | --- | --- |
| `validate_diagram_params()` | module-level helper | Co-locate with mixin |
| **`WiredDiagram`** protocol | structural typing | **Required** |
| `check_algebraic_loops()` | topology DFS | **Move implementation here**; `compiler.py` imports it |
| `subsystems`, `connections` | registry | |
| `connection_verbose` | wiring UX | |
| `_composition_entry`, `_composition_output` | composition shortcuts | |
| `_init_wiring()` | mixin init helper | Called from diagram `__init__` |
| `add_subsystem` | registry | |
| `subsystem_id`, `subsystem_signal` | registry | |
| `connect` | port wiring | boundary `"input"` / `"output"` |
| `connect_new_output_port` | boundary expose | Uses shared gather |
| `compute_state_properties` | stacked `n`, `state_index`, labels | |
| `refresh` | subsystem refresh + rebuild metadata | |
| `params` property + setter | nested live view | |
| `_subsystem_params` | per-subsys params routing | |
| `get_local_state` | slice `x` via `state_index` | |
| `get_local_input` | assemble local `u` from graph | third slot → `port.compute` |
| `get_subsys_input_port` | one input port resolve | |
| `compute_subsys_output_port` | `port.compute(x, u, third, …)` | flow: `t`; step: `k` |
| `check_algebraic_loops` (method) | delegate to module function | optional thin wrapper on mixin |
| `tf` | diagram-level frame merge | recurse `subsystem.tf`; third slot per table |
| `get_dynamic_geometry` | diagram-level geometry merge | same |
| `get_kinematic_geometry` | diagram empty dict today | stays on mixin for both diagram kinds |
| `autowire` | thin delegate | both diagram types later |

**Mixin init** (implementation sketch):

```python
def _init_wiring(self, *, name: str = "Diagram") -> None:
    self.subsystems = {}
    self.connections = {}
    self.connection_verbose = False
    self._composition_entry = None
    self._composition_output = None
    self.name = name
    self.compute_state_properties()
```

`DiagramSystem.__init__`: `System.__init__(self, 0)` then `_init_wiring(name="Diagram")`.

### Stays in `diagram.py` on `DiagramSystem` (flow-only)

| Method | Why here |
| --- | --- |
| `f` | Calls `subsystem.f(local_x, local_u, t, …)` — continuous evolution fork |
| `compile` | `compile_diagram` → `DynamicsEvaluator` |
| `reconstruct_internal_signals` | Flow `Trajectory.t` post-process |

### Phase 2 adds to `diagram.py` on `StepDiagramSystem` (not Phase 0)

| Method | Parallel to |
| --- | --- |
| `step` | `DiagramSystem.f` — calls `subsystem.step(..., k, …)` |
| `compile` | → `compile_step_diagram` → `StepEvaluator` |

Subsystem policy: flow diagram rejects `StepSystem` at compile (optional Phase 2); step diagram
rejects `DynamicSystem` at compile.

## `WiredDiagram` protocol (required)

```python
class WiredDiagram(Protocol):
    name: str
    subsystems: dict[str, System]
    connections: dict[str, dict[str, tuple[str, str] | None]]
    inputs: dict   # from System
    outputs: dict  # from System
    state_index: dict[str, tuple[int, int]]
    n: int
```

Used by:

- `check_algebraic_loops` / `_build_gather_sources` (typed as `WiredDiagram`)
- `build_diagram_topology` generalization (Phase **5c**; Phase 0 does not change `topology.py`)

## What is shared vs forked (reference)

| Layer | Shared | Fork (`f` vs `step`) |
| --- | --- | --- |
| Wiring graph | `connect`, `subsystems`, `connections`, `state_index` | — |
| Topology | `check_algebraic_loops` | — |
| Interpreted gather + ports + viz | `get_local_*`, `compute_subsys_output_port`, `tf`, `get_dynamic_geometry` | third slot: `t` vs `k` |
| Diagram interpreted evolve | loop structure | `f()` / `step()` body |
| `compile()` ceremony | loop detect, `port_ops`, gather recipes | `f_func` vs `step_func`; evaluator class |
| Evaluator fast path | `port_ops` walk | `.f`→`dx`+RK4 vs `.step`→`x_new` |
| Simulation | — | ODE / hybrid plant vs step / hybrid orchestrator |

**Not shared:** `rk4_*` rollout on `DynamicsEvaluator` only. **`dt_base`** is orchestrator /
hybrid-plant scheduling — **not** passed into `StepSystem.step`.

## Compile module (Phase 0 touch: re-export only)

Phase 0 **moves** `check_algebraic_loops` implementation to `wiring.py`. `compiler.py` imports
it from there (dependency direction: compile → wiring, not wiring → compile).

Phase 2 adds:

```text
compile/wiring_plan.py   # build_port_ops, gather helpers (optional extract)  ← shared
compile/compiler.py      # compile_diagram, StateOperation(f_func=...)
compile/step_compiler.py # compile_step_diagram, step_ops(step_func=...)  (name TBD)
```

## In scope (Phase 0)

| Deliverable | File |
| --- | --- |
| `WiredDiagram` protocol | `wiring.py` |
| `check_algebraic_loops(WiredDiagram)` | `wiring.py` |
| `WiredDiagramMixin` with full table above | `wiring.py` |
| `DiagramSystem(WiredDiagramMixin, System)` | `diagram.py` |
| `compiler.py` imports `check_algebraic_loops` from `wiring` | `compile/` |
| Validation gate | tests + smoke |

## Out of scope (later phases)

| Item | Phase |
| --- | --- |
| `StepSystem`, `StepDiagramSystem` class body | 1–2 (`diagram.py`) |
| `compile_step_diagram`, `StepEvaluator`, `JaxStepEvaluator` | 2 |
| `build_diagram_topology(WiredDiagram)` | **5c** |
| `composition.py` `isinstance` generalization | 2 or 5c |
| `hybrid_closed_loop`, `plot_hybrid_diagram` | 5c |

## Contract

- **Public `DiagramSystem` API unchanged** — same method names, same `connect` validation,
  same composition shortcuts (`+`, `>>`, `@`, `autowire`).
- **No semantic change** to compiled evaluators, `f()`, or simulation results.
- **`wiring.py` must not import from `core/compile/`** (system-library law).
- **`diagram.py` may import compile** for `DiagramSystem.compile` / `reconstruct_internal_signals`.

## Validation gate (must pass before Phase 1)

| Check | Command / scope |
| --- | --- |
| Lint | `ruff check .` · `ruff format --check .` |
| Wiring parity | `pytest tests/unittest/test_wiring_mixin.py` |
| Composition + wiring | `pytest tests/unittest/test_composition.py` |
| Diagram topology / plot | `pytest tests/unittest/test_diagrams.py` |
| Diagram compile / eval | `pytest tests/unittest/test_diagram*.py` |
| Analysis on diagrams | `pytest tests/unittest/test_analysis_control.py` (boundary output cases) |
| Full suite (handoff) | `pytest` before merge to main hybrid branch |

**Smoke (manual or script):** from repo root, run unchanged:

- `examples/scripts/diagrams/demo_diagram_shortcuts.py`
- `examples/scripts/diagrams/demo_closed_loop.py`
- `diagram.plot_diagram()` on one existing closed-loop example — visual spot-check optional

**Acceptance:** bit-for-bit floating equivalence not required, but **same topology
snapshots** (`build_diagram_topology`) and **same closed-loop trajectories** on fixed seeds
where tests exist.

## Tests to add (Phase 0)

- `test_wiring_mixin.py`: golden topology / edge list parity; gather + `check_algebraic_loops`
  behave identically on refactored `DiagramSystem`.
- Optional: param nesting + `state_index` after add/remove subsystem sequence.

## Implementation notes

- **Mixin over subclass** — `StepDiagramSystem` is a sibling, not a subclass of `DiagramSystem`.
- **Mechanical diff:** move methods, inherit mixin, delete duplicates — no drive-by renames.
- **Do not** add `StepDiagramSystem` stub in Phase 0.
- Mark **`TODO: User Architectural Review`** on `WiredDiagramMixin` until validation gate passes.

## Downstream

| Phase | Uses Phase 0 |
| --- | --- |
| **1** | `StepSystem.step` / `h` use **`k`** only — no wall time on leaf |
| **2** | `StepDiagramSystem` in **`diagram.py`**; gather + ports pass **`k`**; separate step JIT |
| **4** | `dt_base` fires blocks; **`k`** into diagram eval — not `t` into `StepSystem` |
| **5** | Plant integration uses **`t`**; step side uses **`k`** |
| **5c** | `build_diagram_topology(hybrid.step)` via `WiredDiagram` protocol |
