# Phase 5c: Hybrid diagram visualization & wiring shortcuts

**Status: Done** (July 2026). Composite `hybrid.plot_diagram()`, `build_hybrid_topology`,
Graphviz/Mermaid export, step leaf `kind="step_system"`, `abstract_boundary` collapse of
external Inputs/Outputs routing nodes, and `hybrid_closed_loop` / `Computer @ plant` shortcuts.

**After [Phase 5b](05-hybrid-simulation.md)** (or in parallel once `HybridDiagram` exists).
Makes hybrid systems as easy to **see** and **wire** as continuous `DiagramSystem` today.

**Requires:** Phase 0 wiring mixin, Phase 2 `StepDiagramSystem`, Phase 4 `Computer`, Phase 5
`HybridDiagram`.

## Goals

| Today (continuous) | Target (hybrid) |
| --- | --- |
| `diagram.plot_diagram()` | `hybrid.plot_diagram()` — **flattened combined topology** in one figure |
| `ctl @ plant` → `DiagramSystem` | `hybrid_closed_loop(computer_side, plant, schedule=...)` → `HybridDiagram` |
| `build_diagram_topology(diagram)` | + `build_hybrid_topology(hybrid)` |

## Visualization

### Step diagrams (reuse + small extend)

Generalize `build_diagram_topology` (`minilink/graphical/diagrams/topology.py`) to accept any
**wired diagram** (`DiagramSystem` or `StepDiagramSystem`) — same `subsystems` / `connections`
surface from [Phase 0](00-wiring-refactor.md).

- Step leaf nodes: `kind="step_system"` (vs `"system"`) for styling in `dot.py` `block_html`.
- `StepDiagramSystem.plot_diagram()` via lazy façade (same pattern as `SharedSystemFacades`).

### Hybrid composite plot (new)

**Goal:** `HybridDiagram` is **not** a true `DiagramSystem`, but **`plot_diagram()`** should still
render a **nice combined view**: plant internals + computer internals + boundary edges, as if it
were one flattened topology.

**Layout (Graphviz clusters):**

```text
┌─ cluster: Plant (DiagramSystem) ─────────────────────────┐
│  [internal flow blocks and connections]                  │
└───────────────────────────┬──────────────────────────────┘
                            │  ZOH / sample (multi-channel)
┌─ cluster: Computer ───────┴──────────────────────────────┐
│  ┌─ StepDiagram internals (flattened subsystems) ─────┐  │
│  │  filter → mpc → …                                   │  │
│  └─────────────────────────────────────────────────────┘  │
│  subtitle: dt_base, fire summary (schedule metadata)      │
└──────────────────────────────────────────────────────────┘
```

- **Computer cluster** is a **visual overlay** — a labeled box wrapping the step-diagram
  topology exported by `build_diagram_topology(computer.diagram)`. It does **not** imply
  `Computer` is a wiring node in compile/sim.
- **Plant cluster** = `build_diagram_topology(hybrid.plant)`.
- **Boundary edges** = dashed links between **cluster boundaries** (computer ports ↔ plant ports),
  labeled `ZOH` / `sample` per `BoundaryConnection.direction`.
- **`abstract_boundary=True`** (default on hybrid export) omits external Inputs/Outputs routing
  nodes; hybrid edges anchor on wired subsystem ports.
- **World inputs** (`r`, etc.) attach to the computer or plant cluster border — same convention
  as continuous diagram boundary ports.

**Files:**

| Module | Role |
| --- | --- |
| `minilink/graphical/diagrams/hybrid_topology.py` | `HybridTopology`, `build_hybrid_topology(hybrid)` |
| `minilink/graphical/diagrams/hybrid_dot.py` | `plot_hybrid_diagram`, Graphviz cluster layout |
| `minilink/graphical/diagrams/__init__.py` | re-export `plot_hybrid_diagram` |
| `minilink/core/hybrid_diagram.py` | `HybridDiagram.plot_diagram()` — lazy import |

**Data flow:**

```text
hybrid.plot_diagram()
  → build_hybrid_topology(hybrid)
       ├── build_diagram_topology(hybrid.plant)           # plant cluster
       ├── build_diagram_topology(hybrid.computer.diagram) # step internals
       ├── wrap step topology in Computer cluster metadata
       └── boundary edges from hybrid.connections         # ZOH / sample labels
  → Graphviz: two subgraph clusters + dashed boundary edges
  → schedule subtitle on Computer cluster (dt_base, optional fire summary)
```

**`BoundaryTopologyEdge`:** `direction`, `computer_port`, `plant_port`, label (`ZOH` / `sample`).

**Not in topology:** Computer cross-rate hold buffers (sim-time double buffer); optional debug
annotation only. **`plot_diagram`** shows **Computer** cluster wrapping flattened step-diagram
internals — visual overlay, not a compile node.

**Mermaid:** extend `mermaid.py` exporter or compose two existing exports + boundary links +
Computer wrapper node.

## Wiring shortcuts

### Do not overload `@` across domains

`ctl @ plant` must keep returning **`DiagramSystem`** (continuous homogeneous diagram).
Detecting `StepSystem` + `DynamicSystem` and silently returning `HybridDiagram` is too magical
(`schedule` required, sample delay, different compile paths).

### `hybrid_closed_loop` (recommended)

**File:** `minilink/core/hybrid_composition.py` (or tail of `composition.py` if small)

```python
def hybrid_closed_loop(
    computer_side: System | StepDiagramSystem,
    plant: System | DiagramSystem,
    *,
    schedule: StepSchedule | float,
    computer_out: str = "u",
    plant_in: str = "u",
    plant_out: str = "y",
    computer_in: str = "y",
    ref_port: str = "r",
    output_port: str = "y",
    extra_boundaries: list[BoundaryConnection] | None = None,
) -> HybridDiagram:
    """Canonical Computer ↔ plant feedback; supports extra boundary channels."""
```

Behavior:

1. Wrap leaf `computer_side` in `StepDiagramSystem` if needed (`ctl` id).
2. Wrap leaf plant in `DiagramSystem` if needed (`plant` id); copy leaf `camera_*` hints onto plant diagram.
3. Build `Computer(step_diagram, schedule)`.
4. Expose `r` on computer diagram boundary; expose `y` on plant diagram boundary.
5. `connect_boundary(computer_to_plant)` and `connect_boundary(plant_to_computer)` for defaults.
6. Append `extra_boundaries` for multi-channel (e.g. `v_fb`).
7. `schedule=float` → `StepSchedule(dt_base=float)`.

Layer 1 facade; Layer 3 contract remains explicit `connect_boundary` on `HybridDiagram`.

### Step-side shortcuts (optional, Phase 2 or 5c)

Mirror `composition.py` for **`StepDiagramSystem` only**:

- `step_ctl @ step_plant` — discrete closed loop inside step domain
- `>>` / `+` for step cascades (filter >> mpc)

Reject `DynamicSystem` at wiring time. Lower priority than `hybrid_closed_loop`.

### Phase 6 sugar

```python
def hybrid_mpc_loop(mpc_block, plant, *, mpc_dt: float, ...) -> HybridDiagram:
    """`hybrid_closed_loop` + `MPCStatefulController` defaults."""
```

Lives in `planning/mpc/` or `hybrid_composition.py` — Phase 6 doc.

## Tests

| File | Cases |
| --- | --- |
| `test_hybrid_topology.py` | boundary edges; Computer + Plant cluster ids; schedule label on Computer cluster |
| `test_hybrid_closed_loop.py` | shortcut matches manual `connect_boundary` wiring |
| `test_step_diagram_topology.py` | step nodes `kind="step_system"` |

## Deferred

- Hold registers as diagram nodes (`expand_scheduled_step` lowering path).
- Single flat canvas mixing flow + step blocks without Computer/plant clusters.
- `@` operator polymorphism for hybrid.
