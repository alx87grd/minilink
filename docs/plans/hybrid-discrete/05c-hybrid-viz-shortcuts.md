# Phase 5c: Hybrid diagram visualization & wiring shortcuts

**After [Phase 5b](05-hybrid-simulation.md)** (or in parallel once `HybridDiagram` exists).
Makes hybrid systems as easy to **see** and **wire** as continuous `DiagramSystem` today.

**Requires:** Phase 0 wiring mixin, Phase 2 `StepDiagramSystem`, Phase 5 `HybridDiagram`.

## Goals

| Today (continuous) | Target (hybrid) |
| --- | --- |
| `diagram.plot_diagram()` | `hybrid.plot_diagram()` — one figure, two sides + boundary edges |
| `ctl @ plant` → `DiagramSystem` | `hybrid_closed_loop(step_ctl, plant, schedule=...)` → `HybridDiagram` |
| `build_diagram_topology(diagram)` | + `build_hybrid_topology(hybrid)` |

## Visualization

### Step diagrams (reuse + small extend)

Generalize `build_diagram_topology` (`minilink/graphical/diagrams/topology.py`) to accept any
**wired diagram** (`DiagramSystem` or `StepDiagramSystem`) — same `subsystems` / `connections`
surface from [Phase 0](00-wiring-refactor.md).

- Step leaf nodes: `kind="step_system"` (vs `"system"`) for styling in `dot.py` `block_html`.
- `StepDiagramSystem.plot_diagram()` via lazy façade (same pattern as `SharedSystemFacades`).

### Hybrid composite plot (new)

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
       ├── build_diagram_topology(hybrid.step)      # step cluster
       ├── build_diagram_topology(hybrid.continuous) # plant cluster
       └── boundary edges from hybrid.connections   # ZOH / sample labels
  → Graphviz: two subgraph clusters + dashed boundary edges
  → schedule subtitle (dt_base, optional fire summary)
```

**`BoundaryTopologyEdge`:** `direction`, `step_port`, `continuous_port`, label (`ZOH` / `sample`).

**Not in topology:** orchestrator hold buffers (sim-time); optional subtitle annotation only.

**Mermaid:** extend `mermaid.py` exporter or compose two existing exports + boundary links.

## Wiring shortcuts

### Do not overload `@` across domains

`ctl @ plant` must keep returning **`DiagramSystem`** (continuous homogeneous diagram).
Detecting `StepSystem` + `DynamicSystem` and silently returning `HybridDiagram` is too magical
(`schedule` required, sample delay, different compile paths).

### `hybrid_closed_loop` (recommended)

**File:** `minilink/core/hybrid_composition.py` (or tail of `composition.py` if small)

```python
def hybrid_closed_loop(
    step_side: System | StepDiagramSystem,
    continuous_plant: System | DiagramSystem,
    *,
    schedule: StepSchedule | float,
    step_out: str = "u",
    plant_in: str = "u",
    plant_out: str = "y",
    step_in: str = "y",
    ref_port: str = "r",
    output_port: str = "y",
) -> HybridDiagram:
    """Canonical sampled-controller ↔ continuous-plant feedback topology."""
```

Behavior:

1. Wrap leaf `step_side` in `StepDiagramSystem` if needed (`ctl` id).
2. Wrap leaf plant in `DiagramSystem` if needed (`plant` id).
3. Expose `r` on step diagram boundary; expose `y` on plant diagram boundary.
4. `connect_boundary(step_to_plant)` and `connect_boundary(plant_to_step)`.
5. `schedule=float` → `StepSchedule(dt_base=float)`.

Layer 1 facade; Layer 3 contract remains explicit `connect_boundary` on `HybridDiagram`.

### Step-side shortcuts (optional, Phase 2 or 5c)

Mirror `composition.py` for **`StepDiagramSystem` only**:

- `step_ctl @ step_plant` — discrete closed loop inside step domain
- `>>` / `+` for step cascades (filter >> mpc)

Reject `DynamicSystem` at wiring time. Lower priority than `hybrid_closed_loop`.

### Phase 6 sugar

```python
def hybrid_mpc_loop(mpc_block, plant, *, mpc_dt: float, ...) -> HybridDiagram:
    """`hybrid_closed_loop` + `MPCStepBlock` defaults."""
```

Lives in `planning/mpc/` or `hybrid_composition.py` — Phase 6 doc.

## Tests

| File | Cases |
| --- | --- |
| `test_hybrid_topology.py` | boundary edges; cluster ids; schedule label |
| `test_hybrid_closed_loop.py` | shortcut matches manual `connect_boundary` wiring |
| `test_step_diagram_topology.py` | step nodes `kind="step_system"` (if not covered in Phase 0/2) |

## Deferred

- Hold registers as diagram nodes (`expand_scheduled_step` lowering path).
- Single flat canvas mixing flow + step blocks.
- `@` operator polymorphism for hybrid.
