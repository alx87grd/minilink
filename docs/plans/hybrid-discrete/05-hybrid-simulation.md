# Phase 5: Hybrid Simulation

**After [Phase 4](04-scheduled-orchestrator.md).** Two-side topology: **step diagram** (controller)
+ **continuous diagram** (plant), linked by explicit boundary connections.

**Requires:** Phases 1–2 (step compile), Phase 4 (`StepSchedule` + orchestrator on step side).
Phase 3 (`discretize`) **not** required for MPC/SMC (native controllers + continuous plant).

**Files:** `minilink/core/hybrid_diagram.py`, `minilink/simulation/hybrid_simulator.py`

## Split of concerns (Phase 5)

```text
HybridSimulator @ schedule.dt_base
│
├── ScheduledStepOrchestrator     ← step diagram clock + who fires (Phase 4)
│       trivial fire: single-rate MPC (5a)
│       non-trivial fire: filter + MPC cascade (5b)
│
├── boundary ZOH / sample         ← step ↔ plant edges (HybridSimulator only)
│
└── rk4_rollout_zoh               ← continuous plant over dt_base
```

```text
  r ──► [ StepDiagramSystem ] ──u_cmd──ZOH──► plant.u
              ▲                                    │
              ├──y_meas──sample── plant.y ─────────┤
              └──v_fb────sample── plant.v ─────────┘   (optional)

  World inputs (`r`) attach to step/plant diagram boundaries — not hybrid edges.
```

## `BoundaryConnection`

| Direction | Semantics |
| --- | --- |
| Step output → plant input | **ZOH** over `[t_k, t_k + dt_base)` |
| Plant output → step input | **Sample** at tick boundary |

```python
@dataclass(frozen=True)
class BoundaryConnection:
    step: tuple[str, str]
    continuous: tuple[str, str]
```

`HybridDiagram.connect_boundary(step_sys_id, step_port_id, continuous_sys_id, continuous_port_id)`

## `HybridDiagram`

```python
@dataclass
class HybridDiagram:
    step: StepDiagramSystem
    continuous: DiagramSystem
    schedule: StepSchedule              # required — dt_base lives here
    connections: list[BoundaryConnection] = field(default_factory=list)
```

No separate `dt_base` field — use **`schedule.dt_base`** for plant and step grids (v1: same rate).

## `HybridSimulator`

**Always** calls **`ScheduledStepOrchestrator.tick`** on the step side (single-rate = trivial
`fire`). Owns boundary buffers and plant rollout.

### Tick order

1. **Sample** — plant→step edges → `sample_buffers` → assemble `u_step`
2. **Step** — `orchestrator.tick(x_step, u_step, t, k)` → `step_outputs`
3. **ZOH** — step→plant edges → `zoh_buffers` → assemble `u_plant`
4. **Flow** — `rk4_rollout_zoh(x_flow, u_plant, t_k, schedule.dt_base)`

## Milestones

| Milestone | Content |
| --- | --- |
| **5a** | `rk4_rollout_zoh`, hybrid + **trivial schedule**, `MPCBlock` / `SMCBlock`, straight-line demo |
| **5b** | Cascade hybrid demo — **non-trivial `fire`** (e.g. filter @ 100 Hz + MPC @ 10 Hz) |

## End-to-end API (5a — single-rate MPC)

```python
controller = StepDiagramSystem()
controller.add_subsystem(mpc_block, "mpc")
controller.add_input_port("y")
controller.connect_new_output_port("mpc", "u", "u")

plant = DiagramSystem()
plant.add_subsystem(bicycle, "plant")

schedule = StepSchedule(dt_base=MPC_DT)  # empty fire => all blocks every tick

hybrid = HybridDiagram(step=controller, continuous=plant, schedule=schedule)
hybrid.connect_boundary("output", "u", "plant", "u")
hybrid.connect_boundary("input", "y", "plant", "y")

HybridSimulator(hybrid, ...).run()
```

## End-to-end API (5b — multi-rate controller)

```python
schedule = StepSchedule(dt_base=0.01, fire={"filter": 1, "mpc": 10})
hybrid = HybridDiagram(step=controller, continuous=plant, schedule=schedule)
# same boundary connects; orchestrator handles filter vs mpc rates inside step diagram
HybridSimulator(hybrid, ...).run()
```

## Control blocks

- **`MPCBlock`**, **`SMCBlock`** — `StepSystem` wrappers; see `minilink/planning/mpc/`, `minilink/control/`.

## Tests

- `test_rk4_rollout_zoh.py`
- `test_hybrid_simulator.py` — multi-port boundary; 5a matches hand-rolled MPC
- `test_hybrid_boundary_connect.py`
- `test_hybrid_cascade.py` — 5b filter + MPC
- `test_mpc_block.py`, `test_smc_hybrid.py`
