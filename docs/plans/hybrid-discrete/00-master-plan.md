# Hybrid discrete simulation: Master Plan

Status: draft plan (July 2026). No implementation in this phase.

Architecture for discrete-time blocks, step diagrams, optional discretization, scheduled stepping,
narrow hybrid simulation (discrete controller ↔ continuous plant), and MPC `StepBlock` export.
Supersedes the continuous-time-only stance in [DESIGN.md](../../../DESIGN.md) §3 for this
**subset** — not full Simulink parity.

## Vision — implementation phases

| Phase | Doc | Delivers |
| --- | --- | --- |
| **0** | [00-wiring-refactor.md](00-wiring-refactor.md) | Shared wiring mixin from `DiagramSystem` — **no new behavior** |
| **1** | [01-step-core.md](01-step-core.md) | `StepSystem`, `ZOHHold` — pure `step(x, u, k)`, no clock |
| **2** | [02-step-diagram.md](02-step-diagram.md) | `StepDiagramSystem`, compile, `StepEvaluator`, `StepRunner` |
| **3** | [03-discretization.md](03-discretization.md) | `discretize(DynamicSystem, dt)` → `StepSystem` (optional tool) |
| **4** | [04-scheduled-orchestrator.md](04-scheduled-orchestrator.md) | `StepSchedule.dt_base` + `ScheduledStepOrchestrator` |
| **5** | [05-hybrid-simulation.md](05-hybrid-simulation.md) | `HybridDiagram` + `HybridSimulator` + SMC / cascade demos |
| **5c** | [05c-hybrid-viz-shortcuts.md](05c-hybrid-viz-shortcuts.md) | `plot_hybrid_diagram`, `hybrid_closed_loop` |
| **6** | [06-mpc-step-block.md](06-mpc-step-block.md) | `MPCStepBlock` API — stateless (6a) then warm-start state (6b) |

**Clock rule:** sample time lives in **`StepSchedule.dt_base`** (Phase 4+). Leaf `step` and
Phase 2 diagrams stay time-agnostic; hybrid sim **always** uses the orchestrator on the step side.

## Architecture pipeline

```mermaid
flowchart TB
    subgraph P0 [Phase 0 Wiring]
        WIR[WiredDiagramMixin]
    end

    subgraph P1 [Phase 1 Step leaf]
        SS[StepSystem step]
    end

    subgraph P2 [Phase 2 Step diagram]
        SDS[StepDiagramSystem]
        SE[StepEvaluator]
        SR[StepRunner]
    end

    subgraph P3 [Phase 3 Conversion]
        DISC[discretize]
    end

    subgraph P4 [Phase 4 Clock]
        SCH[StepSchedule dt_base]
        ORCH[ScheduledStepOrchestrator]
    end

    subgraph P5 [Phase 5 Hybrid]
        HD[HybridDiagram]
        HS[HybridSimulator boundary + plant]
    end

    subgraph P5c [Phase 5c Viz and shortcuts]
        PLOT[plot_hybrid_diagram]
        HCL[hybrid_closed_loop]
    end

    subgraph P6 [Phase 6 MPC StepBlock]
        MPC[MPCStepBlock]
    end

    subgraph Cont [Existing continuous]
        DS[DynamicSystem]
        DF[DiagramSystem]
    end

    WIR --> DF
    WIR --> SDS
    SS --> SDS --> SE
    SE --> SR
    DS --> DISC --> SS
    SDS --> ORCH
    SCH --> ORCH
    ORCH --> HS
    SDS --> HD
    DF --> HD
    HD --> HS
    HD --> PLOT
    HD --> HCL
    MPC --> SDS
```

## Split of concerns

| Concern | Owner | Phase |
| --- | --- | --- |
| Shared port wiring / `state_index` | `WiredDiagramMixin` | **0** |
| Pure `step` math | `StepSystem` | 1 |
| Step block wiring | `StepDiagramSystem` / `StepEvaluator` | 2 |
| Continuous → discrete plant block | `discretize()` | 3 (optional) |
| Sample time + multi-rate **inside** step diagram | `StepSchedule` + orchestrator | 4 |
| Step↔plant ZOH/sample + plant integration | `HybridSimulator` | 5 |
| Hybrid plot + `hybrid_closed_loop` | `graphical/` + `hybrid_composition` | **5c** |
| MPC planner → `StepSystem` for simulation | `MPCStepBlock` | 6 |

## Implementation order

| Step | Phase | Deliverable |
| --- | --- | --- |
| **0** | **0** | **`core/wiring.py` mixin; `DiagramSystem` delegates; validation gate (pytest + smoke)** |
| 1 | 1 | `StepSystem`, `ZOHHold`, leaf tests |
| 2 | 2 | `StepDiagramSystem` on mixin, `compile_step_diagram`, `StepEvaluator` |
| 3 | 2 | `StepRunner`, `TimedStepSimulator`, closed-loop tests |
| 4 | 3 | `discretize` verb + tests *(optional)* |
| 5 | 4 | `StepSchedule`, `ScheduledStepOrchestrator`, orchestrator tests |
| 6 | 5 | `rk4_rollout_zoh` |
| 7 | 5 | `HybridDiagram`, `HybridSimulator` (multi-port boundary) |
| 8 | 5 | `SMCBlock` + hybrid demo **(5a)** |
| 9 | 5 | Cascade hybrid demo **(5b**, non-trivial `fire`) |
| 10 | 5c | `build_hybrid_topology`, `plot_hybrid_diagram`, `hybrid_closed_loop` |
| 11 | 6 | `MPCStepBlock` stateless **(6a)** + straight-line MPC demo refactor |
| 12 | 6 | `MPCStepBlock` warm-start state **(6b)** |
| 13 | all | DESIGN / ROADMAP / README |

**Gates:**

- **Phase 0 before Phase 1** — wiring extracted and continuous behavior validated unchanged.
- Phase 2 before 4, 5, and before Phase 3 (discretize returns blocks used in diagrams).
- **Phase 4 before Phase 5** — hybrid always orchestrates the step side.
- Phase 3 optional for hybrid demos (continuous plant + native controller blocks).
- **5a** before **5b** (trivial schedule before multi-rate cascade hybrid).
- **5b** before **5c** (or 5c plot in parallel once `HybridDiagram` lands; shortcuts after 5a).
- **Phase 5 before Phase 6** — hybrid sim exists before MPC `StepBlock` lands.
- **6a** before **6b** (stateless MPC block before warm-start state).

Full design: [hybrid-discrete-simulation.md](../hybrid-discrete-simulation.md).
