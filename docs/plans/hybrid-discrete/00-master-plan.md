# Hybrid discrete simulation: Master Plan

Status: draft plan (July 2026). No implementation in this phase.

Architecture for discrete-time blocks, step diagrams, optional discretization, scheduled stepping,
narrow hybrid simulation (discrete controller ↔ continuous plant), and MPC `StepBlock` export.
Supersedes the continuous-time-only stance in [DESIGN.md](../../../DESIGN.md) §3 for this
**subset** — not full Simulink parity.

## Vision — six implementation phases

| Phase | Doc | Delivers |
| --- | --- | --- |
| **1** | [01-step-core.md](01-step-core.md) | `StepSystem`, `ZOHHold` — pure `step(x, u, k)`, no clock |
| **2** | [02-step-diagram.md](02-step-diagram.md) | `StepDiagramSystem`, compile, `StepEvaluator`, `StepRunner` |
| **3** | [03-discretization.md](03-discretization.md) | `discretize(DynamicSystem, dt)` → `StepSystem` (optional tool) |
| **4** | [04-scheduled-orchestrator.md](04-scheduled-orchestrator.md) | `StepSchedule.dt_base` + `ScheduledStepOrchestrator` |
| **5** | [05-hybrid-simulation.md](05-hybrid-simulation.md) | `HybridDiagram` + `HybridSimulator` + SMC / cascade demos |
| **6** | [06-mpc-step-block.md](06-mpc-step-block.md) | `MPCStepBlock` API — stateless (6a) then warm-start state (6b) |

**Clock rule:** sample time lives in **`StepSchedule.dt_base`** (Phase 4+). Leaf `step` and
Phase 2 diagrams stay time-agnostic; hybrid sim **always** uses the orchestrator on the step side.

## Architecture pipeline

```mermaid
flowchart TB
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

    subgraph P6 [Phase 6 MPC StepBlock]
        MPC[MPCStepBlock]
    end

    subgraph Cont [Existing continuous]
        DS[DynamicSystem]
        DF[DiagramSystem]
    end

    SS --> SDS --> SE
    SE --> SR
    DS --> DISC --> SS
    SDS --> ORCH
    SCH --> ORCH
    ORCH --> HS
    SDS --> HD
    DF --> HD
    HD --> HS
    MPC --> SDS
```

## Split of concerns

| Concern | Owner | Phase |
| --- | --- | --- |
| Pure `step` math | `StepSystem` | 1 |
| Step block wiring | `StepDiagramSystem` / `StepEvaluator` | 2 |
| Continuous → discrete plant block | `discretize()` | 3 (optional) |
| Sample time + multi-rate **inside** step diagram | `StepSchedule` + orchestrator | 4 |
| Step↔plant ZOH/sample + plant integration | `HybridSimulator` | 5 |
| MPC planner → `StepSystem` for simulation | `MPCStepBlock` | 6 |

## Implementation order

| Step | Phase | Deliverable |
| --- | --- | --- |
| 1 | 1 | `StepSystem`, `ZOHHold`, leaf tests |
| 2 | 2 | shared wiring mixin (`core/wiring.py`) |
| 3 | 2 | `StepDiagramSystem`, `compile_step_diagram`, `StepEvaluator` |
| 4 | 2 | `StepRunner`, `TimedStepSimulator`, closed-loop tests |
| 5 | 3 | `discretize` verb + tests *(optional)* |
| 6 | 4 | `StepSchedule`, `ScheduledStepOrchestrator`, orchestrator tests |
| 7 | 5 | `rk4_rollout_zoh` |
| 8 | 5 | `HybridDiagram`, `HybridSimulator` (multi-port boundary) |
| 9 | 5 | `SMCBlock` + hybrid demo **(5a)** |
| 10 | 5 | Cascade hybrid demo **(5b**, non-trivial `fire`) |
| 11 | 6 | `MPCStepBlock` stateless **(6a)** + straight-line MPC demo refactor |
| 12 | 6 | `MPCStepBlock` warm-start state **(6b)** |
| 13 | all | DESIGN / ROADMAP / README |

**Gates:**

- Phase 2 before 4, 5, and before Phase 3 (discretize returns blocks used in diagrams).
- **Phase 4 before Phase 5** — hybrid always orchestrates the step side.
- Phase 3 optional for hybrid demos (continuous plant + native controller blocks).
- **5a** before **5b** (trivial schedule before multi-rate cascade hybrid).
- **Phase 5 before Phase 6** — hybrid sim exists before MPC `StepBlock` lands.
- **6a** before **6b** (stateless MPC block before warm-start state).

Full design: [hybrid-discrete-simulation.md](../hybrid-discrete-simulation.md).
