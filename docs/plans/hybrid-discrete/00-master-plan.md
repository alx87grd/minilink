# Hybrid discrete simulation: Master Plan

Status: draft plan (July 2026). No implementation in this phase.

Architecture for discrete-time blocks, discrete diagrams, multi-rate orchestration, and narrow
hybrid simulation (discrete ZOH → continuous plant ← measurements). Supersedes the
continuous-time-only stance in [DESIGN.md](../../../DESIGN.md) §3 for this **subset** — not
full Simulink parity.

## Vision and Structure

This design splits the hybrid discrete simulation capability into a clear pipeline, described across four detailed contracts:

1. **[Discretization (01-discretization.md)](01-discretization.md)**: Bridging continuous systems to the discrete domain.
2. **[Step Core (02-step-core.md)](02-step-core.md)**: Pure discrete math with no notion of physical time.
3. **[Scheduled Orchestrator (03-scheduled-orchestrator.md)](03-scheduled-orchestrator.md)**: Clock-driven multi-rate diagram execution.
4. **[Hybrid Simulation (04-hybrid-simulation.md)](04-hybrid-simulation.md)**: Connecting discrete controllers to continuous plants.

## Architecture Pipeline

```mermaid
flowchart TB
    subgraph Continuous [Continuous Domain]
        DS_C[DynamicSystem]
        Diag_C[DiagramSystem]
    end

    subgraph Conversion [Discretization Layer]
        DISC[discretize]
    end

    subgraph Layer1 [Layer 1: Pure Discrete Core]
        DS_D[StepSystem<br/>x⁺ = φ(x, u, k)]
        Diag_D[StepDiagramSystem]
        DSim[StepRunner<br/>N steps]
    end

    subgraph Layer2 [Layer 2: Scheduled / Clocked]
        sched[StepSchedule<br/>dt_base, fire map]
        orch[ScheduledStepOrchestrator]
    end

    subgraph Layer3 [Layer 3: Hybrid Simulation]
        HybDiag[HybridDiagram]
        HybSim[HybridSimulator]
    end

    DS_C --> DISC
    DISC --> DS_D
    DS_D --> Diag_D
    Diag_D --> DSim
    
    Diag_D -.->|if physical time| sched
    sched --> orch
    orch -->|evaluates diagram natively| Diag_D
    
    Diag_D -.->|controller side| HybDiag
    Diag_C -.->|plant side| HybDiag
    HybDiag --> HybSim
```

## Phases

The implementation will be delivered in two phases:
- **Phase A**: Single-rate hybrid simulation (MPC/SMC + ZOH + continuous plant).
- **Phase B**: Multi-rate cascade controllers via the `ScheduledStepOrchestrator`.

*Note: Graph expansion via `expand_scheduled_step` has been deliberately moved out-of-scope for the initial releases, relying entirely on the orchestrator approach to keep diagram logic thin.*

## Implementation order

| Step | Component | Deliverable | Phase |
| --- | --- | --- | --- |
| 1 | Layer 1 | `StepSystem`, `ZOHHold`, leaf tests | — |
| 2 | Conversion| `discretize` verb mapping continuous to `StepSystem` | — |
| 3 | Layer 1 | shared diagram wiring mixin (`core/wiring.py`) | — |
| 4 | Layer 1 | `StepDiagramSystem`, `compile_step_diagram` + `StepEvaluator` | — |
| 5 | Layer 1 | `StepRunner` + `TimedStepSimulator` + closed-loop tests | — |
| 6 | Layer 3 | `rk4_rollout_zoh` | — |
| 7 | Layer 3 | `HybridDiagram` + `HybridSimulator` (ZOH + sample) | **A** |
| 8 | Layer 3 | `MPCBlock` + straight-line MPC demo | **A** |
| 9 | Layer 3 | `SMCBlock` (or pattern) + SMC hybrid demo | **A** |
| 10 | Layer 2 | `StepSchedule` + `ScheduledStepOrchestrator` + tests | **B** |
| 11 | Layer 3 | Cascade controller hybrid demo | **B** |
| 12 | all | DESIGN / ROADMAP / README | — |

**Gate:** Phase A hybrid (steps 7–9) begins after step compile + `rk4_rollout_zoh`. Phase B (steps 10–11) begins after Phase A hybrid tests pass.\n