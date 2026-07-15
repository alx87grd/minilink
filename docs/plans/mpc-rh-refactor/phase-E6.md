# Phase E6 — Observability polish

**Status:** stub (expand before coding)  
**Depends on:** [E4](phase-E4.md) (typical) · contracts: [vision.md](vision.md)

## Goal

Polish `ModelPredictiveController` telemetry helpers (e.g.
`get_solve_metadata`); keep animation / plan-history tools; document an
external-node wrap around the existing E2 `compute_command` loop — **no ROS2
package**. Deploy API itself is E2, not invented here.

## Exit / gate

Existing MPC + controller tests green; smoke straight-line with overlays if
applicable.

Next: [phase-E7.md](phase-E7.md).
