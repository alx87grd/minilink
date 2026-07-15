# Phase E5 — Retire PoC leaves

**Status:** stub (expand before coding)  
**Depends on:** flagships already on product API ([E3](phase-E3.md)); remaining
call sites migrated or explicitly kept as thin aliases

## Goal

Remove or thin-wrap `MPCStatelessController` / `MPCStatefulController` /
`mpc_*_controller` factories after everything uses
`ModelPredictiveController` (pre-1.0 rename cleanly). Keep warm_start /
overlays tools.

## Still to move before delete (inventory)

E3 migrated flagships only. Before deleting PoC modules, update or decide:

| Call site | Notes |
| --- | --- |
| `examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_closed_loop_lap.py` | hand loop + `planner.step` |
| Obstacle / stadium / wide / multi-obstacle demos | same |
| `examples/notebooks/demo_mpc_hybrid_track_lap.ipynb` | still documents `mpc_stateful_controller` |
| Unittests importing PoC factories | retarget to `ModelPredictiveController` or keep thin aliases |

Trajopt reference + spatial guide may stay on their teaching APIs (no
product-controller requirement).

## Exit

One controller product: `ModelPredictiveController` (System family).

## Gate

Full MPC / hybrid unittest set green.

## Outline (expand when starting)

| Step | Work |
| --- | --- |
| E5.1 | Point remaining demos / notebook / tests at `ModelPredictiveController` |
| E5.2 | Delete or deprecate PoC leaf modules (keep aliases only if chosen) |
| E5.3 | Shrink hybrid tests to controller surface |

Next: [phase-E6.md](phase-E6.md).
