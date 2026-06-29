# Phase 4 — Demo `_v2`

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Target demo
shapes: [`../02_demo_use_cases.md`](../02_demo_use_cases.md).

~10 demo subclasses get `_v2` methods. Old `time_channel_matrix` stays on the old
path; the v2 path uses dict + `get_dynamic_geometry_v2`.

This phase proves the dynamic/overlay primitives (arrows, torque arcs, MPC
horizon, trails) reach pixel parity through the v2 path **before** any subclass is
deleted — the subclasses themselves are retired in Phase 6.

## Automated gate

Demo PNG parity: each migrated demo's `animate_v2` output matches its `animate`
output (use the `COMPARE_PIPELINES` pattern from use case 11).

## User Review 4

One MPC demo with `COMPARE_PIPELINES=True` (e.g.
`demo_dynamic_bicycle_rate_mpc_straight_line.py`) plus one trajopt demo.
