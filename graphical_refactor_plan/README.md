# Graphical Refactor Plan

Planning home for the **kinematic contract upgrade** (string-keyed `tf` frames +
frame-keyed geometry, parallel `_v2` pipeline, PNG pixel-parity, phased cutover).

## Files

| File | Role |
| --- | --- |
| [`00_original_vision.md`](00_original_vision.md) | Self-contained ground-truth vision: locked architecture and decisions |
| [`01_master_overview.md`](01_master_overview.md) | Master plan: phases, gates, success criteria, locked decisions, risks |
| [`02_demo_use_cases.md`](02_demo_use_cases.md) | Target user-facing demo scripts for the new public API |
| [`03_kinematics_core_math.md`](03_kinematics_core_math.md) | Vision for `core/kinematics.py` — 3×3 `R` + 4×4 `T` layers, frame-tagged variable naming |
| [`phases/phase0_baselines.md`](phases/phase0_baselines.md) | Phase 0 — PNG baseline harness |
| [`phases/phase1_foundation.md`](phases/phase1_foundation.md) | Phase 1 — additive foundation modules |
| [`phases/phase2_v2_pipeline.md`](phases/phase2_v2_pipeline.md) | Phase 2 — parallel v2 pipeline |
| [`phases/phase3_catalog_migration.md`](phases/phase3_catalog_migration.md) | Phase 3 — catalog `_v2` (batched) |
| [`phases/phase4_demos.md`](phases/phase4_demos.md) | Phase 4 — demo `_v2` hooks |
| [`phases/phase5_cutover.md`](phases/phase5_cutover.md) | Phase 5 — cutover (delete old, rename) |
| [`phases/phase6_overlays.md`](phases/phase6_overlays.md) | Phase 6 — Scene / SceneHistory / Replay overlays |
| [`phases/phase7_collision.md`](phases/phase7_collision.md) | Phase 7 — collision reuse of the `tf` dict |

## Reading order

1. Skim `00_original_vision.md` for the why.
2. `01_master_overview.md` for the how and the phase gates.
3. `03_kinematics_core_math.md` for the reusable core math foundation.
4. `02_demo_use_cases.md` for the destination user API.
5. Execute one phase file at a time; each has its own validation gate + user review.

## Status

**Phase 0 complete** (branch `refactor-v4`): 36 PNG baselines + `manifest.json` +
`tests/unittest/test_kinematic_regression.py` (passing). Awaiting User Review 0
before Phase 1. Phases 1–7 pending.
