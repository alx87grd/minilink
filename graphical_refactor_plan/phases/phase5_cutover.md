# Phase 5 — Cutover

Master plan: [`../01_master_overview.md`](../01_master_overview.md).

**The single phase that touches the old API.** After this, `_v2` no longer exists.

## Steps

1. Delete old hooks + `Animator` + channel hacks (`time_channel_matrix`,
   `scale_pose2d_matrix`, `arrow_transform`, `line_between_transform`,
   `torque_pose2d_matrix`, `extract_amplitude`).
2. Rename `_v2` → final names (`tf`, `get_kinematic_geometry`,
   `get_dynamic_geometry`, `Animator`, `animate`, `show`).
3. Base `System` defaults → `{}` (D2) on the final hooks; add opt-in `skin`
   attribute to base `System` (`None` default).
4. **Catalog consolidation:** retire `DynamicBicycleCar3D` subclass — one
   `DynamicBicycle` class with `skin = car_skin_2d` default; 3D look via
   `skin = car_skin_3d` (see [`../02_demo_use_cases.md`](../02_demo_use_cases.md)).
5. Update tests, baselines, [`DESIGN.md`](../../DESIGN.md),
   [`README.md`](../../README.md)/[`ROADMAP.md`](../../ROADMAP.md).

## Automated gate

Full pytest; Phase 0 baselines match; **zero `_v2`** remaining (grep-clean).

## User Review 5

Full catalog demo sweep; approve merge.
