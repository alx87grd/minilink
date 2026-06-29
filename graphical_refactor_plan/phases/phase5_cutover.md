# Phase 5 — Cutover

Master plan: [`../01_master_overview.md`](../01_master_overview.md).

**The single phase that touches the old API.** After this, `_v2` no longer exists.

## Steps

1. Delete old hooks + `Animator` + channel hacks (`time_channel_matrix`,
   `scale_pose2d_matrix`, `arrow_transform`, `line_between_transform`,
   `torque_pose2d_matrix`, `extract_amplitude`).
2. Rename `_v2` → final names (`tf`, `get_kinematic_geometry`,
   `get_dynamic_geometry`, `Animator`, `animate`, `render`).
3. Base `System` `get_kinematic_geometry` body → the skin-delegating form (D2);
   the `skin` attribute itself was already added in Phase 1 (`None` default).
4. **Catalog consolidation:** retire `DynamicBicycleCar3D` subclass — one
   `DynamicBicycle` class with `skin = car_skin_2d` default; 3D look via
   `skin = car_skin_3d` (see [`../02_demo_use_cases.md`](../02_demo_use_cases.md)).
5. Update baselines, [`DESIGN.md`](../../DESIGN.md),
   [`README.md`](../../README.md)/[`ROADMAP.md`](../../ROADMAP.md), and the
   graphics tests that exercise the deleted contracts:
   - **`test_camera_transform.py`** — tests the deleted `get_camera_transform`;
     rewrite against camera hints + resolver (or delete if covered by Animator2 tests).
   - **`test_catalog_migration.py`**, **`test_dynamic_bicycle_graphics.py`** —
     update to `tf` / keyed-dict geometry + `skin`.
   - **`test_pendulum_plants.py`**, **`test_engine_world_system.py`**,
     **`test_ancf_tire_jax.py`**, **`test_symbolic.py`** — update any
     `get_kinematic_transforms` / `time_channel_matrix` references.

## Automated gate

Full pytest; Phase 0 baselines match; **zero `_v2`** remaining (grep-clean).

## User Review 5

Full catalog demo sweep; approve merge.
