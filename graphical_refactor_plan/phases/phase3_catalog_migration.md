# Phase 3 — Catalog `_v2` (Batched)

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Core math:
[`../03_kinematics_core_math.md`](../03_kinematics_core_math.md).

Add `_v2` methods (`tf_v2`, `get_kinematic_geometry_v2`, `get_dynamic_geometry_v2`);
**old methods stay byte-identical**. Pixel parity per batch via
`sys.render(x,u,t)` vs `sys.render_v2(x,u,t)`.

`_v2` `tf` is written native-array / JAX-traceable (D1), consuming
`core/kinematics.py` for shared FK.

## Batches

| Batch | Files | Parity plants |
| --- | --- | --- |
| 3a Vehicles | `dynamic_bicycle`, `steering`, propulsion, … | DynamicBicycle + `skin=car_skin_3d`, KinematicBicycle |
| 3b Pendulum | pendulum, cartpole, double_pendulum | Pendulum, CartPole |
| 3c Manipulators | `arms.py` | TwoLink, FiveLink |
| 3d Rest | aerial, marine, integrators, engines, … | Drone2D, SingleMass |
| 3e Diagram | `diagram.py`, `modal.py` | diagram demo smoke |

Migration order follows the vision doc's catalog order: vehicles first (highest
skin payoff), then pendulum (torque-arc dynamic-geometry validation), then
manipulators (shared `_planar_*` helpers), then the rest, then diagram/modal call
sites.

## Automated gate

Per-batch: `render_v2()` pixel-identical to `render()` for the parity plants; full
pytest green; Phase 0 baselines unchanged.

**3a explicit gate:** `DynamicBicycle` with `skin=car_skin_3d` via `render_v2()` is
pixel-identical to the old `DynamicBicycleCar3D` via `render()` — this is the
invariant that lets Phase 5 retire the `DynamicBicycleCar3D` subclass.

## User Review 3a–3e

`sys.render(x,u,t)` vs `sys.render_v2(x,u,t)` side-by-side per batch.
