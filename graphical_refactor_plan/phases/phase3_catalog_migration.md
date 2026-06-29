# Phase 3 — Catalog `_v2` (Batched)

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Core math:
[`../03_kinematics_core_math.md`](../03_kinematics_core_math.md).

> **Status: 3a–3e complete.** All catalog plants, blocks, engines, and the
> diagram aggregator now carry `tf_v2` / `get_kinematic_geometry_v2` /
> `get_dynamic_geometry_v2`; the old methods are byte-identical. **Gate met:**
> `render(x,u,t)` vs `render_v2(x,u,t)` is **0 px** for every migrated plant
> (headless harness:
> [`scripts/check_render_v2_parity.py`](../../scripts/check_render_v2_parity.py)).
> Migrated:
> - **3a Vehicles** — `dynamic_bicycle.py` (`DynamicBicycle` 2-D +
>   `DynamicBicycleCar3D` 3-D + Jax variants), `steering.py`, `propulsion.py`,
>   `mountain_car.py`.
> - **3b Pendulum** — `pendulum.py` (`Pendulum`, `InvertedPendulum`,
>   `TwoIndependentPendulums`), `cartpole.py` (`CartPole`, `RotatingCartPole`,
>   `UnderactuatedRotatingCartPole`, `JaxCartPole`), `double_pendulum.py`
>   (`DoublePendulum`, `Acrobot`).
> - **3c Manipulators** — `arms.py` (speed-controlled, one/two/three-link,
>   five-link planar) via shared `_planar_*` helpers.
> - **3d Rest** — `mass_spring_damper/linear.py`, `aerial/{drone,rocket,plane}.py`,
>   `marine/boat.py`, `equations/integrators.py`, `blocks/transfer_function.py`,
>   `symbolic/mechanics/export.py`, and the JAX engines
>   `engines/{world,ancf_tire_jax}.py`.
> - **3e Diagram** — `core/diagram.py` aggregates subsystem frames/geometry with
>   `prefix_keys` namespacing; the full ordered draw list is built per subsystem
>   (kinematic-then-dynamic) in the dynamic hook so each subsystem reproduces its
>   own (validated) `render_v2` order, matching the legacy concatenation.
>
> **Pixel-identical primitives over honest redesign (per user).** `suspension.py`
> and the ANCF arrows/segments reuse the legacy stretch/scale transforms (spring
> via `line_between_transform`; ANCF arrows via the anisotropic
> `_vector_arrow_transform`) keyed to v2 frames, so no visual changes. The 2-D
> `TorqueArrowV2` is split into separate arc + head `Line2D` objects in the
> renderer to match legacy anti-aliasing. A camera parity bridge in
> `animation/camera.py` honors custom `get_camera_transform` overrides when no
> `camera_follow_frame` hint is set.
>
> **Deferred:** `dynamic_bicycle_SL.py` (pre-existing broken `tire_models` import,
> unimportable). **Known:** `dynamic_bicycle__rest` Phase-0 baseline drifts vs the
> uncommitted local baseline PNG (a 3a-era legacy artifact); `render() ==
> render_v2()` is still 0 px, so the refactor preserves parity — regenerate the
> baseline only on a deliberate, reviewed visual change.
>
> **Design decision (supersedes the original 3a gate).** Velocity/force arrows are
> *dynamic geometry*, not a skin concern, so they are written **per class**, not
> driven by a skin:
> - base `DynamicBicycle` → 2 centerline arrows (one per axle), matching its
>   legacy 2-D look (black centerline chassis via `car_skin_2d(color="black")`);
> - `DynamicBicycleCar3D` → overrides `get_dynamic_geometry_v2` with the 4
>   corner arrows (z-lifted to the hubs), matching the legacy 3-D look.
>
> Skins (`car_skin_2d` / `car_skin_3d`) cover **static** geometry only and share
> one bicycle `tf` (both axle and corner wheel frames; the 3-D body box bakes its
> ride-height/x-offset in `local_transform`). Consequence: the original gate
> "`DynamicBicycle + skin=car_skin_3d` ≡ old `Car3D`" is replaced by **subclass
> self-parity** (`Car3D.render` ≡ `Car3D.render_v2`), and `Car3D` is **not**
> retired into the base plant at Phase 5 (a static skin can't carry the 4-arrow
> dynamic set) — it stays a thin subclass.

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

**3a explicit gate (revised — see status banner):** subclass self-parity —
`DynamicBicycleCar3D.render_v2()` is pixel-identical to
`DynamicBicycleCar3D.render()` (the 3-D static skin *and* the 4 corner arrows),
and `DynamicBicycle.render_v2()` ≡ `DynamicBicycle.render()` for the 2-D look.
The arrows being a per-class `get_dynamic_geometry_v2` (not a skin) means `Car3D`
stays a thin subclass rather than collapsing into `base + car_skin_3d`.

## User Review 3a–3e

`sys.render(x,u,t)` vs `sys.render_v2(x,u,t)` side-by-side per batch.
