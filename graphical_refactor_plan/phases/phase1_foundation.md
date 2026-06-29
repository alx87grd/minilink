# Phase 1 — Additive Foundation

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Core math
vision: [`../03_kinematics_core_math.md`](../03_kinematics_core_math.md).

New modules only; **old hooks and `Animator` untouched**. Nothing in this phase
changes rendered output.

## Modules

| Module | Band | Role |
| --- | --- | --- |
| [`minilink/core/kinematics.py`](../../minilink/core/kinematics.py) | core | two-layer SE algebra: 3×3 `Rx`/`Ry`/`Rz` + 4×4 `SE3`/`SE2`/`translation`/`identity`/`inv`/`apply` (`apply` = `apply_transform` renamed + relocated from [`robot.py:165`](../../minilink/planning/spatial/robot.py); fix call sites, no alias per pre-1.0 rule — see [`03_kinematics_core_math.md`](../03_kinematics_core_math.md)) |
| [`graphical/animation/primitives.py`](../../minilink/graphical/animation/primitives.py) | internal | primitive classes inc. new `Arrow`, `TorqueArrow` (honest geometry); `arrow_pts`/`torque_arc_pts` are private helpers used **inside** these classes |
| [`graphical/animation/visualization.py`](../../minilink/graphical/animation/visualization.py) | internal | `flatten_draw_list`, merge/prefix (animator-only) |
| [`graphical/animation/camera.py`](../../minilink/graphical/animation/camera.py) | internal | `resolve_camera_from_hints` + camera factories (`follow_frame_camera`, `fixed_camera`, `camera_matrix`) — re-exported publicly via `graphical.catalog` |
| [`graphical/catalog/shapes.py`](../../minilink/graphical/catalog/shapes.py) | **public** | curated primitives re-exported from `animation/primitives.py`: `Box`, `Circle`, `Line` (alias of `CustomLine`), `Arrow`, `Rod`, `Point`, `HorizonPolyline`, `TrajectoryPolyline`, … |
| [`graphical/catalog/skins.py`](../../minilink/graphical/catalog/skins.py) | **public** | skin functions `(plant) -> dict` **only**: `car_skin_2d`, `car_skin_3d`, `merge_skins`, `debug_state_skin` |

Add `GraphicPrimitive.local_transform` (old renderers ignore until cutover).

## Public catalog vs internal modules (organization principle)

Mirror `dynamics.catalog` (curated public plants) with a `graphical.catalog` of
curated public graphics. **Two catalogs, one package:**

```
minilink/graphical/catalog/
  __init__.py   # one-stop public re-export: shapes + skins + camera factories
  shapes.py     # user-facing primitive subset (re-exported from animation/primitives.py)
  skins.py      # skin functions (plant) -> dict only
```

- **Public (`graphical.catalog`)** — what demos and student plants import: shape
  primitives, skin functions, and camera factories. Stable, friendly names. The
  classes/functions live in `animation/`; the catalog is the curated re-export
  surface (mirrors how `dynamics.catalog` re-exports plants).
- **Internal (`graphical/animation/`)** — what renderer/library authors touch:
  full `primitives.py` implementation, `visualization.py` (`flatten_draw_list`),
  `camera.py` (resolver **and** camera factories), renderers. `flatten_draw_list`
  is **not public** — only the animator calls it.
- **Camera factories** (`follow_frame_camera`, `fixed_camera`, `camera_matrix`) are
  implemented in `animation/camera.py` and re-exported by `catalog/__init__.py` —
  **not** placed in `skins.py`. Overlay drawables (`SceneHistory`, `Replay`, and
  `Scene.as_visualizer`) join the public re-export when they land (overlay phase).

Rule of thumb: a primitive's **class** lives in `animation/primitives.py`;
`catalog/shapes.py` re-exports the ones students should reach for. Skins are
**pure functions** (no dataclasses) and live directly in `catalog/skins.py`.

Demo import: `from minilink.graphical.catalog import Box, Circle, Arrow, car_skin_3d`

**Transform style split:** `core/kinematics.py` symbols (`SE2`, `Rz`, …) serve
**catalog internals** and advanced reuse. Demo scripts and student `tf` overrides
use **inline 4×4** with `xp = array_module(x)`.

## `skin` attribute (LOCKED — Option B)

Swappable-look plants carry an opt-in attribute `skin` (a callable `(plant) ->
dict`, or `None`). The **final** contract method will delegate to it:

```python
# core/system.py — FINAL shape (installed at Phase 5, NOT Phase 1)
class System:
    skin = None  # opt-in; None => empty geometry

    def get_kinematic_geometry(self):
        return {} if self.skin is None else self.skin(self)
```

`skin` is to `get_kinematic_geometry` as `params` is to `f`: the method is the
contract the animator calls; the attribute is the policy it reads. Swap a look with
one assignment — `car2.skin = car_skin_3d` — and the method passes `self`
explicitly (no Python self-binding foot-gun). **Do not** reassign the method
(`sys.get_kinematic_geometry = fn`): instance-assigned functions don't receive
`self`. Tier 1 plants ignore `skin` and just override the method with an inline
dict.

> **Phasing (critical — do not install the block above in Phase 1).** In Phase 1
> only the `skin = None` **attribute** is added to base `System`; the old
> `get_kinematic_geometry` (debug points, [`system.py:339`](../../minilink/core/system.py))
> stays **byte-identical** so Phase 0 baselines are unchanged (execution principle
> #2; Phase 1 gate). The skin **delegation** lives on the `_v2` hook in Phases 2–4
> (`get_kinematic_geometry_v2` consults `self.skin` — see
> [`phase2`](phase2_v2_pipeline.md)) and only becomes the real
> `get_kinematic_geometry` body at the Phase 5 cutover (D2).

## Automated gate

Full pytest green; Phase 0 baselines unchanged.

## User Review 1

Review `kinematics.py` and `flatten_draw_list` against the vision doc.
