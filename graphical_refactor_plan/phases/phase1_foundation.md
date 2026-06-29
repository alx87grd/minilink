# Phase 1 — Additive Foundation

> **Status: complete.** All modules below shipped (`core/kinematics.py` with
> `apply` relocated from `robot.py`; `local_transform` + `points_at` on
> primitives; honest `shapes_v2`; `visualization.flatten_draw_list`;
> `animation/camera`; public `graphical/catalog`; `System.skin`). Full pytest
> green and ruff clean; old render path unchanged. Awaiting User Review 1.

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Core math
vision: [`../03_kinematics_core_math.md`](../03_kinematics_core_math.md).

New modules only; **old hooks and `Animator` untouched**. Nothing in this phase
changes rendered output.

## Modules

| Module | Band | Role |
| --- | --- | --- |
| [`minilink/core/kinematics.py`](../../minilink/core/kinematics.py) | core | two-layer SE algebra: 3×3 `Rx`/`Ry`/`Rz` + 4×4 `SE3`/`SE2`/`translation`/`identity`/`inv`/`apply` (`apply` = `apply_transform` renamed + relocated from [`robot.py:165`](../../minilink/planning/spatial/robot.py); fix call sites, no alias per pre-1.0 rule — see [`03_kinematics_core_math.md`](../03_kinematics_core_math.md)) |
| [`graphical/animation/primitives.py`](../../minilink/graphical/animation/primitives.py) | internal | **legacy classes stay byte-identical** (`Arrow`, `TorqueArrow`, polylines, camera helpers, hacks — used by ~20 catalog files + renderers + Phase 0 baselines). Add only: `GraphicPrimitive.local_transform`, and a `points_at(t)` **method** on the polylines (additive; old `compute_pts` path untouched). |
| [`graphical/animation/shapes_v2.py`](../../minilink/graphical/animation/shapes_v2.py) | internal | **NEW honest primitives** (LOCKED — separate classes): `ArrowV2`, `TorqueArrowV2` with the honest constructor (`base`/`vector`/`scale`) that bake their own geometry via private `arrow_pts`/`torque_arc_pts` (no column-norm scale, no `T[3,3]` channel). Distinct from the legacy `Arrow`/`TorqueArrow`; the `_v2` suffix is grep-cleaned at Phase 5 when they replace the legacy classes in `primitives.py`. |
| [`graphical/animation/visualization.py`](../../minilink/graphical/animation/visualization.py) | internal | `flatten_draw_list` + merge/prefix helpers (animator-only). **New logic, not a renamed old path:** see *`flatten_draw_list` role* below. |
| [`graphical/animation/camera.py`](../../minilink/graphical/animation/camera.py) | internal | **NEW** `resolve_camera_from_hints` + factories (`follow_frame_camera`, `fixed_camera`); **re-exports** the existing `camera_matrix`/`world_to_camera` from `primitives.py` (physical relocation + delete deferred to Phase 5, so `system.py`/renderer imports stay valid). |
| [`graphical/catalog/shapes.py`](../../minilink/graphical/catalog/shapes.py) | **public** | re-exports the stable primitives from `animation/primitives.py` (`Box`, `Circle`, `Line` = `CustomLine`, `Rod`, `Point`, `HorizonPolyline`, `TrajectoryPolyline`, …) **and** the honest `ArrowV2`/`TorqueArrowV2` under the public names `Arrow`/`TorqueArrow` (so demos import the honest arrows; the legacy `animation.primitives.Arrow` is never the public one). |
| [`graphical/catalog/skins.py`](../../minilink/graphical/catalog/skins.py) | **public** | skin functions `(plant) -> dict` **only**: `car_skin_2d`, `car_skin_3d`, `merge_skins`, `debug_state_skin`. Authored against the target frame vocabulary; **not pixel-validated until batch 3a** wires them. |

Add `GraphicPrimitive.local_transform` (plain `np.ndarray` identity default; old
renderers ignore it until cutover).

## Honest vs legacy primitives (LOCKED — separate classes)

`Arrow`/`TorqueArrow` **already exist** in `primitives.py` (unit + column-norm
scaling / `T[3,3]` sweep channel) and are used by the frozen old pipeline and the
Phase 0 baselines, so they **cannot be reworked in place** without breaking parity.
Decision: introduce the honest-geometry versions as **distinct new classes**
(`ArrowV2`/`TorqueArrowV2`), used **only by the `_v2` path**; legacy classes stay
untouched through Phase 4. Consequences:

- The public catalog (`graphical.catalog.Arrow`) is the **honest** class; the
  legacy `animation.primitives.Arrow` is internal-only and never re-exported.
- Because the honest primitives are a different type, the **renderers gain
  additive `isinstance` branches** for them in **Phase 2** (old branches untouched
  → old path stays pixel-identical). This is the one renderer edit in the v2 work.
- **Phase 5** deletes the legacy `Arrow`/`TorqueArrow` + hacks, moves the honest
  classes into `primitives.py` under the final names `Arrow`/`TorqueArrow`
  (dropping the `V2` suffix), and `catalog/shapes.py` returns to a plain re-export.

## `flatten_draw_list` role (new keyed-dict → flat-list adapter)

`flatten_draw_list` is **new logic**, not a v2 rename of the existing renderer
path. Today the contract is already flat and index-aligned: `get_kinematic_geometry()`
returns a **list** and `get_kinematic_transforms(x,u,t)` a **list**, and the
renderer just `zip`s them 1:1 ([`matplotlib_renderer.draw_frame`](../../minilink/graphical/animation/renderers/matplotlib_renderer.py)),
so there is no "flatten" step. The v2 hooks instead return **frame-keyed dicts**
(`tf -> dict[key, 4x4]`, geometry `-> dict[key, [prim]]`), so a new adapter must
turn them back into the flat `(primitive, world 4x4)` list the renderers already
consume:

- input: three already-resolved dicts `(frames, kinematic, dynamic)` — the helper
  is **signature-agnostic** (it never sees `x,u,t`), which is what lets the
  primary *and* the time-only overlays share it (Phase 6);
- output: for each `key`, `(prim, frames[key] @ prim.local_transform)` for every
  primitive at that key, merging the cached kinematic dict with the rebuilt
  dynamic dict;
- validates geometry keys ⊆ frame keys (the "unknown key fails loudly" rule);
  `"world"` is always treated as present.

It is the v2 analog of the old `Animator._prepare_transforms` index-alignment, plus
the `local_transform` compose and the kinematic/dynamic merge. **Phase 1 ships only
the helper**; the orchestrator that calls it (`Animator2`) is built in Phase 2 —
see *Animator2 lifecycle* in [`phase2`](phase2_v2_pipeline.md).

## Public catalog vs internal modules (organization principle)

Mirror `dynamics.catalog` (curated public plants) with a `graphical.catalog` of
curated public graphics. **Two catalogs, one package:**

```
minilink/graphical/catalog/
  __init__.py   # one-stop public re-export: shapes + skins + camera factories
  shapes.py     # stable primitives re-exported from animation/primitives.py,
                # PLUS honest Arrow/TorqueArrow re-exported from animation/shapes_v2.py
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
- **Camera factories** (`follow_frame_camera`, `fixed_camera`) are **new** in
  `animation/camera.py`; `camera_matrix`/`world_to_camera` stay in `primitives.py`
  for now and `camera.py` re-exports them (physical move at Phase 5). All are
  re-exported by `catalog/__init__.py` — **not** placed in `skins.py`. Overlay
  drawables (`SceneHistory`, `Replay`, and `Scene.as_visualizer`) join the public
  re-export when they land (overlay phase).

Rule of thumb: a primitive's **class** lives in `animation/` (`primitives.py` for
stable/legacy shapes, `shapes_v2.py` for the honest `Arrow`/`TorqueArrow` until
Phase 5 folds them into `primitives.py`); `catalog/shapes.py` re-exports the ones
students should reach for. Skins are **pure functions** (no dataclasses) and live
directly in `catalog/skins.py`.

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
