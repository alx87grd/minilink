# Phase 1 — Additive Foundation

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Core math
vision: [`../03_kinematics_core_math.md`](../03_kinematics_core_math.md).

New modules only; **old hooks and `Animator` untouched**. Nothing in this phase
changes rendered output.

## Modules

| Module | Band | Role |
| --- | --- | --- |
| [`minilink/core/kinematics.py`](../../minilink/core/kinematics.py) | core | JAX-functional transform algebra |
| [`graphical/animation/primitives.py`](../../minilink/graphical/animation/primitives.py) | internal | primitive classes inc. new `Arrow`, `TorqueArrow` (honest geometry); `arrow_pts`/`torque_arc_pts` are private helpers used **inside** these classes |
| [`graphical/animation/visualization.py`](../../minilink/graphical/animation/visualization.py) | internal | `flatten_draw_list`, merge/prefix (animator-only) |
| [`graphical/animation/camera.py`](../../minilink/graphical/animation/camera.py) | internal | `resolve_camera_from_hints` |
| [`graphical/catalog/shapes.py`](../../minilink/graphical/catalog/shapes.py) | **public** | curated primitives: `Box`, `Circle`, `Line`, `Arrow`, `Rod`, `Point`, … |
| [`graphical/catalog/skins.py`](../../minilink/graphical/catalog/skins.py) | **public** | skin functions `(plant) -> dict`: `car_skin_2d`, `car_skin_3d`, `merge_skins`, `debug_state_skin` + camera factories |

Add `GraphicPrimitive.local_transform` (old renderers ignore until cutover).

## Public catalog vs internal modules (organization principle)

Mirror `dynamics.catalog` (curated public plants) with a `graphical.catalog` of
curated public graphics. **Two catalogs, one package:**

```
minilink/graphical/catalog/
  __init__.py   # one-stop re-export of shapes + skins
  shapes.py     # re-exports the user-facing primitive subset
  skins.py      # skin functions + camera factories
```

- **Public (`graphical.catalog`)** — what demos and student plants import: shape
  primitives and skin/camera functions. Stable, friendly names.
- **Internal (`graphical/animation/`)** — what renderer/library authors touch:
  full `primitives.py` implementation, `visualization.py` (`flatten_draw_list`),
  `camera.py` resolver, renderers, builders. `flatten_draw_list` is **not public**
  — only the animator calls it.

Rule of thumb: a primitive's **class** lives in `animation/primitives.py`;
`catalog/shapes.py` re-exports the ones students should reach for. Skins are
**pure functions** (no dataclasses) and live directly in `catalog/skins.py`.

Demo import: `from minilink.graphical.catalog import Box, Circle, Arrow, car_skin_3d`

**Transform style split:** `core/kinematics.py` helpers (`pose2d_matrix`, …) serve
**catalog internals** and advanced reuse. Demo scripts and student `tf` overrides
use **inline 4×4** with `xp = array_module(x)`.

## `skin` attribute (LOCKED — Option B)

Swappable-look plants carry an opt-in attribute `skin` (a callable `(plant) ->
dict`, or `None`). The contract method delegates:

```python
# core/system.py — base
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

> Note: in Phase 1 this `skin` attribute is added as part of the additive surface,
> but the **base default stays on the old debug-point hooks until Phase 5** (D2).
> Only the `_v2` hooks (Phase 2) return `{}`.

## Automated gate

Full pytest green; Phase 0 baselines unchanged.

## User Review 1

Review `kinematics.py` and `flatten_draw_list` against the vision doc.
