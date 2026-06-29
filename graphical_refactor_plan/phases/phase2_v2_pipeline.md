# Phase 2 — Parallel v2 Pipeline

> **Status: complete.** Shipped: `_v2` hooks on `System` (`tf_v2`,
> `get_kinematic_geometry_v2` delegating to `skin`, `get_dynamic_geometry_v2`),
> `camera_follow_frame` / `camera_priority` hints, `animation/animator2.py`
> (`Animator2`), `facades.render_v2()` / `animate_v2()`, and additive
> `ArrowV2`/`TorqueArrowV2` branches in all four renderers (old branches
> untouched). Old `Animator`, old hooks, and base old defaults unchanged; full
> pytest green, empty-sys `Animator2` smoke passes. Awaiting User Review 2.

Master plan: [`../01_master_overview.md`](../01_master_overview.md).

Add v2 entry points; **do not modify** old `Animator`, old hooks, or base-class old
defaults.

## Hooks (empty until Phase 3)

```python
# system.py — OLD hooks unchanged (debug points stay)
def get_kinematic_geometry_v2(self):
    # base v2 hook honors the skin attribute from Phase 1 (this is the
    # delegation that becomes the FINAL get_kinematic_geometry body at Phase 5)
    return {} if self.skin is None else self.skin(self)
def tf_v2(self, x, u, t=0, params=None): return {}
def get_dynamic_geometry_v2(self, x, u, t=0, params=None): return {}
```

> The v2 kinematic hook **must** consult `self.skin` (not a flat `{}`); Phase 3a's
> parity plant is `DynamicBicycle` + `skin=car_skin_3d`, which only renders through
> the v2 pipeline if the skin attribute is read here.

## Camera hint attributes (additive, old path untouched)

Add the two new camera hints to `System.__init__` alongside the existing
`camera_target` / `camera_plot_axes` / `camera_scale`
([`system.py:109`](../../minilink/core/system.py)); `Animator2`'s resolver reads
them, the old `Animator` ignores them:

```python
self.camera_follow_frame = None   # str key into tf(), or None
self.camera_priority = 0.0        # tie-break when several sources exist
```

## Animator2 lifecycle (built here)

`Animator2` is the v2 **orchestrator** — the only new class that ties the Phase 1
helpers to the existing renderers. It is **built in this phase** (Phase 1 shipped
only its ingredients: `flatten_draw_list` and `resolve_camera_from_hints`). Per
frame it:

1. resolves the **primary** drawable's hooks — `frames = sys.tf_v2(x,u,t)`,
   `kinematic = sys.get_kinematic_geometry_v2()` (cached once),
   `dynamic = sys.get_dynamic_geometry_v2(x,u,t)`;
2. calls `flatten_draw_list(frames, kinematic, dynamic)` → flat `(prim, world 4x4)`
   list (unzipped into the `(primitives, transforms)` the existing
   `renderer.draw_frame` already takes — renderers reused, only the additive
   honest-primitive branches added);
3. `camera = resolve_camera_from_hints(...)` (honors `animate(camera=...)` override).

Its scope across phases (so it is clear when this class changes):

| Phase | Animator2 |
| --- | --- |
| 2 | **built** — single **primary** drawable only (no overlays) |
| 3–4 | **no change** — signature-agnostic, so catalog/demo `_v2` hooks "just work" |
| 5 | **renamed** to `Animator` (old `Animator` deleted, `_v2` dropped) |
| 6 | **extended** — accept `overlays=[...]`, resolve hooks per drawable (primary `x,u,t`; overlays `t`-only), run the same `flatten_draw_list` on each, concatenate `[primary] + overlays`, add camera source-selection (Layer 4) |

## Frame model: per-frame draw list (design for it now)

The old pipeline computes `primitives = get_kinematic_geometry()` **once**
([`animator.py:169`](../../minilink/graphical/animation/animator.py)) and threads
that single fixed list into every builder; each `frame` carries only
`transforms`+`camera`. Reviving `get_dynamic_geometry` means **geometry changes per
frame**, so `Animator2`'s frame model must generalize: each frame carries its **own**
flattened `(primitives, transforms)` pair (kinematic cached once + dynamic rebuilt
per frame, via `flatten_draw_list`). `draw_frame(primitives, transforms, …)` already
takes per-frame primitives, so its **signature is unchanged**; the change is that
the **native/inline/export builders** must read per-frame primitives instead of one
fixed list:

| Renderer path | Change needed | Dynamic-geometry support |
| --- | --- | --- |
| matplotlib non-native loop, pygame (immediate) | none beyond honest-primitive branches (`draw_frame` re-flattens each frame) | full |
| matplotlib `FuncAnimation` / inline / export | small: `update()` reads `frame["primitives"]` (it already `clear()`s each frame) | full |
| meshcat / plotly **native** (retained-mode) | none for the static layer (upload-once + `set_transform`) | dynamic stays **frozen at t=0** — pre-existing limitation (DESIGN §4.7), no regression |

> Design `Animator2` for the per-frame primitive list **from Phase 2** even though
> the dynamic hook returns `{}` here (primitives are effectively static until
> Phase 3b's torque arc / Phase 4 overlays) — otherwise the frame model has to be
> reworked at 3b. The retained-mode "re-stream dynamic geometry" optimization is a
> later, renderer-internal item, **not** this refactor.

## Deliverables

- [`animator2.py`](../../minilink/graphical/animation/animator2.py) — the
  orchestrator above: resolve primary hooks → `flatten_draw_list` → camera resolver
  → existing renderers (per-frame draw list)
- **Renderer honest-primitive branches** — add additive `isinstance` branches for
  `ArrowV2`/`TorqueArrowV2` to the renderers' `draw_primitive` (the honest arrows
  are a distinct type from the legacy ones — see [phase1](phase1_foundation.md)
  *Honest vs legacy primitives*). **Old branches untouched** so the old path stays
  pixel-identical; only the v2 path exercises the new branches.
- Extend existing [`facades.py`](../../minilink/core/facades.py) (the
  `render`/`animate` shortcuts already live here) with `render_v2()` and
  `animate_v2()` (single-frame parity uses `render`/`render_v2`, mirroring the
  existing `render` shortcut at [`facades.py:368`](../../minilink/core/facades.py))
- Polylines: add `points_at(t)`; v2 path uses honest `t` (old path keeps `T[3,3]`
  until Phase 5)

## Automated gate

Phase 0 baselines green; `Animator2` smoke on empty sys.

## User Review 2

`DynamicBicycle.animate()` unchanged; `animate_v2()` / `render_v2()` empty until Phase 3.
