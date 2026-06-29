# Phase 2 — Parallel v2 Pipeline

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

## Deliverables

- [`animator2.py`](../../minilink/graphical/animation/animator2.py) — flatten +
  camera resolver → existing renderers
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
