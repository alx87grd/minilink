# Phase 2 — Parallel v2 Pipeline

Master plan: [`../01_master_overview.md`](../01_master_overview.md).

Add v2 entry points; **do not modify** old `Animator`, old hooks, or base-class old
defaults.

## Hooks (empty until Phase 3)

```python
# system.py — OLD hooks unchanged (debug points stay)
def get_kinematic_geometry_v2(self): return {}
def tf_v2(self, x, u, t=0, params=None): return {}
def get_dynamic_geometry_v2(self, x, u, t=0, params=None): return {}
```

## Deliverables

- [`animator2.py`](../../minilink/graphical/animation/animator2.py) — flatten +
  camera resolver → existing renderers
- [`facades.py`](../../minilink/core/facades.py) — `show_v2()`, `animate_v2()`
- Polylines: add `points_at(t)`; v2 path uses honest `t` (old path keeps `T[3,3]`
  until Phase 5)

## Automated gate

Phase 0 baselines green; `Animator2` smoke on empty sys.

## User Review 2

`DynamicBicycle.animate()` unchanged; `animate_v2()` empty until Phase 3.
