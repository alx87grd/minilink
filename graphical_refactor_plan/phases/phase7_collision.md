# Phase 7 — Collision Reuse

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Core math:
[`../03_kinematics_core_math.md`](../03_kinematics_core_math.md). Vision:
[`../00_original_vision.md`](../00_original_vision.md) § *Collision reuse*.

**Status: complete.** Collision probes reuse the planner plant's ``sys.tf`` frame
dict; production demos bind frameless geometry at the planner.

---

## Goal

One forward-kinematics pass feeds both the rendered chassis and the clearance
probes: collision ``Shape``s attach to **frame keys** in ``sys.tf``; world probes
use :func:`~minilink.core.kinematics.apply`. Binding is **planner-only** — always
``bind(sys_mpc, …)`` or the single plant in a ``PlanningProblem``, never the
simulation/animation plant.

---

## Public API

| Symbol | Role |
| --- | --- |
| `disc(r)`, `car_outline(L, W, margin=0)`, `point_probe()` | Frameless body-frame geometry |
| `bind(sys, geometry, *, frame="body")` | Single-frame bind (default `"body"`) |
| `bind(sys, [("body", geom), ("link3", disc(r))])` | Multi-frame (list of pairs) |
| `sphere()`, `car()`, `point()` | Legacy state-indexed bodies (tests only) |

```python
body = bind(sys_mpc, car_outline(2.4, 0.2, margin=0.05))
field = scene.clearance_field(body)
```

---

## Implementation notes

- Internal storage: plain ``tuple[(frame_key, tuple[Shape, ...]), …]`` — no dataclass.
- :func:`~minilink.planning.spatial.collision.iter_probes` is the single loop for
  ``state_fields`` and ``plotting``.
- :func:`~minilink.core.kinematics.apply` uses column ``n-1`` when a 4×4 pose
  transforms 2-D points (planar embed).

---

## Automated gate

Spatial tests green; bound-path parity vs legacy state-indexed robots; planner
demos grep-clean for ``car(..., position=`` / ``sphere(..., position=``.

## User Review 7

Optional.
