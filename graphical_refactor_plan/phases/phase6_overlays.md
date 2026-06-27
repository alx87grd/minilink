# Phase 6 — Overlays (Scene / SceneHistory / Replay)

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Target shapes:
[`../02_demo_use_cases.md`](../02_demo_use_cases.md) (use cases 4, 7, 8).

Add `Scene`, `SceneHistory`, `Replay`, and `animate(overlays=[...])` (D3a). Replace
the `MpcPlan*` demo subclasses with composition at the animation boundary.

## Scene direction (locked)

The existing spatial/collision `Scene`
([`planning/spatial/scene.py`](../../minilink/planning/spatial/scene.py)) is
**primary as a collision object** and **gains the drawable hooks as a secondary
aspect** (like `System` is primary dynamics + secondary rendering). Mirror the
contract onto that Scene; do **not** create a graphics-only Scene.

Graphics-only, time-indexed data (reference path, MPC futures, executed trail)
lives in `SceneHistory`, **not** in `Scene`:

- `SceneHistory(horizon=HorizonPolyline(plans), trail=TrajectoryPolyline(traj), ...)`
  — frames `{"world": I}`, geometry via each primitive's `points_at(t)`.
- `Replay(drawable, trajectory)` — full-skin ghost posed at `x(t)`.

## Boilerplate removed

No `MpcPlan*` subclass, no `time_channel_matrix`, no manual transform-list
alignment — overlays compose at `animate(overlays=[scene, history])`.

## Automated gate

Overlay parity: MPC/trajopt demos render the same pixels as their Phase 4 forms
with less code.

## User Review 6

Architectural review + multi-obstacle MPC demo.
