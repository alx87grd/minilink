# Phase 6 — Overlays (Scene / SceneHistory / Replay)

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Target shapes:
[`../02_demo_use_cases.md`](../02_demo_use_cases.md) (use cases 4, 7, 8).

Add the **overlay add-on contract** (the `System` hooks, time-only),
`Scene.as_visualizer()`, `SceneHistory`, `Replay`, and `animate(overlays=[...])`
(D3a). Replace the `MpcPlan*` demo subclasses with composition at the animation
boundary.

## Modules

| Module | Band | Role |
| --- | --- | --- |
| [`graphical/animation/drawables.py`](../../minilink/graphical/animation/drawables.py) | internal | `Overlay` base (time-only `tf(t)` / `get_kinematic_geometry()` / `get_dynamic_geometry(t)`, `{}` defaults), `SceneHistory`, `Replay`, and the `Shape -> primitive` obstacle-skin adapter backing `Scene.as_visualizer()` |
| [`graphical/catalog/__init__.py`](../../minilink/graphical/catalog/__init__.py) | **public** | re-exports `SceneHistory`, `Replay` so demos import overlays from `graphical.catalog` (alongside shapes/skins/camera factories) |
| [`planning/spatial/scene.py`](../../minilink/planning/spatial/scene.py) | planning | adds `as_visualizer()` — a thin method that **lazily imports** the graphical adapter; the frozen `Scene` keeps **zero** hard graphical dependency |

## Overlay contract (the `System` hooks, time-only)

Overlays are **not** a new contract: it is the **same three drawable hooks** with
`(x,u)` dropped — `tf(self, t)`, `get_kinematic_geometry(self)`,
`get_dynamic_geometry(self, t)` — each with a `{}` default so an add-on overrides
only what it draws. This is the one place the **Phase-5 `Animator`** (formerly
`Animator2`, renamed at cutover) is extended: it now accepts
`animate(overlays=[...])`, resolves hooks per drawable (primary: `x,u,t`; overlay:
`t` only), then calls the **same** `flatten_draw_list(frames, kinematic, dynamic)`
on each and concatenates `[primary] + overlays`. This preserves the kinematic(cached) vs
dynamic(rebuilt) split that retained-mode renderers (meshcat) need — no `is_dynamic`
flag, no `T[3,3]` channel. `Scene.as_visualizer()` overrides only
`get_kinematic_geometry()`; `SceneHistory` only `get_dynamic_geometry(t)`; `Replay`
forwards all three at its own `x(t)` (keeping the ghost's named frames + real skin).

## Scene direction (locked — Option B)

The existing spatial/collision `Scene`
([`planning/spatial/scene.py`](../../minilink/planning/spatial/scene.py)) stays a
**collision-first source** and does **not** implement either drawable contract.
Instead it **exports** a `t`-only overlay via `scene.as_visualizer()`,
symmetric with its existing `scene.clearance_field(robot).as_constraint()` and
`scene.cost_field(robot).as_cost()` exports. `as_visualizer()` **lazily imports**
the graphical adapter inside the method body — mirroring the existing `scene.plot()`
lazy-matplotlib pattern and DESIGN.md's "`graphical/` is imported lazily from
anywhere" rule — so planning never hard-depends on the graphical band. The
**graphical-band** adapter owns the *default obstacle skin* (maps each
`core.geometry` `Shape` to a default primitive posed in world). A raw `Scene` passed into `overlays` is rejected
(call `.as_visualizer()`) — no silent auto-adapt, preserving symmetry with
`.as_cost()`. Do **not** bolt drawable hooks onto the frozen `Scene` dataclass.

Graphics-only, time-indexed data (reference path, MPC futures, executed trail)
lives in `SceneHistory`, **not** in `Scene`:

- `SceneHistory(horizon=HorizonPolyline(plans), trail=TrajectoryPolyline(traj), ...)`
  — overrides only `get_dynamic_geometry(t)`, building world polylines via `points_at(t)`.
- `Replay(drawable, trajectory)` — forwards the wrapped drawable's three hooks at
  its own `x(t)` (full-skin ghost, named frames preserved).

Rendering-only objects compose only on the **render axis** (`animate(overlays=...)`),
never as diagram blocks (a stateless, port-less block would pollute the `f`/`h` /
state-stacking contract). A future `Scene.tf(t)` for moving obstacles (shared by
collision + render) slots in behind `as_visualizer()` without changing the API;
deferred to the planner upgrade.

## Boilerplate removed

No `MpcPlan*` subclass, no `time_channel_matrix`, no manual transform-list
alignment — overlays compose at `animate(overlays=[scene.as_visualizer(), history])`.

## Automated gate

Overlay parity: MPC/trajopt demos render the same pixels as their Phase 4 forms
with less code.

## User Review 6

Architectural review + multi-obstacle MPC demo.
