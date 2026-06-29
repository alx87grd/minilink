# Target Demo-Script Shapes (Final Public API)

Ground truth: [`00_original_vision.md`](00_original_vision.md). Plan of record:
[`01_master_overview.md`](01_master_overview.md).

Examples below show **post–Phase 5/6** scripts — the destination demos should
reach. During Phases 2–4, substitute `animate_v2` and `_v2` hooks where noted.

---

## Script ergonomics principles

These rules govern all demo-script examples and catalog student-facing code:

| Concern | Student / demo style | Library-internal style |
| --- | --- | --- |
| **Input ports** | `self.add_input_port("steer", labels=["delta"])` — already on `System`; no `InputPort` import | `InputPort` in `core/signals.py` |
| **Shapes & skins** | `from minilink.graphical.catalog import Box, Circle, Arrow, car_skin_3d` — public catalog (`shapes` + `skins`) | `graphical/animation/` primitives/visualization/renderers for library authors |
| **`tf` math** | Inline 4×4 with `xp = array_module(x)`, `xp.cos` / `xp.stack` — reads like textbook SE(2); tag locals with frame keys (`W_T_body`, `body_T_wheel`) | `SE2`, `Rz`, … symbols in `core/kinematics.py` for shared catalog helpers |
| **Vehicle look** | Assign the **`skin`** attribute (callable `(plant) -> dict`); contract hook stays `get_kinematic_geometry` | Skin functions in `graphical/catalog/skins.py` |

**Skin swap — contract hook + opt-in `skin` attribute (LOCKED, Option B):**

The animator only ever calls `get_kinematic_geometry() -> dict[str, list[prim]]`.
Skins are **pure functions** `(plant) -> dict` in `graphical.catalog`. The base
method delegates to an opt-in `skin` attribute; swapping is one assignment.

```python
# core/system.py — base contract
class System:
    skin = None
    def get_kinematic_geometry(self):
        return {} if self.skin is None else self.skin(self)

# dynamic_bicycle.py — class-level default
class DynamicBicycle(DynamicSystem):
    skin = car_skin_2d

# demo: swap look — same tf(), same f()
car2 = DynamicBicycle()
car2.skin = car_skin_3d
```

`skin : get_kinematic_geometry :: params : f` — the method is the contract, the
attribute is the policy it reads. The method passes `self` explicitly, so no
Python self-binding gotcha.

**Reject** `sys.get_kinematic_geometry = fn`: a function assigned to an *instance*
is not bound, so it gets no `self` and a `(plant)` skin breaks. Use the `skin`
attribute (or a subclass override) instead. **`DynamicBicycleCar3D` is retired**
(migration debt today).

**Tier 1 / one-off (subclass override — `skin` not needed):**

```python
class CrimsonCar(DynamicBicycle):
    skin = lambda self: car_skin_3d(self, color="crimson")   # or override the method
```

---

## Use case 1 — Quick-and-dirty custom plant (Tier 1)

Minimal ceremony: ports via `System` helpers, shapes from the friendly catalog,
`tf` as explicit matrix math.

```python
"""examples/scripts/custom/demo_my_robot.py"""
from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem
from minilink.graphical.catalog import Box, Circle


class MyRobot(DynamicSystem):
    def __init__(self):
        DynamicSystem.__init__(self, 3, name="MyRobot")
        self.L, self.W = 1.2, 0.6
        self.add_input_port("steer", labels=["delta"])

    def f(self, x, u, t=0, params=None):
        # ... textbook dynamics ...
        ...

    def get_kinematic_geometry(self):
        return {
            "body": [Box(length_x=self.L, length_y=self.W, color="steelblue")],
            "wheel": [Circle(radius=0.15, color="black")],
        }

    def tf(self, x, u, t=0, params=None):
        xp = array_module(x)
        c, s = xp.cos(x[2]), xp.sin(x[2])
        W_T_body = xp.array([
            [c, -s, 0.0, x[0]],
            [s,  c, 0.0, x[1]],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        delta = u[0]
        cd, sd = xp.cos(delta), xp.sin(delta)
        body_T_wheel = xp.array([
            [cd, -sd, 0.0, self.L * 0.35],
            [sd,  cd, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        return {"body": W_T_body, "wheel": W_T_body @ body_T_wheel}


if __name__ == "__main__":
    robot = MyRobot()
    traj = robot.compute_trajectory(t_f=5.0)
    robot.animate(traj)
```

**Conventions:** frame-tagged locals (`W_T_body`, `body_T_wheel`) — see
[`03_kinematics_core_math.md`](03_kinematics_core_math.md). Fixed offset on same
frame → `prim.local_transform` or primitive `center`; not a new frame key.
Catalog shared FK may use `core/kinematics` helpers internally — demos show the
matrix directly.

---

## Use case 2 — Swapping vehicle skin (Tier 2)

**One plant class, two looks** — dynamics and `tf` unchanged; swap the `skin`
attribute. No `DynamicBicycleCar3D` subclass.

```python
"""Same trajectory, two skins — notebook cell or script."""
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle
from minilink.graphical.catalog import car_skin_3d

car1 = DynamicBicycle()        # class default: skin = car_skin_2d

car2 = DynamicBicycle()
car2.skin = car_skin_3d        # swap look — same tf(), same f()

traj = car1.compute_trajectory(t_f=8.0)
car1.animate(traj, scene_title="Centerline skin")
car2.traj = traj
car2.animate(traj, scene_title="Four-wheel 3D skin")

# Custom skin without a subclass — a plain function (plant) -> dict:
car2.skin = lambda p: car_skin_3d(p, color="crimson")
```

**Payoff:** deletes duplicated `tf` and the entire `DynamicBicycleCar3D` subclass.
Three independent axes stay separate: `f` (dynamics), `tf` (frames), `skin`
(visual). Animator still calls only `get_kinematic_geometry()`.

**Phase 0 note:** baselines may still snapshot `DynamicBicycleCar3D` until Phase 5;
parity tests migrate to `DynamicBicycle` with `skin=car_skin_3d` before cutover.

---

## Use case 3 — Dynamic arrows / torque (instantaneous geometry)

`Arrow` is a **first-class primitive** (in `catalog.shapes`), not a hand-built
`Line`. It owns its geometry (no `T[3,3]` scaling hack); the frame supplies world
orientation, so no manual `c,s = cos/sin(theta)` in the demo:

```python
# Inside DynamicBicycle (catalog — not demo boilerplate)
from minilink.graphical.catalog import Arrow

def get_dynamic_geometry(self, x, u, t=0, params=None):
    v_local = x[3:5]  # already body-frame
    return {
        "body": [Arrow(base=(0.0, 0.0), vector=v_local, scale=0.25, color="red")]
    }
```

`Arrow` and `TorqueArrow` are honest primitives (Phase 1/2): they build their own
point geometry internally via private `arrow_pts` / `torque_arc_pts`; the renderer
just draws them at `frames[key] @ local_transform`. Pendulum torque arc keys to
`"link0"` — frame orientation carries the arc; demo script stays:

```python
pend = Pendulum()
pend.u[:] = 0.5
traj = pend.compute_trajectory(t_f=3.0)
pend.animate(traj)
```

---

## Use case 4 — Export rendering info (inspect without drawing)

There is **one drawable contract** — `tf`, `get_kinematic_geometry`,
`get_dynamic_geometry` — and the **only** difference is the driving signal. A
**state-driven drawable** (`System`/`DiagramSystem`) is a function of `(x,u,t)`; an
**overlay add-on** (`Scene.as_visualizer()`, `SceneHistory`, `Replay`) is the same
hooks **time-only** — `tf(t)`, `get_kinematic_geometry()`, `get_dynamic_geometry(t)`
— with `{}` defaults so it overrides just the one it draws. The animator runs the
same internal `flatten_draw_list` on both (not public — demos use the hooks directly).

```python
"""Read a state-driven drawable's rendering info at one instant — public API only."""
import numpy as np
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle

sys = DynamicBicycle()
x = np.array([1.0, 2.0, 0.3, 5.0, 0.0, 0.0])
u = np.array([15.0, 0.05])
t = 1.25

frames = sys.tf(x, u, t)                  # dict[str, 4x4 world]
skin = sys.get_kinematic_geometry()       # dict[str, list[prim]]
dynamic = sys.get_dynamic_geometry(x, u, t)

for key, T in frames.items():
    print(key, T[:3, 3])                  # world origin of each frame
```

**Scene — collision-first source; visualization via `as_visualizer()` (Phase 6):**

A `Scene` (in [`planning/spatial/scene.py`](../minilink/planning/spatial/scene.py))
is **a collision-first spatial source** (`obstacles`, `workspace_fields`). It does
**not** implement the drawable contract; it **exports** a drawable the same way it
already exports a set/cost — so a raw `Scene` is never passed where a drawable is
expected (the animator rejects it with a "call `scene.as_visualizer()`" message,
symmetric with `.as_cost()`).

```python
from minilink.planning.spatial.scene import Scene

scene = Scene(obstacles=obstacles)                            # collision-first source

# three symmetric exports — Scene stays a pure planning object:
constraint = scene.clearance_field(robot).as_constraint()    # -> Set
cost       = scene.cost_field(robot).as_cost()               # -> CostFunction
viz        = scene.as_visualizer()                           # -> overlay (t-only hooks)
```

`scene.as_visualizer()` returns a **graphical-band** drawable whose `tf` is empty
(world-fixed) and whose `get_kinematic_geometry()` maps each obstacle `core.geometry`
`Shape` to a default primitive keyed to `world` (the *default obstacle skin*;
`as_visualizer(style=...)` can override). The collision data stays the single
source of truth and the picture is a view onto it — one obstacle set feeds both
clearance probes and the rendering. (A future `Scene.tf(t)` for moving obstacles
slots in behind this export without changing the API; deferred to the planner
upgrade.)

**SceneHistory — overlay add-on (Phase 6):** time-indexed plan/trail data that is
*not* collision geometry (MPC futures, executed trail) lives in an overlay that
overrides only `get_dynamic_geometry(t)` (geometry rebuilt via `points_at(t)`):

```python
from minilink.graphical.catalog import HorizonPolyline, SceneHistory, TrajectoryPolyline

history = SceneHistory(
    horizon=HorizonPolyline(mpc_plans, color="orange", linewidth=1.5),
    trail=TrajectoryPolyline(executed_traj, color="green"),
)
dynamic = history.get_dynamic_geometry(t=playback_t)  # dict[key, [prim]] via points_at(t)
```

**Replay — full-skin ghost (Phase 6):** an overlay wrapping another drawable + its
own trajectory; it **forwards** the wrapped drawable's three hooks at its own `x(t)`,
so the ghost keeps its named frames and real skin:

```python
from minilink.graphical.catalog import Replay

ghost = Replay(other_robot, other_traj)
frames = ghost.tf(t=now)                  # other_robot.tf(*traj.sample(now), now)
skin = ghost.get_kinematic_geometry()     # other_robot's skin (cached)
```

---

## Use case 5 — Quick camera tweaks (hints as data)

No method override — set attributes before `animate()`:

```python
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle

car = DynamicBicycle()
car.camera_scale = 14.0
car.camera_target[:] = (0.0, 0.0, 0.0)   # look-at offset (added to follow frame)
car.camera_follow_frame = "body"          # follow vehicle
car.camera_plot_axes = (0, 1)             # XY plane

traj = car.compute_trajectory(t_f=10.0)
car.animate(traj)
```

Diagram auto-selects plant camera (controller has empty skin, lower priority):

```python
closed_loop = controller >> plant  # plant.camera_follow_frame wins
closed_loop.animate(traj)
```

---

## Use case 6 — Advanced camera (callable override)

Cinematic or state-dependent view without subclassing `get_camera_transform`:

```python
import numpy as np
from minilink.graphical.catalog import follow_frame_camera, fixed_camera, camera_matrix

# Factory callable — passed to animate()
follow = follow_frame_camera("body", scale=12.0, target_offset=(2.0, 0.0, 0.0))

def zoom_on_speed(frames, x, u, t):
    """Custom: pull back as speed increases. Callable is the whole contract — no class."""
    speed = float(np.hypot(x[3], x[4]))
    scale = 8.0 + 0.5 * speed
    target = frames["body"][:3, 3]
    return camera_matrix(target, plot_axes=(0, 1), scale=scale)

car.animate(traj, camera=follow)           # Layer 3 override
car.animate(traj, camera=zoom_on_speed)    # custom callable
car.animate(traj, camera=fixed_camera(np.array([0, 0, 0]), scale=20.0))
```

Multi-robot: overlays never carry camera hints — to follow robot2, animate it as
**primary**, or pass an explicit `camera=` callable. Ghost a second robot with
`Replay` without stealing the camera:

```python
ghost = Replay(robot2, traj2)
robot1.animate(traj1, overlays=[scene.as_visualizer(), ghost])  # primary robot1 keeps camera
robot2.animate(traj2, camera=follow)                            # or follow robot2 as primary
```

---

## Use case 7 — MPC demo (Phase 6 target — no plant subclass)

**Today (Phase 4 interim):** demo subclasses bolt overlays into
`get_kinematic_geometry` + `time_channel_matrix`.

**Target (Phase 6):** composition at the animation boundary:

```python
"""examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py — target shape"""
import numpy as np
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import JaxDynamicBicycleRateInputs
from minilink.graphical.catalog import HorizonPolyline, Line, SceneHistory, TrajectoryPolyline, follow_frame_camera
from minilink.planning.spatial.scene import Scene

# ... planner setup, compute traj, mpc_plans list ...

robot = JaxDynamicBicycleRateInputs()  # plain catalog plant — no MpcPlan* subclass

# Spatial scene (collision-first source); reference path is graphics-only → SceneHistory/overlay
scene = Scene(obstacles=obstacles)
history = SceneHistory(
    reference=Line(ref_pts, color="gray", linewidth=1),
    trail=TrajectoryPolyline(executed_traj, color="limegreen", linewidth=2),
    horizon=HorizonPolyline(mpc_plans, color="darkorange", linewidth=1.5),
)

robot.traj = executed_traj
robot.animate(
    executed_traj,
    overlays=[scene.as_visualizer(), history],
    camera=follow_frame_camera("body", scale=12.0),
    time_factor_video=2.0,
    show=True,
)
```

**Lines removed vs today:** no `MpcPlanBicycleRate` class, no `time_channel_matrix`,
no manual transform list alignment.

---

## Use case 8 — Holonomic corridor trajopt (Phase 6 target)

```python
"""examples/scripts/planning/trajopt/demo_holonomic_corridor.py — target shape"""
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.graphical.catalog import SceneHistory, TrajectoryPolyline
from minilink.planning.spatial.scene import Scene

robot = HolonomicMobileRobot()
# ... planner, track, obstacles, traj ...

# Existing spatial Scene (obstacles) — as_visualizer() renders the default obstacle skin
scene = Scene(obstacles=obstacles)
history = SceneHistory(trail=TrajectoryPolyline(traj, color="blue"))

robot.animate(traj, overlays=[scene.as_visualizer(), history], camera_scale=12.0)
```

---

## Use case 9 — Diagram cascade (frame namespacing)

No demo changes beyond using final API; `DiagramSystem.tf` prefixes frames:

```python
"""Diagram animation — frames auto-prefixed plant:body, controller:..."""
diagram = controller >> plant
diagram.animate(traj)  # skin from plant; controller StaticSystem returns {}
```

Inspect namespaced frames:

```python
frames = diagram.tf(traj.x[:, k], traj.u[:, k], traj.t[k])
assert "plant:body" in frames
```

---

## Use case 10 — Non-spatial / schematic plant

Default `{}` — nothing drawn in diagrams unless opted in:

```python
class CSTR(DynamicSystem):
    """No spatial state — schematic only."""
    def get_kinematic_geometry(self):
        return debug_state_skin(self)  # opt-in helper from graphical.catalog

    def tf(self, x, u, t=0, params=None):
        return {}  # or schematic frame keys if skin uses them
```

---

## Use case 11 — During migration (Phases 2–4 only)

Compare pipelines in one script before cutover:

```python
COMPARE_PIPELINES = True
traj = sys.compute_trajectory(t_f=5.0)

sys.animate(traj, show=False, save="baseline.gif")      # old — source of truth

if COMPARE_PIPELINES:
    sys.animate_v2(traj, show=False, save="candidate.gif")  # must match pixels
```
