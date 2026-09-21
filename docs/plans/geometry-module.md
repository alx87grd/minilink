# Workspace geometry module

**Status:** core contract — design agreed 2026-09-21, not started.  
**Lane:** core (`minilink.core.geometry`), then a teaching catalog of courses.  
**Depends on:** [core-objects-4-fields.md](core-objects-4-fields.md) for the `Field` ABC (can move spatial files first and rename `StateField` → `Field` in the same pass or after).

Workspace math is split today: SDF solids in [`core/geometry.py`](../../minilink/core/geometry.py), everything else under [`planning/spatial/`](../../minilink/planning/spatial/). That home is an accident of first consumer (trajopt). Control cannot import planning ([RULES.md](../../RULES.md) 3.2), so [`PurePursuit`](../../minilink/control/geometric.py) takes a raw waypoint array. Demos rebuild the same oval and cones by hand.

**Placement test:** if you would still import it to score a controller with no `PlanningProblem`, it is not `planning/`.

```python
track, scene = racecar_circuit()
body = bind(car, point_probe())
cost = track.distance_field(body).as_cost() + scene.clearance_field(body).as_cost(shaping=quadratic_hinge())
loop = PurePursuit(track) @ car
J = cost.total_cost(loop.compute_trajectory(tf=10))
```

Graphics stay a second vocabulary on purpose ([architecture review §4.6](../reviews/2026-09-05-architecture-review.md)): glyphs, skins, S30, S44 are not this module. Constitution still holds: a few textbook objects (`Shape`, `Path`, `Track`, `Scene`, posed probes), not a computational-geometry library.

## Before and after

| Object | Before | After |
| --- | --- | --- |
| `Shape`, `Sphere`, `Box`, `Union`, `Inflated` | [`core/geometry.py`](../../minilink/core/geometry.py) | `core/geometry/shapes.py` (package, same names) |
| `SE2`, `SE3`, `Rx/Ry/Rz`, `translation`, `inv`, `apply` | [`core/kinematics.py`](../../minilink/core/kinematics.py) | **unchanged** — pose algebra, sibling of geometry |
| `ReferencePath`, `PolylinePath`, `from_waypoints`, `circuit_waypoints` | [`planning/spatial/paths.py`](../../minilink/planning/spatial/paths.py) | `core/geometry/paths.py` |
| `is_closed` | [`control/geometric.py`](../../minilink/control/geometric.py) | `core/geometry/paths.py` |
| `ReferenceTrack` | [`planning/spatial/track.py`](../../minilink/planning/spatial/track.py) | `core/geometry/track.py` |
| `Scene` | [`planning/spatial/scene.py`](../../minilink/planning/spatial/scene.py) | `core/geometry/scene.py` |
| `WorkspaceField`, `GaussianField` | [`planning/spatial/workspace_fields.py`](../../minilink/planning/spatial/workspace_fields.py) | `core/geometry/` beside Scene |
| `disc`, `point_probe`, `car_outline` | [`planning/spatial/collision.py`](../../minilink/planning/spatial/collision.py) | `core/geometry/probes.py` |
| `CollisionBody`, `bind` | same file | `core/geometry/collision.py` |
| `StateField` → `Field`, `FieldSet`, `FieldCost` | [`planning/spatial/state_fields.py`](../../minilink/planning/spatial/state_fields.py) | [`core/fields.py`](core-objects-4-fields.md) |
| `ClearanceField`, `CostDensityField`, `PathDistanceField`, `CorridorMarginField` | same file | `core/geometry/fields.py` (subclass `Field`) |
| `quadratic_hinge`, `quadratic_excess`, `inverse_barrier`, `occupancy` | [`planning/spatial/shaping.py`](../../minilink/planning/spatial/shaping.py) | `core/fields.py` next to `Field.as_cost` |
| `FieldGrid`, `sample_grid` | [`planning/spatial/grid.py`](../../minilink/planning/spatial/grid.py) | `core/geometry/grid.py` |
| `plot_scene`, `plot_track`, cost rasters | [`planning/spatial/plotting.py`](../../minilink/planning/spatial/plotting.py) | `graphical/` (lazy; `scene.plot()` / `track.plot()` stay on the object) |
| `TrackCorridorOverlay` | [`planning/spatial/overlays.py`](../../minilink/planning/spatial/overlays.py) | `graphical/` |
| named courses | copy-pasted in demos | `core/geometry/catalog.py` |
| `planning.spatial` | owner | re-export, then **delete** |

Student import after the move: `from minilink.core.geometry import Scene, bind, racecar_circuit`. Band facades may keep `minilink.planning` names for one migration commit ([RULES.md](../../RULES.md) 3.4), then drop them.

**Stay put**

| Object | Home | Why |
| --- | --- | --- |
| `PurePursuit` | `control/geometric.py` | a law; it *reads* a Path/Track |
| `CBFSafetyFilter` (planned) | `control/` | a block; it *reads* a clearance `Field` |
| `DubinsSteering`, RRT extenders | `planning/search/` | connectors; they *read* a curve |
| `PlanningProblem`, trajopt, RRT, RL | `planning/` | verbs |
| `region_of_attraction` | `analysis/` | verb; will *read* a `Field`/`Set` |
| catalog plants, `tf()` | `dynamics/` | plants emit poses |
| glyph `Sphere`/`Box`, skins, renderers | `graphical/` | drawing (S30) |
| `Set`, `CostFunction`, `Trajectory`, `System` | `core/` as today | existing nouns |

`Shape` is `sdf(p)` (occupied workspace). `Field` is `value(x)` (state-domain scalar). Do not fold them. A Dubins *path* (the curve) can later be a `Path` kind in geometry; `DubinsSteering` stays search.

PurePursuit’s lookahead stays **closest waypoint, then along the polyline** — not `PolylinePath.project`. Sharing the path *object* is the win; the law stays in control.

## Catalog

Short teaching list, like `minilink.catalog` for plants. No `Course` DTO: `track, scene = racecar_circuit()`.

| Factory | What | Today |
| --- | --- | --- |
| `oval_circuit(length, width, radius, half_width=…)` | rounded-rectangle `Track` (wraps `circuit_waypoints`) | tests, [`racecar_lap_3d.py`](../../examples/projects/racecar/racecar_lap_3d.py) |
| `racecar_circuit()` | 6×4 m oval, `half_width=0.6`, far-straight cones | three UdeS racecar demos (MPC, dyn MPC, RL) |
| `holonomic_forest()` | scatter of discs in `[-6, 6]²` | [`rrt_holonomic_obstacles.py`](../../examples/demos/rrt/rrt_holonomic_obstacles.py) |

Leave in the owning demo until a second teaching script needs them: wide MPC circuit, slalom, U-turn corridor, car_trajopt corner. `circuit_waypoints` stays a thin alias during the split. The RL racecar demo may keep extra cones as arguments on `racecar_circuit`, not a second factory.

## Where new things go later

Workspace = `p ∈ ℝ²/ℝ³`. State = `(x, u, t)`. Same test.

| New thing | Example | Home |
| --- | --- | --- |
| Another solid | capsule, polygon, `BicubicGridSDF` | `core/geometry/shapes.py` |
| Another curve | spline, clothoid, Dubins *path* | `core/geometry/paths.py` |
| Variable-width / Frenet | `Track` with `s`-varying width | `core/geometry/track.py` |
| Another course | slalom once two demos share it | `core/geometry/catalog.py` (promote from the demo; no track DB) |
| Another probe | capsule body, link spheres | `core/geometry/probes.py` |
| Workspace → state map | clearance along a link | `core/geometry/fields.py` |
| Density on `p` | mud, slope | `WorkspaceField` in geometry |
| Workspace SDF grid | voxel/bicubic as a `Shape` | geometry; `Field` only if the table is over `x` |
| `V`, `J`, `Q` | Lyapunov, DP, critic | `core/fields.py` |
| A tracker / Stanley / CBF | outputs `u` | `control/` |
| RRT-Connect, new steering | search connector | `planning/search/` |
| Collision inside `f` | uses `CollisionBody` + `tf` | `dynamics/` (plant); geometry is the description |
| A glyph or skin | how it looks | `graphical/` |

Do not grow a computational-geometry library (meshes, CSG, clipper, public GJK). New solids earn `sdf(p)` and a teaching caller.

```
minilink/core/geometry/      # workspace nouns
minilink/core/fields.py      # Field ABC, shaping, V/J/Q
minilink/core/kinematics.py  # poses T, R
minilink/control/            # laws that read Path/Track/Field
minilink/analysis/           # verbs that read Field/Set
minilink/planning/           # planners; spatial/ goes away
minilink/dynamics/           # plants that emit tf
minilink/graphical/          # drawing only
```

## Implementation (later)

1. `geometry.py` → `core/geometry/shapes.py`.
2. Move Path, Track, Scene, probes, `bind`, spatial Fields, grid; shaping next to `Field.as_cost`; plots/overlays under `graphical/` with lazy methods on Scene/Track.
3. `planning.spatial` re-exports, then delete in the same change as call-site updates ([RULES.md](../../RULES.md) 3.4).
4. Catalog factories; switch the three UdeS racecar demos and the RRT forest demo.
5. PurePursuit accepts a `Path` / `Track`.
6. [core-objects-4](core-objects-4-fields.md) §4.1: spatial subclasses already live in geometry when `Field` lands.

## Out of scope

- Glyph rename (S30) and posed-geometry hook (S44)
- Cataloguing research-lane circuits
- A `Course` wrapper
- Folding `Shape` into `Field`
- A general geometry library or track database
- Moving RRT extenders / Dubins steering
