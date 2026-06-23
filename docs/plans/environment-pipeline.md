# Phase 2 — Obstacles, RobotBody, and the Environment pipeline

Status: in progress (Phase 1 + soft traversability landed). Owner area: `planning/environment/`.
Foundation in place: `core/geometry.py` (`Shape`/`Sphere`/`Box`/`Union`/`Inflated`), cost
algebra (`SumCost`/`ScaledCost`, `+`/`*`), `Set.__and__` (`&`), `planning/environment/features.py`
(`ScalarField`/`GaussianField`), and `Environment.cost_density` / `cost_field`.

## Context

Phase 1 built the dependency-free primitives. Phase 2 assembles them into the **Environment
pipeline**: a scene that exports as **either** a hard free-space constraint (`Set`) **or** a
soft traversability cost (`CostFunction`) / cost map, from one definition, so the same scene
drives trajectory optimization, MPC, and the future RRT family. Soft obstacles
(traversability) are first-class. The layer depends only on `core/` and **never imports
`graphical/`** (contact geometry is split from rendering).

First cut is intentionally small: **disc-only `RobotBody`** and hard obstacles. The
future-proofing lives in the *combination seam*, not in the robot — so disc-only costs
nothing later.

## The combination seam: `StateField` (decided)

The free-set `margin(x)` and the obstacle `cost.g(x)` are the **same computation** — evaluate
an environment field over the robot body, as a function of state — differing only in the last
step (threshold vs penalize). So the combination of `RobotBody` + `Environment` is a
first-class **`StateField`** (fused, but not yet committed to constraint or cost), and the two
conversions are generic adapters on it:

```python
state_field.as_constraint(lower=0.0, upper=None) -> Set            # FieldSet
state_field.as_cost(weight=1.0, shaping=None)    -> CostFunction   # FieldCost
```

Every extension falls out without new plumbing:

| Need | Expression |
|---|---|
| Free space (hard → constraint) | `env.clearance_field(robot).as_constraint()` |
| Traversability (soft → cost) | `env.cost_field(robot).as_cost(weight=w)` |
| Hard → soft barrier (potential field) | `env.clearance_field(robot).as_cost(shaping=barrier)` |
| Soft → hard keep-out | `env.cost_field(robot).as_constraint(upper=c_max)` |
| New robot body (capsule, multi-sphere) | only the field's derivation changes; `RobotBody` interface fixed |
| New env content (raster, moving obstacle) | only `Environment.clearance`/`cost_density` change |
| Viz / RRT cost-to-go / MPC stage cost | sample `state_field.value(x)` directly |

Placement: `StateField` + `FieldSet`/`FieldCost` live in `planning/environment/`, promoted to
`core/` only if a consumer outside planning needs them.

## Contracts (clean & generic — the source of truth)

| Object | Module | Core method(s) | `u`? |
|---|---|---|---|
| `Shape` | `core/geometry.py` | `sdf(p, t=0, params=None) → scalar` (native-array, >0 outside); `contains(p,t,params)→bool`; `dim`, `__or__`, `inflate` | no |
| `Set` | `core/sets.py` | `margin(z, t=0, params=None) → vector` (≥0 feasible); `contains`, `sample`, `__and__` | no (state set) |
| `InputSet` | `core/sets.py` | `margin(u, x=None, t=0, params=None) → vector` | yes |
| `CostFunction` | `core/costs.py` | `g(x, u, t=0, params=None) → scalar`; `h(x, t=0, params=None) → scalar`; `+`/`*` | yes (`g`) |
| `RobotBody` | `planning/environment/robot.py` | `shapes → tuple[Shape,…]` (per part); `body_poses(x, u=None, t=0, params=None) → tuple[T,…]` (aligned 1-to-1) | optional |
| `Environment` | `planning/environment/environment.py` | `clearance(p, t=0, params=None) → scalar`; `cost_density(p, t=0, params=None) → scalar`; `clearance_field(robot)/cost_field(robot) → StateField` | no |
| `StateField` | `planning/environment/fields.py` | `value(x, u=None, t=0, params=None) → vector`; `as_constraint(lower,upper)→Set`; `as_cost(weight,shaping)→CostFunction` | optional |

Rules these signatures enforce:
1. **Native-array equation paths** for `sdf`/`body_poses`/`clearance`/`cost_density`/`value`/
   `margin`/`g`/`h` (the `xp` idiom, branch-free) → the whole pipeline traces and differentiates.
   `contains`/`sample`/`as_cost_map` are the only NumPy boundaries.
2. **`(t, params)` threaded uniformly** (moving obstacles / scenario sweeps reuse param plumbing).
3. **`u` only where control is meaningful** — `CostFunction.g`, `InputSet`, and *optionally*
   `body_poses`/`StateField.value`. **State sets carry no `u`**: `FieldSet.margin(z,t,params)`
   calls `value(z, u=None, …)`; `FieldCost.g(x,u,t,params)` calls `value(x, u, …)`. A pose used
   in a constraint must be u-independent (normally is — pose is forward kinematics of the state).
4. **`T` is a homogeneous `(d+1)×(d+1)` transform** (3×3 in 2-D, 4×4 in 3-D), built with
   `xp.stack` so it traces. Free helper `apply_transform(T, q) = T[:d,:d] @ q + T[:d,d]` places a
   body point. The robot exposes the *poses*; the `StateField` applies them to the parts'
   collision points and subtracts radii.
5. **Multi-body from day one.** `RobotBody` is a *sequence of rigid parts* — `shapes` (static,
   per part) and `body_poses` (one transform per part, aligned 1-to-1). This mirrors the
   `System` visualization contract (`get_kinematic_geometry` + `get_kinematic_transforms`,
   "1-to-1 with the static geometry", `core/system.py:339`), which already drives multi-link
   animation. A disc is a one-part robot; a manipulator is N parts whose `body_poses` is forward
   kinematics — **no downstream change**. The sphere restriction lives in one helper
   `collision_spheres(shape)` (`Sphere → [(center, radius)]`; `Capsule → spheres along the
   segment`, later).

`Set` and `CostFunction` are unchanged; `FieldSet`/`FieldCost` are their only new adapters.

## Pipeline (where each responsibility lives)

```
state x ─RobotBody: shapes + body_poses(x,u,t,params)=(T_i) → apply per part ─Env.clearance/cost_density→ StateField.value ─→ FieldSet.margin (Set)
                                                                                                                              └─→ FieldCost.g    (Cost)
```

- **`RobotBody`** — a sequence of rigid parts: `shapes` (per-part body-frame geometry, same
  primitives as obstacles) and `body_poses(x, u, t, params) → tuple[T,…]` (one homogeneous
  transform per part from forward kinematics, not raw state, aligned 1-to-1 with `shapes`). No
  representation accessor; collision is *derived* by applying each `T` to its part. Native-array
  equation path so everything downstream stays differentiable. A disc is one part; a manipulator
  is many.
- **`Environment`** — the scene (hard `Shape` list + soft `ScalarField` list). Workspace-point
  queries: `clearance(p)` (signed distance to obstacles) and `cost_density(p)` (traversability).
- **`StateField`** — the fused, state-dependent quantity (`value(x)`). For a `Sphere` body
  (first cut) `clearance_field.value = env.clearance(apply_transform(T, center)) − radius`; richer
  bodies make it a per-part vector without changing `RobotBody`. Built by
  `env.clearance_field(robot)` and `env.cost_field(robot)`.
- **`FieldSet` / `FieldCost`** — the *only* place a `Set`/`CostFunction` is produced, generic
  over any `StateField`.

## Design refinements (carried from earlier review)

- **The robot body is geometry + pose methods, multi-body from day one.** `RobotBody` is a
  sequence of parts: `shapes` (per part) + `body_poses(x, u, t, params) → tuple[T,…]` (transforms,
  not raw state). Collision is *derived*, never a stored `world_spheres`-style representation.
  Asymmetry vs obstacles: obstacles may be any `Shape` (we only sample `obstacle.sdf(point)`),
  but the robot's collision parts are restricted to the sphere/capsule family so clearance stays
  the cheap exact query `obstacle.sdf(apply_transform(T, center)) − radius`. A point is a radius-0
  `Sphere`. The body parts, not the obstacles, own the radii, so one `Environment` is reusable
  across robots.
- **No `Obstacle` wrapper** — a hard obstacle is just a `Shape`; safety inflation is
  `shape.inflate(margin)` (reuses `Inflated`).
- **Soft features are `ScalarField`s** (own small ABC) — traversability is a smooth field,
  not a solid.
- **Composition via operators** already built: `X = bounds & free_set`, `cost = base + w *
  obstacle_cost`, goal `Xf = BallSet(goal, r)`.

## Module layout (`minilink/planning/environment/`, no barrel re-exports)
- `__init__.py` — one-line docstring (namespace marker).
- `robot.py` — `RobotBody` (disc/point now; spheres/capsule later).
- `fields.py` — `StateField` ABC + `FieldSet(Set)` + `FieldCost(CostFunction)` (generic seam).
- `features.py` — `ScalarField` ABC, `GaussianField` (soft sources). *(step 2 partial)*
- `environment.py` — `Environment` (`clearance`/`cost_density` point queries;
  `clearance_field`/`cost_field` → `StateField`; `as_cost_map`).

Dependency law: imports `core.geometry`, `core.sets`, `core.costs`, `core.backends`,
`core.system` only.

## Component specs

### `robot.py` — disc first, multi-body contract
A robot is a **sequence of rigid parts**: `shapes` (static body-frame geometry, per part) and
`body_poses(x, u, t, params)` (one homogeneous transform per part, aligned 1-to-1 with
`shapes`). Poses are transforms from forward kinematics, *not* raw state. Collision is derived
in the `StateField`. The first cut ships only the single-part planar case (`PlanarBody`), but
the contract is already multi-body, so a `Manipulator` is a later addition with no downstream
change.

```python
class RobotBody(ABC):
    """Collision model: per-part body-frame geometry + one world pose per part (forward kinematics).

    Mirrors the System visualization contract (static geometry 1-to-1 with transforms),
    but with collision-grade geometry. One part for a disc; N parts for a manipulator.
    """
    @property
    @abstractmethod
    def shapes(self): ...                                       # tuple[Shape,...] body-frame, per part
    @abstractmethod
    def body_poses(self, x, u=None, t=0.0, params=None): ...    # tuple[T,...] aligned with shapes

@dataclass(frozen=True)
class PlanarBody(RobotBody):
    """Single-part planar body: translation from `position` indices, optional `heading` index."""
    shape: Shape                         # body-frame geometry (a Sphere for the first cut)
    position: tuple = (0, 1)             # state components giving the body-frame origin in world
    heading: int | None = None           # state component giving planar rotation (optional)

    @property
    def shapes(self):
        return (self.shape,)

    def body_poses(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)
        tx, ty = x[self.position[0]], x[self.position[1]]
        c = 1.0 if self.heading is None else xp.cos(x[self.heading])
        s = 0.0 if self.heading is None else xp.sin(x[self.heading])
        # SE(2) homogeneous transform, assembled with xp.stack so it traces
        T = xp.stack([xp.stack([c, -s, tx]),
                      xp.stack([s,  c, ty]),
                      xp.asarray([0.0, 0.0, 1.0])])
        return (T,)

def disc(radius, *, position=(0, 1), dim=2):  return PlanarBody(Sphere(np.zeros(dim), radius), position)
def point(*, position=(0, 1), dim=2):         return PlanarBody(Sphere(np.zeros(dim), 0.0), position)

def apply_transform(T, q):                    # world point of body point q
    xp = array_module(T)
    d = q.shape[0]
    return T[:d, :d] @ q + T[:d, d]

def collision_spheres(shape):                 # body-frame (center, radius) probes; the only sphere-restriction site
    if isinstance(shape, Sphere): return ((shape.center, shape.radius),)
    raise NotImplementedError("first cut supports Sphere body parts")   # Capsule/Union later
```
`body_poses`/`apply_transform` are native-array equation paths. Follow-ups: `Capsule` parts
(`collision_spheres` samples along the segment), 3-D `SE(3)`, and a `Manipulator` whose
`body_poses` is real forward kinematics — none of which touch `StateField`/`Environment`.

### `fields.py` — the generic seam
```python
class StateField(ABC):
    """A native-array vector function of state, viewable as a Set or a Cost.

    The fused result of placing a robot in an environment, before deciding whether
    it acts as a hard constraint or a soft cost.
    """
    @abstractmethod
    def value(self, x, u=None, t=0.0, params=None): ...   # (k,) native-array, differentiable

    def as_constraint(self, *, lower=0.0, upper=None) -> Set:
        return FieldSet(self, lower, upper)
    def as_cost(self, *, weight=1.0, shaping=None) -> CostFunction:
        return FieldCost(self, float(weight), shaping)

@dataclass(frozen=True)
class FieldSet(Set):
    field: StateField
    lower: float | None = 0.0
    upper: float | None = None
    def margin(self, z, t=0.0, params=None):              # state set → no u; pass u=None
        xp = array_module(z)
        v = self.field.value(z, None, t, params)
        parts = []
        if self.lower is not None: parts.append(v - self.lower)   # feasible when value >= lower
        if self.upper is not None: parts.append(self.upper - v)   # ... and value <= upper
        return xp.concatenate(parts)

@dataclass(frozen=True)
class FieldCost(CostFunction):
    field: StateField
    weight: float = 1.0
    shaping: object | None = None         # callable v -> shaped (native-array), e.g. a barrier
    def g(self, x, u, t=0.0, params=None):               # cost has u → forward it
        xp = array_module(x)
        v = self.field.value(x, u, t, params)
        shaped = v if self.shaping is None else self.shaping(v)
        return self.weight * xp.sum(shaped)
    def h(self, x, t=0.0, params=None):
        return 0.0
```

### `environment.py`
```python
@dataclass(frozen=True)
class Environment:
    obstacles: tuple = ()        # hard Shapes (use .inflate(margin) for safety)
    fields: tuple = ()           # soft ScalarFields (step 2)
    workspace_dim: int = 2

    # workspace-point queries
    def clearance(self, p, t=0.0, params=None):
        # signed distance to the nearest obstacle; large positive when there are none
        return Union(self.obstacles).sdf(p, t, params)
    def cost_density(self, p, t=0.0, params=None):
        return sum(field.density(p, t, params) for field in self.fields)

    # lift to state fields via the robot (the combination)
    def clearance_field(self, robot) -> StateField:   # value(x) = clearance at the placed body − radius
        return ClearanceField(self, robot)
    def cost_field(self, robot) -> StateField:        # value(x) = cost_density at the placed body
        return CostDensityField(self, robot)

    def as_cost_map(self, bounds, resolution, t=0.0, params=None) -> CostMap   # NumPy boundary

@dataclass(frozen=True)
class ClearanceField(StateField):
    env: Environment
    robot: RobotBody
    def value(self, x, u=None, t=0.0, params=None):
        xp = array_module(x)
        out = []
        for shape, T in zip(self.robot.shapes, self.robot.body_poses(x, u, t, params)):
            for center, radius in collision_spheres(shape):       # body-frame probes
                world = apply_transform(T, center)
                out.append(self.env.clearance(world, t, params) - radius)
        return xp.stack(out)                                      # one component per (part, sphere)
# Multi-body (manipulator) and capsule parts extend the two loops with no RobotBody change.
# CostDensityField is analogous, summing cost_density at the probe centers.
```
Note: when `obstacles` is empty, `clearance_field` is unused (no constraint); demos always add
obstacles. `clearance` over an empty `Union` is undefined — guard `clearance_field`/`cost_field`
to require a non-empty source, or return an always-feasible field.

## Integration — `PlanningProblem` stays generic
```python
env   = Environment(obstacles=(Sphere([2, 2], 0.5), Box([4, -1], [5, 3]).inflate(0.2)),
                    workspace_dim=2)
robot = disc(radius=0.3, position=(0, 1))            # robot.py PlanarBody helper

X    = BoxSet.from_system_state(sys) & env.clearance_field(robot).as_constraint()
cost = base_quadratic + 10.0 * env.cost_field(robot).as_cost()        # once soft fields exist
problem = PlanningProblem(sys, x_start=x0, X=X, cost=cost, Xf=BallSet(goal, 0.4))
```
- **Trajopt** routes a non-`BoxSet` `X` through `X.margin(...)` (`direct_collocation.py:313`),
  so `FieldSet` becomes smooth inequality constraints for free. **Tradeoff:** `bounds &
  free_set` is an `IntersectionSet`, so the box bounds become general constraints rather than
  the fast variable-bound path; acceptable for Phase 2.
- **RRT** (Phase 3) calls `X.contains(x)` — the NumPy boolean path.
- **Moving obstacles / scenarios** flow through `ProblemParameters.sets`/`.cost`; the
  `(t, params)` signature is threaded end to end (first-pass shapes ignore it; param-dependent
  shapes are a follow-up).

## NumPy/JAX correctness (same rules as Phase 1)
- `RobotBody.body_poses`, `StateField.value`, `Environment.clearance`/`cost_density`, `FieldSet.margin`,
  `FieldCost.g` are equation paths: `xp = array_module(...)`, branch-free, no Python
  `min`/`max`/`abs`, no `float()`/`bool()` mid-math, no mutation → the constraint margin and
  the cost stay differentiable for trajopt/MPC.
- `contains` and `as_cost_map` are NumPy boundaries (sampling/reporting), outside the trace.

## Style compliance (`agent.md`)
Frozen dataclasses for records (`PlanarBody`/`FieldSet`/`FieldCost`/`Environment`/concrete
fields); `RobotBody`/`StateField`/`ScalarField` are `ABC` mother classes; bare equation signatures on
`value`/`margin`/`g`/`body_poses`/`clearance`; section banners + first-screen rule;
libraries silent; no single-use private helpers; **no `graphical/` import**; `~10-line
__main__` in `environment.py`.

## Build order (first cut = 1–4)
1. `robot.py`: `RobotBody` ABC + `PlanarBody` + `disc`/`point` helpers + `apply_transform`
   (`shapes` + `body_poses` + `collision_spheres`, traceable).
2. `fields.py`: `StateField` ABC + `FieldSet` + `FieldCost`.
3. `environment.py`: `Environment` (obstacles only) → `clearance`, `clearance_field`;
   `ClearanceField`.
4. Demo `examples/scripts/planning/demo_environment.py` (disc robot, a couple of obstacles,
   plot free/occupied) + tests.
5. Soft side: `features.py` (`ScalarField`, `GaussianField`); `Environment.cost_density`,
   `cost_field`, `CostDensityField`; `as_cost_map`.
6. Follow-ups: barrier shaping helper, multi-sphere / `capsule` bodies (heading rotation),
   param/moving obstacles, `CostGrid`/raster, soft→hard threshold demo, `System.contact_geometry()`
   hook, 3D.

## Tests & verification
- `tests/unittest/test_environment.py`:
  - `disc`/`point` `body_poses` returns aligned `(T,)`; `apply_transform(T, center)` tracks
    translation (+ heading rotation); shape carries the radius. Cover a 2-part stub robot to
    lock the multi-body path.
  - `Environment.clearance` against hand-placed obstacles; **`jax.grad` of
    `env.clearance_field(robot).as_constraint().margin` w.r.t. `x` runs** (end-to-end
    differentiable pipeline).
  - `FieldSet`: `.contains` agrees with a brute-force grid occupancy sweep; `.margin` is a
    per-sphere vector; a `disc` of radius `r` shifts the free boundary out by `r` vs a `point`;
    `upper=` produces a keep-out band.
  - `FieldCost`: `g` rises over a `GaussianField` peak; `shaping` applied; weight scales.
  - **Cross-export check (the design promise):** one `StateField` → both `.as_constraint()` and
    `.as_cost(shaping=barrier)`; an interior point fails `contains` and has high barrier cost.
  - JAX-twin parity, gated on `.[jax]` with a skip message.
- Optional integration smoke: a tiny direct-collocation solve on `HolonomicMobileRobot` with
  `X = bounds & env.clearance_field(robot).as_constraint()` avoids an obstacle (can defer to Phase 3).
- Run `/opt/anaconda3/envs/dev-h26/bin/python -m pytest`; `ruff check` + `ruff format --check`
  on touched files; `MPLBACKEND=Agg` for the demo smoke.

## Docs to update
- `DESIGN.md`: add `planning/environment/` to the package map and a short contract — the
  combination seam (`StateField` → `Set` via `as_constraint`, → `CostFunction` via `as_cost`),
  `RobotBody` = per-part `shapes` + `body_poses(x,u,t,params)→tuple[T,…]`, `core`-only dependency.
- `ROADMAP.md`: bump the "Geometry / environment" row as the Environment lands.

## Downstream (Phase 3)
RRT family under `planning/search/` consumes `env.clearance_field(robot).as_constraint()`
(collision) and `env.cost_field(robot).as_cost()` (RRT* edge cost): `RRTPlanner(Planner)` +
injected `Extender` (`KinodynamicExtender` rolls out `sys.f` for any system;
`SteeringExtender(SteeringFunction)` for straight-line/Dubins), with `RRTStarPlanner` as a
later rewiring layer.
