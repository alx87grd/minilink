# Plan: RRT motion planners (`planning/search/`)

## Context

pyro grew three drifting RRTs, each welded to the dynamics object
(`sys.isavalidstate`, `sys.steer`, `sys.distance`, `sys.get_path_points`). minilink's
`planning/spatial` foundation + the generic `PlanningProblem` now let a *single* clean RRT
source every concern from an abstraction instead of the system:

- **collision** = `problem.X.contains(x)` where `X = bounds & scene.clearance_field(robot).as_constraint()`
- **goal** = `problem.Xf.contains(x)` (a `BallSet` *is* the goal radius)
- **state/input sampling** = `problem.X.sample(rng)` / `problem.U.sample(rng)` (both samplable)
- **dynamics** = `problem.sys.f` via `simulation/`
- **steering/distance** = an injected `SteeringFunction` strategy (off the system)

So RRT touches only `problem.X`/`Xf`/`U`/`sys` — never `Scene`, `RobotBody`, or geometry;
the robot radius/footprint is wired into `X` at problem construction. The system stays pure.
Reserved home: `planning/search/` (ROADMAP P3, TRL 1→2).

## Decisions (locked)
- First cut = one `RRTPlanner` + the `Extender` seam, shipping **both** `KinodynamicExtender`
  and `SteeringExtender`. `RRTStarPlanner` (rewiring) deferred.
- Edge collision = **fixed-resolution sampling** of `problem.X.contains` along the edge
  (generic over any `Set X`; resolution is an `RRTOptions` knob). SDF adaptive stepping is a
  follow-up.

## Architecture (mirrors `TrajectoryOptimizationPlanner` + `Transcription`)
- `RRTPlanner(Planner)` — orchestrator; `__init__(problem, *, extender, options)`;
  `compute_solution() -> Trajectory`. Loop: goal-biased sample → nearest node →
  `extender.extend(...)` → validate edge (fixed-res `problem.X.contains`, input bounds via
  `problem.U`) → add node → goal test (`problem.Xf.contains`) → on success backtrack →
  `Trajectory`. Consumes only `problem.X`/`Xf`/`U`/`sys`/`x_start`/`x_goal`. Exposes
  `planner.tree` for viz. Collision-checking lives **on the orchestrator** (the `Extender`
  stays pure geometry/dynamics) — per the orchestrator/strategy split.
- `Extender(ABC)` — `extend(from_x, target_x, problem, rng) -> Edge | None`. `Edge` = states
  `(k+1, n)`, inputs `(k, m)` or `None`, duration, cost. No collision checks inside.
  - `KinodynamicExtender(options)` — sample/enumerate inputs from `problem.U`, roll
    `problem.sys.f` a short horizon (reuse `simulation/`, not a hand-rolled Euler), return the
    rollout whose endpoint is nearest `target_x`. Real `(x,u,t)` → feasible `Trajectory`.
  - `SteeringExtender(steering)` — `steering.connect(from_x, target_x)` → path+cost, truncated
    to a step size.
- `SteeringFunction(ABC)` — `connect(x0, x1) -> (path, cost)` + `distance(x0, x1)` (the
  nearest-neighbor metric). `StraightLineSteering` (holonomic), `DubinsSteering` (bounded-turn
  car). This is where pyro's `sys.steer`/`sys.distance` move to.
- `tree.py` — `Node(x, parent, edge, cost)`, `Tree.nearest(x, metric)`, `Tree.near(x, radius)`
  (for RRT\*), `extract_trajectory(node) -> Trajectory`.
- `RRTOptions` — `max_nodes`, `goal_bias`, `edge_resolution`, `step_size`/`horizon`,
  `seed`/`rng`, `goal_tolerance` (fallback when `Xf` is a singleton).

Result: `compute_solution() -> Trajectory` (mirrors trajopt). Kinodynamic = dynamically
feasible `(x,u,t)`; steering = states along edges with placeholder `u` where steering defines
none. Nearest-neighbor metric comes from the extender/`SteeringFunction`, not the system.

## Files
- NEW `planning/search/__init__.py`, `tree.py`, `steering.py`, `extenders.py`, `rrt.py`
  (later `rrt_star.py`).
- NEW `tests/unittest/test_rrt.py`.
- NEW `examples/scripts/planning/demo_rrt_kinodynamic.py`,
  `demo_rrt_steering.py` (reuse a `Scene` for collision + `scene.plot` heatmap with the
  path/tree overlaid).
- EDIT `DESIGN.md` (planning/search contract), `ROADMAP.md` (TRL bump, P3 item).

## Reuse (don't reinvent)
- `Planner` ABC + `_store_result`/`plot_solution`/`animate_solution` — `planning/planner.py`.
- `PlanningProblem` (`sys`, `X`, `U`, `X0`, `Xf`, `cost`) — `planning/problems.py`.
- `Set.sample`/`contains`, `BallSet`, `IntersectionSet`, `BoxInputSet.sample` — `core/sets.py`.
- `scene.clearance_field(robot).as_constraint()` → `problem.X` — `planning/spatial/`.
- `simulation/` integrator for kinodynamic rollout; `Trajectory` — `core/trajectory.py`.
- Demo plants `HolonomicMobileRobot`, `KinematicBicycle` — `dynamics/catalog/vehicles/steering.py`.

## Verification
- `tests/unittest/test_rrt.py`: tree nearest-neighbor; seeded `KinodynamicExtender` RRT reaches
  the goal on `HolonomicMobileRobot` with a `Scene` obstacle (`X = bounds & free`); straight-line
  `SteeringExtender` RRT routes start→goal around an obstacle; `DubinsSteering` respects the turn
  radius; an edge crossing an obstacle is rejected; the returned `Trajectory`'s states all satisfy
  `problem.X.contains`; seeded rng is deterministic.
- Demos headless (`MPLBACKEND=Agg`): `scene.plot` clearance heatmap + RRT path/tree overlay.
- `pytest` + `ruff check`/`format`; no docstring equations; ROADMAP TRL update.

## Out of scope (follow-ups)
`RRTStarPlanner` (rewiring), RRT-Connect/bidirectional, informed sampling, SDF adaptive
collision stepping, oriented-body (SE(2)/SE(3)) steering.
