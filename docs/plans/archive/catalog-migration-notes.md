# Catalog Migration Notes

Working notes for the catalog cleanup pass (one module at a time): double-check
EoM, verify graphical output, migrate dynamic parameters to `self.params`, tidy
the `__main__` demos, and eliminate `_graphics.py`.

## Conventions (decided in review)

### Parameters

- **EoM parameters** (mass, inertia, gravity, aero/tire coefficients, ...) go in
  the standard `self.params` dict. Methods read them via
  `params = self.params if params is None else params`, unpack the few values
  they need into local variables, then write the pure-math equation
  (textbook-readable, per `agent.md`). Prefer clear symbols (e.g. `Iyy`).
- **Graphical / camera parameters** (lengths used only for drawing, `camera_scale`,
  `dynamic_range`, ...) stay as plain instance attributes, not in `self.params`.

### Graphics / `_graphics.py` elimination

`minilink/dynamics/catalog/_graphics.py` should not exist long-term.

- **Generic primitives + transform/camera/scene helpers** live in
  `minilink.graphical.animation.primitives` (e.g. `arrow_transform`,
  `follow_xy_camera`, `ground_line`, `identity_matrix`, `empty_transform`,
  `line_between_transform`, `rod_between_transform`, `point_transform`,
  `heading_from_vector`). Already relocated there.
- **Shapes reused by 2+ modules** (`vehicle_body`, `wheel_box`, `spring_line`)
  also go to `primitives` (avoids duplication).
- **Single-use shapes** are inlined into their owning class as a small method
  (e.g. plane's `unit_line` -> `chord_line()`).
- During the pass, `_graphics.py` re-exports the relocated helpers so
  un-migrated modules keep working; it is deleted once no module imports it.
- Dead helper `rectangle_outline` removed.

### `__main__` demos

- Use `sys` as the instance variable name.
- Set `sys.x0`.
- **Constant input** -> set `sys.inputs["u"].nominal_value = ...` and use the
  regular ODE solve `sys.compute_trajectory(tf=...)`.
- **Time-varying input** -> use `sys.compute_forced(lambda t: ..., tf=...)`.
- Add `sys.animate()` when the system has graphical output.
- Add `sys.plot_phase_plane()` for 2-state systems.

### State bounds and plot axis limits

- Plots read `state.lower_bound` / `state.upper_bound` for default axis limits.
  Phase-plane bound resolution order: explicit `bounds=` arg -> finite state
  bounds -> trajectory min/max (padded) -> `(-10, 10)` fallback.
- Set state bounds to pin a stable phase-plane frame / vector-field extent
  (as `VanderPol` does with `[-3, 3]`).
- Caveat: these bounds are dual-purpose; `BoxSet.from_system_state()` uses them
  as the state constraint box for trajectory optimization.

## Module change log

- **equations/integrators.py** — done. Graphics simplified to one `Point` per
  state (no body/arrows, no ground line); imports repointed to `primitives`;
  added symmetric `[-10, 10]` state bounds (Simple/Double/Triple) so the phase
  plane frames consistently; `DoubleIntegrator` demo now uses nominal input +
  `compute_trajectory`.
- **aerial/plane.py** — done. EoM params in `self.params` (incl. `Cl_alpha`,
  `Iyy`); `plane_body` moved into class as `body_shape()`; `unit_line` inlined
  as `chord_line()`; imports repointed to `primitives`.
- **equations/transfer_function.py** — done. Imports repointed to `primitives`;
  `__main__` uses `sys` + `compute_forced` + `animate()` (1 state, no phase plane).
- **mass_spring_damper/linear.py** — done. `spring_line` moved to `primitives`;
  `mass_box` inlined as module-private `_mass_box` (matches existing
  `_force_arrow_transform` style); generic helpers repointed to `primitives`;
  `__main__` uses nominal input + `compute_trajectory` + `animate` (6 states, no
  phase plane). Later migrated to params-driven state space (see below): masses
  now subclass the method-based `StateSpaceSystem`, building `A/B/C/D` from
  `self.params` (mass/stiffness/damping); `_abcd`/`refresh` dropped; graphics read
  `self.params["k"]`.

### State-space abstraction refactor

- **dynamics/abstraction/state_space.py** — `StateSpaceSystem` is now a
  method-based base: `A(t, params)`, `B(t, params)`, `C(t, params)` (default
  identity), `D(t, params)` (default zero), with `f`/`h` threading `params`
  exactly like `MechanicalSystem.H(q, params)`. Subclasses build matrices from
  `params` and can be time-varying. Added `LTISystem(StateSpaceSystem)` for the
  common constant-matrix case (holds the old constructor + dimension validation,
  preserves NumPy/JAX array identity); introspect matrices via `sys.A()`.
- `TransferFunction` now subclasses `LTISystem`.
- `mass_spring_damper` masses (`Single/Two/ThreeMass` + floating variants) now
  subclass `StateSpaceSystem`, store EoM params in `self.params`, and build
  `A/B/C/D` from them (unpacking into short locals per `agent.md`). Floating
  variants set `self.params["k1"] = 0.0` instead of calling `refresh()`.
- Tests: `test_state_space_system.py` exercises `LTISystem` (matrix access via
  `sys.A()`) plus a small param-driven `StateSpaceSystem` subclass;
  `test_catalog_migration.py` reads MSD matrices via `single.A()`/`two.C()`/etc.
- Consequence: matrices are methods everywhere, so `sys.A` is a bound method;
  use `sys.A()` for the array (e.g. `numpy.linalg.eigvals(sys.A())`).

- **pendulum/pendulum.py** — done. Already imported from `primitives`. Pulled
  graphics-only `l1` out of `self.params` into a `self.l1` attribute (EoM uses
  `lc1`/`I1`/`m1`, not `l1`); removed a dead `length` local; gave
  `TwoIndependentPendulums` a fitting camera (`camera_scale=4.0`); `__main__`
  uses `sys` + `compute_forced` (time-varying input) + `plot_phase_plane`
  (2-state) + `animate`.

- **pendulum/pendulum.py** (InvertedPendulum fix) — `InvertedPendulum.theta` is
  measured from the upward vertical (negated `g`), so it now overrides
  `get_kinematic_transforms` to pose the shared geometry with a `+pi` offset
  (zero-angle renders upright instead of upside down).
- **pendulum/pendulum.py** (single-DOF naming) — dropped the `1` index on the
  single-pendulum classes (`Pendulum`, `InvertedPendulum`, `TwoIndependentPendulums`):
  params are now `m, l, I, gravity, d`. Unified `lc1`/`l1` into one `l` (CoM
  distance == drawn rod length), so `l` lives in `self.params` (EoM uses it,
  graphics reads it). EoM numerically unchanged (`H=2`). Halved camera scale
  (rod shrank 2->1). Side effect: README quick-start (`params["l"]`,
  `params["m"]`) is now valid against `Pendulum`. Keep subscripts on genuine
  multi-body systems (double_pendulum, cartpole).
- **pendulum/double_pendulum.py** — done. Already on `primitives`. Pulled
  graphics-only `l2` and `ground_half_width` out of `self.params` (`l1` stays:
  EoM uses it; link-2 dynamics use `lc2`/`I2`, not `l2`); `__main__` uses `sys` +
  `compute_forced` (time-varying) + `animate` (4 states, no phase plane).

- **pendulum/cartpole.py** — done. Already on `primitives`. Moved graphics-only
  params (`pole_length` [was `l`], `cart_length/height/depth`, `ground_half_width`)
  out of `self.params` into instance attributes; EoM dict is now `lcg, m1, m2,
  gravity` for `CartPole`/`JaxCartPole` (set by the shared `_configure_cartpole`
  helper, reused by the NumPy/JAX twins). Applied new guidelines: unpacked params
  in `RotatingCartPole` `H/C/g/d` (no dict indexing in the math), `# fmt: off`
  column-aligned 2x2 matrices, short intent comments. Inlined the single-use
  `_joint_positions` (RotatingCartPole) and the module-level `_pose2d_offset_z`
  (now a 2-line `pole_pose[2, 3] = pole_z` in `CartPole.get_kinematic_transforms`).
  `__main__` uses `sys` + `compute_forced` (time-varying) + `animate` (4 states,
  no phase plane). Test update: `test_pendulum_plants.py` reads `sys.cart_depth`
  instead of `sys.params["cart_depth"]`.

- **vehicles/mountain_car.py** — done. EoM params (`mass`, `gravity`, hill-shape
  `a`/`w`) moved into `self.params`; the terrain helpers `z`/`dz_dx`/`d2z_dx2`
  and `H`/`C`/`B`/`g` now unpack params into short locals (no `self.`/dict
  indexing in the math) with intent comments. Imports repointed from
  `_graphics` to `primitives` (`Arrow`/`Circle`/`CustomLine`/`arrow_transform`/
  `identity_matrix`/`translation_matrix`, all generic—no single-use shape to
  inline). EoM double-checked against pyro `mountaincar.py`: numerically
  identical, no discrepancy. `__main__` uses `sys` + constant throttle via
  `inputs["u"].nominal_value` + `compute_trajectory` + `plot_phase_plane`
  (2-state) + `animate`. No test edit needed (`test_catalog_migration.py` reads
  `mountain.H(...)`, which threads defaults).

- **vehicles/propulsion.py** — done. EoM params (`length`, `xc`, `yc`, `mass`,
  `gravity`, `rho`, `cdA`, `mu_max`, `mu_slope`, plus `wheel_radius`/
  `wheel_inertia` on the torque variant) moved into `self.params`; `camera_scale`
  stays a plain attribute (derived from `params["length"]`). Imports repointed
  from `_graphics` to `primitives` (`vehicle_body`/`wheel_box` imported, not
  redefined; no module-specific shape to inline). Applied new guidelines:
  `slip2force`/`acceleration`/`_slip`/`f` unpack params into short locals with
  intent comments and no `self.`/dict indexing in the math; inlined the single-use
  `ratios()` into `acceleration` (only `rr`/`ry` used—`rf` was already dead).
  EoM double-checked against pyro `vehicle_propulsion.py`: numerically unchanged
  (kept minilink's `+1e-6` slip denominator vs pyro's `|v|+0.0`; dropped the
  normal-force print checks, already absent in minilink). `__main__` uses `sys` +
  `x0` + constant slip via `inputs["u"].nominal_value` + `compute_trajectory` +
  `plot_phase_plane` (2-state slip model) + `animate`.

- **vehicles/steering.py** — done. Imports repointed from `_graphics` to
  `primitives` (shared `vehicle_body`/`wheel_box` imported, not redefined; all
  other shapes generic—no single-use shape to inline). Wheelbase `length` is the
  lone EoM param and now lives in `self.params` for `KinematicBicycle`/
  `KinematicCar`/`UdeSRacecar`; `ConstantSpeedKinematicCar` carries
  `{"speed", "length"}`. EoM `f` methods unpack params into short locals (no dict
  indexing / `self.` in the math) with a short intent comment; the holonomic
  robots have no EoM params. Graphics-only attributes (`width`, `tire_length`,
  `tire_width`, `camera_scale`, plus `camera_plot_axes` on the 3D robot) are now
  explicit instance attributes (replacing the old `getattr` fallbacks). EoM
  double-checked against pyro `vehicle_steering.py`: numerically unchanged
  (verified `f()` before/after on all 6 classes). `__main__` uses `sys` +
  `compute_forced` (time-varying steer) + `animate` (3 states, no phase plane).
  No test edit needed (`test_catalog_migration.py` reads `camera_scale`/Arrow
  counts/`f`, all preserved).

## Remaining modules

All catalog modules migrated. `aerial/{rocket,drone,plane}.py`,
`vehicles/{dynamic_bicycle,suspension,steering,propulsion,mountain_car}.py`,
`marine/boat.py`, `manipulators/arms.py`, `equations/*`,
`mass_spring_damper/linear.py`, and `pendulum/*` all follow the guidelines.
`_graphics.py` deleted; generic helpers (`vehicle_body`, `wheel_box`, transforms,
camera/scene helpers) now live in `graphical/animation/primitives.py`. No source,
test, or example references `_graphics` anymore.

## Catalog-wide QA + uniformization pass

After migration, the whole catalog was re-verified term-by-term against pyro
(EoM + graphical kinematics) and given a uniform style pass.

EoM verdict: **no math bugs** anywhere. Every `H/M, C, g, d, B, f` matches pyro
to machine precision. Two intentional deviations kept because minilink is *more*
correct: `equations` `ThreeMass.B` uses `1/m3` (pyro's `1/m2` is a latent bug),
and `propulsion` regularizes the slip denominator with `+1e-6` (pyro divides by
zero at `v=0`).

Fixes applied in this pass:
- `pendulum/cartpole.py` — fixed a cosmetic graphics bug where the linear
  `CartPole` double-counted `wheel_dx` (wheels drew at `pos ± cart_length/2`
  instead of `± cart_length/4`); wheel `Point`s now sit at body-local x=0 and the
  per-wheel pose carries the offset. Rewrote `CartPole` `H/C/B/g` as `# fmt: off`
  column-aligned `np.array` literals to match the JAX twin and `RotatingCartPole`.
  Dropped the dead `self.m = 0` in `UnderactuatedRotatingCartPole.__init__`.
- Naming uniformity (local symbol == params key): `aerial/{rocket,drone,plane}.py`
  now unpack `mass`/`inertia` (were `m`/`Iyy`). Every module unpacks params into
  locals named exactly like the dict keys.
- `pendulum/double_pendulum.py` — `H/C/g` rewritten as aligned matrix literals
  with intent comments; inlined the single-use `_joint_positions`; dropped dead
  `self.m = 0` in `Acrobot`.
- `aerial/drone.py` — dropped dead `self.m = 0` in `Drone2DWithSideThruster`.
- `equations/oscillators.py` — `VanderPol.f` now unpacks `mu` (was dict-indexed in
  the equation) and names the rate `ddy`.
- `equations/transfer_function.py` — build `signal.TransferFunction` once for
  poles/zeros (was constructed twice).
- `pendulum/pendulum.py` — brief intent comments on `H`/`g`.
- `vehicles/suspension.py` — moved the road-profile arrays `a`/`w`/`phi` into
  `self.params` (parity with `mountain_car`); `z`/`dz` now accept and thread
  `params`, so a `params` override reaches the terrain forcing.

Open items deferred for maintainer decision (not changed here):
- `vehicles/dynamic_bicycle.py` — `d()` honours a passed `params`, but
  `tire_forces`/`wheel_velocities` read `self.params`/`self.r_f`/`self.r_r`
  directly, so a `params` override is only half-applied (latent; invisible in
  nominal sim). Tied to whether `r_f`/`r_r` should move into `params` (they enter
  the slip/normal-force math) versus stay attributes for the demo scripts.
- Graphics-fidelity nuances (all cosmetic): `HolonomicMobileRobot3D` velocity
  arrow drops `vz` and renders 2D by default; `KinematicCar` body drawn centered
  on `(x, y)` vs the rear-axle model reference; `suspension` sprung mass drawn at
  true `y` (pyro offsets by `+3.0` to avoid terrain overlap); minor body-silhouette
  size differences for `Rocket`/`Drone`.
