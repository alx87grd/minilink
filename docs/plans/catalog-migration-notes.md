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
- **mass_spring_damper/linear.py** — done. StateSpace (no dict; A/B/C/D covered by
  reference tests). `spring_line` moved to `primitives`; `mass_box` inlined as
  module-private `_mass_box` (matches existing `_force_arrow_transform` style);
  generic helpers repointed to `primitives`; `__main__` uses nominal input +
  `compute_trajectory` + `animate` (6 states, no phase plane).

- **pendulum/pendulum.py** — done. Already imported from `primitives`. Pulled
  graphics-only `l1` out of `self.params` into a `self.l1` attribute (EoM uses
  `lc1`/`I1`/`m1`, not `l1`); removed a dead `length` local; gave
  `TwoIndependentPendulums` a fitting camera (`camera_scale=4.0`); `__main__`
  uses `sys` + `compute_forced` (time-varying input) + `plot_phase_plane`
  (2-state) + `animate`.

## Remaining modules

pendulum/{double_pendulum,cartpole}.py,
aerial/{rocket,drone}.py, vehicles/{dynamic_bicycle,steering,propulsion,
suspension,mountain_car}.py, marine/boat.py, manipulators/arms.py — then delete
`_graphics.py`.
