# Vehicle models: the run

**Status:** run 1 in review on `vehicle-models-phase1` (VM-5 and VM-8 wait on the maintainer, §6); run 2 not started.  
**What it is:** the brief of the automatic runs (AGENTS.md, "Automatic runs") that build
[vehicle-models.md](vehicle-models.md): the steps in order, one commit each, what proves
each one, and when to stop and ask. The design is in that plan (its §10 says how the pieces
are written); this doc restates none of it.  
**Scope:** two runs, one pull request each. Run 1 is the plan's phase 1 (the foundations, the
car part of v0.4's catalog clean-up, K8-0 in [TODO.md](TODO.md)). Run 2 is its phase 2 (the
3-D car), started after run 1 is merged. Phases 2b and 3 are not in this brief.

Steps are numbered `VM-1`, `VM-2`, … (this doc's own prefix).

## 1. Rules of the run

- **Branch:** `vehicle-models-phase1` (run 2: `vehicle-models-phase2`), cut from `dev` once
  the plan is merged there, otherwise from `vehicle-models-plan`. One commit per step, one
  pull request into `dev` at the end, never merged by the agent.
- **Read first:** AGENTS.md, CONSTITUTION, RULES, the plan, then the tests of the car family
  (`test_racecar_plant.py`, `test_racecar_tires.py`, `test_dynamics_catalog.py`,
  `test_catalog_backends.py`, `test_teaching_surface.py`): they are the output contract.
- **The textbook rule:** every equation path follows RULES §5; each diff with math is read
  next to `planning/dynamic_programming/dp.py`, and the report says so.
- **Read only:** `examples/projects/car_trajopt/` and `examples/projects/pathtracking/`. A
  step that would need an edit there stops and asks.
- **Course material:** no file under `examples/teaching/courses/` is renamed or moved. An
  import of a name that leaves is updated in place, and the pull request lists each such edit.
- **Student-facing text** (README, examples, notebooks): only the edits a renamed or removed
  name forces, each listed in the pull request.

## 2. The bar: same behaviour, measured

The maintainer's ruling (plan §7): phase 1 keeps the demos' behaviour, not their bytes.

**Before any edit** (step VM-0), a seeded baseline is written to JSON in the scratchpad, run
twice and compared to prove it is deterministic. For each car class on the catalog, on NumPy
and on JAX:

- `f` and every output port at 200 seeded states and inputs, standstill and reverse included;
- one 5 s simulation from a seeded start under a seeded input sequence;
- for the rate-input bicycle and the racecar, one trajectory optimization and ten MPC steps
  from `benchmarks/scenarios/`.

**After each step**, the baseline runs again:

| Step kind | Bar |
| --- | --- |
| a move, a rename, a new file nothing calls | byte-identical |
| the brush law written per unit load (VM-2) | `f` and the outputs within 10⁻⁹ relative; the simulation's final state within 10⁻⁶; each solve converges in the same number of iterations ±2, to a cost within 10⁻⁶ relative |

**At the end of the run:** the regression gates pass against their stored baselines with no
`--update`, and every car demo under `examples/demos/` runs and is compared, by its final
state, with the same demo on the starting commit.

## 3. Run 1: the foundations

| Step | What | Done when |
| --- | --- | --- |
| VM-0 | The baseline of §2 and a list of every reader of the names that leave (`KinematicCar`, `DynamicBicycleCar3D`, `UdeSRacecarDyn3D`, `brush_tire_forces`, `params["length"]`, the label `"theta"` on a car) | the baseline is deterministic; the list is in the pull request |
| VM-1 | The tire module (plan §10.2): the four laws, `tire_slip` and `linear_tire_forces` moved in from `dynamic_bicycle.py`, `wheel_slip`, `smooth_floor`. Nothing calls the new laws yet | new tests pass on NumPy and JAX: slope `c` at zero slip; the brush law and the magic formula never pass `μ Fz`, alone or combined; the three laws agree within 5 % up to a demand `z = 0.3`; the magic formula peaks at `μ Fz`; gradients finite at zero slip; `LinearSaturation` equals today's function bit for bit. `test_dynamics_catalog.py` no longer imports from `examples/projects/`. Baseline byte-identical |
| VM-2 | The powered car calls its tire (plan §10.4): `tire=` on the class, the rolling resistance in the car, `brush_tire_forces` removed and its tests moved to `BrushTire().forces` | the racecar's tests pass unchanged in what they assert; the baseline is within §2's tolerance |
| VM-3 | Rungs 3 and 3b get the `tire=` slot, with today's path as the default | default construction byte-identical; with `BrushTire()` the car simulates, traces under JAX and has per-axle stiffness keys |
| VM-4 | `DrivetrainBicycle` in its own file, `UdeSRacecarDyn` a thin preset on it, `wheel_angles=True`, `UdeSRacecarDyn3D` removed and its readers migrated | `UdeSRacecarDyn` byte-identical to VM-3's; `UdeSRacecarDyn(wheel_angles=True)` byte-identical to the former `UdeSRacecarDyn3D`; the generic car's sanity test (plan §10.5) passes; its default numbers are listed in the pull request |
| VM-5 | `KinematicCar` and `DynamicBicycleCar3D` become skins; their readers, the catalog index and the test fixtures migrated | the same draw lists as before for both looks (the graphics contract test); baseline byte-identical |
| VM-6 | `psi` in the labels and local names of every car file; `length` leaves `KinematicBicycle`'s `params` | baseline byte-identical; no reader of the old label or key remains |
| VM-7 | The terrain module and its static block (plan §10.6), wired into no car | tests: each built-in gradient against a finite difference of its height; NumPy and JAX agree; the block in a diagram returns the fields at its input points |
| VM-8 | One port layout across the bicycle rungs | **only on the maintainer's ruling** (plan §9); skipped otherwise, and the pull request says so |
| VM-9 | The docs the change makes stale: DESIGN, the catalog index and its docs page, ROADMAP's ledger, TODO's row for the car cut, the plan's status line | the docs build passes with warnings as errors |
| VM-10 | The review pass, then two independent reviewers (AGENTS.md, "Reviews"), then the gates on the final tree: ruff, `pytest -n auto`, the demo-check bridge on its own, the regression gates, the notebook checks | all pass; the pull request reports what moved and why |

## 4. Run 2: the 3-D car

Started after run 1 is merged. The equations are in plan §10.7.

| Step | What | Done when |
| --- | --- | --- |
| VM-11 | The attitude block alone: `R`, the body rates `E [p, q, r]`, and `Ė [p, q, r]` in closed form | tests: `Ė [p, q, r]` against a finite difference; a torque-free body keeps its energy and its world angular momentum over 10 s within the solver's tolerance |
| VM-12 | `SpringDamper` and the corner: gap, compression, rate, load, contact point | tests: at rest on flat ground the four loads sum to the weight within the floor's residual; a dropped car settles at the static compression; a car above the ground carries no load |
| VM-13 | `SuspensionCar3D.f` on flat ground, with its ports and outputs | the reduction tests of plan §7 (steady-state yaw rate against rung 4 on a narrow track; a planar controller for rung 4 drives it through its first nine states); mirror symmetry; coasting never gains energy; NumPy and JAX agree; the Jacobian is finite at rest; `jit` and a gradient in `params` run |
| VM-14 | `tf`, the look on `car_skin_3d`, `fastest_mode` with the suspension's mode, `wheel_angles=True` with four angles | the solver hint is under the fastest mode of the linearized car at rest; the animation runs headless |
| VM-15 | The terrains under the car: `Slope`, `Bumps`, and the friction field | tests: with `mu` near zero the car on a slope of angle `β` accelerates at `g sin β`; on bumps the body's heave stays bounded; an ice patch lowers the grip of the wheels on it only |
| VM-16 | The `rollover` output, and a run ended by a terminal set on the roll angle | a turn too fast for the track and the height of the centre of gravity trips it; the same turn at half the speed does not |
| VM-17 | Two demos under `examples/demos/`: the car over bumps, animated; a path tracker written for rung 4 driving the 3-D car | both run headless under the demo checks; the catalog's both-backends contract test lists the new plant |
| VM-18 | Docs, the review pass, the two reviewers and the gates, as VM-9 and VM-10 | as VM-9 and VM-10 |

## 5. Stop and ask

- A step whose bar is byte-identical moves a number.
- VM-2 misses §2's tolerance, or a solve stops converging.
- A regression gate fails, or would need `--update`.
- A speed gate is slower than `dev` beyond noise (the two sides alternated, the borderline
  line repeated) and cannot be fixed.
- A step needs an edit in a read-only folder, a renamed course file, or a CONSTITUTION change.
- `DynamicBicycleCar3D`'s corner arrows cannot move into a skin without changing the skin
  mechanism's public contract.
- A public name beyond those of plan §10.8 is needed.
- A rebase conflict.

Every other open choice takes the recommended option and is logged in §6.

## 6. Decisions taken during the run

Each entry: the step, the choice, the options it was taken over, and why.

- **VM-0, the branch.** Cut from `vehicle-models-plan`: `dev` does not contain the plan yet.
- **VM-0, the baseline harness.** A script in the session's scratchpad, not committed: the
  car keys stay fixed across the run and a small table maps each key to the constructor of
  the moment (`KinematicCar()` before VM-5, `KinematicBicycle` with `a = 2, b = 3` after).
  It covers the thirteen vehicle classes of the catalog (and the stacked- or named-port
  variants), `f` and every output port at 200 seeded states and inputs on NumPy and on JAX
  (`jit`), a 5 s run on each backend, and for `BicycleDynRate` and `UdeSRacecarDyn` one
  direct-collocation solve (JAX, SLSQP, 8 steps) and ten MPC ticks on the compiled face.
  The racecar's solve is the harness's own: no benchmark scenario solves on it.
- **VM-0, the racecar's solve re-posed before any edit.** The first racecar solve (SLSQP,
  `ftol = 10⁻⁶`, 60 iterations) turned a 10⁻¹⁵ change in `f` into an O(1) change of the
  MPC run, so it could not judge VM-2's tolerance. Both solves now run to `ftol = 10⁻⁹`
  (200 iterations at most) and the racecar's cost weighs the wheel rate and the power a
  little more (`Q` 0.01 and 0.001, `R` 0.001 on `P_cmd`, `u_bar = [5 W, 0]`); a 10⁻¹³
  nudge of the start then moves the MPC states by 3·10⁻⁶ and its iteration counts by 1 at
  most. The baseline was captured again from a clean worktree of the starting commit,
  and run twice, byte-identical.
- **VM-1, `axle_slips` keeps its own lines until VM-2.** Routing it through `wheel_slip`
  moves the last bit under JAX `jit` (XLA rewrites `a / √b` as `a · rsqrt(b)` when the root
  is written twice, not when it is shared), while NumPy and eager JAX stay identical. So
  VM-1, whose bar is byte-identical, keeps `axle_slips` as it is, and VM-2, whose bar is a
  tolerance, routes it through `wheel_slip`; writing the root twice in `wheel_slip` to
  match a compiler's rewrite was the other option.
- **VM-1, "the three laws agree within 5 %".** Read as 5 % of the friction limit `μ Fz`:
  relative to the force itself, the brush law is 10 % under the linear tire at a demand
  of 0.3 (its shape is `z − z²/3 + z³/27`), so no law with a friction limit can meet that
  reading. Measured (pure lateral slip, `z` up to 0.3): linear and brush 0.029, linear and
  magic formula 0.007, brush and magic formula 0.022, in units of `μ Fz`.
- **VM-1, the project tests leave `test_dynamics_catalog.py`.** The done-when asks that
  the file import nothing from `examples/projects/`; it imported the pathtracking tires and
  also the car_trajopt extras, ladder and profiles and the pathtracking helpers. All the
  checks of project code move, unchanged, to a new `tests/unittest/test_car_projects.py`
  (the other option, deleting the project checks, loses coverage). The project folders
  are not edited.
- **VM-1, `linear_tire_forces` stays beside `LinearSaturation` for one step.** It moves
  into `tires.py` unchanged and the rungs still call it, so the step is a move; VM-3 makes
  the rungs call the law and removes the function.
- **VM-2, `z_floor` leaves `PUBLIC_RACECAR_PARAMS`.** It is the brush law's own number, so
  the tire copies it into the car's `params` at construction (`{**PUBLIC_RACECAR_PARAMS,
  **tire.params}`) and the preset keeps the car's numbers only; the car's `params` keep the
  same keys and values. Keeping it in the preset too was the other option (two homes for
  one number).
- **VM-2, `tire=None` means the rung's own default.** `UdeSRacecarDyn(tire=None)` binds
  `BrushTire()`; a default argument holding an instance would be shared by every car.
- **VM-2, the rolling-resistance test.** The brush law no longer knows the speed, so the
  check that the front axle resists its rolling moves from the tire to the car's
  `compute_tire_physics`.
- **VM-3, how the rung binds its tire path.** `__init__` binds two methods once:
  `axle_forces` (`saturated_linear_forces` or `tire_law_forces`) and `axle_stiffnesses`
  (for the solver hint), and `compute_tire_physics` delegates to the first; RULES 2.3's
  pattern of binding a function at construction. Assigning over `compute_tire_physics`
  itself was the other option, but it would shadow the racecar's override.
- **VM-3, no rolling resistance on rungs 3 and 3b with a module law.** "Rung 4's path"
  there means its tire keys (`mu`, `c_kappa`, `c_alpha_f`, `c_alpha_r`, the law's own),
  its smooth slip and its law, not the racecar's `C_rr`: rung 3 has none, and phase 3's
  switch then changes the law and the slip alone, so its before and after measure the
  tire. `Ca` and `Ck` leave `params` when a law is given (one home per number).
- **VM-3, the solver hint written per axle.** `fastest_mode` reads `(C_αf, C_αr, C_κ)` from
  the bound `axle_stiffnesses` and writes the linearization with the two axles apart
  (`−(a C_αf − b C_αr)` for the shared `−C_α (a − b)`). On the default numbers (`a = b`)
  the bits are the same; for `a ≠ b` the hint may move by its last bit.
- **VM-3, `linear_tire_forces` removed.** The rungs call `LinearSaturation` through
  `tire.forces`; the bit-for-bit test keeps the original function as its reference.
- **VM-4, how the preset hands over its numbers.** `DrivetrainBicycle.__init__` writes its
  own numbers as a dict literal (as `DynamicBicycle` does) and bounds the command ports
  from `params` with an internal `bound_commands()`; `UdeSRacecarDyn` sets
  `{**PUBLIC_RACECAR_PARAMS, **tire.params}` after it and calls `bound_commands()` again.
  The other options needed a new public name (a `params=` keyword, a class-level preset
  attribute, a module constant for the generic numbers), which §5 reserves for the
  maintainer.
- **VM-4, the wheel angles bound once.** `wheel_angles=True` binds `rolling_angle_rates`
  (the two rates of the former `UdeSRacecarDyn3D.f`), `False` an empty block, and `f`
  concatenates it after the nine; `tf` and the look read the plain `self.wheel_angles`
  flag (graphics, not an equation path).
- **VM-4, no contact arrows on `UdeSRacecarDyn(wheel_angles=True)`.** The former
  `UdeSRacecarDyn3D` drew none (`get_dynamic_geometry` returned `{}`), so the racecar
  keeps that when its wheel angles are states and the course notebook and the realtime
  demo draw what they drew. Drawing the 2-D axle arrows on it was the other option.
- **VM-4, `UdeSRacecarDyn(wheel_angles=True)` no longer brings the 3-D skin.** As the plan
  writes it, the caller sets `car.skin = racecar_skin_3d`: the realtime demo does; the
  course notebook already set its skin before each animation, so it needed only the
  import and the constructor.
- **VM-4, the readers of `UdeSRacecarDyn3D`.** `minilink.catalog`'s index and the
  vehicles' `__all__` drop it and add `DrivetrainBicycle`; the check registry runs
  `DrivetrainBicycle` and `UdeSRacecarDyn(wheel_angles=True)`; `docs/api/dynamics.rst`
  documents the new module.
- **VM-5, stopped: waiting for the maintainer** (§5). Three of its stop conditions hold:
  - `DynamicBicycleCar3D`'s eight corner arrows depend on the state, and a skin is static
    geometry, `(plant) -> dict`, built once (`System.get_kinematic_geometry`); per-frame
    geometry comes only from `get_dynamic_geometry`. Moving the arrows into a skin changes
    the skin contract.
  - `examples/projects/pathtracking/cascade_path_tracking.py` (read only) imports and
    builds `DynamicBicycleCar3D`.
  - `KinematicCar`'s four-wheel look needs a new skin function in
    `graphical/catalog/skins.py`, a public name that plan §10.8 does not list.

  `KinematicCar` and `DynamicBicycleCar3D` stay as they are; the steps after VM-5 do not
  depend on them (VM-6 renames `KinematicCar`'s heading and moves its `length` reads to
  `a + b`). The questions, with their options, are in the pull request.
- **VM-6, the local name of the wheelbase.** `KinematicBicycle.f` keeps its local
  `length = a + b`, and `KinematicCar`'s look computes `length = a + b` from `params`; only
  the key leaves. `axle_offsets` in `skins.py` keeps its `params["length"]` fallback: it
  serves any plant (`LongitudinalSlipCar` keeps its own `length`).
- **VM-7, the terrains' shapes.** `Bumps` is the egg-crate `z = A sin(k x) sin(k y)`,
  `k = 2π / terrain_wavelength` (the other reading, `A (sin k x + sin k y) / 2`, is a
  washboard along each axis); `Slope` rises along `x`, `z = x tan β` with
  `β = terrain_slope` (0.1 rad by default). Both carry a uniform `terrain_mu` (1.0); the
  ice patch is run 2's (VM-15).
- **VM-7, the fields' signature.** `height(x, y, t=0.0, params=None)`, the same for
  `gradient` (a pair) and `friction`: the plan's order, with the defaults every equation
  path of the catalog takes.
- **VM-7, the block's ports.** `TerrainBlock(terrain=None, n_points=4)` takes
  `u = [x_1 … x_n, y_1 … y_n]` and gives three named outputs, `height`, `gradient`
  (`dz_dx` of every point, then `dz_dy`) and `friction`; it has no `y`, so it wires with
  `connect`, not `>>`. Its `params` is the terrain's own dict, one home.
- **VM-7, where the terrain is imported from.** Like the tire laws it stays out of the
  vehicles' `__all__` and the catalog index ("plants only in `__all__`"): it is imported
  from `minilink.dynamics.catalog.vehicles.terrain`. Adding it to a facade is a public-name
  decision, left to the maintainer.
- **VM-10, the reviewers' findings applied** (each reproduced first; baseline
  byte-identical after them, and `DrivetrainBicycle`'s numbers, solver hint and `f` the same
  bits):
  - `DynamicBicycle(tire=LinearSaturation())` took the per-unit-load path and fed the
    absolute law per-unit stiffnesses (forces some 7000 times too soft, silently): the rung
    now binds its path on the law's kind, and `DrivetrainBicycle` refuses
    `LinearSaturation` with a message naming the laws it takes.
  - `DrivetrainBicycle` kept its parent's bindings for the linear law (`axle_stiffnesses`
    raised `KeyError('Ca')`): it now passes its tire to the parent, builds its numbers on
    the parent's (no second copy of rung 3's car) and reads its stiffnesses through the
    bound `axle_stiffnesses`.
  - `TerrainBlock`'s `params` was a copy of its terrain's dict (`System.params` copies an
    assignment), not one home: it now holds the terrain's dict at `"terrain"`, rule 2's
    composite form, with a test of the identity.
  - The wheelbase's second owner: `KinematicCar` and `UdeSRacecar` kept `self.a`,
    `self.b`, which the skins read before `params`, so an edit of `params["a"]` left the
    drawing stale; the attributes leave.
  - Clean-ups: one branch for the tire path in `DynamicBicycle.__init__`, its parameter
    comment true for both key sets; a copied comment in `LinearSaturation`; the empty
    wheel-angle block beside its twin; a test's `c_alpha_r` against `Fz_r`; a retired
    `length` key in a racecar test; `pyro-port-remaining.md` and ROADMAP's wording.
- **VM-10, the reviewers' findings not applied:**
  - writing `DrivetrainBicycle.compute_tire_physics` through `DynamicBicycle.tire_law_forces`
    (one copy of the axle forces): plan §10.4 writes rung 4's forces on its own page with
    `axle_slips`, and the change moves the racecar's last bits (1.6·10⁻¹³) in a review pass;
    left for the maintainer, with the proposals below;
  - a shallow `copy.copy` of a car keeps its bound tire path on the original: the library
    copies with `deepcopy`, which rebinds; RULES 2.3's binding pattern is kept;
  - `test_racecar_tires.py` repeating some of `test_tires.py` on the racecar's numbers: the
    brief moved those tests there by name;
  - proposals that change a public name or behaviour, in the pull request: the `phi_f`,
    `phi_r` grip labels beside the `phi_rear`, `phi_front` angles; `(kappa, alpha)` as the
    slips' return order; one key for the rear wheel's viscous drag (`bw_rear`, `bw_drive`);
    retiring `axle_slips`; `rear_wheel_ground_torque` taking `Fx_r` (the tire forces are
    computed twice per `f`, and it reads `params.get("bw_rear", 0.0)`); the reverse-mode
    gradient of `LinearSaturation` at zero slip (NaN, as on the starting commit).

## 7. Ledger

The numbers that moved, each with how it was measured (machine, median of N, warm or cold).

- **VM-1:** none; the baseline is byte-identical. The magic formula at its default shape
  (`c = 13/rad`, `μ = 1`, one NumPy run on 300 001 angles): peak 1.000 at 12.2°, 0.982 at
  20°, 0.943 at 40°, as the plan computed.
- **VM-2** (the racecar on `BrushTire().forces`, its slips through `wheel_slip`): only the
  three racecar keys move, within the bar. `f` and the outputs: 3.4·10⁻¹³ relative at most
  (JAX `jit`, the stacked-port racecar), 7·10⁻¹⁵ on NumPy; the 5 s runs' final states:
  7.8·10⁻¹⁴. The racecar's trajectory optimization: 65 iterations before and after, cost
  1.96026131594 both, relative change 1.6·10⁻¹¹, decision vector 1.1·10⁻⁶; its ten MPC
  ticks: the same iteration count on every tick, costs within 1.1·10⁻⁷, states within
  1.7·10⁻⁶. The rate bicycle: byte-identical. Solve time (cloud container, 3 rounds
  alternating the two trees, median of 5 warm solves each): the racecar's first solve on
  a compiled face 0.266, 0.309, 0.272 s before and 0.285, 0.305, 0.264 s after; the rate
  bicycle's 0.141, 0.127, 0.096 s before and 0.107, 0.131, 0.134 s after: within noise.
- **VM-3:** none; byte-identical to VM-2's capture.
- **VM-4:** none; byte-identical to VM-3's capture, `UdeSRacecarDyn(wheel_angles=True)` in
  the former `UdeSRacecarDyn3D`'s place.
- **`DrivetrainBicycle`'s default numbers** (new, unvalidated, for the maintainer's
  review). Rung 3's car: `mass` 1500 kg, `inertia` 2500 kg m², `a = b` 1.0 m, `r_f = r_r`
  0.3 m, `rho` 1.225 kg/m³, `CdA` 0.66 m², `gravity` 9.81, `mu` 1.0, `v_min_epsilon`
  0.1 m/s; stiffnesses per unit load from its `Ca`, `Ck` at the static load 7357.5 N:
  `c_alpha_f = c_alpha_r` 8.155 /rad, `c_kappa` 13.59. New for the drive and the
  steering: `P_max` 100 kW, `tau_sat` 2500 N m (above the rear grip `μ Fz_r r_r` =
  2207 N m, so a launch at full power spins the wheel, as on the racecar), `engine_tau`
  0.2 s, `Jw_rear` 1.6 kg m² (rung 3b's), `bw_drive` 0.1 N m s, `bw_rear` 0, `tau_fric`
  10 N m, `C_rr` 0.015, `delta_max` 0.6 rad, `steering_tau` 0.1 s, `steer_rate_max`
  1.0 rad/s, `w_min_epsilon` 1.0 rad/s, `v_c` 0.1 m/s, `w_c` 1.0 rad/s; `z_floor` 10⁻⁶
  from the brush tire. Measured (NumPy, one run): from rest at full power 20.8 m/s after
  5 s; top speed 59.6 m/s (215 km/h), against 60.0 m/s from the power, the drag, the
  rolling resistance and the drivetrain's drag alone (the rest is the rear tire's 1.7 %
  slip); its fastest mode, the solver hint, 8.8 µs at rest (the wheel against the rear
  tire's stiffness at the slip floor), so a 5 s run from rest takes 8 s of wall time.
- **VM-6:** none; byte-identical to VM-4's capture.
- **VM-7:** none; byte-identical to VM-6's capture (nothing calls the terrain).
