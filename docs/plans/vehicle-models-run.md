# Vehicle models: the run

**Status:** run 1 in progress on `vehicle-models-phase1`.  
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

## 7. Ledger

The numbers that moved, each with how it was measured (machine, median of N, warm or cold).

- **VM-1:** none; the baseline is byte-identical. The magic formula at its default shape
  (`c = 13/rad`, `μ = 1`, one NumPy run on 300 001 angles): peak 1.000 at 12.2°, 0.982 at
  20°, 0.943 at 40°, as the plan computed.
