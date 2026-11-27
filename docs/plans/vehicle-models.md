# Vehicle models: what is left to build

The design of the parts of the car family that are not in the code. Lane: teaching catalog
(`minilink/dynamics/catalog/vehicles/`); the specialized models (off-road research) go to the
maintainer's private repository and build on the same pieces.

**What exists.** The ladder from the point to the 3-D car with suspension
(`HolonomicMobileRobot`, `KinematicBicycle`, `DynamicBicycle`, `BicycleDynRate`,
`DrivetrainBicycle`, `SuspensionCar3D`), the tire laws, the terrains, the presets
(`UdeSRacecar`, `UdeSRacecarDyn`, `FormulaCar3D`, `OffRoadCar3D`) and their looks. How a car
holds its parts' numbers is DESIGN §4, "Parameters"; the equations are each class's docstring.

The open design questions of the built cars (the 3-D car's rear axle, the point the kinematic
bicycle follows, the public face of the parts, three small points of the review) are in
[QUESTIONS.md](../dev/QUESTIONS.md). The order of work is [TODO.md](TODO.md)'s.

## 1. The rules every new piece keeps

| Rule | Statement |
| --- | --- |
| Appending | a rung appends states and never reorders them: rung 4's nine `x, y, psi, vx, vy, r, w_rear, delta, P` are the first nine of every car above it, with the same meaning (`vx, vy` in the heading frame, yawed and not rolled or pitched; `r = ψ̇`) |
| Optional states | one order on every rung: the nine, the body's six (`z, phi, theta, vz, p, q`), the wheel-hop states, the wheel rolling angles last; an option that is off leaves no gap |
| Presets and skins | a model that differs only by its numbers is a preset class with no equations; one that differs only by its look is a skin |
| Parts | a tire, a terrain and a spring own their numbers, held by the car at `params["tire"][wheel]`, `params["terrain"]`, `params["suspension"][wheel]`; the car's own numbers stay flat, one suffix per axle, left and right symmetric |
| Tire laws | linear in the load, `F = Fz · φ(κ, α)`, with stiffnesses per unit load; `Fz` is an argument, never computed inside the law; the car calls the tire's slip at one place |
| Friction | two owners, multiplied: the tire's `mu` and the terrain's `friction(x, y, t)` |
| Load transfer | never in the single track's equations (the bicycles keep static axle loads; each dynamic rung reports the rigid car's ratios as its `ltr` output, fed back nowhere); on the double track only, always an exact continuous model, never a lag on the acceleration |
| Equations | on each plant's own page (RULES 7.8); the rungs share only the generic blocks of `tires.py` and `terrain.py` |
| Project folders | `examples/projects/car_trajopt/` and `pathtracking/` are read only |

## 2. The tire and slip switch

Three moves, decided together, one commit each, after the maintainer's pass over the demos:

1. **Rungs 3 and 3b move from `LinearSaturation` to the brush law.** Without a `tire=`, the
   two rungs bind `LinearSaturation`: linear forces scaled back onto the friction circle, with
   absolute stiffnesses `Ca` [N/rad], `Ck` [N] shared by both axles and the slip `|vx| + ε`.
   The switch is a change of default: its before and after are the same class built twice.
   `LinearSaturation` then retires, with the flat keys `Ca`, `Ck`, `mu`, `v_min_epsilon`.
2. **Every law moves to one slip function,** the smooth one:

   ```python
   # Slips over the rolling speed, its floor smooth at rest:
   # κ = (ω R − vx) / √(vx² + ε²), α = −atan(vy / √(vx² + ε²))
   v_roll = xp.sqrt(vx**2 + v_eps**2)
   # v_roll = xp.abs(vx) + v_eps   # the formula before the switch
   ```

   The tire module holds two today: `tire_slip` (`|vx| + ε`, rungs 3 and 3b without a tire)
   and `wheel_slip` (`√(vx² + ε²)`, every other car). Against `|vx| + ε` the smooth form:
   - is smooth at `vx = 0`;
   - biases the slips by about `ε²/(2 vx²)` instead of `ε/|vx|` (with ε = 0.1 m/s: 0.5 %
     instead of 9 % at 1 m/s, 0.005 % instead of 1 % at 10 m/s; from the formulas).

   Its costs: between rest and a few `ε` the tires are stiffer (up to √2 at `vx = ε`); every
   number on rungs 3 and 3b moves. The previous formula stays as a commented line under the
   new one.
3. **The longitudinal car joins the tire module** (`LongitudinalSlipCar`,
   `LongitudinalTorqueCar`): a `tire=` slot, its slip on the module's function, its parameter
   names aligned with the ladder's (`mu`, `c_kappa`, `CdA`, `r_r`, `Jw_rear` for `mu_max`,
   `mu_slope`, `cdA`, `wheel_radius`, `wheel_inertia`). Whether its default stays the sigmoid
   `μ(s)` is decided then; the magic formula's peak opens the optimal-slip lesson (ABS,
   traction control). If that lesson needs a visible peak, raise `C` (1.6 gives a limit of
   0.59 of the peak at large slip, against 0.89 at the default 1.3).

**What the switch fixes:**

| Defect | Where |
| --- | --- |
| Two slip regularizations near standstill in the catalog (a third, `max(|vx|, |ωR|, ε)`, in the project folders, not changed here) | move 2 |
| Gradient kinks in the optimization plants: the `|vx| + ε` slip, the hard friction circle | moves 1 and 2 |
| The reverse-mode gradient of `LinearSaturation` at zero slip is NaN (measured: `jax.grad` of `Fy` in `α` at 0) | move 1 |
| `DynamicBicycle` is neutral-steer by default: one shared `Ca` and `a = b` give `K = m/L (b/Ca_f − a/Ca_r) = 0` | move 1: the brush law reads one stiffness per axle |

**Each move is measured before and after:**

- **Reference:** the regression gates on the bicycle, trajectory optimization
  (`benchmarks/scenarios/e4_trajopt_parity.py`) and MPC (`f_mpc_parity.py`), against their
  stored baselines, re-recorded once a switch is accepted; the car demo the maintainer keeps,
  run and looked at on both sides; for the longitudinal car, the course labs' scripts (GRO501
  APP2, GMC714 lab 6).
- **Numbers recorded on both sides:** for each solve, converged or not, iterations, final
  cost; solve time (the median of 5 warm runs on one machine, the two sides alternated); the
  largest state difference between the two optimal trajectories; for the MPC, the closed-loop
  tracking error.
- **A switch stays** if every solve still converges, the cost and the tracking error are
  within noise or better, and the time is not slower beyond noise. Otherwise stop and ask.

## 3. The UdeS Racecar on the 3-D car

The racecar preset brought to `SuspensionCar3D`, with its real skin and its parameters: a
proxy that behaves roughly like the real car, never an identified model of it (identifying
the car is the students' work in class). Its class is not named `UdeSRacecarDyn3D`: that name
meant "the nine states and two wheel angles" in course material and would silently change
meaning. The name is the maintainer's, chosen when it is built.

## 4. `Car3D`: the double track without suspension

A rung between the drivetrain bicycle and the 3-D car: rung 4's nine states, nothing added,
four wheels, the track width, quasi-static load transfer. It is `SuspensionCar3D`'s limit
with an infinitely stiff suspension, and it shows only the onset of a rollover (an inner
wheel's load reaching zero).

The body stays planar and rigid. Each wheel's load follows the accelerations of the centre of
gravity. That is a loop: the loads depend on the accelerations, the accelerations on the tire
forces, and the forces on the loads. It is solved exactly, inside `f`:

1. The tires are linear in the load: `F_i = Fz_i φ_i`, where `φ_i` depends on the state only.
2. The loads are affine in the acceleration `a = (a_x, a_y)`:

   ```text
   Fz_i = Fz_i⁰ + G_i a
   ```

   `G_i` comes from the centre-of-gravity height `h`, the wheelbase, the track and the front
   share of the roll stiffness.
3. Newton's law is then a linear 2×2 system:

   ```text
   (m I − Σ R_i φ_i G_i) a = Σ R_i φ_i Fz_i⁰ − drag
   ```

   The yaw acceleration `ṙ` then follows from the moments with the known loads.

The result is a continuous ODE, pure, on NumPy and JAX, with no state added and no lag.
`LongitudinalSlipCar` already solves this loop by hand in 1-D: its denominator
`m (1 + μ r_y)` is the solved loop.

Its limits:

- The matrix is singular only at the tipping limit (roughly `μ h / L → 1`).
- A smooth floor on `Fz` keeps a wheel's lift-off continuous. With the floor the loads are no
  longer affine in the acceleration, so the 2×2 solve is exact only while the four wheels
  carry load. On a tall, narrow car lift-off is ordinary limit cornering (an inner wheel
  lifts at `a_y = g · track / (2h)`, below `μ g` when `track / (2h) < μ`), so `Car3D` is
  built with a Newton solve inside `f` from the start.
- A load-sensitive tire also makes the loop nonlinear: the same Newton solve.

**What the transfer changes with the module's tire laws.** All of them are linear in the
load, so an axle's total force is `φ · (Fz_left + Fz_right)`:

- front-to-back transfer changes the axle totals: braking oversteer shows directly;
- side-to-side transfer leaves each axle's total unchanged while both wheels grip: it shows
  as a wheel lifting off (and as body roll on the 3-D car), not as a change of balance.
  Changing the balance (roll-stiffness tuning) needs a load-sensitive tire, whose grip per
  unit load falls as the load rises: a law behind the same interface, later or in the
  private repository.

## 5. Options on the 3-D car

Designed for, none scheduled.

- **Wheel hop** (`massless_wheel=False`): each corner's wheel mass, the two-mass quarter car
  per corner, 2 states per corner appended after the body's six. The wheel radius and the
  tire's vertical stiffness are decided with it.
- **Independent wheel speeds** (an open differential): the contact code takes one rate per
  wheel, so the extra rates append with the optional blocks and `w_rear` stays the rear
  axle's mean rate; the first nine keep their meaning. The question of the locked axle is in
  QUESTIONS.md.
- **Body contact points** (`body_contact=True`, off by default): a few points on the body
  with the corner's penalty law, so a car that rolls over lands and keeps tumbling. This is
  the edge of "not a contact engine" (CONSTITUTION §1.4).
- **The contact along the body's suspension axis,** for off-road work on relief. The corner's
  contact sits under the attachment along the terrain's normal, `C = P_att − d n`. A strut
  fixed to the body carries its contact sideways as the car rolls, by about `h_cg tan φ`:
  2 cm on the default car in a steady turn (φ = 0.04 rad), 17 cm on the off-road preset at
  φ = 0.25 rad, a fifth of its 0.8 m half-track (geometry, not a simulation). The rollover
  margin on steep relief is then likely overestimated; by how much is not measured. Three
  candidates, to prototype and measure on the off-road preset's rollover threshold before one
  is picked:
  1. *as built:* `C = P_att − d n`, the load along `n`; defined at every attitude, no solve;
  2. *the geometry only:* the contact where the body's z axis through the attachment meets
     the tangent plane, `C = P_att − s ẑ_b` with `s = d / (n · ẑ_b)`, the load still the
     spring force along `n`. It fixes the sideways shift, keeps `f` a plain ODE, and needs a
     smooth floor on `n · ẑ_b`, which vanishes with the car on its side;
  3. *the strut as a strut:* the spring carries only the force along `ẑ_b`, so
     `F_spring = Fz (n · ẑ_b) + F_tire · ẑ_b`: the load depends on the tire force, a
     closed-form solve inside `f` while the tire laws are linear in the load.

  With it: the terrain read under the contact, not under the attachment, and a wheel of
  finite radius meeting a bump before its centre does.
- **A vector path for the four wheels** when they share one tire law. Measured on one cloud
  machine (CPU, double precision, medians of 5 to 7): one `f` call takes about 265 µs on
  NumPy (80 µs for the drivetrain bicycle) and about 52 µs compiled (about 6 µs for the
  bicycle); under JAX the Python loop over the wheels costs nothing (unrolled at trace time).
  The cause of the compiled gap is not found: the attitude, the 3×3 solve and the cross
  products take 4 to 6 µs each, alone. The path is not measured.

## 6. Tires, terrains and plants, later

- **A load-sensitive tire law,** relaxation length and the full magic-formula weighting
  functions: behind the same interface, in the private repository unless a course asks.
- **A published fit for the magic formula's shape** (`C`, `E`), cited in the module, in place
  of the default `C = 1.3`, `E = −0.5`.
- **A height map from a grid,** with smooth interpolation (bicubic, not bilinear: no kink at
  every grid line), when a demo needs one. Terrains stay smooth against the wheel radius:
  each wheel touches at one point. A `Hills` terrain, or terrains that add up, wait for a
  demo too.
- **The bridge to the planning fields:** the terrain's height, gradient and friction are the
  workspace fields that [geometry-module.md](geometry-module.md) moves to `core/geometry/`,
  so a cost can avoid steep or slippery ground with no second representation. Overhangs,
  bridges and walls are not terrain (one height per point): they stay shapes placed on it.
- **`QuarterCar` on the corner law and the terrain:** to review, weighed against keeping its
  own equations readable on one page. It stays a standalone demo plant until then.
- **More presets** of the 3-D car, one per look.
- **The project folders' duplicates** of catalog code (the point, the kinematic bicycle, the
  engine and servo rungs, the linear and Pacejka tires): decided when the projects are next
  touched.

## 7. The demo that waits

A path tracker written for the drivetrain bicycle driving the 3-D car through its first nine
states: the model-mismatch study the appending rule gives for free. It waits for the ruling
on the rear axle (QUESTIONS.md: at the default track the locked axle steers 65 % as much as
the bicycle) and for the maintainer's choice of demos.

## 8. Choices taken while building, for the maintainer's look

Taken by the agent during the build, each the recommended option; none is ruled.

- **The suspension:** one `SpringDamper` part per wheel (`k`, `c`, `F_eps`), preloaded with
  the corner's static load, so the car rests at `h_cg` whatever its mass and stiffness and
  the spring carries no free length.
- **No aerodynamic downforce** on the 3-D car: the formula preset's demo keeps its schedule
  under the rear grip.
- **The contact vectors** are a display attribute (`show_contact_vectors`), not a
  constructor option.
- **The default numbers are new and unvalidated:** `DrivetrainBicycle`'s (rung 3's car plus a
  drivetrain and a steering scaled from the racecar's: `P_max` 100 kW, `tau_sat` 2500 N m,
  `Jw_rear` 10 kg m², a slip floor of 0.5 m/s; the 3-D car's `h_cg` and track, held from
  `DynamicBicycle` up, for the `ltr` output, the drawn track narrowing from 1.92 m to 1.6 m; the racecar preset's `h_cg` 0.06 m is an estimate), `SuspensionCar3D`'s (the same car plus a
  track, `h_cg`, the roll and pitch inertias, a heave mode near 1.5 Hz at a damping ratio
  near 0.3), and the two presets' (`FORMULA_CAR_PARAMS`, `OFFROAD_CAR_PARAMS`).
- **The skins of the two presets** are procedural cartoon meshes (no brand, no real vehicle).
- **The solver hint** of the cars with a drivetrain keeps about 4 % of margin under Euler's
  stability limit at rest; a realtime game stepping coarser than the hint makes a parked
  car's rear wheel chatter (up to 0.5 m/s of wheel-surface speed with Euler), and stays
  finite.
- **Names and signatures a review proposed,** each a public change: the `phi_f`, `phi_r` grip
  labels beside the `phi_rear`, `phi_front` wheel angles; `(kappa, alpha)` as the slips'
  return order; one key for the rear wheel's viscous drag (`bw_rear` and `bw_drive` today);
  `rear_wheel_ground_torque` taking `Fx_r` (the tire forces are computed twice per `f`);
  `DrivetrainBicycle.compute_tire_physics` written through the parent's axle forces (it moves
  the racecar's last bits, 1.6·10⁻¹³).
