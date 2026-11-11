# Vehicle models: one tire module, one car ladder, a 3-D future

**Status:** draft, opened 2026-11-11 from the maintainer's direction (below); nothing ruled,
nothing landed. Its decisions (`VM-D1` to `VM-D10`) wait for the maintainer, one at a time.  
**Lane:** teaching catalog (`minilink/dynamics/catalog/vehicles/`) and one research project
(`examples/projects/`).  
**Rung:** v0.4's catalog clean-up. Once ruled, this plan is the design of the car part of
that clean-up (the car cut, K8-0 in [TODO.md](TODO.md)). It also touches the review of the
mechanical hierarchy and the cars' base (S32), and catalog hygiene's "one port layout across the
bicycle rungs; one owner for the wheelbase" (S64).

## 1. Direction and constraints

The maintainer's direction (2026-11-11):

1. **A big consolidation, but keep a bit more in minilink than a bare minimum:** self-driving
   will be a major use case.
2. **One tire module in minilink with three tire models**, one unified interface, swappable on
   a plant.
3. **A simplified modelling ladder** in minilink; the specialized models go to a project.
4. **The UdeS Racecar is UdeS's flagship teaching platform:** it stays public, at least as a
   preset.
5. **Future-proof:** a 3-D car with pitch and roll dynamics and suspension will come. What
   lands now must not have to be undone for it.

What self-driving asks of the library, and what this plan therefore keeps public: a kinematic
model for planning, a dynamic model for MPC and estimation, a realistic actuated plant to
simulate against, tire laws that can be swapped to study model mismatch (plan on one law, drive
on another), and sensor outputs (speed, slips, grip, IMU) on the plants.

## 2. Today

Measured by an audit on `dev` (2026-11-11; `grep` over the repository for each class name).
It found **25 car-like plant classes and 6 tire laws** in four places:

| Place | Models | Tire laws |
| --- | --- | --- |
| Catalog (`minilink/dynamics/catalog/vehicles/`) | `HolonomicMobileRobot`, `KinematicBicycle`, `KinematicCar`, `UdeSRacecar`, `DynamicBicycle`, `DynamicBicycleCar3D`, `BicycleDynRate`, `UdeSRacecarDyn`, `UdeSRacecarDyn3D`, `LongitudinalSlipCar`, `LongitudinalTorqueCar`, `QuarterCar`, `MountainCar` | linear with a friction circle (`dynamic_bicycle.py`); Fiala brush (`tires.py`); sigmoid `μ(s)` (`propulsion.py`) |
| Trajectory-optimization project (`car_trajopt/vehicles/`) | `Holonomic`, `HolonomicAccel`, `BicycleKin`, `BicycleAcc`, `BicycleDynTauRate`, `BicycleDynServo`, `BicycleDynEngine`, three point and constant-speed extras; `CarProfile` presets | the catalog's linear law |
| Path-tracking project (`pathtracking/common/vehicle.py`) | `ForceInputBicycle`, `RearDriveBicycle`, `EngineBicycle` | its own `LinearTire`, `Pacejka` (NumPy only, parameters as attributes) |
| Experimental tier | — | ANCF flexible ring (a carcass model, not a force law; stays there) |

How the models vary:

- **The physics is mostly the same.** Every planar dynamic model has the same body
  (`M v̇ + C v + d = 0`, `M = diag(m, m, I_z)`), a static axle load and drag.
- **They differ in actuation:** what drives the rear wheel (wheel rate, wheel acceleration,
  torque, lagged torque, power, throttle, force) and the steering (angle, rate, servo).
- **Four classes exist only for their look.**

What the audit found wrong, each checked on the code:

- Four slip regularizations near standstill: `|vx| + ε`, `√(vx² + ε²)`, `max(|vx|, |ωR|, ε)`,
  and `|v| + 10⁻⁶` clipped to ±0.5.
- Gradient kinks in the plants used for optimization: the `|vx| + ε` slip, the hard friction
  circle, and `sign` / `where` in the ladder's engine.
- `DynamicBicycle` is neutral-steer by default. With one shared `Ca` and `a = b`, the
  understeer gradient `K = m/L (b/Ca_f − a/Ca_r)` is exactly 0.
- `KinematicBicycle` ignores `params["length"]`: `f` reads `a + b`.
- The path-tracking vehicle has 0 reads of `params`. The library's catalog test imports its
  `Pacejka` from `examples/projects/`.

Lateral force at matched cornering stiffness, from the repository's own functions
(`C_α = 13 μFz/rad`, `μ = 1`, `Fz = 1`, Pacejka at its defaults `B = 10, C = 1.3, D = 1,
E = 0.97`; NumPy, one deterministic run):

| α [deg] | 0.5 | 1 | 2 | 4 | 6 | 12 | 20 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| linear + circle | 0.113 | 0.227 | 0.454 | 0.908 | 1.000 | 1.000 | 1.000 |
| brush | 0.109 | 0.210 | 0.389 | 0.661 | 0.837 | 0.999 | 1.000 |
| Pacejka (defaults) | 0.113 | 0.221 | 0.409 | 0.655 | 0.777 | 0.897 | 0.938 |

The brush and Pacejka laws agree to about 4°. At its defaults Pacejka has no peak, so today it
does not show the one behaviour that sets it apart (the force falling past the peak).

## 3. The tire module

One file, `minilink/dynamics/catalog/vehicles/tires.py`, holds the three laws, the one slip
function, and the grip measure. It is a teaching ladder of its own; each law adds one idea:

| Law | Adds | Force (pure slip, lateral; longitudinal the same with `κ`, `c_kappa`) |
| --- | --- | --- |
| **Linear** | the cornering stiffness | `Fy = c_alpha Fz α` |
| **Brush (Fiala)** | the friction limit `μ Fz`, reached smoothly | `Fy = c_alpha Fz α · g(z)`, `z = c_alpha α / μ`, `g` the brush saturation |
| **Magic formula (Pacejka)** | a peak, then a fall to sliding friction | `Fy = μ Fz sin(C atan(B α − E (B α − atan(B α))))` |

### 3.1 One interface

Every law is evaluated **per wheel**, on the wheel's slips and its normal load:

```python
Fx, Fy = tire.forces(kappa, alpha, Fz, params)   # wheel frame, [N]
```

- `Fz` is an argument, never computed inside the law. Today the plant passes the static axle
  load. The 3-D car will pass the suspension's corner load, through the same call (§5).
- `params` holds the law's coefficients (numbers only, RULES 2.3). Which law runs is a
  structural choice fixed at construction: `DynamicBicycle(tire=BrushTire())` (`VM-D1`).
- Every core law is written once with `xp`, traces under JAX, and is C¹ in the slips, so
  trajectory optimization and gradients see no kink (`VM-D2` covers the linear law's limit).
- The law owns its small-slip stiffness, so `fastest_mode()` and the course's linearization
  read it from the tire, not from a formula copied into each plant.

### 3.2 One parameterization shared by the three laws

All three laws read the same three numbers per axle: the friction coefficient `mu`, and the
slip stiffnesses **per unit load**, `c_alpha` [1/rad] and `c_kappa` [-]. Today's racecar uses
this convention. The magic formula derives its stiffness factor from them:
`B = c / (C μ)`, so its slope at zero slip is `B C D = c` per unit load, with `D = μ`. It adds
only its shape `C` and curvature `E`. So **swapping a law keeps the small-slip behaviour and the
friction limit the same; only the shape of the saturation changes.** That is what a
model-mismatch study needs. Load sensitivity (`μ` and `c` falling with `Fz`) can enter a law
later without changing the interface.

### 3.3 One slip function, combined slip

- `wheel_slips(vx, vy, omega, R, v_eps)` → `(kappa, alpha)`, on the wheel-frame contact
  velocity, with the smooth floor `√(vx² + ε²)`. It replaces the four regularizations.
- **Combined slip:**
  - the linear law: independent `Fx` and `Fy`;
  - the brush law: its one combined slip demand (today's ellipse);
  - the magic formula: the pure-slip forces scaled onto the friction ellipse.

  The full magic-formula weighting functions (`G_xα`, `G_yκ`) stay in the project (§6).
- Rolling resistance belongs to the wheel, not to the tire law. It leaves `brush_tire_forces`
  for the plant's wheel terms.
- `friction_use` (the used fraction of the ellipse) stays and becomes the standard `grip`
  output.

## 4. The modelling ladder

Four rungs in the catalog, each adding one physical idea. The longitudinal car is a side
branch for the courses. A model that differs only by its numbers is a **preset**, one that
differs only by its look is a **skin**.

| Rung | Model | State | Inputs | Adds | Presets and skins |
| --- | --- | --- | --- | --- | --- |
| 0 | `HolonomicMobileRobot` | `x, y` | `vx, vy` | a point that moves where told | — |
| 1 | `KinematicBicycle` | `x, y, θ` | `v, δ` | the no-slip constraint, the wheelbase | full-size car (4-wheel skin, today `KinematicCar`); **UdeS Racecar** (3-D skin, today `UdeSRacecar`) |
| 2 | `DynamicBicycle` | + `vx, vy, r` | `w_rear, δ` | mass, yaw inertia, **tires** (swappable) | 3-D skin (today `DynamicBicycleCar3D`) |
| 3 | the actuated car (`VM-D7` names it) | + `w_rear, δ, P` | `P_cmd, δ_cmd` | wheel spin, power-limited drive, servo steering | **UdeS Racecar** (today `UdeSRacecarDyn`); full-size car; wheel angles for the 3-D skin (today `UdeSRacecarDyn3D`) |
| side | `LongitudinalTorqueCar` | `x, v, ω, φ` | wheel torque | slip ratio and traction in 1-D | — |
| later | the 3-D car (§5) | 6-DOF body, 4 corners | per corner | pitch, roll, heave, suspension, load transfer | — |

- **The UdeS Racecar** is a public preset on rungs 1 and 3: its parameter set
  (`PUBLIC_RACECAR_PARAMS` today) and its 3-D skin, under a public name (`VM-D6`).
- **Rung 3 is the generalized racecar.** `UdeSRacecarDyn`'s drive and servo are already the
  smooth versions of the ladder's `BicycleDynEngine` and `BicycleDynServo`, so those two collapse
  into it.
- **The rate-input bicycle** (`BicycleDynRate`, the MPC plant: tutorial, MPC demos, benchmarks)
  is rung 2 with integrators on its inputs. Where it lives is `VM-D5`.
- **Sensor outputs on rungs 2 and 3:** `speed`, `slip`, `grip`, `imu`, as the racecar has them
  today. Estimation and self-driving read them by name.
- `QuarterCar` and `MountainCar` are not handling models. `MountainCar` follows the catalog
  clean-up's rule. `QuarterCar` is the seed of the 3-D car's suspension corner (`VM-D9`).

## 5. Ready for the 3-D car

Not built now; these choices are made now so that it fits without undoing rungs 2 and 3:

1. **Per-wheel tire calls with `Fz` as an argument** (§3.1). Load transfer, longitudinal and
   lateral, enters by computing `Fz` per corner; the tire module does not change.
2. **Contact velocity in the wheel frame.** The plant computes each contact point's velocity
   from the body twist and rotates it into the wheel frame before `wheel_slips`. The same code
   serves two axles or four corners. Camber and the steer of each wheel enter as that frame's
   rotation.
3. **The body as a body-frame mechanical system:** `M v̇ + C(v) v + d = 0`, `q̇ = N(q) v`, as
   `DynamicBicycle` writes it today. The 3-D body is the same equation with `q` = position and
   roll, pitch, yaw, and `v` the 6-D twist. Where this sits in the mechanical hierarchy is the
   pending review of the mechanical hierarchy and the cars' base (S32): this plan needs only
   that the review keep the body-frame form.
4. **One state layout on every rung:** `[pose; body velocities; wheel spins; actuator states;
   display-only angles]`. A rung appends; it never reorders. This is catalog hygiene's "one
   port layout across the bicycle rungs" (S64).
5. **Corner-indexed parameters.** Today, axle suffixes (`c_alpha_f`, `c_alpha_r`). The 3-D
   car needs four corners and per-corner suspension (`k`, `b`, the unsprung mass). The naming
   chosen now must extend to `fl, fr, rl, rr` (`VM-D3`).
6. **The ground** is the plane `z = 0` today. The 3-D car will read a terrain height
   `z(x, y)` (the quarter car's road profile becomes a terrain field). That is for the 3-D
   car's own plan, not this one.

## 6. The vehicle project

One folder, `examples/projects/vehicles/` (today's `car_trajopt/vehicles/` grown into it),
the home of every specialized model. All of them are built on the catalog's body, tire module
and slip function, so none copies them:

- the research rungs that remain classes: `HolonomicAccel`, `BicycleAcc` (no-slip with
  acceleration input), `BicycleDynTauRate` (torque-driven wheel), the point and constant-speed
  extras;
- `CarProfile` and its named envelopes (passenger car, race car, 1:5), the planning-limits
  layer;
- the full magic-formula combined slip (the weighting functions), relaxation length (tire
  lag), load-sensitive coefficients;
- the path-tracking vehicle, rebuilt on the catalog with `params`: the front-wheel spin state
  and the throttle engine are the only parts it adds;
- the cross-fidelity maps between rungs (TODO §6's later idea), if they return.

The duplicates are deleted, not moved: `Holonomic` (same as rung 0), `BicycleKin` (rung 1),
`BicycleDynEngine` and `BicycleDynServo` (rung 3), and the path-tracking `LinearTire` and
`Pacejka` (the tire module).

## 7. Steps

One commit each. The steps that move a number carry a ledger (the plant, the demo, the
number before and after); the others are byte-identical against a seeded baseline (AGENTS.md).

- **VM-1 The tire module.** The three laws, the interface, `wheel_slips`; no plant changes.
  Tests:
  - each law at zero slip gives the same stiffness;
  - `|F| ≤ μ Fz` for the brush and magic-formula laws;
  - continuity of the value and the slope across the brush branch;
  - NumPy and JAX agree.
- **VM-2 Rung 3 on the module.** The racecar takes `tire=`, brush by default. Byte-identical,
  since it already runs the brush law on the smooth slips.
- **VM-3 Rung 2 on the module.** `DynamicBicycle` with per-axle stiffness per unit load and the
  default tire ruled by `VM-D10`. Numbers move: GMC714 lab 6, every `BicycleDynRate` demo, the
  benchmarks.
- **VM-4 Presets and skins** (K8-0). `KinematicCar`, `DynamicBicycleCar3D`, `UdeSRacecar` and
  `UdeSRacecarDyn3D` folded as `VM-D6` and `VM-D7` rule. The wheelbase gets one owner. The
  GRO501 notebook and the demos migrate in the same change.
- **VM-5 The rate-input bicycle**, as `VM-D5` rules.
- **VM-6 The vehicle project.** The duplicates deleted, the research rungs and the path-tracking
  vehicle onto the catalog, the catalog test no longer importing project code.
- **VM-7 Docs.** DESIGN's catalog section, the README's catalog line, the TRL ledger, ROADMAP
  §4's course rows if a name changed.
- **Later: the 3-D car**, its own plan when scheduled. §5 is its contract with this one.

## 8. Decisions for the maintainer

In this order; each one's recommendation is the agent's. Each goes to QUESTIONS.md when it
comes up, and its ruling lands back here.

- **VM-D1 How a plant takes its tire.** The options:
  - (a) a small tire object with `forces()` and its default coefficients:
    `DynamicBicycle(tire=BrushTire())`;
  - (b) a string choosing among the three: `tire="brush"`;
  - (c) the force function itself: `tire=brush_forces`.

  (a) lets a project add a law without touching the core; (b) does not; (c) leaves the law's
  own coefficients without an owner. **Recommendation: (a).**
- **VM-D2 The linear law's limit.** Is the linear law pure (`F = c Fz α`, no limit: the
  textbook, the brush law then adds the limit), or linear with a friction circle, as
  `DynamicBicycle` has today (a kink in the gradient)? **Recommendation:** pure.
- **VM-D3 Parameter names for axles and corners.** Flat suffixed keys (`c_alpha_f`, later
  `c_alpha_fl`), or one nested dict per axle or corner (`params["front"]["c_alpha"]`). No leaf
  plant nests its `params` today, so nesting needs a check against the parameter trees
  (`core/params.py`) and compile first. **Recommendation:** flat suffixes now; nesting decided
  with the 3-D car.
- **VM-D4 The magic formula's defaults.** Re-fit `C` and `E` so that the curve peaks (dry
  asphalt: the peak `μ Fz` near 6°–8°, then a fall), with `B` derived (§3.2).
  **Recommendation:** yes, with the fitted numbers and their source in the module.
- **VM-D5 The rate-input bicycle.** The options:
  - (a) keep `BicycleDynRate` as the one extra class;
  - (b) an `inputs="rates"` construction option on `DynamicBicycle`;
  - (c) integrators wired in front of rung 2 (a diagram), measured for MPC speed against the
    leaf first.

  **Recommendation:** (a) for v0.4, the least churn; (c) evaluated later with numbers.
- **VM-D6 The UdeS Racecar's public face.** The options:
  - (a) keep `UdeSRacecar` and `UdeSRacecarDyn` as thin classes that set only the preset and
    the skin. This is a stated exception to the catalog clean-up's "no two supported plants
    differ by parameters alone";
  - (b) a preset argument on the generic models: `KinematicBicycle(preset="udes_racecar")`;
  - (c) a public parameter set plus a skin, assembled by the user.

  **Recommendation:** (a). It is the name students and the GRO501 notebook know, and a flagship
  earns the exception.
- **VM-D7 Rung 3's name.** Keep `UdeSRacecarDyn` as the generic actuated car, or give the
  rung a generic name (for example `PoweredBicycle`), with the UdeS Racecar its preset.
  **Recommendation:** a generic name, decided with `VM-D6` so the notebooks migrate once.
- **VM-D8 The longitudinal car's tire.** Keep its sigmoid `μ(s)` (in two courses), or move it
  onto the tire module's longitudinal slice. **Recommendation:** keep the sigmoid until a course
  term ends, then decide.
- **VM-D9 `QuarterCar`.** Keep it in the catalog as the 3-D car's future suspension corner
  (cleaned up then), or let it follow the catalog clean-up's rule (used by its test alone,
  so it moves to the experimental tier). **Recommendation:** keep it.
- **VM-D10 Rung 2's default tire.** Linear (the course's textbook) or brush (a realistic
  limit). **Recommendation:** linear on rung 2, since GMC714 lab 6 linearizes it; brush on
  rung 3.
