# Vehicle models: a car ladder, a tire module, a terrain, a 3-D car

**Status:** phases 1, 2 and 2b built by run 1 of [vehicle-models-run.md](vehicle-models-run.md),
in review on `vehicle-models-phase1`. The maintainer was interviewed on 2026-11-11; the answers
are recorded in §2. A second review the same day added the rulings at the end of §2's table. An
independent review of the built branch followed (§11): its fixes that keep names and behaviour
are applied, and its three design questions wait in [QUESTIONS.md](../dev/QUESTIONS.md).  
**Lane:** teaching catalog (`minilink/dynamics/catalog/vehicles/`). The specialized models go
to the maintainer's private repository, not to minilink.  
**Rung:** the car part of v0.4's catalog clean-up (the car cut, K8-0 in [TODO.md](TODO.md)),
then a 3-D car after it. It touches the review of the mechanical hierarchy and the cars' base
(S32), and catalog hygiene's "one port layout across the bicycle rungs; one owner for the
wheelbase" (S64).

## 1. Purpose

Consolidate the car models into a small ladder that teaches one idea per rung. Give the
dynamic rungs a shared set of pieces:

- a tire module with three swappable laws;
- a terrain;
- a law for each corner's normal load.

A 3-D car with suspension can then be built on those pieces, and the maintainer's private
research models (off-road) can build on the same pieces.

Who uses it: the maintainer's courses and research. Self-driving is the main use case:
- planning on a kinematic model;
- MPC and estimation on a dynamic one;
- a realistic plant to simulate against;
- tire laws swapped to study model mismatch.

## 2. The maintainer's answers (2026-11-11)

| Topic | Answer |
| --- | --- |
| Users | the maintainer's courses and research; advanced specialized models in the maintainer's private repository |
| Ladder | 1 point → 2 kinematic bicycle → 3 dynamic bicycle → 3b dynamic bicycle with rate inputs → 4 powered car; 5 the longitudinal car as a side step of 3 and 4 (maximum braking and acceleration, no steering); 6a `Car3D`, a double-track rigid car whose four wheel loads follow the accelerations (no suspension; designed for, not built); 6 a 3-D car with suspension on a hard surface, at video-game fidelity |
| 3b | kept public: the trajectory-optimization and MPC demos, the tutorial and the benchmarks use it; its own class (`BicycleDynRate` today), sharing rung 3's chassis through the parent class, so rung 3's `f` stays a clean course page |
| UdeS Racecar | the flagship teaching platform; public at least as a preset. A proxy that behaves roughly like the real car, for testing ideas (today `UdeSRacecarDyn`), never an identified model of it: identifying the real car is the students' work in class. Its public face: thin classes (`UdeSRacecar` on rung 2, `UdeSRacecarDyn` on rung 4) that set only the preset numbers and the skin, no equations; the one written exception to the catalog clean-up's "no two supported plants differ by parameters alone". `UdeSRacecarDyn3D` becomes a `wheel_angles=True` option on rung 4, as the car cut (K8-0) already says |
| Tire | a tire object carrying its force law and its default numbers; the defaults are copied into the car's `params` at construction; three laws, plus a transitional `LinearSaturation` law that keeps today's numbers until phase 3, then retires (§5.1, §7) |
| Terrain | a height `z(x, y)`, its gradient and a friction `μ(x, y)`; given to the car at construction, like the tire; also exported as a static block for custom diagrams |
| Rung 6 and rung 4 | different models, but sharing the names and the positions of states and parameters as far as possible |
| State layout | rung 4's state is the first part of rung 6's state, in the same order; the 3-D states come after it |
| Heading | `psi` on every rung (state label `"psi"`); rung 6 uses roll `φ`, pitch `θ`, yaw `ψ` |
| Suspension | the design supports spring-dampers (level i) and wheel hop (level ii); the first build is level (i) only. Wheel hop is a construction option on rung 6, `massless_wheel=True` by default (level i); `massless_wheel=False` adds each corner's wheel mass, its states appended at the end |
| Load transfer | not on the single track: the bicycle series (rungs 3 to 4) keeps static axle loads, as the transfer is not worth it there. On the double track only: quasi-static on `Car3D` (four wheels, no suspension; designed for, not built), and from the suspension on rung 6. Always an exact continuous model, never a lag on the acceleration (§6). Clarifies the same day's "only on the 3-D car" |
| Slip | today's slip definitions kept during the refactor, so no demo number moves; the smooth formula is tried later, together with the switch from `LinearSaturation` to the brush law, measured before and after (§5.1, §7). Supersedes the same day's answer to switch the slip during phase 1 |
| Parameter names | flat, one suffix per axle (`c_alpha_f`, `k_r`), left and right symmetric, as the racecar and the textbook write them; rung 6 keeps the same; asymmetric corners are a later extension (private repository) |
| Magic formula | default shape `C = 1.3`, `E = −0.5`, both directions: the peak `μ Fz` near the brush law's full slide (12° at `c = 13/rad`), then a fall to `sin(C π/2) ≈ 0.89` of the peak; both numbers in `params`; a fit to a published tire, cited in the module, can replace them when it is written |
| Names | rung 4's generic class is `DrivetrainBicycle` ("powered" is too vague): the UdeS Racecar's `UdeSRacecarDyn` sits on it; rung 6 is `SuspensionCar3D`, named by what it adds |
| `QuarterCar` | a standalone, simpler demo plant, not part of the ladder: kept independent and unchanged for now; rung 6's corner and terrain do not build on it (its support status follows the catalog clean-up's own rule). To review in a later phase: whether rebuilding it on the corner law and the terrain is cleaner, weighed against keeping its own equations readable on one page |
| Rollover | modelled on rung 6: roll passes 90° and 180° (the roll–pitch–yaw attitude is singular only at pitch ±90°, an end-over-end flip, documented); a `rollover` output gives the margin (the smallest wheel load, the roll angle), and a terminal set can end a run at rollover. What the model does after the car rolls past about 90° is not specified and not a concern for now: kept as simple as possible, no special contact rule (only the tire contacts touch the terrain). `Car3D` shows only the onset (an inner wheel's load reaching zero). Body contact points, so the car lands and keeps tumbling, are a later option, off by default |
| Phases | 1 the foundations; 2 the 3-D car; 2b one or two good-looking generic 3-D skins for the 3-D car, two extremes (an open-wheel formula car, an off-road side-by-side); 3 a generic parameter preset for the 3-D car, the UdeS Racecar preset brought to it with its real skin and parameters, and the tire and slip switch (§7) |
| Longitudinal car | left as it is through phases 1 and 2; in phase 3, with the tire and slip switch, it joins the tire module (a `tire=` slot, the module's slip, the ladder's parameter names) |
| Project folders | `car_trajopt/` and `pathtracking/` are read only, as inspiration; this plan does not change them |
| Tire parts (2026-11-11, after run 1's review) | each tire is a part with its own `params`, held by the car like a diagram's subsystem: the tire owns its grip `mu`, its stiffnesses per unit load `c_alpha`, `c_kappa`, its slip floor `v_eps` and its shape (`C`, `E` on the magic formula), and its slip computation. One tire per wheel, at `params["tire"][wheel]`, short wheel names (`f`, `r` on the bicycle; `fl`, `fr`, `rl`, `rr` on the 3-D car), so 4, 6 or 8 different tires fit later. A tire passed at construction is copied to every wheel and wins with its own numbers. The road's grip: see the Friction row. Supersedes the Tire, Tire numbers and Tire call rows |
| Rungs 3 and 3b without a tire | keep their flat `Ca`, `Ck`, `mu`, `v_min_epsilon` (today's numbers) until phase 3; given a tire, they take the per-wheel parts like rung 4 |
| Solver hint (run 1's review) | `DrivetrainBicycle`'s defaults are an easy car to simulate (slip floor 0.5 m/s, `Jw_rear` 10), and its step hint is about 1 ms: a hint for quick runs, not a property of the model |
| Inputs | one stacked `u` on every car; `named_ports=True` keeps the named ports for the demos and diagrams that use them |
| Frames | one frame set on every rung (`body`, `axle_front`, `wheel_fl`, `wheel_fr`, plus `wheel_rl`, `wheel_rr` on the 3-D car), so a 3-D skin's body fits the bicycle too; `KinematicCar` and `DynamicBicycleCar3D` retire; the contact-force and contact-velocity arrows are a display option on the 3-D car (`show_contact_vectors`) |
| Demo bar | a demo passes when it completes its task as before |
| Wheel radius and vertical stiffness | decided in phase 2's wheel-hop step (level ii), not before |
| Friction (after phase 2b) | two owners, multiplied, both 1 by default: the tire owns the rubber's `mu`, the terrain owns the road's factor. Every car rung holds a terrain (flat by default; the planar rungs read only its friction, at their two axles); a uniform road is `params["terrain"]["friction"]`, a map is a terrain subclass overriding `friction(x, y, t)`. Grip at a contact: `mu_tire × terrain.friction(x, y)`. `road_friction` leaves the car. Supersedes the Tire parts row's `road_friction` and the "dropped terrain mu" of the same day |
| Topology (after phase 2b) | the 3-D car is tested and shown on real relief with the built-in terrains, kept simple: rolling hills from `Bumps` at a large wavelength (1 m high, 25–30 m long), no new terrain class. Each wheel touches at one point, so terrains stay smooth against the wheel radius; a `Hills` terrain or terrains that add up wait for a demo |
| Phase 1's bar (review) | the same behaviour on the demos, not identical bytes: a rewrite that reorders the arithmetic is accepted (§7). Clarifies "no demo number moves" above |
| Frame of the first nine (review) | on rung 6, `vx, vy` are the velocities in the heading frame (yawed, not rolled or pitched) and `r = ψ̇`, so they mean what they mean on rung 4 (§4) |
| Differential (review) | the first 3-D plant has a locked rear axle (one `w_rear`); the design allows independent wheel speeds later (§4) |
| Optional states (review) | one order on every rung: the nine, the body's six, the wheel-hop states, the wheel angles last (§4) |
| The switch's place (review) | it stays in phase 3: the prototype is built fully before any demo's behaviour changes |

## 3. Today

An audit on `dev` on 2026-11-11 (`grep` over the repository for each class name) found
**25 car-like plant classes and 6 tire laws**:

| Place | Models | Tire laws |
| --- | --- | --- |
| Catalog (`minilink/dynamics/catalog/vehicles/`) | `HolonomicMobileRobot`, `KinematicBicycle`, `KinematicCar`, `UdeSRacecar`, `DynamicBicycle`, `DynamicBicycleCar3D`, `BicycleDynRate`, `UdeSRacecarDyn`, `UdeSRacecarDyn3D`, `LongitudinalSlipCar`, `LongitudinalTorqueCar`, `QuarterCar`, `MountainCar` | linear with a friction circle; Fiala brush; sigmoid `μ(s)` |
| `examples/projects/car_trajopt/vehicles/` | 7 ladder variants, 3 point and constant-speed extras, `CarProfile` | the catalog's linear law |
| `examples/projects/pathtracking/common/vehicle.py` | `ForceInputBicycle`, `RearDriveBicycle`, `EngineBicycle` | its own linear law and `Pacejka` (NumPy only, parameters as attributes) |
| Experimental tier | — | ANCF flexible ring (a carcass model, not a force law) |

**The physics is mostly the same.** Every planar dynamic model has the same body,
`M v̇ + C v + d = 0` with `M = diag(m, m, I_z)`, a static axle load and drag. **The variants
differ in actuation:** what drives the rear wheel, and how the steering is driven. **Four
classes exist only for their look.**

Defects and traps, each checked on the code, with the phase that fixes it (§7):

| Defect | Fixed in |
| --- | --- |
| **Four slip regularizations near standstill:** `|vx| + ε`, `√(vx² + ε²)`, `max(|vx|, |ωR|, ε)`, and `|v| + 10⁻⁶` clipped to ±0.5 | phase 3's switch (one slip function); the third is the project folders', not changed here (§8) |
| **Gradient kinks in the optimization plants:** the `|vx| + ε` slip, the hard friction circle, and `sign` and `where` in the ladder's engine | the slip and the circle in phase 3's switch; the engine's `sign` is in `car_trajopt/`, not changed here (§8) |
| **`DynamicBicycle` is neutral-steer by default.** With one shared `Ca` and `a = b`, `K = m/L (b/Ca_f − a/Ca_r) = 0` | phase 3's switch: the brush law reads `c_alpha_f` and `c_alpha_r` |
| **`KinematicBicycle` ignores `params["length"]`:** `f` reads `a + b` | phase 1 (one owner for the wheelbase) |
| **The catalog test imports project code:** `test_dynamics_catalog.py` imports `Pacejka` from `examples/projects/` | phase 1 (the magic formula is in the tire module) |

Lateral force at matched cornering stiffness, from the repository's own functions:

- inputs: `C_α = 13 μFz/rad`, `μ = 1`, `Fz = 1`;
- Pacejka at its defaults `B = 10, C = 1.3, D = 1, E = 0.97`;
- computed on NumPy, in one deterministic run.

| α [deg] | 0.5 | 1 | 2 | 4 | 6 | 12 | 20 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| linear + circle | 0.113 | 0.227 | 0.454 | 0.908 | 1.000 | 1.000 | 1.000 |
| brush | 0.109 | 0.210 | 0.389 | 0.661 | 0.837 | 0.999 | 1.000 |
| Pacejka (defaults) | 0.113 | 0.221 | 0.409 | 0.655 | 0.777 | 0.897 | 0.938 |

The brush and Pacejka laws agree to about 4°. At its defaults Pacejka has no peak.

## 4. The ladder

| Rung | Model | State (appended to the rung below it) | Inputs | Teaches |
| --- | --- | --- | --- | --- |
| 1 | point | `x, y` | `vx, vy` | a point that moves where told |
| 2 | kinematic bicycle | `x, y, psi` | `v, δ` | the no-slip constraint, the wheelbase |
| 3 | dynamic bicycle | `+ vx, vy, r` | `w_rear, δ` | mass, yaw inertia, tires |
| 3b | dynamic bicycle, rate inputs | `+ w_rear, δ` | `ẇ_rear, δ̇` | actuators as states: limits become bounds, rates become costs (the MPC plant) |
| 4 | `DrivetrainBicycle` | `+ P` | `P_cmd, δ_cmd` | wheel spin, power-limited drive, servo steering |
| 5 | longitudinal car (side step) | `x, v, ω, φ` | wheel torque | slip ratio, maximum braking and acceleration |
| 6a | `Car3D` (designed for, not built) | rung 4's nine, nothing added | as rung 4 | the double track: four wheels, the track width, quasi-static load transfer (§6) |
| 6 | `SuspensionCar3D` | rung 4's nine, then `z, φ, θ, vz, p, q`, then the optional blocks (below) | as rung 4 | heave, roll, pitch, suspension, load transfer, terrain |

- **One rule, every rung: a rung appends states and never reorders them.** A controller,
  estimator or MPC written for rung 4 therefore reads the first nine states of rung 6
  unchanged. The planar MPC can drive the 3-D car directly, which is a model-mismatch study
  for free.
- **The first nine keep their meaning on rung 6.** `x, y` are the world's horizontal
  position and `psi` the yaw. `vx, vy` are the velocity of the centre of gravity in the
  heading frame (yawed with the car, not rolled or pitched), and `r = ψ̇`. They are not the
  components of the rolled and pitched body twist; the two differ as the roll and the pitch
  grow, most at rollover. The map of the last bullet below does the conversion.
- **Optional states append in one order:** the nine, the body's six, the wheel-hop states
  (`massless_wheel=False`), then the wheel rolling angles (`wheel_angles=True`) last. An
  option that is off leaves no gap. On rung 4 the same rule gives today's layout: the nine,
  then the two angles.
- **The rear axle is locked on the first 3-D plant:** one `w_rear`, both rear wheels at that
  rate, the front wheels rolling free. In a tight turn the two rear tires then slip against
  each other, a push the bicycle does not have; the demo states it. **Independent wheel speeds
  are designed for:** the contact code takes one rate per wheel, so a later option can make
  them states. Proposed for then: the extra rates append with the optional blocks, and
  `w_rear` stays the rear axle's mean rate, so the first nine keep their meaning.
- **A model that differs only by its numbers is a preset, and one that differs only by its look
  is a skin.** That covers today's full-size kinematic car, the 3-D looks, and the wheel
  angles needed by the 3-D animation.
- **The UdeS Racecar is a public preset** on rungs 2 and 4 (and on 6 when it lands): its
  parameter set and its 3-D skin. It is a proxy of the real car, not an identification of it.
  Its class on rung 6 is not named `UdeSRacecarDyn3D`: that name means "the nine states and
  two wheel angles" today, in one course file among others, and would silently change meaning.
  The name is chosen in phase 3.
- **Sensor outputs on rungs 3 and up:** `speed`, `slip`, `grip`, `imu`, by name, as the racecar
  has them today.
- **Rung 6's equations order the body's coordinates as the 6-DOF mechanics needs**
  (`[position, attitude; twist]`), and map to and from the appended state layout in one place.

## 5. The shared pieces

The dynamic rungs (3 to 6) are built from four pieces, each a structural choice fixed at
construction (RULES 2.3). Each piece holds its own numbers, its dict held by the car's `params` at its key, which
stays the one numeric tree: overrides, gradients, parameter sweeps and system identification
reach every number.

```python
car = SuspensionCar3D(tire=BrushTire(), terrain=Bumps(), suspension=SpringDamper())
car.params["tire"]["fl"]["mu"] = 0.6      # one number, at its part's key
car.params["terrain"]["friction"] = 0.7   # the road, for every wheel
```

### 5.1 The tire

```python
kappa, alpha = tire.slip(vx, vy, w, R, params)                 # per wheel, wheel frame
Fx, Fy = tire.forces(kappa, alpha, Fz, friction, params)       # [N]
```

- **The tire owns its numbers** (the Tire parts row of §2): `mu`, `c_alpha`, `c_kappa`, the
  slip floor `v_eps`, and `C`, `E` on the magic formula, in its own `params`, held by the car
  at `params["tire"][wheel]`. The car passes that dict and the road's friction factor under
  the wheel (§5.2); the grip is `friction × mu`, floored at 10⁻⁹ so a road of zero friction
  stays finite.
- **One base class, `Tire`,** holds what every law shares (`slip`, `stiffnesses`,
  `friction_use`); a law writes only its `forces`.

| Law | Adds | Lateral force (longitudinal: the same with `κ`, `c_kappa`) |
| --- | --- | --- |
| linear | the cornering stiffness | `Fy = c_alpha Fz α` |
| brush (Fiala) | the friction limit, reached smoothly | `Fy = c_alpha Fz α · g(c_alpha α / μ)` |
| magic formula (Pacejka) | a peak, then a fall | `Fy = μ Fz sin(C atan(B α − E (B α − atan(B α))))` |

**The transitional law, `LinearSaturation`** (the maintainer, 2026-11-11):
- It is today's `linear_tire_forces` moved behind the interface unchanged: linear forces
  scaled back onto the friction circle `|F| ≤ μ Fz`, with today's absolute stiffnesses
  `Ca`, `Ck` shared by both axles, and today's slip `|vx| + ε`.
- It is the default of rungs 3 and 3b during the refactor, so phase 1 moves no demo's
  behaviour.
- In phase 3, the demos switch to the brush law, and `LinearSaturation`
  retires: the later switch of §7, decided together with the slip.

It does not follow the shared parameterization below, nor the property of being linear in the
load.

- **Shared numbers.** All three laws read the stiffnesses **per unit load**
  (`c_alpha` [1/rad], `c_kappa` [-]); the brush law and the magic formula also read `mu`. The
  linear law has no friction limit and ignores it. The magic formula derives `B = c / (C μ)`, so its
  slope at zero slip is `c`; it adds only its shape `C` and its curvature `E`. Swapping a law
  keeps the small-slip behaviour; between the brush law and the magic formula it also keeps
  the friction limit, and only the shape of the saturation changes.
- **The default shape falls slowly.** Computed at `c = 13/rad`, `μ = 1`: the peak is 1.000
  at 12.2°, then 0.98 at 20° and 0.94 at 40°; the 0.89 is the limit at large slip. If the
  optimal-slip lesson (phase 3) needs a visible peak, raise `C` (1.6 gives a limit of 0.59);
  decided with that lesson.
- **Linear in the load.** With these numbers every law has the form `F = Fz · φ(κ, α)`.
  It is what decides how load transfer acts on the 3-D car (§6).
- **`Fz` is an argument, never computed inside the law.** Its source is the normal-load law
  (§5.3).
- **Who owns the slip** (the maintainer, 2026-11-11):
  - **The car decides** which slip formula it uses, and calls it at one place: one method that
    turns each wheel's contact velocity (wheel frame) into `(κ, α)`. It then hands the slips to
    `tire.forces(...)`.
  - **The formula itself is a function of the tire module**, imported by the car, never
    rewritten in a plant. The tire law stays a pure map from slip to force.
- **The slips during the refactor:** no behaviour moves, so the tire module holds today's two
  formulas, moved unchanged:
  - `|vx| + ε`, today's `tire_slip`, called by rungs 3 and 3b;
  - `√(vx² + ε²)`, the former `axle_slips` written per wheel, called by rung 4 and by every
    new car.

  A tire swapped onto a car uses that car's slip formula.
- **One slip function later.** It replaces the four regularizations and is decided with the
  later switch (§7). The candidate is the smooth form:

  ```python
  # Slips over the rolling speed, its floor smooth at rest:
  # κ = (ω R − vx) / √(vx² + ε²), α = −atan(vy / √(vx² + ε²))
  v_roll = xp.sqrt(vx**2 + v_eps**2)
  # v_roll = xp.abs(vx) + v_eps   # the formula before the switch
  ```

  Against `|vx| + ε` it:
  - is smooth at `vx = 0`;
  - biases the slips by about `ε²/(2 vx²)` instead of `ε/|vx|` (with ε = 0.1 m/s: 0.5 %
    instead of 9 % at 1 m/s, and 0.005 % instead of 1 % at 10 m/s; computed from the
    formulas).

  Its costs:
  - between rest and a few `ε` the tires are stiffer (up to √2 at `vx = ε`); at rest the two
    formulas agree;
  - every number on rungs 3 and 3b moves.

  When it lands, the previous formula stays as a commented line under the new one (the
  maintainer, 2026-11-11).
- **Combined slip: one slip demand for every law** (§10.2). The demand is
  `z = √((c_kappa κ / μ)² + (c_alpha α / μ)²)`, the linear force over the friction limit; a
  law is its shape `F / (μ Fz) = shape(z)`, and the force points along the demand. The linear
  law is `shape(z) = z`, which is independent `Fx` and `Fy`. The magic formula takes its curve
  along `z` instead of scaling two pure-slip forces onto the ellipse: that scaling has a kink
  at the ellipse, and this form has none, stays inside the ellipse by construction, and is
  the pure-slip curve exactly when one slip is zero.
- **Written with `xp`:** NumPy and JAX through one path, C¹ in the slips. The law owns its
  small-slip stiffness, which feeds the solver's time-constant hint and the course's
  linearization.
- **Left out of minilink:** load-sensitive coefficients, relaxation length, and the full
  magic-formula weighting functions. They can be added behind the same interface, in the
  private repository.

### 5.2 The terrain

A hard surface, described as named fields over the plane, each a function of the position
and the time:

```python
z = terrain.height(x, y, t, params)
dz_dx, dz_dy = terrain.gradient(x, y, t, params)
mu = terrain.friction(x, y, t, params)      # multiplies the tire's μ
```

- **The gradient defaults to the exact derivative of the height.** A built-in terrain gives
  it in closed form (NumPy has no autodiff), as `QuarterCar` does today with `z` and `dz`.
- **Its numbers are in `params`:** bump height, slope angle, the friction of an ice patch.
- **A height map from a grid uses smooth interpolation** (bicubic, not bilinear): no kink at
  every grid line.
- **A private subclass can add fields** (soil stiffness, rolling resistance) without changing
  the interface. Soft, deformable ground is out of scope.
- **Exported as a static block** for custom diagrams: wheel positions in; height, normal and
  friction out. This makes no algebraic loop, since the wheel positions depend only on the
  state.
- **The same fields serve planning:** they are the workspace fields that the geometry plan
  (geometry-module.md) moves to `core/geometry/`, so a cost can avoid steep or slippery ground
  with no second representation.
- **Not covered:** overhangs, bridges and walls. A height map has one height per point;
  obstacles stay `core/geometry` shapes placed on it.

### 5.3 The normal load (per corner)

| Law | Where `Fz` comes from | States added | Rung |
| --- | --- | --- | --- |
| static | the weight split, `m g b / L` | none | 3, 3b, 4 (today) |
| quasi-static transfer | the exact solve of §6 | none | 6a (`Car3D`) |
| spring-damper (level i) | compression from the body pose and the terrain height: `Fz = Fz0 + k (h_cg − d) − c ḋ` (§10.7), with a smooth floor at 0 (the wheel lifts off; no kink, as §3 asks of the optimization plants) | none beyond rung 6's body | 6 |
| spring-damper and wheel mass (level ii) | the two-mass quarter car per corner | 2 per corner | 6, with `massless_wheel=False` |

### 5.4 The body

`M v̇ + C(v) v + d = 0`, `q̇ = N(q) v`, in the body frame:

- planar for rungs 3 to 4 and for `Car3D`: `q = [x, y, psi]`;
- 6-DOF for rung 6: position and roll, pitch, yaw.

The contact velocity of each wheel comes from the body twist and is rotated into the wheel
frame. The same code serves two axles or four corners. Where this body sits in the
mechanical hierarchy is the pending review (S32): this plan needs only that the review keep
the body-frame form.

## 6. Load transfer

**Not on the single track** (the maintainer, 2026-11-11): the bicycle series (rungs 3 to 4)
keeps static axle loads. **On the double track, two ways:**

**`Car3D` (rung 6a, designed for, not built): quasi-static, on four wheels, no suspension.**
The body stays planar and rigid. Each wheel's load follows the accelerations of the centre of
gravity. That is a loop: the loads depend on the accelerations, the accelerations on the tire
forces, and the forces on the loads. It is solved exactly, inside `f`:

1. The tires are linear in the load: `F_i = Fz_i φ_i`, where `φ_i` depends on the state
   only (§5.1).
2. The loads are affine in the acceleration `a = (a_x, a_y)`:

   ```text
   Fz_i = Fz_i⁰ + G_i a
   ```

   `G_i` comes from the centre-of-gravity height `h`, the wheelbase, the track and the
   front share of the roll stiffness.
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
- A smooth floor on `Fz` keeps a wheel's lift-off continuous. With the floor the loads are
  no longer affine in the acceleration, so the 2×2 solve is exact only while the four wheels
  carry load; near lift-off it needs the Newton solve below. On a tall, narrow car that is
  ordinary limit cornering (an inner wheel lifts at `a_y = g · track / (2h)`, below `μ g`
  when `track / (2h) < μ`), so `Car3D` is built with the Newton solve from the start.
- A load-sensitive tire also makes the loop nonlinear: the same Newton solve inside `f`.

Its state is rung 4's nine, so `SuspensionCar3D` appends to it as well. `Car3D` is
`SuspensionCar3D`'s limit with an infinitely stiff suspension.

**`SuspensionCar3D` (rung 6): from the suspension, with no loop.** Each corner's load comes
from its spring compression and compression rate, which are states. Pitch under braking moves
load forward; roll in a turn moves it to the outer wheels. The model stays a plain continuous
ODE, with no solve inside `f`.

**What the transfer changes with the module's tire laws.** All three laws are linear in the
load (`F = Fz · φ(slip)`), so an axle's total force is `φ · (Fz_left + Fz_right)`, on both
cars:
- **Front-to-back transfer changes the axle totals.** Braking oversteer shows directly.
- **Side-to-side transfer leaves each axle's total unchanged** while both wheels grip. It
  shows as a wheel lifting off, and on rung 6 as body roll, not as a change of balance.
  Changing the balance (roll-stiffness tuning) needs a load-sensitive tire, whose grip per
  unit load falls as the load rises. That law can be added behind the same interface later,
  or in the private repository.

## 7. Phases

The maintainer's order (2026-11-11). Suspension level (i) is the only one built in phase 2.

- **Phase 1, the foundations.** This phase builds:
  - the tire module;
  - the terrain fields and their static block;
  - the interface of the normal-load law, with the static law;
  - ladder 1 to 5 consolidated: presets, skins, `psi`, one owner for the wheelbase, one port
    layout across the bicycle rungs (S64; the default is asked at that step, §9).

  It sits inside v0.4's catalog clean-up. **No demo's behaviour moves in phase 1:** each tire
  law keeps its slip, and rungs 3 and 3b keep `LinearSaturation`.

  **The bar is the same behaviour on the demos, not identical bytes** (the maintainer,
  2026-11-11). Writing the brush law per unit load reorders its arithmetic: on the racecar's
  numbers, 34 % of 100 000 random slip angles change in the last bit (relative 5·10⁻¹⁶, one
  NumPy run). So each car demo and regression gate is run before and after; a solve that
  still converges to the same trajectory within the gates' tolerances passes. This is the
  one written exception to the byte-identical bar of behaviour-preserving refactors.

  **Done when** every car demo and gate behaves the same, the catalog test no longer imports
  project code, and each law has its tests: slope `c` at zero slip, the friction limit, the
  three laws agreeing at small slip, NumPy and JAX agreeing.
- **Phase 2, the 3-D car.** Rung 6 at suspension level (i): spring-damper corners on the
  terrain, first flat, then the built-in terrains, with a teaching self-driving demo.
  **Done when** the reduction tests pass: on flat ground with stiff springs, the 3-D car's
  steady-state yaw rate in a constant-radius turn matches rung 4's within a stated tolerance
  (a wide track makes the locked axle's push visible, so the test uses a narrow one); at rest
  the four loads sum to the weight; a planar controller written for rung 4 drives it through
  its first nine states.
- **Phase 2b, generic 3-D skins.** Find or design one or two good-looking 3-D skins for
  rung 6, at the quality of the UdeS Racecar's skin, but generic. Two extremes of the same
  model:
  - **an open-wheel formula car:** low, wide, stiff;
  - **an off-road side-by-side:** a high centre of gravity, long suspension travel, the
    rollover-prone one.

  Phase 3's presets can then pair each look with matching numbers. The constraints:
  - **Generic:** never a replica of a real vehicle, and no brand names, logos or recognisable
    styling of a manufacturer.
  - **License:** a found mesh must carry a license the repository can ship (CC0, or CC-BY
    with its attribution recorded beside the asset). Otherwise the skin is built from the
    mesh primitives in `minilink/graphical/meshes.py`, as parts of the racecar skin are.
  - **Size:** about the racecar's assets (580 KB today).
  - **Wheels as separate meshes on their own frames,** so they steer, spin and move with the
    suspension, and the body rolls and pitches visibly.
- **Phase 3, the 3-D car's presets:**
  - a generic parameter preset that drives plausibly (and, with 2b, one per skin);
  - the UdeS Racecar preset brought to rung 6, with its real skin and its parameters (still
    a proxy of the car, §2);
  - **the tire and slip switch** (below).
- **Phase 3's tire and slip switch** (the maintainer, 2026-11-11: in phase 3, after the demo
  clean-up; confirmed at the review, which proposed moving it earlier: the prototype is built
  fully before any demo's behaviour changes). Three moves, decided together, one commit each:
  - rungs 3 and 3b move from `LinearSaturation` to the brush law;
  - every law moves to the one smooth slip function, with the previous slip kept as a
    commented line;
  - the longitudinal car joins the tire module: a `tire=` slot, its slip on the module's
    function, its parameter names aligned with the ladder's (`mu`, `c_kappa`, `CdA`, `r_r`,
    `Jw_rear`, today `mu_max`, `mu_slope`, `cdA`, `wheel_radius`, `wheel_inertia`). Whether
    its default stays the sigmoid is decided then; the magic formula's peak opens the
    optimal-slip lesson (ABS, traction control).

  Until then the longitudinal car is left as it is. Each move is measured before and after:
  - **Reference** (the maintainer, 2026-11-11):
    - for the numbers, the regression gates on the bicycle: trajectory optimization
      (`benchmarks/scenarios/e4_trajopt_parity.py`) and MPC (`f_mpc_parity.py`), against
      their stored baselines, re-recorded once a switch is accepted;
    - the flagship car demo `mpc_car_minimal`, run and looked at on both sides;
    - for the longitudinal car, the course labs' scripts (GRO501 APP2, GMC714 lab 6).
  - **Numbers recorded on both sides:**
    - for each solve: converged or not, iterations, final cost;
    - solve time: the median of 5 warm runs on one machine, the two sides alternated;
    - the largest state difference between the two optimal trajectories;
    - for the MPC: the closed-loop tracking error.
  - **Recorded in a ledger** in this plan.
  - **A switch stays** if every solve still converges, the cost and the tracking error are
    within noise or better, and the time is not slower beyond noise. Otherwise stop and ask.
- **Later, designed for but not scheduled:**
  - `Car3D` (rung 6a);
  - wheel hop (level ii, `massless_wheel=False`);
  - more presets;
  - a load-sensitive tire law;
  - independent wheel speeds on rung 6 (an open differential; §4);
  - body contact points on rung 6 (`body_contact=True`, off by default): a few points on the
    body with the corner's penalty law, so a car that rolls over lands and keeps tumbling.
    This is the edge of "not a contact engine" (CONSTITUTION §1.4), which is why it stays
    unscheduled;
  - the review of rebuilding `QuarterCar` on the corner law and the terrain (§2);
  - **the contact along the body's suspension axis,** for a more realistic off-road
    simulation (§10.7's validity note). Three candidates, to prototype and measure on the
    off-road preset's rollover threshold before one is picked:
    1. *today:* `C = P_att − d n`, the load along `n`; defined at every attitude, no solve;
    2. *the geometry only:* the contact where the body's z axis through the attachment meets
       the tangent plane, `C = P_att − s ẑ_b` with `s = d / (n · ẑ_b)`, the load still the
       spring force along `n`. It fixes the sideways shift, keeps `f` a plain ODE, and needs
       a smooth floor on `n · ẑ_b`, which vanishes with the car on its side;
    3. *the strut as a strut:* the spring carries only the force along `ẑ_b`, so
       `F_spring = Fz (n · ẑ_b) + F_tire · ẑ_b`: the load depends on the tire force, a
       closed-form solve inside `f` while the tire laws are linear in the load.

    With it: the terrain read under the contact, not under the attachment, and a wheel of
    finite radius meeting a bump before its centre does.

## 8. What leaves, what stays

- Inside minilink: today's look-only and parameter-only classes become presets and skins.
  Their names: §2.
- The two project folders stay as they are (read only). Their models inspired this design
  and are not migrated by it.
- Their duplicates of catalog code (the point, the kinematic bicycle, the engine and servo
  rungs, the linear and Pacejka tires) are a later clean-up, decided when the projects are
  next touched.

## 9. Open questions

No design question is open: the interview and the review of 2026-11-11 answered those this
plan raised. A new one goes to QUESTIONS.md, and its answer lands here.

Asked at their step, not before:
- **The one port layout of the bicycle rungs** (phase 1): today rung 3 and the racecar default
  to named ports and rung 3b to one stacked `u`, and 38 lines in the examples, tests and
  benchmarks pass `named_ports`.
- **The class name of the racecar preset on rung 6** (phase 3; §4).

## 10. Implementation

How the pieces are written. The order of the work, each step's "done when" and the rules of
the run are in [vehicle-models-run.md](vehicle-models-run.md).

### 10.1 Files

All under `minilink/dynamics/catalog/vehicles/`:

| File | Holds | Change |
| --- | --- | --- |
| `tires.py` | the slip functions, the four laws, `brush_saturation`, `smooth_sign`, `smooth_floor`, `friction_use` | grows: today it holds the brush law only |
| `terrain.py` | `Terrain` (flat), `Slope`, `Bumps`, the static block | new |
| `steering.py` | rungs 1 and 2 | `KinematicCar` leaves; `length` leaves `params` |
| `dynamic_bicycle.py` | rungs 3 and 3b | the linear tire functions move to `tires.py`; `DynamicBicycleCar3D` leaves |
| `drivetrain_bicycle.py` | rung 4, `DrivetrainBicycle` | new: the equations of today's `UdeSRacecarDyn`, moved |
| `racecar.py` | `PUBLIC_RACECAR_PARAMS`, `UdeSRacecar`, `UdeSRacecarDyn`, `stiff_tires`, `link_frames` | shrinks to numbers and looks |
| `suspension_car.py` | rung 6, `SuspensionCar3D`, and the `SpringDamper` law | new, phase 2 |
| `car_presets_3d.py` | `FormulaCar3D`, `OffRoadCar3D`: their numbers, tires, springs and looks | new, phase 2b |
| `propulsion.py`, `suspension.py`, `mountain_car.py` | the longitudinal cars, `QuarterCar`, `MountainCar` | untouched until phase 3 |

Plant equations stay on each plant's own page (RULES 7.8). What the rungs share is only the
generic blocks of `tires.py` and `terrain.py`: no contact-kinematics helper, no drivetrain
helper, no chassis base class beyond today's `DynamicBicycle` parent (the pending review of
the mechanical hierarchy, S32, decides that one).

### 10.2 The tire laws

Each saturating law is one line: its shape along the slip demand.

```python
class BrushTire(Tire):
    """Fiala brush tire: the friction limit, reached smoothly."""

    def forces(self, kappa, alpha, Fz, friction=1.0, params=None):
        params = self.params if params is None else params
        xp = array_module(kappa, alpha, Fz)

        mu = xp.maximum(friction * params["mu"], GRIP_FLOOR)
        c_alpha = params["c_alpha"]
        c_kappa = params["c_kappa"]

        # Slip demand: the linear force over the friction limit, per direction
        s_x = c_kappa * kappa / mu
        s_y = c_alpha * alpha / mu
        z = xp.sqrt(s_x**2 + s_y**2 + SLIP_DEMAND_FLOOR**2)

        # F = μ Fz shape(z) along the demand, with shape(z) = z g(z) for the brush
        g = brush_saturation(z)
        Fx = mu * Fz * s_x * g
        Fy = mu * Fz * s_y * g

        return Fx, Fy
```

| Law | `shape(z) / z`, the factor on the linear force | Own numbers |
| --- | --- | --- |
| `LinearTire` | `1` | none |
| `BrushTire` | `g(z) = 1 − z/3 + z²/27` while `z < 3`, then `1/z` | none |
| `MagicFormulaTire` | `sin(C atan(b − E (b − atan b))) / z`, with `b = z / C` | `C`, `E` |
| `LinearSaturation` (transitional) | today's `linear_tire_forces`, statement for statement | none |

- **The magic formula in `z`.** `B = c / (C μ)` and `z = c · slip / μ` give `B · slip = z / C`,
  so the curve depends on the demand alone. Its slope at zero is 1 (the linear tire) and its
  peak is 1 (the friction limit).
- **`LinearSaturation` takes absolute stiffnesses.** Its own numbers are `Ck` [N] and
  `Ca` [N/rad], and it does not multiply by `Fz`. This is its
  one documented difference from the other laws, and it leaves with it in phase 3.
- **The slips are two module functions,** each for one wheel, from that wheel's contact
  velocity in its own frame:
  - `tire_slip(vx, vy, w, R, v_eps)`: today's function, `|vx| + ε`, unchanged;
  - `wheel_slip(vx, vy, w, R, v_eps)`: the `√(vx² + ε²)` form, the body of the former
    `axle_slips` written for one wheel.
- **`smooth_floor(F, F_eps)`** is the load floor of §5.3: `½ (F + √(F² + F_eps²))`.
- **Small-slip stiffness.** `c · Fz` for the module's laws, `Ca` for `LinearSaturation`. The
  cars' `fastest_mode` already computes it from `params`; each tire gives it under a load (`tire.stiffnesses(Fz)`).

### 10.3 Parameter keys

- **A part's numbers live in the part's own `params`,** held by the car at a key per wheel:
  `params["tire"]["fl"]["mu"]`, `params["suspension"]["rr"]["k"]`, `params["terrain"]`.
  The keys inside a part carry no prefix (`mu`, `c_alpha`, `C`, `E`, `amplitude`).
- **The car's own numbers stay flat:** `mass`, `a`, `b`, `track`, `h_cg`; the road's
  friction factor is the terrain's (`params["terrain"]["friction"]`).
- **Rungs 3 and 3b keep `Ca`, `Ck`, `mu`, `v_min_epsilon`** when no tire is given, until
  phase 3.

### 10.4 How a car calls its tire

Rung 4's axle forces, after phase 1:

```python
def compute_tire_physics(self, v_body, u_inputs, params=None):
    params = self.params if params is None else params

    tire = self.tire
    mass = params["mass"]
    ...

    vx = v_body[0]
    ...

    # Static axle loads
    Fz_f = mass * gravity * b / (a + b)
    Fz_r = mass * gravity * a / (a + b)

    # Each wheel's tire computes its own slip and force, with its own numbers
    kappa_f, alpha_f = tire_f.slip(vx_f, vy_f, w_f, R_f, params["tire"]["f"])
    kappa_r, alpha_r = tire_r.slip(vx_r, vy_r, w_r, R_r, params["tire"]["r"])
    friction_f, friction_r = self.contact_friction(q, params, t)  # the terrain, at each axle
    Fx_f, Fy_f = tire_f.forces(kappa_f, alpha_f, Fz_f, friction_f, params["tire"]["f"])
    Fx_r, Fy_r = tire_r.forces(kappa_r, alpha_r, Fz_r, friction_r, params["tire"]["r"])

    # Rolling resistance of the free-rolling front axle: −C_rr Fz_f sign(vx)
    Fx_f = Fx_f - C_rr * Fz_f * smooth_sign(vx, v_c)

    return Fx_f, Fy_f, Fx_r, Fy_r
```

- The rolling resistance moves from the tire function to the car: it needs the car's speed,
  which a map from slip to force does not have.
- `brush_tire_forces` leaves; the tests that call it move to `BrushTire().forces`.
- **Rungs 3 and 3b get the same `tire=` slot.** With no tire given, they bind today's path:
  `LinearSaturation`, `tire_slip`, the keys `Ca`, `Ck`, `mu`. Given a module law, they bind
  rung 4's path: the tire's own numbers, one copy per wheel at `params["tire"]`. The choice is made once, at construction (RULES 2.3). Phase 3's
  switch is then a change of default, and its before and after are the same class built twice.

### 10.5 Presets, skins and the names that leave

| Today | After phase 1 |
| --- | --- |
| `UdeSRacecarDyn` (the equations) | `DrivetrainBicycle` holds them; `UdeSRacecarDyn(DrivetrainBicycle)` sets `PUBLIC_RACECAR_PARAMS`, the display sizes and `tf`, nothing else |
| `UdeSRacecarDyn3D` | `UdeSRacecarDyn(wheel_angles=True)`, then `car.skin = racecar_skin_3d`. The option lives on `DrivetrainBicycle`: it appends `phi_rear, phi_front` |
| `UdeSRacecar` | unchanged in use: `KinematicBicycle` with the racecar's numbers and frames |
| `KinematicCar` | retired: `KinematicBicycle` carries the shared frame set and a full-size car's display sizes |
| `DynamicBicycleCar3D` | retired: `DynamicBicycle` with `car.skin = car_skin_3d`; the corner arrows are dropped from the bicycle (the 3-D car shows its contact vectors as an option) |
| (new, phase 2b) | `FormulaCar3D` and `OffRoadCar3D`: the 3-D car with a racer's and a buggy's numbers, tires, springs and looks (`formula_car_skin_3d`, `offroad_car_skin_3d`) |

- **`DrivetrainBicycle`'s default numbers continue rung 3's car:** the same mass, inertia,
  `a`, `b`, radii and drag as `DynamicBicycle`, the per-unit-load stiffnesses derived from
  its `Ca`, `Ck`, and new numbers only for the drivetrain and the steering, scaled from the
  racecar's to a full-size car. They are new and unvalidated: the run lists them for the
  maintainer's review, with a test that the car launches, reaches the top speed its power and
  drag give, and turns.
- **The heading is `psi`** in the state labels and the local names of every car file.
- **The wheelbase has one owner,** `a` and `b`. The `length` key leaves `KinematicBicycle`'s
  `params`; its readers compute `a + b`.

### 10.6 The terrain

```python
class Bumps(Terrain):
    """Sinusoidal bumps along x and y."""

    def __init__(self):
        self.params = {"amplitude": 0.05, "wavelength": 2.0}

    def height(self, x, y, t, params): ...
    def gradient(self, x, y, t, params): ...
    def friction(self, x, y, t, params): ...
```

- `Terrain` itself is the flat ground: height 0, gradient 0, and a uniform friction
  factor `params["friction"]`, 1 by default. A friction map is a subclass overriding
  `friction(x, y, t)`; every car rung reads it under its contacts.
- Each built-in terrain writes its gradient in closed form, and a test checks it against a
  finite difference of the height.
- The static block is one class in `terrain.py`: the positions of `n` points in; their
  height, gradient and friction out.
- **Every car rung holds a terrain** (the Friction row of §2): the planar rungs read its
  friction under their two axles, the 3-D car its height, gradient and friction under its
  four corners, each at the time of the call.
- **The grid height map (bicubic) and the bridge to the planning fields are not built** until
  a demo or the geometry plan's spatial fields need them.

### 10.7 The 3-D car's equations (phase 2)

**State.** `x = [x, y, psi, vx, vy, r, w_rear, delta, P, z, phi, theta, vz, p, q]`.

- `(vx, vy, vz)` is the velocity of the centre of gravity in the heading frame `H` (the world
  frame turned by `psi`), so `ż = vz`.
- `r = ψ̇`, `p = φ̇`, `q = θ̇`: the rates of the three attitude angles. They are the body's
  angular rates at small roll and pitch only.
- Attitude `R = Rz(ψ) Ry(θ) Rx(φ)`.

**Kinematics.** With these coordinates it is the bicycle's:

```text
[ẋ, ẏ] = Rz(ψ) [vx, vy],   ż = vz,   [φ̇, θ̇, ψ̇] = [p, q, r]
```

**Translation,** in the heading frame, which turns at `r` about the vertical:

```text
m (v̇x − r vy) = ΣFx,   m (v̇y + r vx) = ΣFy,   m v̇z = ΣFz − m g
```

**Rotation,** in the body frame. The body rates are `ω = E(φ, θ) [p, q, r]`:

```text
ω₁ = p − r sin θ
ω₂ = q cos φ + r sin φ cos θ
ω₃ = −q sin φ + r cos φ cos θ

I ω̇ + ω × I ω = τ,   then   [ṗ, q̇, ṙ] = E⁻¹ (ω̇ − Ė [p, q, r])
```

`Ė [p, q, r]` is written in closed form (three lines). `E` is singular at `θ = ±90°`, the
end-over-end flip of §2. This block is the one place that maps between the state's layout
and the 6-DOF mechanics (§4).

**Each corner** `i`, from its attachment point `c_i` on the body (body frame):

```text
P_i = p_cg + R c_i                      attachment point, world
n_i = (−∂z/∂x, −∂z/∂y, 1) / ‖·‖         terrain normal under it
d_i = n_i,z (P_i,z − z_terrain)         gap, along the normal
ḋ_i = n_i · Ṗ_i                         gap rate
Fz_i = smooth_floor(Fz0_i + k (h_cg − d_i) − c ḋ_i)   normal load, along n_i
C_i = P_i − d_i n_i                     contact point
```

- The tire force acts at `C_i`, in the plane normal to `n_i`, along the wheel's heading: the
  body's x axis projected on that plane, turned by `δ` on the front wheels.
- The slips come from the velocity of the body's point at `C_i`, in the wheel's frame, through
  `wheel_slip`. The rear wheels both turn at `w_rear`; the front ones roll free (`κ = 0`).
- Because the forces act at the ground and the centre of gravity sits above it, braking
  pitches the car and a turn rolls it: the load transfer of §6, with no extra term.
- **The simplifications,** stated in the class docstring: the spring acts along the terrain
  normal, not along the body's z axis; the terrain is read under the attachment point; `ḋ`
  neglects the terrain's curvature. All three vanish on flat ground at small roll.
- **A simple first model, good on mostly flat terrain only** (the maintainer, 2026-11-12).
  The contact `C = P_att − d n` stays under the attachment along the terrain's normal. A
  strut fixed to the body carries its contact sideways as the car rolls, by about
  `h_cg tan φ` toward the inside of the lean: 2 cm on the default car in a steady turn
  (φ = 0.04 rad), 17 cm on the off-road preset at φ = 0.25 rad, a fifth of its 0.8 m
  half-track (geometry, not a simulation). The outer contact is then drawn too far out, so
  the rollover margin on steep relief is likely overestimated; by how much is not measured.
  To check before the off-road car is used for realistic rollover work: the list of §7,
  "the contact along the body's suspension axis".
- **Past 90° of roll** the same law holds the attachment points off the ground, so the car
  rests on its side or roof and never falls through. Nothing more is specified (§2).

- **The spring is preloaded:** it carries the corner's static load `Fz0_i` at the ride height
  `h_cg`, so the car rests at `h_cg` whatever its mass and stiffness; there is no free length.

**Drive and steering:** rung 4's three equations, written again in this class's `f`, with the
ground torque `(Fx_rl + Fx_rr) r_r`. The class is a model of its own, a `DynamicSystem` leaf,
not a subclass of the drivetrain bicycle: a test pins the numbers the two share and the
drive and servo rates they write alike.

**Outputs:** `y` (the state), `speed`, `slip` (the four `κ`, the four `α`), `grip`, `imu`
(specific force and angular rates in the body frame), `power`, `loads` (the four `Fz`) and
`rollover` (the smallest load, the roll angle).

**Default numbers:** `DrivetrainBicycle`'s, so the two rungs are the same car, plus the
track, the height of the centre of gravity, the roll and pitch inertias, the spring, the
damper and the free length (a heave mode near 1.5 Hz at a damping ratio near 0.3). New and
unvalidated, listed for the maintainer's review.

**Look:** `tf` gives the body its full pose and each wheel a frame at its contact, steered and
lifted with the suspension; `car_skin_3d` draws it until phase 2b's skins.

### 10.8 Names proposed

Public names are the maintainer's. The run uses these and lists them in its pull request:

- tires: `LinearTire`, `BrushTire`, `MagicFormulaTire`, `LinearSaturation`; `wheel_slip`,
  `smooth_floor`;
- 3-D presets and looks: `FormulaCar3D`, `OffRoadCar3D`, `formula_car_skin_3d`,
  `offroad_car_skin_3d`;
- terrain: `Terrain`, `Slope`, `Bumps`, `TerrainBlock`;
- cars: `DrivetrainBicycle`, `SuspensionCar3D` (both ruled, §2), `SpringDamper`;
- options: `tire=`, `terrain=`, `suspension=`, `wheel_angles=`, `massless_wheel=`.

## 11. The independent review of the built branch

A second-opinion review of `vehicle-models-phase1` (two independent reviewers, one reproducing
the physics, one judging the architecture) found the equations right: momentum balances to
10⁻¹⁰ on 200 random states on flat ground, a slope and bumps; the loads at rest sum to the
weight; the load transfer matches theory; the planar rungs are bit-identical to `dev`.

Applied, every number unchanged on a seeded baseline of every rung:

- one `Tire` base class; a grip floor on a road of zero friction (it gave NaN on arrays, a division error on
  floats);
- one axle-force body on the bicycles; the terrain read at the time of the call on every
  catalog rung (`t` comes last on the bicycles' hooks, after `params` and `q`, so the
  read-only project rungs that call them without it still read `t = 0`; on the 3-D car
  it sits before `params`, as on `f`);
- `SuspensionCar3D` a model of its own (four inherited planar methods and five bound
  helpers raised `KeyError` on it); the presets in `car_presets_3d.py`, each applied in one step that also sets the command
  limits and the rest state (`set_numbers`, `rest_state()`);
- the nine simulation tests of the 3-D car skip without JAX; its demos run on NumPy.

Left for the maintainer, in QUESTIONS.md: the rear axle of the 3-D car (locked, it steers
65 % as much as the drivetrain bicycle at the default track), the point the kinematic
bicycle's `(x, y)` follows, and the parts' public face (constructor numbers on the terrains,
where the parts are imported from, `TerrainBlock`).

Known and left as they are:

- the solver hint keeps about 4 % of margin under Euler's stability limit at rest, and the
  realtime game steps about seven times coarser than the hint: a parked car's rear wheel
  chatters there (up to 0.5 m/s of wheel-surface speed with Euler), and stays finite;
- `x0` and the command limits are set at construction and by `set_numbers`: after a direct
  edit of `params["h_cg"]`, `car.x0 = car.rest_state()` brings the car back to rest.

