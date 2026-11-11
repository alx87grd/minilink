# Vehicle models: a car ladder, a tire module, a terrain, a 3-D car

**Status:** designed; nothing built. The maintainer was interviewed on 2026-11-11; the answers
are recorded in §2, and no question is open (§9).  
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
| Phases | 1 the foundations; 2 the 3-D car; 3 a good skin and generic parameter preset for the 3-D car, the UdeS Racecar preset brought to it with its real skin and parameters, and the tire and slip switch (§7) |
| Longitudinal car | left as it is through phases 1 and 2; in phase 3, with the tire and slip switch, it joins the tire module (a `tire=` slot, the module's slip, the ladder's parameter names) |
| Project folders | `car_trajopt/` and `pathtracking/` are read only, as inspiration; this plan does not change them |

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

Defects and traps, each checked on the code:

- **Four slip regularizations near standstill:** `|vx| + ε`, `√(vx² + ε²)`,
  `max(|vx|, |ωR|, ε)`, and `|v| + 10⁻⁶` clipped to ±0.5.
- **Gradient kinks in the optimization plants:** the `|vx| + ε` slip, the hard friction
  circle, and `sign` and `where` in the ladder's engine.
- **`DynamicBicycle` is neutral-steer by default.** With one shared `Ca` and `a = b`,
  `K = m/L (b/Ca_f − a/Ca_r) = 0`.
- **`KinematicBicycle` ignores `params["length"]`:** `f` reads `a + b`.
- **The catalog test imports project code:** `test_dynamics_catalog.py` imports `Pacejka`
  from `examples/projects/`.

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
| 6 | `SuspensionCar3D` | rung 4's nine, then `z, φ, θ, vz, p, q`, then the corner states (§5.3) | as rung 4 | heave, roll, pitch, suspension, load transfer, terrain |

- **One rule, every rung: a rung appends states and never reorders them.** A controller,
  estimator or MPC written for rung 4 therefore reads the first nine states of rung 6
  unchanged. The planar MPC can drive the 3-D car directly, which is a model-mismatch study
  for free.
- **A model that differs only by its numbers is a preset, and one that differs only by its look
  is a skin.** That covers today's full-size kinematic car, the 3-D looks, and the wheel
  angles needed by the 3-D animation.
- **The UdeS Racecar is a public preset** on rungs 2 and 4 (and on 6 when it lands): its
  parameter set and its 3-D skin. It is a proxy of the real car, not an identification of it.
- **Sensor outputs on rungs 3 and up:** `speed`, `slip`, `grip`, `imu`, by name, as the racecar
  has them today.
- **Rung 6's equations order the body's coordinates as the 6-DOF mechanics needs**
  (`[position, attitude; twist]`), and map to and from the appended state layout in one place.

## 5. The shared pieces

The dynamic rungs (3 to 6) are built from four pieces, each a structural choice fixed at
construction (RULES 2.3). Each piece copies its default numbers into the car's `params`, which
stays the one numeric tree: overrides, gradients, parameter sweeps and system identification
reach every number.

```python
car = SuspensionCar3D(tire=BrushTire(), terrain=Bumps(), suspension=SpringDamper())
car.params["mu_f"] = 0.6          # one number, on the car
```

### 5.1 The tire

```python
Fx, Fy = tire.forces(kappa, alpha, Fz, params)   # per wheel, wheel frame [N]
```

| Law | Adds | Lateral force (longitudinal: the same with `κ`, `c_kappa`) |
| --- | --- | --- |
| linear | the cornering stiffness | `Fy = c_alpha Fz α` |
| brush (Fiala) | the friction limit, reached smoothly | `Fy = c_alpha Fz α · g(c_alpha α / μ)` |
| magic formula (Pacejka) | a peak, then a fall | `Fy = μ Fz sin(C atan(B α − E (B α − atan(B α))))` |

**The transitional law, `LinearSaturation`** (the maintainer, 2026-11-11):
- It is today's `linear_tire_forces` moved behind the interface unchanged: linear forces
  scaled back onto the friction circle `|F| ≤ μ Fz`, with today's absolute stiffnesses
  `Ca`, `Ck` shared by both axles, and today's slip `|vx| + ε`.
- It is the default of rungs 3 and 3b during the refactor, so phase 1 moves no demo number.
- In phase 3, the demos switch to the brush law, and `LinearSaturation`
  retires: the later switch of §7, decided together with the slip.

It does not follow the shared parameterization below, nor the property of being linear in the
load.

- **Shared numbers.** All three laws read `mu` and the stiffnesses **per unit load**
  (`c_alpha` [1/rad], `c_kappa` [-]). The magic formula derives `B = c / (C μ)`, so its
  slope at zero slip is `c`; it adds only its shape `C` and its curvature `E`. Swapping a law
  keeps the small-slip behaviour and the friction limit; only the shape of the saturation
  changes.
- **Linear in the load.** With these numbers every law has the form `F = Fz · φ(κ, α)`.
  It is what decides how load transfer acts on the 3-D car (§6).
- **`Fz` is an argument, never computed inside the law.** Its source is the normal-load law
  (§5.3).
- **Who owns the slip** (the maintainer, 2026-11-11):
  - **The car decides** which slip formula it uses, and calls it at one place: one method that
    turns each wheel's contact velocity (wheel frame) into `(κ, α)`. It then hands the slips to
    `tire.forces(kappa, alpha, Fz, params)`.
  - **The formula itself is a function of the tire module**, imported by the car, never
    rewritten in a plant. The tire law stays a pure map from slip to force.
- **The slips during the refactor:** no number moves, so the tire module holds today's two
  formulas, moved unchanged (same operations, same order, so the results stay
  bit-identical):
  - `|vx| + ε`, today's `tire_slip`, called by rungs 3 and 3b;
  - `√(vx² + ε²)`, today's `axle_slips` written per wheel, called by rung 4 and by every
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
- **Combined slip:**
  - linear: independent `Fx` and `Fy`;
  - brush: its one combined slip demand;
  - magic formula: the pure-slip forces scaled onto the friction ellipse.
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
| spring-damper (level i) | compression from the body pose and the terrain height: `Fz = k Δ + b Δ̇`, floored at 0 (the wheel lifts off) | none beyond rung 6's body | 6 |
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
- A smooth floor on `Fz` keeps a wheel's lift-off continuous.
- A load-sensitive tire makes the loop nonlinear: a Newton solve inside `f`.

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
  - ladder 1 to 5 consolidated: presets, skins, `psi`, one owner for the wheelbase.

  It sits inside v0.4's catalog clean-up. **No demo number moves in phase 1:** each tire law
  keeps its slip, and rungs 3 and 3b keep `LinearSaturation`.
- **Phase 2, the 3-D car.** Rung 6 at suspension level (i): spring-damper corners on the
  terrain, first flat, then the built-in terrains, with a teaching self-driving demo.
- **Phase 3, the 3-D car's look and presets:**
  - a good 3-D skin for rung 6;
  - a generic parameter preset that drives plausibly;
  - the UdeS Racecar preset brought to rung 6, with its real skin and its parameters (still
    a proxy of the car, §2);
  - **the tire and slip switch** (below).
- **Phase 3's tire and slip switch** (the maintainer, 2026-11-11: in phase 3, after the demo
  clean-up). Three moves, decided together, one commit each:
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
  - the review of rebuilding `QuarterCar` on the corner law and the terrain (§2).

## 8. What leaves, what stays

- Inside minilink: today's look-only and parameter-only classes become presets and skins.
  Their names: §2.
- The two project folders stay as they are (read only). Their models inspired this design
  and are not migrated by it.
- Their duplicates of catalog code (the point, the kinematic bicycle, the engine and servo
  rungs, the linear and Pacejka tires) are a later clean-up, decided when the projects are
  next touched.

## 9. Open questions

None: the interview of 2026-11-11 answered every question this plan raised. A new question
goes to QUESTIONS.md, and its answer lands here.
