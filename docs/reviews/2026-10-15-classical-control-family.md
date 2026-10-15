# One classical control family: the rulings (2026-10-15)

**Date:** 2026-10-15
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** After the linear analysis band was settled (the same day's record), the
maintainer asked for the landscape of the linear controllers: a PID with filtered derivative
and anti-windup beside simpler P, PI, PD blocks, an impedance controller that assumes a
measured rate, a proportional controller in a module of its own, and the question of a PD
that is not causal without a rate or a filter. The design converged in conversation; this
file records the rulings, one entry each, and where each landed. Steps S62 and S73 landed
the same day; the plan doc `classical-control-family.md` that carried the draft is gone,
its contract in DESIGN §3 and §4.

---

## Rulings

- **R1 One classical family, one law per block.** `P`, `PI`, `PD`, `PID` in
  `control/siso.py` and the rate-measured `ImpedanceController`,
  `ImpedanceIntegralController` in `control/impedance.py` share one constructor grammar
  (per-axis gains, `dof`, the actuator limits, `e_int_min` / `e_int_max` where there is an
  integrator) and carry exactly the states their terms need, so their poles and zeros are the
  hand calculation's. No mode keyword switches a law: `ports` (the classical four) and
  `tracking_ref` (the impedance pair) are port layouts. Lives in: both modules, DESIGN §4
  "Control laws", the control facade docstring.
- **R2 What is measured decides the derivative.** A block with one measurement per axis
  filters its derivative (a pole at `−1/tau`); a block that measures `[q; dq]` reads the rate
  and carries no filter state. An unfiltered PD on one measurement is improper and has no
  block; `Lead` is the filtered PD in the frequency-domain parametrization. Lives in: the
  `PD` docstring, the `impedance.py` module docstring, DESIGN §4.
- **R3 `P` replaces `ProportionalController`.** `P(Kp, *, dof, ports="error", u_min,
  u_max)`, zero states, display name "P"; `control/output.py` and its API page leave, no
  alias (the freeze is paused between terms). Every call site migrated in the same commit;
  the ones that wire `r` and `y` by hand, and the sampled loop (`P(...) % dt`, which needs a
  measurement port), pass `ports="reference"`. Landed with S62.
- **R4 The family is decoupled; the impedance gains are matrices; the integral gain is
  per axis.** The classical four run `dof` independent loops with per-axis vector gains,
  because the integrator hold pairs a command axis with its own error and a coupled MIMO
  PID would need another anti-windup. `ImpedanceController` and
  `ImpedanceIntegralController` take `n×n` `Kp` and `Kd` (a scalar or a vector is diagonal,
  a full matrix couples the axes, the textbook stiffness and damping), and `Ki` stays a
  per-axis vector for the same hold. A coupled static gain on an output error with no rate
  measured is `Gain(K) @ plant`, verified to give the trajectory of the former matrix
  `ProportionalController`. Lives in: `impedance.py`, `siso.py` (`as_gain_matrix`), DESIGN §4.
- **R5 The limits are parameters, infinite by default; no anti-windup classes.** The plain
  linear law is the default; anti-windup is what the integrator does once a limit is
  declared. Windup is shown with `PID(...) >> Saturation(...)`, its cure with
  `PID(..., u_max=...)` (`examples/demos/control/pid_anti_windup.py`).
- **R6 The shared pieces are pure functions.** `held_integral` (``ė_int = e``, held on an
  axis whose command or integral state sits on a limit), `filtered_rate`
  (``tau ṁ_filt = m − m_filt``) and `clipped` (``u_min ≤ u ≤ u_max``) are module functions
  of `control/siso.py`, arrays in, arrays out, called from each member's `f` and `ctl`; no
  member inherits them from another. `as_dof_vector` moved there with them, so the
  dependency runs robotics → generic. Behaviour-preserving: a 341-record seeded baseline
  compared byte-identical before and after. Landed with S73.
- **R7 The names stay.** `ImpedanceController` and `ImpedanceIntegralController` keep their
  robotics names (the GMC714 vocabulary; `JointImpedance(dof=)` stays the factory), rather
  than a rename into the family's grammar.

## Where a parametric path changes

The impedance pair's `params` now hold `Kp`, `Kd` as `(n, n)` arrays and the limits. A
parametric evaluation that restates a block's params (`rk4_step_p`, `f_p`) writes the three
gains inside the block's own dictionary: `{**ctl.params, "Kp": kp * I, "Ki": ki[None],
"Kd": kd * I}` with `I = eye(n)`. The JAX tuning demo and the showcase notebook do so.
