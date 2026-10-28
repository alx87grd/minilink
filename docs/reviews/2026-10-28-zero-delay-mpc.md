# The zero-delay MPC: the old demos' performance on the new API (2026-10-28)

**Audience:** Prof. Alexandre Girard (Maintainer).
**Ask (2026-10-28):** the delayed MPC stays the default, but the demos keep the old MPC's
performance: a mode with no delay, warm-started, equivalent to the old one, with no regression on
the bicycle demos.
**Branch:** `dev-v0.3-mpc-nodelay`, off `dev-v0.3-hardening` at `9f960138`. The open choices are
in [QUESTIONS](../dev/QUESTIONS.md) ("The zero-delay MPC").

## Why the demos regressed

The old MPC (`warm_start=True`, the demos' form) solved at each tick from the measurement,
warm-started from its last plan, and applied the plan's first input at once. The new MPC has a
delayed form (the plan solved at the previous tick, R123) and a pure one (a cold solve per tick),
neither of which is that controller. At the circuit's 0.2 s period and about 9 m/s, one period of
delay is 1.8 m of lag: the car stalls at about 4 s.

## What the branch adds

`ModelPredictiveController(planner, delay=False)` is now the old controller on the planner's
compiled face (`WarmStartedPredictiveController`, a native sampled law whose state is the last
plan): `z*_k = pv.solve(P_k, z0=warm)`, `u_k = pv.command(z*_k, x̂_k, t_k)`, `z_{k+1} = z*_k`. The
warm start is the last plan shifted one period with its first state knot at the measurement; the
first tick starts from the default guess at the measured start. `warm_start=False` keeps the pure
form. Its outputs (`u`, `z`, `x_ref`, `u_ref`) feed through from the bound ports, so it closes the
dual-rate loop with zero delay too. Each tick solves once: the block keeps its solves by `(t_k,
z_k, P_k)` on the host (the open question). A traced call is never stored: on a native planner the
closed loop is one scan and differentiates.

Every MPC demo moves to `delay=False` with its constants kept. The pathtracking project's wiring is
fixed (its plant exposed `y_mpc`, so `@` refused it on `dev-v0.3-hardening` already).

## The evidence

Distance driven and wall time in this container, the old code at `9be4f023` (before the run):

| Demo | Old code | New default (delayed) | `delay=False` (this branch) |
| --- | --- | --- | --- |
| Integrator (NumPy), final `x` (goal 2) | 1.9926, 0.6 s | 1.9963 | **1.9996**, 0.6 s |
| Car minimal | 19.28 m, 4.1 s | 19.19 m | 19.20 m, 2.8 s |
| Kinematic racecar | 35.94 m, 3.6 s | 33.91 m | 34.05 m, 2.3 s |
| Dynamic racecar | 21.76 m, 11.8 s | 13.97 m | 19.94 m, 7.9 s |
| Slalom | 88.10 m, 11.4 s | 70.77 m, 32.8 s | 84.13 m, 8.9 s |
| Circuit, 24 s | 112.4 m, **stalls at 14 s**, 55 s | 18.8 m, stalls at 4 s | **202.4 m, no stall**, 20 s |
| Dual rate, final `x` | 18.24 | 18.47 | 18.61 |
| Pathtracking | 161.7 m | refused at wiring | stalls (below) |

- **The mechanism is the old controller.** A scratch prototype of this mode that replays the old
  warm-start guess gives the old runs to the bit: the integrator exactly, the kinematic racecar to
  1.8e-14, the car minimal to 2e-8; the dynamic racecar, the slalom and the circuit agree to 1e-9
  until SLSQP's loose `ftol` makes them diverge, with the same distance driven.
- **The old guess had a timing slip.** `mpc_warm_start_guess` re-timed the shifted plan by `k·Δ`
  while the plan was timed from 0, so from tick 2 it read the plan `(k − 1)·Δ` ahead, and after
  `tf/Δ` ticks it was the last plan's next-to-last knot held constant. That poor guess makes SLSQP
  iterate more, and at the demos' loose `ftol` (1.0, 0.1, 0.05) it gave better plans than the
  correct shift: the racecars and the slalom drive 5 to 8 % less on this branch, and solve faster.
  Tightening the dynamic racecar's `SLSQP_FTOL` from 1.0 to 0.1 (an experiment, not committed)
  gives **23.5 m in 11.1 s** against the old 21.8 m in 11.8 s.
- **Pathtracking** starts at a heading of exactly 0, where SLSQP stops with "Singular matrix E in
  LSQ subproblem" at every tick; the old run escaped through its first failed iterates. With the
  circuit demo's own nudge (`theta0 += 1e-4`, here `vehicle.x0[2] = 1e-4`; an experiment, not
  committed) this branch drives the old path: 161.5 m against 161.7 m, within about 1 m at every
  4 s checkpoint.
- **One solve per tick:** 121 solves for the circuit's 120 ticks (the extra one the compile's
  shape check); without the store the loop would solve four times per tick (the command, the
  update, the plant's input and the overlays read back after the run).
- **The bicycle and Ipopt** (RD-33): the Ipopt adapter is the same as at `9be4f023`, neither gives
  cyipopt a Lagrangian Hessian, so that limit is not a regression; `cyipopt` is not installable in
  this container, so it was not run.

## Not done, yours

- The demos' `ftol` retune (the racecars, the slalom), or the old guess replayed instead.
- The pathtracking start heading.
- The notebooks that run an MPC (GRO501 `racecar_toward_mpc`, the showcase, `mpc_spatial_stack`)
  still run the delayed default; this branch does not touch them while the hardening lane is
  open.
