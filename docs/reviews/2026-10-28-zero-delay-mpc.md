# The zero-delay MPC: the old demos' behaviour on the new API (2026-10-28)

**Audience:** Prof. Alexandre Girard (Maintainer).
**Ask (2026-10-28):** the delayed MPC stays the default; the demos get the old MPC back, zero
delay and warm-started, with no regression on the bicycle demos.
**Branch:** `dev-v0.3-mpc-nodelay`, off `dev-v0.3-hardening` at `9f960138`.

## Why the demos regressed

The old MPC (`warm_start=True`, the demos' form) solved at each tick from the measurement,
warm-started from its last plan, and applied the plan's first input at once. The new MPC had a
delayed form (the plan solved at the previous tick, R123) and a pure one (a cold solve per tick),
neither of which is that controller. At the circuit's 0.2 s period and about 9 m/s, one period of
delay is 1.8 m of lag: the car stalled at about 4 s.

## The maintainer's decisions (2026-10-28)

1. **No cache: the zero-delay MPC is a diagram of pure blocks** (option C, over a solve stored
   inside one block, which R123 removed, and over solving twice per tick, the fallback). R123
   holds as written.
2. **Retune the demos' SLSQP tolerance** where the correct warm start needs it, rather than
   replaying the old warm start's timing slip.
3. **Nudge the pathtracking project's start heading** off the exact axis.

## What the branch does

`ModelPredictiveController(planner, delay=False)` is the textbook MPC, an optimizer and a memory
(`WarmStartedPredictiveController`, a `SampledDiagramSystem`):

```
y ──► [ PlanSolver: (y, z_prev) → z*_k ] ──► [ PlanCommand: z*_k → u_k ] ──► u
                 ▲                       │
                 └──── [ PlanMemory z⁻¹ ] ◄┘
```

- `PlanSolver` (static): `z*_k = pv.solve(P_k, z0=warm)`, `warm` the last plan rebased to
  `t_k` with its first state knot at the measurement, the default guess at `P_k` while there is
  no plan yet; the last plan kept when a solve returns none usable.
- `PlanCommand` (`PlanFollower` on the shared face): `u_k = pv.command(z*_k, t_k)`.
- `PlanMemory` (a `UnitDelay(dim=n_z)` from the default guess): the last plan.
- `mpc % dt` clocks the three blocks; outputs `u` and `z` (the dual rate takes `z` with zero
  delay); `warm_start=False` keeps the pure form.

**One signal pass per period.** The sampled evaluator's dense run computed the loop's signals
twice per period (in `step`, then again in `substep_points` for the plant's held input), so any
block's output ran twice. The period now computes them once and hands them to both; the v0.3
seeded baseline is byte-identical before and after. A tick solves once in the run; a read-back
after the run (the plant's input, the overlays) recomputes the signals and solves again.

Also: the plan readers find the block whose state is the plan (the delayed MPC, or the
zero-delay MPC's memory, through a clocked view or a `>>` that inlines the box), and the
plant-input read-back follows a nested box's output port to its leaf (the dual-rate loops failed
there before). The pathtracking project's measurement port is named `y`, so `@` wires it again.

## The evidence

Progress along the track (or final `x`), and wall time in this container; the old code at
`9be4f023` (before the overnight run):

| Demo | Old code | New default (delayed) | `delay=False`, constants kept | Retuned |
| --- | --- | --- | --- | --- |
| Circuit, 24 s | 1.11 laps, **stalls at 14 s**, 55 s | stalls at 4 s | **2.07 laps, no stall**, 21 s | — |
| Integrator (NumPy), final `x` (goal 2) | 1.9926 | 1.9963 | **1.9996**, 0.5 s | — |
| Car minimal, distance | 19.28 m, 4.1 s | 19.19 m | 19.20 m, 2.7 s | — |
| Dynamic racecar | 21.5 m, 14.2 % outside the lane, 11.8 s | 13.97 m driven | 19.6 m | **`SLSQP_FTOL` 1.0 → 0.3: 24.1 m, 12.8 % outside, 10.0 s** |
| Kinematic racecar | 1.98 laps, briefly outside (0.61 m of 0.6) | — | **U-turns at 3 s** (0.35 lap) | **`ftol` 0.05 → 0.01: 1.77 laps, always inside**, 2.2 s |
| Slalom, final `x` | 87.1 m, 11.4 s | 66.7 m, 32.8 s | 83.0 m, 9.7 s | kept (0.01: 83.8 m, 12.2 s) |
| Dual rate, final `x` | 18.24 | 18.47 | 18.61 | — |
| Pathtracking | 0.69 lap | refused at wiring | stalls (singular heading) | **nudge: 0.69 lap, inside** |

- **The mechanism is the old controller.** A scratch prototype that replays the old warm-start
  guess gives the old runs to the bit (the integrator exactly, the kinematic racecar to
  1.8e-14, the car minimal to 2e-8).
- **The old guess had a timing slip.** `mpc_warm_start_guess` re-timed the shifted plan by `k·Δ`
  while the plan was timed from 0: from tick 2 it read the plan `(k − 1)·Δ` ahead, and after
  `tf/Δ` ticks it was the last plan's next-to-last knot held constant. At the demos' loose
  `ftol`, that poor guess made SLSQP iterate further, which happened to drive faster.
- **The kinematic racecar's 1.98 laps are not the MPC's optimum:** converged solves (`maxiter`
  100 to 200, `ftol` to 1e-3) all give 1.77 laps inside the lane. Its cost has no direction
  term (a cruise speed and the lane), so with `ftol` 0.05 the correct warm start lets it U-turn;
  0.01 keeps it on course.
- **Slalom:** the old run accelerated harder (9.9 against 9.5 m/s at 10 s); a tighter `ftol`
  closes under a third of the gap at more time, so its constant stays.
- **Pathtracking** starts at a heading of exactly 0, where SLSQP stops with "Singular matrix E
  in LSQ subproblem" at every tick; the old run escaped through its first failed iterates. The
  circuit demo already nudged its own start for the same reason.
- **The bicycle and Ipopt** (RD-33): the Ipopt adapter is unchanged since `9be4f023`; neither
  gives cyipopt a Lagrangian Hessian, so that limit is not a regression (`cyipopt` is not
  installable in this container).

## Not done

The notebooks that run an MPC (GRO501 `racecar_toward_mpc`, the showcase, `mpc_spatial_stack`)
still run the delayed default: this branch does not touch them while the hardening lane is open.
