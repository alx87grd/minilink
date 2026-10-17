# Automation by convention: a second opinion, and the rulings (2026-10-13)

**Date:** 2026-10-13
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** An independent review of
[automation-by-convention.md](../plans/automation-by-convention.md) as decided on
2026-10-11, read on `dev` at `e25bb9b`, with the plan's premises probed against the code.
The verdict: the plan is sound and its four ideas (ports drive the wiring, three reserved
names are the roles, no controller classes, the cost scored on the plant by identity) are
the right minimal system. The review found one internal contradiction, three sharp edges
in the operator rules, one naming inconsistency with the plan's own principle, and a
schedule risk. The maintainer ruled on each the same day.
**Applied:** the same day, into the live plan, the workboard and ROADMAP. This file is the
decision record: one entry per ruling, with the question that raised it and where the
ruling now lives. The 2026-10-11 record stays as it was; its top block points here.

---

## Checked against the code

Facts the plan rests on, confirmed by probes and still standing:

- **The catalog exposes `x`.** 45 of the 49 catalog constructors give an `x` output; the
  exceptions are the named-port vehicles `DynamicBicycle`, `DynamicBicycleCar3D`,
  `UdeSRacecarDyn` and `UdeSRacecarDyn3D`, which the plan wires by hand anyway. The "16 of
  50" count in the 2026-10-11 record's §2.7 was stale when written; the record is history
  and keeps it, with this note as the correction.
- **`MechanicalSystem.h` already returns `x = [q; dq]`.** The `q` and `dq` ports are extra
  `StateSliceOutputPort`s, so dropping them (decision 11) is a pure port removal, not a
  change of `y`.
- **The kinematic controllers need no Demux.** `TaskKinematic` reads `y` of a
  `SpeedControlledManipulator`, whose state is `q`, so the dimensions already match.
- **`KinematicBicycle` has a plain `u`.** APP4's LQR path is unaffected by `command_port`;
  only `DynamicBicycle` and the dynamic racecars refuse it unless built with
  `named_ports=False`, and those are hand-wired by decision.
- **No site composes two loops.** No `loop >> loop` or `loop + loop` exists in the library,
  tests or examples.
- **Reading inputs by name:** `get_port_values_from_u` appears in 11 library lines, not the
  8 the record counted; all are plants and become positional in AC-OP as planned.
- **Precedence:** `@` binds tighter than `>>` in Python, so `Step() >> ctl @ plant` is
  `Step() >> (ctl @ plant)` and the step always reaches the loop's one free input `r`.

## Rulings (2026-10-13)

- **R1 `rename_ports` is simultaneous.** The 2026-10-11 rule ("refuse a new name that is
  already a port of the block, so swaps and chains are impossible") refused the plan's own
  example, the chain `Saturation(-1, 1).rename_ports(u="u_cmd", y="u")`. Ruling: the
  renames of one call apply at once; the call is refused only when a new name collides with
  a port not renamed in the same call, or when two new names are equal. Safety comes from
  library equations reading `u` by position (a rename cannot change the math) and from the
  by-name helper raising a clear message for a stale name. Lives in: the plan's §4.1.
- **R2 `@` reads a diagram plant's state.** `lqr @ (Saturation(-1, 1) >> arm)` is the
  plan's own actuator recipe and the D1 case that started the loop review, yet the `>>`
  edge carries only `y`, so an `x`-reading controller was refused until the user wrote
  `.expose("x")`. State feedback is the most common loop shape in the census. Ruling: for a
  controller that reads `x`, `@` uses a diagram plant's own state (the stack of its blocks'
  states, what `expose("x")` would put on the edge) with no extra call; the refusal stays
  for a leaf built without `expose_state`. The same holds for `computer @ plant` once MPC
  reads `x` (the GRO501 racecar's `inner` box). `expose("x")` remains for hand wiring and
  plots. Lives in: the plan's §2.1, §2.4, §4.3 and §4.5.
- **R3 One guard in `>>`.** A source's output is `y` and a reference-layout controller has
  `r` and `y`, so the same-name rule turned `Step() >> P()` into a step on the measurement:
  it enters through `e = r − y` with a minus sign and no message. The replay found one test,
  but it is what a student writes to look at a controller alone. Ruling: before the same-name
  rule, when A has no inputs and B has a free input `r`, a `y → y` match is refused with
  "`Step() >> P()` would drive the measurement `y`; write `Step(port="r") >> P()` for the
  controller alone, or `Step() >> (P() @ plant)` for the loop". It is the only special case
  in `>>`, and §9 says no second one is added. Lives in: the plan's §4.2 and §9.
- **R4 Words for the loop keywords.** `n=` is the state-count keyword of every constructor
  (`super().__init__(n=...)` 41 times in the library and `DynamicSystem(n=...)` in the course
  notebooks), and `d` is the damping method of the mechanical base. The plan's own rule is
  "blocks get words, signals get symbols", and the value of each keyword is a block that
  receives the name `reference`, `disturbance` or `noise`; the sensitivity functions already
  take `plant=`, `controller=` and `filter=`. Ruling:
  `closed_loop(controller, plant, *, reference, disturbance, noise, filter, estimator,
  validate)`. The ports stay the letters `r`, `d`, `n`. The 2026-09-30 keywords
  `closed_loop(w=, v=)` are renamed once to the words, with no alias; AC-2c then changes the
  letter only on the two noisy plants' ports. Lives in: the plan's at-a-glance, §4.2, §4.3,
  §6 and §7; [randomness.md](../plans/randomness.md) "Port names".
- **R5 The `@` dispatch is one rule.** "Connect every controller input named `y` or `x` to
  the plant output of the same name; if the controller has neither, insert `Error` on its
  command entry." It builds the same loop for the 19 library controllers, and it defines the
  case the three-row table left open: a controller that reads both `x` and `y` (an LQI on the
  state plus the output error, a safety filter). The table stays as the worked examples.
  Lives in: the plan's §3 (`block_roles` reports which of `y`, `x` a block reads) and §4.3.
- **R6 `loop >> loop` and `loop + loop` refuse.** Both operands carry `controller` and
  `plant`, and a clash of reserved names raises. No site does it; the plan now says so and
  points at separate runs and `compare()`. Lives in: the plan's §4.
- **R7 P4 starts now, in parallel.** `LuenbergerObserver` is a `DynamicSystem` with ports
  `u`, `y` → `x`; nothing in the block or in `luenberger()` / `kalman()` needs AC-3. The
  notebook wires it by hand (`add_subsystem`, `connect`: requirement 1 of the 2026-10-11
  record) until AC-4's `closed_loop(estimator=)`. P4 is the largest GRO501 gap and v0.2 is
  due this month. Lives in: TODO §2 (order and the P4 row), ROADMAP §3 and §5.2.
- **R8 The rename is one sweep at the end of wave B.** The only stated reason AC-2 precedes
  AC-OP is `feedback`'s entry rule using `command_port`, which is AC-2a. AC-2b (the ids) and
  AC-2c (the letters) block nothing before RN-4 (v0.3). Ruling: build AC-0, AC-2a, AC-OP,
  AC-3, then AC-2b and AC-2c as one mechanical sweep, so `composition.py` is rewritten once
  rather than renamed and then rewritten. Decision 11 (`q` / `dq`), decision 10's
  `rename_ports` and `port=`, the MPC rename and AC-8a are off the GRO501 path and may trail
  P11. AC-3 no longer "unblocks P4". Lives in: the plan's §7, TODO §2, the top block of the
  2026-10-11 record.

## Where the 2026-10-11 record and the plan now differ

The record is read-only history. Where its text says `closed_loop(d=, n=)` or `r=`, read
the words of R4; where it says `expose("x")` is needed on a plant box, read R2; where it
says `rename_ports` refuses chains, read R1; where it orders AC-2 before AC-OP, read R8. The
plan carries the live rules.
