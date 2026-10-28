# Session report: the v0.3 run's survey and the zero-delay MPC (2026-10-28)

**Audience:** the agent that merges `dev-v0.3-mpc-nodelay` into `dev-v0.3` (read §0 first), and
Prof. Alexandre Girard (Maintainer).
**Branch:** `dev-v0.3-mpc-nodelay`, cut from `dev-v0.3-hardening` at `9f960138` while the
overnight run was still on step 7. Its commits: `45f50d2a`, `06abc91f`, `1c6af008`, `d4cd6868`,
and this report.
**What this is:** the one record of the session: what was asked, what was found, what the
maintainer decided, what the branch does, the evidence, and what comes next. The design text
also lives where it belongs (DESIGN §4's MPC paragraph, [planner-and-mpc.md](../plans/planner-and-mpc.md)
§6.3); this record does not restate rulings, it records the session's.

## 0. For the merging agent

**State at the time of writing (2026-10-28, ~23:30 UTC).** `dev-v0.3-hardening` is merged into
`dev-v0.3` (`e59bbd46`), and `dev-v0.3` carries one more merge since (`e7cb90bf`, "compile
once"). A dry run of `git merge-tree origin/dev-v0.3 dev-v0.3-mpc-nodelay` gives two conflicts:

- **`minilink/core/compile/evaluators/discrete.py`:** keep both sides. `dev-v0.3` added
  `SampledDiagramEvaluator.plant_input(...)` (the plant's gathered input at a tick); this branch
  changed `substep_points(..., signals)` to take the tick's signal buffer and `rollout_dense`
  to compute the signals once per period (§4.2). Check that `plant_input` and any new caller of
  `substep_points` on `dev-v0.3` pass `signals`.
- **`minilink/planning/trajectory_optimization/mpc/viz.py`:** take `dev-v0.3`'s
  `plant_inputs_of` (it now reads the plant's input through the compiled plan's gather, which
  follows a nested controller by construction). This branch's `leaf_signal_of` then has no
  caller: delete it. Keep this branch's `holds_plan_of` and its use in `block_path_of` (the plan
  readers must find the zero-delay MPC's memory, §4.1).

`docs/dev/DESIGN.md` and `tests/unittest/test_mpc.py` merge without conflict; read both results
once (the MPC paragraph of DESIGN §4, the MPC test classes).

**Then, in order:**
1. "Always before push" (ruff, whole repo).
2. `pytest tests/unittest/test_mpc.py tests/unittest/test_sampled_loop.py
   tests/unittest/test_step_discrete.py tests/unittest/test_textbook_style.py
   tests/unittest/test_v03_core.py`, then the full `pytest`.
3. The evaluator change must stay byte-identical: capture `benchmarks/v03_baseline.py` on
   `dev-v0.3` before the merge and after it, and `cmp` the two (it was byte-identical on this
   branch, §5).
4. "Regression gates, CI flags" and `tests/demo_checks/run_flagship_demos.py` (the MPC demos
   changed).
5. The "compile once" merge may have changed how `planner.compile()` builds the face; the
   zero-delay MPC's blocks share one face (`PlanSolver`, `PlanCommand`, `PlanMemory` all hold
   the same `pv`), so check `test_the_loop_is_the_hand_loop_with_one_solve_per_tick` (exactly
   one solve per tick) and `test_the_zero_delay_form_traces_on_a_native_planner` still pass.
6. Carry the three rows of §6 into [TODO.md](../plans/TODO.md) (§1.2 or §1.5 for the core
   rows, §1.8 for the MPC row), each pointing here; this branch did not edit TODO to avoid a
   conflict with the run.

**Do not:** change the course notebooks that run an MPC (GRO501 `racecar_toward_mpc`, the
showcase, `mpc_spatial_stack`): they run the delayed default, and whether they move to
`delay=False` is the maintainer's teaching call (AGENTS.md, "Course notebooks"). Ask.

## 1. What the session did

1. **A survey of the overnight run** (asked at ~19:45 UTC): `dev-v0.3` had steps 0 to 6 merged
   (the names sweep, the sampled loop, the math objects and randomness, the parametric program
   and the planner's compiled face, the one score, the MPC on the planner's face, the fields),
   step 7 (hardening, budget script, notebooks) in progress; acceptance scripts 1 to 7 and 10
   to 13 green, the budget script reporting the mass parameter family at 3.7× the plain batch
   (bar 2×); remote CI red only on the notebook smoke check, which step 7 fixed. The run's own
   record is [the run log](2026-10-29-v03-run-log.md); the run flagged that the MPC demos drove
   worse under the new one-period delay (its decision RD-52), which started the rest.
2. **The MPC regression** (the maintainer's ask): the delayed MPC stays the default, but the
   demos must get the old MPC back, zero delay and warm-started, with no regression on the
   bicycle demos.
3. **A prototype** in the scratchpad, then the zero-delay form on this branch, first with a
   per-tick solve store, then, on the maintainer's decision, rebuilt from pure blocks (§3, §4).

## 2. Findings

1. **Why the demos regressed.** The old MPC (`warm_start=True`, the demos' form) solved at
   each tick from the measurement, warm-started from its last plan, and applied the plan's
   first input at once. The new API had a delayed form (the plan solved at the previous tick,
   R123) and a pure one (a cold solve per tick), neither of which is that controller. At the
   circuit's 0.2 s period and ~9 m/s, one period of delay is 1.8 m of lag: the car stalled at
   ~4 s.
2. **The old warm start had a timing slip.** `mpc_warm_start_guess` re-timed the shifted plan
   by `k·Δ` while the plan was timed from 0: from tick 2 it read the plan `(k − 1)·Δ` ahead, and
   after `tf/Δ` ticks it was the last plan's next-to-last knot held constant. At the demos'
   loose SLSQP `ftol`, that poor guess made SLSQP iterate further, which happened to drive
   faster on some demos.
3. **The mechanism is equivalent.** A prototype of the zero-delay mode that replays the old
   guess reproduces the old runs to the bit: the integrator exactly, the kinematic racecar to
   1.8e-14, the car minimal to 2e-8; the others agree to ~1e-9 until SLSQP's loose tolerance
   makes them diverge chaotically, with the same distance driven.
4. **Some old numbers were not what they seemed.** The old circuit run stalled at 14 s in its
   second lap. The pathtracking project starts at a heading of exactly 0, where SLSQP stops with
   "Singular matrix E in LSQ subproblem" at every tick; the old run escaped through its first
   failed iterates (the circuit demo already nudged its own start for the same reason). The
   kinematic racecar's cost has no direction term, so with the correct warm start and `ftol`
   0.05 it U-turns; its old 1.98 laps came from unconverged plans (converged solves all give
   1.77 laps, inside the lane).
5. **A zero-delay output needs the tick's solve twice** (the output and the state update) **and
   again in every read-back** (the plant's input, the overlays). A block that stores its solve
   is the hidden cache R123 removed.
6. **The sampled evaluator computed every signal twice per period** in the dense run
   (`compute_trajectory`): once in `step`, again in `substep_points` for the plant's held
   input. Any block's output ran twice; it is fixed here (§4.2).
7. **Composition gaps, found while building the zero-delay MPC as a sub-diagram:**
   - **`box % dt` gives the blocks inside the wrong time.** A step diagram put on a clock
     becomes a `SampledSystem` wrapping a tick law, so a static block inside reads the tick
     count (measured: `0, 1, 2, 3` where seconds are `0, 0.1, 0.2, 0.3`). The zero-delay MPC has
     its own `%` that clocks each block; the general fix is a core row (§6).
   - **Read-backs recompute every signal.** `compute_internal_signals_dict` and the read-backs
     built on it fill the whole signal buffer, so a block whose output is expensive (a solve)
     is recomputed for every sample read back.
   - **The tuning path depends on the form:** the delayed MPC's knobs are `controller.cost.R`,
     the composite's `controller.solve.cost.R`.
   - **`>>` inlines a box's blocks**, so block paths change (`controller/memory`); tools find
     the plan by role, never by assuming the MPC is one leaf.
8. **Bicycle and Ipopt** (the run's RD-33): the Ipopt adapter is unchanged since before the
   run; neither version gives cyipopt a Lagrangian Hessian, so its non-convergence on the
   bicycle's compiled program is not a regression. Not run here: `cyipopt` does not install in
   this container.

## 3. The maintainer's decisions (2026-10-28)

1. The delayed MPC stays the default; the demos run a zero-delay, warm-started mode equivalent
   to the old MPC.
2. **No cache: the zero-delay MPC is a small diagram of pure blocks** (a solver, a command and a
   memory), over a solve stored in one block (R123 holds as written) and over solving twice per
   tick (the fallback had the diagram been too messy).
3. **Retune the demos' SLSQP `ftol`** where the correct warm start needs it, rather than
   replaying the old guess's timing slip.
4. **Nudge the pathtracking project's start heading** off the exact axis.
5. **Composition first:** clocking a box (`box % dt`) is a core problem to fix before anything
   builds on it.
6. **Two simulation tools:** a way to simulate while recording the signals, and a way to
   reconstruct a limited set of internal signals (not the whole buffer); with both, the
   read-back cost of an expensive block goes away.
7. **After 5 and 6, revisit** making the three-block architecture the standard for every MPC
   form, the delayed one included. The tuning path does not matter, as long as there is one
   standard (`controller.cost.R` or `controller.solve.cost.R`).

## 4. What the branch does

### 4.1 The zero-delay MPC

`ModelPredictiveController(planner, delay=False)` returns `WarmStartedPredictiveController`, a
`SampledDiagramSystem` of three pure blocks, the textbook optimizer and memory:

```
y ──► [ PlanSolver: (y, z_prev) → z*_k ] ──► [ PlanCommand: z*_k → u_k ] ──► u
                 ▲                       │
                 └──── [ PlanMemory z⁻¹ ] ◄┘
```

- `PlanSolver` (static): `z*_k = pv.solve(P_k, z0=warm)`, `warm` the last plan rebased to
  `t_k` with its first state knot at the measurement; the default guess at `P_k` (from the
  measured start) while there is no plan yet; the last plan kept when a solve returns none
  usable.
- `PlanCommand` (`PlanFollower` on the shared face, `declare_follower`): `u_k = pv.command(z*_k,
  t_k)`.
- `PlanMemory` (a `UnitDelay(dim=n_z)` from the default guess): the last plan.
- `mpc % dt` clocks the three blocks (its own `__mod__`, finding 7); outputs `u` and `z` (the
  dual rate takes `z` with zero delay); `warm_start=False` keeps the pure form;
  `UnitDelay(dim=1)` is new.
- The plan readers (`mpc_plans_from_rollout`, the overlays) find the block whose state is the
  plan (`holds_plan_of`: the delayed MPC, or the zero-delay MPC's memory, through a clocked
  view or a `>>` that inlined the box).

The first version on this branch (`45f50d2a`) stored each tick's solve in the block; `d4cd6868`
replaced it with the diagram above. Nothing of the store remains.

### 4.2 One signal pass per period in the dense run

`SampledDiagramEvaluator.rollout_dense`'s period computes the signals once and hands them to the
state update and to `substep_points`; `rollout_dense` checks `params` once. A tick solves once in
the run. A read-back after the run still recomputes the signals, so it solves again (the second
tool of decision 6 removes that).

### 4.3 The demos

Every MPC demo and project that ran the old warm-started MPC runs `delay=False`, constants kept,
except the maintainer's retunes: the dynamic racecar's `SLSQP_FTOL` 1.0 → 0.3, the kinematic
racecar's `ftol` 0.05 → 0.01. The dual-rate project drops the `lookahead` that compensated the
delay. The pathtracking project's measurement port is named `y` (its plant exposed `y_mpc`, so
`@` refused the loop on `dev-v0.3-hardening` already) and it starts at a heading of `1e-4`.

### 4.4 Docs and tests

DESIGN §4's MPC paragraph and [planner-and-mpc.md](../plans/planner-and-mpc.md) §6.3 describe the
zero-delay form. Tests (`test_mpc.py`): the three blocks and their clock; the warm guess and the
first guess; the solve block against a hand solve; the loop against a hand-written loop, with
exactly one solve per tick; a failed solve keeping the last plan; the dual rate; the form traced
on a native planner (one scan, a finite gradient). `test_step_discrete.py`: a vector unit delay.
`test_sampled_loop.py`'s refusal test builds the cold form with `warm_start=False`.

## 5. The evidence

Progress along the track (or the final `x`), the time outside the lane where it matters, and
the wall time in this container; the old code at `9be4f023` (before the overnight run):

| Demo | Old code | New default (delayed) | `delay=False`, constants kept | Retuned or nudged |
| --- | --- | --- | --- | --- |
| Circuit, 24 s | 1.11 laps, **stalls at 14 s**, 55 s | stalls at 4 s | **2.07 laps, no stall**, 21 s | — |
| Integrator (NumPy), final `x` (goal 2) | 1.9926 | 1.9963 | **1.9996**, 0.5 s | — |
| Car minimal, distance | 19.28 m, 4.1 s | 19.19 m | 19.20 m, 2.7 s | — |
| Dynamic racecar | 21.5 m, 14.2 % outside, 11.8 s | 14.0 m driven | 19.6 m | **`SLSQP_FTOL` 0.3: 24.1 m, 12.8 % outside, 10.0 s** |
| Kinematic racecar | 1.98 laps, briefly outside (0.61 m of 0.6) | — | **U-turns at 3 s** (0.35 lap) | **`ftol` 0.01: 1.77 laps, always inside**, 2.2 s |
| Slalom, final `x` | 87.1 m, 11.4 s | 66.7 m, 32.8 s | 83.0 m, 9.7 s | kept (0.01: 83.8 m, 12.2 s) |
| Dual rate, final `x` | 18.24 | 18.47 | 18.61 | — |
| Pathtracking | 154.6 m, inside | refused at wiring | stalls (singular heading) | **heading 1e-4: 153.7 m, inside** |

- The composite and the stored-solve version gave the same numbers on every demo; the composite
  runs at the same speed.
- The slalom's old run accelerated harder (9.9 against 9.5 m/s at 10 s); a tighter `ftol` closes
  under a third of the gap at more time, so its constant stays.
- **Checks on `d4cd6868`:** ruff (whole repo); the full `pytest` (no failure; skips: `cyipopt`,
  `gymnasium`, Graphviz); "Regression gates, CI flags" passed; flagship demos 14 passed, 4
  skipped; `benchmarks/v03_baseline.py` byte-identical before and after the evaluator change
  (in this container, whose capture differs from the committed file only by the gymnasium
  entries); every MPC demo's `mpc_animation_overlays(...)` call builds.
- **The textbook check:** the new equation code (`PlanSolver.compute_z`, `warm_guess`,
  `pinned_shift`, `UnitDelay`, the dense run's period) was read beside
  `planning/dynamic_programming/dp.py`; nested solves in equation lines were split into named
  assignments and an `np` under `xp` fixed (`test_textbook_style` green).

## 6. Recommendation and next steps

**The three-block pattern is the right MPC architecture**, for every form: the forms become
wirings of the same blocks. Zero delay: the command reads the solver. Delayed: the command reads
the memory (the computation delay a visible `z⁻¹` between optimizer and actuator). Pure: no
memory (the solver's unwired `z_prev` reads its nominal value, the default guess). Dual rate: the
command on a faster clock (`PlanFollower` is that block). Lookahead: a predictor block on the
plan's start. **But not yet:** today the delayed form is one pure block that solves in its state
update, so its read-backs are free; as a composite its solve becomes a signal, and every
read-back would re-solve the run until the tools of decision 6 exist.

The rows, in order (the merging agent files them in TODO, §0):

1. **Clocking a box** (core, decision 5): `box % dt` clocks each of the box's blocks and keeps
   their faces (a block in seconds reads seconds), instead of wrapping the box as a tick law.
   Done when a step box holding a static block reads `t_k` in seconds under `% dt`, the
   zero-delay MPC's own `__mod__` is deleted for it, and the baselines stay byte-identical.
2. **Simulating while recording the signals, and reconstructing a few** (core, decision 6): a
   run that records the signals it computes (the plant's input, a named port) in the same scan,
   and a read-back that computes only the ancestors of the requested ports. Done when a
   zero-delay MPC run with its overlays solves exactly once per tick, read-backs included, and
   the delayed form's read-backs stay free.
3. **One MPC architecture** (MPC, decision 7, after 1 and 2): the delayed and pure forms as
   wirings of `PlanSolver`, `PlanCommand`, `PlanMemory` (and a predictor block for
   `lookahead`); one tuning path for every form; `DelayedPredictiveController` and
   `PurePredictiveController` folded into the factory. Done when the delayed form's numbers are
   byte-identical (the v0.3 baseline, acceptance script 5, the budget script) and the plan
   readers find every form by role. It amends R148's wording ("the MPC one block in two
   forms"): record the ruling when it lands.

## 7. Not done, or not verified

- Ipopt (`cyipopt` does not install here): the compiled Ipopt tests were skipped.
- The course notebooks that run an MPC still run the delayed default (§0, "Do not").
- The slalom stays ~5 % behind the old run (§5).
- The prototype scripts and the comparison harness live in the session's scratchpad, not in the
  repo; the numbers above are their output.
