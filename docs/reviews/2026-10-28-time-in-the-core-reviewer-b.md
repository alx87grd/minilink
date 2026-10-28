# Time in the core: what `t` is, for every block in every diagram (reviewer B, the implementation)

**Date:** 2026-10-28. **Audience:** Prof. Alexandre Girard (Maintainer).
**Object:** the converged proposal of the 2026-10-28 session on the meaning of `t` in `(x, u, t; params)`
across the continuous, the tick and the sampled kinds, read against the compile stack
(`core/compile/plan.py`, `evaluators/discrete.py`, `base.py`, `stepping.py`, `diagram.py`),
`core/discrete/*`, `analysis/discretization.py`, `simulation/discrete/computer.py` and
`hybrid_simulator.py`, `planning/evaluator.py`, the MPC block, the sources, and the rulings R145–R148.
**Status:** an agent's review, not a ruling. It edits no other file. Small prototypes under the
session scratchpad (`reviewer-b/proto_*.py`) back the claims marked *(checked)*.

## 0. One screen

- **Verdict: keep the converged proposal; it is the right shape. Two points need a decision before
  it is coded, one of them is where the implementation bites.**
  1. **The home of the clock origin (point 5).** "`t_start` is one leaf of the sampled diagram's
     tree, written through to every clocked block" cannot be built as written on top of the
     earlier ruling that a sampled system's `params` *is* its source's own dict, live and not
     nested (R146): a live dict cannot gain a key without mutating the operand, and a merged copy
     breaks the student idiom `loop.params["plant"]["m"] = 4`. I recommend: a wrapper's tree is
     `{"t_start": 0.0, "source": <the source's live dict>}`; a sampled diagram shows one
     `t_start` beside its blocks' leaves (`{"t_start", "controller", "plant"}`) and writes it
     through, as it writes `x0` through; its evaluator re-forms each clocked block's tree at the
     call. The origin stays a parameter (traced: `jax.grad` and `vmap` over start times work,
     *checked*), which a structural origin would preclude, and the RL lane needs it (a random
     start on a reference trajectory).
  2. **A silent failure in point 3.** A bare time-reading static block in a sampled loop reads
     the tick index as if it were seconds: `TrajectorySource(traj) >> ...` in a 100 Hz loop reads
     the reference at 0, 1, 2 … seconds on ticks 0.01 s apart, with no error. Purity means the
     diagram cannot tell whether an `h` reads `t`. Refuse what the catalog knows: every block
     of `blocks/sources.py` derives from `Source`, so a sampled diagram refuses a bare `Source`
     at wiring, naming `% Δ`. A user's own time-reading static block stays documented behavior.
- **Everything else holds and is cheap:** divisors, hold slots and the block's own index `k // d`
  are a few fields on the plan's operation rows; all divisors 1 reproduces today's step plan
  line for line; the tick is one pure map that traces, differentiates (w.r.t. a gain and w.r.t.
  `t_start`, *checked* against central differences) and batches; the hyperperiod unroll gives
  the same numbers as the `where` form bit for bit (*checked*); a firing predicate on `k` alone
  keeps a `lax.cond` alive under `vmap` over starts and seeds, so a slow traced MPC is not
  solved on every base tick (*checked*: 4× on a toy).
- **Index passing is the robust choice.** The alternative where the evaluator hands clocked
  blocks seconds and a tick law recovers its count from them fails: `floor((t − t_start)/period)`
  is wrong in 21 % of exact-grid cases and 60 % of accumulated-time cases (*checked*); only an
  integer handed as an integer is exact. The converged proposal passes an integer to every block.
- **My own design, derived first (§1), is the converged proposal with four refinements:** the
  tree shape above; the `Source` refusal; the firing as a `cond` or a hyperperiod unroll, not a
  `where`; and one owner of the schedule (the plan), the interpreted `step` of a sampled diagram
  calling it rather than re-implementing the holds.
- **Regrets to head off (§4):** the silent index-as-seconds; two homes of `t_start`; a nested
  sampled diagram implemented with its own clock (there is one clock, the gcd of all); `t0` and
  `t_start` as two words for one origin.

## 1. My own design, written before judging the proposal

### 1.1 The principle, one sentence

**The third argument is the clock of the diagram a block sits in, and seconds inside a tick world
exist only where a clock is declared: in a `SampledSystem`, whose tick face
`step(x, u, k) = update(x, u, t_start + k·sample_period)` is the one conversion in the library.**

Everything below follows from three facts of the code as it stands. The continuous diagram's
evaluator hands `t` (seconds) to every port compute and every `f` (`diagram.py`,
`port_signals`); the step diagram's evaluator hands `k` to every port compute and every `step`,
static blocks included (`discrete.py`, `port_signals`); and the only object that converts between
the two today is `Discretization.step`, which binds `t_k = k·dt` (`analysis/discretization.py`,
with no origin: DESIGN §5's "`t_k = t₀ + k dt`" is not what the code does). The MPC block does the
same conversion for itself (`t_solve = t0 + k·dt_mpc`, `mpc/controller.py`): "a step that knows
the clock and converts inside" is already how the one time-reading step block is written.

### 1.2 The rule per kind

| Kind | Its equations | What `t` is | Who supplies it |
| --- | --- | --- | --- |
| `DynamicSystem` | `f(x, u, t)`, `h(x, u, t)` | seconds, absolute | the integrator: `Simulator` (`t0`), the step model's stages (`stepping.py`), a `SampledSystem`'s `update` |
| static `System` (`n = 0`) | `h(x, u, t)` | **the clock of the diagram it sits in**: seconds in a continuous diagram, the tick index in a step or sampled diagram | the diagram's evaluator, the same third slot every block gets |
| `StepSystem` | `step(x, u, k)`, `h(x, u, k)` | its own count of firings; no clock, no seconds, ever | the step or sampled diagram (`k`, or `k // d` under a divisor) |
| `SampledSystem` | `update(x, u, t)`, `output(x, u, t)` in seconds, the Åström–Wittenmark `x(kh + h) = F(x(kh), u(kh), kh)`; tick faces `step(x, u, k)`, `h(x, u, k)` | seconds inside, `t = t_start + k·sample_period`; the tick outside | the loop hands `k`; the block converts; `t_start` is its parameter leaf, `sample_period` structural |

A static block that reads time (the catalog's `Source`s: `Step`, `Sine`, `WhiteNoise`,
`TrajectorySource`; a gain schedule) is clocked to read seconds in a sampled loop:
`Sine(omega) % Δ`, a `SampledSystem` whose `output` samples the source's `h` at `t_k`. A static
block that reads no time (`Gain`, `Sum`, `Saturation`, the error block) needs nothing and sits in
any diagram as it is: the static catalog is not doubled.

A native sampled law subclasses `SampledSystem` with no source and writes `update` and `output`
in seconds: the delayed MPC (`update(z, y, t)`: bind `t_start = t` and `x_start = y`, shift, solve;
`output`: `policy(z, y, t)`), a discrete Kalman filter with `A(t)`, `C(t)`, MPPI with its shifted
warm start, a diffusion policy's action chunk. `%` on it sets its period.

### 1.3 The rule per diagram

- **`DiagramSystem` (continuous):** every block's compute and every `f` get `t` in seconds; the
  origin is the run's `t0` (`Simulator(t0=)`, `rollout(x0, u_seq, t0, dt)`), as today.
- **`StepDiagramSystem` (tick):** every block gets `k`; the record's `t` is `k` (R147). A `Step`
  source inside steps at move `k = step_time`; documented, and what the code does today.
- **`SampledDiagramSystem`:** one base tick `Δ`, the gcd of its clocked blocks' periods; a block of
  divisor `d` fires on `k mod d = 0`, **receives its own count `k // d`**, and converts inside if
  it is clocked; its outputs between firings are held state slots; a tick law and a time-free
  static block run at the base tick and get `k`; the origin `t_start` is one leaf of the
  diagram's tree, written through to its clocked blocks; the record's `t` is `t_start + k·Δ`.
  Nested: a sampled diagram inside a sampled diagram is one clocked block of period
  `Δ_inner`, hence divisor `Δ_inner / Δ_outer` (refused unless whole), receiving its own base
  index and the outer's `t_start`; there is **one clock**, and inlining at wiring (what `+` and
  `>>` already do) makes it the gcd of every leaf's period.

### 1.4 The plan, concretely

What `StepExecutionPlan` gains (`plan.py`), beside today's `port_ops` and `step_ops`:

```text
PortOperation / StepOperation   + divisor: int (1 for every block but a slow clocked one)
                                + clocked: bool (a SampledSystem: its tree is re-formed at the call)
                                + hold_slice: slice | None (a port op of a slow block: its held output's state slot)
StepExecutionPlan               + base_period: float | None (None on a pure step diagram)
                                + hold_dim: int (the held outputs, appended after the blocks' states)
```

The tick (`StepDiagramEvaluator.step`, today's two sweeps kept):

```text
port sweep, in topological order:
    k_i   = k // d_i
    fire  = k mod d_i == 0                      (a Python bool on NumPy; a traced predicate on JAX)
    y_i   = compute_i(x_i, u_i, k_i, tree_i)    if fire   else  X[hold_i]
    (d_i = 1: y_i = compute_i(x_i, u_i, k, tree_i), today's line)
step sweep:
    x_i'  = step_i(x_i, u_i, k_i, tree_i)       if fire   else  x_i
    X[hold_i]' = y_i                             (the held output is a state: the textbook ZOH)
tree_i = params[id]                                          for a tick law or a static block
       = {"t_start": params["t_start"], "source": params[id]}   for a wrapper (discretize, %)
       = {**params[id], "t_start": params["t_start"]}            for a native sampled law
```

With every divisor 1 and no clocked block, `hold_dim = 0`, `base_period = None`, and the two sweeps
are today's lines exactly: a pure step diagram pays nothing. The conversion lives in one method:

```python
class SampledSystem(StepSystem):
    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        t_start = params["t_start"]
        h = self.sample_period

        # The sampling instant of tick k
        t = t_start + k * h

        x_next = self.update(x, u, t, params)

        return x_next
```

`update` is `step_body(f, x, u, u, t, h, solver, substeps)` on a continuous source (today's
`Discretization.step` with the origin added), the source's `h` sampled on a static source, and
`source.step(x, u, k, params)` on a tick source (whose `update` does not exist: a tick law has no
seconds; the wrapper's tick face delegates the count).

**The firing on the traced face.** The proposal says a `where`; both branches then run, so a slow
block costs its full work on every base tick, and a traced MPC at divisor 5 is solved five times
too often. Two better forms, same numbers: `lax.cond` on `k mod d == 0`, a predicate on the scan
index alone, which `vmap` over starts, seeds and parameters leaves as a branch (*checked*:
`proto_cond3.py`, a real `cond` in the batched jaxpr, 49 ms against 196 ms for the state-dependent
predicate that does become a `select`); or the hyperperiod unroll, one scan over blocks of
`L = lcm(d_i)` base ticks with the firing decided in Python, which is bit-identical to the `where`
form (*checked*: `proto_plan.py`, `|dJ| = 0.0`) and has no branch at all. The sampled-path plan
already allows both (§8.2, "left to the implementation").

**`t_start` through the stack.** `compute_rollout(n_steps, t_start=0.0)` on a sampled diagram or
a bare sampled system sets the leaf of the tree the evaluator receives; `jax.grad` w.r.t. it and
`vmap` over it are the ordinary leaf mechanics (*checked*: `dJ/dt_start` matches a central
difference to 7 digits; a batch of 8 start times in one `vmap`). Under `problem.compile` the
problem's `t_start` leaf (R148) is that leaf: the evaluator passes it to the loop as it splices
`x_start` into `x0` today (`planning/evaluator.py`, `run`), so the time-varying cost and the loop's
clocked blocks read the same `t_k = t_start + k·Δ`, one home.

### 1.5 What I checked (`scratchpad/reviewer-b/`)

- `proto_plan.py`: a three-block sampled plan (a `Sine % 0.05`, a gain `% 0.05`, a time-varying
  plant at `Δ = 0.01`) as a NumPy loop and as one `lax.scan`: states agree to `1.5e-17`; the
  hyperperiod unroll equals the `where` form exactly; `jax.grad` of the cost w.r.t. the gain and
  w.r.t. `t_start` match central differences (`-7.96631312` / `-7.96631312`, `-28.871224` /
  `-28.871224`); `vmap` over `t_start` runs; `k // d` and `k mod d` are plain `int64` arithmetic
  under the x64 policy.
- `proto_index.py`: recovering an integer from seconds, `floor((t − t_start)/period)`, is wrong in
  6721 of 32000 exact-grid cases and 4779 of 8000 accumulated-time cases; `round` happens to hold
  on the exact grid but needs the origin the block does not have. An integer must travel as one.
- `proto_cond3.py`: the `cond` survival under `vmap` above.

## 2. The scenarios

Each row: what `t` is for the blocks, who supplies it, and the refusal if any.

| Scenario | What `t` is, who supplies it | Refusal / note |
| --- | --- | --- |
| Continuous diagram with a `Sine` reference | seconds from the integrator (`Simulator` `t0 + kΔ`, the RK4 stage times `t + h/2`, `t + h`); `Sine.h` reads them | none; today's path |
| Time-varying plant `f(x, u, t)` discretized by RK4 | `SampledSystem.update(x, u, t)` runs `step_body` at `t`, the stages at `t + h/2`, `t + h`; `step(x, u, k)` binds `t = t_start + k·h` | closes today's gap (`t_k = k·dt`, no origin) |
| Chess / grid world, pure `StepDiagramSystem`, with a static `Gain` | `k` to every block; `Gain` ignores it | none |
| … with a `Step` source | `Step.h` reads `t = k`: it steps at move `step_time` | documented (R147's `t = k`); a `Source` is refused only in a *sampled* diagram |
| z-domain filter alone | `compute_rollout`: `k`; the filter never reads it; record `t = k` | none |
| … inside a sampled loop, bare | runs at the base tick `Δ`, gets `k`; never reads it | none |
| … under `filt % 0.01` in a 1 kHz loop | a wrapper over a tick source, `d = 10`, fires on `k mod 10 = 0`, its tick face gets `k // 10`, its outputs held | none; `update` does not exist on it |
| `pid % 0.01 @ plant` | `Δ = 0.01`, every `d = 1`; the PID's `update(z, e, t) = z + Δ·f(z, e, t)` at `t = t_start + kΔ` (its `f` never reads `t`); the plant's `update` integrates at the same `t` with substeps | none; record `t = t_start + kΔ` |
| `(filt % 0.01) >> (ctl % 0.1) @ plant` | `Δ = 0.01`; `ctl` `d = 10`, a hold slot for `u`, fires with `k // 10` and converts to `t_start + (k // 10)·0.1`, equal to `t_start + kΔ` at the firing; `filt` and the plant at `d = 1` | none |
| `(mpc % 0.05) @ plant`, the MPC binding `t_start` | the MPC is a native sampled law of period 0.05; its `update(z, y, t)` forms `P_k = {…, "x_start": y, "t_start": t}` and solves; its `step` binds `t = t_start + k·0.05`; the plant at `Δ = 0.05` with substeps | none; the planner's own `t_start` leaf is dead in the loop (bound each tick), as R148's bound leaves are |
| Dual rate `(mpc % 0.05) >> (follower % 0.01)` | `Δ = 0.01`; the MPC `d = 5`, a hold slot for `z`; the follower `d = 1`, `output(z, y, t)` = `policy(z, y, t_start + kΔ)`; within the tick where the MPC fires the follower reads the new `z` (R125) | none; one slow period of delay, not two |
| A `TrajectorySource` reference in a sampled loop | bare: reads `t = k`, the reference at 0, 1, 2 … s on 0.01 s ticks; clocked (`% Δ`): `output` samples it at `t_start + kΔ` | **refuse a bare `Source` in a sampled diagram at wiring, naming `% Δ`** (§3.2) |
| A gain schedule `K(t)` sampled | `schedule % Δ`: `output` samples `K` at `t_k`; bare: reads `k` | the user's own static block: documented, no refusal possible |
| A discrete Kalman filter with a time-varying model | a native sampled law: `update(x̂, (u, y), t)` reads `A(t)`, `C(t)`; `step` converts; `%` sets its period | none |
| A sampled loop started at `t_start = 100`, `problem.compile` with a time-varying cost | the problem's `t_start` leaf is the loop's leaf; the cost reads `t_k = 100 + kΔ`, the clocked blocks the same; `grad` w.r.t. `t_start` flows through both | none; D15: `dt=` omitted reads `Δ`, given must equal it |
| `discretize(problem, dt)` and `StepPlanningProblem` on a tick law | the step problem's `transition(x, u, k)` is the sampled plant's tick face, `t = t_start + k·dt` inside; a tick-law problem's `t_start` is an integer tick (R148), its cost reads `k` | none |
| The RL environment on `transition` | holds `(x, k)`; `t_start` a leaf of the problem's tree, so `reset` may draw it and `vmap` over it (a random start on a reference) | none |
| Nested: a continuous `DiagramSystem` as the source of `discretize()` | the wrapper's `update` integrates the stacked `f` at absolute `t`; every inner block, a `Sine` included, gets seconds | none |
| Nested: a `SampledDiagramSystem` as a block of a bigger sampled diagram | one clocked block of period `Δ_inner`, divisor `Δ_inner / Δ_outer`, its own base index `k // d` and the outer's `t_start` written through; inlined at wiring where the operators inline | refuse `Δ_inner` not a whole multiple of `Δ_outer`; **one clock**, never two |
| The realtime runner / hardware in the loop | the host ticks the same tick faces with `k` and the loop's `t_start` (the wall-clock origin), reads the device between ticks, integrates the plant with the same `t`; the step diagram is its oracle at the ticks | jitter and asynchrony out of scope (ROADMAP §7); `k` stays the firing count |
| Delay compensation (`t + Δ`) | with no `t` port there is nothing to rewire: the MPC's own binding shifts, `ModelPredictiveController(planner, lookahead=Δ)` binding `t_start = t + Δ` and `x_start` from a predictor | a keyword on the block that knows what it compensates |
| Event-triggered replanning (out of scope) | not precluded: a trigger input and a `where` inside the law's own `update` (solve when triggered, else hold `z`), the clock still periodic; an event clock would be a new kind later | none today |
| `jax.grad` through the sampled loop w.r.t. a gain and w.r.t. `t_start` | both leaves of the tree; the firing predicate carries no gradient; *checked* | none |
| A Monte Carlo (`vmap`) over starts and seeds | `x0`, `t_start`, seeds batched; the firing predicate depends on `k` alone, so a `lax.cond` stays a branch; *checked* | none |
| The `Trajectory` of `compute_rollout` / `compute_trajectory` | rollout: `t = k` on a tick system, `t = t_start + kΔ` on a sampled one; trajectory: `t = t_start + k·Δ/substeps`, bit for bit at the ticks (R147); the blocks and the record agree | `compute_trajectory` refused on a tick system |

## 3. Verdict on the converged proposal

### 3.1 Keep

Points 1, 2, 4 and 6 as written, and point 3's rule. In particular: *time belongs to the loop*
(the evaluator already hands one third slot to every block, and nothing in the code wants a
second channel); *`SampledSystem` is the sampled-data kind with two faces and a native law
subclasses it* (today's MPC is that block under other names: `t0`, `dt_mpc`, `t_solve = t0 +
k·dt_mpc`); *multi-rate in the plan, one pure map on the base tick, all divisors 1 today's plan*
(§1.4 shows it is a handful of fields); *the record's `t` equal to the blocks' `t`* (R147).

Point 1's "a static block in a step diagram reads `t = k`" is what both paths do today
(`StepDiagramSystem.step` and `StepDiagramEvaluator.port_signals` pass `k` to static computes),
and the tests pin it (`tests/unittest/test_step_discrete.py`, the reference path against the
evaluator at each `k`).

### 3.2 Wrong or risky, with the failure cases

**(a) Point 5 is underspecified where it bites: the tree of a clocked block.** R146 says a
`SampledSystem`'s `params` "are the source's own dict, live and not nested". Then `t_start`
cannot be its leaf: adding a key to the source's dict modifies the operand (`plant.params` would
show `t_start` after `discretize(plant, dt)`, against "no operator modifies its operands"), and a
merged copy `{**source.params, "t_start": …}` is not live, so the documented idiom
`loop.params["plant"]["m"] = 4` writes into a temporary and is lost: a silent teaching-surface
bug. Nothing in the proposal says which. The three honest resolutions:

- *(i) nested, with a hoisting view (my recommendation):* the wrapper's tree is
  `{"t_start": 0.0, "source": source.params}` (live for `["source"]["m"]`); the sampled diagram's
  view is `{"t_start": …, "controller": {…}, "plant": {…}}` (one origin, each block's leaves
  under its id, a wrapper's being its source's), its setter writing `t_start` through to every
  clocked block as `x0` is written through (R124), its evaluator re-forming each clocked block's
  tree at the call (§1.4, `tree_i`). About forty lines in `SampledDiagramSystem.params`,
  `params_snapshot` and the two sweeps; infrastructure, not an equation.
- *(ii) nested, no hoist:* the student writes `loop.params["plant"]["source"]["m"]` on a sampled
  loop and `loop.params["plant"]["m"]` on a continuous one: the graduating syntax breaks
  (CONSTITUTION §3). Rejected.
- *(iii) structural:* `discretize(sys, dt, t_start=0.0)`, written through at wiring like `x0`,
  ten lines, R146 verbatim. Rejected because the origin then cannot be traced: no `vmap` over
  start times (the RL lane's random start on a stored reference, which has no phase leaf to
  randomize instead), no `grad` w.r.t. it, and the problem's `t_start` leaf (R148) would have to
  become structural too or silently disagree with the loop under `pe.evaluate({"t_start": …})`.

**(b) Point 3 fails silently for the catalog's sources.** Purity means a diagram cannot tell
whether a static `h` reads its third argument, so a bare time-reading block in a sampled loop
reads the index with no error:

```python
ref  = TrajectorySource.from_trajectory(plan)        # sampled at seconds 0 … 5
loop = ref >> (pid % 0.01) @ plant                    # a 100 Hz loop
loop.compute_rollout(500)                             # ref.h(x, u, t=k): the reference at 0, 1, 2 … seconds,
                                                      # read on ticks 0.01 s apart; wrong by 100×, no message
```

`WhiteNoise(sample_period=0.01)` bare in the same loop computes `sample_index(k, 0.01) = 100·k`:
distinct draws, but not the ones its continuous twin draws, so a `%` and a bare placement give
different noise with no warning. The fix is structural and exact for the catalog: every
time-reading block in `blocks/sources.py` derives from `Source`, so `SampledDiagramSystem` refuses
a bare `Source` at `add_subsystem`, naming `% Δ`, the way a step diagram refuses a continuous
leaf today (`require_step_subsystem`). A user's own time-reading static block remains the
documented rule ("the third argument is the diagram's clock").

**(c) Point 4's `where` costs a slow block its full work every base tick.** A traced MPC at
divisor 5 under a `where` is solved five times per period. Use a `lax.cond` on `k mod d == 0`
(the predicate depends on the scan index alone, so `vmap` over starts, seeds and parameters keeps
the branch; *checked*) or the hyperperiod unroll (bit-identical; *checked*). Same numbers, a
different cost; the plan doc already allows it (sampled-path §8.2). State it so the implementer
does not write the `where`.

**(d) Two implementations of the schedule.** `StepDiagramSystem.step` is an interpreted reference
path beside the compiled plan, and the tests compare them at every `k`. A sampled diagram's
interpreted `step` would re-implement the divisors, the `k // d` and the hold slots: two owners of
one law (RULES 7.8). One owner: the interpreted `step` of a sampled diagram runs the plan's
NumPy sweep, or the sampled diagram has no interpreted path and the test oracle is the
re-clocked loop (R147).

**(e) The numbers move once more than planned.** Today's `Computer.tick` hands every block the
*base* tick `k_tick`, slow ones included; `MPCBroadcastController` reads it as `t = t0 +
k_tick·dt_broadcast`. Under point 4 a slow block gets its own count `k // d`. The hybrid demos
re-pin for the same-tick semantic (R125) already; this is part of the same move, to be listed in
the closeness report so nobody hunts for it.

**(f) `jacobian(wrt="t")` on the seconds face.** Today the Jacobian refuses `wrt="t"` on a step
system ("k is an integer", `jacobian.py`). A `SampledSystem` has a seconds face; offer
`wrt="t"` on `update` / `output`, and `wrt="params"` reaches `t_start` for free.

**(g) D15 can be closed by this design:** `problem.compile(controller)` on a sampled controller
reads `Δ` off the loop; `dt=` given must equal it, else refused naming the loop's base period.

### 3.3 The other options, in the plan's terms

| Option | Divisors, holds, index | `t_start` | The conversion | Traces / grad / vmap / NumPy | Nested | Realtime | Cost and risk |
| --- | --- | --- | --- | --- | --- | --- | --- |
| R148's `t` port (dropped) | as §1.4 | a clock source block feeding a `t` wire; the MPC reads it | none in the blocks; the clock block converts | all fine | the wire must cross boundaries | the host writes the wire | one more standard port and a clock block in every continuous diagram; a wiring convention every time-reading block repeats (QUESTIONS, "the clock port") |
| A unit declaration on `StepSystem` (rejected) | as §1.4 | a leaf or attribute | the evaluator reads the flag and converts | fine | fine | fine | a mode flag in the core; rejected on taste, rightly |
| The evaluator hands seconds to clocked blocks and `k` to tick laws (replaced) | as §1.4, plus a per-op flag "reads seconds" derived from the source's class | an argument of `rollout`, nothing in any tree | in the evaluator, once | fine | **fails:** a nested sampled diagram must recover its base index from seconds (*checked*, §1.5) unless always inlined; a tick law under `%` needs its count and cannot get it from seconds | fine | ~the same lines; the flag is the rejected declaration moved into the compiler |
| The converged proposal | as §1.4 | "one leaf written through", home unspecified (§3.2 a) | `SampledSystem.step`, once | fine (*checked*) | fine: index passing is exact | fine | the tree shape and the `Source` refusal are the open risks |
| This review (§1) | as §1.4 | the wrapper's `{"t_start", "source"}` tree, the diagram's hoisting view | `SampledSystem.step`, once | fine (*checked*) | one clock, inlined where the operators inline | fine | ~40 lines of view and re-forming; the `Source` refusal ~10 lines; `cond` or hyperperiod ~20 lines |

Rough lines for the whole: `plan.py` +60 (the fields, the gcd and divisors, the hold layout),
`discrete.py` +80 (the two sweeps with firing and holds, the re-formed trees, the hyperperiod
scan), `SampledSystem` +120 as the renamed `Discretization` with `update`/`output`, the static and
tick sources, the tree and the origin, `SampledDiagramSystem` +80 (the view, the write-through,
the `Source` refusal, `compute_rollout(t_start=)`), tests. No change to `DiagramSystem`, the
continuous evaluators, `Simulator` or `f` (CONSTITUTION §5.3, the continuous contract untouched).

### 3.4 Same, a refinement, or different?

A refinement. Same principle, same six points; it settles the tree of a clocked block (§3.2 a),
adds the `Source` refusal (b), names the firing's form (c) and the one owner of the schedule (d),
and answers the delay-compensation and D15 details.

## 4. Regret risks, three years out

1. **The silent index-as-seconds.** The one thing a student will hit and not understand: a
   reference block dropped bare into a sampled loop. The `Source` refusal removes it for the
   catalog; the sentence "the third argument is the diagram's clock" in DESIGN §4 covers the
   rest. Without the refusal this is the regret.
2. **Two homes of `t_start`.** The MPC keeps `(z, t_last)` (R148) and the planner's tree holds
   `t_start`; the loop holds `t_start`; a wrapper holds it; the problem holds it. With the
   hoisting view there is one *written* home per object and one *read* home in the loop's tree;
   QUESTIONS' "one owner of the plan's time" (option a, `shift(z, t)` absolute) removes `t_last`
   as well. Decide both together.
3. **A nested sampled diagram built with its own clock.** If the nesting rule is not written,
   the first implementer of `@` on sampled operands will give the inner diagram a scan of its
   own. Write "one clock, the gcd of every leaf's period; a nested box is one clocked block of
   its base period" into DESIGN §4.
4. **`t0` and `t_start`, two words for one origin.** `Simulator(t0=)`, `rollout(x0, u_seq, t0,
   dt)`, `HybridSimulator(t0=)`, `RealtimeSimulator.run(x0, t0)` say `t0`; the problem (R148) and
   this design say `t_start`. One word in the names sweep; I would keep `t_start` for the leaf
   (it stands beside `x_start`) and let the continuous runners' `t0` follow in the sweep, or be
   documented as the same quantity.
5. **The firing written as a `where`.** Not a wrong number, a 5× cost that someone will later
   "optimize" by changing the scan's shape and the numbers with it (RULES 7.7). Write the
   hyperperiod unroll or the `cond` first.
6. **Event-triggered and asynchronous clocks.** Not precluded: a trigger is an input and a
   `where` inside a law's own `update`; an event clock would be a fourth kind of diagram, not a
   change to these three. `sample_period` structural keeps the schedule static, which is what
   lets the hyperperiod be unrolled; a traced period would forbid it. Keep it structural.
7. **`Trajectory.t = k` on a tick system.** A plot axis labelled in seconds showing moves. R147
   chose it knowingly; the plotting lane can label the axis `k` when the system has no period.
   Not a core regret.

## 5. Naming and the contract sentences

**Names.** `SampledSystem` (R146), `sample_period` structural, `t_start` the origin leaf (beside
`x_start`), `update(x, u, t, params)` and `output(x, u, t, params)` the seconds faces
(Åström–Wittenmark's "state update" and "output"; `step` and `h` stay the tick faces every
`StepSystem` has), `lookahead=` the MPC's delay compensation, `Source` the refused bare kind.

**DESIGN §4, `System`, to add after "Math":**

> The third argument of `h` and of every port `compute` is the clock of the diagram the block
> sits in: seconds in a continuous diagram, the tick index in a step diagram and in a sampled
> diagram. A block that reads no time sits in any diagram as it is. A block that reads time
> carries a clock in a sampled loop, `block % Δ`, which samples its `h` at `t_k = t_start +
> k·Δ`; a bare `Source` is refused there at wiring.

**DESIGN §4, `StepSystem`:**

> `step(x, u, k, params)` and `h(x, u, k, params)`: `k` counts the block's own firings. A step
> system has no clock and never sees seconds; a chess game, a grid world and a z-domain filter
> are one kind.

**DESIGN §4, `SampledSystem`:**

> A system on a clock, `x(t + h) = update(x(t), u(t), t)`, `y(t) = output(x(t), u(t), t)`, in
> seconds. Its tick faces `step(x, u, k)` and `h(x, u, k)` bind `t = t_start + k·sample_period`,
> the one place the library turns a tick into seconds. `sample_period` is structural (a sweep,
> never a gradient); `t_start` is the leaf `params["t_start"]`. `discretize(sys, dt)` (RK4) and
> `sys % dt` (Euler) return one over a `source`: a continuous system (`update` integrates its
> `f` over the period, the input held), a static block (`output` samples its `h`), or a step
> system (`step` delegates the count; it has no seconds face). Its tree is `{"t_start",
> "source"}`, the source's dict live. A native sampled law (the MPC, a Kalman filter) subclasses
> it with no source and writes `update` and `output`; `%` sets its period.

**DESIGN §4, `SampledDiagramSystem`:**

> A step diagram with a clocked block. Its base period `Δ` is the gcd of its clocked blocks'
> periods; a period that is not a whole multiple of `Δ` is refused at wiring. A block of
> divisor `d = period / Δ` fires on `k mod d = 0`, receives its own count `k // d`, and converts
> inside if it is clocked; between firings its outputs are held, and the held outputs are state
> slots of the diagram. A tick law or a static block inside runs at `Δ` with the base index.
> Every block fires at `k = 0`; signals propagate within the tick. The tick is one pure map
> `X_{k+1} = F(X_k, U_k, k; P)`; its tree is `{"t_start", <block id>: …}`, one origin written
> through to every clocked block, each block's leaves under its id (a wrapper's, its source's).
> A sampled diagram inside another is one clocked block of its base period: there is one clock.
> `compute_rollout(n_steps, t_start=0.0)` records `t = t_start + k·Δ`; under `problem.compile`
> the origin is the problem's `t_start`.

**DESIGN §5, "The step model", the sentence to correct now:** `discretize`'s `step` "binds
`t_k = t₀ + k dt`" — the code binds `t_k = k·dt`; until the leaf lands, write what the code does.
