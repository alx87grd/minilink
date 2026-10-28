# What `t` means in the core, across the three kinds of system (reviewer A: the textbook and the user)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** an independent review of the converged proposal of 2026-10-28 on the meaning of the
third argument of `(x, u, t; params)` in continuous, step and sampled systems and their diagrams
(the brief lists the files read: AGENTS.md, CONSTITUTION, DESIGN §4 and §5, sampled-path.md,
core-architecture.md §2 and §5, planner-and-mpc.md §6, the rulings R145–R148, and the code of
`System`, `StepSystem`, `StepDiagramSystem`, `Discretization`, the discrete evaluators, the plan
builder, `Computer` and the source blocks).
**Angle:** sampled-data control theory (Åström & Wittenmark, Franklin–Powell–Workman), discrete-time
systems, and what a student of a classical-control or optimal-control course writes and reads.
**Status:** a review, not a ruling. Nothing but this file was edited.

## Summary (one screen)

**Verdict: keep the converged proposal's skeleton; amend one rule and fix one piece of plumbing.**
The skeleton is right and textbook-faithful: time belongs to the loop, a tick law never sees
seconds, `SampledSystem` is the sampled-data kind with one equation in two faces, multi-rate is a
pure map on the base tick with held outputs as state, the record of a run says what the blocks
read. Four of its six points I would sign as written (2, 4, 5 in intent, 6).

**The one wrong rule (points 1 and 3): a bare static block in a sampled loop reads the index.**
In a sampled loop the record says `t = k·Δ` seconds (R147, point 6 of the proposal) while a bare
`Step`, `Sine`, `TrajectorySource` or gain schedule reads `t = k` ticks: the proposal contradicts
its own point 6 ("blocks and the record agree"). For a student this is a silent wrong signal in the
most ordinary line of a digital-control course:

```python
loop = closed_loop(pid % 0.01, plant, reference=Step(step_time=1.0))   # steps at 10 ms, not 1 s
```

No refusal can catch it (a block does not declare that it reads `t`, and the maintainer rightly
rejected such declarations). The fix costs one line of the plan builder and removes a rule the
user must remember: **a static block reads the clock of the diagram it sits in — seconds in a
continuous diagram and in a sampled one, the index only where no clock exists (a pure step
diagram).** `%` on a static block then means one thing, as everywhere: its period (a slower
reference generator, held); at the base period `Sine()` and `Sine() % Δ` are the same block, the
harmless identity R146 already grants a tick law. This is Simulink's own behaviour under a
fixed-step discrete solver (a source with "continuous" sample time is evaluated at the base
rate, in seconds), which is the mental model students arrive with.

**The plumbing (point 5): `t_start` cannot be "written through to every clocked block like
`x0`".** A `SampledSystem` is a view whose `params` are its source's own, so it has nowhere to
hold a traced leaf; a structural float written through at wiring precludes `jax.grad` and `vmap`
over `t_start` and `problem.compile` binding the problem's `t_start` at run time, both on the
brief's scenario list. The clean form: `t_start` is one leaf of the sampled diagram's tree
(`loop.params["t_start"]`, 0 by default; `compute_rollout(t_start=)` writes it; `problem.compile`
binds the problem's), and **the compiled plan is the clock**: it hands every block whose
equations are in seconds (a static block, a `SampledSystem`) `t = t_start + k·Δ`, and every tick
law its own index `k // d`. `SampledSystem.step(x, u, k) = update(x, u, k·T)` stays as the
origin-0 adapter for standalone use (value iteration, the RL environment, `jacobian`); a native
sampled law overrides `update` and `output` only. One formula, one home, every consumer.

**A simplification that falls out:** on a firing tick `(k // d)·(d·Δ) = k·Δ`, so a block in
seconds never needs "its own index": only a tick law under `%` reads `k // d`. The proposal's two
index conventions collapse to one.

**Regret risks, in order:** (1) the silent-units pitfall above, which would be fixed in three years
by exactly this amendment, moving every demo's numbers then instead of now; (2) a structural
`t_start` closing the gradient and Monte Carlo over the start time; (3) `where`-firing of a slow
MPC solving `d` times too often on the traced face (semantics fine; the hyperperiod unroll is the
implementation's business, keep it out of the contract); (4) `SampledSystem` over a static source
has `n = 0`, which `StepSystem` refuses today (`require_step_state`): relax it for that case.

The contract sentences for DESIGN §4 are in §5 below; the scenario table in §2.

## 1. My own design, derived before judging the proposal

### The principle, in one sentence

**The third argument of every equation is the time of the diagram the block sits in, in that
diagram's unit; a tick law reads its own index and never seconds; a sampled law is one equation
with a seconds face and a tick face; and in a sampled diagram the compiled plan is the one clock.**

### Why this is the textbook's rule

Sampled-data control theory (Åström & Wittenmark ch. 1–3; Franklin, Powell & Workman ch. 4) writes
the sampled plant as `x(kh + h) = Φ x(kh) + Γ u(kh)` and a reference generated in the computer as
`r(kh)`: a sequence indexed by `k`, whose *time* is `kh`, the period `h` belonging to the sampler,
never to the block's own equation. A z-domain design (`H(z)`, a digital filter) is written on `k`
alone and has no `h` in it. A chess game has only `k`. So there are three honest readings of the
slot, decided by *where the block sits*, not by what the block is:

- in a flow (a continuous diagram), `t` is seconds, and every block, static ones included, reads
  seconds at the integrator's stage times;
- in a pure tick world (a step diagram with no clock), there are no seconds anywhere, and `t` is
  the index `k`; a `Step(step_time=5)` there steps at move 5, the only meaning it can have;
- in a sampled-data loop (a step diagram with a clock `Δ`), the computer's sequences have a time
  `t_k = t_start + k·Δ`; a block written in seconds (a reference `r(t)`, a schedule `K(t)`, a
  plant's flow `f(x, u, t)`) is read at `t_k`, which is the sampler in the textbook's figure; a
  block written on the index (a digital filter) reads its index.

This is also the behaviour the dominant tool trained students on: Simulink with a fixed-step
discrete solver evaluates a block of "continuous" sample time (a `Sine Wave`, a `Clock`-fed
function) at the fundamental sample time, in seconds, and lets a `Gain` inherit the rate; a
discrete filter sees only its sample hits. The student never writes a sample time on a sine source
to make it read seconds inside a digital loop.

### The rule per kind

| Kind | Its equation reads | Where it gets it | Never |
| --- | --- | --- | --- |
| `DynamicSystem` (flow) | `f(x, u, t)`, `h(x, u, t)`: seconds | the `Simulator`'s grid and stage times; `discretize` at `t_k + c_i·Δ/substeps` | an index |
| static `System` (`n = 0`) | `h(x, u, t)`: **the time of the diagram it sits in** — seconds under a clock, the index under none | the diagram's evaluator or plan | a clock of its own; a mode |
| `StepSystem` (tick law) | `step(x, u, k)`, `h(x, u, k)`: its own index | the scan; in a sampled diagram `k // d` | seconds |
| `SampledSystem` (sampled law) | one equation, two faces: `update(x, u, t)`, `output(x, u, t)` in seconds; `step(x, u, k) = update(x, u, k·T)`, `h` likewise, origin 0 | a diagram's plan calls the seconds face at `t_start + k·Δ`; a standalone consumer with an origin (a problem's `transition`) calls the seconds face; the step tools call the tick face | a native sampled law overriding `step` |

`update` / `output` are Simulink's own verbs for a discrete block's state update and output at a
sample hit (`mdlUpdate`, `mdlOutputs`), so the pair reads as a known thing, not a minilink
invention; they are one equation with `step` / `h`, never a second one.

### The rule per diagram

| Diagram | What it passes to its blocks | The clock's origin | The record (`compute_rollout`) |
| --- | --- | --- | --- |
| `DiagramSystem` (continuous) | seconds, to every block | the simulator's `t0` | `t` seconds |
| `StepDiagramSystem` (pure tick) | `k`, to every block; a static block reads `t = k` | `k = 0`, or a step problem's integer `t_start` as an offset | `t = k` |
| `SampledDiagramSystem` (a clock `Δ`) | **the plan converts, per face:** `t = t_start + k·Δ` to every block in seconds (static blocks, `SampledSystem`s, the discretized plant); `k // d` to every tick law; a block of divisor `d` fires on `k mod d = 0`, its outputs held as state slots between firings; everyone fires at `k = 0`; signals propagate within the tick (R125) | one leaf, `loop.params["t_start"]`, 0 by default; written by `compute_rollout(t_start=)` or bound by `problem.compile` to the problem's leaf | `t = t_start + k·Δ`; the dense path `t_start + k·Δ/substeps` (R147) |

The sampled diagram stays one pure map on the base tick, `X_{k+1} = F(X_k, U_k, k; P)` with
`t_start ∈ P`: `jit`, `grad`, `vmap`, `jacobian`, `linearize` over a hyperperiod and
`problem.compile` see nothing but a step diagram with one more leaf. All divisors 1 is today's
step plan plus one addition in the third slot.

### What `%` means, everywhere

`%` sets a block's period and nothing else: on a continuous law its Euler difference equation at
that period (R145); on a static block the sampler `r_k = r(t_k)` at that period, which at the
base period is the same block bare; on a tick law a divisor (a tag, no seconds face: `update`
refuses, naming the tick); on a native sampled law its period. `discretize(plant, Δ)` is the same
object with RK4. One verb, one meaning per kind, and no case where `%` changes *units*.

### What never happens

Seconds never reach a tick law. A wall clock never reaches a block: the host runner alone reads
it, and hands the blocks the nominal tick time `t_start + k·Δ` and index, exactly as a real
computer's control task reads its tick count (jitter is the runner's affair and out of scope). A
block never declares a unit, a mode or a clock of its own; a block's `params` never carry a `dt`.

## 2. The scenario table

`Δ` is the base period of the sampled diagram; "plan" is the compiled plan of a sampled diagram
(the interpreted reference `step` mirrors it); "seconds face" is `update` / `output` on a
`SampledSystem` or `h` on a static block.

| Scenario | What `t` is, per block | Who supplies it | Refusal / note |
| --- | --- | --- | --- |
| Continuous diagram with a `Sine` reference | seconds for every block; `r = sin(ω t)` | the `Simulator`'s grid and stage times | unchanged |
| Time-varying plant `f(x, u, t)` discretized by RK4 | inside `update(x, u, t)`: the stages at `t + c_i·Δ/substeps` (`step_body` does this today); the tick face `step(x, u, k)` at `t = k·Δ` | the plan (`t_start + k·Δ`) in a loop; the problem's `transition` with its `t_start`; origin 0 standalone | the source's `params` may not carry `dt` (kept) |
| Chess / grid world with a static `Gain` and a `Step` source | `k` for every block; the `Gain` ignores it; `Step(step_time=5)` steps at move 5 | the step evaluator's scan | no seconds exist, so no unit can be wrong |
| z-domain filter alone | its index `k` | `compute_rollout` | `compute_trajectory` refused (R147) |
| z-domain filter bare in a sampled loop | its index = the base `k`; it never sees `Δ` | the plan | — |
| z-domain filter under `% 0.02` in a `0.01` loop | its own index `k // 2`, fires on even `k`, output held | the plan | `update` on it refuses: "a tick law has no clock face" |
| `pid % 0.01 @ plant` | PID: `z_{k+1} = z_k + 0.01·f(z_k, e_k, t_k)`, `u_k = h(z_k, e_k, t_k)`, `t_k = t_start + 0.01 k` (a PID's `f` never reads `t`); plant: `discretize(plant, 0.01, substeps=n)`, its `f` at the stage times | the plan | reads as the hand-written Euler emulation (FPW ch. 6); the derivative is the filtered one Euler'd, not a backward difference — a `siso.py` matter, not a `t` one |
| `(filt % 0.01) >> (ctl % 0.1) @ plant` | `Δ = 0.01`; `filt` `d = 1`; `ctl` `d = 10`, fires on `k mod 10 = 0`, `u` held ten ticks in a state slot; each in seconds reads `t_start + 0.01 k` | the plan | FPW's multirate figure, as written |
| `(mpc % 0.05) @ plant`, the MPC binding `t_start` | `Δ = 0.05`; `update(z, y, t)`: bind the planner's `t_start := t`, `shift(z, t − t_last)`, solve; `output`: `pv.policy(z, y, t)`; `t = t_start + 0.05 k` | the plan, from the loop's `t_start` leaf (the problem's under `problem.compile`) | the MPC never knows `Δ` (R148 kept) |
| Dual rate `(mpc % 0.05) >> (follower % 0.01)` | `Δ = 0.01`; MPC `d = 5`, fires on `k mod 5 = 0` at `t = t_start + 0.01 k` (which equals `(k // 5)·0.05` there), `z` held; follower `d = 1`, `pv.policy(z_held, y, t)` every tick | the plan | one slow period of delay, as R148 §6.4 states |
| `TrajectorySource` reference in a sampled loop | seconds: the sample at `t_k` (`"previous"` is the ZOH) | the plan | **under the proposal, bare, it reads `t = k` and collapses to its last sample after `t_f` ticks** |
| Gain schedule `K(t)` sampled | `(K_sched % 0.01) @ plant` or bare inside a sampled loop: `K(t_k)` | the plan | under the proposal bare inside a loop it reads `K(k)`: `K_after` from tick `t_f` on |
| Discrete Kalman filter with a time-varying model | a native sampled law: `update(x̂, [u; y], t)` with `A_d(t)`, `B_d(t)` from the model at `t_k` (`solver="exact"` gives them); or, given as sequences `A_k`, a pure tick law on `k` | the plan | both honest; the seconds face is what makes the first one possible |
| Sampled loop started at `t_start = 100` (`problem.compile`, time-varying cost) | every block in seconds reads `100 + k·Δ`; the cost's `g` at the same `t_k` in `transition`; tick laws read `k` from 0 | the problem's `t_start` leaf, bound to the loop's | under the proposal a bare reference reads `k` while the cost reads `100 + k·Δ`: the loop and its score disagree on time |
| `discretize(problem, dt)` and `StepPlanningProblem` on a tick law | sampled: `transition` calls `plant.update(x, u, t_start + k·dt)`, `N = tf/dt`; native: `c(x, u, k)`, `t_start` an integer offset of `k` | the problem's step face | — |
| The RL environment on `transition` | the environment's time is `k`; seconds exist only inside the sampled `transition` | the compiled problem | today's `env.step(x, t, u, key)` carries seconds; it carries `k` |
| Nested: a continuous `DiagramSystem` as the source of `discretize()` | inside, seconds at the stage times for every block (a `Sine` disturbance there is integrated, not held) | `update` | the right home for a within-period disturbance |
| A `SampledDiagramSystem` as a block of a bigger sampled diagram (two base clocks?) | the inner diagram is a clocked block of period `Δ_in`; the outer `Δ = gcd(…, Δ_in)`, `d = Δ_in/Δ`; it fires on `k mod d = 0` at `t = t_start + k·Δ` and its own plan hands its blocks `t_start + k_in·Δ_in`, the same seconds; one origin: the inner `t_start` leaf is the outer's | the two plans, or one after `flatten` | equal numbers flattened or nested, one test pins it; no refusal |
| Realtime runner / hardware in the loop | blocks read the nominal `t_start + k·Δ` and `k`; the wall clock paces the runner and never enters a block; the plant block is the device | the host clock runner | out of scope: jitter, a measured-time option |
| Delay compensation (`t + Δ`) | a knob of the native sampled law, which knows its own period: it binds `t_start := t + T` and `x_start` to a predictor's output | the block's `update` | not precluded; no `t` port needed (R148 §6.4's wiring form goes with the port) |
| Event-triggered replanning | a select inside the sampled law's `update` on `(x, u, t)`: re-solve or keep; the firing schedule stays periodic | the block | out of scope today, not precluded by any option |
| `jax.grad` through the sampled loop w.r.t. a gain and w.r.t. `t_start` | the gain through `params["controller"]`; `t_start` through the leaf into every seconds face (`t_start + k·Δ`); a tick law contributes zero | the plan's conversion | **a structural `t_start` written through at wiring (the proposal's "like `x0`") closes this** |
| Monte Carlo (`vmap`) over starts and seeds on the loop | `x_start`, seeds and, if wanted, `t_start` are leaves; one `vmap` | the tree | — |
| The `Trajectory` of `compute_rollout` and `compute_trajectory` | sampled: `t = t_start + k·Δ` (ticks) and `t_start + k·Δ/substeps` (dense); pure: `t = k`; continuous: seconds | R147 | blocks and the record agree for every block, bare ones included |

## 3. Verdict on the converged proposal

### What I would keep as written

- **Point 2, `SampledSystem` as the sampled-data kind with two faces, native laws subclassing
  it.** This is Åström–Wittenmark's object and the Simulink `update` / `output` pair; the delayed
  MPC, the time-varying Kalman filter, MPPI's shifted warm start and a diffusion policy's action
  chunk all need exactly a seconds face with memory. "A native sampled law subclasses it" is the
  right revision of R146's "nobody subclasses it".
- **Point 4, multi-rate in the plan on one base tick, held outputs as state slots.** It is the
  textbook's sampled-data model of a hold (a memory), and the loop stays one pure map.
- **Point 6, the record.** Right, and the test the rest must pass.
- **The rejections.** A unit declaration on a block is a mode switch in the core (R118's own
  rule); a wrapper feeding seconds into `step(x, u, k)` breaks the tick contract; three static
  kinds break "One signature" and double or triple the catalog; the `t` port says time twice
  (the slot and an input) and shows a `t` input to every port tool. All four deserved to go.

### What is wrong: points 1 and 3 together (a bare static block in a sampled loop reads the index)

The proposal's own words contradict each other. Point 1: "a block reads the time of the diagram it
sits in". Point 6: "blocks and the record agree on what `t` is". In a sampled diagram the record
is `t = k·Δ`; a bare static block reads `t = k`. They agree only for clocked blocks.

Concrete failure cases, each silent (no refusal is possible, since a block does not say whether
it reads `t`):

```python
# 1. The digital-control lab's first closed loop: the reference steps at tick 1 = 10 ms.
loop = closed_loop(pid % 0.01, plant, reference=Step(step_time=1.0))

# 2. A planned trajectory replayed into a sampled follower: interp(t=k, t_knots) holds the last
#    knot from tick t_f on; the plan is "done" after t_f ticks, not t_f seconds.
loop = TrajectorySource.from_trajectory(plan, "u", interpolation="previous") >> (follower % 0.01)

# 3. A finite-horizon LQR schedule inside a sampled box (not the %-ed block itself):
#    K(t) is read at t = k, so K_after from tick t_f on.
box = K_sched >> (Saturation() % 0.01)

# 4. A time-varying cost scored on the same loop: the cost reads t_start + k·Δ (R148),
#    the bare reference reads k; the run and its score live on two clocks.
pe = problem.compile(loop)
```

And the "two ways to say one thing" test the brief asked for fails in the worst way: in one loop,
`Step(step_time=1.0) % 0.01` and `Step(step_time=100)` are the same signal spelled in two units,
and `Step(step_time=1.0)` bare is a third spelling that means something else. Under my rule, the
only identity is `Sine()` ≡ `Sine() % Δ` at the base period: the same block, the same numbers,
the identity R146 already grants a tick law ("a pure step block inside runs at `Δ`").

The argument for the proposal's rule was that the tick world stays pure and the only converter is
`SampledSystem.step`. Both survive the amendment: a tick law still never sees seconds, and the
conversion `t = t_start + k·Δ` lives in one function the plan calls for every block in seconds,
which the proposal needs anyway for its clocked blocks (see the plumbing below). The per-kind
decision the plan makes — "this block's face is seconds, that one's is the index" — is the same
decision it already makes between port ops and step ops, by base class (DESIGN §3: "the
evolution kind is the type, never a flag"). It is not a dispatch in the evaluator loop and not a
mode on a block.

Is it a hidden mode on the *static block*? No more than today: a `Sine` in a continuous diagram
reads seconds and in a grid world reads `k` already (the code's behaviour, and R147's `t = k`).
The block has one body; the diagram has a kind. What the amendment removes is a *third* reading
(the index in a loop that has a clock) that only the user could have kept straight.

### What is risky: point 5's plumbing

"`t_start` is one leaf of the sampled diagram's tree, written through to every clocked block (like
`x0` views)". `x0` is a structural attribute on each block, assembled and written through by the
diagram. A `SampledSystem` is a view whose `params` *are* its source's dict (R146), so it cannot
hold a traced leaf; the only "write through" available is a structural float. Then:

- `jax.grad(objective)(pe.params)` has no `t_start` to differentiate (a scenario on the list);
- `vmap` over start times is impossible;
- `problem.compile` binding the problem's `t_start` leaf at run time (R148: "the horizon
  `[t_start, t_start + tf]`", costs and sets reading absolute time) would need a recompile per
  value.

The clean form is the one in §1: the leaf lives on the diagram (`loop.params["t_start"]`, a
reserved key beside the block ids, as a problem's tree already holds `x_start` and `t_start`
beside `plant`), and the plan converts. `SampledSystem.step(x, u, k) = update(x, u, k·T)` is the
origin-0 adapter for standalone consumers (value iteration, the RL environment, `jacobian`, the
interpreted `StepDiagramSystem.step`), and any consumer that has an origin calls the seconds face.
A consequence worth writing down: **a block in seconds never needs "its own index"** — on a
firing tick `(k // d)·d·Δ = k·Δ` exactly — so point 4's "receives ITS OWN index `k // d` and
converts inside" is needed only for a tick law under `%`. One index convention fewer.

(I considered carrying the clock as a state slot of the loop, `t_{k+1} = t_k + Δ`, the textbook
trick that makes a time-varying system autonomous; `t_start` would then be an `x0` entry, already a
leaf. It would duplicate the problem's `t_start` leaf and put a clock in every `x` the user plots.
The params leaf is the better home.)

### Minor

- `SampledSystem` over a static source has `n = 0`; `StepSystem.__init__` refuses `n < 1`
  (`require_step_state`). Relax it for a sampled static block, whose only memory (the hold) is a
  slot of the *diagram*, not of the block.
- Point 3's example `Step(t_step=1.0)`: the parameter is `step_time`.
- `%` on a tick law returns a `SampledSystem` with no seconds face; say so (`update` refuses,
  naming the tick), so that nobody later "gives it seconds" (the rejected wrapper, by another
  door).
- A `StepDiagramSystem` built by hand with `add_subsystem` of a clocked block: refuse, naming
  `SampledDiagramSystem`, the mirror of today's refusals; the operators build the right class.

### Same, a refinement, or different?

A refinement. Same skeleton (points 2, 4, 6; the rejections), one rule amended (a bare static
block in a sampled diagram reads the clock, so `%` on it sets its period and nothing else), one
piece of plumbing moved (the plan converts; `t_start` a traced leaf of the loop's tree), and the
"own index" simplification that follows.

## 4. Regret risks

What the maintainer will wish had been done differently in three years, and what each option
precludes.

1. **The silent-units pitfall (the proposal's points 1 and 3).** The first time a student's
   `reference=Step(step_time=1.0)` steps at 10 ms in a sampled loop, the fix will be the amendment
   above, and it will move the numbers of every demo and notebook that by then holds a bare source
   in a sampled loop. The same change now moves nothing: no sampled loop today holds a source block
   (the hybrid demos force the reference from outside, in seconds, through `compute_forced`, the
   very behaviour the amendment keeps).
2. **A structural `t_start`.** Closes the gradient and the Monte Carlo over the start time and
   makes `problem.compile` recompile per `t_start`. Irreversible only in the sense that every
   consumer would have been written around a float; choose the leaf now.
3. **Two index conventions.** "Own index for clocked blocks, base index for the rest" is a second
   thing to document, test and remember, for no number. Keep one: seconds for faces in seconds,
   `k // d` for a tick law under `%`.
4. **`where`-firing as the contract.** A slow MPC (`d = 5`) under a traced `where` solves five
   times too often; `lax.cond` skips it outside `vmap` only; the hyperperiod unroll is the real
   fix. The proposal already leaves this to the implementation; keep the contract to "fires on
   `k mod d = 0`, outputs held" and never promise the mechanism.
5. **The pure step diagram's `t = k` for static blocks.** The only regret would be a continuous
   source in a grid world; no clock exists there, so nothing is wrong in units, and `Step(step_time=5)`
   meaning "move 5" is what a tabular problem wants. No action.
6. **Non-commensurate periods and event clocks.** The gcd with a rational tolerance covers every
   classroom case (`0.1` and `0.03` → `0.01`; `0.1` and `1/30` refused). An event-triggered law
   is a select inside its own `update` on the periodic schedule (not precluded); a truly
   asynchronous clock is a different runner, out of scope for every option alike.
7. **`SampledSystem` with no seconds face for a tick source.** Harmless now; the regret would be
   someone adding one later. The contract sentence below forbids it.
8. **The name `update`.** Simulink's word; the alternative `advance` says less. No regret either
   way as long as the contract states `step(x, u, k) = update(x, u, k·T)`, one equation.

Nothing in the amended design precludes a future need on the brief's list. The one future need
the *proposal* precludes is the gradient and the batch over `t_start`, through its plumbing, not
its principle.

### "One signature" and "Equation-path purity" against each option

| Option | One signature | Purity | Note |
| --- | --- | --- | --- |
| Unit declaration on `StepSystem` (rejected) | kept in shape | kept | a mode switch in the core, against R118's own rule |
| Wrapper passing seconds into `step(x, u, k)` (rejected) | broken: `k` is an index by contract | kept | the tick world leaks |
| Three static kinds (disliked) | broken: three third slots | kept | the catalog doubled |
| A `t` port (R148, dropped) | time twice: the slot and an input | kept | every port tool sees a `t` input |
| Per-kind seconds in the evaluator loop (dropped in session) | kept | kept | the decision belongs at build, not per call |
| Converged proposal (bare static reads `k` in a sampled loop) | kept | kept | the record and a bare block disagree; two spellings of one signal |
| This review (bare static reads the clock; the plan converts; `t_start` a leaf) | kept: one third slot per kind, its unit the diagram's | kept: `t_start + k·Δ` is a pure function of `(k, params)` | one formula, one home |

## 5. Naming and the contract sentences for DESIGN §4

Names: `SampledSystem` (R146), `sample_period` on the object and `dt` at the calls (QUESTIONS'
option (a), which I endorse), `update` / `output` for the seconds face (Simulink's pair),
`t_start` for the clock's origin (R148's leaf, the same word on the problem), `Δ` in prose for
the base period.

The sentences, to sit under "`System`" after the `StepSystem` bullet:

- **Time.** The third argument of every equation is the time of the diagram the block sits in,
  in that diagram's unit: seconds in a continuous diagram and in a sampled one (a diagram with a
  clock), the tick index `k` in a pure step diagram (no clock). A static block (`n = 0`) reads
  whichever its diagram passes; it has no clock, no unit and no mode of its own, and the same
  `Sine` serves every kind of diagram. A `StepSystem` reads its own index `k` and never seconds:
  a tick law has no clock. A block's `params` never carry a sample time.
- **`SampledSystem(source, sample_period, *, solver, substeps)`** is the sampled-data kind, a
  `StepSystem` on a clock: one equation with two faces. `update(x, u, t, params)` and
  `output(x, u, t, params)` are its equations in seconds, `x(t + T) = update(x(t), u(t), t)`; the
  tick faces are `step(x, u, k, params) = update(x, u, k·T, params)` and `h` likewise, the clock's
  origin 0, and are never overridden. `discretize(sys, dt)` (RK4) and `sys % dt` (Euler) return
  one: a continuous source's `update` integrates `f` over the period, the input held, the stages
  at `t + c_i·T/substeps`; a static source's `output` samples `h` at `t`; a tick source keeps its
  tick face only (`update` refuses, naming the tick). A native sampled law (the delayed MPC, a
  discrete filter with a time-varying model) subclasses it with no source and overrides `update`
  and `output` only; `%` on it sets its period.
- **The sampled diagram is the clock.** A `SampledDiagramSystem` has a base period `Δ`, the
  greatest common divisor of its blocks' periods, and one origin, the leaf
  `loop.params["t_start"]` (0 by default; `compute_rollout(t_start=)` writes it; `problem.compile`
  binds the problem's). Its compiled plan, one pure map `X_{k+1} = F(X_k, U_k, k; P)` on the base
  tick, hands every block whose equations are in seconds (a static block, a `SampledSystem`) the
  time `t = t_start + k·Δ`, and every tick law its own index `k // d`; a block of divisor
  `d = period/Δ` fires on `k mod d = 0`, its outputs held in state slots between firings; every
  block fires at `k = 0`; signals propagate within the tick. `%` sets a block's period and nothing
  else: on a static block it is the sampler, the same block at the base period; a `Sine()` bare
  in a sampled loop is `sin(ω t_k)`. Seconds never reach a tick law; a wall clock never reaches a
  block (the host runner reads it and hands the blocks the nominal tick time).
- **The record agrees with the blocks:** `compute_rollout` returns `t = t_start + k·Δ` on a sampled
  system and `t = k` on a pure one; `compute_trajectory` on a sampled system with a continuous
  source keeps the substeps, `t_start + k·Δ/substeps`.

One line for DESIGN §5, beside "The step model": *`discretize(plant, dt)` is a `SampledSystem`;
its `step(x, u, k)` binds `t_k = k·dt`; a consumer with an origin (a problem's `transition`, a
sampled diagram's plan) calls `update(x, u, t_start + k·dt)` instead.*

And the CONSTITUTION's "One signature", one clause added where it says "the model's arguments
`(x, u, t; p)`": *`t` is the time of the diagram the object sits in, seconds under a clock and
the tick index without one.* Invariant 1's amendment (the sampled loop a step diagram of the
sibling algebra) is already scheduled by R146 and needs no further word from this review.
