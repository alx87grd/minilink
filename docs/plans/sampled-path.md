# The sampled loop

**Where it sits:**
- the core's hybrid loop, "one model, two runners" ([core-architecture.md](core-architecture.md)
  §5); its rows are [TODO.md](TODO.md) §1.5, with the conventions' sampled-loop steps
  (AC-8a, AC-8b of [automation-by-convention.md](automation-by-convention.md));
- the decisions: the 2026-10-27 preliminary choices, R145–R147 of
  [the core-vision record](../reviews/2026-10-27-core-vision-decisions.md) and R125–R127 of
  [the design-questions record](../reviews/2026-10-27-design-questions-rulings.md) (R127 revised
  by R146 and R147); before them R104, R105 and R123
  ([record](../reviews/2026-10-25-open-questions-rulings.md)) and R13, R14
  ([record](../reviews/2026-10-22-rulings-session-1.md));
- the MPC in the loop: [planner-and-mpc.md](planner-and-mpc.md) (R148), not restated here;
- what `t` is for every block in every diagram, and `SampledSystem` as the sampled-data kind:
  R158 ([record](../reviews/2026-10-28-session-rulings.md)), on three independent reviews
  ([A](../reviews/2026-10-28-time-in-the-core-reviewer-a.md),
  [B](../reviews/2026-10-28-time-in-the-core-reviewer-b.md),
  [C](../reviews/2026-10-28-time-in-the-core-reviewer-c.md)).

**Lane:** core; every step is **[ask — core]**.

The plant half of the loop, the step model (`discretize`, the compiled evaluator's `step` and
`rollout`), landed on 2026-10-24; its contract is DESIGN §5 ("One body per method", "The step
model"). This doc holds the other half: a sampled controller (a digital PID, a held neural policy,
MPC, MPPI, the RL policies) on a clock, closed on a continuous plant. The section number (§8) is
kept from the full plan because other documents cite it.

## 8. The sampled loop (S31)

The loop is not a tool beside the algebra any more (R13's "a tool, not a `System`" and R120's
wording are revised by R127 and R146): it is a system of the step algebra, built by composition,
with a clock on its blocks. CONSTITUTION invariant 1 (the sampled loop a step diagram of the
sibling algebra) and §6's grammar sentence (the operators on both algebras) are amended in the
landing change; the text comes to the maintainer with it.

### 8.1 Time in the core: the kinds, and what `t` is for each (R146, R158)

**The principle (R158).** Time is one channel, the third argument of every equation, and what it
holds is decided by the diagram a block sits in, never by the block: seconds wherever the diagram
has a clock (a continuous diagram, a sampled one), the count where it has none (a pure step
diagram). A tick law reads its own count and never seconds. A block on a clock is one equation
with two faces. No block declares a unit, a mode or a clock of its own, and a block's `params`
never carry a sample time.

| Kind | Its equations | What `t` is | Who supplies it |
| --- | --- | --- | --- |
| static `System` (`n = 0`: a gain, a sum, a saturation, a reference, a schedule) | `h(x, u, t)` | the time of the diagram it sits in: seconds under a clock, the count `k` in a pure step diagram | the diagram; one catalog serves every kind of diagram |
| `DynamicSystem` | `f(x, u, t)`, `h(x, u, t)` | seconds, always | the integrator, or its `SampledSystem` |
| `StepSystem` (a tick law: chess, a grid world, a z-domain filter) | `step(x, u, k)`, `h(x, u, k)` | its own count of firings; no clock, no seconds, ever | the step or sampled diagram (`k`, or `k // d` under a divisor) |
| `SampledSystem` (a system on a clock) | **the seconds face is the equation:** `update(x, u, t)`, `output(x, u, t)`, Åström–Wittenmark's `x(kh + h) = F(x(kh), u(kh), kh)`; **the tick face serves the step algebra:** `step(x, u, k) = update(x, u, k·sample_period)`, `h` likewise, the clock's origin 0 | seconds inside, the count outside | the base class, one line; a consumer with an origin (a sampled diagram's plan, a problem's `transition`, the host runner) calls the seconds face |

```text
StepSystem                               pure tick: step(x, u, k, params); no clock, no dt
  ├─ StepDiagramSystem                   a diagram of step systems, by the assembly code (§8.4)
  │    └─ SampledDiagramSystem           a diagram with a clocked block: the computer, and computer @ plant
  └─ SampledSystem                       any system on a clock; discretize, % and the native laws
```

- **`StepSystem`** stays clean: no `dt`, no `source`. Chess, a grid world and a native z-domain
  design are one. Seconds never reach it.
- **`SampledSystem(source, sample_period, *, solver="rk4", substeps=1)`**, "this system on this
  clock", one equation with two faces. Its implementations, each writing the seconds face:
  - a continuous source (`discretize(sys, dt)`, `sys % dt`): `update` integrates `source.f` over
    the period, the input held, the stages at `t + c_i·Δ/substeps` (the step model of DESIGN §5,
    today's `ev.step(x, u, t, dt=)`); `output` is `source.h` at `t`;
  - a static source under `%`: no state, `output` samples `source.h(u, t)`; needed only to slow
    or hold the block (a divisor above one, §8.2), never to give it seconds; a sampled static
    block has `n = 0` (`require_step_state` relaxed for it);
  - a tick source under `%`: the tick face delegates `source.step(x, u, k)` and `h`; it has no
    seconds face (`update` refuses, naming the tick); it declares that it counts;
  - **a native sampled law** subclasses it with no `source` and writes `update` and `output` in
    seconds: the delayed MPC first ([planner-and-mpc.md](planner-and-mpc.md) §6), later a
    discrete Kalman filter with a time-varying model, MPPI with its shifted warm start, a diffusion
    policy's action chunk. `%` on a native law sets its period. (Revises R146's "nobody subclasses
    it".)
  - a view, not a container: its `params`, ports, state and labels are the source's own; a
    parent's tree holds it as one block; `sample_period` is structural, not a parameter (a sweep,
    never a gradient); the tick face is never overridden.
- **Today:** `Discretization` (`analysis/discretization.py`) is this view under another name, its
  `step` already binding `t_k = k·dt` and calling the source in seconds; it is renamed
  `SampledSystem` (its `dt` becomes `sample_period`, R157; `ExactDiscretization` folds in as
  `solver="exact"`). `Computer` and `StepSchedule` (`simulation/discrete/computer.py`) and
  `HybridDiagram` (`core/discrete/hybrid_diagram.py`, not a `System`) go; "computer" stays the word
  of the host runner (§8.6) and of the drawing's box.
- **Draws.** A block that reads seconds draws from `(seed, ⌊t/Δ⌋)` of its own period, the noise
  block's convention (randomness.md §4; the seeded policy, R129; the MPC's per-tick seed); a pure
  tick law from `(seed, k)` (D14).

### 8.2 The diagrams: what each hands its blocks; multi-rate in the sampled diagram's plan (R146, R158)

| Diagram | What it passes to its blocks | The clock's origin | The record (`compute_rollout`) |
| --- | --- | --- | --- |
| `DiagramSystem` (continuous) | seconds, to every block, at the integrator's times | the run's `t0` | `t`, seconds |
| `StepDiagramSystem` (pure tick) | `k`, to every block; a static block reads `t = k` (what the code does today, `discrete.py`'s `port_signals`) | none | `t = k` |
| `SampledDiagramSystem` (a clock `Δ`) | **the plan is the clock:** `t_k = t_start + k·Δ`, computed once per tick, to every block whose equations are in seconds (a static block, a `SampledSystem`'s seconds face); its own count `k // d` to a tick law under `%`; the base count `k` to a bare tick law | the run's argument, like `x0`: `compute_rollout(n_steps, t_start=0.0)`, bound by `problem.compile` from the problem's `t_start` leaf (R148) | `t = t_start + k·Δ`; the dense path `t_start + k·Δ/substeps` (§8.5) |

- **A sampled diagram is a step diagram with a period.** Its compiled face is still one pure map on
  the base tick, `X_{k+1} = F(X_k, U_k, k; t_start, P)`; `jit`, `grad`, `vmap`, `jacobian`, the
  poles over a hyperperiod and `problem.compile` see a step diagram with one more argument. The
  multi-rate structure lives in the compiled plan (`plan.py`), not in the blocks:
  - **at wiring:** the base period `Δ` is the greatest common divisor of the clocked blocks'
    periods (a rational tolerance, today's `1e-9` of `rate_to_divisor`); each clocked block gets
    its divisor `d = period / Δ`, a period that is not a whole multiple refused; a tick law and a
    time-free static block have `d = 1`; the plant block of `computer @ plant` is
    `discretize(plant, Δ, substeps=n)` (§8.4);
  - **the state:** the blocks' states, then one hold slot per output of every block with `d > 1`
    (a zero-order hold is a memory, so it is state: the textbook sampled-data model);
  - **the tick:** a block of divisor `d` fires on `k mod d = 0`; when it fires its outputs are
    computed and its state stepped, otherwise its outputs are read from the hold slots and its
    state kept; every block fires at `k = 0`, so no hold needs an initial value; signals
    propagate within the tick (§8.3);
  - **the arguments, per face, decided at build by base class** (as the plan already separates
    port ops from step ops): a seconds face gets `t_k`; a tick law under `%` gets `k // d`, an
    exact integer; a bare tick law gets `k`. No `StepSystem` ever receives seconds, and the pure
    step diagram's plan is untouched (one integer to every op, as today).
  - **the firing's form** is the implementation's, never the contract's: a `lax.cond` on
    `k mod d` (its predicate on the scan index alone, so it survives `vmap` over starts and
    seeds) or the hyperperiod `lcm(d)` unrolled, both bit-identical to a select; never a `where`,
    which would solve a slow MPC `d` times per period.
- **A worked tick,** `(filt % 0.01) >> (ctl % 0.1) @ plant`: `Δ = 0.01`, `d_filt = 1`,
  `d_ctl = 10`, `d_plant = 1`. On base tick `k = 20`, `t_k = 0.20 s`: the filter's `update` runs
  at `0.20 s`; the controller fires (`20 mod 10 = 0`), its `update` runs at `0.20 s`; the plant
  takes one RK4 period of `0.01 s` with its substeps. On `k = 21` the controller does not fire
  and the plant reads its held command. All clocks agree because `(k // d)·d·Δ = k·Δ` on every
  firing tick, so a block in seconds never needs its own index.
- **A static block in a sampled loop reads seconds.** `Sine(omega) >> (pid % 0.01) @ plant` is
  the discrete-time sinusoid at `t_k`; `Step(step_time=1.0)` steps at one second; a gain
  schedule `K(t)` reads the clock. `Sine()` and `Sine() % Δ` at the base period are the same
  block, bit for bit, so `%` on a static block means one thing everywhere, its period (a slower,
  held reference). In a pure step diagram the same `Step` steps at move 1, the only meaning a
  clockless loop has.
- **A nested sampled or step box is inlined into one flat plan** at compile (ids joined as the
  flatten code already does), so a loop has one clock and one origin, the gcd over every leaf; the
  box stays an object for `@`'s roles, `plot_diagram` and `trajectory_of`. Never two scans.
- **Refusals at wiring:** a clocked block in a hand-built `StepDiagramSystem` (naming
  `SampledDiagramSystem`); a period that is not a multiple of the base; a continuous block with
  states in any step diagram (today's `require_step_subsystem`); a tick law designed at one rate
  and left bare runs at the base rate, which the docs say in one sentence (`filt % 0.01` names its
  rate).
- **Out of scope:** periods that are not commensurate, asynchronous or event-triggered clocks,
  jitter (ROADMAP §7). An event-triggered law is a select inside its own `update` on the periodic
  schedule, not precluded; a truly asynchronous clock is another runner.

### 8.3 One tick, one meaning; delay is a block (R125, R123)

- Every step diagram and every runner propagates within the tick: if A feeds B, B at tick `k`
  reads A's output at tick `k`, as a block diagram's equations say.
- Latency exists only where a block states it: a `UnitDelay`, a held slow output (§8.2), the
  MPC's one-period computational delay.
- An algebraic loop (a cycle with no delay) is refused at wiring.
- **Today:** `Computer.tick` reads the previous tick's committed buffer, one hidden tick of
  latency per wire, and hands slow blocks the base tick; both go, and the hybrid demos' numbers
  move once (R80's closeness report).
- **The MPC:** its tick latch (`MPCTickLatch`) goes; the default delayed form is a native
  `SampledSystem` that solves in `update` and reads its plan from its state, and the pure form
  (`delay=False`) is a static block. Both are designed in
  [planner-and-mpc.md](planner-and-mpc.md) §6. The pure form covers what R123's option B (a
  Mealy `update` returning state and output) was kept as a study for, a zero-delay output from an
  expensive computation, with no change to the `StepSystem` contract: it needs one signal pass per
  tick in the step evaluators (planner-and-mpc.md §10).

### 8.4 The operators; `sampled @ continuous` is where the worlds meet (R126, R145, R146)

- **`%` is `discretize` with Euler** (R145): `sys % dt` is `discretize(sys, dt, solver="euler")`
  for any system. A law with continuous states (a PID, a lead-lag, an observer) becomes its
  difference equation, `z_{k+1} = z_k + dt·f(z_k, e_k)`; a static block and a `StepSystem` are
  tagged with their period; a native sampled law gets its period. `discretize(sys, dt)` keeps RK4
  as its default, for plants; `%` takes no `solver=`. Both return a `SampledSystem` and leave the
  source unchanged.
- **`>>`, `+` and `@` between step systems** have their continuous meaning, on the same assembly
  code (`core/assembly.py`); the result is a `StepDiagramSystem`, or a `SampledDiagramSystem`
  when a block is clocked.
- **`sampled @ continuous` is the one place the two worlds meet:** the continuous plant becomes
  `discretize(plant, Δ, substeps=n)` under the role name `plant`, and `computer @ plant` is a
  `SampledDiagramSystem`; no loop class of its own. Every other mix of a continuous and a step
  operand is refused, naming `%`.
- **The step tools apply to the loop:** `jacobian` (and `wrt="t"` on a seconds face), the
  linearization and the sampled-data loop's poles against the unit circle (for a multi-rate loop,
  over one hyperperiod), `problem.compile`, `vmap`, `grad`.

### 8.5 Two run verbs that agree; the dense path (R147)

- **`compute_rollout`**, on every `StepSystem`: the ticks, a `Trajectory`, `t = k` on a pure step
  system and `t = t_start + k·Δ` on a sampled one (`t_start=0.0` its argument). It is what
  `evaluate` scores and `jax.grad` differentiates.
- **`compute_trajectory`**, wherever continuous time is underneath: a `DynamicSystem`
  (unchanged), a `SampledSystem` or `SampledDiagramSystem` with a continuous source. It is
  refused on a pure step system and on a clocked step source.
- **The dense run is the same scan with its substeps kept.** `step_body`
  (`core/compile/evaluators/stepping.py`) already computes the plant at `t_k + j·Δ/substeps`; the
  dense path keeps every substep end, so its value at each tick is the tick value, bit for bit.
  The input is the held staircase; outputs are evaluated at every substep point.
- **One new evaluator method beside `rollout`**: `rollout` keeps its shape `(N+1, n)`, the new
  method returns `(N·substeps+1, n)` from one shared period body, on the system and the
  sampled-diagram evaluators. `compute_rollout` calls `rollout`, `compute_trajectory` the new
  method; no flag changes `rollout`'s return shape.
- **The substep count is structural, on the loop's plant `SampledSystem`**, defaulting from the
  automatic-`dt` rule (`AUTO_DT_SCALE × τ_min` against `Δ`), so `compute_rollout`,
  `compute_trajectory` and `evaluate` integrate with the same count.
- **The cross-check:** `compute_trajectory(solver="scipy")` is the host runner (§8.6) with an
  adaptive plant between the held inputs, agreeing within its tolerance. The loop re-clocked at
  `Δ/substeps` is the dense method's test oracle, to `rtol 1e-12`.
- `x0_plant=` and `x0_computer=` go: one `x0`, written through to each block (R124).
- The hybrid demos and tutorial 06 re-pin once, with the same-tick semantic (§8.3) and the MPC
  delay.

### 8.6 Two runners over the same blocks

Neither runner owns a copy of a block's equations; both follow §8.3, and both hand a block the
same time: the nominal tick time `t_start + k·Δ` to a seconds face, the count to a tick law. A
wall clock never reaches a block.

- **The pure step diagram** is the model and the judge: one scan of the loop's tick, `jit`,
  `grad`, `vmap`, analysis, `problem.compile` and `evaluate`. The blocks decide tracing: a gain, a
  neural policy, a digital PID or a fixed-iteration MPC trace end to end
  ([native-optimizers.md](native-optimizers.md) §4.6–§4.6b); a host-only block (an Ipopt MPC)
  runs the same scan on the NumPy face.
- **The host clock runner** ticks the blocks on the host and integrates the plant between ticks.
  It serves `compute_trajectory(solver="scipy")` (`solve_ivp` restarted at each held-input change,
  no event detection since the sample times are known), host-only blocks and the **realtime
  runner**: game mode and hardware in the loop, which read the keyboard or the device between
  ticks and pace the ticks on the wall clock (jitter is the runner's affair). The pure step
  diagram is its test oracle at the ticks, a bare reference included, since both hand it `t_k`.
- **Today:** `HybridSimulator` (`simulation/discrete/hybrid_simulator.py`) is a Python tick loop
  over the mutable `Computer`, and the realtime simulator (`simulation/realtime/`) has its own
  loop; both become thin callers of the pure blocks. The MPC's `export_to_computer` goes
  (planner-and-mpc.md §9).

### 8.7 Open points

- **The `core` → `simulation` import:** today `core/discrete/hybrid_diagram.py` and
  `hybrid_composition.py` load `simulation/discrete/computer.py` on first use (a written
  exception, R100); those modules go with R146, so what is left is whether the host clock runner
  keeps a lazy import.
- **The realtime runner's re-base** on the pure blocks: in the v0.3 core, or with the realtime
  architectural review (its v1.0 row in [TODO.md](TODO.md)), the reviewers' view.
- **The role name of the computer side** inside `computer @ plant` (`controller`, or a box holding
  `controller`, `error` and `filter`): automation-by-convention.md §4.5, settled with the landing
  change.
- **Between the samples** (the one-score question, [QUESTIONS.md](../dev/QUESTIONS.md)):
  time-varying sets and the margins between samples; one failure rule and the input hold.
- **Names for the sweep:** the dense evaluator method (`rollout_path`, `rollout_dense`, …);
  `update` / `output` (Simulink's pair; `advance` the alternative); `t0` and `t_start`, one word
  for the run's origin on every runner.

### 8.8 The sampled loop's details, D13–D15

The ids stay, as QUESTIONS.md and the rulings records cite them.

#### D13 The run-state caches (S31; RULES 2.4)

**Today:** `StepSystem.rollout` (`core/discrete/step_system.py`) and `HybridDiagram.last_result`
cache a run beside the allowed `self.traj`.

**Recommended (not ruled):** they go. A run returns its result; `loop.compute_trajectory()` and
`loop.compute_rollout()` return the `Trajectory` and store it as `loop.traj` like every `System`,
nothing else. The facades that read `last_result` read `traj`.

#### D14 Controller memory and the draws on the tick (S31; future-policies.md §5 points 2, 4)

**Ruled 2026-10-25 (R104), placed by R158:** no new channel. A sampled controller with memory
*is* a system with state, which the loop carries across ticks (MPPI's shifted warm start, a
diffusion policy's action chunk, the MPC's `z`): a native `SampledSystem` when its law reads
seconds, a `StepSystem` when it counts. Its draws come from its own `params["seed"]` and the
noise block's counter, `⌊t/Δ⌋` of its own period for a seconds reader and `k` for a tick law, so
`vmap` over seeds and `problem.realize(key, n)` work unchanged (R144: a source block gets an
integer seed leaf).

```python
class SeededPolicy(SampledSystem):       # memory in x, draws from (seed, ⌊t/Δ⌋)
    def update(self, x, u, t, params=None):
        eps = standard_normal(params["seed"], sample_index(t, self.sample_period), self.n_samples)
        ...
```

#### D15 The compiled problem dispatches on the policy's kind (S31; V1 step 7b)

**Ruled 2026-10-25 (R105):** `problem.compile(policy, dt=)` reads the policy. A continuous law
runs the loop as a continuous diagram; a sampled controller (a `SampledSystem` or a
`SampledDiagramSystem`, anything built by `%`) runs `policy @ plant` as the step diagram of §8.6,
one scan. One `compile` and one `evaluate` for both kinds. Today the second branch refuses
(`require_continuous` in `planning/evaluator.py` refuses a `StepSystem` controller and names the
hybrid loop); it lands with the sampled face (TODO §1.5). **Closed by R158:** `dt=` omitted reads
the loop's base period `Δ`; `dt=` given must equal it, else refused naming `Δ`.

## Kept for S53: the price of a parameter family

Measured on 2026-10-22 (`benchmarks/params_binding.py`, CPU, float64), before the step model
landed: a batch of 1000 rollouts × 1000 RK4 steps under `vmap` took 127 ms on the pendulum with
shared parameters and 673 ms with one `m` per rollout (5×); on the cart-pole, 1307 ms and
1276 ms (none). Shared parameters are compile-time constants, so XLA folds `g/l`, `m·l²`,
`1/I` once; a per-member parameter is a `(B,)` operand, nothing folds, and the derived
quantities are recomputed at every step for every member. Results never change. Two
mitigations for the profiling row (S53), neither a design change: hoist the plant's
pre-computation block (the `m·l²` lines RULES 5.3 already separates) out of the scan when
parameters are batched, computing it once per member; or carry the drawn parameters as
constant columns of the batched state so they ride the same memory stream.
