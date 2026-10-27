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
- the MPC in the loop: [planner-and-mpc.md](planner-and-mpc.md) (R148), not restated here.

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

### 8.1 The tree (R146)

```text
StepSystem ─── StepDiagramSystem         pure tick: step(x, u, k, params); no clock, no dt
  └─ SampledSystem                       any system on a clock; discretize and % return it
      └─ SampledDiagramSystem            a diagram with a clocked block: the computer, and computer @ plant
```

- **`StepSystem`** stays clean: no `dt`, no `source`. Chess, a grid world and a native z-domain
  design are one.
- **`SampledSystem(source, sample_period, *, solver="rk4", substeps=1)`**, "this system on this
  clock":
  - `source` is always set: a continuous system (its `step` integrates `source.f` over the
    period, the input held), a static block (its `h` sampled) or a pure `StepSystem` (its `step`
    delegated); `solver` and `substeps` apply to a continuous source only;
  - a view, not a container: its `params`, ports, state and labels are the source's own; a
    parent's tree holds it as one block; nobody subclasses it;
  - `sample_period` is structural, not a parameter (a sweep, never a gradient).
- **`SampledDiagramSystem`**: any diagram with a clocked block; its base period is the greatest
  common divisor of its blocks' periods (§8.2).
- **Today:** `Discretization` (`analysis/discretization.py`) is this view under another name; it
  is renamed `SampledSystem` (its `dt` becomes `sample_period`; `ExactDiscretization` folds in as
  `solver="exact"`). `Computer` and `StepSchedule` (`simulation/discrete/computer.py`) and
  `HybridDiagram` (`core/discrete/hybrid_diagram.py`, not a `System`) go; "computer" stays the
  word of the host runner (§8.6) and of the drawing's box.

### 8.2 Periods on blocks: the base period, divisors, held outputs (R146)

- A period is set on a block (`ctl % 0.1`), never on a schedule; the schedule is derived.
- The base period `Δ` is the greatest common divisor of the clocked blocks' periods. A period that
  is not an integer multiple of `Δ` is refused at wiring. A pure step block inside runs at `Δ`.
- A block of divisor `d = period / Δ > 1` fires on `k mod d = 0`; between firings its outputs are
  held, and **the held outputs are state slots** of the diagram (a zero-order hold is a memory, so
  it is a state of the discretized loop: the textbook sampled-data model). The tick stays one pure
  map `X_{k+1} = F(X_k, u_k, k; p)`.
- A multi-rate computer is ordinary composition: `(filt % 0.01) >> (ctl % 0.1)`; the MPC's two
  rates are `mpc % 0.05` beside a `% 0.01` block ([planner-and-mpc.md](planner-and-mpc.md) §6.4).
- **Left to the implementation:** on the traced face a firing is a select on `k mod d`; a runner
  may skip idle blocks or unroll the hyperperiod `lcm(d_i)` as optimizations with the same
  numbers.
- **Out of scope:** periods that are not commensurate, asynchronous or event-triggered clocks,
  jitter (ROADMAP §7).

### 8.3 One tick, one meaning; delay is a block (R125, R123)

- Every step diagram and every runner propagates within the tick: if A feeds B, B at tick `k`
  reads A's output at tick `k`, as a block diagram's equations say.
- Latency exists only where a block states it: a `UnitDelay`, a held slow output (§8.2), the
  MPC's one-period computational delay.
- An algebraic loop (a cycle with no delay) is refused at wiring.
- **Today:** `Computer.tick` reads the previous tick's committed buffer, one hidden tick of
  latency per wire; it is removed, and the hybrid demos' numbers move once (R80's closeness
  report).
- **The MPC:** its tick latch (`MPCTickLatch`) goes; the default delayed form solves in `step`
  and reads its plan from its state, and the pure form (`delay=False`) is a static block.
  Both are designed in [planner-and-mpc.md](planner-and-mpc.md) §6. The pure form covers what
  R123's option B (a Mealy `update` returning state and output) was kept as a study for, a
  zero-delay output from an expensive computation, with no change to the `StepSystem` contract:
  it needs one signal pass per tick in the step evaluators (planner-and-mpc.md §10).

### 8.4 The operators; `sampled @ continuous` is where the worlds meet (R126, R145, R146)

- **`%` is `discretize` with Euler** (R145): `sys % dt` is `discretize(sys, dt, solver="euler")`
  for any system. A law with continuous states (a PID, a lead-lag, an observer) becomes its
  difference equation, `z_{k+1} = z_k + dt·f(z_k, e_k)`; a static block and a `StepSystem` are
  tagged with their period. `discretize(sys, dt)` keeps RK4 as its default, for plants; `%`
  takes no `solver=`. Both return a `SampledSystem` and leave the source unchanged.
- **`>>`, `+` and `@` between step systems** have their continuous meaning, on the same assembly
  code (`core/assembly.py`); the result is a `StepDiagramSystem`, or a `SampledDiagramSystem`
  when a block is clocked.
- **`sampled @ continuous` is the one place the two worlds meet:** the continuous plant becomes
  `discretize(plant, Δ, substeps=n)` under the role name `plant`, and `computer @ plant` is a
  `SampledDiagramSystem`; no loop class of its own. Every other mix of a continuous and a step
  operand is refused, naming `%`.
- **The step tools apply to the loop:** `jacobian`, the linearization and the sampled-data loop's
  poles against the unit circle (for a multi-rate loop, over one hyperperiod), `problem.compile`,
  `vmap`, `grad`.

### 8.5 Two run verbs that agree; the dense path (R147)

- **`compute_rollout`**, on every `StepSystem`: the ticks, a `Trajectory`, `t = k` on a pure step
  system and `t = k·Δ` on a sampled one. It is what `evaluate` scores and `jax.grad`
  differentiates.
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

Neither runner owns a copy of a block's equations; both follow §8.3.

- **The pure step diagram** is the model and the judge: one scan of the loop's tick, `jit`,
  `grad`, `vmap`, analysis, `problem.compile` and `evaluate`. The blocks decide tracing: a gain, a
  neural policy, a digital PID or a fixed-iteration MPC trace end to end
  ([native-optimizers.md](native-optimizers.md) §4.6–§4.6b); a host-only block (an Ipopt MPC)
  runs the same scan on the NumPy face.
- **The host clock runner** ticks the blocks on the host and integrates the plant between ticks.
  It serves `compute_trajectory(solver="scipy")` (`solve_ivp` restarted at each held-input change,
  no event detection since the sample times are known), host-only blocks and the **realtime
  runner**: game mode and hardware in the loop, which read the keyboard or the device between
  ticks. The pure step diagram is its test oracle at the ticks.
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
- **`problem.compile(controller, dt=)` on a sampled controller** (D15, R141): whether `dt=` must
  equal the loop's base period, or is read from it.
- **Between the samples** (the one-score question, [QUESTIONS.md](../dev/QUESTIONS.md)):
  time-varying sets and the margins between samples; one failure rule and the input hold.
- **Names for the sweep:** the dense evaluator method (`rollout_path`, `rollout_dense`, …).

### 8.8 The sampled loop's details, D13–D15

The ids stay, as QUESTIONS.md and the rulings records cite them.

#### D13 The run-state caches (S31; RULES 2.4)

**Today:** `StepSystem.rollout` (`core/discrete/step_system.py`) and `HybridDiagram.last_result`
cache a run beside the allowed `self.traj`.

**Recommended (not ruled):** they go. A run returns its result; `loop.compute_trajectory()` and
`loop.compute_rollout()` return the `Trajectory` and store it as `loop.traj` like every `System`,
nothing else. The facades that read `last_result` read `traj`.

#### D14 Controller memory and the draws on the tick (S31; future-policies.md §5 points 2, 4)

**Ruled 2026-10-25 (R104):** no new channel. A sampled controller with memory *is* a `StepSystem`
with state, which the loop carries across ticks (MPPI's shifted warm start, a diffusion policy's
action chunk, the MPC's `(z, t_last)`). Its draws come from its own `params["seed"]` and the tick,
the noise block's counter-based pattern, so `vmap` over seeds and `problem.realize(key, n)` work
unchanged (R144: a source block gets an integer seed leaf).

```python
class SeededPolicy(StepSystem):          # memory in x, draws from (seed, k)
    def step(self, x, u, k=0, params=None):
        eps = standard_normal(params["seed"], k, self.n_samples)   # the noise block's draw
        ...
```

#### D15 The compiled problem dispatches on the policy's kind (S31; V1 step 7b)

**Ruled 2026-10-25 (R105):** `problem.compile(policy, dt=)` reads the policy. A continuous law
runs the loop as a continuous diagram; a sampled controller (a `SampledSystem` or a
`SampledDiagramSystem`, anything built by `%`) runs `policy @ plant` as the step diagram of §8.6,
one scan. One `compile` and one `evaluate` for both kinds. Today the second branch refuses
(`require_continuous` in `planning/evaluator.py` refuses a `StepSystem` controller and names the
hybrid loop); it lands with the sampled face (TODO §1.5).

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
