# The sampled loop

**Status:** the step model this loop runs on landed on 2026-10-24 (the step-model work of
the continuous core, S75, steps SM-0 to SM-8): `discretize` and the compiled evaluator's one
`step` and `rollout`, called by every sampled consumer. Its contract is DESIGN §5 ("One body
per method", "The step model"); the decisions taken during the run are in
[the run's log](../reviews/2026-10-24-v03-run-decisions.md) (§2, §9, §10). The design of that
half (the eight discretizations it replaced, the knobs, the measurements, the steps) is in git
history. This doc now holds the other half: the sampled loop (S31), ruled a tool with ticks
(R13, R14 of [the rulings](../reviews/2026-10-22-rulings-session-1.md)), its details parked
(D13–D15 of [end-state-api.md](end-state-api.md)).  
**Lane:** core (the hybrid tool, the sampled kernel, every sampled controller) — every step is
**[ask — core]**.  
**Rung:** v0.4: S31 in [TODO.md](TODO.md) §5, after the continuous path (R86), with the
automation-by-convention plan's last step (AC-8b) and the sampled face of the differentiable
problem (its step 7b).

A sampled controller (a digital PID, a held neural policy, MPC, MPPI, the RL policies) runs
once per sample period on a computer, and the plant is stepped by the step model between two
samples. This doc designs the loop that joins the two clocks: one kernel over clock ticks, in
a NumPy and a JAX face. The section number (§8) is kept from the full plan because other
documents cite it.

## 8. The sampled loop (S31)

The step model (DESIGN §5) is the plant half; the loop half runs a sampled controller on it.

**Ruled 2026-10-22 (R13): the loop is a tool, not a `System`; the name `HybridDiagram` stays for
now. Ruled the same day (R14): ticks** (`step(x, u, k, params)`; the loop converts to seconds
where it calls the plant). The rest (D13–D15 of [end-state-api.md](end-state-api.md)) is parked
to the hybrid phase, after the continuous core.

- **Two model algebras, both closed.** The continuous one (`System`, diagrams, `@`, `>>`,
  `+`) and the discrete-time one (`StepSystem`, `StepDiagramSystem`). The loop that joins the
  two clocks is neither: an object with `compute_trajectory` and `compile`, built by `%`, and
  not a `System`. Nesting a sampled loop in a diagram has no consumer, `linearize` and the
  frequency tools have no meaning on it, and multi-clock scheduling as a framework feature is
  out of scope (ROADMAP §7).
- **One sampled rollout kernel, two faces.** The controller held over its period, the plant
  stepped by the step model between samples, a `StepSchedule` for multi-rate; a NumPy face
  and a JAX face, as `Simulator` and `compile` have. Its two copies today
  (`HybridSimulator` in `simulation/discrete/hybrid_simulator.py`, the MPC computer export,
  `export_to_computer` in `planning/trajectory_optimization/mpc/controller.py`)
  become its callers; the RL environment is a consumer of the step model only, the learner
  calling the policy itself (ruled 2026-10-22).
- **Two loops on purpose.** The compiled problem (`problem.compile(arg, dt)`, R15) keeps the
  continuous closed loop as a diagram, for evaluation and the program export of a continuous
  law; this kernel is the loop of every sampled controller. A sampled controller optimized by
  gradient (a digital PID, a held neural policy) goes through this kernel, once V1's step 7b
  dispatches it there (D15, the hybrid phase).
- **The kernel never breaks tracing; the computer decides.** A gain, a neural policy or a
  digital PID traces end to end (the RL planner, gain tuning, a digital PID tuned by
  gradient). MPC with SciPy or Ipopt inside runs on the NumPy face. A traceable inner solver
  later makes MPC a differentiable policy too, so the MPC block's `step` stays a pure
  function of arrays with the solver inside: the fixed-iteration MPC tick and MPPI's tick of
  [native-optimizers.md](native-optimizers.md) §4.6–§4.6b.
- **Parked to the hybrid phase (D13–D15):** whether the run-state caches
  (`StepSystem.rollout` in `core/discrete/step_system.py`, `HybridDiagram.last_result` in
  `core/discrete/hybrid_diagram.py`) join `self.traj` as allowed caches (RULES 2.4); the
  `core` → `simulation` import (`core/discrete/hybrid_diagram.py` and `hybrid_composition.py`
  load `simulation/discrete/computer.py` on first use); whether
  the tick carries controller memory (MPPI's warm start, a diffusion policy's action chunk;
  random draws come from the controller's seed and the tick, as the noise block's;
  [future-policies.md](future-policies.md)); how the compiled problem dispatches a sampled
  controller to this kernel (V1's step 7b).

### 8.1 The pure tick of a multi-rate loop (draft ideas, 2026-10-25)

**The question (the maintainer, 2026-10-25).** The early `Computer` design chose not to write a
pure step function for a multi-clock loop, because the slow clock's state must be held while the
fast clock updates, and the continuous plant integrated between samples. A fully native, pure
step model of the hybrid loop (one `lax.scan`, `jit`, `vmap`, `grad`: native-optimizers.md §4.6)
needs that solved. Nothing below is ruled.

**What the code already does.** `Computer.tick` (`simulation/discrete/computer.py`) is nearly
pure already: its only mutable fields are the tick `k`, the block states `x` and the committed
signal buffer `signal_read`. A block fires when `k mod d = 0` (`StepSchedule`'s integer divisors,
so the rates are commensurate by construction); a skipped block holds by the copy of the read
buffer into the write buffer; fired blocks read the committed buffer only. What keeps it from
tracing: in-place writes, a Python `if` on `k`, NumPy only (`require_numpy_backend`), each op
reading `op.bound_params` (an explicit `params` never reaches the blocks), and the MPC block's
tick latch, a side effect.

**Draft 1: the held values are state.** Lift the runtime's mutable fields into the carry:

```text
X_k = (x_computer, held, x_plant)          held = the committed signal buffer (the ZOH latches)
X_{k+1}, y_k = tick(X_k, u_k, k; p)        one base tick, pure
```

This is the textbook sampled-data model: a zero-order hold is a memory, so it is a state of the
discretized loop. It is R104's rule ("a sampled controller's memory is its state") applied to
the hold itself, and the tick latch goes the same way (the MPC's plan in its `StepSystem` state).

**Draft 2: firing as a select, not a branch.** Inside the tick, for each block with divisor `d`:
`fired = (k mod d == 0)`, then `x_i⁺ = where(fired, step_i(x_i, u_i, k, p_i), x_i)` and the same
for its held outputs. Pure, traceable, differentiable (the gradient flows through the branch
taken). Same semantics as today's double buffer, so `Computer.tick` is the test oracle: the pure
tick must match it byte for byte on NumPy.

**Draft 3: three ways to lay out the rates, by cost.**

| Layout | How | Cost | Fits |
| --- | --- | --- | --- |
| **Nested scan** (two clocks) | outer `scan` over controller periods; inner fixed RK4 substeps of the plant with the input held | no wasted work; what `HybridSimulator` does today in a Python loop | the common case: one controller, one plant (PID, MPC, MPPI, a neural policy) |
| **Base tick with selects** (any commensurate rates) | one `scan` over the base tick, Draft 2's `where` per block | slow blocks are computed and discarded on the ticks they skip | general multi-rate, small ratios |
| **Hyperperiod unrolled** | one step per hyperperiod `lcm(d_i)`; inside it the schedule is static Python, only fired blocks computed | no wasted work; the traced graph grows with `lcm(d_i)` | multi-rate with a short hyperperiod; an optimization of the second |

The nested scan first, as the kernel of the two-clock loop; the base tick next, for the general
`StepSchedule`; the hyperperiod as a later optimization.

**Draft 4: the plant between samples.** A fixed-step integrator (RK4 substeps, `stepping.py`'s
`step_body`) inside the tick: the sample times are known in advance (commensurate rates), so no
event detection is needed, which is what makes the loop pure. An adaptive solver
(`solve_ivp`) stays on the NumPy face; Diffrax is a separate question (QUESTIONS.md, S27).

**Draft 5: what stays out.** Rates that are not integer multiples of a base tick, asynchronous
or event-triggered clocks, jitter: the arbitrary multi-clock scheduling ROADMAP §7 puts out of
scope. The schedule's divisors, `dt_base` and the substep count are structural (fixed at
compile); the parameter tree is the loop's, every block named (R121).

**Open for the maintainer.**
1. *(The maintainer's direction, 2026-10-26: yes, §8.5.)* Is the pure tick of the closed loop also exposed as a `StepSystem` (it is one,
   mathematically: `X_{k+1} = F(X_k, k; p)`)? That would let the step tools read it, the
   linearization of R122 included (the sampled-data loop's stability as the eigenvalues of its
   map), but R13 and R120 rule the loop a simulation tool, not a `System`; a read-only view
   (`loop.as_step_map()`) would sit between the two.
2. Does the realtime and game mode keep a stateful runner over the pure tick (it must read the
   keyboard between ticks), as a thin shell rather than a second implementation?
3. The computational delay (a one-tick delay of the command, as on a real computer): a schedule
   option, or modelled by the user as a delay block?

### 8.2 Orchestrator or pure transcription: the options compared (draft, 2026-10-25)

The maintainer asked for the full comparison of keeping a clock orchestrator that calls each
block when its clock fires, against transcribing every rate into a pure step with conditional
updates and the held signals as state. Four options, because "keep the orchestrator" splits in
two:

- **O1 Orchestrator, stateful blocks** (today): `Computer.tick` mutates `k`, `x` and the signal
  buffers; `HybridSimulator` integrates the plant between ticks in a Python loop; a block may
  hold side effects (the MPC's tick latch, a host SciPy solve).
- **O2 Orchestrator over pure blocks**: the same scheduler, but every block is a pure
  `step(x, u, k, p)` (§8.1, R104), and the runner only keeps the clock and the buffers.
- **O3 Pure tick at the base rate**: one pure `tick(X, u, k; p)` with the holds in `X` and each
  block's update a select on `k mod d` (§8.1, Drafts 1–2).
- **O4 Pure structured transcription**: the schedule unrolled statically. A nested scan for two
  clocks (controller period outside, plant substeps inside), or one step per hyperperiod
  `lcm(d_i)` for several (a cyclic executive). No conditionals.

✓ good, ~ possible with a cost, ✗ not possible.

| Aspect | O1 Orchestrator, stateful | O2 Orchestrator, pure blocks | O3 Pure base tick, selects | O4 Pure structured |
| --- | --- | --- | --- | --- |
| **Deployed on a real computer (hard realtime)** | ✓ the natural shape of an RTOS: one task per rate, the slow task preempted by the fast one | ✓ same, and each block's code is a pure function to export (C, `jit`) | ~ every block runs every base tick unless compiled to real `if`s (C branches cheaply; JAX's `where` does not): worst-case time is the sum of all blocks | ~ a cyclic executive, classic in avionics: deterministic frames, but a slow block must fit inside one minor frame (no preemption) |
| **A slow block longer than a fast period** (an MPC at 50 Hz beside a 1 kHz loop) | ✓ by preemption, in execution | ✓ same | ✗ as execution; ✓ as semantics, with a one-period delay state on its output | ✗ as execution; ✓ as semantics, the same delay state |
| **Realtime in Python** (game mode, hardware in the loop: I/O between ticks) | ✓ today's shape | ✓ each block jitted, called by the host loop | ~ the pure tick jitted and called once per real tick (JAX dispatch about 10–50 µs a call) | ~ the same, one call per frame |
| **Simulation on NumPy** | ✓ skips idle blocks | ✓ same | ~ computes idle blocks and throws them away | ✓ no waste |
| **Simulation on JAX** | ✗ Python loop around a host runner | ~ one dispatch per block per tick: slow | ✓ one `lax.scan`, but idle blocks computed (worst for an expensive slow block like an MPC) | ✓ one `lax.scan`, no waste: the fastest |
| **Compile the computer side alone** (`jit`, C export) | ~ blocks one by one; the schedule stays Python | ✓ blocks one by one, and the schedule is plain data to generate a runner from | ✓ the computer's tick as one function | ✓ the computer's frame as one function |
| **Compile the whole closed loop** (plant inside) | ✗ the plant runs in `solve_ivp` or a host loop | ✗ the loop stays on the host | ✓ fixed-step plant substeps inside the tick | ✓ the same, substeps nested in each controller period |
| **Adaptive plant integration** (`solve_ivp`), events | ✓ | ✓ | ✗ fixed step only (Diffrax later, S27) | ✗ the same |
| **`vmap`** (Monte Carlo, families) | ✗ loop over trials | ✗ the same | ~ yes, but `lax.cond` becomes a select under `vmap`: idle blocks computed in every trial | ✓ clean |
| **Gradients of the closed-loop cost** | ✗ | ✗ through the loop (✓ per block) | ✓ through the select (the branch taken); memory per tick, `jax.checkpoint` per tick | ✓ best: memory per controller period, checkpoint per period |
| **Analysis tools on the loop** (R122's linearization, the stability of the sampled-data loop) | ✗ no single map | ✗ the same; per-block analysis only | ~ one map, but periodic in `k` (period `lcm(d_i)`): stability needs the product over a hyperperiod | ✓ one map per period, time-invariant: its eigenvalues are the loop's stability directly (the textbook `x⁺ = Φx + Γu` of a sampled-data loop) |
| **Host-only blocks** (SciPy or Ipopt MPC, hardware I/O, stateful drivers) | ✓ | ✓ through a block that calls out (no tracing) | ✗ (`pure_callback`, no gradient) | ✗ the same |
| **Asynchronous or non-commensurate rates, jitter** | ~ possible in principle (ROADMAP §7: out of scope) | ~ the same | ✗ | ✗ |
| **Debugging, introspection** | ✓ breakpoints, prints | ✓ | ~ jitted: harder (run on NumPy to debug) | ~ the same |
| **Reads like the textbook** | the engineer's view: tasks at rates | the same | the sampled-data model, with conditionals | the sampled-data model, cleanest (`Φ`, `Γ` per period) |
| **Constitution fit** (purity first; R13, R120: the loop a tool, not a `System`) | ✗ side effects in blocks | ✓ blocks pure, the runner a tool | ✓ pure; the map is a tool's internal, unless exposed (§8.1, open point 1) | ✓ the same |
| **Maintenance** | one runner | one runner plus pure blocks | one kernel | one kernel; the hyperperiod unrolling grows with `lcm(d_i)` |

**Reading the matrix.** No option wins every row: an orchestrator is the shape of *execution*
(a real computer, Python realtime, host-only blocks), a pure transcription is the shape of
*semantics* (what the loop computes, compiled, batched, differentiated, analysed). The
difference that matters most is the slow block: a real computer runs it across several fast
periods by preemption, which no pure step expresses as execution; the pure model expresses its
*effect* instead, a one-period delay on its output, which is also the honest model of what the
real computer does.

**The draft recommendation: both layers, one set of block functions.**
1. **Pure blocks everywhere** (O2's blocks): every block a pure `step(x, u, k, p)`, its memory
   in its state (R104), no latch. This is required by every other row and costs nothing.
2. **The pure loop is the reference model** for simulation, gradients, `vmap` and analysis: O4
   (the nested scan) for the two-clock loop first, O3 (base tick with selects) for a general
   `StepSchedule`, O4's hyperperiod as its optimization.
3. **The orchestrator stays as the execution shell** (O2): realtime and game mode, hardware in
   the loop, host-only blocks, the NumPy face; it calls the same pure block functions, so it is
   a runner, not a second model, and the pure loop is its test oracle (the same numbers on
   NumPy, byte for byte).
4. **Deployment on a real computer** generates its runner from the schedule (tasks per rate, or
   a cyclic executive) around the exported pure blocks; the computational delay of a slow block
   is in the model as a delay state, so simulation and hardware agree.

### 8.3 One model, two runners; the MPC's tick latch in a pure model (draft, 2026-10-26)

**One model, not two.** §8.2's recommendation keeps O2 and O4 side by side, but as two *runners*
of one description: the pure blocks plus the schedule (plain data). O4 transcribes it into one
pure map, with full JAX capability; O2 executes it on a host, without computing idle blocks.
Neither owns its own copy of a block's equations, and O4 is O2's test oracle.

**What O4 composes with.** O4 applied to the computer alone turns a multi-rate computer into one
single-rate step system at its frame period `lcm(d_i) · dt_base`, which joins the step algebra
(`StepDiagramSystem`) beside other step systems, a plant discretized with `discretize` included.
One limit: at the frame period the plant sees one input per frame, so a fast loop that must see
the plant inside the frame (a 1 kHz inner loop around a 50 Hz MPC) composes only if the plant is
inside the transcription (§8.1, Draft 4's substeps), not outside it at the frame rate. Whether
the transcribed map is exposed as a `StepSystem` is §8.1's open point 1 (R13, R120).

**The latch.** `MPCTickLatch` (`planning/trajectory_optimization/mpc/controller.py`) exists
because four functions want the same solve in one tick: the output ports `u_ff`, `x_ff` and `z`
(each fed through from `y`) and the block's `step`. It caches the solve by `(k, y)`: a hidden
cache, so the block is not pure, and a traced tick cannot use it. Four ways out, from the
simplest:

| Option | What changes | One solve per tick? | Cost |
| --- | --- | --- | --- |
| **A. Computational delay**: the solve in `step`, the outputs read from the state | `step` computes `z_{k+1}` from `y_k`; `u_ff` is the first input of `z_k`, so the outputs have no feedthrough from `y` | ✓ | the command applied at `k` comes from the measurement at `k − 1`: one period late, as on a real computer whose solve takes time (§8.2) |
| **B. One update returning both**: `update(x, u, k, p) → (x_next, y)`, the block's step and outputs from one call (a Mealy form, the shape of a `scan` body) | a new optional method on step blocks; the step plan calls it once and fills the ports and the state | ✓ | a change to the `StepSystem` contract: **[ask — core]** |
| **C. Rely on the compiler**: call the pure solve four times inside one traced tick | none | ✗ on NumPy; on JAX only if XLA merges the identical calls (not guaranteed through `while_loop` and `scan`) | fragile |
| **D. The solve as a port the step reads**: the step receives the block's own port values of the same tick | the step plan feeds a block's outputs to its own step | ✓ | the same contract question as B, in another form |

**Ruled 2026-10-26 (R123): A for the MPC.** The latch goes; the block solves in `step` and its
outputs read the state, one period late, as on a real computer. A needs no new contract, makes
the block pure, and is the delay state §8.2 already uses for a slow block. The MPC's closed-loop
numbers move by that one-period delay when it lands (with S31): the MPC baselines are refreshed
then, under R80's closeness check. B stays a study for a zero-delay output from an expensive
computation, a core contract change for the maintainer.

### 8.4 The end vision for this limited hybrid (the maintainer, 2026-10-26, draft)

> `computer @ plant` is fully supported as one discrete model on a base clock, the common
> denominator of all the computer's clocks. The computer exports as a pure step block; the
> plant exports as a step block on the same base clock, with internal substeps if needed. This
> limited kind of hybrid is then fully closed, as one step diagram. In parallel, a computer keeps
> another runner, with a clock, for realtime and faster simulation (as today), but every block
> inside is pure, unlike today's MPC with its cache. Whether closed-loop simulation keeps a
> specialised orchestrator or uses only the step-diagram runner remains to be seen.

**The pieces, and what exists.**

| Piece | What it is | Today |
| --- | --- | --- |
| The base clock | `dt_base` with every block firing on `k mod d_i = 0`, the rates integer multiples (the "limited" hybrid: commensurate clocks only) | ✓ `StepSchedule(dt_base, fire)`, `from_rates` refuses a rate that is not a whole divisor |
| The computer as a pure step block | its blocks, its held signals in its state (§8.1, Draft 1), firing as a select on `k` (Draft 2), one `step(X, u, k, p)` at `dt_base` | ✗ `Computer.tick` mutates its fields; NumPy only |
| The plant as a step block | `discretize(plant, dt_base, solver="rk4", substeps=n)`: the plant one base period later, its input held | ✓ `analysis/discretization.py` (and `"exact"` for a linear plant) |
| The closed loop as one step diagram | the two step blocks in a `StepDiagramSystem`: the computer reads `y_k = h(x_k)`, the plant steps over the period with `u_k` held, the zero-order hold of the textbook sampled-data loop | ✓ the step algebra exists (CONSTITUTION invariant 1); ✗ `computer @ plant` builds a `HybridDiagram`, not this |
| Every block pure | memory in state (R104), the MPC's latch gone (R123) | ✗ the MPC latch |
| The clock runner | the computer's realtime and fast-simulation runner, calling the same pure blocks (§8.2's O2) | ✓ `Computer.tick` as the shape; to make a runner over pure blocks |
| Closed-loop simulation: orchestrator or step diagram | open (below) | `HybridSimulator` today |

**Does it make sense: yes, with four points to keep in view.**

1. **It is R13's kernel, made explicit.** R13 already rules "a scan over clock ticks on the step
   model with the computer's step between samples"; the vision names its form: discretize the
   plant at the base clock, then everything is the step algebra. It refines R120's wording rather
   than reversing it: the *coupling* of a continuous plant and a computer stays a tool
   (`HybridDiagram`, the conversion), and what the conversion produces is an ordinary
   `StepDiagramSystem`, single-clock, a `System` of the step algebra. A sentence in CONSTITUTION
   invariant 1 or in DESIGN may be wanted to say so: the maintainer's call when the code lands.
2. **The base clock is the semantics; execution may be smarter.** At `dt_base`, a slow block is
   a select that computes and discards on the ticks it skips (§8.2's O3). That is the model; a
   runner may skip idle blocks (O2) or unroll the hyperperiod (O4) as optimizations with the same
   numbers.
3. **The fast loop must stay inside the base clock.** The plant is discretized at `dt_base`, the
   fastest rate, so a 1 kHz inner loop sees the plant every tick; discretizing the plant at a
   slower frame rate would lose it (§8.3).
4. **The plant's integration is fixed-step inside the step diagram.** An adaptive `solve_ivp`
   with tight tolerances stays with a runner on the NumPy face. This is the main argument for
   keeping a specialised closed-loop orchestrator.

**Closed-loop simulation: both paths, in R15's shape (agreed with the maintainer 2026-10-26,
draft).** The continuous side already settled the same question: R15 makes the fixed-step
compiled rollout the one engine and judge, and `evaluate(..., method="scipy")` swaps the run for
the adaptive `Simulator` "and nothing else". The hybrid loop follows it:

| Path | Role | What it reuses |
| --- | --- | --- |
| **The step-diagram runner** (the pure computer step plus `discretize(plant, dt_base, substeps=n)`) | the engine and the judge: `jit`, `grad`, `vmap`, analysis, `evaluate` | everything |
| **Adaptive hybrid simulation** (a `method="scipy"` swap) | only the plant's integration between samples changes: `solve_ivp` restarted at every sample, no event detection since the sample times are known | the same pure computer tick, the plant's `f` |
| **The clock runner** | execution, not simulation: realtime, game mode, hardware in the loop | the same pure blocks |

The adaptive path stays for stiff plants, a long sample period over fast plant dynamics, a dense
plant trajectory between samples for plots, teaching (`solve_ivp` as the reference integrator),
and as a cross-check of the substep count, the role `evaluate(method="scipy")` plays for a
continuous loop. It must not be a second model: today's `HybridSimulator`, with its own tick loop
and block handling, shrinks to "call the pure computer tick, integrate the plant over the
period".

**Open: the default of `compute_trajectory()` on a hybrid loop** (the maintainer, 2026-10-26: "it
remains to be seen"). The facts that bear on it:

- **The continuous rule** (`Simulator.select_solver`): SciPy adaptive on a smooth loop; Euler on a
  discontinuous one; fixed-step RK4 at a divisor of the period as soon as a block holds a signal
  over a sample period (DESIGN §4, the three periods). A sampled loop is exactly that case.
- **The evaluator's default** is the fixed-step engine (R15: `evaluate(..., method="rk4")`).
- **Today's hybrid default** is already fixed-step: `HybridDiagram.compute_trajectory` integrates
  the plant by RK4 substeps inside each base tick (`plant_dt_inner`).

**The resolution of the plant trajectory (the maintainer, 2026-10-26).** A step-diagram runner
emits one plant state per base tick, sparse next to today: `HybridSimulator` records every RK4
substep inside each tick (its `t_plant_parts`, `x_plant_parts`), so a 10 Hz MPC on a fast plant
plots smoothly. `Discretization.step` (`analysis/discretization.py`) computes the same substeps
(`step_body(..., substeps)`) and returns only `x_next`: the dense points are computed either way,
and the question is only what the runner records. Three ways, which give the same numbers:

| Option | How | Cost | Fits |
| --- | --- | --- | --- |
| **A. A finer base clock** | `dt_base` set at the plant's resolution; the computer fires every `d` ticks; the substeps become base ticks | none new (`StepSchedule` allows it); the carry recorded at every base tick, so more memory under `grad` | when the fine grid is wanted everywhere; RK4 over `dt_base` per tick is the same arithmetic as the substeps, so A and B agree |
| **B. Record the substeps** | the runner emits the plant's intermediate states as the scan's per-tick output, through a substep path on `Discretization` (`(x_next, xs_sub)`), internal to the runner | memory only, and only when asked; no change to the `StepSystem` contract | `compute_trajectory`: today's density by default |
| **C. Reconstruct after the run** | re-integrate each tick from the recorded `(x_k, u_k)`, a `vmap` over ticks, deterministic so the same numbers (as `reconstruct_internal_signals` does for a diagram's signals) | a cheap pass after the run; nothing in the main run or under `grad` | plots of an `evaluate` or a gradient run, whose main pass records per tick |

So resolution is decoupled from the computer's clock: `compute_trajectory` records the substeps
by default (B), as dense as today; `evaluate` and the gradient path record per tick and
reconstruct on request (C); a finer base clock (A) stays available.

**The alternative split (the maintainer, 2026-10-26): simulate with the orchestrator, analyse
with the step diagram.** `compute_trajectory` on a hybrid loop defaults to the clock orchestrator
over the pure blocks with an adaptive plant integration (`solve_ivp` between samples); the pure
step diagram serves analysis, composition, `evaluate`, gradients and `vmap`. (Today's default is
the orchestrator with fixed-step RK4 substeps, `plant_eval.rollout(x_plant, u_held, t_k, h)` per
tick: no adaptive option exists yet.)

| | Step diagram by default | Orchestrator + adaptive by default |
| --- | --- | --- |
| The continuous precedent | the `Simulator`'s rule for a held signal (fixed-step RK4 at a divisor of the period) and R15's engine | `compute_trajectory` → `Simulator` (adaptive on a smooth loop), `evaluate` → the engine: R15's split, applied literally |
| `compute_trajectory` and `evaluate` | the same numbers by default | differ by the integration error, as they already do on a smooth continuous loop |
| Accuracy | set by the substep count | set by a tolerance; no substep count to choose |
| Dense plant trajectory | recorded substeps (B above) | the solver's interpolant, natural |
| Stiff plants | need many substeps | handled (`scipy_stiff`) |
| Speed | one compiled scan on JAX; a Python loop on NumPy | a `solve_ivp` restart per held-input change (about 0.1–1 ms each): fine at a controller's rate, slow if restarted at a 1 kHz base tick |
| Hybrid numbers today | unchanged (R123's delay aside) | move once: the demos and the hybrid regression gates (`f.hybrid_zoh`, `f.dual_rate`) re-baselined |

**The agent's read: a sound split, on three conditions.** It is the continuous design applied
literally, and it puts each path where it is strongest: the adaptive orchestrator for the most
accurate reference plot, the step diagram for everything that composes, compiles or
differentiates.
1. **No second model:** the orchestrator calls the same pure computer tick (O2); only the plant's
   integration differs.
2. **Restart only when the plant's input changes:** `solve_ivp` runs from one change of the held
   plant input to the next (the instants a block feeding the plant fires), not at every base
   tick, so a fast base clock with a slow controller stays cheap. The restart is what makes an
   adaptive solver correct across the steps of a held input, which is why the `Simulator`
   avoids adaptive solvers on a loop with a held signal it does not restart at.
3. **A cross-check test:** the orchestrator's run and the step diagram's agree within the
   integration tolerance on the hybrid demos, the way `evaluate(method="scipy")` cross-checks a
   continuous loop.

Both defaults are defensible; the choice is a priority. Is `compute_trajectory` the most
accurate reference (the orchestrator), or the same numbers as `evaluate` and the gradients (the
step diagram)? The continuous side answers "most accurate" for a smooth loop. Not ruled.

The first draft recommendation was the step-diagram runner by default, with the substeps
chosen by the continuous rule (the largest divisor of `dt_base` at or under the plant's time
constant scaled by `AUTO_DT_SCALE`), the substeps recorded (B above) so the plot is as dense as
today's, and `solver="scipy"` for the adaptive path: it keeps the numbers and the resolution
today's default gives (R123's delay aside) and makes `compute_trajectory`, `evaluate` and the
gradients agree by default. Not ruled.

### 8.5 `computer @ plant` is a step diagram that knows it is hybrid (the maintainer, 2026-10-26, draft)

> The result of `computer @ plant` is considered to be the step diagram, at least for the
> composition algebra and the analysis shortcuts. Some methods know it is a hybrid system and
> use that for special operations: drawing the diagram, computing the trajectory, running the
> computer on its clock in realtime. By default it is just a regular step diagram.

**The shape.** `computer @ plant` returns a subclass of `StepDiagramSystem` (the name is open;
R13 keeps `HybridDiagram` for now) holding two blocks on the base clock, the computer as one pure
step block (§8.1) and `discretize(plant, dt_base, substeps=n)`, and keeping the continuous plant
and the schedule as its source, the way `Discretization` (`analysis/discretization.py`) is a
`StepSystem` that keeps its continuous `source`.

| Inherited as a step diagram (the algebra, unchanged) | Added or specialised because it knows it is hybrid |
| --- | --- |
| `step`, `compute_rollout`, `compile` (the step diagram's evaluator: `jit`, `scan`, `vmap`, `grad`) | `compute_trajectory()`: a trajectory in seconds, dense plant (§8.4), on the default chosen there (the step runner, or the orchestrator with an adaptive plant) |
| joining a larger `StepDiagramSystem` as a block | `plot_diagram()`: the two-side hybrid drawing (`graphical/diagrams/hybrid_topology.py`), computer and continuous plant, not two step boxes |
| the step tools: `jacobian`, and R122's linearization with its eigenvalues against the unit circle, the sampled-data loop's stability | the clock runner (§8.2's O2): realtime, game mode, hardware in the loop |
| the parameter tree, every block named (R121) | plots on the time axis: ticks converted to seconds with `dt_base` (R14: the loop converts where it meets the plant) |

**The agent's read: it works well, and it is the cleanest version of the vision.**

- **No method is redefined against its parent.** `StepSystem.compute_trajectory` raises today
  ("use `compute_rollout`"); the subclass adds a working one, so `compute_rollout` keeps the pure
  step meaning everywhere and `compute_trajectory` is the new, hybrid-aware verb. A tool that
  takes any step system never sees a different answer from it.
- **Its precedent is in the code.** `Discretization` is a `StepSystem` that knows its continuous
  source and reads its parameters through it.
- **It satisfies CONSTITUTION invariant 1's first sentence** ("everything that can be viewed as a
  system is a System") better than today's `HybridDiagram`, a plain class beside the algebra.
- **One limit, accepted:** placed inside a larger plain `StepDiagramSystem`, it is an ordinary
  block; its hybrid-aware verbs belong to the outermost loop that knows its clock, as a
  `Discretization` inside a diagram is just a step block.
- **The two runs must agree:** `compute_rollout` (the step semantics) and `compute_trajectory` (if
  its default is the adaptive orchestrator) differ by the integration error only; §8.4's
  cross-check test holds them together.

**What it changes in the rulings (for the maintainer, when ruled).** R13 ("the sampled loop is a
tool") and R120 (CONSTITUTION invariant 1: "the sampled loop (`Computer`, `HybridDiagram`) is a
simulation tool coupling the two clocks rather than an open `System` seam") say the opposite of
"the result is a step diagram". The reconciliation: the *coupling* of a continuous plant and a
computer stays a tool, the conversion (`discretize` at the base clock plus the computer's pure
step); its *result* is a `System` of the step algebra. That needs a CONSTITUTION amendment of
invariant 1's second sentence, maintainer-owned, when this lands.

**Order:** the continuous core it waits for landed on 2026-10-24 (the step model, then V1's
steps 1–6); the loop comes in v0.4 (R86), with P10's AC-8b; the hand-copied `HybridDiagram`
facades go with it; the hybrid topology drawing (`graphical/diagrams/hybrid_topology.py`)
stays for teaching.

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
