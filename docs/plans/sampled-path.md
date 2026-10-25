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
1. Is the pure tick of the closed loop also exposed as a `StepSystem` (it is one,
   mathematically: `X_{k+1} = F(X_k, k; p)`)? That would let the step tools read it, the
   linearization of R122 included (the sampled-data loop's stability as the eigenvalues of its
   map), but R13 and R120 rule the loop a simulation tool, not a `System`; a read-only view
   (`loop.as_step_map()`) would sit between the two.
2. Does the realtime and game mode keep a stateful runner over the pure tick (it must read the
   keyboard between ticks), as a thin shell rather than a second implementation?
3. The computational delay (a one-tick delay of the command, as on a real computer): a schedule
   option, or modelled by the user as a delay block?

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
