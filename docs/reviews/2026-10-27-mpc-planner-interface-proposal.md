# The MPC and its planner: one interface for every planner (2026-10-27, proposal)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Status: a proposal; recorded, as revised in the same session, as [R148](2026-10-27-core-vision-decisions.md)** (the planner compiled to a pure face with a solution vector `z` and `policy(z, x, t)`; §3.1's `Plan`, `guess` and policy template, and §3.3's details, are superseded there).
**Originally: a proposal, not a ruling.** Decision 9 of
[the consolidated core vision](2026-10-27-consolidated-core-vision.md) §8, studied after the review
session of the same day ([R141–R147](2026-10-27-core-vision-decisions.md)) changed the interface
at every new case the maintainer raised. This study fixes the frame first, lists every use case,
weighs every candidate interface, and was attacked by an independent reviewer before it was
written; each of the reviewer's findings is either fixed here or named as out of the contract.

**Bound by:** R135 (the planner does not know it runs in a loop; the MPC owns the loop
mechanics; shifting is the planner's knowledge), R123 (the default applies at `k` the command
computed from the measurement at `k − 1`; no hidden cache), R104 (memory is state; draws from a
seed and the tick), R130–R131 (a program takes the problem's tree; a new leaf value never
retraces; sizes structural), R141 (continuous and step problems), R144 (`x_start` a leaf),
R145–R146 (`%`, `SampledSystem`, periods on blocks).

## 1. The use cases

| | Case | | Case |
| --- | --- | --- | --- |
| U1 | trajopt MPC on Ipopt, host only | U13 | model mismatch: the MPC's plant vs the real plant, one tree |
| U2 | native fixed-iteration MPC, its closed loop one scan, `grad` wrt a cost weight, `vmap` | U14 | Monte Carlo over the loop |
| U3 | adaptive MPC: estimated `x̂`, mass, friction fed in each tick | U15 | realtime, hardware in the loop |
| U4 | a moving reference on a port; a time-varying cost | U16 | a failed solve |
| U5 | MPPI: a seed per tick, the shifted nominal input as warm start | U17–U18 | terminal ingredients, constraints moving per tick |
| U6 | RRT as the replanner: random, no program | U19 | logging and animating each tick's plan |
| U7 | value iteration as the replanner: the plan is a feedback law | U20 | warm data beyond the decision (multipliers, MPPI's `U`) |
| U8 | LQR re-designed each tick | U21 | a shrinking horizon to a fixed final time |
| U9 | an RL planner as the replanner (absurd, but it should mean something) | U22 | a predictor compensating the delay (`x̂` and `t` one period ahead) |
| U10 | a future pure discrete-time planner on a `StepPlanningProblem` | U23 | event-triggered replanning |
| U11 | dual rate: slow planning, fast application of the plan | U24 | the one-liner `(ModelPredictiveController(planner) % 0.05) @ plant` |
| U12 | two forms: the pure law, the delayed law (R123) | U25 | an MPC whose output is a reference for an inner PID (`x_ref`) |

## 2. The candidates, and why five of them fail

| Candidate | Fails on |
| --- | --- |
| A. Four verbs `solve(params, t, z0)`, `shift`, `unpack`, `program` | U6, U7, U9 (no `z`); time as an argument cannot reach `J(z, params)` |
| B. D27's `decision_dimension`, `prepare_online`, `warm_start_guess(z_prev, y, dt_mpc, k)` | R135 (loop-aware); U6, U7 |
| C. Two verbs, the MPC carrying the `PlanningSolution` itself | a solution holds Python objects (a live `problem`, a closure `cost_to_go`, tables as attributes; `results.py`, `dp.py:536`, `blocks/sources.py:317`) and `solve` mutates the planner (`last_solution`, caches): it cannot be state or trace |
| C′. C as three blocks `Replan → UnitDelay → Execute` | a solve in a block's output runs once per read: the compiled step diagram computes the port signals in `step` and again in `outputs` (`core/compile/evaluators/discrete.py:186-201`), the option R123 rejected; the tree paths become `controller.replan.cost.Q`; the delay's `x0` needs a whole plan at tick 0 |
| E. Planner-specific code in the MPC | R135, RULES (no probing) |

What all five miss: **the MPC needs a pure core and a fixed-structure plan, and the planner owns
both.** `solve` and `PlanningSolution` stay the user's verb and record, built on that core.

## 3. The proposal

### 3.1 The planner: a pure core under the user's `solve`

| Member | What it is | Who reads it |
| --- | --- | --- |
| `params` | the planner's tree: its problem's leaves (`x_start`, `t_start`, `plant`, `cost`, the sets, `discount_rate`) and its method's numbers under `"method"` (a learning rate, MPPI's temperature), with `seed` where it draws | everyone; the MPC's tree is exactly this (U2 tunes `controller.cost.Q` or `controller.method.lr`) |
| `plan(params=None, *, warm=None) -> Plan` | the solve as a function: pure where the algorithm is (native fixed-iteration trajopt, JAX value iteration, MPPI on its seed); no mutation of the planner | the MPC, every tick |
| `Plan` | a fixed-structure pytree of arrays: `policy` (the parameters of the policy template), `t_start` (when it was solved from), `success`, `warm` (the planner's own warm data: `z`, multipliers, MPPI's `U`, value iteration's `J`), `prediction` (`t, x, u` at the knots, where the planner has one) | the MPC holds it as state |
| `policy` | the policy template: a stateless `System` (`n = 0`, checked) whose `params` a plan fills; `u = policy.h(x, ·, t, params=plan.policy)`. A trajectory source for trajopt, MPPI and RRT; a lookup law for value iteration; a gain for LQR; a network for RL | the MPC applies it; a fast follower applies it (U11) |
| `guess(params=None) -> Plan` | the plan before any solve: the initial guess, `success = False`; it gives the plan's structure without solving | the MPC's state at tick 0 |
| `solve(params=None, *, warm=None) -> PlanningSolution` | today's user verb: `plan`, then the policy filled, the trajectory and the evaluation, on the host | users, unchanged |
| `program` | the parametric `MathematicalProgram`, where the planner has one | export and tuning; outside the MPC contract |

- **Shifting is inside `plan`** (R135): the planner reads `warm.t_start` and `params["t_start"]`
  and moves its own warm data by the difference, on its own knots. Knots and the sample period
  stay independent.
- **The start:** a planner pins the start from the `x_start` leaf; `X0` is derived from it.
  Today shooting and collocation read `X0` first (`shooting.py:288`, `direct_collocation.py:412`),
  and the MPC overwrites both (`trajectory_optimization/planner.py:668`); that becomes one rule.
- **Fixed structure:** every planner declares a plan of fixed size. RRT resamples its path to `N`
  knots and stays host-only (its algorithm is not pure).

### 3.2 Time, in the problem

- A problem gains `t_start`; the horizon is `[t_start, t_start + tf]`, and `t_start = 0`
  reproduces today.
- **The discount and the horizon count from `t_start`**, while costs, sets and references read
  absolute time: `J = ∫_{t_s}^{t_s+T} e^{−ρ(t − t_s)} g(x, u, t) dt + e^{−ρT} h`. Otherwise a
  discounted cost is scaled by `e^{−ρ t_s}` (at `ρ = 0.1`, `t = 100 s`, `4.5e−5`), which defeats the
  solver's tolerances (`problems.py:83` writes it on absolute time today).
- On a step problem `t_start` is the tick offset, an integer. **Integer leaves are allowed, never
  in `wrt`** (R131 already says integers never in `wrt`); this amends R118's "integer seeds only"
  and drops the "seed is the one integer leaf" special case.

### 3.3 The MPC: one block, two forms

```python
mpc = ModelPredictiveController(planner)                       # the delayed form (R123), the default
mpc = ModelPredictiveController(planner, delay=False)          # the pure law
mpc = ModelPredictiveController(planner, inputs={"x_start": "x_hat", "plant.m": "m_hat",
                                                 "plant.mu": "mu_hat"})   # U3: bindings
loop = (mpc % 0.05) @ plant                                    # U24
```

- **Bindings.** `inputs` maps leaves of the planner's tree to the block's input ports. The
  default is `{"x_start": "y", "t_start": "t"}`. A bound leaf stays in the tree as the value used
  when its port is not wired, so the tree's shape never depends on the wiring. The `seed` leaf,
  when the planner has one, is drawn from the block's own seed and the tick (R104).
- **Time is a port, `t`, wired by convention** (automation-by-convention, a new standard name):
  the loop's clock, seconds in a sampled or continuous loop, the tick in a pure step loop. The MPC
  never knows `Δ`. Rewiring it is the predictor of U22 (`t + Δ` with `x̂(t + Δ)`).
- **The delayed form is one `StepSystem`** (R123, option A of `sampled-path.md` §8.3): its state is
  the plan (`x0 = planner.guess()`); `step` runs `plan(params ⊕ bindings, warm=state)` and keeps
  the previous plan when `success` is false (U16); `h` applies the stored plan's policy to the
  current `x̂` at `t`, or the plant's nominal input before the first success. **The plan is one
  period late; applying it is instantaneous**: for an open-loop plan `u_k` depends on `y_{k−1}`
  only, for a feedback plan (value iteration, LQR) the law also reads `x̂_k`. One solve per tick,
  no cache.
- **The pure form is one static block:** `h` runs `plan(..., warm=None)` and applies it, same tick,
  no memory, no warm start. **Its cost:** a static block's output is computed wherever it is read,
  and the step evaluators compute the signals twice per tick today. The pure form therefore
  needs **one signal pass per tick** in the step evaluators (an internal change, which also helps
  any expensive static block); until then it solves twice on the host, with the same result,
  because its draws come from the seed and the time.
- **Ports out:** `u`; `plan` (the flat plan, for dual rate and logging, U11, U19); `x_ref` and
  `u_ref` read off the prediction, for planners that have one (U25; today's `u_ff`, `x_ff` and `z`,
  `mpc/controller.py:974-995`).
- **The tree:** `pe.params["controller"]` is `planner.params`: `controller.cost.Q`,
  `controller.plant.m` (the mismatch study, U13), `controller.method.lr`.
- **Dual rate (U11):** `(mpc % 0.05) >> (PlanFollower(planner.policy) % 0.01)`, the follower
  applying the template with its parameters read from the `plan` port.

### 3.4 What `ModelPredictiveController` drops

`dt_mpc`, `t0`, `warm_start`, `MPCTickLatch`, the stateless and stateful class pair,
`export_to_computer`, `dt_broadcast`, `Command`, `replan_divisor`.

### 3.5 Each use case

U1 the delayed block on the host runner; U2 traced, `grad` wrt `controller.cost.Q` or
`controller.method.lr`; U3 bindings, `(estimator % 0.01) >> (mpc % 0.05)`; U4 a reference port
bound to a cost leaf, or `t_start`; U5 the seed and the warm `U` inside the plan; U6 host only, the
path resampled; U7 traced on JAX value iteration, the policy a lookup law; U8 the meaning is the
planner's; U9 host; U10 the same contract, `t` the tick; U11 the follower; U12 `delay=`; U13–U14
the tree, `vmap`; U15 the same blocks on the host runner; U16 the previous plan kept; U17–U18 bound
leaves of cost and set parameters (sets reading `params`, the core's §1); U19 the `plan` port; U20
the plan's `warm`; U22 the `t` port rewired with a predictor; U23 a select in `step` (a later
option); U24 the one-liner; U25 `x_ref`.

## 4. Out of the contract, named

- **U21, a shrinking horizon:** `N` is structural. A later option keeps `N` and makes `tf` a leaf
  of planners with a fixed knot count; it conflicts with `N = tf/dt` of the compiled step face
  (R141), so it is not part of v0.3.
- **Scenario and robust MPC:** a problem tree holding laws is the planner's business (it expands
  them into its own parameters); tube MPC binds `x_start` to the previous plan's prediction.
- **Binding part of a start** (`x_start` of one block of a diagram plant): syntax later.
- **RRT traced:** never; its algorithm is not pure.

## 5. The cost of this proposal

1. Every planner gains `plan`, `policy`, `guess` and `params`, a pure core with no mutation;
   `solve` is rebuilt on it. Trajopt and MPPI first (the MPC's consumers), the others as they are
   wrapped.
2. Every policy keeps its plan in `params` (R140's row: `TrajectorySource` and `LookupPolicy` keep
   their tables as attributes today).
3. One standard port name, `t`, wired by convention.
4. One signal pass per tick in the step evaluators, for the pure form.
5. `t_start` on problems, the discount counted from it, integer leaves allowed.
