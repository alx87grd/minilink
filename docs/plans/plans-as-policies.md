# Plans as policies: the faces of LQR and value iteration, and the MPC reading the state

Status: ruled 2026-11-06 (the maintainer's design session, then two independent reviews; no
decision open). Three changes, ordered by value; the first two landed in v0.4 and their
contracts are DESIGN's: the pure cores, `GridField` and the planners' cost-to-go as fields,
discounted LQR (DESIGN §4, "Fields", and §6), value iteration's finite horizon and the law past
the horizon for both planners (DESIGN §6). This doc holds the design of the third: the compiled
faces of LQR and value iteration and the MPC protocol that reads the state. Lane: core. The row
and its "done when": [TODO.md](TODO.md) §1.8. It is additive (no public name breaks, the
open-loop loops byte-identical), so it may land after the `0.4.0` tag.

## 1. The goal

One interface every planner fits, checked by one conformance test: every planner's face closes
in the dual-rate loop, and every face whose `z` fits a tick closes in the MPC loop, without a
special case (a finite value-iteration plan, whose `z` is the stacks, closes in the follower
only, §2).

- **A plan is a policy.** Whatever a planner computes, its plan `z` gives the command at a
  state and a time, `u = command(z, x, t)`. An open-loop plan ignores `x`; a feedback plan
  reads it. The offline solution's controller and the face's command are the same function of
  `z`.
- **What the conformance test is not.** Closing LQR or value iteration in the MPC loop proves
  the contract; it is not how they are used. Their solve does not read the measured state (`x0`
  enters neither the Riccati equation nor the grid tables), so re-solving every tick recomputes
  the same plan. Their use stays the offline one, `solution.controller @ plant`, or a plan
  refreshed at a slow rate when the problem's numbers move (the dual-rate loop, §5).

**The code today.** DESIGN §6 writes `pv.command(z, x, t)`; the MPC (`PlanCommand`) and
`PlanFollower` pass `x = None`, because the two faces built so far (trajectory optimization,
MPPI) are open loop; both append `t0` last in `z`. The dual-rate loop
`(mpc % slow) >> (PlanFollower(planner) % fast)` solves at the slow rate and applies the last
plan at the fast one; `>>` wires one output to one input (`z`), so a follower has no
measurement. LQR and value iteration have no compiled face.

## 2. The contract: two axes, one face

A plan sits on two independent axes:

- **law form:** open loop, `u(t)`, or feedback, `u(x, t)`;
- **time:** stationary (an infinite-horizon problem) or time-varying over `[t0, tf]`.

A command is deterministic: a stochastic law takes its noise from a noise block, never from
the face. Discrete-time LQR on step problems is out of scope here.

| Face method | Rule, every planner |
| --- | --- |
| `z`, `pv.n_z` | one flat array: the law's numbers, plus what `solve` warm-starts from, plus the origin `t0` **last** when the plan varies in time |
| `pv.feedback` | whether `command` reads the state; the MPC and the follower add a measurement input only when it is true |
| `pv.params` | the planner's tree (a copy), as today |
| `pv.initial(params)` | a usable default plan at the tree (never a corner command) |
| `pv.solve(params=None, z0=None) -> (z, status)` | a function of the tree; a method that cannot warm-start ignores `z0`; `status["cost"]` is the plan's cost from `x0` |
| `pv.command(z, x, t)` | always called with the state; an open-loop plan ignores `x`, a stationary plan ignores `t` |
| past the plan's end | a feedback plan continues with its stationary law at its final point (`PP-D14`, as the solution's controller does today); an open-loop plan holds its last input |
| `pv.shift(z, t, x0=None)` | the warm guess for the next solve: the trajectory faces re-read their knots at the tick and hold the tail; a method that ignores `z0` returns `z` unchanged |
| `pv.prediction(z, t)` | only on a plan that carries its states (`pv.predicts`) |
| `pv.solution(z, params, status)` | the host-side `PlanningSolution`, equal to `planner.solve()` at the same tree on the fields of §6 |

The face keeps no last solution (RULES 2.4) and compiles once (DESIGN §4 rule 6). Every
planner on the two axes:

| Planner | Law | Time | `z` | `command(z, x, t)` | Face |
| --- | --- | --- | --- | --- | --- |
| Trajectory optimization | open loop | varying (finite only) | knots, `t0` | `u(t)` | built |
| MPPI | open loop | varying (finite only) | input sequence, `t0` | `u(t)` | built |
| Trajectory plus tracking (`lqr_trajectory`) | feedback | varying | knots, `K(t)`, `K∞` at `x_d(tf)`, `t0` | `u_d(t) − K(t)(x − x_d(t))`; the stationary law at the goal past `tf` | later, same contract |
| LQR, infinite | feedback | stationary | `K`, `x̄`, `ū` | `ū − K (x − x̄)` | §4 |
| LQR, finite | feedback | varying | `K(t)`, `K∞`, `x̄`, `ū`, `t0` | `ū − K(t − t0)(x − x̄)`; `K∞` past `tf` | §4 |
| Value iteration, infinite | feedback | stationary | `J`, the command table `u` | the lookup of `u` at `x` | §3 |
| Value iteration, finite | feedback | varying | the stacks `J_k`, `u_k`, `t0` | the lookup of `u_k`, `k` the stage of `t` | §3 |
| Tabular RL | feedback | stationary | the `Q` table | the greedy action of `Q` at `x` | v0.5 |
| Neural RL | feedback | stationary | the weights | the network at `x` | v0.5 |
| RRT, RRT* (never traced) | open loop | varying | path resampled, `t0` | `u(t)` | v0.5 |

**The test of the contract.** Each built row closes in `ModelPredictiveController(planner) % dt`
and in the dual-rate loop, the command reading the state bound to `x0` (§5). Every MPC tick is
receding: `t0` is the tick and the horizon a length from it. A finite value-iteration plan is
refused in the MPC loop (its stacks are `(2N + 1)` grid tables per tick, recomputed to the same
numbers) and closes in the follower only; an infinite one is refused above a size limit
(`PP-Q6`, §8).

## 3. Value iteration's face

`DynamicProgrammingPlanner.compile()` gives `pv` on the contract of §2, `pv.feedback = True`:

- **`z`:** infinite, `[J ; u_table]` flat, the command table the law reads (the
  `LookupPolicy`'s `u_table`, `nodes · m` numbers, never action ids as floats); finite,
  `[J_0 … J_N ; u_0 … u_{N−1} ; t0]`.
- **`pv.solve(params, z0)`:** the grid tables (successors, running costs, terminal cost) are
  built at compilation, at the default tree. A tree that changes any leaf the tables read
  (the plant, the cost, `X`, `U`, `gamma`, `out_of_bound_cost`) rebuilds them on the host,
  refused under a trace with the leaf named; `x0` and `t0` never rebuild them (the sweeps'
  times shift with `t0`, under the frozen-tables rule of DESIGN §6). The infinite mode
  warm-starts from `z0`'s `J`; the finite mode sweeps from the terminal cost.
- **`pv.initial(params)`:** `J` the terminal cost, the command table at the input nearest `ū`
  (never action `0`, the corner of the input box).
- **`pv.command(z, x, t)`:** the lookup law's arithmetic on `z`'s table at `x` (finite: at the
  stage of `t`, the stage rule of `LookupPolicy`).
- **`status`:** `{"success", "n_iterations", "delta", "cost"}`, `cost = J(x0)`.
- **A fixed number of sweeps per solve,** in the infinite mode only: truncated, warm-started
  value iteration (not "real-time dynamic programming", which backs up states asynchronously
  along simulated paths). Its spelling is `PP-Q8`.

## 4. LQR's face

- **`z`:** infinite, `[K ; x̄ ; ū]`; finite, `[K(s) ; K∞ ; x̄ ; ū ; t0]` on the relative times
  `s`, `K∞` the stationary gain at the final operating point (`stationary_gain`, the
  `K_after` slot of `TimeVaryingFeedback`).
- **`pv.solve(params)`:** `lqr_matrices` and the Riccati solve at the tree, on the host (SciPy:
  refused under a trace, with the message naming the host loop); `z0` ignored; `status["cost"]
  = (x0 − x̄)ᵀ P (x0 − x̄)` (or `S(t0)`).
- **`pv.initial(params)`:** the solve at the tree (a direct method has no other guess).
- **`pv.command(z, x, t)`:** `ū − K (x − x̄)`, or `ū − K(t − t0)(x − x̄)` read on the schedule,
  `ū − K∞ (x − x̄)` past `tf`.
- **`pv.solution(z, params)`:** the controller, the record (`P` or `S(t)`, the poles at the
  tree) and the cost-to-go, `QuadraticField(x̄, P)` or `QuadraticField(x̄, S, t)`.

## 5. The MPC and the follower read the state

- **Which state:** the command of a feedback face reads the port bound to `x0` (the default
  binding `{"x0": "y"}`, the measurement). An MPC on a feedback face refuses to build without an
  `x0` binding. A predictor bound to `x0` feeds the solve; the command reads the same port, so a
  user who wants the law on the raw measurement binds it there and puts the predictor on the
  solve's own port (the worked snippet goes in DESIGN with `PP-7`).
- **With `delay=True`:** the delayed plan meets the current measurement (the law of the plan
  solved at `k − 1`, applied to `x̂_k`).
- **The follower:** `PlanFollower` gains a measurement input `y` only when `pv.feedback` is
  true, so every open-loop dual-rate loop keeps its wiring and its numbers. `>>` wires `z`; the
  follower's `y` is closed by `@` on the plant like the MPC's (the snippet and its test land
  before the faces: `examples/projects/mpc/mpc_dual_rate.py`, `test_mpc.py`, `test_v03_core.py`
  and `benchmarks/scenarios/f_mpc_parity.py` keep passing unchanged). A follower on a feedback
  face whose `y` is left open refuses to run rather than read a nominal state.
- **"Replan every `M` ticks"** is that dual-rate loop: the plan solved at the slow rate, the
  feedback law applied at the fast one; no new option.

## 6. What "equal to `planner.solve()`" means

`pv.solution(pv.solve(p)[0], p, status)` and `planner.solve(params=p)` are compared, per
backend, on: the controller's `params` (value iteration's `u_table`, `levels`, `t`; LQR's `K`,
`x̄`, `ū`, `t`), the record's numbers (`iterations`, `delta`, `converged`, `mode`, the tables or
`P`, `S`, the poles), and the cost-to-go's `params`. A run's history and its timings are
workflow, outside the comparison.

## 7. The `Q` table, later

A table over the grid's states × actions is a table on a joint grid: `Q` is a `GridField` over
the joint levels of `(x, u)`, with `min_over_u()` (the `J` field) and `greedy()` (the
`LookupPolicy`) its two reductions (`PP-D11`). They land with tabular RL in v0.5 (TODO §4, the
rest of the field object), and tabular RL's face reads them.

## 8. Steps and the agent's choices

Local steps; the order of work and the "done when" are TODO §1.8's.

- `PP-7` The MPC and the follower read the state (`pv.feedback`, the `x0` binding, the
  follower's `y`); the dual-rate snippet and its test, before the faces.
- `PP-8` The two compiled faces: §6's comparison in both modes, NumPy and JAX;
  `test_no_recompile.py` extended; the conformance loops of §1.
- `PP-9` The contract of §2, §5 and §6 into DESIGN §6 (written when it lands, DESIGN holding
  landed contracts only); ROADMAP's ledger rows; this doc deleted.

The agent's to settle, recorded here:

- `PP-Q6` **Size limits:** a face's `n_z` refused above a limit (proposed: 10⁶ numbers in the
  MPC loop; the finite stacks already refuse above 1 GiB), the message giving `N`, the nodes
  and the bytes. Finite value iteration refused in the MPC loop, accepted in the follower.
- `PP-Q8` **The fixed sweep count of a face solve** (infinite mode only): `n_iterations=k` on
  the planner (RULES 2.7's exact count), beside `max_iterations` the cap; refused in the finite
  mode, where `N` is the horizon's.
- `PP-Q7` **The transition-matrix form of the arrival** (`J_arrival = W J_next`, the
  interpolation weights `W` built once since the successors never move): a later speed-up of
  value iteration, adopted only if the tables stay byte-identical or the maintainer accepts
  the move.

## 9. The rulings that shape this change (2026-11-06)

The rulings `PP-D2`, `PP-D4` to `PP-D6`, `PP-D8` to `PP-D10`, `PP-D12`, `PP-D13`, `PP-D15` and
`PP-D16` landed with changes 1 and 2 (their contracts are DESIGN §4 "Fields" and §6); `PP-D17`
(the GRO860 notebooks unchanged, their wording revisited when the course is next taught) is a
note in TODO §1.8. Those that shape the faces:

- `PP-D1` **The contract** of §2: two axes (open loop or feedback; stationary or
  time-varying); `command(z, x, t)` always receives the state; a time-varying `z` carries `t0`;
  the test is that every planner closes in the dual-rate loop, and in the MPC loop when its
  `z` fits a tick (§1).
- `PP-D3` **The policy stays in `z`** (as the command table, §3).
- `PP-D7` **The MPC reads the state** (§5); replanning at a slow rate is the dual-rate loop.
- `PP-D11` **A `Q` table is a `GridField` over `(x, u)`;** its reductions land with tabular RL
  in v0.5 (§7).
- `PP-D14` **Past its horizon, a feedback plan applies its stationary law at its final point:**
  LQR, `K∞` at the final operating point (`x̄`; `x_d(tf)`, `u_d(tf)` for tracking), carried in
  `z`; value iteration, stage 0, the full-horizon law; open-loop plans hold their last input.
