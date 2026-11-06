# Plans as policies: one face for every planner, two horizons, the cost-to-go as a field

Status: written 2026-11-06 from the maintainer's design session; the rulings of that session
are §8 (`PP-D1` to `PP-D12`). Lane: core. The rows: [TODO.md](TODO.md) §1.8 ("Plans as
policies"); the order of work and each step's "done when" live there. Contracts in code today:
[DESIGN.md](../dev/DESIGN.md) §4 ("Fields") and §6 (the compiled face, the MPC).

**Why this doc.** The compiled face of the LQR and value-iteration planners was a v0.4
leftover. Building it showed that value iteration has no clean finite horizon, that the MPC
assumes every plan is open loop, and that no planner returns its cost-to-go as the `Field` the
constitution names. Settled together, the three give the interface every future planner fits:
a plan is a policy, finite or infinite in time, with its cost-to-go a field.

**Where each idea lives** (one fact, one home):

| Idea | Home |
| --- | --- |
| The face contract on two axes, every planner's place on it (§3) | this doc until it lands, then DESIGN §6 |
| Value iteration's two modes, its pure backup (§4) | this doc, then DESIGN §6 ("RRT / DP") |
| LQR's pure core and face (§5) | this doc, then DESIGN §6 |
| `GridField` and the planners' cost-to-go as fields (§6) | this doc, then DESIGN §4 ("Fields") |
| The MPC reading the state (§7) | this doc, then DESIGN §6 ("MPC on the planner's face") |
| The rest of the field object (`CallableField`, `LinearApproximator`, `PolicyEvaluator`, tabular RL's `Q`) | TODO §4, v0.5 (unchanged, shaped by §6.4) |

---

## 1. The goal

One interface that any planner fits, checked by one test: every planner closes in the MPC
loop, and in the dual-rate loop, without a special case.

1. **A plan is a policy.** Whatever a planner computes, its plan `z` gives the command at a
   state and a time, `u = command(z, x, t)`. An open-loop plan ignores `x`; a feedback plan
   reads it. The offline solution's law and the face's command are the same function of `z`.
2. **Two horizons, one algorithm.** A planner reads the problem's horizon: finite, the plan
   varies in time over `[t0, tf]`; infinite, the plan is stationary. LQR already does this
   (the differential or the algebraic Riccati equation); value iteration gets it.
3. **The core math first.** Each planner's file opens with a pure function holding the
   textbook equation, arrays and the parameter tree in, arrays out. The planner and the face
   both call it; neither holds a second copy of the algorithm.
4. **The cost-to-go is a field.** `J(x)` or `J(x, t)` is the constitution's `Field`: it traces,
   takes `params`, and converts to a set or a cost like every field.

## 2. The state of the code (2026-11-06)

- **The face contract already gives the command the state.** DESIGN §6 writes
  `pv.command(z, x, t)`; the MPC's `PlanCommand` and `PlanFollower` pass `x = None`, because
  every face built so far (trajectory optimization, MPPI) is open loop.
- **Replanning every `M` ticks already exists.** The dual-rate loop
  `(mpc % slow) >> (PlanFollower(planner) % fast)` solves at the slow rate and applies the
  last plan at the fast one.
- **Value iteration has one mode.** `solve()` sweeps to convergence whatever the problem's
  horizon; its terminal time is read from the horizon but its sweep count is not.
  `solve_steps(n)` runs `n` sweeps and keeps only the last table, a stationary law. The
  GRO860 notebooks teach that a finite-horizon policy is non-stationary; the library does not
  deliver one. Every notebook and demo runs value iteration on an infinite problem; only unit
  tests solve a finite one with `solve()`.
- **Each sweep's `(J_k, π_k)` pair is consistent:** `π_k` is the best action with `k` steps left
  and `J_k` its cost. Nothing in the numbers needs correcting.
- **No planner returns a field.** LQR's `cost_to_go` is a closure that copies
  `QuadraticField`'s math and calls `float()`; value iteration's is SciPy interpolation
  (`fill_value=0`: a state past the grid costs nothing); RL's wraps the critic. None traces,
  takes `params`, or offers `as_terminal_cost` or `as_set`.
- **The math sits in methods that unpack `self`.** `LQRPlanner.solve` holds the linearization
  and the Riccati solve inline; `value_iteration` holds the sweep loop and calls
  `backward_step_table`, which reads `self.grid`, `self.gamma`, `self.interpolation`.

## 3. The contract: two axes, one face

A plan sits on two independent axes:

- **law form:** open loop, `u(t)`, or feedback, `u(x, t)`;
- **time:** stationary (an infinite-horizon problem) or time-varying over `[t0, tf]`.

| Face method | Rule, every planner |
| --- | --- |
| `z`, `pv.n_z` | one flat array: the law's numbers, plus what `solve` warm-starts from, plus the origin `t0` when the plan varies in time |
| `pv.params` | the planner's tree (a copy), as today |
| `pv.initial(params)` | the default `z` at the tree |
| `pv.solve(params=None, z0=None) -> (z, status)` | a function of the tree; a method that cannot warm-start ignores `z0` |
| `pv.command(z, x, t)` | always called with the state; an open-loop plan ignores `x`, a stationary plan ignores `t` |
| past the plan's end | the last stage holds |
| `pv.shift(z, t, x0=None)` | identity for a stationary plan; a time-varying plan re-based to the tick (`t0 ← t`) |
| `pv.prediction(z, t)` | only on a plan that carries its states (`pv.predicts`) |
| `pv.solution(z, params, status)` | the host-side `PlanningSolution`, byte-identical to `planner.solve()` at the same tree |

The face keeps no last solution (RULES 2.4) and compiles once (DESIGN §4 rule 6). Every
planner on the two axes:

| Planner | Law | Time | `z` | `command(z, x, t)` | `shift` | Built |
| --- | --- | --- | --- | --- | --- | --- |
| Trajectory optimization | open loop | varying (finite only) | knots, `t0` | `u(t)` | knots re-read at the tick | yes |
| MPPI | open loop | varying (finite only) | input sequence, `t0` | `u(t)` | first input dropped, last held | yes |
| LQR, infinite | feedback | stationary | `K`, `x̄`, `ū` | `ū − K (x − x̄)` | identity | §5 |
| LQR, finite | feedback | varying | `t0`, `K(t)`, `x̄`, `ū` | `ū − K(t − t0) (x − x̄)` | re-based | §5 |
| Value iteration, infinite | feedback | stationary | `J`, `π` | `π(x)` | identity | §4 |
| Value iteration, finite | feedback | varying | `t0`, the stacks `J_k`, `π_k` | `π_k(x)`, `k` the stage of `t` | re-based | §4 |
| Tabular RL | feedback | stationary | the `Q` table | the greedy action of `Q` at `x` | identity | v0.5 |
| Neural RL | feedback | stationary | the weights | the network at `x` | identity | v0.5 |
| RRT, RRT* (never traced) | open loop | varying | path resampled, `t0` | `u(t)` | the passed part dropped | v0.5 |

**The test of the contract.** Each row closes in `ModelPredictiveController(planner) % dt` and
in the dual-rate loop: the command block passes the measurement and the tick's time. A
time-varying feedback plan holds its whole schedule, so a shrinking-horizon loop applies it
without re-solving, and a receding one re-solves with `t0` at the tick: both through `shift`
and `t0`, no special case. An infinite problem under trajectory optimization stays the
classic receding finite plan (the problem's horizon is the plan's).

## 4. Value iteration: one backup, two modes

### 4.1 The pure core, first in `dp.py`

```python
def bellman_backup(J_next, G, x_next, levels, gamma, method="linear"):
    """One Bellman backup on a grid: J(x) = min_u [ c(x, u) + γ J_next(x_next) ], with π(x) its argmin."""
    # The cost-to-go one step ahead at every successor, read on the grid
    J_arrival = grid_interpolation(levels, J_next, x_next, method)

    # Q(x, u) = c(x, u) + γ J_next(x_next), for every (node, action) pair at once
    Q = G + gamma * J_arrival

    # The best action at every node
    J = Q.min(axis=1)
    pi = Q.argmin(axis=1)

    return J, pi
```

The table step's body, lifted out with its numbers unchanged; the JAX engine's `step` already
has this shape (`step(J_next, G, coords, in_bounds, gamma)`) and reads the same. The per-node
double loop (`backend="loop"`, the reference written for teaching) stays a method: it reads the
plant one pair at a time. Whether the backup reads the grid through its levels (the shared
interpolation of §6.2) or through the grid's own SciPy interpolator is decided by the
byte-identity of the sweeps (§9, open point 1).

### 4.2 The loop, visible in the planner

`value_iteration` keeps the sweep loop, the stopping rule, the progress printing and the
history; it calls `bellman_backup` once per sweep. The pure function is one backup, not the
loop: the loop is where the workflow lives (`verbose`, `record_history`), and it reads as the
textbook recursion.

### 4.3 The two modes, chosen by the problem

| | Finite horizon | Infinite horizon |
| --- | --- | --- |
| Problem | `horizon=T` (or a step problem's `n_steps`) | `horizon=inf` or unset |
| Sweeps | exactly `N = T / dt`; refused when `T` is not a multiple of `dt` | until `max |J − J_next| < tol`, at most `max_iterations` |
| Kept | every `J_k` and `π_k`, ordered forward in time (`k = 0` at `t0`) | the last `J`, `π` |
| Law | `π_k(x)`, held over the stage `[t_k, t_k + dt)`, the last stage past `tf` | `π(x)` |
| Cost-to-go | `J(x, t)`, a `GridField` with a time axis (§6) | `J(x)`, a `GridField` |
| JAX | one `lax.scan` over `N` sweeps, stacking the tables | today's `lax.while_loop` |
| Record | the mode, the sweeps, the stacks in `tables` | as today |

`solve()` reads `problem.horizon_kind()`, as LQR does. `solve_steps(n)` stays, documented for
what it is: the **receding `n`-step law**, `n` sweeps from the terminal cost and the last
table used as a stationary law (`n` steps always left). The cart-pole and double-pendulum
demos rely on it to simulate past the horizon. The memory of the finite mode is
`N × nodes` for each stack; a solve whose stacks exceed a limit is refused with the numbers
(§9, open point 2).

### 4.4 The lookup law gains a schedule

`LookupPolicy(grid, pi, t=None)`: `pi` one table, or a stack `(N, nodes)` on the stage times
`t`, read held per stage (the stage of `t`, the last one past the end). Its `params` gain the
times: `{"u_table", "levels", "t"}`, `u_table` then `(N, *grid_shape, m)`. One class, no new
name (the stationary law is the schedule of one stage).

### 4.5 The face

`DynamicProgrammingPlanner.compile()` gives `pv` on the contract of §3:

- `z`: infinite, `[J ; π]` flat (`π` as floats, exact below `2⁵³`), `n_z = 2 · nodes`; finite,
  `[t0 ; J_0 … J_N ; π_0 … π_{N−1}]`. The policy stays in `z`: rebuilding it from `J` costs a
  full backup and needs the model, and a pure command cannot cache it (LQR keeps `K` in `z`
  for the same reason). `J` remains the solution in the textbook's sense, and becomes the
  `GridField` of §6.
- `pv.solve(params, z0)`: the grid tables (successors, running costs) are built at
  compilation, at the default tree. A tree that changes the plant's numbers rebuilds them on
  the host (refused under a trace, with the leaf named); any other tree reuses them. The
  infinite mode warm-starts from `z0`'s `J`; the finite mode always sweeps from the terminal
  cost.
- `pv.initial(params)`: `J` the terminal cost, `π` zeros.
- `pv.command(z, x, t)`: the lookup law of `z` at `x` (finite: at the stage of `t`), the
  same arithmetic as `LookupPolicy.action`.
- A fixed number of sweeps per solve (the real-time dynamic programming of an MPC tick,
  warm-started from the last `J`): a planner keyword, `n_iterations=k` (RULES 2.7: an exact
  count), beside the cap `max_iterations`.

## 5. LQR: the pure core and its face

### 5.1 The pure core, first in `lqr.py`

```python
def lqr_gains(sys, params, *, horizon=None, n_steps=1001, method="auto", eps=1e-6):
    """The LQR law of the plant linearized at the tree's operating point: K and P, or the schedule (t, K(t), S(t))."""
    x_bar = params["method"]["x_bar"]
    u_bar = params["method"]["u_bar"]
    t = params["method"]["t"]

    # A = ∂f/∂x and B = ∂f/∂u at (x̄, ū, t), B over the command port
    model = design_model(sys, x_bar, u_bar, t, params["plant"], method=method, eps=eps)
    A = model.A()
    B = model.B()

    # The weights over the command port
    Q = params["cost"]["Q"]
    R = command_block(sys, params["cost"]["R"])
    S_f = params["cost"]["S"]

    if horizon is None:
        # Aᵀ P + P A − P B R⁻¹ Bᵀ P + Q = 0, and K = R⁻¹ Bᵀ P
        P = care(A, B, Q, R)
        K = np.linalg.solve(R, B.T @ P)
        return K, P

    # −Ṡ = Aᵀ S + S A − S B R⁻¹ Bᵀ S + Q, swept backward from S(t_f) = S_f, and K(t) = R⁻¹ Bᵀ S(t)
    return lqr_gain_schedule(A, B, Q, R, S_f, horizon, n_steps=n_steps)
```

`LQRPlanner.solve` calls it at its own tree, then builds the law, the poles, the record and the
cost-to-go; the face calls it at the solve's tree. The textbook lines are those of
`LQRPlanner.solve` today, unchanged.

### 5.2 The face

- `z`: infinite, `[K ; x̄ ; ū]` flat; finite, `[t0 ; K(t) ; x̄ ; ū]` on the schedule's
  relative times.
- `pv.solve(params)`: `lqr_gains` at the tree (milliseconds on the host; the method warm-starts
  from nothing, `z0` ignored).
- `pv.command(z, x, t)`: `ū − K (x − x̄)`, or `ū − K(t − t0) (x − x̄)` read on the schedule
  and held past its end (the `TimeVaryingFeedback` arithmetic).
- `pv.solution(z, params)`: the law, the record (`P` or `S(t)` and the poles at the tree) and
  the cost-to-go, `QuadraticField(x̄, P)` or `QuadraticField(x̄, S, t)` (§6).

## 6. The cost-to-go as a field

### 6.1 What a field is (DESIGN §4, landed)

A `Field` is a scalar function on the model's arguments, `V(x, u, t; p)`: `value` (pure,
NumPy or JAX, `params` a tree), `gradient`, and four conversions named by what they return
(`as_set`, `as_input_set`, `as_cost`, `as_terminal_cost`). `QuadraticField(c, P, t=None)`
reads a schedule linearly in `t`, held past its ends. The field is what lets one object plug
into every tool that takes a set or a cost: `QuadraticField(x̄, S).as_terminal_cost()` already
ends a short MPC with the LQR value (`examples/demos/compile/terminal_field_mpc.py`).

### 6.2 `GridField`, in `core/fields.py`

```python
class GridField(Field):
    """J(x), or J(x, t), read on a regular grid: multilinear between nodes, the edge held outside."""

    def __init__(self, levels, values, t=None, *, method="linear", outside=None): ...

    def value(self, x, u=None, t=0.0, params=None):
        # x held inside the grid on each axis (or `outside` past it, when a number is given)
        # J(x) = Σ_corners J_corner Πₖ (yₖ or 1 − yₖ); a time axis read linearly in t
        ...
```

- **Its place:** `core/fields.py`, beside `QuadraticField`. It needs the levels per axis and the
  table, never the `StateSpaceGrid`; the planner builds it from its grid (`grid.field(J)`).
- **Its arithmetic:** one native interpolation, multilinear and nearest, through `xp`, shared
  with `LookupPolicy` (one function, *a table on a regular grid read at `x`*). It traces, takes
  `params` (`levels`, `values`, `t`), and its gradient is `jax.grad` or central differences.
  The spline methods (`"cubic"`, `"quintic"`) stay NumPy-only, as today.
- **Outside the grid:** the edge held (`x` clamped to the grid on each axis); `outside=` a
  number when a user asks for one. Today's `J = 0` past the grid goes.
- **Time:** a leading axis of `values` on the times `t`, read linearly in `t` and held past its
  ends, the rule of `QuadraticField` (every field reads time the same way). The lookup law
  holds per stage (§4.4): the law and the field each follow their own meaning.
- **Its numbers:** the sweeps keep their own interpolation, so the tables stay byte-identical.
  `cost_to_go(x)` read natively may differ from SciPy's in the last bits (about `1e-16`), and
  a query past the grid now returns the edge value: each change gets its own test.

### 6.3 The planners' cost-to-go

- Value iteration: `GridField(levels, J)`, or `GridField(levels, J_stack, t=t_k)` on a finite
  horizon.
- LQR: `QuadraticField(x̄, P)`, or `QuadraticField(x̄, S, t)`; `quadratic_cost_to_go` goes.
- `sol.cost_to_go(x)` keeps working (`Field.__call__`), and the plots read the field.

What it opens:

```python
J = DynamicProgrammingPlanner(problem, ...).solve().cost_to_go
J(x); J(x, t)                    # as today; the finite horizon
J.as_terminal_cost()             # the VI value ends a short nonlinear MPC
J.as_set(upper=INF - 1)          # the states that reach the goal: the capture basin
J.gradient(x)                    # ∂J/∂x
```

### 6.4 The `Q` table, later

A table over the grid's states × actions is a table on a joint grid: `Q` is a `GridField` over
the joint levels of `(x, u)`, with `min_over_u()` (the `J` field) and `greedy()` (the
`LookupPolicy`) its two reductions. They land with tabular RL in v0.5; nothing about `Q` is
built here, and `GridField` needs no special shape for it.

## 7. The MPC reads the state

`PlanCommand` and `PlanFollower` gain the measurement as an input and pass it to
`pv.command(z, x̂, t)`. The trajectory and MPPI faces ignore it, so their loops stay
byte-identical. With it:

- LQR and value-iteration plans close in `ModelPredictiveController(planner) % dt`;
- "replan every `M` ticks" is the dual-rate loop, `(mpc % (M dt)) >> (PlanFollower(planner) % dt)`,
  a feedback plan applied at the fast rate between solves; no new option;
- value iteration with `n_iterations=k` per tick is real-time dynamic programming, warm-started
  from the last `J` (the MPC tick binds `x0` and `t0`, which never touch the grid tables, so the
  sweeps trace with no rebuild).

## 8. The rulings of 2026-11-06

- `PP-D1` **The contract** of §3: two axes (open loop or feedback; stationary or
  time-varying); `command(z, x, t)` always receives the state; a time-varying `z` carries `t0`;
  the last stage holds past the end; the test is that every planner closes in the MPC loop.
- `PP-D2` **Value iteration supports both horizons**, chosen by the problem (§4.3); the finite
  horizon (formerly S63, v0.5) moves to this work.
- `PP-D3` **The policy stays in `z`** (§4.5).
- `PP-D4` **`solve_steps(n)` stays,** documented as the receding `n`-step law.
- `PP-D5` **`LookupPolicy` takes a schedule** for the finite horizon (§4.4).
- `PP-D6` **The core math first:** a pure function at the top of each planner's file,
  `bellman_backup` (one backup; the loop stays in the planner, with the workflow) and
  `lqr_gains`; the planner and the face both call it.
- `PP-D7` **The MPC reads the state** (§7); replanning every `M` ticks is the dual-rate loop.
- `PP-D8` **Every planner's cost-to-go is a `Field`:** LQR and value iteration in this work, RL
  with the rest of the field object in v0.5.
- `PP-D9` **`GridField(levels, values, t=None)` in `core/fields.py`,** on one native
  interpolation shared with the lookup law.
- `PP-D10` **Outside the grid, the edge is held,** `outside=` a number on request.
- `PP-D11` **A `Q` table is a `GridField` over `(x, u)`;** its reductions land with tabular RL
  in v0.5.
- `PP-D12` **A grid field reads time linearly,** held past its ends; the law holds per stage.

## 9. Open points (details, the agent's call unless the maintainer says otherwise)

1. **The backup's interpolation.** If the shared native interpolation reproduces SciPy's
   linear one bit for bit inside the grid, `bellman_backup` uses it on every backend; if not,
   the sweeps keep the grid's SciPy interpolator (their tables byte-identical) and only the
   field and the law read natively.
2. **The finite mode's memory limit.** The default refusal threshold (proposed: 1 GB for the two
   stacks, the message giving `N`, `nodes` and the bytes), and whether `record_history=False`
   lets a user keep `π_k` only.
3. **The LQR schedule's time origin.** `lqr_gain_schedule` samples `0 … tf`, not `t0 … t0 + T`:
   the planner and the face read it relative to `t0` (as the `z` of §5.2 does), and the
   finite `QuadraticField(x̄, S, t)` is built on absolute times.
4. **`n_iterations` on value iteration.** The spelling of the fixed sweep count per face solve
   (RULES 2.7's exact count); whether the offline `solve()` takes it too.
5. **A finite horizon that is not a multiple of `dt`.** Refused (proposed), or rounded with a
   warning.

## 10. Steps and the split

Local steps; the order of work and "done when" are TODO §1.8's.

**First change: contract, pure cores, fields** (no number moves but the two field changes of
§6.2):
- `PP-1` The contract in DESIGN §6 (§3's two tables).
- `PP-2` `bellman_backup` and `lqr_gains` lifted out, the planners calling them: seeded
  baselines byte-identical.
- `PP-3` The shared grid interpolation; `GridField` in `core/fields.py`; value iteration's and
  LQR's `cost_to_go` as fields; `quadratic_cost_to_go` removed.

**Second change: the two modes, the faces, the MPC:**
- `PP-4` Value iteration's finite mode: the stacks, `LookupPolicy`'s schedule, `J(x, t)`, the
  JAX `lax.scan`, the record, `solve_steps` documented; tests on the grid world's tables
  checked by hand (GRO860's notebook holds them).
- `PP-5` The two compiled faces: `pv.solution(pv.solve(p)[0])` byte-identical to
  `planner.solve()` in both modes, NumPy and JAX; `test_no_recompile.py` extended.
- `PP-6` The MPC reads the state: LQR, value iteration (both modes) in the MPC loop and the
  dual-rate loop; trajectory and MPPI loops byte-identical.
- `PP-7` DESIGN §4 and §6, ROADMAP's ledger rows, TODO; this doc deleted once landed.
