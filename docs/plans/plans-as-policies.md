# Plans as policies: one face for every planner, two horizons, the cost-to-go as a field

Status: written 2026-11-06 from the maintainer's design session (rulings `PP-D1` to `PP-D12`,
§9), then consolidated the same day after two independent reviews (one on the code and the
contracts, one on the control theory and the user API); what the reviews changed is §11; the
decisions they raised were ruled the same day (`PP-D13` to `PP-D17`), and none is open. On
2026-11-06 the steps were re-sliced into three changes ordered by value (§12); change 1 (the
math and the fields) is built on the branch `plans-as-policies-1` (§13). Lane: core.
The rows: [TODO.md](TODO.md) §1.8, three rows, one per change; the order of work and each
change's "done when" live there. Contracts in code today:
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
| LQR's lowering and face (§5) | this doc, then DESIGN §6 |
| `GridField` and the planners' cost-to-go as fields (§6) | this doc, then DESIGN §4 ("Fields") |
| The MPC and the follower reading the state (§7) | this doc, then DESIGN §6 ("MPC on the planner's face") |
| The rest of the field object (`CallableField`, `LinearApproximator`, `PolicyEvaluator`, tabular RL's `Q`) | TODO §4, v0.5 (shaped by §6.5) |

---

## 1. The goal

One interface that any planner fits, checked by one conformance test: every planner's face
closes in the dual-rate loop, and every face whose `z` fits a tick closes in the MPC loop,
without a special case (a finite value-iteration plan, whose `z` is the stacks, closes in the
follower only, §3).

1. **A plan is a policy.** Whatever a planner computes, its plan `z` gives the command at a
   state and a time, `u = command(z, x, t)`. An open-loop plan ignores `x`; a feedback plan
   reads it. The offline solution's law and the face's command are the same function of `z`.
2. **Two horizons, one algorithm.** A planner reads the problem's horizon: finite, the plan
   varies in time over `[t0, tf]`; infinite, the plan is stationary. LQR already does this
   (the differential or the algebraic Riccati equation); value iteration gets it.
3. **The core math first.** Each planner's file opens with the pure function that holds its
   textbook equation; the planner and the face both call it; neither holds a second copy.
4. **The cost-to-go is a field.** `J(x)` or `J(x, t)` is the constitution's `Field`: it traces,
   takes `params`, and converts to a set or a cost like every field.

**What the conformance test is not.** Closing LQR or value iteration in the MPC loop proves
the contract; it is not how they are used. An LQR or value-iteration solve does not read the
measured state (`x0` enters neither the Riccati equation nor the grid tables), so re-solving
every tick recomputes the same plan. Their use stays the offline one,
`solution.policy @ plant`, or a plan refreshed at a slow rate when the problem's numbers move
(the dual-rate loop, §7).

## 2. The state of the code (2026-11-06)

- **The face contract already gives the command the state.** DESIGN §6 writes
  `pv.command(z, x, t)`; the MPC's `PlanCommand` and `PlanFollower` pass `x = None`, because
  every face built so far (trajectory optimization, MPPI) is open loop. Both existing faces
  append `t0` last in `z` (`z[-1]`).
- **Replanning at a slow rate already exists.** The dual-rate loop
  `(mpc % slow) >> (PlanFollower(planner) % fast)` solves at the slow rate and applies the last
  plan at the fast one. `>>` wires exactly one output to one input (`z`), so a follower has no
  measurement today.
- **Value iteration has one mode.** `solve()` sweeps to convergence whatever the problem's
  horizon; its terminal time is read from the horizon (`final_time = t0 + horizon`) but its
  sweep count is not. `solve_steps(n)` runs `n` sweeps and keeps the last table, a stationary
  law; its record says `fixed_horizon=True`. The GRO860 notebooks teach that a finite-horizon
  policy is non-stationary and map their hand-written `exact_dp(N)` to `solve_steps(N)`.
- **Three student-facing examples solve value iteration on a finite-horizon problem:** the
  README's comparison of planners (`README.md`, `examples/demos/core/readme_examples.py`:
  `horizon=4.0`, the loop then simulated to `tf=10`), `examples/tutorial/showcase_minilink.ipynb`
  (`horizon=4.0`) and `examples/tutorial/advanced/02_optimal_control.ipynb` (`horizon=5.0`). The
  horizon is there for the trajectory planners on the same problem; value iteration ignores it
  and returns the converged stationary law. The GRO860 notebooks and the value-iteration demos
  run infinite problems.
- **Each sweep's `(J_k, π_k)` pair is consistent:** `π_k` is the best action with `k` steps left
  and `J_k` its cost. Nothing in the numbers needs correcting.
- **Off the grid, four readers disagree.** The sweeps price a successor that leaves the grid as
  inadmissible (`G` holds the failure cost) and read its arrival cost as `0` (SciPy
  `fill_value=0`; JAX `jnp.where(in_bounds, …, 0)`); value iteration's `cost_to_go` returns
  `0` past the grid; `LookupPolicy` extends its edge cells and clips `u`.
- **No planner returns a field.** LQR's `cost_to_go` is a closure that copies
  `QuadraticField`'s math and calls `float()`; value iteration's is SciPy interpolation; RL's
  wraps the critic. None traces, takes `params`, or offers `as_terminal_cost` or `as_set`.
- **LQR ignores two numbers of the problem.** `lqr.py` never reads `discount_rate` (a
  discounted problem gets undiscounted gains), and the finite schedule runs over `0 … T`
  whatever `t0` is.
- **Where the math sits.** `LQRPlanner.linear_model()` linearizes and `solve` holds the Riccati
  solve; `care`, `lqr_gain` and `lqr_gain_schedule` (`control/design.py`,
  `analysis/riccati.py`) are already pure. Value iteration's `backward_step_table` reads
  `self.grid`, `self.gamma`, `self.interpolation`; the JAX engine's `step` has the pure shape;
  the `"loop"` backend is the reference double loop written for teaching.

## 3. The contract: two axes, one face

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
| past the plan's end | a feedback plan continues with its stationary law at its final point (`PP-D14`); an open-loop plan holds its last input |
| `pv.shift(z, t, x0=None)` | the warm guess for the next solve: the trajectory faces re-read their knots at the tick and hold the tail; a method that ignores `z0` returns `z` unchanged |
| `pv.prediction(z, t)` | only on a plan that carries its states (`pv.predicts`) |
| `pv.solution(z, params, status)` | the host-side `PlanningSolution`, equal to `planner.solve()` at the same tree on the fields of §8 |

The face keeps no last solution (RULES 2.4) and compiles once (DESIGN §4 rule 6). Every
planner on the two axes:

| Planner | Law | Time | `z` | `command(z, x, t)` | Built |
| --- | --- | --- | --- | --- | --- |
| Trajectory optimization | open loop | varying (finite only) | knots, `t0` | `u(t)` | yes |
| MPPI | open loop | varying (finite only) | input sequence, `t0` | `u(t)` | yes |
| Trajectory plus tracking (`trajectory_lqr`) | feedback | varying | knots, `K(t)`, `K∞` at `x_d(tf)`, `t0` | `u_d(t) − K(t)(x − x_d(t))`; the stationary law at the goal past `tf` | later, same contract |
| LQR, infinite | feedback | stationary | `K`, `x̄`, `ū` | `ū − K (x − x̄)` | §5 |
| LQR, finite | feedback | varying | `K(t)`, `K∞`, `x̄`, `ū`, `t0` | `ū − K(t − t0)(x − x̄)`; `K∞` past `tf` | §5 |
| Value iteration, infinite | feedback | stationary | `J`, the command table `u` | the lookup of `u` at `x` | §4 |
| Value iteration, finite | feedback | varying | the stacks `J_k`, `u_k`, `t0` | the lookup of `u_k`, `k` the stage of `t` | §4 |
| Tabular RL | feedback | stationary | the `Q` table | the greedy action of `Q` at `x` | v0.5 |
| Neural RL | feedback | stationary | the weights | the network at `x` | v0.5 |
| RRT, RRT* (never traced) | open loop | varying | path resampled, `t0` | `u(t)` | v0.5 |

**The test of the contract.** Each built row closes in `ModelPredictiveController(planner) % dt`
and in the dual-rate loop, the command reading the state bound to `x0` (§7). Every MPC tick is
receding: `t0` is the tick and the horizon a length from it. A finite value-iteration plan is
refused in the MPC loop (its stacks are `(2N + 1)` grid tables per tick, recomputed to the same
numbers) and closes in the follower only; an infinite one is refused above a size limit
(§10, `PP-Q6`).

## 4. Value iteration: one backup, two modes

### 4.1 The backup, in each engine's body

Ruled 2026-11-07 (`PP-D6` amended): the backup is written in the body of the NumPy table step
(`backward_step_table`) and of the JAX step (`dp_jax.py`), not in a helper; the textbook lines
are the lesson (RULES 5.3):

```python
# The cost-to-go one step ahead at every pair's successor (zero off the grid)
J_arrival = grid.interpolate(J_next, x_next.reshape(-1, n), method).reshape(N, A)

# Q(x, u) = c(x, u) + γ J_next(x_next)
Q = G + gamma * J_arrival

# J(x) = min_u Q(x, u), π(x) = argmin_u Q(x, u)
J = Q.min(axis=1)
pi = Q.argmin(axis=1)
```

The arrival reads `J_next` at the successors of every pair, shape `(nodes, actions)`, zero off
the grid: on NumPy the grid's SciPy reader, on JAX `map_coordinates` with its `in_bounds` mask,
inline the same way. The `"loop"`
backend stays the reference double loop written for teaching, the one deliberate second
spelling. The transition-matrix form, `J_arrival = W J_next` with the interpolation weights
`W` built once (the successors never move), is a later speed-up of `arrival` alone (§10,
`PP-Q7`).

### 4.2 The loop, visible in the planner

`value_iteration` keeps the sweep loop, the stopping rule, the progress printing and the
history; it runs one backup per sweep. The engine's step is one backup, not the
loop: the loop is where the workflow lives (`verbose`, `record_history`), and it reads as the
textbook recursion.

### 4.3 The two modes

| | Finite horizon | Infinite horizon |
| --- | --- | --- |
| Problem | `horizon=T`, or a step problem's `n_steps` | `horizon=inf` or unset |
| Sweeps | `N = steps_of(T, dt)` (the library's rule: refused unless `dt` divides `T`); a step problem's `N = n_steps` | until `max |J − J_next| < tol`, at most `max_iterations` |
| Kept | the stacks, forward in time: `J_0 … J_N` (`J_N = h`) and `u_0 … u_{N−1}`, stage `i` at `t_i = t0 + i dt` | the last `J` and its law |
| Law | `u_i(x)` for `t ∈ [t_i, t_{i+1})`; past `tf`, `u_0(x)`, the full-horizon law (`PP-D14`: the stationary law at the end, approximated without a second solve, documented as such) | the lookup of `u` |
| Stage of `t` | `i = round((t − t0) / dt)` taken down when `t` falls short of `t_i` beyond a tolerance (no `floor` on float ticks); a step problem's tick difference | — |
| Cost-to-go | `J(x, t)`, the value discounted to `t` (the problem's from `t0` is `γ^i J_i`) | `J(x)` |
| JAX | one `lax.scan(reverse=True)` over `N` sweeps, stacking the tables | today's `lax.while_loop` |
| Record | `mode="finite"`, `N`, the stacks in `tables` | `mode="infinite"` (ruled 2026-11-07: the mode names the horizon, `converged` the outcome); `tol` documented as a change per sweep, not an error bound on `J` (with `γ < 1` the bound is `γ·tol/(1 − γ)`; with `γ = 1` none) |

Symbols: `i` is the stage counted forward from `t0`; `k` stays the sweep count (steps left),
`i = N − k`. The stacks are the per-sweep history, reversed: they are the solution's single
home, and `record_history` stays the workflow switch of the infinite mode's animation.

- **How `solve()` picks the mode:** the problem's horizon (`PP-D2`); the three examples of §2
  are migrated in the same change, each saying its intent (`PP-D13`).
- **`solve_steps(n)`** stays, documented for what it is: the receding `n`-step law, `n`
  sweeps from the terminal cost and the stage-0 table used as a stationary law (Bertsekas'
  receding horizon). Its record says `mode="receding"` (`fixed_horizon` goes, it would clash
  with the finite mode). A test ties the two: the finite mode's `J_0` equals `solve_steps(N)`'s
  `J`, and its `u_i` equals `solve_steps(N − i)`'s law.
- **The failure cost on a finite horizon** is the cost bound over `N` steps, not the geometric
  limit (`grid_failure_cost` takes `N`).
- **Tables frozen at `tf`.** A precomputed grid builds the running-cost table once, at `tf`; a
  finite solve on a problem whose `f` or `g` reads `t` needs `precompute=False`, and the record
  says which tables were frozen.
- **Memory:** `N × nodes` per stack, refused above a limit with the numbers (`PP-Q6`).

### 4.4 The lookup law gains a schedule

`LookupPolicy(grid, pi, t=None)`: `pi` one table, or a stack `(N, nodes)` on the stage times
`t`, held per stage (the stage rule of §4.3). Its `params` gain the times: `{"u_table",
"levels", "t"}`, `u_table` then `(N, *grid_shape, m)`. One class, no new name. Outside the grid
it keeps its own rule (edge cells extended, `u` clipped to the input box): a law and a field
treat the outside differently on purpose (§6.3).

### 4.5 The face

`DynamicProgrammingPlanner.compile()` gives `pv` on the contract of §3, `pv.feedback = True`:

- **`z`:** infinite, `[J ; u_table]` flat, the command table the law reads (the
  `LookupPolicy`'s `u_table`, `nodes · m` numbers, never action ids as floats); finite,
  `[J_0 … J_N ; u_0 … u_{N−1} ; t0]`.
- **`pv.solve(params, z0)`:** the grid tables (successors, running costs, terminal cost) are
  built at compilation, at the default tree. A tree that changes any leaf the tables read
  (the plant, the cost, `X`, `U`, `gamma`, `out_of_bound_cost`) rebuilds them on the host,
  refused under a trace with the leaf named; `x0` and `t0` never rebuild them (the sweeps'
  times shift with `t0`, under the frozen-tables rule of §4.3). The infinite mode warm-starts
  from `z0`'s `J`; the finite mode sweeps from the terminal cost.
- **`pv.initial(params)`:** `J` the terminal cost, the command table at the input nearest `ū`
  (never action `0`, the corner of the input box).
- **`pv.command(z, x, t)`:** the lookup law's arithmetic on `z`'s table at `x` (finite: at the
  stage of `t`).
- **`status`:** `{"success", "iterations", "delta", "cost"}`, `cost = J(x0)`.
- **A fixed number of sweeps per solve,** in the infinite mode only: truncated, warm-started
  value iteration (not "real-time dynamic programming", which backs up states asynchronously
  along simulated paths). Its spelling is `PP-Q8`.

## 5. LQR: the lowering first, the existing pure solvers, the face

### 5.1 The pure core, first in `lqr.py` (landed with change 1)

The Riccati solvers are already pure and live in `control/design.py` (`lqr_gain`,
`lqr_gain_schedule`) and `analysis/riccati.py` (`care`); a second spelling of them would be a
second copy. What `lqr.py` adds is the lowering of the problem to the textbook matrices, so its
file opens with that, arrays and numbers in, arrays out (the planner passes its operating
point, its cost's weights and the plant's design numbers; the face will pass the tree's):

```python
def lqr_matrices(sys, x_bar, u_bar, Q, R, S, *, t=0.0, params=None, method="auto", eps=1e-6):
    """(A, B, Q, R, S): a quadratic problem lowered to the textbook LQR matrices at (x̄, ū, t)."""
    # A = ∂f/∂x and B = ∂f/∂u at (x̄, ū, t), B over the command port alone
    model = design_model(sys, x_bar, u_bar, t, params, method=method, eps=eps)
    A = model.A()
    B = model.B()

    # The input weight over the command port: R[c, c]
    R = command_block(sys, R)

    return A, B, Q, R, S
```

`LQRPlanner.solve` calls it, then the existing solvers, with the textbook lines of the solve
unchanged (`P = care(A_ρ, B, Q, R)`, `K = R⁻¹ Bᵀ P`, or the backward sweep of `S(t)`).

### 5.2 Two numbers LQR must read

- **The discount rate** (`PP-D16`, landed with change 1): on `e^{−ρt}`-discounted costs the
  planner solves the textbook equivalent, the undiscounted problem on `A_ρ = A − (ρ/2) I`
  (the finite sweep too), and warns that the problem is discounted (unusual for LQR). The
  shift sits in `solve`, where the Riccati equations read `A_ρ`, not in `lqr_matrices`: the
  closed-loop poles in the record are the plant's, `eig(A − B K)`. The
  equivalence holds as the problem weighs its terms: the running cost by `e^{−ρ(t − t0)}` and
  the terminal cost by `e^{−ρT}` (`problems.py`'s cost bound reads it so); with
  `x̃ = e^{−ρ(t − t0)/2} x`, `ũ = e^{−ρ(t − t0)/2} u`, the terminal weight `S` is unchanged and the
  law `u = −K(t) x` is the same. A test checks it against value iteration.
- **The time origin:** the finite schedule is computed on `0 … T` and read relative to `t0`
  (`K(t − t0)`); the solution's `TimeVaryingFeedback` and `QuadraticField(x̄, S, t)` carry the
  absolute times `t0 + s`. This was an open point; it is the only consistent reading.

### 5.3 The face

- **`z`:** infinite, `[K ; x̄ ; ū]`; finite, `[K(s) ; K∞ ; x̄ ; ū ; t0]` on the relative times
  `s`, `K∞` the stationary gain at the final operating point (one more `care`; the
  `K_after` slot of `TimeVaryingFeedback`).
- **`pv.solve(params)`:** `lqr_matrices` and the Riccati solve at the tree, on the host (SciPy:
  refused under a trace, with the message naming the host loop); `z0` ignored; `status["cost"]
  = (x0 − x̄)ᵀ P (x0 − x̄)` (or `S(t0)`).
- **`pv.initial(params)`:** the solve at the tree (a direct method has no other guess).
- **`pv.command(z, x, t)`:** `ū − K (x − x̄)`, or `ū − K(t − t0)(x − x̄)` read on the schedule,
  `ū − K∞ (x − x̄)` past `tf`.
- **`pv.solution(z, params)`:** the law, the record (`P` or `S(t)`, the poles at the tree) and
  the cost-to-go, `QuadraticField(x̄, P)` or `QuadraticField(x̄, S, t)`. A test checks that a
  long schedule's `S(t)` passes the field's symmetry check.

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
    """J(x), or J(x, t), read on a regular grid: multilinear between nodes."""

    def __init__(self, levels, values, t=None, *, method="linear", outside=None): ...

    def value(self, x, u=None, t=0.0, params=None):
        # Outside the grid: the edge held (x clamped on each axis), or the number `outside`
        # J(x) = Σ_corners J_corner Πₖ (yₖ or 1 − yₖ); a time axis read linearly in t
        ...
```

- **Its place:** `core/fields.py`, beside `QuadraticField`. It needs the levels per axis and the
  table, never the `StateSpaceGrid`; the planner builds it from its grid (`grid.field(J)`).
- **Its arithmetic:** one native multilinear (and nearest) reading of a table on a regular
  grid through `xp`, shared with `LookupPolicy` (the cell search and the corner weights, one
  function; each caller applies its own outside rule). It reads a stack of states `(M, n)`,
  one value per row, as `QuadraticField` does; it traces and takes `params` (`levels`,
  `values`, `t`); its gradient is the exact per-cell multilinear gradient, never a finite
  difference across a cell edge. The spline methods (`"cubic"`, `"quintic"`) stay NumPy-only
  and do not trace in `params`.
- **Time:** a leading axis of `values` on the times `t`, read linearly in `t` and held past its
  ends, the rule of `QuadraticField`; the law holds per stage (§4.4).

### 6.3 Outside the grid, and the infeasible cells

- **The sweeps keep their rule** (arrival cost `0` off the grid, the pair priced by `G`):
  the backup never sees the field's outside rule, so the tables stay byte-identical.
- **A bare `GridField` holds the edge** (`PP-D10`); `outside=` a number on request.
- **The planner's cost-to-go** charges `out_of_bound_cost` outside the grid (`PP-D15`): the
  meaning the sweeps give to leaving it, so `J.as_set(upper=…)` never contains an off-grid
  state and `J.as_terminal_cost()` never pulls a plan out of the box (`grid.field(J)` passes
  it).
- **The infeasible cells:** the cells pinned at the failure cost are a cliff, and linear
  reading ramps across it (the gradient there is about `INF / Δx`). The feasible set is
  `J.as_set(upper=threshold)` with the threshold the planner's `out_of_bound_cost`, named on the
  solution, never a magic `INF − 1`; `as_terminal_cost` on such a field is documented as a
  terminal cost for a sampling or host solver, not a gradient one, across that cliff.

### 6.4 The planners' cost-to-go

- Value iteration: `grid.field(J)`, or `grid.field(J_stack, t=t_i)` on a finite horizon.
- LQR: `QuadraticField(x̄, P)`, or `QuadraticField(x̄, S, t)`; `quadratic_cost_to_go` goes.
- `sol.cost_to_go(x)` keeps working (`Field.__call__`), and the plots read the field.

```python
J = DynamicProgrammingPlanner(problem, ...).solve().cost_to_go
J(x); J(x, t)                     # as today; the finite horizon
J.as_terminal_cost()              # the VI value ends a short nonlinear MPC (a sampling solver)
J.as_set(upper=planner.out_of_bound_cost)   # the states that reach the goal
J.gradient(x)                     # ∂J/∂x, exact inside a cell
```

### 6.5 The `Q` table, later

A table over the grid's states × actions is a table on a joint grid: `Q` is a `GridField` over
the joint levels of `(x, u)`, with `min_over_u()` (the `J` field) and `greedy()` (the
`LookupPolicy`) its two reductions. They land with tabular RL in v0.5; nothing about `Q` is
built here.

## 7. The MPC and the follower read the state

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

## 8. What "equal to `planner.solve()`" means

`pv.solution(pv.solve(p)[0], p, status)` and `planner.solve(params=p)` are compared, per
backend, on: the policy's `params` (value iteration's `u_table`, `levels`, `t`; LQR's `K`,
`x̄`, `ū`, `t`), the record's numbers (`iterations`, `delta`, `converged`, `mode`, the tables or
`P`, `S`, the poles), and the cost-to-go's `params`. A run's history and its timings are
workflow, outside the comparison.

## 9. The rulings of 2026-11-06

- `PP-D1` **The contract** of §3: two axes (open loop or feedback; stationary or
  time-varying); `command(z, x, t)` always receives the state; a time-varying `z` carries `t0`;
  the test is that every planner closes in the dual-rate loop, and in the MPC loop when its
  `z` fits a tick (§1).
- `PP-D2` **Value iteration supports both horizons**, chosen by the problem; the finite
  horizon (formerly S63, v0.5) moves to this work. The three examples of §2 migrate (`PP-D13`).
- `PP-D3` **The policy stays in `z`** (as the command table, §4.5).
- `PP-D4` **`solve_steps(n)` stays,** documented as the receding `n`-step law.
- `PP-D5` **`LookupPolicy` takes a schedule** for the finite horizon (§4.4).
- `PP-D6` **The core math first:** for LQR, `lqr_matrices` at the top of its file (§5.1: the
  Riccati solvers are already pure); the planner and the face both call it. Amended
  2026-11-07: value iteration's backup is written in the body of each engine's step, its
  helper `bellman_backup` removed (§4.1).
- `PP-D7` **The MPC reads the state** (§7); replanning at a slow rate is the dual-rate loop.
- `PP-D8` **Every planner's cost-to-go is a `Field`:** LQR and value iteration in this work, RL
  with the rest of the field object in v0.5.
- `PP-D9` **`GridField(levels, values, t=None)` in `core/fields.py`,** on one native reading of
  a table shared with the lookup law.
- `PP-D10` **Outside the grid, a `GridField` holds the edge,** `outside=` a number on request.
  (The planner's cost-to-go charges the failure cost instead, `PP-D15`.)
- `PP-D11` **A `Q` table is a `GridField` over `(x, u)`;** its reductions land with tabular RL
  in v0.5.
- `PP-D12` **A grid field reads time linearly,** held past its ends; the law holds per stage.

After the reviews (asked one by one, same day):

- `PP-D13` **The problem decides the mode, and the three examples migrate** (the README
  comparison, the showcase, the optimal-control tutorial): value iteration on
  `problem.replace(horizon=None)` where the converged law is meant, the finite law where it is
  shown on purpose; one rule, LQR's.
- `PP-D14` **Past its horizon, a feedback plan applies its stationary law at its final point.**
  Holding the last stage is myopic (a tie at action `0` when `h = 0`; `K(tf) = 0` for LQR with
  `S = 0`), and stage 0 is wrong for a law linearized along a trajectory (the maintainer's
  point: `K(t0)` of a swing-up is designed with the pendulum down). So: LQR, `K∞` at the final
  operating point (`x̄`; `x_d(tf)`, `u_d(tf)` for tracking), one more Riccati solve, carried in
  `z`; value iteration, stage 0, the full-horizon law, its stationary law approximated without a
  second solve (the grid is global, no linearization point); open-loop plans hold their last
  input. A simulation longer than the horizon is the only reader: scoring stops at `tf`, and the
  MPC re-solves every tick.
- `PP-D15` **The planner's cost-to-go charges the failure cost outside the grid;** a bare
  `GridField` keeps the edge held (`PP-D10`).
- `PP-D16` **Discounted LQR solves the undiscounted equivalent on `A − (ρ/2) I`,** with a
  warning that the problem is discounted (unusual for LQR); the known issue closes with it.
- `PP-D17` **The GRO860 notebooks stay as they are.** Their `solve_steps(N)` cells keep the same
  `J_0` tables and plots, and their words stay true of what they plot; the wording (the
  time-varying law is `solve()` on `horizon = N dt`) is revisited when the course is next
  taught, a note in TODO, not a change here.

## 10. Decisions the reviews raised

Ruled on 2026-11-06: `PP-Q1` to `PP-Q4` became `PP-D13` to `PP-D16` (§9); `PP-Q5` became
`PP-D17`. No decision is open.

The agent's to settle, recorded here:

- `PP-Q6` **Size limits:** the finite mode's stacks and a face's `n_z` refused above a limit
  (proposed: 1 GB for the stacks; a face in the MPC loop above 10⁶ numbers), the message giving
  `N`, the nodes and the bytes. Finite value iteration is refused in the MPC loop, accepted in
  the follower.
- `PP-Q7` **The transition-matrix form of `arrival`** (`W` built once): a speed-up measured
  after the change, adopted only if the tables stay byte-identical or the maintainer accepts the
  move.
- `PP-Q8` **The fixed sweep count of a face solve** (infinite mode only): `n_iterations=k` on
  the planner (RULES 2.7's exact count), beside `max_iterations` the cap; refused in the finite
  mode, where `N` is the horizon's.

## 11. What the reviews changed

- §1: the MPC test is a conformance check, not the use of LQR and value iteration.
- §2: corrected: three examples run value iteration on a finite horizon; the off-grid readers;
  LQR's discount and time origin; where LQR's linearization sits.
- §3: `t0` last in `z`; `pv.feedback`; `initial` never a corner; `status["cost"]`; `shift` the
  warm guess, unchanged for a method that ignores `z0`; the shrinking-horizon claim dropped;
  trajectory plus tracking added as a row; commands deterministic; discrete LQR out of scope.
- §4: the backup reads the arrival (zero off the grid), one backup on NumPy and
  JAX (written in each engine's body since 2026-11-07); stage index with a tolerance, `steps_of` for `T / dt`, one symbol each for the stage and
  the sweep count; the discounted meaning of `J(x, t)`; `solve_steps`' record and a test tying it
  to the finite mode; the failure cost over `N`; frozen tables named; the stacks the single
  home; `z` holds the command table, not action ids; the table-rebuild rule covers every leaf
  the tables read; truncated value iteration, not real-time dynamic programming.
- §5: no new Riccati spelling: `lqr_matrices` lowers the problem, the existing pure solvers
  solve it; nominal plant numbers; the discount and the time origin; the host-only face.
- §6: the outside rules kept apart (sweeps, law, field); the infeasible cliff and the
  feasibility threshold; batched reading; the exact multilinear gradient.
- §7: which state the command reads; the follower's measurement input and its wiring.
- §8: the fields "equal to `planner.solve()`" compares.
- §12: the MPC and follower wiring land before the faces; the shared reading's change to the
  law gets its own test; re-sliced on 2026-11-06 into three changes ordered by value.

## 12. Steps, in three changes ordered by value

Local steps; the order of work and each change's "done when" are TODO §1.8's. Re-sliced on
2026-11-06: the work users feel (the readable math, the fields, the finite horizon, the LQR
fixes) comes first; the faces and the MPC protocol, additive and with the least practical
payoff (§1: the two methods do not read the measured state, and their faces run on the host),
come last. Each change merges on its own.

**Change 1: the math and the fields** (no table moves; each field move has its own test):
- `PP-1` The backup (with its arrival reading, NumPy and JAX; in each engine's body since the
  2026-11-07 ruling) and `lqr_matrices` first in its file, the planners calling it: seeded baselines byte-identical.
- `PP-2` The shared grid reading; `GridField` in `core/fields.py`; value iteration's and LQR's
  `cost_to_go` as fields (`PP-D15` outside the grid); `quadratic_cost_to_go` removed. Number
  moves, each tested: the cost-to-go read natively (about `1e-16` inside the grid), the
  cost-to-go outside the grid, the lookup law if the shared reading changes it anywhere.
- `PP-3` Discounted LQR (`PP-D16`): the shifted `A`, the warning, the test against value
  iteration on a discounted double integrator; the known issue closes.
- DESIGN §4 ("Fields") and §6 ("RRT / DP") carry what landed.

**Change 2: the finite horizon** (a behaviour change; the infinite mode byte-identical):
- `PP-4` Value iteration's finite mode: the stacks, `LookupPolicy`'s schedule, `J(x, t)`, the
  JAX scan, the record's `mode`, `solve_steps` documented, the size limit.
- `PP-5` The law past the horizon for both planners (`PP-D14`): `K∞` at the final operating
  point in LQR's finite law (`K_after`), value iteration's stage 0.
- `PP-6` The three examples migrated (`PP-D13`); tests: the finite mode against `solve_steps`,
  LQR and value iteration agreeing on the finite-horizon double integrator, the `h = 0` case
  past `tf`, the stage index on float ticks; every demo calling `solve_steps` run.
- DESIGN §6 carries the two modes.

**Change 3: the faces and the MPC protocol** (additive: no public name breaks, the open-loop
loops byte-identical, so it may land after the `0.4.0` tag without blocking it):
- `PP-7` The MPC and the follower read the state (`pv.feedback`, the `x0` binding, the
  follower's `y`); the dual-rate snippet and its test.
- `PP-8` The two compiled faces: §8's comparison in both modes, NumPy and JAX;
  `test_no_recompile.py` extended; the conformance loops of §1.
- `PP-9` The contract of §3, §7 and §8 into DESIGN §6 (written when it lands, DESIGN holding
  landed contracts only); ROADMAP's ledger rows; this doc deleted.

## 13. What change 1 built (branch `plans-as-policies-1`)

- `bellman_backup(J_next, G, arrival, gamma)` opened `dp.py`, the NumPy table step and the JAX
  step calling it, each with its own `arrival` (SciPy; `map_coordinates` with the in-bounds
  mask); the 2026-11-07 ruling wrote it back into both steps' bodies (§4.1), byte-identical.
  `lqr_matrices` opens `lqr.py`. Seeded baseline byte-identical (NumPy, loop and JAX engines;
  `solve` and `solve_steps`; a discounted, a finite-horizon and a `gamma`-overridden problem;
  the lookup law inside and outside the grid; LQR in both horizons).
- `GridField` in `core/fields.py`, with `grid_value` (the cell reading), `grid_gradient` (the
  exact multilinear gradient) and `StateSpaceGrid.field`. `LookupPolicy` reads its command
  table through `grid_value`: byte-identical inside and outside the grid.
- Value iteration's `cost_to_go` is `grid.field(J)`: bit-exact with the SciPy reading inside
  the grid, the failure cost past it (it read 0). `planner.value_at` reads it. LQR's is
  `QuadraticField` (within `1e-14` of the closure it replaced, removed).
- One API consequence: a field is `V(x, u, t)`, so time goes by keyword, `J(x, t=t)`; the one
  library caller that passed it positionally (the cost-to-go heatmap) and one test follow.
- Discounted LQR on `A_ρ`, with its warning; tested against the discounted Riccati residual and
  against value iteration on a discounted double integrator. The core tutorial
  `08_planners.ipynb` solves LQR on a discounted problem (`discount_rate=0.2`): its LQR numbers
  move to the discounted problem's and the warning shows; the notebook itself is unchanged
  (the maintainer's).
- The JAX compiled path, checked against `dev`: the jitted Bellman step and the compiled sweep
  loop trace to the identical program (about 29 ms per compiled solve either way); the traced
  lookup law computes `1 − y` once per axis instead of per corner (two fewer operations, the
  same values). On `dev` the lookup law cannot be compiled with JAX (its NumPy table is
  indexed by a traced cell), although its docstring says it traces. Ruled 2026-11-06, after a
  per-block backend preference was tried and dropped: the law takes its table into the point's
  array module (one line), so a JAX closed loop runs it and matches NumPy, and
  `backend="auto"` scores it on JAX like every law that traces (one evaluation pays the JAX
  compile, about 0.7 s here; `backend="numpy"` keeps the old timing). `GridField` converts its
  own table the same way. The core planners tutorial gives LQR the undiscounted problem
  (value iteration keeps its `ρ = 0.2`, which its sweeps need to converge: undiscounted they
  stop at the 1000-sweep cap with `ΔJ = 19`).

## 14. What change 2 built (branch `v04-build`)

- `solve()` reads the problem's horizon: `face.n_steps` set, exactly `N` sweeps with every
  stage kept (`record.mode == "finite"`); unset, today's sweeps to `tol` (`"infinite"`,
  byte-identical: the seeded baseline of change 1, its finite case swapped for the same problem
  on no horizon, `cmp` equal on the NumPy, loop and JAX engines). `solve_steps(n)` is the
  receding law (`"receding"`); `fixed_horizon` is gone from the record.
- The stages live on the raw result as `record.tables.schedule = (t, J, pi)`, LQR's
  `record.schedule` shape: `t_0 … t_N`, `J_0 … J_N` (`J_N = h`), `π_0 … π_{N−1}`.
  `record.tables.J` and `pi` stay stage 0, so the plots, `value_at` and `save` read what they
  read. On JAX the `N` sweeps are one `lax.scan` (`compiled_scan`, one compile per `N`);
  verbose or recorded, one jitted sweep at a time as before.
- `LookupPolicy(grid, pi, t=None)`: a stacked `pi` on its stage times, the stage of `t` by
  `stage_of` (traced), stage 0 outside `[t_0, t_N)`. `action(x, params=None, *, t=0.0)`.
- `cost_to_go` is `J(x, t)`, `grid.field(J_stack, t)`.
- The failure cost sums over the sweeps' horizon: `face.failure_cost_over(face.n_steps, γ)`
  (a discounted finite problem's moves from the geometric limit to the `N`-step sum).
- LQR's finite law runs on the problem's clock (`t0 + s`) and applies `K∞` past `tf`
  (`stationary_gain`, `K(tf)` held when the algebraic equation has no stabilizing solution).
- The three examples solve value iteration on `problem.replace(horizon=None)`, each saying why:
  on the README's 4 s swing-up the finite law stays down (`J = 39.3`, the pendulum hanging for
  4 s, cheaper than pumping with no terminal cost), which is right for that problem and not
  what the example shows.
- Measured (pendulum, 101 × 101 nodes, 11 inputs, `dt = 0.05`, best of 3, 300-sweep cap): the
  infinite mode NumPy 2.81 s against 2.70 s on the base (noise, same code path), JAX 260 ms
  against 256 ms; the finite mode (`N = 80`) NumPy 1.30 s, JAX 165 ms.

## 15. Decisions taken during the automatic run (2026-11-07)

Choices the plan left open, each taken as recommended; the maintainer may overturn any.

- **Where the stages live:** `DynamicProgrammingResult.schedule = (t, J, pi)`, mirroring LQR's
  `RiccatiRecord.schedule`, with `J` and `pi` kept as stage 0 (no reader of the tables moves).
- **The frozen tables:** the record's `frozen_at` (the final time) on a finite solve whose
  running-cost and transition tables served every sweep (a precomputed grid, the JAX engine);
  `None` otherwise.
- **The stage tolerance:** `1e-9` of a period, so a tick summed from `dt` reads its own stage.
- **The memory limit:** 1 GiB over `J`, `π` and the law's command table, refused with the
  numbers and the two ways out (a coarser grid, or the converged law).
- **LQR without `K∞`:** a finite horizon needs no stabilizability, so a pair without a
  stabilizing algebraic solution holds `K(tf)` past the horizon instead of failing.
- **The examples:** none shows the finite law on purpose, so all three migrate to the converged
  law; the finite mode is shown by its tests.

**PlanningSolution shape (ruled 2026-11-07).** A planner returns one keyword-only record,
`PlanningSolution(problem, controller, record, trajectory=None, cost_to_go=None,
evaluation=None, params=None)`, under one rule: what is handed out (`controller`) is exactly
what is scored (`evaluation`) and drawn, with no hidden `% dt`. The controller is the block as
it is: a feedback law (value iteration in every mode, LQR, tabular and deep learning) closes as
`controller @ plant`, continuous; an open-loop plan source (trajectory optimization, the
compiled face's `pv.solution`, MPPI, RRT) drives as `controller >> plant`; on a step problem the
law ticks with the plant. `trajectory` has one meaning, the plan an open-loop planner computed,
`None` for a feedback law even with `evaluate=True`; `cost_to_go` is a `Field` or `None` (the
deep RL critic a `CriticField`, its weights its `params`). The policy as a function on the
solution (`solution.policy(x, t)`, `planner.policy(x, t)`, ruled earlier the same day) is
reversed: the function is the compiled face's `pv.command(z, x, t)`. Gone with it: every
planner's `nominal_trajectory`, `holds_policy`, the held rollout and its adapters, RL's
`rollout_law`. A planner's evidence is one `evaluate(problem, controller)` of the controller as
it is (a continuous law on the evaluator's automatic step, which a continuous loop with no `dt`
now runs on; a clocked one on its clock; a step problem's on its face, whose `run` takes a
controller block). So value iteration, tabular and deep learning are scored continuously, no
longer held on their `dt`, and `solve(evaluate=True)`, `evaluate(problem, sol.controller)` and
`compare` agree; a law held on its grid is the other controller `law % dt`. The views
(`plot_trajectory`, `animate`, `plot_cost`) draw the plan, else the controller run once from the
start through the problem's evaluator, nothing stored; the planner mirrors the solution
(`controller`, `cost_to_go`, the views).
